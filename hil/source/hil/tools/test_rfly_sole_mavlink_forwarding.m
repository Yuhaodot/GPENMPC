function report=test_rfly_sole_mavlink_forwarding(outputRoot,source,ports,mode)
% Test localhost forwarding through the MEX.
% Preserve incomplete slices as raw records without reassembly.
if nargin<3,ports=[62381 62382];end
if nargin<4,mode='FULL';end
assert(ismember(string(mode),["FULL","MIXED_BATCH_ONLY","WITNESSED_SUBSTRING_ONLY","FRAGMENT_PARSE_ONLY","MIXED_FRAME_ONLY","PARSE_FAILURE_ONLY","PARSE_BATCH_ONLY"]), ...
    'gpenmpcNative:ForwardingTestMode','Unknown bounded test selection.');
mixedOnly=strcmp(mode,'MIXED_BATCH_ONLY');
witnessOnly=strcmp(mode,'WITNESSED_SUBSTRING_ONLY');
assert(numel(ports)==2&&ports(1)~=ports(2)&&all(ports>=62200&ports<=62399&ports==fix(ports)));
build=fileparts(fileparts(mfilename('fullpath')));addpath(fullfile(build,'host_runtime'));
assert(~isfolder(outputRoot)&&~isfile(outputRoot),'gpenmpcNative:ForwardingTestOutput','Choose an unused output path.');
mkdir(outputRoot);checks=struct('name',{},'pass',{});raw={};transport=[];peer=[];local=[];remote=[];
cleanup=onCleanup(@finish);
cfg=struct('source',source,'scope','HOST_ONLY_LOOPBACK','allow_loopback',true, ...
    'local_host','127.0.0.1','remote_host','127.0.0.1','local_port',ports(1),'remote_port',ports(2), ...
    'local_system',uint8(245),'local_component',uint8(190), ...
    'remote_system',uint8(1),'remote_component',uint8(1),'maximum_poll_datagrams',8, ...
    'copter_serial_forwarding',true);
try
    d=mavlinkdialect(fullfile(build,'m600_coptersim','matlab_validation','+m600check','px4_health_events.xml'),2);
    local=mavlinkio(d,'SystemID',cfg.local_system,'ComponentID',cfg.local_component);
    remote=mavlinkio(d,'SystemID',cfg.remote_system,'ComponentID',cfg.remote_component);
    peer=udpport('datagram','IPV4','LocalHost','127.0.0.1','LocalPort',ports(2));
    m=createmsg(d,'PING');m.Payload.time_usec=uint64(2^53)+uint64(719);m.Payload.seq=uint32(89);
    complete=uint8(serializemsg(remote,m));complete=complete(:);
    actualSlices={};actualComplete=[];mixedRecords=struct([]);mixedFragments=struct([]);
    if strcmp(mode,'PARSE_BATCH_ONLY')
        retainedPath=fullfile(gpenmpc_external_path('parse_batch'),'RETAINED_PARSE_BATCH.mat');
        retained=load(retainedPath,'b');batch=retained.b;
        batch=batch([batch.received_bytes]>0);
        assert(isequal(double([batch.received_bytes]),[40 44 96 18 96]));
        transport=gpenmpcNative.RflySoleMavlinkTransport(cfg,d,local);
        for k=1:numel(batch),sendPeer(batch(k).bytes(:));end;pause(.01);
        started=tic;
        while numel(mixedRecords)+numel(mixedFragments)<5&&toc(started)<1
            [r,f]=transport.poll();mixedRecords=[mixedRecords;r];mixedFragments=[mixedFragments;f]; %#ok<AGROW>
        end
        state=transport.status();raw{end+1}=state;
        decoded=[mixedRecords.decoded_message];
        check('parse_batch_only_complete_position_and_altitude_admitted', ...
            numel(mixedRecords)==2&&isequal([decoded.MsgID],[32 141])&&all([mixedRecords.validated]));
        check('parse_batch_three_original_slices_retained_no_control_authority', ...
            numel(mixedFragments)==3&&state.forwarding_fragment_bytes==210 ...
            &&~state.failed&&state.accepted_frames==2 ...
            &&all(~[mixedFragments.message_generated])&&all(~[mixedFragments.control_authority]) ...
            &&isequal(mixedFragments(3).raw_fragment,batch(5).bytes(:)));
        sendPeer(complete);[next,~]=pollOne();
        check('parse_batch_next_complete_crc_frame_still_delivered',numel(next)==1&&next.validated&&next.decoded_message.MsgID==4);
        dropTransport();
        rejectsBatch({batch(5).bytes(:)},'parse_batch_bad_flags_without_prefix_rejected');
        wrongPrefix=batch(3).bytes(:);wrongPrefix(6)=uint8(2);
        rejectsBatch({wrongPrefix,batch(4).bytes(:),batch(5).bytes(:)},'parse_batch_wrong_source_prefix_rejected');
        tooLong=[batch(5).bytes(:);zeros(35,1,'uint8')];
        rejectsBatch({batch(3).bytes(:),batch(4).bytes(:),tooLong},'parse_batch_continuation_exceeds_declared_frame_rejected');
        bad=complete;bad(3)=bitor(bad(3),uint8(2));
        rejectsBatch({batch(3).bytes(:),bad},'parse_batch_complete_expected_source_bad_flags_not_hidden');
        bad=complete;bad(end)=bitxor(bad(end),uint8(1));
        rejectsBatch({batch(3).bytes(:),bad},'parse_batch_complete_bad_crc_not_hidden');
    elseif strcmp(mode,'MIXED_FRAME_ONLY')
        retainedPath=fullfile(gpenmpc_external_path('mixed_frame_parse'),'SHORT_HIL','RAW_BOARD_LOCAL_SHORT_HIL.mat');
        retained=load(retainedPath,'rawIo');batch=retained.rawIo.raw_transport.last_receive.records;
        batch=batch([batch.received_bytes]>0);
        assert(isequal(double([batch.received_bytes]),[96 96 244 30]));
        transport=gpenmpcNative.RflySoleMavlinkTransport(cfg,d,local);
        for k=1:numel(batch),sendPeer(batch(k).bytes(:));end;pause(.01);
        [mixedRecords,mixedFragments]=transport.poll();state=transport.status();raw{end+1}=state;
        decoded=[mixedRecords.decoded_message];
        check('mixed_frame_complete_crc_frames_only',numel(mixedRecords)==2 ...
            &&isequal([decoded.MsgID],[331 74])&&all([mixedRecords.validated]));
        check('mixed_frame_exact_payload_slices_not_messages',numel(mixedFragments)==2 ...
            &&state.accepted_frames==2&&~state.failed&&state.forwarding_fragment_bytes==192 ...
            &&all(~[mixedFragments.message_generated])&&all(~[mixedFragments.control_authority]));
        dropTransport();
        altered=batch(1).bytes(:);altered(20)=bitxor(altered(20),uint8(1));
        rejectsBatch({altered,batch(3).bytes(:)},'mixed_frame_changed_slice_rejected');
        rejectsBatch({batch(1).bytes(:)},'mixed_frame_without_complete_witness_rejected');
        bad=batch(3).bytes(:);bad(end)=bitxor(bad(end),uint8(1));
        rejectsBatch({batch(1).bytes(:),bad},'mixed_frame_bad_witness_crc_rejected');
        bad=complete;bad(3)=bitor(bad(3),uint8(2));
        rejectsBatch({bad},'mixed_frame_complete_bad_flags_rejected');
    elseif ismember(string(mode),["FRAGMENT_PARSE_ONLY","PARSE_FAILURE_ONLY"])
        if strcmp(mode,'PARSE_FAILURE_ONLY')
            retainedPath=fullfile(gpenmpc_external_path('parse_failure'),'EXISTING_PARSE_FAILURE.mat');
            retained=load(retainedPath,'s');batch=retained.s.native.last_receive.records;
        else
            retainedPath=fullfile(gpenmpc_external_path('fragment_parse'),'SHORT_HIL','RAW_BOARD_LOCAL_SHORT_HIL.mat');
            retained=load(retainedPath,'rawIo');batch=retained.rawIo.raw_transport.last_receive.records;
        end
        batch=batch([batch.received_bytes]>0);
        assert(isequal(double([batch.received_bytes]),[40 96 96]));
        transport=gpenmpcNative.RflySoleMavlinkTransport(cfg,d,local);
        for k=1:numel(batch),sendPeer(batch(k).bytes(:));end;pause(.01);
        [mixedRecords,mixedFragments]=transport.poll();state=transport.status();raw{end+1}=state;
        if strcmp(mode,'PARSE_FAILURE_ONLY')
            % Production poll yields at its existing elapsed-work boundary;
            % one retained original batch can span multiple bounded calls.
            pendingTime=tic;
            while numel(mixedFragments)<2&&toc(pendingTime)<.5
                [moreRecords,moreFragments]=transport.poll();
                mixedRecords=[mixedRecords;moreRecords];mixedFragments=[mixedFragments;moreFragments]; %#ok<AGROW>
                state=transport.status();raw{end+1}=state;
            end
        end
        check('fragment_parse_only_complete_position_frame_admitted',numel(mixedRecords)==1 ...
            &&mixedRecords.validated&&mixedRecords.decoded_message.MsgID==32&&state.accepted_frames==1);
        check('fragment_parse_two_slices_retained_without_state_or_crc_claim',numel(mixedFragments)==2 ...
            &&state.forwarding_fragment_bytes==192&&~state.failed ...
            &&all(~[mixedFragments.message_generated])&&all(~[mixedFragments.control_authority]));
        if strcmp(mode,'PARSE_FAILURE_ONLY')
            sendPeer(complete);[nextRecords,~]=pollOne();
            check('next_complete_frame_valid_after_unknown_payload_slice',numel(nextRecords)==1 ...
                &&nextRecords.validated&&nextRecords.decoded_message.MsgID==4 ...
                &&isequal(nextRecords.raw_frame,complete));
        end
        dropTransport();
        rejectsBatch({batch(3).bytes(:)},'unrelated_bad_crc_without_prefix_still_rejected');
        bad=complete;bad(end)=bitxor(bad(end),uint8(1));
        rejectsBatch({bad},'complete_expected_source_bad_crc_still_rejected');
        rejectsBatch({batch(2).bytes(:),bad},'complete_bad_crc_not_hidden_by_incomplete_prefix');
    elseif witnessOnly
        retainedPath=fullfile(gpenmpc_external_path('native_safety_failure'),'RETAINED_NATIVE_SAFETY_FAILURE.mat');
        retained=load(retainedPath,'retainedFailure');
        batch=retained.retainedFailure.rawIo.raw_transport.native.last_receive.records;
        actualSlices={batch(3).bytes(:),batch(4).bytes(:)};actualComplete=batch(5).bytes(:);
        assert(isequal([actualSlices{:}],reshape(actualComplete(1:192),96,2))&&numel(actualComplete)==244);
        transport=gpenmpcNative.RflySoleMavlinkTransport(cfg,d,local);
        sendPeer(actualSlices{1});sendPeer(actualSlices{2});sendPeer(actualComplete);pause(.01);
        [mixedRecords,mixedFragments]=transport.poll();state=transport.status();raw{end+1}=state;
        check('recorded_prefix_pseudoheader_full_same_batch',numel(mixedRecords)==1 ...
            &&mixedRecords.validated&&mixedRecords.decoded_message.MsgID==331 ...
            &&isequal(mixedRecords.raw_frame,actualComplete)&&numel(mixedFragments)==2);
        proof=mixedFragments(2).duplicate_witness;
        check('whole96_not_only_pseudo68_exact_crc0_witness', ...
            isequal(mixedFragments(2).raw_fragment,actualSlices{2}) ...
            &&proof.fragment_start_1based==97&&proof.fragment_end_1based==192 ...
            &&proof.witness_datagram_index==3&&proof.official_crc_status==0 ...
            &&proof.system_id==1&&proof.component_id==1 ...
            &&isequal(proof.witness_raw_frame,actualComplete));
        check('preceding96_prefix_and_actual_qpc_retained', ...
            proof.preceding_prefix.datagram_index==1 ...
            &&isequal(proof.preceding_prefix.raw_datagram.bytes,actualSlices{1}) ...
            &&proof.witness_raw_datagram.dequeue_ns==mixedRecords.original_host_receive_ns ...
            &&mixedFragments(2).raw_datagram.dequeue_ns==mixedFragments(2).original_host_receive_ns);
        check('witness_does_not_admit_message_or_update_source',state.accepted_frames==1 ...
            &&state.forwarding_fragment_count==2&&state.forwarding_fragment_bytes==192 ...
            &&~state.failed&&~proof.witness_message_admitted_here&&~proof.source_or_age_updated ...
            &&~mixedFragments(2).message_generated&&~mixedFragments(2).control_authority);
        dropTransport();
        altered=actualSlices{2};altered(20)=bitxor(altered(20),uint8(1));
        rejectsBatch({actualSlices{1},altered,actualComplete},'changed_fragment_byte_rejected');
        rejectsBatch(actualSlices,'no_complete_witness_rejected');
        badWitness=actualComplete;badWitness(end)=bitxor(badWitness(end),uint8(1));
        rejectsBatch({actualSlices{1},actualSlices{2},badWitness},'bad_crc_witness_rejected');
        fresh=copy(d);originalMessage=deserializemsg(fresh,actualComplete.');delete(fresh);
        wrong=mavlinkio(d,'SystemID',uint8(2),'ComponentID',uint8(1));
        wrongWire=uint8(serializemsg(wrong,originalMessage));delete(wrong);
        rejectsBatch({actualSlices{1},actualSlices{2},wrongWire(:)},'wrong_source_witness_rejected');
        badComplete=complete;badComplete(end)=bitxor(badComplete(end),uint8(1));
        carrier=createmsg(d,'TUNNEL');carrier.Payload.payload_length=uint8(numel(badComplete));
        carrier.Payload.payload(1:numel(badComplete))=badComplete;
        carrierWire=uint8(serializemsg(remote,carrier));
        assert(~isempty(strfind(char(carrierWire(:).'),char(badComplete.'))));
        rejectsBatch({badComplete,carrierWire(:)},'whole_bad_crc_frame_never_substring_exempt');
    else
    % Replay recorded serial-forwarding packet slices.
    retainedPath=fullfile(gpenmpc_external_path('startup_observation'), ...
        'POST_FAILURE_UDP_OBSERVATION_OFFICIAL_STREAM_PARSER.mat');
    assert(strcmp(sha(retainedPath),'F1510947AF614FE840471C50BA027389390C38FC0CFD7D3A6713D9E9931A469F'));
    retained=load(retainedPath,'raw');
    actualSlices={uint8(retained.raw{12}.bytes(:)),uint8(retained.raw{13}.bytes(:))};
    actualComplete=uint8(retained.raw{14}.bytes(:));
    assert(isequal([actualSlices{:}],reshape(actualComplete(1:192),96,2)) ...
        &&numel(actualComplete)==244);
    transport=gpenmpcNative.RflySoleMavlinkTransport(cfg,d,local);
    sendPeer(actualSlices{1});sendPeer(actualSlices{2});sendPeer(actualComplete);
    pause(.01); % Queue all three actual datagrams before the one bounded poll.
    [mixedRecords,mixedFragments]=transport.poll();mixedState=transport.status();raw{end+1}=mixedState;
    nativeBatch=mixedState.last_receive.records;
    nativeBatch=nativeBatch([nativeBatch.received_bytes]>0);
    check('actual_three_datagrams_dequeued_in_one_poll',numel(nativeBatch)==3 ...
        &&isequal(nativeBatch(1).bytes,actualSlices{1}) ...
        &&isequal(nativeBatch(2).bytes,actualSlices{2}) ...
        &&isequal(nativeBatch(3).bytes,actualComplete));
    check('fragment_then_complete_no_blank_struct_record',numel(mixedRecords)==1 ...
        &&isstruct(mixedRecords.decoded_message)&&mixedRecords.validated ...
        &&mixedRecords.decoded_message.MsgID==331 ...
        &&isequal(mixedRecords.raw_frame,actualComplete)&&mixedState.accepted_frames==1);
    check('both_original_slices_retained_without_reassembly',numel(mixedFragments)==2 ...
        &&isequal(mixedFragments(1).raw_fragment,actualSlices{1}) ...
        &&isequal(mixedFragments(2).raw_fragment,actualSlices{2}) ...
        &&mixedState.forwarding_fragment_count==2&&mixedState.forwarding_fragment_bytes==192 ...
        &&~mixedState.forwarding_reassembly_performed&&~mixedState.failed);
    check('mixed_batch_preserves_each_actual_dequeue_observation', ...
        mixedRecords.original_host_receive_ns==nativeBatch(3).dequeue_ns ...
        &&mixedFragments(1).original_host_receive_ns==nativeBatch(1).dequeue_ns ...
        &&mixedFragments(2).original_host_receive_ns==nativeBatch(2).dequeue_ns);
    dropTransport();
    end
    if ~mixedOnly&&~witnessOnly&&~ismember(string(mode),["MIXED_FRAME_ONLY","PARSE_FAILURE_ONLY","PARSE_BATCH_ONLY"])
    transport=gpenmpcNative.RflySoleMavlinkTransport(cfg,d,local);
    prefix=complete(1:end-1);sendPeer(prefix);[r,f]=pollOne();s=transport.status();raw{end+1}=s;
    check('incomplete_frame_not_a_message',isempty(r)&&numel(f)==1&&s.accepted_frames==0 ...
        &&~s.failed&&s.forwarding_fragment_count==1&&isequal(f(1).raw_fragment,prefix));
    check('fragment_full_original_bytes_and_uint64_time', ...
        isequal(f(1).raw_datagram.bytes,prefix)&&isa(f(1).original_host_receive_ns,'uint64') ...
        &&f(1).original_host_receive_ns==f(1).raw_datagram.dequeue_ns ...
        &&isequal(f(1).dequeueinfo.dequeue_qpc,f(1).raw_datagram.dequeue_qpc) ...
        &&~f(1).message_generated&&~f(1).mavlink_source_verified&&~f(1).control_authority ...
        &&strcmp(f(1).classification,'INCOMPLETE_FORWARDING_FRAGMENT_NOT_A_MESSAGE'));
    check('status_retains_same_fragment',isequaln(s.last_forwarding_fragments,f) ...
        &&s.forwarding_fragment_bytes==uint64(numel(prefix))&&isempty(s.last_frames));
    sendPeer(complete);[r,f]=pollOne();s=transport.status();raw{end+1}=s;
    check('later_complete_frame_accepted_once',numel(r)==1&&isempty(f)&&s.accepted_frames==1 ...
        &&isequal(r.raw_frame,complete)&&r.decoded_message.Payload.time_usec==m.Payload.time_usec ...
        &&s.forwarding_fragment_count==1&&~s.forwarding_reassembly_performed);
    [r,f]=transport.poll();check('empty_poll_does_not_replay',isempty(r)&&isempty(f)&&transport.status().accepted_frames==1);
    dropTransport();

    transport=gpenmpcNative.RflySoleMavlinkTransport(cfg,d,local);
    % Reject each incomplete slice even when their concatenation would form a valid message.
    cut=7;left=complete(1:cut);right=complete(cut+1:end);
    assert(right(1)~=253&&right(1)~=254);
    sendPeer(left);[r,f]=pollOne();check('short_header_not_accepted',isempty(r)&&numel(f)==1);
    sendPeer(right);[r,f]=pollOne();s=transport.status();raw{end+1}=s;
    check('missing_full_frame_never_reassembled',isempty(r)&&numel(f)==1 ...
        &&s.accepted_frames==0&&s.forwarding_fragment_count==2&&~s.failed ...
        &&isequal(f.raw_fragment,right));
    dropTransport();

    transport=gpenmpcNative.RflySoleMavlinkTransport(cfg,d,local);
    sendPeer([complete;left]);[r,f]=pollOne();s=transport.status();raw{end+1}=s;
    check('valid_prefix_plus_fragment_kept_separately',numel(r)==1&&numel(f)==1 ...
        &&isequal(r.raw_frame,complete)&&isequal(f.raw_fragment,left) ...
        &&r.original_host_receive_ns==f.original_host_receive_ns ...
        &&f.dequeueinfo.frame_offset==numel(complete)+1 ...
        &&isequal(f.raw_datagram.bytes,[complete;left]));
    dropTransport();

    % Replay a batch containing a 31-byte prefix, a 96-byte middle slice and
    % a complete 244-byte Msg331. The middle slice resembles a MAVLink header;
    % classify it as forwarding evidence only when prefix, CRC and source match.
    fragmentFixturePath=fullfile(gpenmpc_external_path('fragment_continuity_fixture'),'SHORT_HIL', ...
        'RAW_BOARD_LOCAL_SHORT_HIL.mat');
    fragmentFixture=load(fragmentFixturePath,'rawIo');
    fragmentBatch=fragmentFixture.rawIo.raw_transport.native.last_receive.records;
    fragmentPrefix=fragmentBatch(7).bytes(:);fragmentMiddle=fragmentBatch(8).bytes(:);
    fragmentComplete=fragmentBatch(9).bytes(:);
    assert(numel(fragmentPrefix)==31&&numel(fragmentMiddle)==96&&numel(fragmentComplete)==244 ...
        &&isequal(fragmentPrefix,fragmentComplete(1:31)) ...
        &&isequal(fragmentMiddle,fragmentComplete(32:127)) ...
        &&fragmentMiddle(1)==253&&bitand(fragmentMiddle(3),uint8(254))~=0);
    transport=gpenmpcNative.RflySoleMavlinkTransport(cfg,d,local);
    sendPeer(fragmentPrefix);sendPeer(fragmentMiddle);sendPeer(fragmentComplete);pause(.01);
    [r,f]=transport.poll();s=transport.status();raw{end+1}=s;
    check('fragment_continuity_exact_batch_accepts_only_complete_msg331',numel(r)==1&&r.validated ...
        &&r.decoded_message.MsgID==331&&isequal(r.raw_frame,fragmentComplete) ...
        &&s.accepted_frames==1&&numel(f)==2&&~s.failed);
    middleProof=f(2).duplicate_witness;
    check('fragment_continuity_middle_slice_requires_contiguous_prefix_crc0_source_witness', ...
        isequal(f(1).raw_fragment,fragmentPrefix)&&isequal(f(2).raw_fragment,fragmentMiddle) ...
        &&strcmp(f(2).reason, ...
            'EXACT_CONTIGUOUS_MIDDLE_SLICE_OF_SAME_BATCH_CRC0_SOURCE_MATCHED_FRAME') ...
        &&middleProof.preceding_prefix.datagram_index==1 ...
        &&middleProof.fragment_start_1based==32&&middleProof.fragment_end_1based==127 ...
        &&middleProof.official_crc_status==0&&middleProof.system_id==1 ...
        &&middleProof.component_id==1&&~f(2).message_generated&&~f(2).control_authority);
    dropTransport();
    rejectsBatch({fragmentMiddle,fragmentComplete},'fragment_continuity_missing_contiguous_prefix_rejected');
    changedMiddle=fragmentMiddle;changedMiddle(20)=bitxor(changedMiddle(20),uint8(1));
    rejectsBatch({fragmentPrefix,changedMiddle,fragmentComplete},'fragment_continuity_changed_middle_rejected');
    badFragmentComplete=fragmentComplete;badFragmentComplete(end)=bitxor(badFragmentComplete(end),uint8(1));
    rejectsBatch({fragmentPrefix,fragmentMiddle,badFragmentComplete},'fragment_continuity_bad_crc_witness_rejected');
    freshFragmentDecoder=copy(d);originalFragment=deserializemsg(freshFragmentDecoder,fragmentComplete.', ...
        'OutputAllMessages',true);delete(freshFragmentDecoder);
    wrongSourceDecoder=mavlinkio(d,'SystemID',uint8(2),'ComponentID',uint8(1));
    wrongSourceWire=uint8(serializemsg(wrongSourceDecoder,originalFragment));delete(wrongSourceDecoder);
    rejectsBatch({fragmentPrefix,fragmentMiddle,wrongSourceWire(:)}, ...
        'fragment_continuity_wrong_source_witness_rejected');

    bad=complete;bad(end)=bitxor(bad(end),uint8(1));rejects(cfg,bad,'bad_crc_sticky');
    bad=complete;bad(3)=bitor(bad(3),uint8(2));rejects(cfg,bad,'complete_bad_flags_sticky');
    strict=rmfield(cfg,'copter_serial_forwarding');rejects(strict,prefix,'default_truncated_rejected');
    rejects(strict,right,'default_nonmagic_rejected');
    badCfg=cfg;badCfg.copter_serial_forwarding=1;failed=false;
    try,t=gpenmpcNative.RflySoleMavlinkTransport(badCfg,d,local);t.close();delete(t);catch,failed=true;end
    check('forwarding_flag_must_be_logical',failed);
    end
    report=struct('pass',all([checks.pass]),'passed',sum([checks.pass]),'total',numel(checks), ...
        'checks',checks,'scope','HOST_ONLY_LOOPBACK','ports',ports,'source',source, ...
        'selection',char(mode),'retained_fragment_source',retainedPath, ...
        'COM_calls',0,'NoUI_launches',0,'model_runs',0,'control_messages',0, ...
        'fragment_reassembly',false,'class_sha256',sha(fullfile(build,'host_runtime','+gpenmpcNative','RflySoleMavlinkTransport.m')));
    save(fullfile(outputRoot,'RAW.mat'),'checks','raw','cfg','complete','report', ...
        'actualSlices','actualComplete','mixedRecords','mixedFragments');
    f=fopen(fullfile(outputRoot,'RESULT.json'),'w');assert(f>=0);g=onCleanup(@()fclose(f));
    fwrite(f,jsonencode(report,PrettyPrint=true),'char');clear g
    clear cleanup
catch ex
    failure=struct('identifier',ex.identifier,'message',ex.message,'stack',ex.stack);
    if ~isempty(transport),try,raw{end+1}=transport.status();catch,end,end
    save(fullfile(outputRoot,'FAILURE_RAW.mat'),'checks','raw','cfg','failure');
    clear cleanup
    rethrow(ex)
end
    function check(name,value)
        checks(end+1)=struct('name',name,'pass',logical(value));
        assert(value,'gpenmpcNative:ForwardingTest','%s',name);
    end
    function sendPeer(bytes)
        write(peer,uint8(bytes(:).'),'uint8','127.0.0.1',ports(1));
    end
    function [r,f]=pollOne()
        r=struct([]);f=struct([]);started=tic;
        while isempty(r)&&isempty(f)&&toc(started)<2
            [r,f]=transport.poll();if isempty(r)&&isempty(f),pause(.002);end
        end
        assert(~isempty(r)||~isempty(f),'gpenmpcNative:ForwardingTestReceive','No localhost test datagram received.');
    end
    function rejects(configuration,bytes,name)
        transport=gpenmpcNative.RflySoleMavlinkTransport(configuration,d,local);
        sendPeer(bytes);failed=false;
        try,pollOne();catch,failed=true;end
        s=transport.status();raw{end+1}=s;
        check(name,failed&&s.failed&&s.accepted_frames==0);
        nativeRecords=s.last_receive.records;nonempty=arrayfun(@(x)~isempty(x.bytes),nativeRecords);
        nativeRecords=nativeRecords(nonempty);
        check([name '_original_bytes'],~isempty(nativeRecords)&&isequal(nativeRecords(1).bytes,bytes));
        first=s.first_error;original=s.last_receive;sendPeer(complete);blocked=false;
        try,transport.poll();catch,blocked=true;end
        after=transport.status();
        check([name '_later_full_cannot_recover'],blocked&&after.accepted_frames==0 ...
            &&isequaln(first,after.first_error)&&isequaln(original,after.last_receive));
        dropTransport();
    end
    function rejectsBatch(packets,name)
        transport=gpenmpcNative.RflySoleMavlinkTransport(cfg,d,local);
        for bi=1:numel(packets),sendPeer(packets{bi});end;pause(.01);
        failed=false;failureReport='';
        % Production poll may yield after a datagram's bounded work slice.
        % Exercise the retained tail instead of mistaking that yield for
        % acceptance of a later deliberately-invalid datagram.
        started=tic;
        while ~failed&&toc(started)<1
            try,transport.poll();catch ex,failed=true;failureReport=getReport(ex,'extended','hyperlinks','off');end
        end
        s=transport.status();raw{end+1}=struct('state',s,'failure_report',failureReport);
        nr=s.last_receive.records;nr=nr([nr.received_bytes]>0);
        exact=numel(nr)==numel(packets);
        if exact,for bi=1:numel(packets),exact=exact&&isequal(nr(bi).bytes,packets{bi}(:));end,end
        check(name,failed&&s.failed&&s.accepted_frames==0&&exact);
        before=s.last_receive;blocked=false;
        try,transport.poll();catch,blocked=true;end
        after=transport.status();
        check([name '_sticky_original_batch'],blocked&&isequaln(before,after.last_receive) ...
            &&isequaln(s.first_error,after.first_error));
        dropTransport();
    end
    function dropTransport()
        if ~isempty(transport),transport.close();delete(transport);transport=[];end
    end
    function finish()
        if ~isempty(transport),try,transport.close();catch,end;try,delete(transport);catch,end,end
        objects={peer,local,remote};
        for k=1:numel(objects),if ~isempty(objects{k}),try,delete(objects{k});catch,end,end,end
    end
end
function h=sha(path)
f=fopen(path,'rb');assert(f>=0);g=onCleanup(@()fclose(f)); %#ok<NASGU>
m=java.security.MessageDigest.getInstance('SHA-256');
while ~feof(f),b=fread(f,1048576,'*uint8');m.update(typecast(b,'int8'));end
h=upper(reshape(dec2hex(typecast(m.digest(),'uint8'),2).',1,[]));
end
