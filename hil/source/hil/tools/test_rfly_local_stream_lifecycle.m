function report=test_rfly_local_stream_lifecycle(outputRoot)
% Test stream lifecycle through function-handle IO with retained C bodies.
arguments,outputRoot (1,1) string,end
build=string(fileparts(fileparts(mfilename('fullpath'))));oldPath=path;
guard=onCleanup(@()path(oldPath)); %#ok<NASGU>
addpath(fullfile(build,'host_runtime'),'-begin');
assert(~isfolder(outputRoot));mkdir(outputRoot);
d=mavlinkdialect(fullfile(build,'m600_coptersim','matlab_validation','+m600check','px4_health_events.xml'),2);
base=fullfile(build,'rfly_vendor_integration','full_inner_abi');
wire.snapshot=readFirst(fullfile(base,'snapshot_wire_fixture','RLS1_SNAPSHOTS.bin'),382);
pair=readFirst(fullfile(base,'snapshot_wire_fixture','RGP1_RGR1_PAIRS.bin'),596);
wire.gp_request=pair(1:310);
wire.committed_state=readFirst(fullfile(gpenmpc_external_path('stream_lifecycle_exchange'),'COMMITTED_RLC2.bin'),1494);
snapshot=gpenmpcNative.RflyLocalSnapshotDecoder(wire.snapshot,uint64(1));
gp=gpenmpcNative.RflyLocalGpCodec.decodeRequest(wire.gp_request);
committed=gpenmpcNative.RflyLocalCommittedDecoder(wire.committed_state,uint64(1));
confirmed=gpenmpcNative.rflyOriginalHostMonotonicNs()-uint64(1000000);
task=repmat('A',1,64);session=repmat('B',1,64);configuration=hex(gp.configuration_sha256);
registered=struct('local_full_inner',true,'identity',snapshot.identity,'leg_index',uint8(1), ...
    'task_sha256',task,'execution_session_sha256',session,'configuration_sha256',configuration, ...
    'reference_asset_sha256',hex(committed.reference_asset_sha256));
i=snapshot.identity;
echo=struct('uid',i.uid,'process_session_generation',i.boot_generation,'system',i.system, ...
    'component',i.component,'link_lifecycle_generation',uint64(7),'configuration_payload_sha256',configuration);
parsed=struct('leg',uint8(1),'task_sha',task,'state',3,'start_requests',0,'stop_requests',0,'session_fault',0);
association=struct('local_full_inner',true,'registration_result','Registered', ...
    'echo_confirmation_result','Confirmed','execution_session_sha256',session, ...
    'original_host_receive_ns',confirmed,'echo',echo,'confirm_receipt',struct('parsed_fields',parsed), ...
    'fixture_scope','EXPLICIT_MOCK_REGISTRATION');
expected=struct('local_full_inner',true,'uid',i.uid,'session_generation',i.boot_generation, ...
    'source_system',i.system,'source_component',i.component,'target_system',uint8(255),'target_component',uint8(190), ...
    'execution_session_sha256',session,'configuration_sha256',configuration,'task_sha256',task, ...
    'leg_index',uint8(1),'confirmed_host_rx_ns',confirmed,'link_lifecycle_generation',uint64(7));
checks=struct('name',{},'pass',{});
try
f=mockFixture(expected,association,d,wire);owner=gpenmpcNative.RflyLocalStreamLifecycle(f.io,registered,association);
fs=f.state();check('constructor_no_send_or_stream_claim',fs.send_calls==0&&~owner.StreamObserved&&owner.EnableAttempts==0);
request=owner.enable();fs=f.state();check('one_exact_existing_stream_enable',fs.send_calls==1 ...
    &&strcmp(request.transport_receipt.encoding.original_command,'mavlink stream -d /dev/ttyACM0 -s GPENMPC_LOCAL_WIRE -r -1') ...
    &&owner.EnableSendReturned&&~owner.StreamObserved&&~request.board_acknowledged);
for channel=["snapshot","gp_request","committed_state"]
    item=f.item(char(channel),false);observed=owner.observe(item);
    check(['actual_body_and_mock_same_io_' char(channel)],observed.original_stream_traffic_observed ...
        &&~observed.enable_command_acknowledged&&~observed.caused_by_enable_proven ...
        &&~observed.release_proven&&~observed.freshness_renewed ...
        &&isequaln(owner.LastObservation,item));
end
st=owner.status();check('all_three_original_channels_observed_not_ack',isequal(st.observation_counts,uint64([1 1 1])) ...
    &&st.original_stream_traffic_observed&&~st.enable_command_acknowledged&&~st.release_proven);
off=owner.disable();st=owner.status();check('exact_existing_disable_no_release_claim',owner.DisableSendReturned ...
    &&strcmp(off.transport_receipt.encoding.original_command,'mavlink stream -d /dev/ttyACM0 -s GPENMPC_LOCAL_WIRE -r 0') ...
    &&~st.stream_disabled_proven&&~st.release_proven);
fs=f.state();calls=fs.send_calls;owner.disable();fs=f.state();check('successful_disable_repeat_read_only',fs.send_calls==calls&&owner.DisableAttempts==1);
reject('observation_after_disable_rejected',@()owner.observe(item),'gpenmpcNative:LocalStreamObservationState');

f=mockFixture(expected,association,d,wire);owner=gpenmpcNative.RflyLocalStreamLifecycle(f.io,registered,association);owner.enable();
reject('repeated_enable_not_resent',@()owner.enable(),'gpenmpcNative:LocalStreamEnableOnce');
fs=f.state();check('one_enable_attempt_retained',owner.EnableAttempts==1&&fs.send_calls==1);
owner.disable();check('cleanup_after_local_failure_available',owner.DisableAttempts==1&&owner.DisableSendReturned&&owner.Failed);

f=mockFixture(expected,association,d,wire);f.set('bound',false);
reject('unbound_rejected_without_constructor_send',@()gpenmpcNative.RflyLocalStreamLifecycle(f.io,registered,association),'gpenmpcNative:LocalStreamUnbound');
fs=f.state();check('unbound_zero_sends',fs.send_calls==0);
f=mockFixture(expected,association,d,wire);bad=expected;bad.execution_session_sha256=repmat('D',1,64);f.set('expected',bad);
reject('mismatched_session_rejected',@()gpenmpcNative.RflyLocalStreamLifecycle(f.io,registered,association),'gpenmpcNative:LocalStreamSession');
f=mockFixture(expected,association,d,wire);bad=association;bad.fixture_scope='NOT_THE_ACTUAL_IO_ASSOCIATION';
reject('copied_other_association_rejected',@()gpenmpcNative.RflyLocalStreamLifecycle(f.io,registered,bad),'gpenmpcNative:LocalStreamAssociation');

f=mockFixture(expected,association,d,wire);owner=gpenmpcNative.RflyLocalStreamLifecycle(f.io,registered,association);
f.set('fail_send',true);reject('send_failure_retains_attempt',@()owner.enable(),'gpenmpcFixture:StreamSendFailure');
check('partial_send_not_returned_success',owner.EnableAttempts==1&&~owner.EnableSendReturned ...
    &&owner.EnableReceipt.api_invoked&&~owner.EnableReceipt.api_returned ...
    &&owner.EnableReceipt.current_transport_receipt_observed ...
    &&owner.EnableReceipt.transport_receipt.messages_attempted==1 ...
    &&owner.EnableReceipt.transport_receipt.messages_send_returned==0 ...
    &&owner.EnableReceipt.transport_receipt.partial);
f.set('fail_send',false);fs=f.state();beforePolls=fs.poll_calls;owner.disable();fs=f.state();
check('cleanup_after_transport_failure_does_not_poll_failed_ingress',owner.DisableSendReturned ...
    &&fs.poll_calls==beforePolls&&owner.Failed&&owner.DisableAttempts==1);

f=mockFixture(expected,association,d,wire);owner=gpenmpcNative.RflyLocalStreamLifecycle(f.io,registered,association);
f.set('fail_before_receipt',true);reject('send_failure_before_io_receipt_not_fabricated',@()owner.enable(),'gpenmpcFixture:StreamSendFailure');
check('missing_send_receipt_remains_unknown',owner.EnableAttempts==1&&~owner.EnableSendReturned ...
    &&isempty(owner.EnableReceipt.transport_receipt)&&~owner.EnableReceipt.current_transport_receipt_observed);

f=mockFixture(expected,association,d,wire);owner=gpenmpcNative.RflyLocalStreamLifecycle(f.io,registered,association);owner.enable();
f.set('fail_send',true);reject('disable_send_failure_not_success',@()owner.disable(),'gpenmpcFixture:StreamSendFailure');
check('failed_disable_receipt_preserved',~owner.DisableSendReturned&&owner.DisableAttempts==1 ...
    &&owner.DisableReceipts{1}.transport_receipt.partial);
f.set('fail_send',false);owner.disable();check('explicit_cleanup_retry_keeps_both_attempts',owner.DisableAttempts==2 ...
    &&numel(owner.DisableReceipts)==2&&owner.DisableSendReturned&&owner.DisableReceipts{1}.transport_receipt.partial);

f=mockFixture(expected,association,d,wire);owner=gpenmpcNative.RflyLocalStreamLifecycle(f.io,registered,association);owner.enable();
old=f.item('snapshot',true);
reject('original_pre_enable_timestamp_rejected',@()owner.observe(old),'gpenmpcNative:LocalStreamOldObservation');
check('old_observation_not_relabelled',~owner.StreamObserved ...
    &&old.original_host_receive_ns<owner.EnableReceipt.transport_receipt.original_host_submit_ns(1));
f=mockFixture(expected,association,d,wire);owner=gpenmpcNative.RflyLocalStreamLifecycle(f.io,registered,association);owner.enable();
item=f.item('snapshot',false);bad=item;bad.origin.execution_session_sha256=repmat('E',1,64);
reject('foreign_origin_rejected',@()owner.observe(bad),'gpenmpcNative:LocalStreamOrigin');
f=mockFixture(expected,association,d,wire);owner=gpenmpcNative.RflyLocalStreamLifecycle(f.io,registered,association);owner.enable();
item=f.item('snapshot',false);bad=item;bad.original_callback_indices(1)=uint64(0);
reject('non_original_callback_index_rejected',@()owner.observe(bad),'gpenmpcNative:LocalStreamOriginalIndex');
f=mockFixture(expected,association,d,wire);owner=gpenmpcNative.RflyLocalStreamLifecycle(f.io,registered,association);owner.enable();
item=f.item('snapshot',false);bad=item;bad.decoded_fragments{1}.Payload.payload(10)=bitxor(bad.decoded_fragments{1}.Payload.payload(10),uint8(1));
reject('copied_altered_callback_rejected',@()owner.observe(bad),'gpenmpcNative:LocalStreamOriginalCallback');
f=mockFixture(expected,association,d,wire);owner=gpenmpcNative.RflyLocalStreamLifecycle(f.io,registered,association);owner.enable();
item=f.item('snapshot',false);owner.observe(item);
reject('repeated_observation_generation_rejected',@()owner.observe(item),'gpenmpcNative:LocalStreamBodyIdentity');
f=mockFixture(expected,association,d,wire);owner=gpenmpcNative.RflyLocalStreamLifecycle(f.io,registered,association);owner.enable();
bad=expected;bad.link_lifecycle_generation=bad.link_lifecycle_generation+uint64(1);f.set('expected',bad);
reject('same_owner_rebound_session_rejected_before_cleanup_write',@()owner.disable(),'gpenmpcNative:LocalStreamSession');
fs=f.state();check('changed_session_no_disable_write',fs.send_calls==1&&owner.DisableAttempts==0);
report=struct('passed',all([checks.pass]),'test_count',numel(checks),'checks',checks, ...
    'scope','HOST_LOCAL_STREAM_LIFECYCLE_MOCK_IO_WITH_RETAINED_C_BODIES', ...
    'actual_command_encoder_used',true,'actual_local_decoders_used',true,'actual_tunnel_reassembler_used',true, ...
    'registration_and_callback_metadata_mock',true,'sockets_opened',0,'COM',0,'hardware_actions',0, ...
    'board_stream_enabled_proven',false,'board_stream_disabled_proven',false,'release_proven',false);
save(fullfile(outputRoot,'RAW.mat'),'checks','registered','association','expected','report');
file=fopen(fullfile(outputRoot,'RESULT.json'),'wt');assert(file>=0);fileGuard=onCleanup(@()fclose(file));
fprintf(file,'%s\n',jsonencode(report,PrettyPrint=true));clear fileGuard
disp(jsonencode(report));
catch problem
    failure=struct('scope','HOST_LOCAL_STREAM_LIFECYCLE_FIXTURE_FAILURE', ...
        'identifier',problem.identifier,'message',problem.message,'stack',problem.stack, ...
        'report',getReport(problem,'extended','hyperlinks','off'),'completed_checks',checks, ...
        'sockets_opened',0,'COM',0,'board',0,'hardware_actions',0);
    save(fullfile(outputRoot,'PARTIAL.mat'),'checks','failure');
    file=fopen(fullfile(outputRoot,'FAILURE.json'),'wt');assert(file>=0);
    fileGuard=onCleanup(@()fclose(file));fprintf(file,'%s\n',jsonencode(failure,PrettyPrint=true));clear fileGuard
    rethrow(problem)
end
    function check(name,ok)
        checks(end+1)=struct('name',name,'pass',logical(ok));assert(ok,'%s',name);
    end
    function reject(name,fn,identifier)
        ok=false;try,fn();catch ex,if ~strcmp(ex.identifier,identifier),rethrow(ex);end;ok=true;end
        check(name,ok);
    end
end
function fixture=mockFixture(expected,association,dialect,wire)
state=struct('expected',expected,'association',association,'bound',true,'fail_send',false,'fail_before_receipt',false, ...
    'transport_failed',false,'send_calls',0,'poll_calls',0,'raw_mavlink',{{}},'last_send',[]);
fixture=struct('io',struct('pollCanonical',@poll,'evidence',@evidence,'sendCanonicalSession',@send), ...
    'set',@setValue,'state',@getState,'item',@item);
    function setValue(name,value),state.(name)=value;end
    function s=getState(),s=state;end
    function s=poll()
        state.poll_calls=state.poll_calls+1;
        if state.transport_failed,error('gpenmpcFixture:FailedIngressPoll','Mock failed ingress cannot be polled.');end
        s=struct('bound',state.bound,'board_local_full_inner',true,'same_existing_mavlinkio',true, ...
            'additional_connections',0,'failure','');
    end
    function r=evidence()
        r=struct('canonical_exchange_expected',state.expected,'canonical_exchange_association',state.association, ...
            'raw_mavlink',{state.raw_mavlink},'canonical_session_last_send',state.last_send);
    end
    function r=send(action,value)
        state.send_calls=state.send_calls+1;
        if state.fail_before_receipt,error('gpenmpcFixture:StreamSendFailure','Mock API failure before a transport receipt.');end
        target=struct('system',state.expected.source_system,'component',state.expected.source_component);
        [messages,encoding]=gpenmpcNative.RflySessionCommandEncoder(action,value,dialect,target);
        assert(strcmp(action,'stream_local')&&numel(messages)==1,'Only one exact stream command in this fixture.');
        submitted=gpenmpcNative.rflyOriginalHostMonotonicNs();
        r=struct('encoding',encoding,'messages_attempted',1,'messages_send_returned',0, ...
            'original_host_submit_ns',submitted,'original_host_send_return_ns',uint64(0), ...
            'partial',false,'error','','same_existing_mavlinkio',true,'board_acknowledged',false, ...
            'final_sequence_generated_by_existing_link',true,'actual_wire_bytes_available',false);
        if state.fail_send
            r.partial=true;r.error='gpenmpcFixture:StreamSendFailure';state.last_send=r;state.transport_failed=true;
            error('gpenmpcFixture:StreamSendFailure','Explicit mock attempted prefix, not an actual socket send.');
        end
        r.messages_send_returned=1;r.original_host_send_return_ns=gpenmpcNative.rflyOriginalHostMonotonicNs();
        state.last_send=r;
    end
    function completed=item(channel,old)
        bytes=wire.(channel);e=state.expected;e.boot_generation=e.session_generation;e.require_learning_audit=true;
        reassembler=gpenmpcNative.RflyLocalTunnelReassembler(e,uint64(5000000000));
        % Use a 5 s host fixture assembly bound.
        switch channel
            case 'snapshot',s=gpenmpcNative.RflyLocalSnapshotDecoder(bytes,uint64(1));generation=s.source_generation;schema=10;
            case 'gp_request',s=gpenmpcNative.RflyLocalGpCodec.decodeRequest(bytes);generation=s.output_generation;schema=8;
            case 'committed_state',s=gpenmpcNative.RflyLocalCommittedDecoder(bytes,uint64(1));generation=s.output_generation;schema=14;
        end
        count=ceil(numel(bytes)/119);
        for k=1:count
            template=createmsg(dialect,'TUNNEL');p=template.Payload;
            part=bytes((k-1)*119+1:min(k*119,numel(bytes)));p.payload(:)=uint8(0);
            p.target_system=e.target_system;p.target_component=e.target_component;p.payload_type=uint16(42002);
            p.payload_length=uint8(9+numel(part));p.payload(1)=uint8(schema*16+k-1);
            p.payload(2:9)=be64(generation);p.payload(10:9+numel(part))=part;
            m=struct('MsgID',uint32(385),'SystemID',e.source_system,'ComponentID',e.source_component, ...
                'Seq',uint8(k),'Payload',p);
            rx=gpenmpcNative.rflyOriginalHostMonotonicNs();
            if old,rx=state.last_send.original_host_submit_ns(1)-uint64(count-k+1);end
            row=struct('topic','TUNNEL','original_host_receive_ns',rx,'decoded_message',m, ...
                'decoded_source','ORIGINAL_MAVLINKIO_TUNNEL_CALLBACK','raw_frame_available',false);
            state.raw_mavlink{end+1}=row;
            origin=struct('execution_session_sha256',e.execution_session_sha256, ...
                'link_lifecycle_generation',e.link_lifecycle_generation,'original_callback_index',uint64(numel(state.raw_mavlink)));
            reassembler.ingest(m,rx,gpenmpcNative.rflyOriginalHostMonotonicNs(),origin);
        end
        completed=reassembler.take(channel,gpenmpcNative.rflyOriginalHostMonotonicNs());assert(~isempty(completed));
    end
end
function b=readFirst(path,count)
file=fopen(path,'rb');assert(file>=0);guard=onCleanup(@()fclose(file)); %#ok<NASGU>
b=fread(file,count,'*uint8');assert(numel(b)==count);
end
function b=be64(v)
[~,~,order]=computer;if order=='L',v=swapbytes(v);end;b=reshape(typecast(v,'uint8'),[],1);
end
function h=hex(b),h=upper(reshape(dec2hex(b,2).',1,[]));end
