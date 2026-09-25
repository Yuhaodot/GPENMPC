function report=test_m600_local_gp_same_io(outputRoot,fixtureRoot,committedFixture,taskFixture,runtimeStateOnly,sourcePairOnly,compiledGp)
% Test GP callback and same-link RGR1 replies with loopback peers.
arguments
    outputRoot (1,1) string
    fixtureRoot (1,1) string
    committedFixture (1,1) string = ""
    taskFixture (1,1) string = ""
    runtimeStateOnly (1,1) logical = false
    sourcePairOnly (1,1) logical = false
    compiledGp (1,1) logical = false
end
build=string(fileparts(fileparts(mfilename('fullpath'))));oldPath=path;
io=[];peer=[];board=[];service=[];
try
addpath(fullfile(build,'host_runtime'),fullfile(build,'matlab_validation'), ...
    fullfile(build,'m600_coptersim','matlab_validation'));
assert(~isfolder(outputRoot));mkdir(outputRoot);diary(fullfile(outputRoot,'MATLAB_DIARY.txt'));
sp=fullfile(fixtureRoot,'RLS1_SNAPSHOTS.bin');gp=fullfile(fixtureRoot,'RGP1_RGR1_PAIRS.bin');
sourcePaths=[string(mfilename('fullpath'))+'.m';sp;gp; ...
    string(which('m600check.makeM600CopterSimIo'));string(which('gpenmpcNative.RflyLocalTunnelReassembler')); ...
    string(which('gpenmpcNative.validateLocalGpReplyForSend'));string(which('gpenmpcNative.RflyLocalGpService')); ...
    string(which('gpenmpcNative.RflyLocalGpCodec'));string(which('gpenmpcNative.RflyLocalSnapshotDecoder'))];
if strlength(committedFixture)>0
    sourcePaths=[sourcePaths;committedFixture;string(which('gpenmpcNative.RflyLocalCommittedDecoder'))];
end
if strlength(taskFixture)>0
    sourcePaths=[sourcePaths;taskFixture;string(which('gpenmpcNative.RflyLocalTaskCodec'));string(which('gpenmpcNative.validateLocalTaskInputForSend'))];
end
sourceHashes=arrayfun(@sha,sourcePaths);
snap=reshape(readbytes(sp),382,[]);pairs=reshape(readbytes(gp),596,[]);
assets=gpenmpcNative.loadCanonicalAssets();
q=gpenmpcNative.RflyLocalGpCodec.decodeRequest(pairs(1:310,1));
gpenmpcNative.RflyLocalSnapshotDecoder(snap(:,1),uint64(100));
cfg=struct('local_mavlink_port',62321,'remote_mavlink_port',62322,'truth_port',62323,'coptersim_time_port',62324, ...
    'target_system',double(q.identity.system),'target_component',double(q.identity.component), ...
    'clock_max_rtt_s',1,'clock_sync_samples',2,'clock_sync_period_s',1,'clock_max_age_s',5, ...
    'clock_max_uncertainty_s',1,'clock_max_utc_drift_s',1,'clock_max_time_heartbeat_age_s',5, ...
    'clock_max_time_heartbeat_lag_s',5,'maximum_truth_lag_s',5,'maximum_raw_records',1000,'state_max_age_s',1, ...
    'live_enabled',true,'outer_preflight_pass',true, ...
    'canonical_exchange',struct('runtime','BOARD_LOCAL_FULL_INNER', ...
        'assembly_limit_ns',uint64(5000000000),'gp_reply_host_max_age_ns',uint64(5000000000), ...
        'local_input_host_max_age_ns',uint64(5000000000),'completed_queue_capacity',4));
checks=struct('name',{},'pass',{});
if runtimeStateOnly,cfg.local_short=struct('runtime_state_only',true,'environment_ledger_capacity',256);end
bad=cfg;bad.canonical_exchange.runtime='WRONG';
check('unknown_runtime_rejected_before_socket',reject(@()m600check.makeM600CopterSimIo(bad)));
bad=cfg;bad.canonical_exchange=rmfield(bad.canonical_exchange,'gp_reply_host_max_age_ns');
check('missing_reply_bound_rejected_before_socket',reject(@()m600check.makeM600CopterSimIo(bad)));
io=m600check.makeM600CopterSimIo(cfg);
peer=udpport('datagram','IPV4','LocalHost','127.0.0.1','LocalPort',cfg.remote_mavlink_port,'Timeout',.2);
d=mavlinkdialect(fullfile(build,'m600_coptersim','matlab_validation','+m600check','px4_health_events.xml'),2);
board=mavlinkio(d,'SystemID',double(q.identity.system),'ComponentID',double(q.identity.component));
now=gpenmpcNative.rflyOriginalHostMonotonicNs();
e=struct('source_system',q.identity.system,'source_component',q.identity.component, ...
    'target_system',uint8(255),'target_component',uint8(190),'uid',q.identity.uid, ...
    'session_generation',q.identity.boot_generation,'link_lifecycle_generation',uint64(7), ...
    'confirmed_host_rx_ns',now,'execution_session_sha256',repmat('B',1,64), ...
    'configuration_sha256',upper(reshape(dec2hex(q.configuration_sha256,2).',1,[])));
serviceExpected=struct('uid',e.uid,'boot_generation',e.session_generation, ...
    'board_system',e.source_system,'board_component',e.source_component,'host_system',e.target_system, ...
    'host_component',e.target_component,'link_lifecycle_generation',e.link_lifecycle_generation, ...
    'confirmed_host_rx_ns',e.confirmed_host_rx_ns,'execution_session_sha256',e.execution_session_sha256);
prepared=[];
if compiledGp
 p=fullfile(gpenmpc_external_path('canonical_gp_wire_mex'),'canonical_gp_wire_mex.mexw64');
 addpath(fileparts(p));
 serviceExpected.gp_backend=struct('kind','CANONICAL_GP_WIRE_MEX','path',p, ...
  'binary_sha256','21017CD36C857466AE538EAE716C868173D2C54DF4D2B5CC458E3C6681CEBE6B');
 prepared=gpenmpcNative.RflyLocalGpService.prepareBackend(assets,serviceExpected.gp_backend);
end
service=gpenmpcNative.RflyLocalGpService(assets,serviceExpected,prepared);
if compiledGp,check('prepared_same_owner_original_GP_not_process_worker',~service.AsyncEnabled&&isempty(prepared.async_pool));end
state=io.bindCanonicalSession(e);check('actual_io_selects_local_only',state.board_local_full_inner ...
    &&isequal(state.channels,["gp_request","snapshot","committed_state"])&&state.additional_connections==0);
if sourcePairOnly
    assert(runtimeStateOnly&&strlength(committedFixture)>0);
    for j=1:2
        ds=gpenmpcNative.RflyLocalSnapshotDecoder(snap(:,j),now);
        frames=pack(snap(:,j),10,ds.source_generation);
        for k=1:numel(frames),write(peer,frames{k},'uint8','127.0.0.1',cfg.local_mavlink_port);end
    end
    waitCount(8);queued=io.pollCanonical(false);
    check('both_original_sources_retained_without_capacity_change',queued.completed_queue_counts(2)==2&&queued.completed_queue_capacity==4);
    sources={io.takeCanonical('snapshot',false),io.takeCanonical('snapshot',false)};
    check('latest_input_source_and_older_numeric_source_both_valid',sources{1}.decoded.source_generation==1 ...
        &&sources{2}.decoded.source_generation==2&&isequal(sources{1}.message,snap(:,1)) ...
        &&isequal(sources{2}.message,snap(:,2)));
    check('original_callback_times_kept',sources{1}.original_host_receive_ns==sources{1}.fragment_rx_ns(1) ...
        &&sources{2}.original_host_receive_ns==sources{2}.fragment_rx_ns(1) ...
        &&sources{2}.original_host_receive_ns>=sources{1}.original_host_receive_ns);
    cb=reshape(readbytes(committedFixture),1494,[]);dc=gpenmpcNative.RflyLocalCommittedDecoder(cb(:,1),now);
    frames=pack(cb(:,1),14,dc.output_generation);
    for k=1:numel(frames),write(peer,frames{k},'uint8','127.0.0.1',cfg.local_mavlink_port);end
    waitCount(21);record=io.takeCanonical('committed_state',false);
    check('later_RLC_matches_retained_original_not_latest_state',record.decoded.source_generation==sources{1}.decoded.source_generation ...
        &&record.decoded.source_timestamp_ns==sources{1}.decoded.original_sample_us*uint64(1000) ...
        &&record.decoded.source_generation~=sources{2}.decoded.source_generation);
    badBytes=snap(:,3);badBytes(200)=bitxor(badBytes(200),uint8(1));
    frames=pack(badBytes,10,uint64(3));
    for k=1:numel(frames),write(peer,frames{k},'uint8','127.0.0.1',cfg.local_mavlink_port);end
    waitCount(25);check('invalid_body_still_rejected_before_numeric_use',reject(@()io.takeCanonical('snapshot',false)));
    report=struct('all_pass',all([checks.pass]),'checks',checks,'total',numel(checks), ...
        'scope','ACTUAL_SAME_IO_RETAINED_SOURCE_PAIR_ONLY_NO_BOARD_OR_SOLVE','COM',0,'board',0,'outputs',0);
    evidence=io.evidence();save(fullfile(outputRoot,'RAW.mat'),'report','sources','record','evidence');
    writejson(fullfile(outputRoot,'RESULT.json'),report);disp(jsonencode(report));
    assert(report.all_pass);finish();return
end
s=gpenmpcNative.RflyLocalSnapshotDecoder(snap(:,1),now);
sf=pack(snap(:,1),10,s.source_generation);gf=pack(pairs(1:310,1),8,q.output_generation);
for k=1:4
    write(peer,sf{k},'uint8','127.0.0.1',cfg.local_mavlink_port);
    if k<=3,write(peer,gf{k},'uint8','127.0.0.1',cfg.local_mavlink_port);end
end
waitCount(7);request=io.takeCanonical('gp_request');observation=io.takeCanonical('snapshot');
check('real_callbacks_preserve_both_actual_c_bodies',isequal(request.message,pairs(1:310,1)) ...
    &&isequal(observation.message,snap(:,1))&&numel(request.decoded_fragments)==3);
result=service.process(request.message,request.original_host_receive_ns, ...
    gpenmpcNative.rflyOriginalHostMonotonicNs(),request.origin);
c=gpenmpcNative.RflyLocalGpCodec.decodeReply(pairs(311:596,1));
check('actual_original_gp18_one_call',service.Calls==1&&service.Completed==1 ...
    &&max(abs(result.result18(:)-c.result18(:)))<=1e-10);
sent=io.sendCanonicalLocalGp(result.reply_bytes,request);wire=cell(3,1);joined=zeros(286,1,'uint8');
for k=1:3
    timer=tic;while peer.NumDatagramsAvailable==0&&toc(timer)<3,pause(.005);end
    assert(peer.NumDatagramsAvailable>0);row=read(peer,1,'uint8');wire{k}=uint8(row.Data(:));
    m=deserializemsg(d,wire{k});p=m.Payload;n=double(p.payload_length)-9;
    check(sprintf('actual_gp_reply_wire_%d',k),m.MsgID==385&&m.SystemID==255&&m.ComponentID==190 ...
        &&p.payload_type==42002&&p.target_system==e.source_system&&p.target_component==e.source_component ...
        &&p.payload(1)==uint8(144+k-1));
    joined(119*(k-1)+1:119*(k-1)+n)=p.payload(10:9+n);
end
check('actual_three_wire_frames_reconstruct_original_reply',isequal(joined,result.reply_bytes) ...
    &&sent.messages_send_returned==3&&sent.same_existing_mavlinkio&&~sent.board_receipt_proven);
evidence=io.evidence();tx=evidence.raw_transmit_messages;
check('send_attempts_and_returns_retained',numel(tx)==3 ...
    &&all(cellfun(@(x)x.send_attempted&&x.send_returned,tx)) ...
    &&all(cellfun(@(x)x.message.MsgID==385&&x.message.Payload.payload_type==42002,tx)));
check('original_callback_clock_not_renewed',request.original_host_receive_ns==request.fragment_rx_ns(1) ...
    &&sent.binding.original_host_receive_ns==request.original_host_receive_ns ...
    &&~sent.binding.control_publication_expiry_used);
check('legacy_numeric_sender_disabled',reject(@()io.sendCanonicalPackets({uint8(1)})));
receivedFrames=7;closedIncoming={};
if strlength(committedFixture)>0
    rawClosed=readbytes(committedFixture);learning=isequal(rawClosed(1:4),uint8('RLC2').');
    bodyLength=1398;wireSchema=12;if learning,bodyLength=1494;wireSchema=14;end
    closedBytes=reshape(rawClosed,bodyLength,[]);assert(size(closedBytes,2)==60);
    fragmentCount=ceil(bodyLength/119);
    for k=1:60
        decoded=gpenmpcNative.RflyLocalCommittedDecoder(closedBytes(:,k),uint64(100));
        frames=pack(closedBytes(:,k),wireSchema,decoded.output_generation);
        for j=1:numel(frames),write(peer,frames{j},'uint8','127.0.0.1',cfg.local_mavlink_port);end
        receivedFrames=receivedFrames+fragmentCount;waitCount(receivedFrames);
        if runtimeStateOnly&&k==1
            pending=io.evidence();queued=pending.canonical_completed_queues{3}{1};
            check('RLC_complete_original_retained_without_RX_numeric_decode',isempty(queued.decoded) ...
                &&isequal(queued.message,closedBytes(:,k))&&~queued.control_authority&&~queued.freshness_renewed);
        end
        item=io.takeCanonical('committed_state');
        closedIncoming{end+1,1}=item; %#ok<AGROW>
        check(sprintf('actual_RLC_same_io_%02d',k),isequal(item.message,closedBytes(:,k)) ...
            &&numel(item.decoded_fragments)==fragmentCount&&item.decoded.joint_installs==uint64(k) ...
            &&item.decoded.query_sequence==uint64(k)&&~item.decoded.next_open_prediction_included ...
            &&~item.decoded.control_authority&&~item.freshness_renewed);
        v=item.decoded;decodedFields=[be(v.state64);be(v.closed5);be(v.reference6); ...
            be(v.installed_phase2);be(v.kernel61);be(v.published_control16)];
        check(sprintf('actual_C_to_MATLAB_all_numeric_bits_%02d',k),isequal(decodedFields,closedBytes(99:1266,k)));
        if learning
            check(sprintf('actual_C_to_MATLAB_learning_bits_%02d',k),v.learning_audit_available ...
                &&isequal(be(v.learning12),closedBytes(1367:1462,k)) ...
                &&isequaln(v.closed_gp_evidence.observed_innovation_f_mps2,v.learning12(1:3)));
        end
    end
    for k=1:5
        b=closedBytes(:,1);
        if k==1,b(1267)=uint8(5);end % Historical-state fixture.
        if k==2,b(1268)=uint8(2);end % invalid availability flag
        if k==3,b(95:98)=be(uint32(6));end
        if k==4,b(23:30)=uint8(0);end
        if k==5,b(611:618)=be(0.5);end % Ambiguous closure flag.
        md=java.security.MessageDigest.getInstance('SHA-256');md.update(typecast(b(1:end-32),'int8'));
        b(end-31:end)=reshape(typecast(md.digest(),'uint8'),[],1);
        check(sprintf('closed_semantic_negative_%d',k),reject(@()gpenmpcNative.RflyLocalCommittedDecoder(b,uint64(100))));
    end
end
taskFrames=0;taskSent=[];taskMessage=[];
if strlength(taskFixture)>0
    pairsInput=reshape(readbytes(taskFixture),1029,[]);
    for k=1:size(pairsInput,2)
        rb=pairsInput(1:382,k);tb=pairsInput(383:end,k);
        m=gpenmpcNative.RflyLocalTaskCodec.decode(tb);
        bound=struct('schema','RFLY_LOCAL_ORIGINAL_INPUT_BINDING_V1','numerical_inputs_complete',true, ...
            'control_authority',false,'source',m.source,'rotor',m.rotor,'payload',m.payload,'wind',m.wind,'leg_index',uint64(m.leg_index));
        registered=struct('identity',m.source.identity,'execution_session_sha256',hex(m.execution_session_sha256), ...
            'configuration_sha256',hex(m.configuration_sha256),'task_sha256',hex(m.task_sha256), ...
            'reference_asset_sha256',hex(m.reference_asset_sha256),'leg_index',m.leg_index);
        rebuilt=gpenmpcNative.RflyLocalTaskCodec.encode(bound,rb,uint64(100),m.outer,registered);
        check(sprintf('actual_CPP_RLI_roundtrip_%d',k),isequal(rebuilt,tb));
    end
    % The original numeric source is reused; ONLY the explicitly simulated
    % session/hash and HOST callback/creation clock are rebound for loopback.
    fs=pack(rb,10,m.source.source_generation);
    for k=1:4,write(peer,fs{k},'uint8','127.0.0.1',cfg.local_mavlink_port);end
    receivedFrames=receivedFrames+4;waitCount(receivedFrames);originalSource=io.takeCanonical('snapshot');
    check('new_source_original_callback_received',isequal(originalSource.message,rb));
    registered.execution_session_sha256=e.execution_session_sha256;
    outer=m.outer;outer.original_host_source_receive_ns=originalSource.original_host_receive_ns;
    outer.original_creation_ns=gpenmpcNative.rflyOriginalHostMonotonicNs();
    outer.original_expiry_ns=outer.original_host_source_receive_ns+cfg.canonical_exchange.local_input_host_max_age_ns;
    taskMessage=gpenmpcNative.RflyLocalTaskCodec.encode(bound,rb,originalSource.original_host_receive_ns,outer,registered);
    taskSent=io.sendCanonicalLocalInput(taskMessage,originalSource);joinedTask=zeros(647,1,'uint8');
    for k=1:6
        timer=tic;while peer.NumDatagramsAvailable==0&&toc(timer)<3,pause(.005);end
        assert(peer.NumDatagramsAvailable>0);row=read(peer,1,'uint8');wireTask=uint8(row.Data(:));
        packet=deserializemsg(d,wireTask);p=packet.Payload;n=double(p.payload_length)-9;
        check(sprintf('same_io_RLI_fragment_%d',k),packet.MsgID==385&&packet.SystemID==255&&packet.ComponentID==190 ...
            &&p.payload_type==42002&&p.target_system==e.source_system&&p.target_component==e.source_component&&p.payload(1)==uint8(208+k-1));
        joinedTask(119*(k-1)+1:119*(k-1)+n)=p.payload(10:9+n);
    end
    taskFrames=6;
    check('RLI_actual_same_io_bytes',isequal(joinedTask,taskMessage)&&taskSent.messages_send_returned==6&&taskSent.same_existing_mavlinkio);
    for k=1:3
        forgedSource=originalSource;when=gpenmpcNative.rflyOriginalHostMonotonicNs();
        if k==1,forgedSource.origin.link_lifecycle_generation=uint64(8);end
        if k==2,forgedSource.decoded_fragments{2}.Payload.payload(30)=bitxor(forgedSource.decoded_fragments{2}.Payload.payload(30),uint8(1));end
        if k==3,when=outer.original_expiry_ns+uint64(1);end
        check(sprintf('RLI_origin_bytes_expiry_negative_%d',k),reject(@()gpenmpcNative.validateLocalTaskInputForSend( ...
            taskMessage,forgedSource,e,cfg.canonical_exchange.local_input_host_max_age_ns,when)));
    end
end
% Pure validator negative inputs cannot cause sends or alter real IO state.
for k=1:5
    forged=request;reply=result.reply_bytes;now=gpenmpcNative.rflyOriginalHostMonotonicNs();
    if k==1,forged.message(100)=bitxor(forged.message(100),uint8(1));end
    if k==2,forged.decoded_fragments{2}.Payload.payload(30)=bitxor(forged.decoded_fragments{2}.Payload.payload(30),uint8(1));end
    if k==3,forged.origin.link_lifecycle_generation=uint64(8);end
    if k==4,reply(200)=bitxor(reply(200),uint8(1));end
    if k==5,now=request.original_host_receive_ns+cfg.canonical_exchange.gp_reply_host_max_age_ns+uint64(1);end
    check(sprintf('pure_negative_%d_no_io',k),reject(@()gpenmpcNative.validateLocalGpReplyForSend( ...
        reply,forged,e,cfg.canonical_exchange.gp_reply_host_max_age_ns,now)));
end
% Structurally valid but not this IO's original callback must be rejected.
retiredGpRows=0;retirement=[];
if runtimeStateOnly
    % Let the original real loopback receipt expire. No timestamp is changed
    % and no fixture clock is substituted into the real sender.
    expiry=request.original_host_receive_ns+cfg.canonical_exchange.gp_reply_host_max_age_ns;
    left=double(max(expiry,gpenmpcNative.rflyOriginalHostMonotonicNs())-gpenmpcNative.rflyOriginalHostMonotonicNs())*1e-9;
    if left>0,pause(left+.002);end
    beforeRetire=io.evidence();retirement=io.sendCanonicalLocalGp(result.reply_bytes,request);
    afterRetire=io.evidence();retiredGpRows=3;
    added=afterRetire.raw_transmit_messages(numel(beforeRetire.raw_transmit_messages)+1:end);
    check('expired_original_reply_retired_without_wire_or_exchange_failure', ...
        strcmp(retirement.status,'EXPIRED_UNSENT_GP_NOT_USED')&&retirement.messages_send_returned==0 ...
        &&~retirement.control_authority&&isempty(afterRetire.canonical_exchange_failure) ...
        &&numel(added)==3&&all(cellfun(@(x)~x.send_attempted&&~x.send_returned,added)) ...
        &&peer.NumDatagramsAvailable==0);
    check('expired_reply_keeps_original_anchor_no_renewal', ...
        retirement.binding.original_host_receive_ns==request.original_host_receive_ns ...
        &&retirement.binding.valid_until_host_ns==expiry);
    check('retirement_never_accepts_future_request_time',reject(@()gpenmpcNative.validateLocalGpReplyForSend( ...
        result.reply_bytes,request,e,cfg.canonical_exchange.gp_reply_host_max_age_ns, ...
        request.fragment_rx_ns(end)-uint64(1),true)));
end
forged=request;forged.fragment_rx_ns=forged.fragment_rx_ns+uint64(1);
forged.original_host_receive_ns=forged.fragment_rx_ns(1);forged.completion_host_receive_ns=forged.fragment_rx_ns(end);
check('forged_callback_origin_no_second_send',reject(@()io.sendCanonicalLocalGp(result.reply_bytes,forged)));
check('no_extra_datagrams_after_rejection',peer.NumDatagramsAvailable==0);
final=io.evidence();check('all_actual_tx_accounted_GP_and_inputs_only',numel(final.raw_transmit_messages)==3+taskFrames+retiredGpRows);
io.close();service.close();
if runtimeStateOnly&&strlength(committedFixture)>0
    % Reject malformed completed history bodies in the IO consumer before Phase or Outer.
    for negative=1:3
        io=m600check.makeM600CopterSimIo(cfg);io.bindCanonicalSession(e);
        b=closedBytes(:,1);v=gpenmpcNative.RflyLocalCommittedDecoder(b,uint64(100));gen=v.output_generation;
        if negative==1,b(end)=bitxor(b(end),uint8(1));end
        if negative==2,gen=gen+uint64(1);end
        if negative==3
            b(1269)=uint8(0);md=java.security.MessageDigest.getInstance('SHA-256');
            md.update(typecast(b(1:end-32),'int8'));b(end-31:end)=reshape(typecast(md.digest(),'uint8'),[],1);
        end
        frames=pack(b,wireSchema,gen);
        for j=1:numel(frames),write(peer,frames{j},'uint8','127.0.0.1',cfg.local_mavlink_port);end
        waitCount(fragmentCount);
        check(sprintf('deferred_RLC_invalid_%d_rejected_before_consumer',negative),reject(@()io.takeCanonical('committed_state')));
        rejected=io.evidence();
        check(sprintf('deferred_RLC_invalid_%d_latched_without_send',negative), ...
            ~isempty(rejected.canonical_exchange_failure)&&isempty(rejected.raw_transmit_messages));
        io.close();
    end
end
after=arrayfun(@sha,sourcePaths);check('source_identity_stable',isequal(sourceHashes,after));
save(fullfile(outputRoot,'RAW.mat'),'evidence','final','request','observation','result','sent','wire','cfg','closedIncoming','taskSent','taskMessage');
report=struct('scope','ACTUAL_MATLAB_SAME_MAVLINKIO_LOCAL_GP_LOOPBACK_ONLY','checks',checks, ...
    'passed',sum([checks.pass]),'total',numel(checks),'all_pass',all([checks.pass]), ...
    'received_private_frames',receivedFrames,'actual_closed_rows',numel(closedIncoming), ...
    'actual_original_gp_calls',1,'actual_outgoing_gp_frames',3, ...
    'actual_outgoing_task_input_frames',taskFrames, ...
    'source_paths',sourcePaths,'sha256',sourceHashes,'admission_identity_fixture',true, ...
    'COM',0,'board',0,'arm_mode_task_commands',0,'outputs',0, ...
    'official_mavlinkio_decode_and_same_owner_send',true,'local_ports',[62321 62322 62323 62324]);
writejson(fullfile(outputRoot,'RESULT.json'),report);
fprintf('Real same-IO local GP %d/%d\n',report.passed,report.total);assert(report.all_pass);
catch failure
    finish();rethrow(failure)
end
finish();
    function check(name,pass)
        checks(end+1)=struct('name',name,'pass',logical(pass)); %#ok<AGROW>
        if ~pass,fprintf(2,'FAILED %s\n',name);end
    end
    function frames=pack(b,schema,gen)
        frames=cell(ceil(numel(b)/119),1);
        for j=1:numel(frames)
            frameMessage=createmsg(d,'TUNNEL');frameMessage.Payload.target_system=uint8(255);frameMessage.Payload.target_component=uint8(190);
            frameMessage.Payload.payload_type=uint16(42002);n=min(119,numel(b)-(j-1)*119);
            frameMessage.Payload.payload_length=uint8(9+n);frameMessage.Payload.payload(1)=bitor(bitshift(uint8(schema),4),uint8(j-1));
            frameMessage.Payload.payload(2:9)=be(gen);frameMessage.Payload.payload(10:9+n)=b(119*(j-1)+1:119*(j-1)+n);
            frames{j}=uint8(serializemsg(board,frameMessage));
        end
    end
    function waitCount(n)
        timer=tic;
        while toc(timer)<3
            st=io.pollCanonical();if st.status.private_fragments_received>=n,return;end
            pause(.005);
        end
        error('gpenmpcNative:LocalIoTestTimeout','Expected actual callback count not observed.');
    end
    function finish()
        % One ordered cleanup: classes stay on path until owners close.
        if ~isempty(service),try,service.close();catch ex,warning('%s',ex.message);end,end
        if ~isempty(io)
            try
                if isfolder(outputRoot)&&~isfile(fullfile(outputRoot,'RESULT.json'))
                    partialIo=io.evidence(); %#ok<NASGU>
                    save(fullfile(outputRoot,'FAILURE_RAW.mat'),'partialIo');
                end
            catch ex,warning('%s',ex.message);end
            try,io.close();catch ex,warning('%s',ex.message);end
        end
        if ~isempty(peer),try,delete(peer);catch ex,warning('%s',ex.message);end,end
        if ~isempty(board),try,delete(board);catch ex,warning('%s',ex.message);end,end
        diary('off');path(oldPath);
    end
end
function b=be(v)
[~,~,e]=computer;if e=='L',v=swapbytes(v);end;b=reshape(typecast(v,'uint8'),[],1);
end
function b=readbytes(p)
f=fopen(p,'rb');assert(f>=0);g=onCleanup(@()fclose(f));b=fread(f,Inf,'*uint8'); %#ok<NASGU>
end
function ok=reject(f)
ok=false;try,f();catch,ok=true;end
end
function s=hex(b),s=upper(reshape(dec2hex(b,2).',1,[]));end
function h=sha(p)
b=readbytes(p);md=java.security.MessageDigest.getInstance('SHA-256');md.update(typecast(b,'int8'));
h=string(upper(reshape(dec2hex(typecast(md.digest(),'uint8'),2).',1,[])));
end
function writejson(p,r)
f=fopen(p,'w','n','UTF-8');assert(f>=0);g=onCleanup(@()fclose(f));fprintf(f,'%s\n',jsonencode(r,PrettyPrint=true)); %#ok<NASGU>
end
