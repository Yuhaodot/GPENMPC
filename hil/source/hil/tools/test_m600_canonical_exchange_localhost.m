function report=test_m600_canonical_exchange_localhost(outputRoot,boardStopOnly)
% Test makeM600CopterSimIo callbacks and sends on loopback ports.
arguments,outputRoot (1,1) string,boardStopOnly (1,1) logical=false,end
build=string(fileparts(fileparts(mfilename('fullpath'))));
addpath(fullfile(build,'host_runtime'),fullfile(build,'matlab_validation'),fullfile(build,'m600_coptersim','matlab_validation'));
if boardStopOnly,assert(isfolder(outputRoot));else,assert(~isfolder(outputRoot));mkdir(outputRoot);end
sourcePaths={which('m600check.makeM600CopterSimIo'),which('m600check.decodeCanonicalExchangePackets'), ...
    which('gpenmpcNative.RflyTunnelReassembler'),which('gpenmpcNative.RflySessionCommandEncoder'), ...
    [mfilename('fullpath') '.m']};
sourceHashes=cellfun(@fileSha,sourcePaths,'UniformOutput',false);
base=fullfile(gpenmpc_external_path('board_commit_exchange_fixture'));
source=readbin(fullfile(base,'SOURCE_1.bin'));feedback=readbin(fullfile(base,'FEEDBACK1112.bin'));
context=readbin(fullfile(base,'CONTEXT316.bin'));numeric=readbin(fullfile(base,'COMMAND381.bin'));
cfg=struct('local_mavlink_port',62101,'remote_mavlink_port',62102,'truth_port',62103,'coptersim_time_port',62104, ...
    'target_system',1,'target_component',1,'clock_max_rtt_s',1,'clock_sync_samples',2,'clock_sync_period_s',1, ...
    'clock_max_age_s',5,'clock_max_uncertainty_s',1,'clock_max_utc_drift_s',1,'clock_max_time_heartbeat_age_s',5, ...
    'clock_max_time_heartbeat_lag_s',5,'maximum_truth_lag_s',5,'maximum_raw_records',1000,'state_max_age_s',1, ...
    'live_enabled',true,'outer_preflight_pass',true,'canonical_exchange',struct('assembly_limit_ns',uint64(2000000000)));
cfg.canonical_rotor_observer=struct('local_port',62105,'maximum_queue',2);
if boardStopOnly
    cfg.local_mavlink_port=62311;cfg.remote_mavlink_port=62312;
    cfg.truth_port=62313;cfg.coptersim_time_port=62314;cfg.canonical_rotor_observer.local_port=62315;
    cfg.mavlink_transport=struct('source',struct('kind','GPENMPC_RFLY_UDP_TRANSPORT_MEX', ...
        'exact_path',fullfile(gpenmpc_external_path('rfly_udp_transport_build'),'production','gpenmpc_rfly_udp_transport_mex.mexw64'), ...
        'sha256','412250893D93D8035A4D2F357E2862807BF11A1333CE3CF7653A128B3E3EAC32'), ...
        'scope','HOST_ONLY_LOOPBACK','allow_loopback',true,'local_host','127.0.0.1','remote_host','127.0.0.1', ...
        'local_port',62311,'remote_port',62312,'local_system',uint8(255),'local_component',uint8(190), ...
        'remote_system',uint8(1),'remote_component',uint8(1),'maximum_poll_datagrams',8,'copter_serial_forwarding',true);
    addpath(fileparts(cfg.mavlink_transport.source.exact_path));
    cfg.canonical_exchange.runtime='BOARD_LOCAL_FULL_INNER';
    cfg.canonical_exchange.gp_reply_host_max_age_ns=uint64(50000000);
    cfg.local_short=struct('environment_ledger_capacity',4,'runtime_state_only',true,'component_initialization',true);
end
checks=struct('name',{},'pass',{});failures={};
badRotorCfg=cfg;badRotorCfg.canonical_rotor_observer.local_port=cfg.truth_port;
reject(@()m600check.makeM600CopterSimIo(badRotorCfg),'m600check:CanonicalRotorConfig','rotor_distinct_port_before_socket');
badRotorCfg=cfg;badRotorCfg.canonical_rotor_observer.maximum_queue=0;
reject(@()m600check.makeM600CopterSimIo(badRotorCfg),'m600check:CanonicalRotorConfig','rotor_bounded_queue_before_socket');
conflict=cfg;conflict.canonical_se3_status_observation=struct();
reject(@()m600check.makeM600CopterSimIo(conflict),'m600check:CanonicalSerialObserverConflict','observer_conflict_before_socket');
io=m600check.makeM600CopterSimIo(cfg);ownerCleanup=onCleanup(@()io.close()); %#ok<NASGU>
peer=udpport('datagram','IPV4','LocalPort',cfg.remote_mavlink_port,'Timeout',0.2);
peerCleanup=onCleanup(@()delete(peer)); %#ok<NASGU>
health=fullfile(build,'m600_coptersim','matlab_validation','+m600check','px4_health_events.xml');
d=mavlinkdialect(health,2);boardSerializer=mavlinkio(d,'SystemID',1,'ComponentID',1);
inputSerializer=mavlinkio(d,'SystemID',255,'ComponentID',190);
serializerCleanup=onCleanup(@()deleteSerializers(boardSerializer,inputSerializer)); %#ok<NASGU>
if boardStopOnly
    expected=struct('source_system',uint8(1),'source_component',uint8(1), ...
        'target_system',uint8(255),'target_component',uint8(190),'uid',uint64(1), ...
        'session_generation',uint64(1),'link_lifecycle_generation',uint64(1), ...
        'confirmed_host_rx_ns',gpenmpcNative.rflyOriginalHostMonotonicNs(), ...
        'execution_session_sha256',repmat('B',1,64));
    io.bindCanonicalSession(expected);
    serial=createmsg(d,'SERIAL_CONTROL');serial.Payload.device=uint8(10);serial.Payload.flags=uint8(1);
    text=uint8(sprintf('ERROR [gpenmpc_rfly_components] LOCAL_CTX f=7 e=5 c=3\n'));
    serial.Payload.count=uint8(numel(text));serial.Payload.data(1:numel(text))=text;
    write(peer,serializemsg(boardSerializer,serial),'uint8','127.0.0.1',cfg.local_mavlink_port);
    pause(.03);io.pollCanonical();before=io.evidence();
    check('actual_board_stop_callback_latched',strcmp(before.fatal,'BOARD_LOCAL_CONTEXT_STOPPED:7'));
    reject(@()io.takeCanonical('gp_request',false),'m600check:CanonicalExchangeClosed','board_stop_refuses_next_control');
    after=io.evidence();
    check('wrapper_does_not_invent_transport_failure',isempty(after.canonical_exchange_failure)&&isempty(after.non_model_fatal));
    check('first_board_cause_retained',isequal(before.first_fatal,after.first_fatal));
    reject(@()io.takeCanonical('snapshot',false),'m600check:CanonicalExchangeClosed','board_stop_cannot_resume');
    % Complete CRC-checked native receive batch after the controller stopped.
    % Its first unrelated packet must not discard following safety telemetry.
    ping=createmsg(d,'PING');heartbeat=createmsg(d,'HEARTBEAT');heartbeat.Payload.base_mode=uint8(0);
    ground=createmsg(d,'EXTENDED_SYS_STATE');ground.Payload.landed_state=uint8(1);
    ack=createmsg(d,'COMMAND_ACK');ack.Payload.command=uint16(400);ack.Payload.result=uint8(0);
    batch=uint8([]);
    for message={ping,heartbeat,ground,ack}
        wire=uint8(serializemsg(boardSerializer,message{1}));batch=[batch;wire(:)]; %#ok<AGROW>
    end
    write(peer,batch,'uint8','127.0.0.1',cfg.local_mavlink_port);pause(.02);
    for k=1:3,io.pollCanonical();end
    safety=io.snapshot(@()[],false);after=io.evidence();
    check('stopped_controller_still_receives_disarmed_ground',safety.armed==0&&safety.landed_state==1 ...
        &&isfinite(safety.heartbeat_rx_s)&&isfinite(safety.extended_rx_s));
    check('stopped_controller_retains_entire_decoded_batch',sum(cellfun(@(r)strcmp(r.topic,'COMMAND_ACK'),after.raw_mavlink))==1);
    check('safety_receive_never_clears_control_stop',strcmp(safety.fatal,before.fatal)&&isempty(after.non_model_fatal));
    bad=uint8(serializemsg(boardSerializer,heartbeat));bad(end)=bitxor(bad(end),uint8(1));
    write(peer,bad,'uint8','127.0.0.1',cfg.local_mavlink_port);pause(.02);io.pollCanonical();
    failed=io.evidence();
    check('actual_crc_failure_remains_fatal_after_board_stop',~isempty(failed.non_model_fatal));
    check('original_board_stop_remains_first_failure',isequal(before.first_fatal,failed.first_fatal));
    check('same_owner_closed',io.close());
    report=struct('passed',all([checks.pass]),'checks',checks,'board_actions',0,'COM',0, ...
        'scope','AFFECTED_BOARD_STOP_SAFETY_RECEIVE__ACTUAL_RAW_IO_LOOPBACK_ONLY');
    disp(jsonencode(report));assert(report.passed);return
end
% Test the observer encoder and same-owner receiver with synthetic peer data.
rotorExpected=struct('session_token',uint64(73),'dll_sha256',uint8((1:32).'), ...
    'maximum_source_age_s',.1,'maximum_receive_age_s',.1);
rotorBytes1=m600check.encodeCanonicalRotorObserver((1:6).',.01,uint64(1),rotorExpected.session_token,rotorExpected.dll_sha256);
rotorBytes2=m600check.encodeCanonicalRotorObserver((2:7).',.02,uint64(2),rotorExpected.session_token,rotorExpected.dll_sha256);
write(peer,rotorBytes1,'uint8','127.0.0.1',cfg.canonical_rotor_observer.local_port);
write(peer,rotorBytes2,'uint8','127.0.0.1',cfg.canonical_rotor_observer.local_port);
pause(.02);rotorIngress=io.takeCanonicalRotorRecords();
check('same_io_actual_rotor_datagrams_exact',numel(rotorIngress.records)==2 ...
    &&isequal(rotorIngress.records{1}.bytes(:),rotorBytes1)&&isequal(rotorIngress.records{2}.bytes(:),rotorBytes2));
check('rotor_original_clock_and_local_endpoint',isa(rotorIngress.records{1}.original_host_receive_ns,'uint64') ...
    &&rotorIngress.records{1}.original_host_receive_ns<=rotorIngress.records{2}.original_host_receive_ns ...
    &&rotorIngress.records{2}.original_host_receive_ns<=rotorIngress.original_poll_ns ...
    &&strcmp(rotorIngress.records{1}.sender_address,'127.0.0.1'));
check('rotor_packet_does_not_invent_independent_model_clock', ...
    isnan(rotorIngress.independent_model_clock.last_source_time_s) ...
    &&~rotorIngress.independent_model_clock.model_ready ...
    &&~rotorIngress.rotor_decode_or_origin_attestation_performed&&~rotorIngress.publication_authority);
rotorEmpty=io.takeCanonicalRotorRecords();
check('rotor_drain_once_raw_original_retained',isempty(rotorEmpty.records) ...
    &&rotorEmpty.received_count==uint64(2)&&numel(io.evidence().raw_rotor_datagrams)==2);
now=gpenmpcNative.rflyOriginalHostMonotonicNs();s=gpenmpcNative.RflySnapshotDecoder(source,now);
f=gpenmpcNative.RflyCommittedFeedbackDecoder(feedback,now);
expected=struct('source_system',s.source_system,'source_component',s.source_component, ...
    'target_system',uint8(255),'target_component',uint8(190),'uid',s.observed_uid, ...
    'session_generation',s.observed_boot_generation,'link_lifecycle_generation',uint64(7), ...
    'confirmed_host_rx_ns',now,'execution_session_sha256',repmat('C',1,64), ...
    'configuration_sha256',hex(s.configuration_sha256));
snapshotFrames=pack(source,3,uint64(s.subscription_generation),boardSerializer,255,190);
feedbackFrames=pack(feedback,5,f.token.output_generation,boardSerializer,255,190);
% Before-bind callback is retained but cannot grant a completed source.
write(peer,snapshotFrames{1},'uint8','127.0.0.1',cfg.local_mavlink_port);waitCount(1);
check('unbound_original_message_retained_not_admitted',io.pollCanonical().unbound_messages_retained==uint64(1));
expected.confirmed_host_rx_ns=gpenmpcNative.rflyOriginalHostMonotonicNs();bound=io.bindCanonicalSession(expected);
check('same_owner_fixed_two_channels',bound.same_existing_mavlinkio&&bound.additional_connections==0 ...
    &&bound.status.maximum_completed_slots==2);
sessionSent=io.sendCanonicalSession('status',struct());
sessionWire=receivePeer();sessionMessage=deserializemsg(d,sessionWire);
check('session_exact_encoder_same_owner_send',sessionMessage.MsgID==126 ...
    &&isequal(reshape(sessionMessage.Payload.data(1:double(sessionMessage.Payload.count)),[],1),sessionSent.encoding.original_command_bytes) ...
    &&sessionSent.messages_attempted==1&&sessionSent.messages_send_returned==1 ...
    &&~sessionSent.board_acknowledged&&isa(sessionSent.original_host_submit_ns,'uint64'));
for k=1:10
    if k<=3,write(peer,snapshotFrames{k},'uint8','127.0.0.1',cfg.local_mavlink_port);end
    write(peer,feedbackFrames{k},'uint8','127.0.0.1',cfg.local_mavlink_port);
end
waitCount(14);r=io.pollCanonical();check('actual_callback_completed_both',all(r.status.ready));
rs=io.takeCanonical('snapshot');rf=io.takeCanonical('feedback');
check('callback_body_exact_actual_cpp_fixture',isequal(rs.message,source)&&isequal(rf.message,feedback));
check('original_uint64_callback_first_retained',isa(rs.original_host_receive_ns,'uint64') ...
    &&rs.original_host_receive_ns==rs.fragment_rx_ns(1)&&rf.original_host_receive_ns==rf.fragment_rx_ns(1) ...
    &&rs.completion_host_receive_ns>=rs.original_host_receive_ns);
raw=io.evidence();tunnelRows=raw.raw_mavlink(cellfun(@(v)strcmp(v.topic,'TUNNEL'),raw.raw_mavlink));
check('first_timestamp_matches_original_callback_record',rs.original_host_receive_ns==tunnelRows{2}.original_host_receive_ns);
check('no_raw_frame_or_clock_mapping_claim',~tunnelRows{2}.raw_frame_available&&~rs.freshness_renewed ...
    &&~rf.board_consumption_proven&&~rs.transport_source_authenticated);
check('take_once',isempty(io.takeCanonical('snapshot'))&&isempty(io.takeCanonical('feedback')));
serial=createmsg(d,'SERIAL_CONTROL');serial.Payload.device=uint8(10);serial.Payload.count=uint8(4);
serial.Payload.flags=uint8(1);serial.Payload.data(1:4)=uint8('TEST');
write(peer,uint8(serializemsg(boardSerializer,serial)),'uint8','127.0.0.1',cfg.local_mavlink_port);waitSerial();
sessionRecords=io.canonicalSessionReceipts();record=sessionRecords{1};
check('serial_original_decoded_callback_contract',isequal(record.decoded_message.Payload.data(1:4),uint8('TEST')) ...
    &&record.decoded_message.SystemID==1&&record.decoded_message.ComponentID==1 ...
    &&isa(record.original_host_receive_ns,'uint64')&&~record.raw_frame_available ...
    &&strcmp(record.decoded_source,'ORIGINAL_MAVLINKIO_SERIAL_CONTROL_CALLBACK'));
contextFrames=pack(context,4,be64(context(5:12)),inputSerializer,1,1);
numericFrames=pack(numeric,2,be64(numeric(1:8)),inputSerializer,1,1);
packets=[contextFrames;numericFrames];
[decoded,decReceipt]=m600check.decodeCanonicalExchangePackets(packets,d,expected);
check('actual_official_decoder_seven_input_frames',numel(decoded)==7&&decReceipt.official_decoder_crc_accepted);
broken=packets;broken{2}(end)=bitxor(broken{2}(end),uint8(1));
reject(@()m600check.decodeCanonicalExchangePackets(broken,d,expected),'m600check:CanonicalPacketDecode','crc_bad_before_any_send');
broken=packets;broken{1}=broken{1}(1:end-1);
reject(@()m600check.decodeCanonicalExchangePackets(broken,d,expected),'m600check:CanonicalPacketFrame','partial_frame_refused');
wrong=expected;wrong.source_system=uint8(9);
reject(@()m600check.decodeCanonicalExchangePackets(packets,d,wrong),'m600check:CanonicalPacketEndpoints','wrong_registered_target_refused');
reject(@()m600check.decodeCanonicalExchangePackets([packets;packets],d,expected),'m600check:CanonicalPacketCount','bounded_send_group');
% Force input serialization and actual-owner send sequence to differ. No
% production serializer is exposed or used for side-channel transmission.
for k=1:20,serializemsg(inputSerializer,serial);end
packets=[pack(context,4,be64(context(5:12)),inputSerializer,1,1); ...
    pack(numeric,2,be64(numeric(1:8)),inputSerializer,1,1)];
[decoded,~]=m600check.decodeCanonicalExchangePackets(packets,d,expected);
sent=io.sendCanonicalPackets(packets);start=tic;wire=cell(7,1);
for k=1:7
    while peer.NumDatagramsAvailable==0&&toc(start)<3,pause(0.005);end
    assert(peer.NumDatagramsAvailable>0,'m600check:LocalhostSendTimeout');
    row=read(peer,1,'uint8');wire{k}=uint8(row.Data);
end
actual=cell(7,1);samePayload=true;
for k=1:7,actual{k}=deserializemsg(d,wire{k});samePayload=samePayload&&isequal(actual{k}.Payload,decoded{k}.Payload);end
check('same_existing_link_payload_unchanged',samePayload&&sent.messages_submitted==7&&sent.same_existing_mavlinkio);
check('final_sequence_owned_by_link_not_input',actual{1}.Seq~=decoded{1}.Seq ...
    &&sent.final_sequence_generated_by_existing_link&&~sent.actual_wire_bytes_available);
check('send_return_not_board_receipt',~sent.board_receipt_proven&&~sent.control_authority ...
    &&all(sent.original_host_send_return_ns>=sent.original_host_submit_ns));
% Real callback receives a second complete source but may not overwrite it.
source2=readbin(fullfile(base,'SOURCE_2.bin'));source3=readbin(fullfile(base,'SOURCE_3.bin'));
s2=gpenmpcNative.RflySnapshotDecoder(source2,now);s3=gpenmpcNative.RflySnapshotDecoder(source3,now);
for frame=reshape(pack(source2,3,uint64(s2.subscription_generation),boardSerializer,255,190),1,[])
    write(peer,frame{1},'uint8','127.0.0.1',cfg.local_mavlink_port);
end
third=pack(source3,3,uint64(s3.subscription_generation),boardSerializer,255,190);
write(peer,third{1},'uint8','127.0.0.1',cfg.local_mavlink_port);waitCount(18);
raw=io.evidence();check('real_callback_overflow_first_fault',raw.canonical_exchange.status.failed ...
    &&contains(raw.canonical_exchange.status.failure,'TunnelOverflow') ...
    &&isequal(raw.canonical_exchange.ready{1}.message,source2));
reject(@()io.bindCanonicalSession(expected),'m600check:CanonicalExchangeBindOnce','no_fault_laundering_rebind');
after=io.evidence();check('original_reassembler_first_fault_unchanged', ...
    after.canonical_exchange.status.failure==raw.canonical_exchange.status.failure);
cleanupSent=io.sendCanonicalSession('stop',struct());cleanupWire=receivePeer();cleanupMessage=deserializemsg(d,cleanupWire);
check('finally_exact_stop_remains_same_owner_after_fault',cleanupSent.messages_send_returned==1 ...
    &&isequal(reshape(cleanupMessage.Payload.data(1:double(cleanupMessage.Payload.count)),[],1),uint8(sprintf('gpenmpc_rfly_session stop\n')).'));
after=io.evidence();check('cleanup_does_not_wash_reassembler_fault',after.canonical_exchange.status.failure==raw.canonical_exchange.status.failure);
for k=1:3,write(peer,rotorBytes1,'uint8','127.0.0.1',cfg.canonical_rotor_observer.local_port);end
pause(.02);rotorOverflow=io.takeCanonicalRotorRecords();
check('actual_rotor_queue_overflow_retains_first_and_fault_packet', ...
    strcmp(rotorOverflow.failure,'ROTOR_OBSERVATION_QUEUE_OVERFLOW')&&numel(rotorOverflow.records)==2 ...
    &&numel(io.evidence().raw_rotor_datagrams)==5);
rotorAgain=io.takeCanonicalRotorRecords();
check('rotor_fault_permanent_after_take',strcmp(rotorAgain.failure,rotorOverflow.failure) ...
    &&isempty(rotorAgain.records)&&rotorAgain.received_count==uint64(5));
check('owned_endpoints_close',io.close());after=io.evidence();check('closed_raw_evidence_retained',after.closed ...
    &&isequal(after.canonical_exchange.ready{1}.message,source2));
rotorPortProbe=udpport('datagram','IPV4','LocalHost','127.0.0.1','LocalPort',cfg.canonical_rotor_observer.local_port);
check('rotor_socket_released_by_same_owner_finally',~isempty(rotorPortProbe));delete(rotorPortProbe);
% Test the disabled option after releasing the first owner and its ports.
nativeCfg=rmfield(cfg,{'canonical_exchange','canonical_rotor_observer'});nativeIo=m600check.makeM600CopterSimIo(nativeCfg);
nativeCleanup=onCleanup(@()nativeIo.close()); %#ok<NASGU>
nativeEvidence=nativeIo.evidence();check('optional_off_original_topics',~nativeEvidence.canonical_exchange_enabled ...
    &&~ismember('TUNNEL',nativeEvidence.subscribed_topics)&&~ismember('SERIAL_CONTROL',nativeEvidence.subscribed_topics));
nativeIo.sendHeartbeat();nativeWire=receivePeer();nativeHeartbeat=deserializemsg(d,nativeWire);
check('old_native_heartbeat_payload_unchanged',nativeHeartbeat.MsgID==0&&nativeHeartbeat.SystemID==255 ...
    &&nativeHeartbeat.ComponentID==190&&nativeHeartbeat.Payload.type==6 ...
    &&nativeHeartbeat.Payload.autopilot==8&&nativeHeartbeat.Payload.base_mode==0);
check('optional_off_owner_closed',nativeIo.close());
check('optional_off_no_rotor_endpoint',~nativeEvidence.canonical_rotor_observer_enabled);
% Use a sequential owner to test callback bursts with the bounded COMPLETE FIFO.
queueCfg=rmfield(cfg,'canonical_rotor_observer');
queueCfg.canonical_exchange.completed_queue_capacity=2;
badQueueCfg=queueCfg;badQueueCfg.canonical_exchange.completed_queue_capacity=0;
reject(@()m600check.makeM600CopterSimIo(badQueueCfg), ...
    'm600check:CanonicalCompletedQueueConfig','completed_queue_invalid_before_socket');
io=m600check.makeM600CopterSimIo(queueCfg);queueCleanup=onCleanup(@()io.close()); %#ok<NASGU>
expected.confirmed_host_rx_ns=gpenmpcNative.rflyOriginalHostMonotonicNs();qb=io.bindCanonicalSession(expected);
check('completed_queue_explicit_combined_bound',qb.completed_queue_capacity==2 ...
    &&all(qb.completed_queue_counts==0)&&qb.same_existing_mavlinkio&&qb.additional_connections==0);
source2Frames=pack(source2,3,uint64(s2.subscription_generation),boardSerializer,255,190);
% One datagram carries both full source messages. No outer drain/poll is
% interleaved with any fragment; official mavlinkio alone invokes callbacks.
burst=[snapshotFrames;source2Frames];burst=cellfun(@(x)x(:),burst,'UniformOutput',false);
write(peer,vertcat(burst{:}),'uint8','127.0.0.1',cfg.local_mavlink_port);
waitCount(6);qstatus=io.pollCanonical();
check('actual_callback_burst_complete_fifo_two',isequal(qstatus.completed_queue_counts,[2 0]) ...
    &&isequal(qstatus.completed_enqueued,uint64([2 0]))&&~any(qstatus.status.ready));
qfirst=io.takeCanonical('snapshot');qsecond=io.takeCanonical('snapshot');
check('complete_fifo_exact_order_no_overwrite',isequal(qfirst.message,source)&&isequal(qsecond.message,source2) ...
    &&qfirst.generation<qsecond.generation&&isempty(io.takeCanonical('snapshot')));
qraw=io.evidence();qrows=qraw.raw_mavlink(cellfun(@(x)strcmp(x.topic,'TUNNEL'),qraw.raw_mavlink));
check('complete_fifo_original_callback_anchors_not_poll',qfirst.original_host_receive_ns==qrows{1}.original_host_receive_ns ...
    &&qsecond.original_host_receive_ns==qrows{4}.original_host_receive_ns ...
    &&qfirst.original_host_receive_ns==qfirst.fragment_rx_ns(1) ...
    &&qsecond.original_host_receive_ns==qsecond.fragment_rx_ns(1) ...
    &&~qfirst.freshness_renewed&&~qsecond.freshness_renewed);
qstatus=io.pollCanonical();check('complete_fifo_drain_counters',all(qstatus.completed_queue_counts==0) ...
    &&isequal(qstatus.completed_taken,uint64([2 0])));
check('complete_fifo_success_owner_closed_before_next_fixture',io.close());
% In a new fixture, fill two snapshot slots and complete RFC1.
% Keep the third COMPLETE in its assembler without overwriting either slot.
io=m600check.makeM600CopterSimIo(queueCfg);overflowCleanup=onCleanup(@()io.close()); %#ok<NASGU>
expected.confirmed_host_rx_ns=gpenmpcNative.rflyOriginalHostMonotonicNs();io.bindCanonicalSession(expected);
burst=[snapshotFrames;source2Frames;feedbackFrames];
burst=cellfun(@(x)x(:),burst,'UniformOutput',false);
write(peer,vertcat(burst{:}),'uint8','127.0.0.1',cfg.local_mavlink_port);waitCount(16);
queueFailed=io.evidence();
check('complete_fifo_combined_overflow_latched',contains(queueFailed.canonical_exchange_failure, ...
    'CanonicalCompletedQueueOverflow')&&isequal(cellfun(@numel,queueFailed.canonical_completed_queues),[2 0]));
check('complete_fifo_failed_packet_and_preceding_retained', ...
    isequal(queueFailed.canonical_completed_queues{1}{1}.message,source) ...
    &&isequal(queueFailed.canonical_completed_queues{1}{2}.message,source2) ...
    &&isequal(queueFailed.canonical_exchange.ready{2}.message,feedback));
reject(@()io.takeCanonical('snapshot'),'m600check:CanonicalExchangeClosed','complete_fifo_no_live_take_after_overflow');
queueCleanupSent=io.sendCanonicalSession('stop',struct());queueCleanupWire=receivePeer();
queueCleanupDecoded=deserializemsg(d,queueCleanupWire);
check('complete_fifo_fault_still_allows_exact_same_owner_stop',queueCleanupSent.messages_send_returned==1 ...
    &&queueCleanupDecoded.MsgID==126&&contains(io.evidence().canonical_exchange_failure,'CanonicalCompletedQueueOverflow'));
check('complete_fifo_owner_closed',io.close());queueAfter=io.evidence();
check('complete_fifo_all_evidence_survives_close',queueAfter.closed ...
    &&isequal(queueAfter.canonical_completed_queues,queueFailed.canonical_completed_queues) ...
    &&isequal(queueAfter.canonical_exchange.ready{2}.message,feedback));
report=struct('passed',all([checks.pass]),'checks',checks,'negative_cases',{failures}, ...
    'scope','ACTUAL_MATLAB_MAVLINKIO_AND_M600_ADAPTER_LOOPBACK_ONLY_MOCK_PREFLIGHT_AND_SESSION', ...
    'maximum_simultaneous_production_mavlink_owners',1,'production_adapter_constructions',4, ...
    'extra_mavlink_or_com_endpoints',0,'optional_same_owner_rotor_receivers',1,'test_loopback_peer_endpoints',1, ...
    'board_actions',0,'simulator_actions',0,'com_opens',0,'actual_registration_proven',false, ...
    'callback_original_time_scope','FIRST_MATLAB_CALLBACK_STOPWATCH_NS', ...
    'assembly_limit_ns_fixture_only',cfg.canonical_exchange.assembly_limit_ns,'source_directory',char(base), ...
    'source_paths',{sourcePaths},'source_sha256',{sourceHashes}, ...
    'sources_unchanged',isequal(sourceHashes,cellfun(@fileSha,sourcePaths,'UniformOutput',false)));
report.passed=report.passed&&report.sources_unchanged;
save(fullfile(outputRoot,'RAW.mat'),'report','raw','after','sessionRecords','rs','rf','wire','sent','packets','sessionSent','sessionWire','cleanupSent','cleanupWire','nativeEvidence','nativeWire', ...
    'rotorIngress','rotorBytes1','rotorBytes2','rotorEmpty','rotorOverflow','rotorAgain', ...
    'qfirst','qsecond','qraw','qstatus','queueFailed','queueAfter','queueCleanupWire');
fid=fopen(fullfile(outputRoot,'RESULT.json'),'w');assert(fid>0);cleanup=onCleanup(@()fclose(fid));
fprintf(fid,'%s\n',jsonencode(report,PrettyPrint=true));clear cleanup
disp(jsonencode(struct('passed',report.passed,'checks',numel(checks),'board_actions',0)));
assert(report.passed,'m600check:CanonicalLocalhostTest','One or more actual localhost adapter checks failed.');
    function check(name,value),checks(end+1)=struct('name',name,'pass',logical(value));end
    function reject(fn,id,name)
        caught='';try,fn();catch ex,caught=ex.identifier;end
        failures{end+1}=struct('name',name,'identifier',caught);check(name,strcmp(caught,id));
    end
    function waitCount(n)
        t=tic;
        while toc(t)<3
            data=io.evidence();count=sum(cellfun(@(x)strcmp(x.topic,'TUNNEL'),data.raw_mavlink));
            if count>=n,return;end;pause(0.005);
        end
        error('m600check:LocalhostReceiveTimeout','Only %d/%d TUNNEL callbacks; fatal=%s',count,n,data.fatal);
    end
    function waitSerial()
        t=tic;while toc(t)<3,if ~isempty(io.canonicalSessionReceipts()),return;end;pause(0.005);end
        error('m600check:LocalhostSerialTimeout');
    end
    function frames=pack(body,version,generation,serializer,targetSystem,targetComponent)
        count=ceil(numel(body)/119);frames=cell(count,1);
        for index=0:count-1
            m=createmsg(d,'TUNNEL');part=[uint8(version*16+index);be(generation);body(index*119+1:min(index*119+119,numel(body)))];
            m.Payload.payload_type=uint16(42002);m.Payload.target_system=uint8(targetSystem);m.Payload.target_component=uint8(targetComponent);
            m.Payload.payload_length=uint8(numel(part));m.Payload.payload(:)=0;m.Payload.payload(1:numel(part))=part;
            frames{index+1}=uint8(serializemsg(serializer,m));
        end
    end
    function wire=receivePeer()
        t=tic;while peer.NumDatagramsAvailable==0&&toc(t)<3,pause(0.005);end
        assert(peer.NumDatagramsAvailable>0,'m600check:LocalhostSessionTimeout','Same-owner session send did not reach test peer.');
        row=read(peer,1,'uint8');wire=uint8(row.Data);
    end
end
function b=readbin(p),f=fopen(p,'rb');assert(f>0);c=onCleanup(@()fclose(f));b=fread(f,inf,'*uint8');end
function h=hex(b),h=upper(reshape(dec2hex(b,2).',1,[]));end
function b=be(v),[~,~,e]=computer;if e=='L',v=swapbytes(v);end;b=reshape(typecast(v,'uint8'),[],1);end
function v=be64(b),v=uint64(0);for k=1:numel(b),v=bitor(bitshift(v,8),uint64(b(k)));end,end
function deleteSerializers(a,b),delete(a);delete(b);end
function value=fileSha(path)
md=java.security.MessageDigest.getInstance('SHA-256');md.update(typecast(readbin(path),'int8'));
value=hex(typecast(md.digest(),'uint8'));
end
