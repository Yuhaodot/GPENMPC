function report=test_local_method_preparation(outputRoot,exerciseFlight,label,streamLifecycleEnabled,compiledGp,compiledInput,rawTransportSource,runtimeStateOnly)
% Test the same-IO MATLAB loop and eNMPC worker with a host peer
% and retained NoUI getter records.
if nargin<2,exerciseFlight=false;end
if nargin<3,label='';end
if nargin<4,streamLifecycleEnabled=false;end
if nargin<5,compiledGp=false;end
if nargin<6,compiledInput=false;end
if nargin<7,rawTransportSource=[];end
if nargin<8,runtimeStateOnly=false;end
assert(islogical(runtimeStateOnly)&&isscalar(runtimeStateOnly));
useRawTransport=~isempty(rawTransportSource);
assert(~useRawTransport||(isstruct(rawTransportSource)&&isscalar(rawTransportSource)));
assert(islogical(streamLifecycleEnabled)&&isscalar(streamLifecycleEnabled));
assert(islogical(compiledGp)&&isscalar(compiledGp));
assert(islogical(compiledInput)&&isscalar(compiledInput));
assert(ischar(label)&&~isempty(regexp(['X' label],'^X[A-Z0-9_]*$','once')));
assert(isscalar(exerciseFlight)&&exerciseFlight>=0&&exerciseFlight<=5&&exerciseFlight==fix(exerciseFlight));
suffix='';if ~isempty(label),suffix=['_' label];end
resultPath=fullfile(outputRoot,['RESULT' suffix '.json']);
build=string(fileparts(fileparts(mfilename('fullpath'))));
if strcmp(label,'GP_INPUT_CONTINUATION')
 report=checkGpInputContinuation(build,outputRoot);return
end
if strcmp(label,'GP_ASYNC_READY')
 report=checkGpAsyncContinuation(build,outputRoot);return
end
addpath(fullfile(build,'host_runtime'),fullfile(build,'matlab_validation'),fullfile(build,'m600_coptersim','matlab_validation'));
addpath(gpenmpc_external_path('native_visual_host_source'),'-end');
assert(~isfile(resultPath));if ~isfolder(outputRoot),mkdir(outputRoot);end
diary(fullfile(outputRoot,'MATLAB_DIARY.txt'));io=[];peer=[];serializer=[];service=[];
completeAndPartialProbe=false;partialProbeReceiveCalls=0;partialProbeReceivesBeforeSend=0;
completedGpProbe=false;completedGpInputSent=false;completedGpExtraReceives=0;
firstUnsentProbe=runtimeStateOnly&&strcmp(label,'FIRST_UNSENT');rejectFirstInput=false;
armWaitProbe=runtimeStateOnly&&~ismember(label,{'AFTER_ARM','COMPLETED_GP'});
stream=[];streamEnable=[];streamBeforeCleanup=[];streamAfterCleanup=[];
streamCleanup=struct('ran',false,'disable_error','','evidence_errors',{{}},'resource_errors',{{}}, ...
 'original_disable_transmit_rows',{{}},'io_failure_before','','io_failure_after','','io_closed',false);
closedEvidence=[];streamGuard=[];if streamLifecycleEnabled,streamGuard=onCleanup(@cleanup);end %#ok<NASGU>
checks=struct('name',{},'pass',{});events={};
try
a=gpenmpcNative.loadCanonicalAssets();
task=fullfile(build,'task_packages','cambridge_canonical','MU_CAMBRIDGE_MA_02__CANONICAL_PHYSICAL_TASK.mat');
taskSha='B876546B468265F84BD028AC549B06EF8ADDCB25CBA09CF389A08F4B21290A5F';
bundle=gpenmpcNative.loadRflyCanonicalDeliveryTask(task,taskSha);
[trajectory,refbind]=gpenmpcNative.bindRflyCanonicalInitialTakeoffTrajectory(bundle.legs{1},zeros(3,1));
policy=struct('expected_session_token',26090501,'payload_by_generation_kg',[2.21;1.75;.98;.55;0], ...
    'mass_by_generation_kg',[11.71;11.25;10.48;10.05;9.5]);
ts=struct('path',task,'sha256',taskSha,'configuration_sha256',char(a.binding.effective_configuration_payload_sha256), ...
    'environment_policy',policy,'copter_id',1);ts.cached_asset=gpenmpcNative.RflyLocalTaskAsset(ts);
environment=gpenmpcNative.RflyLocalEnvironmentLedger(policy,1,256);
getter=gpenmpcNative.RflyLocalOriginalGetterBuffer(4096,'990850A2F40F3FCC2A6C47E63A4065B60FF49AA39CC4749FF443963B06F2EF7E');
actual=load(fullfile(gpenmpc_external_path('matlab_noui_original_reader'),'MATLAB_ORIGINAL_GETTERS.mat'));
detail=jsondecode(fileread(fullfile(gpenmpc_external_path('matlab_noui_original_reader'),'OFFLINE_SENSOR_CONTENT_MATCH.json')));
for first=1:256:size(actual.records,2),getter.ingest(batch(first:min(first+255,size(actual.records,2))));end
empty=batch([]);x=load(fullfile(gpenmpc_external_path('local_original_task_cache'),'RAW.mat'),'receipt');x=x.receipt;
snapshotFixture=fullfile(gpenmpc_external_path('local_snapshot_wire_fixture'),'RLS1_SNAPSHOTS.bin');
assert(isfile(snapshotFixture),'test:MissingSnapshotFixture', ...
 'Configure local_snapshot_wire_fixture with the retained RLS1_SNAPSHOTS.bin test input.');
raw=reshape(readbin(snapshotFixture),382,[]);
% Test the missing-ACK lease with an empty environment ledger before
% starting callbacks; repeated model_time values can match earlier ACKs.
delayedEnvironmentLease=checkDelayedEnvironmentLease();
flightFixture=struct();
if exerciseFlight
 fr=fullfile(gpenmpc_external_path('committed_state_diagnostics'));
 flightFixture.gp=reshape(readbin(fullfile(build,'rfly_vendor_integration','full_inner_abi', ...
  'snapshot_wire_fixture','RGP1_RGR1_PAIRS.bin')),596,[]);
 flightFixture.commits=reshape(readbin(fullfile(fr,'COMMITTED_RLC2.bin')),1494,[]);
 flightFixture.metadata_rebound=true;
 flightFixture.scope='SYNTHETIC_SESSION_SOURCE_KEYS_WITH_RECORDED_C_NUMERICS';
end
echo=readbin(fullfile(gpenmpc_external_path('board_commit_exchange_fixture'),'ACTUAL_FIXTURE_ECHO119.bin'));
cfg=struct('local_mavlink_port',62281,'remote_mavlink_port',62282,'truth_port',62283,'coptersim_time_port',62284, ...
 'target_system',1,'target_component',1,'clock_max_rtt_s',1,'clock_sync_samples',2,'clock_sync_period_s',1,'clock_max_age_s',5, ...
 'clock_max_uncertainty_s',1,'clock_max_utc_drift_s',1,'clock_max_time_heartbeat_age_s',5,'clock_max_time_heartbeat_lag_s',5, ...
 'maximum_truth_lag_s',5,'maximum_raw_records',10000,'state_max_age_s',1,'live_enabled',true,'outer_preflight_pass',true, ...
 'canonical_exchange',struct('runtime','BOARD_LOCAL_FULL_INNER','assembly_limit_ns',uint64(5000000000), ...
 'gp_reply_host_max_age_ns',uint64(5000000000),'local_input_host_max_age_ns',uint64(5000000000), ...
 'completed_queue_capacity',8,'local_window_host_max_age_ns',uint64(1000000000)));
cfg.delivery_environment_contract=policy;cfg.delivery_environment_contract.initial_payload_kg=2.21;
cfg.delivery_environment_contract.remote_port=cfg.remote_mavlink_port;
cfg.local_short=struct('environment_ledger_capacity',256,'runtime_state_only',runtimeStateOnly);
if useRawTransport
 cfg.mavlink_transport=struct('source',rawTransportSource,'scope','HOST_ONLY_LOOPBACK','allow_loopback',true, ...
  'local_host','127.0.0.1','remote_host','127.0.0.1','local_port',cfg.local_mavlink_port,'remote_port',cfg.remote_mavlink_port, ...
  'local_system',uint8(255),'local_component',uint8(190),'remote_system',uint8(1),'remote_component',uint8(1), ...
  'maximum_poll_datagrams',64);
end
d=mavlinkdialect(fullfile(build,'m600_coptersim','matlab_validation','+m600check','px4_health_events.xml'),2);
serializer=mavlinkio(d,'SystemID',1,'ComponentID',1);
peer=udpport('datagram','IPV4','LocalHost','127.0.0.1','LocalPort',cfg.remote_mavlink_port,'Timeout',.2);
io=m600check.makeM600CopterSimIo(cfg);
originalPollCanonical=io.pollCanonical;io.pollCanonical=@pollWithPartialObservation;
originalInputSend=io.sendCanonicalLocalInput;io.sendCanonicalLocalInput=@sendWithFirstUnsentProbe;
request=struct('original_host_challenge',u64(echo(6:21)),'uid',u64(echo(22:29)), ...
 'system',echo(30),'component',echo(31),'host_system',uint8(255),'host_component',uint8(190), ...
 'configuration_payload_sha256',hex(echo(56:87)),'local_full_inner',true,'leg_index',uint8(1),'task_sha256',taskSha);
assert(strcmpi(request.configuration_payload_sha256,ts.configuration_sha256),'test:FixtureConfiguration', ...
 'board_commit_exchange_fixture configuration SHA-256 %s does not match the current canonical configuration SHA-256 %s.', ...
 request.configuration_payload_sha256,ts.configuration_sha256);
tx=io.sendCanonicalSession('prepare_local',struct('challenge',request.original_host_challenge, ...
 'origin_ned_m',[0;0;0],'host_system',uint8(255),'host_component',uint8(190),'leg_index',uint8(1)));drain();
request.original_prepare_submit_ns=tx.original_host_submit_ns(1);
line=sprintf(['RFLY_LOCAL_SESSION state=2 challenge=%s uid=%u system=%u component=%u registration_hrt_us=%u ' ...
 'session_generation=%u link_generation=%u semantics=1 config_sha=%s session_sha=%s task_sha=%s leg=1 ' ...
 'registered=1 echo_confirmed=0 declared_isolation=0 declaration_is_sensor_proof=0 session_fault=0 start_requests=0 stop_requests=0\n'], ...
 hex(echo(6:21)),request.uid,request.system,request.component,u64(echo(32:39)),u64(echo(40:47)),u64(echo(48:55)), ...
 request.configuration_payload_sha256,hex(echo(88:119)),taskSha);
pre=sendText(line);prepared=gpenmpcNative.RflySessionAssociationDecoder(pre,[],request,[],d);
auth=struct('source','OperatorUsbIsolationDeclaration', ...
 'physical_setup_record_sha256',repmat('A',1,64),'original_record','EXPLICIT_HOST_PEER_FIXTURE_ONLY_NOT_LIVE_AUTHORIZATION');
tx=io.sendCanonicalSession('confirm',struct('prepared_receipt',prepared,'physical_setup_record_sha256',auth.physical_setup_record_sha256));drain();
request.original_confirm_submit_ns=tx.original_host_submit_ns(1);
request.original_confirm_session_sha256=prepared.execution_session_sha256;request.confirmed_physical_setup_record_sha256=auth.physical_setup_record_sha256;
confirmed=strrep(strrep(strrep(line,'state=2','state=3'),'echo_confirmed=0','echo_confirmed=1'),'declared_isolation=0','declared_isolation=1');
con=sendText(confirmed);association=gpenmpcNative.RflySessionAssociationDecoder(pre,con,request,auth,d);e=association.echo;
expected=struct('source_system',e.system,'source_component',e.component,'target_system',uint8(255),'target_component',uint8(190), ...
 'uid',e.uid,'session_generation',e.process_session_generation,'link_lifecycle_generation',e.link_lifecycle_generation, ...
 'confirmed_host_rx_ns',association.original_host_receive_ns,'execution_session_sha256',association.execution_session_sha256, ...
 'configuration_sha256',e.configuration_payload_sha256,'local_full_inner',true,'leg_index',uint8(1),'task_sha256',taskSha);
identity=struct('uid',e.uid,'boot_generation',e.process_session_generation,'system',e.system,'component',e.component);
% Bind the reference asset to the task identity.
expected.reference_asset_sha256=taskSha;
registered=struct('local_full_inner',true,'identity',identity,'leg_index',uint8(1),'task_sha256',taskSha, ...
 'execution_session_sha256',association.execution_session_sha256,'configuration_sha256',e.configuration_payload_sha256, ...
 'reference_asset_sha256',expected.reference_asset_sha256);
io.bindCanonicalSession(expected,association);ev=io.evidence();check('actual_same_IO_registration_preserved',isequaln(ev.canonical_exchange_association,association));
% Host-fixture observation bounds.
serviceCfg=struct('source_max_age_ns',uint64(1000000000),'command_lifetime_ns',uint64(400000000),'pending_capacity',8);
if compiledGp
 gpDirectory=fullfile(gpenmpc_external_path('canonical_gp_wire_mex'));
 addpath(gpDirectory,'-begin');
 serviceCfg.gp_backend=struct('kind','CANONICAL_GP_WIRE_MEX', ...
  'path',char(fullfile(gpDirectory,'canonical_gp_wire_mex.mexw64')), ...
  'binary_sha256','21017CD36C857466AE538EAE716C868173D2C54DF4D2B5CC458E3C6681CEBE6B');
end
if compiledInput
 inputDirectory=fullfile(build,'evidence','task_interface','task_wire');
 addpath(fullfile(build,'tools'),inputDirectory,'-begin');
 serviceCfg.input_codec_backend=struct('kind','CANONICAL_LOCAL_TASK_WIRE_MEX', ...
  'path',char(fullfile(inputDirectory,'canonical_local_task_wire_mex.mexw64')), ...
  'binary_sha256','150D341A3E0C5EB3F382FF494CBB69D07BC27231767CB48055052CA9AEA940EB', ...
  'adapter_path',char(fullfile(build,'tools','encode_canonical_local_task_mex.m')), ...
  'adapter_sha256','C2ACAEF7AFA34C6C2497928B562317A4C7F6F6D6194689AA14056BCEA35BD496');
 % Test construction failures before stream enable or worker startup.
 beforeBackendNegatives=io.evidence();badCfg=serviceCfg;
 badCfg.input_codec_backend.binary_sha256=repmat('0',1,64);
 rejectExact('input_codec_wrong_binary_binding_rejected_before_worker', ...
  @()gpenmpcNative.RflyLocalMethodService(a,bundle,trajectory,refbind,registered,association,io,getter,environment,ts,badCfg), ...
  'gpenmpcNative:LocalInputCodecBackendBinding');
 badCfg=serviceCfg;badCfg.input_codec_backend.adapter_path=badCfg.input_codec_backend.path;
 rejectExact('input_codec_wrong_existing_adapter_path_rejected_before_worker', ...
  @()gpenmpcNative.RflyLocalMethodService(a,bundle,trajectory,refbind,registered,association,io,getter,environment,ts,badCfg), ...
  'gpenmpcNative:LocalInputCodecBackendPath');
 afterBackendNegatives=io.evidence();check('input_codec_rejected_construction_sends_nothing', ...
  numel(afterBackendNegatives.raw_transmit_messages)==numel(beforeBackendNegatives.raw_transmit_messages));
end
bad=association;bad.physical_setup_record_sha256=repmat('F',1,64);
reject('copied_unverified_registration_cannot_create_owner',@()gpenmpcNative.RflyLocalMethodService(a,bundle,trajectory,refbind,registered,bad,io,getter,environment,ts,serviceCfg));
if compiledInput
 serviceCfg.prepared_input_codec_backend=gpenmpcNative.RflyLocalMethodService.prepareInputCodecBackend(serviceCfg.input_codec_backend);
end
if compiledGp
 serviceCfg.prepared_gp_backend=gpenmpcNative.RflyLocalGpService.prepareBackend(a,serviceCfg.gp_backend);
end
if streamLifecycleEnabled
 beforeStream=io.evidence();stream=gpenmpcNative.RflyLocalStreamLifecycle(io,registered,association);
 afterStream=io.evidence();check('stream_lifecycle_construction_has_no_command', ...
  numel(afterStream.raw_transmit_messages)==numel(beforeStream.raw_transmit_messages));
 % Install the disable guard before enabling the stream or constructing the service.
 streamEnable=stream.enable();drain();serviceCfg.stream_lifecycle=stream;
 beforeService=io.evidence();
end
serviceCfg.runtime_state_only=runtimeStateOnly;
service=gpenmpcNative.RflyLocalMethodService(a,bundle,trajectory,refbind,registered,association,io,getter,environment,ts,serviceCfg);
backendState=service.status();
if compiledGp
 check('method_owner_forwards_exact_gp_backend_without_constructor_prediction', ...
  strcmp(backendState.gp_backend,'CANONICAL_GP_WIRE_MEX')&&backendState.gp_calls==0 ...
  &&strcmpi(backendState.gp_backend_binding.binary_sha256,serviceCfg.gp_backend.binary_sha256));
else
 check('default_method_owner_keeps_original_matlab_gp',strcmp(backendState.gp_backend,'MATLAB_ORIGINAL'));
end
if compiledInput
 inputBackendState=backendState.input_codec_backend_binding;
 check('method_owner_binds_exact_input_codec_and_adapter_without_constructor_encoding', ...
  strcmp(backendState.input_codec_backend,'CANONICAL_LOCAL_TASK_WIRE_MEX')&&backendState.input_codec_calls==0 ...
  &&strcmpi(inputBackendState.binary_sha256,serviceCfg.input_codec_backend.binary_sha256) ...
  &&strcmpi(inputBackendState.adapter_sha256,serviceCfg.input_codec_backend.adapter_sha256) ...
  &&strcmpi(inputBackendState.path,string(serviceCfg.input_codec_backend.path)) ...
  &&strcmpi(inputBackendState.adapter_path,string(serviceCfg.input_codec_backend.adapter_path)) ...
  &&inputBackendState.binary_hash_checks==1&&inputBackendState.adapter_hash_checks==1&&inputBackendState.construction_load_probe ...
  &&inputBackendState.construction_codec_calls==0&&inputBackendState.construction_numerical_calls==0&&inputBackendState.runtime_file_checks==0 ...
  &&~inputBackendState.fallback_allowed&&inputBackendState.original_source_checks_retained&&inputBackendState.io_callback_and_deadline_checks_retained);
end
if streamLifecycleEnabled
 afterService=io.evidence();check('method_construction_does_not_enable_again', ...
  numel(afterService.raw_transmit_messages)==numel(beforeService.raw_transmit_messages)&&stream.EnableAttempts==1);
end
k=0;
for candidate=1:6
 k=candidate;sendInput(k,true);pollSource('PREPARE',k);s=service.status();
 if s.outer.preparation_calls==1,break;end
end
s=service.status();check('one_original_cold_preparation_no_control',s.outer.preparation_calls==1&&s.outer.outer_submissions==0 ...
 &&s.counts.inputs_sent==0&&s.counts.commits==0);
t=tic;partialWindowProbe=false;pendingFrames={};pendingBefore=0;pendingChecked=false;
while toc(t)<60
 events{end+1}=service.poll(empty,'PREPARE');drain();s=service.status();
 if ~isempty(pendingFrames)&&strcmp(events{end}.status,'RECEIVE_BATCH_CONTINUES')
  check('partial_newer_source_does_not_starve_reference_sender', ...
   s.last_window_tx.send_returned_count>pendingBefore&&isempty(events{end}.source));
  sendFrames(pendingFrames);pendingFrames={};pendingChecked=true;
 end
 if useRawTransport&&runtimeStateOnly&&~partialWindowProbe&&s.outer.prepared&&~s.last_window_tx.send_complete
  k=k+1;body=raw(:,k);body(5:12)=be(identity.uid);body(13:20)=be(identity.boot_generation);
  body(21)=identity.system;body(22)=identity.component;
  body(277:328)=uint8(sscanf(detail.mutually_unique_retained_pairs(k).equal_sensor52_bytes_hex,'%2x'));
  body(351:382)=digest(body(1:350));parts=tunnelFrames(body,10,u64(body(47:54)));
  sendDiagnostic(k);sendFrames(parts(1));pendingFrames=parts(2:end);
  pendingBefore=s.last_window_tx.send_returned_count;partialWindowProbe=true;
 end
 if s.outer.prepared&&s.last_window_tx.send_complete,break;end;pause(.002);
end
if useRawTransport&&runtimeStateOnly,check('actual_raw_partial_window_probe_exercised',partialWindowProbe&&pendingChecked);end
check('prepared_same_worker_and244_window_no_board_install_claim',s.outer.prepared&&s.last_window_tx.send_complete ...
 &&s.last_window_tx.send_returned_count==244&&s.last_board_committed_window==0&&~s.last_window_tx.board_receipt_proven);
% Send an ACK for the new source. Retained pairs 2 and 3 share model_time=.02,
% so an earlier ACK can legitimately match both.
k=k+1;sendInput(k,true);pollSource('PREPARE',k);
check('post_window_complete_ENV_source_retired_without_control',events{end}.source_retired_without_send ...
 &&events{end}.input_binding.retirement_lease.complete&&service.status().counts.inputs_sent==0);
% Build peer fixtures before bootstrap so setup does not consume command lifetime.
peerCases=cell(1,double(exerciseFlight));responses=peerCases;
for caseIndex=1:double(exerciseFlight),peerCases{caseIndex}=preparePeer(k+1+caseIndex+double(firstUnsentProbe),caseIndex);end
if firstUnsentProbe
 unsentPeer=preparePeer(k+2,1);
 afterSentUnsentPeer=preparePeer(k+4,1);
end
continuationIndex=k+7;
if runtimeStateOnly&&exerciseFlight
 gpServiceIndex=k+5;gpServicePeer=preparePeer(gpServiceIndex,1);
 gpResultIndex=k+6;gpResultPeer=preparePeer(gpResultIndex,1);
 continuationPeer=preparePeer(continuationIndex,1);
 negativeContinuationPeer=preparePeer(continuationIndex+1,1);
end
k=k+1;sendInput(k,true);pollSource('BOOTSTRAP',k);
t=tic;while toc(t)<5
 events{end+1}=service.poll(empty,'BOOTSTRAP');drain();s=service.status();
 if ~s.outer.worker.in_flight,break;end;pause(.002);
end
check('one_actual_fresh_outer_after_cold_preparation',s.outer.outer_submissions==1&&s.outer.completed_original_solver_results==1 ...
 &&s.outer.worker.maximum_in_flight==1&&~isempty(s.outer.last_command));
if runtimeStateOnly
 % Test the bootstrap clock with a synthetic .30 s event anchored to the solve,
 % including when no control-history packet has arrived.
 clock=s.outer.clock;
 check('armed_runtime_bootstrap_consumes_actual_outer_slot_zero', ...
  clock.flight_active&&clock.next_outer_due_ns==uint64(300000000)&&clock.outer_slots_accounted==1);
 event=struct('sample_timestamp_ns',clock.last_sample_timestamp_ns+uint64(299000000), ...
  'actual_source_delta_us',uint64(10000),'leg_id',1,'flight_active',true, ...
  'reset_leg',false,'solver_in_flight',false,'phase_rate',1,'board_local_source_bound_us',clock.maximum_actual_source_delta_us);
 [beforeDue,early]=gpenmpcNative.canonicalMeasuredSourceOuterClock(clock,event);
 check('no_early_solver_slot_before_original_030',early.accepted&&~early.dispatch_solver);
 event.sample_timestamp_ns=clock.last_sample_timestamp_ns+uint64(300000000);
 event.actual_source_delta_us=uint64(1000);
 [~,due]=gpenmpcNative.canonicalMeasuredSourceOuterClock(beforeDue,event);
 check('next_outer_slot_not_reanchored_to_delayed_first_commit', ...
  due.accepted&&due.dispatch_solver&&due.dispatch_slot_index==1&&due.dispatch_delay_ns==0);
end
check('no_disarmed_kernel_or_input_or_GP_fabricated',s.counts.inputs_sent==0&&s.counts.commits==0&&s.gp_calls==0 ...
 &&s.host_inner_steps==0&&s.plant_steps==0&&s.additional_connections==0&&s.arm_mode_commands==0);
bi=find(cellfun(@(r)~isempty(r.input_binding),events),1);
check('bounded_receipt_not_duplicate_whole_getter_ring',~isfield(events{bi}.input_binding.original_window,'records') ...
 &&numel(events{bi}.input_binding.matched_original_record)==336);
if exerciseFlight
 if firstUnsentProbe
  k=k+1;write(peer,unsentPeer.diagnostic,'uint8','127.0.0.1',cfg.truth_port);
  sendFrames(unsentPeer.source_frames);rejectFirstInput=true;
  pollSource(@()'ARM_WAIT',k);retired=service.status();
  check('unsent_first_candidate_retired_without_input_commit_or_source_renewal', ...
   strcmp(events{end}.status,'UNSENT_FIRST_FULL_INPUT_RETIRED') ...
   &&events{end}.source_retired_without_send&&~retired.failed&&~retired.source_waiting ...
   &&retired.counts.inputs_sent==0&&retired.counts.commits==0&&retired.gp_calls==0);
 end
 for round=1:double(exerciseFlight)
 k=k+1;one=peerCases{round};write(peer,one.diagnostic,'uint8','127.0.0.1',cfg.truth_port);
 sendFrames(one.source_frames);
 completeAndPartialProbe=runtimeStateOnly&&round==1;
 if armWaitProbe&&round==1
  % Exact real complete source + explicit HOST-only indication that another
  % message is incomplete. No missing bytes are admitted or timestamp reset.
  before=service.status();pollSource(@()'ARM_WAIT',k);
  held=service.status();
  firstInputEvent=events{end};
  check('arm_pending_sends_original_input_without_claiming_board_commit', ...
   held.counts.snapshots_taken==before.counts.snapshots_taken+1&&held.counts.inputs_sent==1 ...
   &&held.counts.commits==0&&held.gp_calls==0&&~held.source_waiting ...
   &&strcmp(firstInputEvent.status,'ORIGINAL_INPUT_SENT_WHILE_ARM_ACK_PENDING'));
  check('complete_valid_source_is_sent_while_newer_message_is_pending', ...
   firstInputEvent.work_timing_ns.input_send>0&&held.counts.inputs_sent==1);
  events{end+1}=service.poll(empty,@()'ARM_WAIT');drain();same=service.status();
  check('arm_pending_same_source_is_not_reencoded_or_retimestamped', ...
   same.input_codec_calls==held.input_codec_calls&&same.counts.snapshots_taken==held.counts.snapshots_taken ...
   &&same.counts.inputs_sent==1&&isempty(events{end}.source));
 else,pollSource('FLIGHT',k);end
 if completeAndPartialProbe
  check('pending_receive_resumed_once_before_input_even_with_complete_state',partialProbeReceivesBeforeSend==2);
  if ~armWaitProbe
   check('fresh_input_followed_by_one_actual_receive_before_historical_work',partialProbeReceiveCalls==3);
  end
 end
 completeAndPartialProbe=false;
 s=service.status();
 responses{round}=struct('input_event',events{end},'after_input',s);
 if armWaitProbe&&round==1,responses{round}.input_event=firstInputEvent;end
 if firstUnsentProbe&&round==1
  k=k+1;write(peer,afterSentUnsentPeer.diagnostic,'uint8','127.0.0.1',cfg.truth_port);
  sendFrames(afterSentUnsentPeer.source_frames);rejectFirstInput=true;
  pollSource('FLIGHT',k);retired=service.status();
  check('unsent_later_observation_does_not_revoke_transmitted_input_or_block_GP', ...
   strcmp(events{end}.status,'UNSENT_FULL_INPUT_RETIRED') ...
   &&events{end}.source_retired_without_send&&~retired.failed&&~retired.source_waiting ...
   &&retired.counts.inputs_sent==1&&retired.counts.commits==0&&retired.gp_calls==0);
 end
 sendFrames(one.gp_frames);
 if runtimeStateOnly,sendFrames(one.commit_frames);end
 if runtimeStateOnly&&strcmp(label,'COMPLETED_GP')
  beforePriority=service.status();timer=tic;
  while toc(timer)<.5
   events{end+1}=service.poll(empty,'FLIGHT');drain();s=service.status();
   if s.gp_completed==round,break;end
   pause(.001);
  end
  responses{round}.gp_event=events{end};responses{round}.after_gp=s;
  check('complete_GP_replied_without_waiting_for_another_source_or_input', ...
   s.gp_completed==round&&s.counts.inputs_sent==beforePriority.counts.inputs_sent ...
   &&strcmp(events{end}.status,'GP_REPLY_RETURNED')&&numel(events{end}.gp)==1 ...
   &&isempty(events{end}.committed)&&~isfield(events{end},'input_send'));
  write(peer,gpServicePeer.diagnostic,'uint8','127.0.0.1',cfg.truth_port);
  sendFrames(gpServicePeer.source_frames);k=gpServiceIndex;
  completedGpProbe=true;pollSource('FLIGHT',k);completedGpProbe=false;s=service.status();
  responses{round}.after_commit=s;responses{round}.commit_event=events{end};
  check('input_and_original_closed_observation_continue_after_priority_GP', ...
   completedGpInputSent&&events{end}.input_send.messages_send_returned==6 ...
   &&s.counts.commits==round&&s.gp_completed==round&&isempty(events{end}.gp));
 elseif runtimeStateOnly
  timer=tic;while toc(timer)<.5
   events{end+1}=service.poll(empty,'FLIGHT');drain();s=service.status();qstate=io.pollCanonical(false);
   if qstate.completed_queue_counts(string(qstate.channels)=="gp_request")>0,break;end;pause(.001);
  end
  check('idle_poll_receives_GP_without_spending_old_input_lease_on_worker', ...
   qstate.completed_queue_counts(string(qstate.channels)=="gp_request")>0 ...
   &&s.gp_completed==round-1&&isempty(events{end}.gp)&&isempty(events{end}.committed));
  write(peer,gpServicePeer.diagnostic,'uint8','127.0.0.1',cfg.truth_port);
  sendFrames(gpServicePeer.source_frames);k=gpServiceIndex;
  completedGpProbe=true;pollSource('FLIGHT',k);completedGpProbe=false;s=service.status();
  check('already_complete_GP_dispatch_has_no_redundant_post_input_receive', ...
   completedGpInputSent&&completedGpExtraReceives==0&&events{end}.input_send.messages_send_returned==6);
  if s.gp_completed<round
   % Send the next six-fragment input while the independent GP future runs.
   check('async_GP_dispatch_does_not_wait_for_prediction', ...
    events{end}.input_send.messages_send_returned==6&&isempty(events{end}.gp)&&~s.failed);
   write(peer,gpResultPeer.diagnostic,'uint8','127.0.0.1',cfg.truth_port);
   sendFrames(gpResultPeer.source_frames);k=gpResultIndex;pollSource('FLIGHT',k);s=service.status();
   check('closed_RLC_serviced_while_independent_GP_is_pending', ...
    events{end}.input_send.messages_send_returned==6&&s.counts.commits==round ...
    &&s.gp_completed==round-1&&~isempty(events{end}.committed)&&isempty(events{end}.gp));
   responses{round}.after_commit=s;responses{round}.commit_event=events{end};
   events{end+1}=service.poll(empty,'FLIGHT');drain();s=service.status();
   check('idle_poll_does_not_marshal_pending_GP_completion', ...
    s.gp_completed==round-1&&isempty(events{end}.gp)&&isempty(events{end}.committed));
   write(peer,continuationPeer.diagnostic,'uint8','127.0.0.1',cfg.truth_port);
   sendFrames(continuationPeer.source_frames);k=continuationIndex;pollSource('FLIGHT',k);s=service.status();
  end
  check('GP_actual_computation_follows_same_poll_complete_input', ...
   events{end}.input_send.messages_send_returned==6&&s.gp_completed==round&&~isempty(events{end}.gp));
 else
  timer=tic;while toc(timer)<.5
   events{end+1}=service.poll(empty,'FLIGHT');drain();s=service.status();if s.gp_completed==round,break;end;pause(.001);
  end
 end
 if ~strcmp(label,'COMPLETED_GP')||~runtimeStateOnly
  responses{round}.gp_event=events{end};responses{round}.after_gp=s;
 end
 if runtimeStateOnly
  check('runtime_GP_and_RLC_do_not_stack_in_one_poll',isempty(responses{round}.gp_event.committed) ...
   &&s.gp_completed==round&&s.counts.commits==round);
  events{end+1}=service.poll(empty,'FLIGHT');drain();s=service.status();
  check('idle_poll_cannot_spend_old_input_lifetime_on_retained_RLC', ...
   isempty(events{end}.committed)&&s.counts.commits==round);
  check('retained_RLC_follows_actual_fresh_six_fragment_input', ...
   responses{round}.commit_event.input_send.messages_send_returned==6 ...
   &&~isempty(responses{round}.commit_event.committed));
 else,sendFrames(one.commit_frames);end
 timer=tic;while s.counts.commits<round&&toc(timer)<.5
  events{end+1}=service.poll(empty,'FLIGHT');drain();s=service.status();if s.counts.commits==round,break;end;pause(.001);
 end
 if ~runtimeStateOnly
  responses{round}.after_commit=s;responses{round}.commit_event=events{end};
 end
 end
 % Deliver the negative peer while the command remains valid;
 % perform post-run numerical assertions afterwards.
 if ~strcmp(label,'COMPLETED_GP')
 activeSource=peerCases{round}.source;closed=peerCases{round}.commit;
 invalid=closed;invalid(31:38)=be(u64(activeSource(47:54))+uint64(1000));invalid(39:46)=be(uint64(round+1));
 invalid(1463:1494)=digest(invalid(1:1462));sendTunnel(invalid,14,uint64(round+1));
 if runtimeStateOnly
  write(peer,negativeContinuationPeer.diagnostic,'uint8','127.0.0.1',cfg.truth_port);
  sendFrames(negativeContinuationPeer.source_frames);k=continuationIndex+1;
 end
 id='';timer=tic;while toc(timer)<.5&&isempty(id)
  try
   if runtimeStateOnly,pollSource('FLIGHT',k);else,events{end+1}=service.poll(empty,'FLIGHT');end
  catch ex,id=ex.identifier;end;pause(.001);
 end
 neg=service.status();check('unmatched_commit_fail_closed_without_new_source_or_solve',strcmp(id,'gpenmpcNative:LocalMethodUnmatchedCommit') ...
  &&neg.failed&&neg.counts.commits==round&&neg.outer.outer_submissions==1&&service.Phase.CommitCount==round);
 flightFixture.negative_failure=id;
 else
  flightFixture.negative_failure='NOT_RERUN_UNAFFECTED_UNMATCHED_COMMIT_CHECK';
 end
 % Check original wire and numeric evidence outside the timed service loop.
 for round=1:double(exerciseFlight)
 one=peerCases{round};r=responses{round};activeSource=one.source;closed=one.commit;gp=one.gp;
 check(sprintf('active_original_input_%d_six_fragments',round),r.after_input.counts.inputs_sent==round ...
  &&r.after_input.pending_input_sources>=1&&r.after_input.counts.commits==round-1);
 if compiledInput
  check(sprintf('active_original_input_%d_uses_bound_compiled_codec_once',round), ...
   strcmp(r.input_event.input_codec_backend,'CANONICAL_LOCAL_TASK_WIRE_MEX') ...
   &&r.after_input.input_codec_calls==uint64(round+double(firstUnsentProbe)) ...
   &&strcmp(r.after_input.input_codec_backend,'CANONICAL_LOCAL_TASK_WIRE_MEX') ...
   &&r.after_input.input_codec_backend_binding.runtime_file_checks==0);
 end
 decodedInput=gpenmpcNative.RflyLocalTaskCodec.decode(activeSourceInput(u64(activeSource(47:54))));
 check('active_input_preserves_source_actual_lag_and_outer_command',decodedInput.source.source_generation==u64(activeSource(47:54)) ...
  &&isequal(decodedInput.original_sensor52,activeSource(277:328))&&decodedInput.outer.generation==s.outer.last_command.generation ...
  &&r.input_event.input_binding.matching.getter_index>0);
 if runtimeStateOnly
  check('retained_sent_input_parse_matches_original_decoder', ...
   isequaln(r.commit_event.committed{1}.outer_observation_inputs.held_input_bytes,decodedInput.original_bytes) ...
   &&isfield(r.commit_event.committed{1}.original_input_source.send,'decoded_input') ...
   &&isequaln(r.commit_event.committed{1}.original_input_source.send.decoded_input,decodedInput));
 end
 check(sprintf('active_same_owner_original_GP_reply_%d',round),r.after_gp.gp_calls==round ...
  &&r.after_gp.gp_completed==round&&r.after_gp.counts.gp_replies==round);
 gc=r.gp_event.gp{1};original=gpenmpcNative.RflyLocalGpCodec.decodeReply(flightFixture.gp(311:596,round));
 check('active_GP_original_numeric_result_not_mock_solver',max(abs(gc.computed.result18(:)-original.result18(:)))<=1e-10 ...
  &&gc.send.messages_send_returned==3&&gpenmpcNative.sameSoleMavlinkOwner(gc.send));
 check(sprintf('active_source_input_commit_phase_outer_%d_same_IO',round),r.after_commit.counts.commits==round ...
  &&(runtimeStateOnly||r.after_commit.pending_input_sources==0)&&r.after_commit.last_board_committed_window==1&&r.after_commit.outer.outer_submissions==1);
 check('retained_C_numeric_payload_unmodified_in_explicit_mock_commit',isequal(closed(99:1266),flightFixture.commits(99:1266,round)) ...
  &&isequal(closed(1367:1462),flightFixture.commits(1367:1462,round)));
 flightFixture.source=activeSource;flightFixture.request=gp;flightFixture.commit=closed;
 flightFixture.actual_input=decodedInput.original_bytes;flightFixture.success_state=s;
 end
 flightFixture.peer_cases=peerCases;flightFixture.responses=responses;
 check('constant_time_original_callback_indices_preserved',numel(gc.request.original_callback_indices)==3 ...
  &&all(diff(gc.request.original_callback_indices)>0)&&all(gc.request.original_callback_indices>0));
 forged=gc.request;forged.original_callback_indices(1)=uint64(1);beforeBad=io.evidence();id='';
 try,io.sendCanonicalLocalGp(gc.computed.reply_bytes,forged);catch ex,id=ex.identifier;end
 afterBad=io.evidence();check('forged_callback_index_cannot_send',strcmp(id,'m600check:CanonicalLocalOriginalCallback') ...
  &&numel(afterBad.raw_transmit_messages)==numel(beforeBad.raw_transmit_messages));
end
service.suspendForNativeLand();after=service.status();check('native_LAND_suspend_is_not_a_LAND_or_zero_proof',after.closed&&after.arm_mode_commands==0);
if streamLifecycleEnabled
 streamBeforeCleanup=stream.status();wanted=uint64([exerciseFlight>0 1 exerciseFlight>0]);
 expectedPolicy='FIRST_NONEMPTY_ORIGINAL_ITEM_PER_CHANNEL_ONLY';
 if runtimeStateOnly
  wanted=zeros(1,3,'uint64');expectedPolicy='RAW_RETAINED_FOR_POSTRUN__NOT_SYNCHRONOUSLY_WITNESSED';
 end
 check('only_first_original_item_per_channel_is_stream_witnessed', ...
  isequal(streamBeforeCleanup.observation_counts,wanted) ...
  &&isequal(after.stream_startup_witness.observation_counts,wanted) ...
  &&strcmp(after.stream_startup_witness.policy,expectedPolicy) ...
  &&~after.stream_startup_witness.all_messages_reobserved);
 check('stream_witness_is_not_enable_ACK_or_release_proof', ...
  ~streamBeforeCleanup.enable_command_acknowledged&&~streamBeforeCleanup.stream_disabled_proven&&~streamBeforeCleanup.release_proven);
end
before=io.evidence();
if useRawTransport
 check('single_raw_owner_replaces_unconnected_official_codec', ...
  strcmp(before.transport_kind,'OFFICIAL_CODEC_SOLE_RAW_UDP') ...
  &&~before.same_existing_mavlinkio&&before.same_existing_transport_owner ...
  &&before.actual_mavlink_subscription_count==0&&before.raw_transport_replaces_mavlinkio_connection);
 check('actual_send_receipts_preserve_complete_wire',all(cellfun(@(r) ...
  isfield(r,'transport_receipt')&&r.transport_receipt.ok ...
  &&r.transport_receipt.bytes_complete&&~isempty(r.transport_receipt.bytes),before.raw_transmit_messages)));
end
% TX has the original message field, unlike the RX topic-dispatch records.
% Preserve actual numerical/runtime evidence before packaging assertions.
save(fullfile(outputRoot,['RAW_EXECUTION' suffix '.mat']),'events','before','s','association','registered','flightFixture','delayedEnvironmentLease');
check('no_arm_mode_task_messages_in_same_IO',all(cellfun(@(r)isfield(r,'message') ...
 &&ismember(double(r.message.MsgID),[126 385]),before.raw_transmit_messages)));
cleanup();check('one_IO_released',true);
if useRawTransport
 check('single_raw_native_owner_closed',closedEvidence.raw_transport.closed);
end
if streamLifecycleEnabled
 check('explicit_finally_disable_returned_before_same_IO_close',streamCleanup.ran ...
  &&isempty(streamCleanup.disable_error)&&isempty(streamCleanup.evidence_errors) ...
  &&isempty(streamCleanup.resource_errors)&&streamCleanup.io_closed ...
  &&streamAfterCleanup.disable_send_returned&&streamAfterCleanup.disable_attempts==1 ...
  &&~isempty(streamCleanup.original_disable_transmit_rows));
 check('cleanup_preserves_original_ingress_failure_and_no_release_claim', ...
  strcmp(streamCleanup.io_failure_before,streamCleanup.io_failure_after) ...
  &&~streamAfterCleanup.stream_disabled_proven&&~streamAfterCleanup.release_proven);
end
report=struct('passed',all([checks.pass]),'checks',checks,'test_count',numel(checks),'hardware_actions',0,'COM',0,'plant_runs',0, ...
 'scope','ACTUAL_MATLAB_SAME_IO_ORIGINAL_WORKER_PREPARATION_EXPLICIT_MOCK_PEER_AND_RETAINED_NOUI_GETTERS', ...
 'live_source_age_bound_validated',false,'state_before_close',s);
report.delayed_environment_lease_fixture=delayedEnvironmentLease;
report.single_raw_transport_enabled=useRawTransport;
if streamLifecycleEnabled
 report.stream_lifecycle=struct('enabled',true,'scope','HOST_MOCK_PEER_STARTUP_WITNESS_INTEGRATION_ONLY', ...
  'observation_policy','FIRST_NONEMPTY_ORIGINAL_ITEM_PER_CHANNEL_ONLY_NOT_ALL_MESSAGES', ...
  'enable',streamEnable,'before_cleanup',streamBeforeCleanup,'after_cleanup',streamAfterCleanup, ...
  'cleanup',streamCleanup,'production_full_task_proven',false);
end
if exerciseFlight
 report.scope='ACTUAL_MATLAB_COMPOSITE_METHOD_SERVICE_EXPLICIT_MOCK_PEER_NO_NEW_C_OR_BOARD_EXECUTION';
 report.flight_fixture_scope=flightFixture.scope;
 report.flight_poll_elapsed_ns=cellfun(@(r)double(r.processing_elapsed_ns),events(cellfun(@(r)strcmp(r.mode,'FLIGHT'),events)));
 selected=events(cellfun(@(r)strcmp(r.mode,'FLIGHT'),events));
 report.flight_work_timing_ns=cellfun(@(r)r.work_timing_ns,selected);
 report.matlab_numerical_threads=maxNumCompThreads;
end
save(fullfile(outputRoot,['RAW' suffix '.mat']),'report','events','before','association','registered','flightFixture','delayedEnvironmentLease');
f=fopen(resultPath,'w');fprintf(f,'%s\n',jsonencode(report,PrettyPrint=true));fclose(f);
disp(jsonencode(struct('passed',report.passed,'test_count',report.test_count)));diary off
catch ex
 debug=struct();if ~isempty(service),debug=service.status();disp(jsonencode(struct('failed',debug.failed,'failure',debug.failure, ...
  'counts',debug.counts,'gp_calls',debug.gp_calls,'pending_sources',debug.pending_input_sources)));end
 if ~isempty(events),disp(jsonencode(struct('last_poll_status',events{end}.status,'source_present',~isempty(events{end}.source))));end
 failurePath=fullfile(outputRoot,'FAILURE_RAW.mat');failures={};
 if isfile(failurePath),old=load(failurePath,'failures');failures=old.failures;end
 failedEvent=[];if ~isempty(service),failedEvent=service.LastEvent;end
 if streamLifecycleEnabled,cleanup();end
 failures{end+1}=struct('label',label,'requested_mock_cycles',double(exerciseFlight), ...
  'error',getReport(ex,'extended','hyperlinks','off'),'events',{events},'state',debug,'failed_poll_event',failedEvent);
 if streamLifecycleEnabled
  failures{end}.stream_lifecycle=struct('enable',streamEnable,'after_cleanup',streamAfterCleanup,'cleanup',streamCleanup, ...
   'observation_policy','FIRST_NONEMPTY_ORIGINAL_ITEM_PER_CHANNEL_ONLY_NOT_ALL_MESSAGES');
 end
 save(failurePath,'failures');
 cleanup();clear streamGuard
 f=fopen(fullfile(outputRoot,'FAILURE.txt'),'a');fprintf(f,'%s\n',getReport(ex,'extended','hyperlinks','off'));fclose(f);diary off;rethrow(ex)
end
clear streamGuard
 function r=checkDelayedEnvironmentLease()
  % Only an explicit HOST envelope is rebound to retained actual getter
  % bytes. No simulated timestamp is labelled as an original board receipt.
  pair=detail.mutually_unique_retained_pairs(1);g=pair.retained_getter_index+1;
  body=raw(:,1);body(277:328)=uint8(sscanf(pair.equal_sensor52_bytes_hex,'%2x'));
  body(351:382)=digest(body(1:350));rx=actual.reads(end)+uint64(1);
  p=x.original_phase_receipts{1};p.source_rls_sha256=digest(body);
  p.original_bytes=[uint8('EXPLICIT_ISOLATED_HOST_PHASE_REBIND_NOT_BOARD_CALLBACK:').';p.original_bytes(:)];
  e0=x.original_environment_receipts{1};
  tx=struct('bytes',e0.original_frame232(:),'original_host_send_ns',rx-uint64(1),'send_returned',true);
  ack=struct('bytes',e0.original_diagnostic264(:),'original_host_receive_ns',rx+uint64(1));
  ack.bytes(25:32)=actual.records(265:272,g); % original accepted model_time bits
  localGetter=leaseGetter();ledger=gpenmpcNative.RflyLocalEnvironmentLedger(policy,1,16);ledger.sent(tx);
  time=readLE(actual.records(265:272,g),'double');
  check('isolated_ENV_send_alone_has_no_same_step_ACK',isempty(ledger.resolve(time)));
  [missing,prior]=localGetter.bind(body,rx,ts,p,ledger);
  check('missing_ENV_retains_incomplete_original_lease', ...
   strcmp(missing.status,'MISSING_SAME_STEP_ACCEPTED_ENVIRONMENT')&&~missing.numerical_inputs_complete ...
   &&~prior.retirement_lease.complete&&isempty(missing.payload));
  beforeLease=localGetter.status();
  ledger.received(ack); % Preserve event ordering.
  [complete,afterLease]=localGetter.bind(body,rx,ts,p,ledger);
  check('same_incomplete_source_resolves_without_retimestamp',complete.numerical_inputs_complete ...
   &&afterLease.retirement_lease.complete ...
   &&isequal(afterLease.retirement_lease.source.original_bytes,prior.retirement_lease.source.original_bytes) ...
   &&afterLease.retirement_lease.source.original_host_receive_ns==rx ...
   &&prior.retirement_lease.source.original_host_receive_ns==rx ...
   &&isequaln(complete.source,missing.source)&&isequaln(complete.rotor,missing.rotor));
  completeState=localGetter.status();
  check('delayed_ENV_does_not_allocate_new_source_or_retire_early', ...
   beforeLease.matched_sources==1&&completeState.matched_sources==1 ...
   &&beforeLease.explicitly_retired_getters==0&&completeState.explicitly_retired_getters==0);
  localGetter.retireResolvedSource(complete.source.source_generation);done=localGetter.status();
  check('resolved_original_lease_is_retired_once',done.explicitly_retired_getters==uint64(g) ...
   &&done.retained_getters==0&&~done.original_timestamps_renewed);
  for defect=1:2
   badGetter=leaseGetter();emptyLedger=gpenmpcNative.RflyLocalEnvironmentLedger(policy,1,16);emptyLedger.sent(tx);
   badGetter.bind(body,rx,ts,p,emptyLedger);emptyLedger.received(ack);badBody=body;badRx=rx;
   if defect==1,badRx=rx+uint64(1);
   else,badBody(260)=bitxor(badBody(260),uint8(1));badBody(351:382)=digest(badBody(1:350));end
   rejectExact(sprintf('delayed_ENV_cannot_replace_original_lease_key_%d',defect), ...
    @()badGetter.bind(badBody,badRx,ts,p,emptyLedger),'gpenmpcNative:GetterLease');
  end
  check('retained_pairs2and3_really_share_an_already_ACKable_model_step', ...
   detail.mutually_unique_retained_pairs(2).original_accepted_time_s==.02 ...
   &&detail.mutually_unique_retained_pairs(3).original_accepted_time_s==.02);
  r=struct('scope','ISOLATED_HOST_GETTER_ENVIRONMENT_LEDGER_WITH_RETAINED_NOUI_BYTES_AND_EXPLICIT_MOCK_RLS_PHASE_ACK', ...
   'same_source_bytes_and_original_receive_preserved',true,'source_receive_ns',rx, ...
   'delayed_ack_receive_ns',ack.original_host_receive_ns,'same_model_time_s',time, ...
   'getter_records',g,'source_generation',complete.source.source_generation, ...
   'no_actual_IO_or_MethodService_clock',true,'live_callback_or_board_proven',false, ...
   'prior_failure_file',char(fullfile(gpenmpc_external_path('stream_startup_method_input_mex'),'FAILURE_RAW.mat')), ...
   'prior_failure_cause','Exact-step ACK reuse is checked independently of expired-disarmed records.');
  function v=leaseGetter()
   v=gpenmpcNative.RflyLocalOriginalGetterBuffer(256,'990850A2F40F3FCC2A6C47E63A4065B60FF49AA39CC4749FF443963B06F2EF7E');
   v.ingest(batch(1:g));
  end
 end
 function b=batch(ix),b=actual.last;b.records=actual.records(:,ix);b.original_read_ns=actual.reads(ix);end
 function pollSource(mode,wanted)
  timer=tic;wantedGeneration=u64(raw(47:54,wanted));
  while toc(timer)<.5
   events{end+1}=service.poll(empty,mode);drain();
   % Match the callback by retained source generation, not aggregate dequeue count.
   if ~isempty(events{end}.source)&&u64(events{end}.source.message(47:54))==wantedGeneration,return;end
   pause(.002);
  end
  error('test:OriginalCallbackTimeout');
 end
 function r=pollWithPartialObservation(varargin)
  if completedGpProbe&&completedGpInputSent&&~isempty(varargin)&&varargin{1}
   completedGpExtraReceives=completedGpExtraReceives+1;
  end
  r=originalPollCanonical(varargin{:});
  if completeAndPartialProbe
   r.snapshot_receive_pending=true;
   if ~isempty(varargin)&&varargin{1},partialProbeReceiveCalls=partialProbeReceiveCalls+1;end
  end
 end
 function r=sendWithFirstUnsentProbe(varargin)
  if completeAndPartialProbe,partialProbeReceivesBeforeSend=partialProbeReceiveCalls;end
  if rejectFirstInput
   rejectFirstInput=false;
   error('m600check:CanonicalLocalInputUnsentExpired','Explicit HOST fixture: no transport call or fragment attempted.');
  end
  r=originalInputSend(varargin{:});
  if completedGpProbe,completedGpInputSent=true;end
 end
 function body=sendInput(k,withAck)
  body=raw(:,k);body(5:12)=be(identity.uid);body(13:20)=be(identity.boot_generation);body(21)=identity.system;body(22)=identity.component;
  sensor=uint8(sscanf(detail.mutually_unique_retained_pairs(k).equal_sensor52_bytes_hex,'%2x'));body(277:328)=sensor;body(351:382)=digest(body(1:350));
  if k==1
   orig=x.original_environment_receipts{1};f=readLE(orig.original_frame232(9:232),'double');[taskRows,~]=ts.cached_asset.read(ts);
   f(4)=0;f(6:7)=gpenmpcNative.canonicalSavedTaskWindAt(taskRows,0,1);io.sendPlantEnvironment(frameValue(f));drain();
  end
  % Complete pure peer encoding before the first original RX timestamp can
  % exist; no timestamps or expiry values are changed to absorb fixture work.
  frames=tunnelFrames(body,10,u64(body(47:54)));
  if withAck,sendDiagnostic(k);end
  sendFrames(frames);
  pause(.005);
 end
 function bytes=activeSourceInput(wanted)
  ev=io.evidence();rows=ev.raw_transmit_messages;parts={};
  for ii=1:numel(rows)
   m=rows{ii}.message;if m.MsgID~=385,continue;end;p=m.Payload;kind=double(p.payload(1));
   if kind>=208&&kind<=213&&u64(p.payload(2:9))==wanted
    parts{kind-207}=reshape(uint8(p.payload(10:double(p.payload_length))),[],1);
   end
  end
  assert(numel(parts)==6&&all(~cellfun(@isempty,parts)));bytes=vertcat(parts{:});bytes=bytes(:);
 end
 function sendTunnel(body,schema,generation)
  sendFrames(tunnelFrames(body,schema,generation));
 end
 function frames=tunnelFrames(body,schema,generation)
  frames=cell(1,ceil(numel(body)/119));
  for ii=0:ceil(numel(body)/119)-1
   m=createmsg(d,'TUNNEL');part=[uint8(schema*16+ii);be(generation);body(ii*119+1:min((ii+1)*119,numel(body)))];
   m.Payload.payload_type=uint16(42002);m.Payload.target_system=uint8(255);m.Payload.target_component=uint8(190);
   m.Payload.payload_length=uint8(numel(part));m.Payload.payload(:)=0;m.Payload.payload(1:numel(part))=part;
   frames{ii+1}=uint8(serializemsg(serializer,m));
  end
 end
 function sendFrames(frames)
  for ii=1:numel(frames),write(peer,frames{ii},'uint8','127.0.0.1',cfg.local_mavlink_port);end
 end
 function one=preparePeer(sourceIndex,ordinal)
  body=raw(:,sourceIndex);body(5:12)=be(identity.uid);body(13:20)=be(identity.boot_generation);
  body(21)=identity.system;body(22)=identity.component;
  body(277:328)=uint8(sscanf(detail.mutually_unique_retained_pairs(sourceIndex).equal_sensor52_bytes_hex,'%2x'));
  body(351:382)=digest(body(1:350));g=detail.mutually_unique_retained_pairs(sourceIndex).retained_getter_index+1;
  diagnostic=x.original_environment_receipts{1}.original_diagnostic264;diagnostic(25:32)=actual.records(265:272,g);
  q=flightFixture.gp(1:310,ordinal);q(5:22)=body(5:22);q(23:30)=be(u64(body(23:30))*uint64(1000));
  q(31:38)=body(47:54);q(39:46)=be(uint64(ordinal));q(47:54)=body(31:38);
  q(55:62)=be(u64(body(23:30))*uint64(1000)+uint64(1000000000));q(279:310)=digest(q(1:278));
  c=flightFixture.commits(:,ordinal);c(5:22)=body(5:22);c(23:30)=q(23:30);c(31:38)=body(47:54);
  c(39:46)=be(uint64(ordinal));c(47:54)=body(31:38);c(55:62)=be(uint64(ordinal));c(71:78)=be(uint64(1));
  c(87:94)=be(uint64(1));c(95:98)=be(uint32(1));c(1335:1366)=uint8(sscanf(taskSha,'%2x'));c(1463:1494)=digest(c(1:1462));
  one=struct('source',body,'gp',q,'commit',c,'diagnostic',diagnostic, ...
   'source_frames',{tunnelFrames(body,10,u64(body(47:54)))},'gp_frames',{tunnelFrames(q,8,uint64(ordinal))}, ...
   'commit_frames',{tunnelFrames(c,14,uint64(ordinal))});
 end
 function sendDiagnostic(k)
  g=detail.mutually_unique_retained_pairs(k).retained_getter_index+1;
  b=x.original_environment_receipts{1}.original_diagnostic264;b(25:32)=actual.records(265:272,g);
  write(peer,b,'uint8','127.0.0.1',cfg.truth_port);
 end
 function records=sendText(text)
  n=ceil(numel(text)/70);beforeCount=numel(io.canonicalSessionReceipts());
  for j=1:n
   m=createmsg(d,'SERIAL_CONTROL');part=uint8(text((j-1)*70+1:min(j*70,numel(text))));
   m.Payload.device=uint8(10);m.Payload.flags=uint8(1);m.Payload.count=uint8(numel(part));m.Payload.data(:)=0;m.Payload.data(1:numel(part))=part;
   write(peer,uint8(serializemsg(serializer,m)),'uint8','127.0.0.1',cfg.local_mavlink_port);
  end
  timer=tic;while numel(io.canonicalSessionReceipts())<beforeCount+n&&toc(timer)<3,pause(.003);end
  all=io.canonicalSessionReceipts();assert(numel(all)==beforeCount+n);records=[all{beforeCount+1:end}];
 end
 function drain(),while peer.NumDatagramsAvailable>0,read(peer,1,'uint8');end;end
 function cleanup()
  if streamLifecycleEnabled
   if streamCleanup.ran,return;end,streamCleanup.ran=true;
   % Keep cleanup available after expected ingress errors without polling or clearing the fault.
   if ~isempty(stream)
    try
     state=stream.status();
     if state.enable_attempts>0
      first=[];
      try
       prior=io.evidence();streamCleanup.io_failure_before=prior.canonical_exchange_failure;
       first=numel(prior.raw_transmit_messages);
      catch problem,streamCleanup.evidence_errors{end+1}=getReport(problem,'extended','hyperlinks','off');end
      % Evidence-read failure must not bypass the explicit disable attempt.
      try,stream.disable();catch problem,streamCleanup.disable_error=getReport(problem,'extended','hyperlinks','off');end
      try
       current=io.evidence();streamCleanup.io_failure_after=current.canonical_exchange_failure;
       if ~isempty(first),streamCleanup.original_disable_transmit_rows=current.raw_transmit_messages(first+1:end);end
      catch problem,streamCleanup.evidence_errors{end+1}=getReport(problem,'extended','hyperlinks','off');end
     end
     streamAfterCleanup=stream.status();
    catch problem
     streamCleanup.disable_error=getReport(problem,'extended','hyperlinks','off');
    end
   end
   if ~isempty(service)
    try,service.close();catch problem,streamCleanup.resource_errors{end+1}=getReport(problem,'extended','hyperlinks','off');end
   end
   if ~isempty(io)
    try,streamCleanup.io_closed=io.close();closedEvidence=io.evidence();catch problem,streamCleanup.resource_errors{end+1}=getReport(problem,'extended','hyperlinks','off');end
    io=[];
   end
   if ~isempty(peer)
    try,delete(peer);catch problem,streamCleanup.resource_errors{end+1}=getReport(problem,'extended','hyperlinks','off');end,peer=[];
   end
   if ~isempty(serializer)
    try,delete(serializer);catch problem,streamCleanup.resource_errors{end+1}=getReport(problem,'extended','hyperlinks','off');end,serializer=[];
   end
   return
  end
  if ~isempty(service),service.close();end
  if ~isempty(io),io.close();closedEvidence=io.evidence();io=[];end
  if ~isempty(peer),delete(peer);peer=[];end;if ~isempty(serializer),delete(serializer);serializer=[];end
 end
 function check(n,v),checks(end+1)=struct('name',n,'pass',logical(v));assert(v,'test:Check','%s',n);end
 function reject(n,fn),id='';try,fn();catch ex,id=ex.identifier;end;check(n,~isempty(id));end
 function rejectExact(n,fn,wanted),id='';try,fn();catch ex,id=ex.identifier;end;check(n,strcmp(id,wanted));end
end
function report=checkGpInputContinuation(build,out)
% Test the scheduling blocks with host callback doubles.
source=fullfile(build,'host_runtime','+gpenmpcNative','RflyLocalMethodService.m');
text=fileread(source);
a=strfind(text,'                receiveStart=obj.now();');
b=strfind(text,'                event.work_timing_ns.receive=obj.now()-receiveStart;');
receive=text(a(1):b(1)-1);
a=strfind(text,'                if runtimeStateOnly&&~obj.ComponentInitialization&&strcmp(mode,''FLIGHT'') ...');
b=strfind(text,'                % The same future can finish while fast input is prepared.');
priority=text(a(find(a<b(1),1,'last')):b(1)-1);
assert(a(find(a<b(1),1,'last'))>findLast(text,'sent=obj.Io.sendCanonicalLocalInput(inputs,obj.Source);'));
checks=struct('name',{},'pass',{});
for k=1:11
 r=oneCase(k);
 if k==1,passed=r.continued&&r.receive_calls==1&&r.gp_calls==1&&r.gp_replies==1;
 elseif k==2,passed=r.continued&&r.receive_calls==2&&r.gp_calls==1&&r.gp_replies==1;
 elseif k==3,passed=r.continued&&r.gp_calls==0&&r.gp_replies==0;
 elseif k==4,passed=strcmp(r.error,'test:ActualGpFailure')&&~r.continued&&r.gp_replies==0;
 elseif k==5,passed=r.continued&&r.receive_calls==2&&r.gp_calls==1&&r.gp_replies==1;
 elseif k==6,passed=strcmp(r.error,'gpenmpcNative:LocalMethodTransport')&&~r.continued&&r.receive_calls==2;
 elseif ismember(k,[7 8]),passed=r.continued&&r.gp_calls==0&&r.gp_replies==0;
 elseif k==9,passed=r.continued&&r.receive_calls==1&&r.gp_calls==0&&r.gp_replies==1;
 elseif k==10,passed=r.continued&&r.receive_calls==2&&r.gp_calls==0&&r.gp_replies==1;
 else,passed=strcmp(r.error,'gpenmpcNative:LocalGpReceiveMissing') ...
   &&strcmp(r.error_message,'Inline GP computation is required for this request.') ...
   &&~r.continued&&r.gp_calls==0&&r.gp_replies==0;
 end
 names={'input_receipt_precedes_GP_work','GP_from_second_receive_is_serviced','no_GP_leaves_source_path_available', ...
  'real_GP_failure_still_propagates','missing_source_services_one_receive_before_GP','actual_bounded_receive_failure_propagates', ...
  'missing_input_receipt_does_not_enable_GP_work','partial_input_receipt_does_not_enable_GP_work', ...
  'inline_result_uses_completed_source_without_recomputing','inline_result_with_pending_source_continues_receive', ...
  'missing_inline_computation_is_rejected'};
 checks(end+1)=struct('name',names{k},'pass',passed); %#ok<AGROW>
 assert(passed,'test:GpInputContinuation','%s: %s',names{k},jsonencode(r));
end
report=struct('passed',all([checks.pass]),'checks',checks,'production_source',source, ...
 'scope','PRODUCTION_SCHEDULING_BLOCKS_WITH_HOST_DOUBLES', ...
 'COM',0,'board',0,'plant',0,'control',0);
if ~isfolder(out),mkdir(out);end
p=fullfile(out,'GP_INPUT_CONTINUATION_RESULT.json');assert(~isfile(p));
fid=fopen(p,'w');assert(fid>=0);guard=onCleanup(@()fclose(fid)); %#ok<NASGU>
fprintf(fid,'%s',jsonencode(report,'PrettyPrint',true));
 function r=oneCase(k)
  r=struct('continued',false,'receive_calls',0,'gp_calls',0,'gp_replies',0,'error','','error_message','');
  request=struct('message',uint8(1),'original_host_receive_ns',uint64(900),'origin',struct());
  if ismember(k,[9 10]),request.inline_gp=struct('computed',struct('reply_bytes',uint8(1)));end
  obj=struct('now',@()uint64(1000),'ComponentInitialization',false, ...
   'Gp',struct('AsyncEnabled',false,'ReceiveInlineEnabled',k>=9,'process',@predict), ...
   'Io',struct('pollCanonical',@receiveCall,'takeCanonical',@(varargin)request, ...
    'sendCanonicalLocalGp',@(varargin)struct('messages_send_returned',3)), ...
   'Counts',struct('gp_replies',0),'witnessStream',@(varargin)[]);
  event=struct('gp',{{}},'work_timing_ns',struct('gp',uint64(0),'receive',uint64(0)), ...
   'input_send',struct('messages_send_returned',6));
  if k==7,event=rmfield(event,'input_send');end
  if k==8,event.input_send.messages_send_returned=5;end
  runtimeStateOnly=true;mode='FLIGHT';serviceHeartbeat=@()[];
  receiveStart=[];link=[];gpQueued=false;mark=[];q=[];witness=[];answer=[];sent=[]; %#ok<NASGU>
  try
   eval(receive);eval(priority);
   r.continued=true;
  catch ex,r.error=ex.identifier;r.error_message=ex.message;end
  r.gp_replies=obj.Counts.gp_replies;
  function v=receiveCall(varargin)
   r.receive_calls=r.receive_calls+1;
   hasGp=k==1||k==4||k>=5||(k==2&&r.receive_calls==2);
   v=struct('failure','','channels',["gp_request","snapshot"], ...
    'completed_queue_counts',[double(hasGp),1],'snapshot_receive_pending',ismember(k,[2 10])&&r.receive_calls==1);
   if ismember(k,[2 5 6 7 8 10])&&r.receive_calls==1,v.completed_queue_counts(2)=0;end
   if k==6&&r.receive_calls==2,v.failure='ACTUAL_RECEIVE_FAILURE';end
  end
  function v=predict(varargin)
   r.gp_calls=r.gp_calls+1;
   if k==4,error('test:ActualGpFailure','Explicit inference failure.');end
   v=struct('reply_bytes',uint8(1));
  end
 end
end
function report=checkGpAsyncContinuation(build,out)
source=fullfile(build,'host_runtime','+gpenmpcNative','RflyLocalMethodService.m');
text=fileread(source);
a=strfind(text,'                % BEGIN_ASYNC_GP_RECEIVE_SERVICE');
b=strfind(text,'                % END_ASYNC_GP_RECEIVE_SERVICE');early=text(a(1):b(1)-1);
a=strfind(text,'                % BEGIN_ASYNC_GP_POST_INPUT_COMPLETION');
b=strfind(text,'                % END_ASYNC_GP_POST_INPUT_COMPLETION');late=text(a(1):b(1)-1);
a=strfind(text,'                % BEGIN_ASYNC_GP_NEW_RX_DISPATCH');
b=strfind(text,'                % END_ASYNC_GP_NEW_RX_DISPATCH');dispatchOnly=text(a(1):b(1)-1);
names={'finished_reply_without_new_input','unfinished_future_does_not_wait', ...
 'dispatch_without_new_input','mismatched_reply_rejected','expired_send_rejected', ...
 'finished_during_input_sent_before_history','new_RX_query_dispatched_before_input_without_fetch_or_send'};
checks=struct('name',{},'pass',{});
for k=1:7
 r=oneCase(k);
 if ismember(k,[1 6]),ok=r.replies==1&&r.dispatches==0&&isempty(r.error);
 elseif k==2,ok=r.replies==0&&r.dispatches==0&&r.polls==2&&isempty(r.error);
 elseif ismember(k,[3 7]),ok=r.replies==0&&r.dispatches==1&&isempty(r.error);
 elseif k==4,ok=r.replies==0&&strcmp(r.error,'gpenmpcNative:LocalGpBackendReply');
 else,ok=r.replies==0&&strcmp(r.error,'test:OriginalSendExpired');end
 checks(end+1)=struct('name',names{k},'pass',ok); %#ok<AGROW>
 assert(ok,'test:GpAsyncContinuation','%s: %s',names{k},jsonencode(r));
end
earlyAt=strfind(text,'                % BEGIN_ASYNC_GP_RECEIVE_SERVICE');
receiveAt=strfind(text,'                receiveStart=obj.now();');
lateAt=strfind(text,'                % BEGIN_ASYNC_GP_POST_INPUT_COMPLETION');
dispatchAt=strfind(text,'                % BEGIN_ASYNC_GP_NEW_RX_DISPATCH');
inputAt=strfind(text,'                % ARM_WAIT may send a current original slow input');
ok=numel(earlyAt)==1&&numel(receiveAt)==1&&numel(lateAt)==1&&numel(dispatchAt)==1 ...
 &&earlyAt<receiveAt&&receiveAt<dispatchAt&&dispatchAt<inputAt&&inputAt<lateAt;
assert(ok,'test:GpAsyncOrder','Ready-result service must precede fresh receive.');
checks(end+1)=struct('name','completion_before_RX_or_after_input_but_new_dispatch_before_input','pass',ok);
guardLines=regexp(text,['(?m)^\s*if \(~gpQueued\|\|obj\.Gp\.ReceiveInlineEnabled\) \.\.\.\r?\n' ...
 '\s*&&\(isempty\(event\.gp\)\|\|obj\.Gp\.AsyncEnabled\|\|obj\.Gp\.ReceiveInlineEnabled\)'],'match');
assert(numel(guardLines)==1,'test:GpAsyncHistorySource','Expected one complete retained-history predicate.');
guardLine=guardLines{1};
% Queued GP, async backend, inline backend, prior reply, expected history access.
historyCases=logical([0 0 0 0 1;0 0 0 1 0;0 0 1 0 1;0 0 1 1 1; ...
 0 1 0 0 1;0 1 0 1 1;0 1 1 0 1;0 1 1 1 1; ...
 1 0 0 0 0;1 0 0 1 0;1 0 1 0 1;1 0 1 1 1; ...
 1 1 0 0 0;1 1 0 1 0;1 1 1 0 1;1 1 1 1 1]);
for k=1:size(historyCases,1)
 row=historyCases(k,:);obj=struct('Gp',struct('AsyncEnabled',row(2),'ReceiveInlineEnabled',row(3)));
 event=struct('gp',{{}});if row(4),event.gp={struct('actual_reply',true)};end
 gpQueued=row(1);historyAllowed=false;
 eval([guardLine newline 'historyAllowed=true;' newline 'end']);
 name=sprintf('history_queue_%d_async_%d_inline_%d_reply_%d',row(1:4));
 ok=historyAllowed==row(5);checks(end+1)=struct('name',name,'pass',ok); %#ok<AGROW>
 assert(ok,'test:GpAsyncHistory','%s',name);
end
report=struct('passed',all([checks.pass]),'checks',checks,'production_source',source, ...
 'scope','PRODUCTION_ASYNC_BLOCKS_AND_HISTORY_PREDICATE_WITH_HOST_DOUBLES', ...
 'hardware_actions',0);
if ~isfolder(out),mkdir(out);end
p=fullfile(out,'GP_ASYNC_READY_RESULT.json');assert(~isfile(p));
fid=fopen(p,'w');assert(fid>=0);guard=onCleanup(@()fclose(fid)); %#ok<NASGU>
fprintf(fid,'%s',jsonencode(report,'PrettyPrint',true));
 function r=oneCase(k)
  r=struct('replies',0,'dispatches',0,'polls',0,'error','');pending=~ismember(k,[3 7]);received=false;
  beforeInput=false;
  q=struct('message',uint8(1),'original_host_receive_ns',uint64(900),'origin',struct());
  fixtureRequest=q;
  obj=struct('now',@()uint64(1000),'ComponentInitialization',false,'PendingGp',q, ...
   'Gp',struct('AsyncEnabled',true,'poll',@pollFuture,'isPending',@pendingState,'begin',@dispatch), ...
   'Io',struct('takeCanonical',@take,'sendCanonicalLocalGp',@send), ...
   'Counts',struct('gp_replies',0),'witnessStream',@(varargin)[]);
  event=struct('gp',{{}},'work_timing_ns',struct('gp',uint64(0)));
  runtimeStateOnly=true;mode='FLIGHT';serviceHeartbeat=@()[];
  mark=[];answer=[];sent=[];witness=[]; %#ok<NASGU>
  try
   eval(early);received=true;gpQueued=k==7;beforeInput=true;
   pollsBefore=r.polls;repliesBefore=obj.Counts.gp_replies;eval(dispatchOnly);
   assert(r.polls==pollsBefore&&obj.Counts.gp_replies==repliesBefore, ...
    'test:NoGpCompletionBeforeInput','Only numerical dispatch is allowed before input.');
   if k==7,assert(r.dispatches==1&&pending,'test:ImmediateNewGpDispatch');end
   beforeInput=false;eval(late);r.replies=obj.Counts.gp_replies;
  catch ex,r.error=ex.identifier;end
  function v=take(varargin)
   v=fixtureRequest;if k==7&&~received,v=[];end
  end
  function yes=pendingState(),yes=pending;end
  function v=pollFuture()
   r.polls=r.polls+1;v=[];
   if ismember(k,[2 3 7])||(k==6&&r.polls==1),return,end
   bytes=uint8(1);if k==4,bytes=uint8(2);end
   v=struct('request',struct('original_bytes',bytes),'reply_bytes',uint8(3));pending=false;
  end
  function dispatch(varargin)
   if k==7,assert(received&&beforeInput,'test:GpDispatchLocation');end
   r.dispatches=r.dispatches+1;pending=true;
  end
  function v=send(varargin)
   if k==5,error('test:OriginalSendExpired','Real sender expiry remains fatal.');end
   v=struct('messages_send_returned',3);
  end
 end
end
function n=findLast(text,needle),indices=strfind(text,needle);assert(~isempty(indices));n=indices(end);end
function v=frameValue(f)
v=struct('schema_version',f(1),'generation',f(2),'source_io_time_s',f(3),'task_reference_time_s',f(4),'payload_kg',f(5), ...
 'wind_ned_xy_mps',f(6:7),'mission_phase',f(8),'reference_jet_ned',f(9:20),'payload_generation',f(21), ...
 'task_clock_paused',logical(f(22)),'session_token',f(23),'board_min_rx_io_time_s',f(24), ...
 'board_valid_flags',f(25),'landed_state',f(26),'continuity_epoch',f(27),'service_release_generation',f(28));
end
function b=readbin(p),f=fopen(p,'rb');assert(f>=0);c=onCleanup(@()fclose(f));b=fread(f,Inf,'*uint8');end
function b=be(v),[~,~,e]=computer;if e=='L',v=swapbytes(v);end;b=reshape(typecast(v(:),'uint8'),[],1);end
function v=u64(b),v=typecast(b(:),'uint64');[~,~,e]=computer;if e=='L',v=swapbytes(v);end;v=v(:);end
function v=readLE(b,t),v=typecast(b(:),t);[~,~,e]=computer;if e=='B',v=swapbytes(v);end;v=v(:);end
function s=hex(b),s=upper(reshape(dec2hex(b,2).',1,[]));end
function h=digest(b),m=java.security.MessageDigest.getInstance('SHA-256');m.update(typecast(b(:),'int8'));h=reshape(typecast(m.digest(),'uint8'),[],1);end
