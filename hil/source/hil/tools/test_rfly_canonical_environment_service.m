function report=test_rfly_canonical_environment_service(outputRoot)
% Test frame encoding and ACK observation with a no-socket IO spy.
build=string(fileparts(fileparts(mfilename('fullpath'))));
addpath(fullfile(build,'host_runtime'),fullfile(build,'matlab_validation'));
assert(~isfolder(outputRoot));mkdir(outputRoot);
source=fullfile(build,'host_runtime','+gpenmpcNative','RflyCanonicalEnvironmentService.m');
before=sha(source);checks=struct('name',{},'pass',{});negatives=struct('name',{},'identifier',{});
contract=struct('expected_session_token',26090501,'initial_payload_kg',2.21, ...
    'payload_by_generation_kg',[2.21;1.75;.98;.55;0], ...
    'mass_by_generation_kg',[11.71;11.25;10.48;10.05;9.5],'remote_port',30100);
cfg=struct('target_system',1,'target_component',1,'heartbeat_max_age_s',3, ...
    'landed_max_age_s',2,'delivery_environment_contract',contract);
clockS=12;sendCalls=0;forbiddenCalls=0;throwSend=false;lastWire=[];lastFields=[];
io=struct('now',@clockNow,'sendPlantEnvironment',@sendEnvironment, ...
    'sendSetpoint',@forbidden,'sendHeartbeat',@forbidden,'requestCommand',@forbidden,'close',@forbidden);
id=struct('schema','RFLY_CALLER_VERIFIED_SAME_IO_IDENTITY_V1','verified',true, ...
    'uid',uint64(123456789012345678),'system_id',1,'component_id',1,'boot_generation',uint64(7), ...
    'configuration_payload_sha256',repmat('A',1,64),'execution_session_sha256',repmat('B',1,64), ...
    'original_observed_io_time_s',11,'original_observed_host_ns',uint64(1000000000), ...
    'provenance','EXPLICIT_HOST_TEST_IDENTITY_RECORD_NOT_CURRENT_BOARD_OBSERVATION');
expected=struct('uid',string(id.uid),'system_id',1,'component_id',1,'boot_generation',id.boot_generation, ...
    'configuration_payload_sha256',id.configuration_payload_sha256, ...
    'rfly_board_commit',struct('execution_session_sha256',id.execution_session_sha256));
s=struct('now_s',12,'heartbeat_rx_s',11.9,'extended_rx_s',11.8,'armed',0,'landed_state',1, ...
    'clock_valid',true,'model_ready',true,'fatal','','delivery_board_continuity_epoch',3, ...
    'autopilot_version',struct('uid',id.uid),'model_diagnostic',struct('decoded',struct('sim_time_s',8)), ...
    'delivery_environment',struct('present',false,'mass_ack_valid',false));
reference=struct('position_ned_m',[1 2 -3],'velocity_ned_mps',[.1 .2 .3], ...
    'acceleration_ned_mps2',[.4 .5 .6],'jerk_ned_mps3',[.7 .8 .9], ...
    'estimated_wind_ned_mps',[2 -1 0]);
view=struct('schema','RFLY_CANONICAL_DELIVERY_TIME_VIEW_V1','saved_task_time_s',12.5, ...
    'controller_phase_s',37.5,'wall_time_used',false,'plant_truth_used',false);
intent=struct('payload_kg',2.21,'payload_generation',0,'release_generation',0, ...
    'task_clock_paused',false,'lifecycle_phase','TRANSIT','payload_update_requested',false);
service=new();r=service.step(s,view,reference,intent,id,expected);
old=gpenmpcTaskIo.makePlantEnvironmentFrameFromSnapshot(s,reference,2.21,0,0,1,12.5,false, ...
    "TRANSIT",26090501,true,true,true);[originalWire,originalFields]=gpenmpcTaskIo.encodePlantEnvironmentV2(old,1,2.21);
check('same_retained_frame_maker_28double_bits',isequal(lastFields,originalFields)&&isequal(lastWire,originalWire));
check('actual_send_is_not_model_ACK',strcmp(r.status,'ENVIRONMENT_SENT_NOT_MODEL_ACK')&&~r.payload_ack.matched);
check('shared_reference_all_p_v_a_j_exact',isequal(lastFields(9:20),[1;2;-3;.1;.2;.3;.4;.5;.6;.7;.8;.9]));
check('uses_saved_task_phase_not_wall_or_controller_phase',lastFields(4)==12.5&&lastFields(4)~=s.now_s&&lastFields(4)~=view.controller_phase_s);
check('original_board_and_model_time_mapping',lastFields(3)==8&&abs(lastFields(24)-7.8)<1e-12);
initialCalls=sendCalls;
for k=1:2000,repeat=service.step(s,view,reference,intent,id,expected);end
check('2000_unchanged_source_polls_never_resend_or_renew',sendCalls==initialCalls ...
    &&strcmp(repeat.status,'NO_NEW_MODEL_SOURCE_NO_SEND')&&service.status().frame_generation==1);
% New model time with a temporarily worse asynchronous board mapping keeps
% the ORIGINAL greatest lower bound; no timestamp is changed in the input.
s2=s;s2.now_s=12.03;s2.model_diagnostic.decoded.sim_time_s=8.01;clockS=s2.now_s;
r2=service.step(s2,view,reference,intent,id,expected);st=service.status();
check('original_bound_regression_retained_not_poll_time',r2.frame.board_min_rx_io_time_s==old.board_min_rx_io_time_s ...
    &&abs(st.last_attempt.raw_board_lower_bound_s-7.78)<1e-12 ...
    &&st.board_lower_bound_clamp_count==1&&st.last_attempt.original_heartbeat_rx_s==11.9);
% Decode ACK fields from the V2 fixture and supply synthetic accepted-core state.
s2.delivery_environment=ackSnapshot(2,0,2.21,8.01,true,0);
ack=service.step(s2,view,reference,intent,id,expected);
check('decoder_ACK_matches_previously_sent_initial_payload',ack.payload_ack.matched ...
    &&~ack.payload_ack.payload_update_ack_for_lifecycle&&ack.payload_ack.actual_payload_kg==2.21);
change=intent;change.payload_kg=1.75;change.payload_generation=1;change.release_generation=1;
change.payload_update_requested=true;change.task_clock_paused=true;change.lifecycle_phase='SERVICE_GROUNDED_DISARMED';
s3=s2;s3.now_s=12.04;s3.model_diagnostic.decoded.sim_time_s=8.02;clockS=s3.now_s;
s3.delivery_environment=ackSnapshot(2,1,1.75,8.02,true,0);
sent=service.step(s3,view,reference,change,id,expected);
check('preexisting_or_future_payload_claim_not_new_send_ACK',~sent.payload_ack.matched&&sent.frame.payload_generation==1);
s3.delivery_environment=ackSnapshot(3,1,1.75,8.02,true,0);
matched=service.step(s3,view,reference,change,id,expected);
check('exact_session_generation_payload_ACK_for_existing_lifecycle',matched.payload_ack.matched ...
    &&matched.payload_ack.payload_update_ack_for_lifecycle&&matched.payload_ack.applied_frame_generation==3);
s3.delivery_environment.mass_ack_valid=false;
stale=service.step(s3,view,reference,change,id,expected);
check('stale_original_delivery_observer_does_not_ACK',~stale.payload_ack.matched);
check('only_same_io_environment_sender_ever_called',forbiddenCalls==0&&service.status().native_control_messages==0 ...
    &&service.status().heartbeat_messages==0&&service.status().disk_writes==0);
beforeClose=sendCalls;service.close();check('close_does_not_close_shared_io_or_reset_plant',sendCalls==beforeClose&&forbiddenCalls==0);
reject(@()service.step(s3,view,reference,change,id,expected),'gpenmpcNative:EnvironmentClosed','closed_cannot_resume');
% Each negative has a fresh DISCARDABLE helper; production failure is latched.
clockS=12;
bad=id;bad.verified=false;negative(s,view,reference,intent,bad,expected,'gpenmpcNative:EnvironmentIdentityReceipt','caller_identity_not_verified');
bad=id;bad.execution_session_sha256=repmat('C',1,64);negative(s,view,reference,intent,bad,expected,'gpenmpcNative:EnvironmentIdentityMismatch','wrong_execution_session');
bad=id;bad.boot_generation=uint64(8);negative(s,view,reference,intent,bad,expected,'gpenmpcNative:EnvironmentIdentityMismatch','wrong_process_session_generation');
bad=id;bad.original_observed_io_time_s=12.01;negative(s,view,reference,intent,bad,expected,'gpenmpcNative:EnvironmentIdentityReceipt','identity_observed_in_future');
badS=s;badS.autopilot_version.uid=uint64(4);negative(badS,view,reference,intent,id,expected,'gpenmpcNative:EnvironmentObservedUid','actual_UID_drift');
badView=view;badView.wall_time_used=true;negative(s,badView,reference,intent,id,expected,'gpenmpcNative:EnvironmentTimeView','wall_clock_view_rejected');
badRef=reference;badRef.jerk_ned_mps3(1)=NaN;negative(s,view,badRef,intent,id,expected,'gpenmpcTaskIo:ReferenceFinite','nonfinite_shared_jerk');
badIntent=intent;badIntent.payload_generation=2;badIntent.release_generation=2;badIntent.payload_kg=.98;
negative(s,view,reference,badIntent,id,expected,'gpenmpcNative:EnvironmentPayloadIntent','payload_generation_skip');
badS=s;badS.armed=1;negative(badS,view,reference,change,id,expected,'gpenmpcNative:EnvironmentUnloadIntent','armed_unload_request');
badS=s;badS.heartbeat_rx_s=8.999;negative(badS,view,reference,intent,id,expected,'gpenmpcNative:EnvironmentBoardEvidenceStale','old_three_second_HB_receive_gate');
badS=s;badS.extended_rx_s=9.999;negative(badS,view,reference,intent,id,expected,'gpenmpcNative:EnvironmentBoardEvidenceStale','old_two_second_EXT_receive_gate');
badS=s;badS.heartbeat_rx_s=12.001;negative(badS,view,reference,intent,id,expected,'gpenmpcNative:EnvironmentBoardEvidenceStale','future_board_receive');
% A fresh heartbeat after stale reception may resume safety-environment service,
% but not task execution.
receiveFailed=new();badS=s;badS.heartbeat_rx_s=8.999;
reject(@()receiveFailed.step(badS,view,reference,intent,id,expected), ...
    'gpenmpcNative:EnvironmentBoardEvidenceStale','observed_receive_age_failure');
firstReceiveFailure=receiveFailed.status().first_failure;
landReceive=intent;landReceive.lifecycle_phase='NATIVE_LAND';landReceive.task_clock_paused=true;
reject(@()receiveFailed.step(badS,view,reference,landReceive,id,expected), ...
    'gpenmpcNative:EnvironmentBoardEvidenceStale','native_land_cannot_send_still_stale_board');
freshReceive=s;freshReceive.now_s=12.01;freshReceive.model_diagnostic.decoded.sim_time_s=8.01;clockS=12.01;
recovered=receiveFailed.step(freshReceive,view,reference,landReceive,id,expected);
check('new_real_board_reception_restores_only_native_land_environment', ...
    strcmp(recovered.status,'ENVIRONMENT_SENT_NOT_MODEL_ACK')&&receiveFailed.Failed ...
    &&isequal(receiveFailed.status().first_failure,firstReceiveFailure));
reject(@()receiveFailed.step(freshReceive,view,reference,intent,id,expected), ...
    'gpenmpcNative:EnvironmentClosed','fresh_receive_never_restarts_failed_task');
clockS=12;
clockS=12.251;negative(s,view,reference,intent,id,expected,'gpenmpcNative:EnvironmentQueuedSnapshotExpired','known_queue_delay_exceeds_original_250ms');
queueFailed=new();
reject(@()queueFailed.step(s,view,reference,intent,id,expected), ...
    'gpenmpcNative:EnvironmentQueuedSnapshotExpired','queue_failure_retained_for_safety');
firstQueueFailure=queueFailed.status().first_failure;
landIntent=intent;landIntent.lifecycle_phase='NATIVE_LAND';landIntent.task_clock_paused=true;
fresh=s;fresh.now_s=12.26;fresh.model_diagnostic.decoded.sim_time_s=8.26;clockS=fresh.now_s;
safe=queueFailed.step(fresh,view,reference,landIntent,id,expected);
check('fresh_native_land_environment_without_restarting_control', ...
    strcmp(safe.status,'ENVIRONMENT_SENT_NOT_MODEL_ACK')&&queueFailed.Failed ...
    &&isequal(queueFailed.status().first_failure,firstQueueFailure));
reject(@()queueFailed.step(fresh,view,reference,intent,id,expected), ...
    'gpenmpcNative:EnvironmentClosed','failed_formal_control_never_resumes');
fresh.now_s=12.27;fresh.model_diagnostic.decoded.sim_time_s=8.27;clockS=fresh.now_s;
fresh.fatal='BOARD_LOCAL_CONTEXT_STOPPED:7';
safe=queueFailed.step(fresh,view,reference,landIntent,id,expected);
check('controller_stop_does_not_disable_same_healthy_plant_native_land', ...
    strcmp(safe.status,'ENVIRONMENT_SENT_NOT_MODEL_ACK')&&queueFailed.Failed);
fresh.clock_valid=false;
reject(@()queueFailed.step(fresh,view,reference,landIntent,id,expected), ...
    'gpenmpcNative:EnvironmentModelClock','native_land_still_requires_real_clock');
fresh.clock_valid=true;fresh.fatal='MODEL_FAILED:3';
reject(@()queueFailed.step(fresh,view,reference,landIntent,id,expected), ...
    'gpenmpcNative:EnvironmentModelClock','native_land_cannot_ignore_model_failure');
fresh.fatal='';fresh.model_diagnostic.decoded.sim_time_s=8.28;clockS=12.521;
reject(@()queueFailed.step(fresh,view,reference,landIntent,id,expected), ...
    'gpenmpcNative:EnvironmentQueuedSnapshotExpired','native_land_retains_original_250ms');
check('secondary_safety_failure_never_rewrites_first_atom', ...
    isequal(queueFailed.status().first_failure,firstQueueFailure));
clockS=12.25;boundary=new();ok=boundary.step(s,view,reference,intent,id,expected);
check('known_queue_delay_exact_original_250ms_allowed',strcmp(ok.status,'ENVIRONMENT_SENT_NOT_MODEL_ACK'));
clockS=12;waiting=new();badS=s;badS.model_ready=false;
beforeWait=sendCalls;waitResult=waiting.step(badS,view,reference,intent,id,expected);
check('before_BEGIN_missing_clock_waits_without_starting_environment',sendCalls==beforeWait&&~waiting.Failed ...
    &&strcmp(waitResult.status,'WAITING_FOR_ORIGINAL_MODEL_CLOCK'));
missingUid=s;missingUid=rmfield(missingUid,'autopilot_version');r=waiting.step(missingUid,view,reference,intent,id,expected);
check('absent_snapshot_UID_keeps_explicit_caller_provenance_no_fabrication', ...
    ~waiting.status().last_attempt.identity_independently_verified_here ...
    &&isequal(waiting.status().last_attempt.caller_verified_same_io_identity,id));
reject(@()waiting.step(badS,view,reference,intent,id,expected),'gpenmpcNative:EnvironmentModelClock','active_model_clock_loss_latches');
latched=waiting.status().first_failure;
reject(@()waiting.step(s,view,reference,intent,id,expected),'gpenmpcNative:EnvironmentClosed','fresh_poll_cannot_wash_clock_failure');
check('original_failure_preserved',isequal(waiting.status().first_failure,latched));
throwSend=true;failedSend=new();beforeAttempt=sendCalls;
reject(@()failedSend.step(s,view,reference,intent,id,expected),'gpenmpcNative:EnvironmentTestSendFailure','actual_send_failure_preserved');
state=failedSend.status();throwSend=false;
check('failed_attempt_is_not_success_or_ACK',sendCalls==beforeAttempt+1&&state.send_attempt_count==1 ...
    &&state.send_return_count==0&&state.last_attempt.send_attempted&&~state.last_attempt.send_returned);
% Independently retain actual plant-side age check with its frozen policy.
p=load(fullfile(gpenmpc_external_path('environment_delivery_model'),'M600_CORE_PARAMETERS.mat'),'deliveryPolicy');p=p.deliveryPolicy;
check('actual_plant_policy_250ms_unchanged',p.max_env_age_s==.25);
f=originalFields;f(2)=1;f(3)=100;f(24)=99.9;f(4)=0;f(22)=1;
plant=gpenmpcTaskIo.initPlantEnvironmentV2(p,100);
[~,admitted]=gpenmpcTaskIo.acceptPlantEnvironmentV2(plant,f,true,100.25,true,p);
[~,late]=gpenmpcTaskIo.acceptPlantEnvironmentV2(plant,f,true,100.251,true,p);
check('actual_acceptor_original_250ms_boundary_not_changed',admitted.pending&&~admitted.task_env_failed ...
    &&late.task_env_failed&&late.failure_code==3);
% Actual saved first leg -> shared takeoff evaluator -> production time view
% -> retained frame maker. Only coordinator/ground observations are fixtures.
bundle=gpenmpcNative.loadRflyCanonicalDeliveryTask(fullfile(build,'task_packages','cambridge_canonical', ...
    'MU_CAMBRIDGE_MA_02__CANONICAL_PHYSICAL_TASK.mat'), ...
    'B876546B468265F84BD028AC549B06EF8ADDCB25CBA09CF389A08F4B21290A5F');
trajectory=gpenmpcNative.bindRflyCanonicalInitialTakeoffTrajectory(bundle.legs{1},zeros(3,1));
for phaseValue=[-.01 0 .02 .04 4.5 19.99 20 20.02 24.99 25 25.02 30]
    originalJet=zeros(3,4);
    for derivative=0:3,originalJet(:,derivative+1)=trajectory.evaluate_fcn(phaseValue,derivative);end
    check(sprintf('prepared_environment_jet_bit_exact_phase_%g',phaseValue), ...
        isequal(typecast(originalJet(:),'uint64'),typecast(reshape(trajectory.environment_jet_fcn(phaseValue),[],1),'uint64')));
end
coordinator=struct('state','FLIGHT','failed',false,'service',struct('leg_index',1,'phase_s',22.5, ...
    'payload_kg',2.21,'outer_suspended',false));
clockS=12;actual=new();actualSnapshot=s;actualIntent=intent;
keys={'position_ned_m','velocity_ned_mps','acceleration_ned_mps2','jerk_ned_mps3'};
for k=1:2
    if k==2,coordinator.service.phase_s=25.5;clockS=12.01;actualSnapshot.now_s=clockS;actualSnapshot.model_diagnostic.decoded.sim_time_s=8.01;end
    actualView=gpenmpcNative.rflyCanonicalDeliveryTimeView(bundle,coordinator,trajectory);
    for order=0:3,actualReference.(keys{order+1})=diag([1,1,-1])*trajectory.evaluate_fcn(coordinator.service.phase_s,order);end
    actualReference.estimated_wind_ned_mps=[2;-1;0]; % Independent fixture wind.
    if k==1,actualIntent.lifecycle_phase='INITIAL_TAKEOFF';else,actualIntent.lifecycle_phase='TRANSIT';end
    actualResult=actual.step(actualSnapshot,actualView,actualReference,actualIntent,id,expected);
    expectedJet=[actualReference.position_ned_m;actualReference.velocity_ned_mps; ...
        actualReference.acceleration_ned_mps2;actualReference.jerk_ned_mps3];
    check(sprintf('actual_saved_task_shared_jet_timeview_frame_%d',k), ...
        isequal(actualResult.frame.reference_jet_ned,expectedJet) ...
        &&actualResult.frame.task_reference_time_s==max(0,coordinator.service.phase_s-25));
end
check('first_preparation_is_not_old_native_hold_or_setpoint',forbiddenCalls==0 ...
    &&actual.status().send_return_count==2&&actualView.saved_task_time_s==.5);
check('production_source_unchanged',strcmpi(before,sha(source)));
report=struct('passed',all([checks.pass]),'total',numel(checks),'checks',checks,'negative_cases',negatives, ...
    'source',source,'source_sha256',sha(source),'original_snapshot_fixture',s,'caller_identity_fixture',id, ...
    'environment_contract',contract,'first_wire_232',originalWire,'first_fields28',originalFields, ...
    'scope','PURE_HOST_EXISTING_FRAME_CODEC_ACK_OBSERVER_AND_PLANT_ACCEPTOR_WITH_NO_SOCKET_IO_SPY', ...
    'identity_provenance','EXPLICIT_HOST_FIXTURE__NOT_ACTUAL_VERIFIED_BOARD_OR_COPTERSIM', ...
    'active_environment_age_s',.25,'heartbeat_receive_age_s',3,'extended_receive_age_s',2, ...
    'registered_host_heartbeat_service_implemented_here',false, ...
    'helper_history_slots',1,'test_send_calls',sendCalls,'forbidden_control_calls',forbiddenCalls, ...
    'actual_saved_task_to_shared_reference_to_timeview_exercised',true, ...
    'COM_opens',0,'board_actions',0,'sockets_created',0,'solver_calls',0,'model_runs',0);
save(fullfile(outputRoot,'RAW.mat'),'report','st','state');f=fopen(fullfile(outputRoot,'RESULT.json'),'w');assert(f>=0);
c=onCleanup(@()fclose(f));fprintf(f,'%s\n',jsonencode(report,PrettyPrint=true));clear c
disp(jsonencode(struct('passed',report.passed,'checks',report.total,'native_control_calls',forbiddenCalls)));
    function obj=new(),obj=gpenmpcNative.RflyCanonicalEnvironmentService(io,cfg);end
    function value=clockNow(),value=clockS;end
    function sent=sendEnvironment(v)
        sendCalls=sendCalls+1;if throwSend,error('gpenmpcNative:EnvironmentTestSendFailure','Explicit IO send-failure fixture.');end
        [lastWire,lastFields]=gpenmpcTaskIo.encodePlantEnvironmentV2(v,cfg.target_system,contract.initial_payload_kg);
        sent=struct('sent_s',clockS,'generation',v.generation,'payload_generation',v.payload_generation, ...
            'service_release_generation',v.service_release_generation,'bytes',numel(lastWire),'remote_port',30100);
    end
    function forbidden(varargin),forbiddenCalls=forbiddenCalls+1;error('gpenmpcNative:ForbiddenControl','Environment helper called another control API.');end
    function check(name,ok),checks(end+1)=struct('name',name,'pass',logical(ok));assert(ok,'gpenmpcNative:EnvironmentTest','%s',name);end
    function reject(fn,idExpected,name)
        actual='';try,fn();catch ex,actual=ex.identifier;end
        negatives(end+1)=struct('name',name,'identifier',actual);check(name,strcmp(actual,idExpected));
    end
    function negative(a,b,c,d,e,f,idExpected,name)
        obj=new();beforeSend=sendCalls;reject(@()obj.step(a,b,c,d,e,f),idExpected,name);
        check([name,'_no_send_and_terminal'],sendCalls==beforeSend&&obj.Failed);
    end
    function value=ackSnapshot(generation,payloadGen,payload,simTime,valid,status)
        terrain=m600check.encodeCopterSimTerrainDiagnostics([0;0;simTime;114.8;1;0;1],zeros(15,1),0,0,false,false, ...
            m600check.initialCopterSimTerrainDiagnosticState());
        a=struct('valid',valid,'task_env_failed',status>0,'failure_code',status,'session_token',contract.expected_session_token, ...
            'applied_frame_generation',generation,'applied_payload_generation',payloadGen, ...
            'actual_payload_kg',payload,'actual_total_mass_kg',contract.mass_by_generation_kg(payloadGen+1));
        body=m600check.encodeCopterSimDeliveryDiagnostics(terrain,a,false);
        wire=[typecast(int32([1234567890,1]),'uint8'),typecast(body(:).','uint8')];
        decoded=m600check.decodeCopterSimDeliveryDiagnostics(wire,1,NaN,contract);
        assert(decoded.packet_valid,'gpenmpcNative:EnvironmentTestAckFixture','Actual decoder rejected the explicit fixture: %s',decoded.status);
        value=struct('present',true,'mass_ack_valid',decoded.environment_extension.mass_ack_valid,'decoded',decoded);
    end
end
function value=sha(path)
f=fopen(path,'rb');assert(f>=0);c=onCleanup(@()fclose(f));b=fread(f,Inf,'*uint8');clear c
d=java.security.MessageDigest.getInstance('SHA-256');d.update(typecast(b,'int8'));
value=upper(reshape(dec2hex(typecast(d.digest(),'uint8'),2).',1,[]));
end
