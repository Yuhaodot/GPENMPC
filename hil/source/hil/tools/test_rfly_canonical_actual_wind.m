function report=test_rfly_canonical_actual_wind(outputRoot,environmentOnly)
if nargin<2,environmentOnly=false;end
% Test saved-task wind interpolation and ENV/ACK codecs with synthetic IO.
build=string(fileparts(fileparts(mfilename('fullpath'))));addpath(fullfile(build,'host_runtime'),fullfile(build,'matlab_validation'));
assert(~isfolder(outputRoot));mkdir(outputRoot);checks=struct('name',{},'pass',{});
canonicalSource=gpenmpc_external_path('canonical_task_runner_source');
check('original_whole_task_source_matches_unique_passport',strcmpi(shaFile(canonicalSource), ...
    'BD9852A0CB4F350727C7A6B40E75004D85EC8AC2168C85EC556F7F93C7DBB7B9'));
task=fullfile(build,'task_packages','cambridge_canonical','MU_CAMBRIDGE_MA_02__CANONICAL_PHYSICAL_TASK.mat');
taskSha='B876546B468265F84BD028AC549B06EF8ADDCB25CBA09CF389A08F4B21290A5F';ts=struct('path',task,'sha256',taskSha);
q=load(task,'physicalTask');r=q.physicalTask.reference;
ix=find(r.leg_index(1:end-1)==1&r.leg_index(2:end)==1&any(diff(r.wind_estimate_xy_mps)~=0,2),1,'first');assert(~isempty(ix));
taskTime=mean(r.global_time_s(ix:ix+1));
actual=interp1(double(r.global_time_s),double(r.actual_wind_xy_mps),taskTime,"linear").';
estimate=interp1(double(r.global_time_s),double(r.wind_estimate_xy_mps),taskTime,"linear").';
check('saved_task_actual_differs_from_estimate_and_left_hold',~isequal(actual,estimate)&&~isequal(estimate,double(r.wind_estimate_xy_mps(ix,:)).'));
actualIndex=find(any(diff(r.actual_wind_xy_mps)~=0,2),1,'first');actualTime=mean(r.global_time_s(actualIndex:actualIndex+1));
[queried,~]=gpenmpcNative.canonicalSavedTaskWindAt(r,actualTime,r.leg_index(actualIndex));
check('actual_task_wind_transition_matches_canonical_linear_not_hold', ...
    isequal(queried,interp1(double(r.global_time_s),double(r.actual_wind_xy_mps),actualTime,"linear").') ...
    &&~isequal(queried,double(r.actual_wind_xy_mps(actualIndex,:)).'));
contract=struct('expected_session_token',26090501,'initial_payload_kg',2.21,'payload_by_generation_kg',[2.21;1.75;.98;.55;0], ...
    'mass_by_generation_kg',[11.71;11.25;10.48;10.05;9.5],'remote_port',30100);
cfg=struct('target_system',1,'target_component',1,'heartbeat_max_age_s',3,'landed_max_age_s',2,'delivery_environment_contract',contract);
clockS=12;sends=0;forbidden=0;lastFields=[];lastBytes=[];
io=struct('now',@clockNow,'sendPlantEnvironment',@send,'sendSetpoint',@deny,'sendHeartbeat',@deny,'requestCommand',@deny);
configuration='A859433D0AA774013341444A4B9AE971A34F12B4FB89C0A004B2CB3E013FCEBA';
id=struct('schema','RFLY_CALLER_VERIFIED_SAME_IO_IDENTITY_V1','verified',true,'uid',uint64(12345),'system_id',1,'component_id',1, ...
    'boot_generation',uint64(7),'configuration_payload_sha256',configuration,'execution_session_sha256',repmat('B',1,64), ...
    'original_observed_io_time_s',11,'original_observed_host_ns',uint64(1000000000),'provenance','EXPLICIT_HOST_IDENTITY_MOCK');
expected=struct('uid',string(id.uid),'system_id',1,'component_id',1,'boot_generation',id.boot_generation,'configuration_payload_sha256',configuration, ...
    'rfly_board_commit',struct('execution_session_sha256',id.execution_session_sha256));
s=struct('now_s',12,'heartbeat_rx_s',11.9,'extended_rx_s',11.8,'armed',0,'landed_state',1,'clock_valid',true,'model_ready',true,'fatal','', ...
    'delivery_board_continuity_epoch',3,'autopilot_version',struct('uid',id.uid),'model_diagnostic',struct('decoded',struct('sim_time_s',8)), ...
    'delivery_environment',struct('present',false,'mass_ack_valid',false));
ref=struct('position_ned_m',[1;2;-3],'velocity_ned_mps',[.1;.2;.3],'acceleration_ned_mps2',[.4;.5;.6],'jerk_ned_mps3',[.7;.8;.9], ...
    'estimated_wind_ned_mps',[estimate;0],'actual_wind_ned_xy_mps',actual,'environment_task_sha256',taskSha);
view=struct('schema','RFLY_CANONICAL_DELIVERY_TIME_VIEW_V1','saved_task_time_s',taskTime,'global_leg_index',1, ...
    'wall_time_used',false,'plant_truth_used',false);
intent=struct('payload_kg',2.21,'payload_generation',0,'release_generation',0,'task_clock_paused',false,'lifecycle_phase','TRANSIT','payload_update_requested',false);
obj=gpenmpcNative.RflyCanonicalEnvironmentService(io,cfg,ts);first=obj.stepCanonical(s,view,ref,intent,id,expected);
check('canonical_ENV_fields6_7_are_actual_not_estimate',isequal(lastFields(6:7),actual)&&~isequal(lastFields(6:7),estimate));
check('reference_controller_estimate_untouched_and_receipt_separate',isequal(ref.estimated_wind_ned_mps,[estimate;0]) ...
    &&isequal(obj.status().last_attempt.original_task_wind_selection.controller_wind_estimate_xy_mps,estimate)&&~first.payload_ack.matched);
before=sends;for k=1:20,obj.stepCanonical(s,view,ref,intent,id,expected);end
check('same_original_model_time_no_new_send_or_credit',sends==before&&obj.status().last_original_model_time_s==8);
withAck=s;withAck.delivery_environment=ackAt(8);a=obj.stepCanonical(withAck,view,ref,intent,id,expected);
check('same_model_original_ACK_permitted_not_send_return',a.payload_ack.matched&&sends==before);
bad=withAck;bad.delivery_environment=ackAt(8.001);reject('ACK different modeltime',@()obj.stepCanonical(bad,view,ref,intent,id,expected));
check('ACK conflict_latches_no_send',obj.Failed&&sends==before);
for kind=1:5
    o=gpenmpcNative.RflyCanonicalEnvironmentService(io,cfg,ts);wrong=ref;wrongView=view;
    if kind==1,wrong=rmfield(wrong,'actual_wind_ned_xy_mps');end
    if kind==2,wrong.actual_wind_ned_xy_mps(1)=NaN;end
    if kind==3,wrong.environment_task_sha256=repmat('0',1,64);end
    if kind==4,wrong.actual_wind_ned_xy_mps=estimate;end
    if kind==5,wrongView.global_leg_index=2;end
    before=sends;reject("bad canonical input "+kind,@()o.stepCanonical(s,wrongView,wrong,intent,id,expected));
    check("input reject no send "+kind,o.Failed&&sends==before);
end
wrongTask=ts;wrongTask.sha256=repmat('0',1,64);reject('wrong actual task identity',@()gpenmpcNative.RflyCanonicalEnvironmentService(io,cfg,wrongTask));
legacy=gpenmpcNative.RflyCanonicalEnvironmentService(io,cfg);reject('legacy object cannot use canonical entry',@()legacy.stepCanonical(s,view,ref,intent,id,expected));
manualCfg=cfg;manualCfg.local_short=struct('component_initialization',true,'service_cfg',struct('operator_reference',true));
manual=gpenmpcNative.RflyCanonicalEnvironmentService(io,manualCfg,ts);
windSamples=zeros(301,2);sample=s;
for k=0:300
 clockS=12+.1*k;sample.now_s=clockS;sample.heartbeat_rx_s=clockS;sample.extended_rx_s=clockS;
 sample.model_diagnostic.decoded.sim_time_s=8+.1*k;
 manual.stepCanonical(sample,view,ref,intent,id,expected);windSamples(k+1,:)=lastFields(6:7).';
end
check('manual_wind_changes_with_model_time_while_task_phase_stays_fixed',all(std(windSamples)>.1));
check('manual_gust_bound_and_initial_value',isequal(windSamples(1,:).',actual)&&all(abs(windSamples-actual.')<=1.6,'all'));
check('manual_profile_records_actual_applied_wind',isequal(manual.status().last_attempt.original_task_wind_selection.manual_disturbance.actual_wind_xy_mps,windSamples(end,:).'));
before=sends;manual.stepCanonical(sample,view,ref,intent,id,expected);
check('manual_same_source_not_resent',sends==before);
plain=gpenmpcNative.RflyCanonicalEnvironmentService(io,cfg,ts);plain.stepCanonical(sample,view,ref,intent,id,expected);
check('autonomous_wind_unchanged_by_manual_profile',isequal(lastFields(6:7),actual));
check('disturbance_never_publishes_control',forbidden==0);
if environmentOnly
    % Test the cached environment path.
    report=struct('passed',all([checks.pass]),'checks',{checks},'hardware_actions',0);
    save(fullfile(outputRoot,'RESULT.mat'),'report','first','lastBytes');disp(report);return
end
old=gpenmpcTaskIo.makePlantEnvironmentFrameFromSnapshot(s,ref,2.21,0,0,1,taskTime,false,"TRANSIT",26090501,true,true,true);
check('original thirteen_argument_compatibility_still_estimate',isequal(old.wind_ned_xy_mps,estimate));
noEstimate=rmfield(ref,'estimated_wind_ned_mps');old=gpenmpcTaskIo.makePlantEnvironmentFrameFromSnapshot(s,noEstimate,2.21,0,0,1,taskTime,false,"TRANSIT",26090501,true,true,true);
check('legacy_absent_estimate_still_zero_only_legacy',isequal(old.wind_ned_xy_mps,zeros(2,1)));
explicit=struct('schema','RFLY_EXPLICIT_CANONICAL_PLANT_WIND_V1','actual_wind_ned_xy_mps',actual);
fresh=gpenmpcTaskIo.makePlantEnvironmentFrameFromSnapshot(s,noEstimate,2.21,0,0,1,taskTime,false,"TRANSIT",26090501,true,true,true,explicit);
check('explicit_maker_never_requires_or_substitutes_estimate',isequal(fresh.wind_ned_xy_mps,actual));
badExplicit=rmfield(explicit,'actual_wind_ned_xy_mps');reject('maker_missing_explicit_actual',@()gpenmpcTaskIo.makePlantEnvironmentFrameFromSnapshot(s,ref,2.21,0,0,1,taskTime,false,"TRANSIT",26090501,true,true,true,badExplicit));
src=fullfile(build,'rfly_vendor_integration','full_inner_abi','snapshot_wire_fixture','RLS1_SNAPSHOTS.bin');
bytes=reshape(readbin(src),382,[]);rx=uint64(9000000000);sample=gpenmpcNative.RflyLocalSnapshotDecoder(bytes(:,1),rx);
e=struct('uid',string(sample.identity.uid),'system_id',double(sample.identity.system),'component_id',double(sample.identity.component), ...
    'boot_generation',sample.identity.boot_generation,'maximum_runtime_age_ns',uint64(100000000));
provider=gpenmpcNative.RflyTaskWindProvider(task,taskSha,e);
[w,wr]=provider.observeLocalCanonical(taskTime,1,bytes(:,1),rx,rx);
check('new_RLS_canonical_provider_estimate_linear_not_actual',isequal(w.estimate_xy_mps,estimate)&&isequal(wr.actual_plant_wind_xy_mps,actual) ...
    &&strcmp(wr.source_packet_kind,'RLS1_BOARD_LOCAL_ORIGINAL_OBSERVATION')&&~wr.controller_uses_actual_wind);
[again,dup]=provider.observeLocalCanonical(taskTime,1,bytes(:,1),rx,rx+uint64(1));
check('same_source_repeat_preserves_original_receive_and_receipt',isequal(w,again)&&isequal(wr,dup));
reject('same_source_phase_cannot_change',@()provider.observeLocalCanonical(taskTime+.001,1,bytes(:,1),rx,rx+uint64(2)));
oldSource=readbin(fullfile(gpenmpc_external_path('board_commit_exchange_fixture'),'SOURCE_1.bin'));
oldDecoded=gpenmpcNative.RflySnapshotDecoder(oldSource,rx);
oldExpected=struct('uid',string(oldDecoded.observed_uid),'system_id',double(oldDecoded.source_system), ...
    'component_id',double(oldDecoded.source_component),'boot_generation',oldDecoded.observed_boot_generation, ...
    'configuration_payload_sha256',upper(reshape(dec2hex(oldDecoded.configuration_sha256,2).',1,[])), ...
    'maximum_runtime_age_ns',uint64(100000000));
oldProvider=gpenmpcNative.RflyTaskWindProvider(task,taskSha,oldExpected);
[oldWind,oldReceipt]=oldProvider.observe(taskTime,oldSource,rx,rx);
check('explicit_old_RSP_provider_preserves_left_hold_compatibility', ...
    isequal(oldWind.estimate_xy_mps,double(r.wind_estimate_xy_mps(ix,:)).') ...
    &&~isequal(oldWind.estimate_xy_mps,estimate)&&oldReceipt.selected_row==ix);
oldRegression=test_rfly_canonical_environment_service(fullfile(outputRoot,'legacy_environment'));
bindingRegression=test_rfly_local_original_inputs(fullfile(outputRoot,'local_binding'));
check('old_compatibility_and_new_fractional_binding_regressions',oldRegression.passed&&bindingRegression.failed==0);
check('no_native_control_or_heartbeat_called',forbidden==0);
paths=[fullfile(build,'host_runtime','+gpenmpcNative','RflyCanonicalEnvironmentService.m'), ...
    fullfile(build,'matlab_validation','+gpenmpcTaskIo','makePlantEnvironmentFrameFromSnapshot.m'), ...
    fullfile(build,'host_runtime','+gpenmpcNative','RflyTaskWindProvider.m'),fullfile(build,'host_runtime','+gpenmpcNative','canonicalSavedTaskWindAt.m'), ...
    fullfile(build,'host_runtime','+gpenmpcNative','bindRflyLocalOriginalInputs.m')];
report=struct('scope','CANONICAL_TASK_WIND_SEPARATION_HOST_ONLY','checks',numel(checks),'failed',sum(~[checks.pass]), ...
    'legacy_environment_checks',oldRegression.total,'local_binding_checks',bindingRegression.checks,'checks_detail',checks, ...
    'source_paths',paths,'source_sha256',arrayfun(@shaFile,paths),'task_sha256',taskSha, ...
    'sample_task_time_s',taskTime,'sample_actual_wind',actual,'sample_estimator_wind',estimate, ...
    'canonical_source_path',canonicalSource,'canonical_source_sha256',shaFile(canonicalSource), ...
    'test_subject','Saved-task actual wind and estimated wind interpolation', ...
    'canonical_interpolation','INTERP1_LINEAR', ...
    'IO_identity_ACK_MOCK',true,'COM',0,'model_runs',0,'board_actions',0);
save(fullfile(outputRoot,'RAW.mat'),'report','lastBytes','first','wr');f=fopen(fullfile(outputRoot,'RESULT.json'),'w');assert(f>=0);c=onCleanup(@()fclose(f));fprintf(f,'%s\n',jsonencode(report,PrettyPrint=true));clear c
disp(jsonencode(struct('checks',report.checks,'failed',report.failed,'legacy',oldRegression.total,'binding',bindingRegression.checks)));assert(report.failed==0);
    function value=clockNow(),value=clockS;end
    function check(name,yes),checks(end+1)=struct('name',char(name),'pass',logical(yes));assert(yes,'test:ActualWind','%s',name);end
    function reject(name,f),id2='';try,f();catch ex,id2=ex.identifier;end;check(name,~isempty(id2));end
    function out=send(v),sends=sends+1;[lastBytes,lastFields]=gpenmpcTaskIo.encodePlantEnvironmentV2(v,1,2.21);out=struct('sent_s',clockS,'generation',v.generation,'payload_generation',v.payload_generation,'service_release_generation',v.service_release_generation,'bytes',numel(lastBytes));end
    function deny(varargin),forbidden=forbidden+1;error('test:Forbidden','Not an environment call');end
    function d=ackAt(t)
        terrain=zeros(32,1);terrain([3,5,7,26])=[t;1;1;1];
        ack=struct('valid',true,'task_env_failed',false,'failure_code',0,'session_token',26090501,'applied_frame_generation',1,'applied_payload_generation',0,'actual_payload_kg',2.21,'actual_total_mass_kg',11.71);
        p=m600check.encodeCopterSimDeliveryDiagnostics(terrain,ack,false);raw=[typecast(int32([1234567890,1]),'uint8'),typecast(p(:).','uint8')];
        decoded=m600check.decodeCopterSimDeliveryDiagnostics(raw,1,NaN,contract);assert(decoded.packet_valid);
        d=struct('present',true,'mass_ack_valid',decoded.environment_extension.mass_ack_valid,'decoded',decoded);
    end
end
function b=readbin(p),f=fopen(p,'rb');assert(f>=0);c=onCleanup(@()fclose(f));b=fread(f,Inf,'*uint8');end
function h=shaFile(p),m=java.security.MessageDigest.getInstance('SHA-256');m.update(typecast(readbin(p),'int8'));h=string(upper(reshape(dec2hex(typecast(m.digest(),'uint8'),2).',1,[])));end
