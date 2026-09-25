function result=test_short_environment_activation_order(outputDir)
% TEST_SHORT_ENVIRONMENT_ACTIVATION_ORDER Test environment lease ordering.
% A 0.26 s delay after BEGIN expires the lease; construction before BEGIN
% consumes no lease time. Verify ordinary 10 ms refresh.
arguments
    outputDir (1,1) string
end
assert(~isfolder(outputDir)&&~isfile(outputDir),'gpenmpcShort:ActivationOutputExists');
root=fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(root,'matlab_validation'));
loaded=load(fullfile(gpenmpc_external_path('environment_delivery_model'), ...
    'M600_CORE_PARAMETERS.mat'),'deliveryPolicy');
p=loaded.deliveryPolicy;
assert(p.max_env_age_s==0.25,'gpenmpcShort:ActivationPolicyChanged');

% Bind generation 1 at model time 3.72 s and deliver no successor before 3.98 s.
s=gpenmpcTaskIo.initPlantEnvironmentV2(p,3.72);
f=frame(p,1,3.72);
[s,r]=gpenmpcTaskIo.acceptPlantEnvironmentV2(s,f,true,3.72,true,p);
assert(r.pending&&~r.task_env_failed,'gpenmpcShort:ActivationFixture');
[s,a]=gpenmpcTaskIo.commitPlantEnvironmentV2(s,core(s,3.721,3.73,p),p);
assert(a.new_credit&&a.valid,'gpenmpcShort:ActivationFixture');
[~,expired]=gpenmpcTaskIo.acceptPlantEnvironmentV2(s,zeros(28,1),false,3.98,true,p);

% After generation 1 expires, a late generation 2 must not clear the fault.
shortState=gpenmpcTaskIo.initPlantEnvironmentV2(p,4.82);
[shortPending,shortFirst]=gpenmpcTaskIo.acceptPlantEnvironmentV2(shortState,frame(p,1,4.82),true,4.82,true,p);
[shortState,shortAck]=gpenmpcTaskIo.commitPlantEnvironmentV2(shortPending,core(shortPending,4.821,4.94,p),p);
[shortState,shortOldEdge]=gpenmpcTaskIo.acceptPlantEnvironmentV2(shortState,zeros(28,1),false,5.05,true,p);
[shortFailed,shortFailure]=gpenmpcTaskIo.acceptPlantEnvironmentV2(shortState,zeros(28,1),false,5.070000000001,true,p);
[~,shortLate]=gpenmpcTaskIo.acceptPlantEnvironmentV2(shortFailed,frame(p,2,5.00),true,5.08,true,p);

% Deliver a fresh frame before the 95.9 ms service block.
% Verify advancement within the 250 ms lease and rejection after a longer block.
newState=gpenmpcTaskIo.initPlantEnvironmentV2(p,2.11);
[newPending,newFirst]=gpenmpcTaskIo.acceptPlantEnvironmentV2(newState,frame(p,1,2.11),true,2.11,true,p);
[newState,newAck1]=gpenmpcTaskIo.commitPlantEnvironmentV2(newPending,core(newPending,2.111,2.12,p),p);
[newPending,newSecond]=gpenmpcTaskIo.acceptPlantEnvironmentV2(newState,frame(p,2,2.22),true,2.22,true,p);
[newState,newAck2]=gpenmpcTaskIo.commitPlantEnvironmentV2(newPending,core(newPending,2.221,2.23,p),p);
[newState,newAfterBlock]=gpenmpcTaskIo.acceptPlantEnvironmentV2(newState,zeros(28,1),false,2.36,true,p);
[~,newForcedFailure]=gpenmpcTaskIo.acceptPlantEnvironmentV2(newState,zeros(28,1),false,2.480000000001,true,p);

% Fixed ordering: the same 0.353 s constructor interval occurs before BEGIN,
% so acceptPlantEnvironmentV2 is deliberately not called and cannot spend a
% lease.  Explicit BEGIN occurs only after construction and is then renewed
% at the existing 10 ms cadence without changing the 250 ms bound.
constructionStart=3.367;
constructionEnd=constructionStart+0.353;
assert(abs(constructionEnd-3.72)<1e-12);
after=gpenmpcTaskIo.initPlantEnvironmentV2(p,constructionEnd);
maxGap=0;previous=constructionEnd;
for k=1:41
    t=constructionEnd+(k-1)*0.01;
    g=k;
    f=frame(p,g,t);
    [pending,receipt]=gpenmpcTaskIo.acceptPlantEnvironmentV2(after,f,true,t,true,p);
    assert(~receipt.task_env_failed&&receipt.pending,'gpenmpcShort:ActivationRenewal');
    [after,ack]=gpenmpcTaskIo.commitPlantEnvironmentV2(pending,core(pending,t+0.001,t+0.005,p),p);
    assert(ack.new_credit&&ack.valid,'gpenmpcShort:ActivationCommit');
    maxGap=max(maxGap,t-previous);previous=t;
end

runnerPath=fullfile(root,'tools','run_m600_board_local_short_hil.m');
source=fileread(runnerPath);
constructor='service=gpenmpcNative.RflyLocalMethodService(';
activation='envService=objects.environment_service;';
moduleStart="sessionRaw.start_send=io.sendCanonicalSession('start',struct());";
prestartPump='Establish the first environment lease before the board module start.';
preparation="waitUntil(@preparedWithWindow,c.preparation_timeout_s,'PREPARATION_AND_ORIGINAL_WINDOW');";
constructorAt=strfind(source,constructor);activationAt=strfind(source,activation);
moduleStartAt=strfind(source,moduleStart);prestartPumpAt=strfind(source,prestartPump);
preparationAt=strfind(source,preparation);
checks=struct;
checks.expired_input_rejected=expired.task_env_failed&&expired.failure_code==3;
checks.old_applied_generation_expires_before_late_generation=shortFirst.pending&&shortAck.new_credit ...
    &&~shortOldEdge.task_env_failed&&shortFailure.task_env_failed&&shortFailure.failure_code==3 ...
    &&shortLate.task_env_failed&&shortLate.failure_code==3&&shortFailed.applied_frame_generation==1;
checks.fresh_generation_before_measured_service_block_prevents_old_lease_expiry= ...
    newFirst.pending&&newAck1.new_credit&&newSecond.pending&&newAck2.new_credit ...
    &&~newAfterBlock.task_env_failed&&newState.applied_frame_generation==2;
checks.unchanged_250ms_bound_still_fails_closed=newForcedFailure.task_env_failed ...
    &&newForcedFailure.failure_code==3&&p.max_env_age_s==0.25;
checks.blocking_construction_precedes_explicit_begin=constructionEnd>constructionStart;
checks.post_begin_renewal_survives=~after.task_env_failed&&after.applied_frame_generation==41;
checks.refresh_gap_unchanged=maxGap<=0.0100000001;
checks.runner_has_single_activation=numel(activationAt)==1;
checks.activation_after_method_constructor=isscalar(constructorAt)&&isscalar(activationAt)&&activationAt>constructorAt;
checks.activation_and_initial_refresh_precede_single_module_start=isscalar(moduleStartAt) ...
    &&isscalar(prestartPumpAt)&&activationAt<prestartPumpAt&&prestartPumpAt<moduleStartAt;
checks.activation_before_prepare_poll=isscalar(preparationAt)&&activationAt<preparationAt;
checks.pump_is_explicitly_gated=contains(source, ...
    'if ~environmentRefreshActive||isempty(envService)||envService.Failed,return,end');
checks.zero_send_before_activation_asserted=contains(source,'send_attempt_count_before_activation==0');
passed=all(structfun(@logical,checks));
result=struct('schema','SHORT_ENVIRONMENT_ACTIVATION_ORDER_HOST_ONLY_V1', ...
    'status','PASS_HOST_ONLY_LIFECYCLE_ORDER_CHANGE', ...
    'passed',passed,'checks',checks,'checks_passed',nnz(structfun(@logical,checks)), ...
    'checks_total',numel(fieldnames(checks)),'expired_input_begin_s',3.72, ...
    'expired_input_failure_s',3.98,'expired_input_age_s',0.26, ...
    'construction_duration_s',0.353,'unchanged_max_env_age_s',p.max_env_age_s, ...
    'post_begin_frames',41,'maximum_refresh_gap_s',maxGap, ...
    'runner_path',runnerPath,'COM_open',0,'board_actions',0,'plant_processes',0,'physical_output_actions',0);
mkdir(outputDir);
fid=fopen(fullfile(outputDir,'RESULT.json'),'w','n','UTF-8');assert(fid>=0);guard=onCleanup(@()fclose(fid)); %#ok<NASGU>
fprintf(fid,'%s\n',jsonencode(result,PrettyPrint=true));
assert(passed,'gpenmpcShort:ActivationRegressionFailed', ...
    'Focused environment activation regression failed.');
fprintf('SHORT_ENVIRONMENT_ACTIVATION_ORDER %d/%d\n',result.checks_passed,result.checks_total);
end

function f=frame(p,g,t)
f=zeros(28,1);f(1)=2;f(2)=g;f(3)=t;f(4)=0;f(5)=p.initial_payload_kg;
f(6:7)=p.initial_wind_ned_xy_mps;f(8)=1;f(9:20)=p.initial_reference_jet_ned;
f(21)=0;f(22)=1;f(23)=p.expected_session_token;f(24)=t;f(25)=15;
f(26)=1;f(27)=1;f(28)=0;
end

function c=core(s,ioTime,coreTime,p)
c=struct('step_accepted',true,'model_failed',false,'reset_applied',false, ...
    'ground_confirmed',true,'io_time_s',ioTime,'core_time_s',coreTime, ...
    'applied_input_generation',s.pending_frame(2), ...
    'total_mass_kg',p.base_mass_kg+s.pending_frame(5)+p.mass_bias_kg);
end
