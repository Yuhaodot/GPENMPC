function [y,d,ack,envReceipt]=copterSimDeliveryCore( ...
    controls,reset,pos0,ang0,terrain15,frame28,currentIoTimeS,initialEnvironment,p,policy)
%#codegen
% One persistent M600 plant, one existing flat-terrain core call per 10ms.
% No socket, file, controller, transport owner, model instance, or task runner.
% reset=true is ONLY an outer-authorized NEW physical instance. It is never
% synthesized by BEGIN, an environment error, a mass change, or an ACK.
%
% Before the first nonzero input, no ENV liveness timer runs. A first packet
% must be an explicit full-payload, paused, task-time-zero V2 BEGIN. Its
% prebound session token refers to the single validated MAVLink owner. BEGIN
% starts the ENV lifetime at currentIoTimeS WITHOUT resetting the physical
% state/time. Any invalid first packet latches a fault and cannot retry BEGIN.
%
% currentIoTimeS is the proved DLL I/O clock domain of frame times, not an
% inferred offset or arbitrary raw PX4 uptime. frame28 is an atomic snapshot.
% Last actual core ground is used before the step; post-step ground and mass
% must also validate before an unloading ACK is committed.
assert(islogical(reset)&&isscalar(reset),'m600check:DeliveryReset','Explicit logical new-instance reset required.');
assert(isequal(size(initialEnvironment.reference_jet_ned),[12,1])&& ...
    isequal(size(initialEnvironment.wind_xy_mps),[2,1]),'m600check:DeliveryEnvironment','Fixed initial environment required.');
persistent initialized envState sessionBegun beginAttempted lastGround lastIoTime beginCount
if isempty(initialized)||reset
    initialTime=0.0;if finiteScalar(currentIoTimeS)&&currentIoTimeS>=0,initialTime=double(currentIoTimeS);end
    envState=gpenmpcTaskIo.initPlantEnvironmentV2(policy,initialTime);
    assert(isequal(initialEnvironment.reference_jet_ned,policy.initial_reference_jet_ned)&& ...
        isequal(initialEnvironment.wind_xy_mps,policy.initial_wind_ned_xy_mps)&& ...
        initialEnvironment.payload_kg==policy.initial_payload_kg,'m600check:DeliveryEnvironment','Initial environment/policy differ.');
    initialized=true;sessionBegun=false;beginAttempted=false;lastGround=false;
    lastIoTime=initialTime;beginCount=uint32(0);
end
stageReceipt=blankReceipt(envState);
if ~envState.task_env_failed
    if ~finiteScalar(currentIoTimeS)||currentIoTimeS<0||currentIoTimeS<lastIoTime
        envState=failEnvironment(envState,uint8(1));
    else
        lastIoTime=currentIoTimeS;
        shapeGood=isa(frame28,'double')&&isreal(frame28)&&isequal(size(frame28),[28,1]);
        zeroInput=false;if shapeGood,zeroInput=all(frame28==0);end
        if ~sessionBegun
            if ~zeroInput
                beginAttempted=true;beginCount=beginCount+uint32(1);
                if ~shapeGood||any(~isfinite(frame28(:)))
                    envState=failEnvironment(envState,uint8(4));
                elseif frame28(23)~=policy.expected_session_token
                    envState=failEnvironment(envState,uint8(12));
                elseif frame28(1)~=2||frame28(2)~=1||frame28(4)~=0|| ...
                        frame28(5)~=initialEnvironment.payload_kg||frame28(21)~=0|| ...
                        frame28(22)~=1||frame28(25)~=15||frame28(26)~=1||frame28(28)~=0
                    envState=failEnvironment(envState,uint8(5));
                else
                    % Only this first legal BEGIN constructs session state.
                    % Physical core invocation below receives the ORIGINAL
                    % reset argument; this is never a physical restart.
                    envState=gpenmpcTaskIo.initPlantEnvironmentV2(policy,currentIoTimeS);
                    [envState,stageReceipt]=gpenmpcTaskIo.acceptPlantEnvironmentV2( ...
                        envState,frame28,true,currentIoTimeS,lastGround,policy);
                    sessionBegun=~envState.task_env_failed;
                end
            end
        else
            [envState,stageReceipt]=gpenmpcTaskIo.acceptPlantEnvironmentV2( ...
                envState,frame28,~zeroInput,currentIoTimeS,lastGround,policy);
        end
    end
end
environmentForStep=envState.environment;
if envState.pending_valid&&~envState.task_env_failed
    environmentForStep.reference_jet_ned=envState.pending_frame(9:20);
    environmentForStep.payload_kg=envState.pending_frame(5);
    environmentForStep.wind_xy_mps=envState.pending_frame(6:7);
end
% The ONLY physical core invocation. Environment faults do not suppress it.
[y,d]=m600check.copterSimFlatTerrainCore(controls,reset,pos0,ang0,environmentForStep,terrain15,p);
ack=heldAck(envState);
if envState.pending_valid&&~d.reset_applied
    actual=struct('step_accepted',logical(d.step_accepted),'model_failed',logical(d.failed), ...
        'reset_applied',logical(d.reset_applied),'ground_confirmed',logical(d.ground_confirmed), ...
        'io_time_s',double(currentIoTimeS),'core_time_s',double(d.sim_time_s), ...
        'applied_input_generation',envState.pending_frame(2),'total_mass_kg',double(y.mass_kg));
    [envState,ack]=gpenmpcTaskIo.commitPlantEnvironmentV2(envState,actual,policy);
end
if ~beginAttempted&&~envState.task_env_failed
    % Explicit unbound pre-session observation; codec maps invalid ACK to16.
    % Its mass is an actual current-plant observation, NOT an applied command.
    ack.session_token=0;ack.actual_payload_kg=initialEnvironment.payload_kg;
    ack.actual_total_mass_kg=y.mass_kg;ack.valid=false;
end
lastGround=logical(d.ground_confirmed&&d.observation_valid&&~d.failed);
statusCode=uint8(0);
if envState.task_env_failed,statusCode=envState.failure_code;
elseif ~ack.valid,statusCode=uint8(16);
elseif envState.pending_valid,statusCode=uint8(17);
end
envReceipt=struct( ...
    'staged_this_call',stageReceipt.staged_this_call, ...
    'pending',envState.pending_valid,'applied_ack_this_call',ack.new_credit, ...
    'environment_for_step',environmentForStep, ...
    'task_may_continue',envState.has_applied_frame&&~envState.task_env_failed&&d.observation_valid&&~d.failed, ...
    'task_env_failed',envState.task_env_failed,'failure_code',envState.failure_code, ...
    'board_ground_disarmed_fresh',stageReceipt.board_ground_disarmed_fresh, ...
    'continuous_ground_dwell_s',stageReceipt.continuous_ground_dwell_s, ...
    'continue_same_plant',true,'plant_step_performed',true,'plant_reset_requested',false, ...
    'applied_frame_generation',envState.applied_frame_generation, ...
    'applied_payload_generation',envState.applied_payload_generation, ...
    'session_begun',sessionBegun,'begin_attempted',beginAttempted,'begin_count',beginCount, ...
    'core_calls_this_invocation',uint8(1),'core_time_s',d.sim_time_s, ...
    'last_evaluated_io_time_s',lastIoTime,'environment_status_code',statusCode);
end
function s=failEnvironment(s,code)
s.task_env_failed=true;s.failure_code=code;s.pending_valid=false;s.ground_disarmed_since_s=-1;
end
function r=blankReceipt(s)
r=struct('staged_this_call',false,'pending',s.pending_valid,'applied_ack_this_call',false, ...
    'environment_for_step',s.environment,'task_may_continue',false,'task_env_failed',s.task_env_failed, ...
    'failure_code',s.failure_code,'board_ground_disarmed_fresh',false,'continuous_ground_dwell_s',0.0, ...
    'continue_same_plant',true,'plant_step_performed',false,'plant_reset_requested',false, ...
    'applied_frame_generation',s.applied_frame_generation,'applied_payload_generation',s.applied_payload_generation);
end
function a=heldAck(s)
a=struct('new_credit',false,'valid',s.has_applied_frame&&~s.task_env_failed, ...
    'session_token',s.expected_session_token,'applied_frame_generation',s.applied_frame_generation, ...
    'applied_payload_generation',s.applied_payload_generation,'actual_payload_kg',s.environment.payload_kg, ...
    'actual_total_mass_kg',s.applied_total_mass_kg,'applied_core_time_s',s.last_commit_core_time_s, ...
    'task_env_failed',s.task_env_failed,'failure_code',s.failure_code,'unload_count',s.unload_count, ...
    'continue_same_plant',true,'plant_step_performed',false,'plant_reset_requested',false);
end
function tf=finiteScalar(v),tf=isnumeric(v)&&isreal(v)&&isscalar(v)&&isfinite(v);end
