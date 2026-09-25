function [s,r]=acceptPlantEnvironmentV2(previous,frame,hasFrame,nowS,plantGround,p)
%#codegen
% Fixed-state pure staging, NOT plant execution or an applied-mass ACK.
% Initialize via initPlantEnvironmentV2. Call each real 10ms core observation.
% frame is a validated-owner/ID 28x1 double snapshot; hasFrame=false is no
% datagram (pass zeros(28,1)). The owner validates BOTH independent PX4
% heartbeat/EXTENDED_SYS_STATE, maps their receive times, and journals binding.
% An environment fault prohibits task advancement but the caller continues
% SAME plant/HIL with s.environment. Never map this fault into core reset or
% its numerical/terrain-failure latch. No state/time dynamics live here.
s=previous;
% Check immutable scalar policy through the fixed-size constructor. Its
% result is unused; this never replaces the caller's persistent state.
gpenmpcTaskIo.initPlantEnvironmentV2(p,previous.initialized_io_time_s);
if s.task_env_failed,r=receipt(s,nowS,false,false);return;end
if ~finiteScalar(nowS)||nowS<0||nowS<s.last_eval_io_time_s||nowS<s.last_commit_io_time_s
    s=fail(s,1);r=receipt(s,nowS,false,false);return;
end
if ~islogical(hasFrame)||~isscalar(hasFrame)||~islogical(plantGround)||~isscalar(plantGround)
    s=fail(s,2);r=receipt(s,nowS,false,false);return;
end
gap=nowS-s.last_eval_io_time_s;s.last_eval_io_time_s=nowS;
if p.expected_session_token~=s.expected_session_token
    s=fail(s,12);r=receipt(s,nowS,false,false);return;
end
age=nowS-s.initialized_io_time_s;
if s.has_applied_frame,age=nowS-s.applied_source_io_time_s;end
if age>p.max_env_age_s
    s=fail(s,3);r=receipt(s,nowS,false,false);return;
end
if s.pending_valid&&nowS-s.pending_io_time_s>p.max_commit_delay_s
    s=fail(s,14);r=receipt(s,nowS,false,false);return;
end
freshCandidate=false;f=zeros(28,1);
if hasFrame
    if ~isa(frame,'double')||~isreal(frame)||~isequal(size(frame),[28,1])||any(~isfinite(frame(:)))
        s=fail(s,4);r=receipt(s,nowS,false,false);return;
    end
    f=frame;
    if f(1)~=2||~integerIn(f(2),1,2^32-1)||~integerIn(f(8),0,6)||~integerIn(f(21),0,4)||~integerIn(f(22),0,1)|| ...
            any(f(3:5)<0)||f(5)>p.initial_payload_kg||~integerIn(f(23),1,flintmax)||f(24)<0|| ...
            ~integerIn(f(25),0,31)||~integerIn(f(26),0,3)||~integerIn(f(27),1,2^32-1)||~integerIn(f(28),0,4)
        s=fail(s,5);r=receipt(s,nowS,false,false);return;
    end
    if f(23)~=s.expected_session_token
        s=fail(s,12);r=receipt(s,nowS,false,false);return;
    end
    if nowS-f(3)>p.max_env_age_s||f(3)-nowS>p.max_future_skew_s|| ...
            nowS-f(24)>p.max_board_age_s||f(24)-nowS>p.max_future_skew_s
        s=fail(s,6);r=receipt(s,nowS,false,false);return;
    end
    if bitand(uint32(f(25)),uint32(15))~=uint32(15)
        s=fail(s,13);r=receipt(s,nowS,false,false);return;
    end
    if s.pending_valid
        if ~isequal(f,s.pending_frame),s=fail(s,14);end
        % Exact pending duplicate may be observed, never stage again.
    elseif s.has_applied_frame&&f(2)==s.applied_frame_generation
        if ~isequal(f,s.applied_frame),s=fail(s,7);end
    else
        if s.has_applied_frame&&(f(2)<s.applied_frame_generation||f(3)<=s.applied_source_io_time_s||f(4)<s.applied_task_time_s)
            s=fail(s,8);
        elseif s.has_board_frame&&(f(27)<s.continuity_epoch||f(24)<s.board_frame(24))
            s=fail(s,8);
        else
            freshCandidate=true;
            if ~s.has_board_frame||f(27)~=s.continuity_epoch,s.ground_disarmed_since_s=-1;end
            s.board_frame=f;s.has_board_frame=true;s.continuity_epoch=f(27);
        end
    end
    if s.task_env_failed,r=receipt(s,nowS,false,false);return;end
end
boardGood=false;
if s.has_board_frame
    b=s.board_frame;
    if nowS-b(24)>p.max_board_age_s||b(24)-nowS>p.max_future_skew_s
        s=fail(s,6);r=receipt(s,nowS,false,false);return;
    end
    boardGood=b(25)==15&&b(26)==1;
end
if plantGround&&boardGood
    if s.ground_disarmed_since_s<0||gap>p.max_ground_sample_gap_s,s.ground_disarmed_since_s=nowS;end
else
    s.ground_disarmed_since_s=-1;
end
if ~freshCandidate,r=receipt(s,nowS,false,boardGood);return;end
oldPayload=s.environment.payload_kg;
if f(5)>oldPayload,s=fail(s,9);r=receipt(s,nowS,false,boardGood);return;end
unload=f(5)<oldPayload;
if unload
    nextGen=s.applied_payload_generation+1;
    allowed=s.has_applied_frame&&nextGen<=4&&f(21)==nextGen&&f(28)==nextGen&&f(22)==1&& ...
        plantGround&&boardGood&&s.ground_disarmed_since_s>=0&&nowS-s.ground_disarmed_since_s>=8.0;
    if allowed,allowed=f(5)==p.service_payload_targets_kg(nextGen);end
    if ~allowed,s=fail(s,10);r=receipt(s,nowS,false,boardGood);return;end
elseif f(21)~=s.applied_payload_generation||~(f(28)==0||f(28)==s.applied_payload_generation)
    s=fail(s,11);r=receipt(s,nowS,false,boardGood);return;
end
s.pending_valid=true;s.pending_frame=f;s.pending_io_time_s=nowS;s.pending_unload=unload;
s.staged_count=s.staged_count+uint32(1);
r=receipt(s,nowS,true,boardGood);
end
function s=fail(s,code)
s.task_env_failed=true;s.failure_code=uint8(code);s.pending_valid=false;s.ground_disarmed_since_s=-1;
end
function r=receipt(s,t,staged,boardGood)
stepEnvironment=s.environment;
if s.pending_valid&&~s.task_env_failed
    stepEnvironment.reference_jet_ned=s.pending_frame(9:20);stepEnvironment.payload_kg=s.pending_frame(5);stepEnvironment.wind_xy_mps=s.pending_frame(6:7);
end
dwell=0;if finiteScalar(t)&&s.ground_disarmed_since_s>=0,dwell=max(0,t-s.ground_disarmed_since_s);end
r=struct('staged_this_call',staged,'pending',s.pending_valid,'applied_ack_this_call',false, ...
    'environment_for_step',stepEnvironment,'task_may_continue',s.has_applied_frame&&~s.task_env_failed, ...
    'task_env_failed',s.task_env_failed,'failure_code',s.failure_code,'board_ground_disarmed_fresh',boardGood, ...
    'continuous_ground_dwell_s',dwell,'continue_same_plant',true,'plant_step_performed',false,'plant_reset_requested',false, ...
    'applied_frame_generation',s.applied_frame_generation,'applied_payload_generation',s.applied_payload_generation);
end
% Codes: 1 clock,2 flag/ground schema,3 elapsed environment deadline,
% 4 frame shape/nonfinite,5 ranges/schema,6 source/board stale/future,
% 7 changed duplicate,8 sequence/time/continuity regression,9 mass increase,
% 10 release/8s proof/target,11 payload-generation contradiction,
% 12 session/boot/clock-map identity,13 invalid board evidence,14 pending.
function tf=finiteScalar(v),tf=isnumeric(v)&&isreal(v)&&isscalar(v)&&isfinite(v);end
function tf=integerIn(v,a,b),tf=finiteScalar(v)&&v==fix(v)&&v>=a&&v<=b;end
