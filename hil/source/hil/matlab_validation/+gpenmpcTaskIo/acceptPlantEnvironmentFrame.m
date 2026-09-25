function [next,r]=acceptPlantEnvironmentFrame(previous,frame,currentIoTimeS,groundConfirmed,board,policy)
% Environment acceptance state machine.
% On failure, task permission latches off and the last accepted environment
% is retained. The caller continues plant and sensor updates during landing.
%
% frame is [] or the 28 doubles decoded from a validated 232-byte envelope.
% The receiver supplies source and session validation.
% See encodePlantEnvironment.m for wire order, NED units and schema.
%
% board is an independently validated observation:
%   source_identity_valid, clock_map_valid, valid: logical scalars;
%   armed: -1 unknown, 0 disarmed, 1 armed;
%   mapped_io_time_s: board observation time independently mapped to DLL I/O
%                    clock with a bounded clock-map uncertainty.
% Payload unloading requires a board-observation interface in the DLL.
% groundConfirmed comes from the current plant ground diagnostic and controls
% permission for simulated payload service.
%
% Caller timing and environment settings:
% initial_payload_kg, initial_wind_ned_xy_mps(2), initial_reference_jet_ned(12),
% max_env_age_s, max_board_age_s, max_future_skew_s, unload_dwell_s,
% max_ground_sample_gap_s. Dwell must be positive. Call on each outer/core
% observation using its measured timestamp.
validatePolicy(policy);
if isempty(previous)
    initialTime=0; % Placeholder only for an immediately rejected invalid clock.
    if finiteScalar(currentIoTimeS)&&currentIoTimeS>=0,initialTime=double(currentIoTimeS);end
    next=struct('environment',struct('reference_jet_ned',reshape(double(policy.initial_reference_jet_ned),12,1), ...
        'payload_kg',double(policy.initial_payload_kg),'wind_xy_mps',reshape(double(policy.initial_wind_ned_xy_mps),2,1)), ...
        'initialized_io_time_s',initialTime,'last_eval_io_time_s',initialTime, ...
        'has_accepted_frame',false,'last_frame',zeros(28,1),'generation',0, ...
        'source_io_time_s',0,'task_reference_time_s',0,'payload_generation',0, ...
        'mission_phase',0,'task_clock_paused',true,'ground_disarmed_since_s',-1, ...
        'task_env_failed',false,'failure_code',uint8(0),'accepted_frame_count',0);
else
    next=previous;
end
if next.task_env_failed
    r=receipt(next,currentIoTimeS,false,false);return
end
if ~finiteScalar(currentIoTimeS)||currentIoTimeS<0||currentIoTimeS<next.last_eval_io_time_s
    next=fail(next,1);r=receipt(next,currentIoTimeS,false,false);return
end
if ~islogical(groundConfirmed)||~isscalar(groundConfirmed)
    next=fail(next,2);r=receipt(next,currentIoTimeS,false,false);return
end
boardFresh=boardEligible(board,currentIoTimeS,policy);
gap=currentIoTimeS-next.last_eval_io_time_s;
next.last_eval_io_time_s=currentIoTimeS;
if groundConfirmed && boardFresh
    if next.ground_disarmed_since_s<0 || gap>policy.max_ground_sample_gap_s
        next.ground_disarmed_since_s=currentIoTimeS;
    end
else
    next.ground_disarmed_since_s=-1;
end
% Deadline precedes acceptance: a packet arriving after expiry cannot wash
% away the missing window. New run initialization is an OUTER explicit act.
if next.has_accepted_frame
    age=currentIoTimeS-next.source_io_time_s;
else
    age=currentIoTimeS-next.initialized_io_time_s;
end
if age>policy.max_env_age_s
    next=fail(next,3);r=receipt(next,currentIoTimeS,false,boardFresh);return
end
if isempty(frame)
    r=receipt(next,currentIoTimeS,false,boardFresh);return
end
if ~isnumeric(frame)||~isreal(frame)||numel(frame)~=28||any(~isfinite(frame(:)))
    next=fail(next,4);r=receipt(next,currentIoTimeS,false,boardFresh);return
end
f=reshape(double(frame),28,1);
if f(1)~=1||any(f(23:28)~=0)||~integerIn(f(2),1,2^32-1)|| ...
        ~integerIn(f(8),0,6)||~integerIn(f(21),0,2^32-1)||~integerIn(f(22),0,1)|| ...
        any(f(3:5)<0)||f(5)>policy.initial_payload_kg
    next=fail(next,5);r=receipt(next,currentIoTimeS,false,boardFresh);return
end
if currentIoTimeS-f(3)>policy.max_env_age_s||f(3)-currentIoTimeS>policy.max_future_skew_s
    next=fail(next,6);r=receipt(next,currentIoTimeS,false,boardFresh);return
end
if next.has_accepted_frame && f(2)==next.generation
    if ~isequal(f,next.last_frame),next=fail(next,7);end
    % Exact duplicate is a held input, never fresh generation/time credit.
    r=receipt(next,currentIoTimeS,false,boardFresh);return
end
if next.has_accepted_frame && (f(2)<next.generation||f(3)<=next.source_io_time_s||f(4)<next.task_reference_time_s)
    next=fail(next,8);r=receipt(next,currentIoTimeS,false,boardFresh);return
end
oldPayload=next.environment.payload_kg;
if f(5)>oldPayload
    next=fail(next,9);r=receipt(next,currentIoTimeS,false,boardFresh);return
end
if f(5)<oldPayload
    dwellSatisfied=next.ground_disarmed_since_s>=0 && ...
        currentIoTimeS-next.ground_disarmed_since_s>=policy.unload_dwell_s;
    if ~next.has_accepted_frame||~groundConfirmed||~boardFresh||~dwellSatisfied|| ...
            f(21)~=next.payload_generation+1
        next=fail(next,10);r=receipt(next,currentIoTimeS,false,boardFresh);return
    end
elseif f(21)~=next.payload_generation
    next=fail(next,11);r=receipt(next,currentIoTimeS,false,boardFresh);return
end
next.environment.reference_jet_ned=f(9:20);next.environment.payload_kg=f(5);
next.environment.wind_xy_mps=f(6:7);next.last_frame=f;
next.generation=f(2);next.source_io_time_s=f(3);next.task_reference_time_s=f(4);
next.payload_generation=f(21);next.mission_phase=f(8);next.task_clock_paused=logical(f(22));
next.has_accepted_frame=true;next.accepted_frame_count=next.accepted_frame_count+1;
r=receipt(next,currentIoTimeS,true,boardFresh);
end
function tf=boardEligible(b,t,p)
names={'source_identity_valid','clock_map_valid','valid','armed','mapped_io_time_s'};
tf=isstruct(b)&&isscalar(b)&&all(isfield(b,names));if ~tf,return,end
for name={'source_identity_valid','clock_map_valid','valid'}
    v=b.(name{1});tf=tf&&islogical(v)&&isscalar(v)&&v;
end
tf=tf&&finiteScalar(b.armed)&&b.armed==0&&finiteScalar(b.mapped_io_time_s)&& ...
    b.mapped_io_time_s>=0&&t-b.mapped_io_time_s<=p.max_board_age_s&& ...
    b.mapped_io_time_s-t<=p.max_future_skew_s;
end
function s=fail(s,code)
s.task_env_failed=true;s.failure_code=uint8(code);
end
function r=receipt(s,t,accepted,boardFresh)
age=NaN;if finiteScalar(t)&&s.has_accepted_frame,age=t-s.source_io_time_s;end
r=struct('accepted_new_frame',accepted,'task_may_continue',s.has_accepted_frame&&~s.task_env_failed, ...
    'task_env_failed',s.task_env_failed,'failure_code',s.failure_code, ...
    'last_accepted_generation',s.generation,'source_age_s',age, ...
    'board_evidence_eligible_for_unload',boardFresh, ...
    'continue_existing_plant_with_last_accepted_environment',true, ...
    'plant_step_or_reset_performed',false,'live_board_evidence_transport_implemented',false);
% failure_code: 1 I/O clock; 2 ground observation schema; 3 elapsed deadline;
% 4 nonfinite/shape; 5 version/reserved/range; 6 source time; 7 changed duplicate;
% 8 sequence/source/task-clock regression; 9 mass increase; 10 unload permission;
% 11 payload-generation inconsistency. A failure never changes environment.
end
function validatePolicy(p)
names={'initial_payload_kg','initial_wind_ned_xy_mps','initial_reference_jet_ned', ...
    'max_env_age_s','max_board_age_s','max_future_skew_s','unload_dwell_s','max_ground_sample_gap_s'};
assert(isstruct(p)&&isscalar(p)&&all(isfield(p,names)),'gpenmpcTaskIo:Policy','Explicit complete policy required.');
for name={'initial_payload_kg','max_future_skew_s'}
    v=p.(name{1});assert(finiteScalar(v)&&v>=0,'gpenmpcTaskIo:Policy','Invalid policy value.');
end
for name={'max_env_age_s','max_board_age_s','unload_dwell_s','max_ground_sample_gap_s'}
    v=p.(name{1});assert(finiteScalar(v)&&v>0,'gpenmpcTaskIo:Policy','Positive timing policy required.');
end
assert(isnumeric(p.initial_wind_ned_xy_mps)&&isreal(p.initial_wind_ned_xy_mps)&& ...
    numel(p.initial_wind_ned_xy_mps)==2&&all(isfinite(p.initial_wind_ned_xy_mps(:))));
assert(isnumeric(p.initial_reference_jet_ned)&&isreal(p.initial_reference_jet_ned)&& ...
    numel(p.initial_reference_jet_ned)==12&&all(isfinite(p.initial_reference_jet_ned(:))));
end
function tf=finiteScalar(v),tf=isnumeric(v)&&isreal(v)&&isscalar(v)&&isfinite(v);end
function tf=integerIn(v,a,b),tf=v==fix(v)&&v>=a&&v<=b;end
