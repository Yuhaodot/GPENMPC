function [sample,state]=decodeCanonicalRotorObserver(bytes,expected,clock,state)
% Pure decoder/receiver guard for encodeCanonicalRotorObserver, no I/O.
% expected: uint64 session_token, uint8[32,1] dll_sha256,
%   maximum_source_age_s, maximum_receive_age_s (explicit caller bounds).
% clock: now_sim_time_s from the SAME model session's independent clock,
%   now_wall_time_s and receive_wall_time_s from one monotonic host clock.
% Empty bytes polls the last accepted sample without renewing its rx time.
% Exact duplicate is ignored without renewing rx time/generation; a conflict,
% gap, regression, identity/time/nonfinite/negative-state error latches failure.
% State is retained through failures. Only an explicit new-session caller may
% initialize []; never reset a failed state to wash out faults in one session.
% Digest comparison binds source bytes; runtime origin and authentication
% require separate validation.
if nargin<4||isempty(state),state=initial();end
sample=emptySample();
if ~validExpected(expected)||~validClock(clock)
    state=latch(state,'INVALID_EXPECTED_OR_CLOCK');sample.reason=state.failure_reason;return
end
if state.failed,sample.reason=state.failure_reason;return,end
if ~state.context_bound
    state.context_bound=true;state.session_token=expected.session_token;state.dll_sha256=expected.dll_sha256;
    state.maximum_source_age_s=expected.maximum_source_age_s;
    state.maximum_receive_age_s=expected.maximum_receive_age_s;
elseif state.session_token~=expected.session_token||~isequal(state.dll_sha256,expected.dll_sha256) ...
        ||state.maximum_source_age_s~=expected.maximum_source_age_s ...
        ||state.maximum_receive_age_s~=expected.maximum_receive_age_s
    state=latch(state,'EXPECTED_CONTEXT_CHANGED');sample.reason=state.failure_reason;return
end
if clock.now_wall_time_s<state.last_call_wall_time_s
    state=latch(state,'HOST_CLOCK_REVERSED');sample.reason=state.failure_reason;return
end
if clock.now_sim_time_s<state.last_call_sim_time_s
    state=latch(state,'MODEL_CLOCK_REVERSED');sample.reason=state.failure_reason;return
end
state.last_call_wall_time_s=clock.now_wall_time_s;
state.last_call_sim_time_s=clock.now_sim_time_s;
if ~isempty(bytes)
    [parsed,reason]=m600check.inspectCanonicalRotorObserverPacket(bytes,expected);
    if ~isempty(reason),state=latch(state,reason);sample.reason=reason;return,end
    bytes=bytes(:);gen=parsed.generation;sourceTime=parsed.sim_time_s;rotor=parsed.rotor_thrust_state_n;
    if clock.receive_wall_time_s>clock.now_wall_time_s,reason='RECEIVE_FROM_FUTURE';
    elseif state.present&&clock.receive_wall_time_s<state.receive_wall_time_s,reason='RECEIVE_CLOCK_REVERSED';
    end
    if ~isempty(reason),state=latch(state,reason);sample.reason=reason;return,end
    duplicate=false;
    if ~state.present
        if gen~=1,reason='INITIAL_GENERATION_NOT_ONE';end
    elseif gen==state.generation
        if isequal(bytes,state.bytes),duplicate=true;else,reason='GENERATION_CONFLICT';end
    elseif gen<state.generation
        reason='GENERATION_REVERSED';
    elseif state.generation==intmax('uint64')
        reason='GENERATION_EXHAUSTED';
    elseif gen~=state.generation+uint64(1)
        state.missing_generations=gen-state.generation-uint64(1);reason='GENERATION_GAP';
    elseif sourceTime<=state.sim_time_s
        reason='SOURCE_TIME_NOT_ADVANCING';
    end
    if ~isempty(reason),state=latch(state,reason);sample.reason=reason;return,end
    if ~duplicate
        state.present=true;state.bytes=bytes;state.generation=gen;state.sim_time_s=sourceTime;
        state.rotor_thrust_state_n=rotor;state.receive_wall_time_s=clock.receive_wall_time_s;
        sample.accepted_new_sample=true;
    end
    sample.duplicate_ignored=duplicate;
end
if ~state.present,sample.reason='NO_OBSERVER_SAMPLE';return,end
sourceAge=clock.now_sim_time_s-state.sim_time_s;
receiveAge=clock.now_wall_time_s-state.receive_wall_time_s;
if sourceAge< -1e-12,reason='SOURCE_FROM_FUTURE';
elseif sourceAge>expected.maximum_source_age_s+1e-12,reason='SOURCE_STALE';
elseif receiveAge<0,reason='RECEIVE_FROM_FUTURE';
elseif receiveAge>expected.maximum_receive_age_s+1e-12,reason='RECEIVE_STALE';
else,reason='';end
if ~isempty(reason)
    state=latch(state,reason);sample.accepted_new_sample=false;sample.reason=reason;return
end
sample.valid=true;sample.reason='VALID_SAME_CORE_ROTOR_OBSERVER';
sample.generation=state.generation;sample.session_token=expected.session_token;
sample.dll_sha256=expected.dll_sha256;sample.sim_time_s=state.sim_time_s;
sample.receive_wall_time_s=state.receive_wall_time_s;
sample.source_age_s=max(0,sourceAge);sample.receive_age_s=receiveAge;
sample.rotor_thrust_state_n=state.rotor_thrust_state_n;
end
function ok=validExpected(e)
fields={'session_token','dll_sha256','maximum_source_age_s','maximum_receive_age_s'};
ok=isstruct(e)&&isscalar(e)&&all(isfield(e,fields));if ~ok,return,end
ok=isa(e.session_token,'uint64')&&isscalar(e.session_token)&&e.session_token>0 ...
    &&isa(e.dll_sha256,'uint8')&&isequal(size(e.dll_sha256),[32,1])&&any(e.dll_sha256~=0) ...
    &&finiteScalar(e.maximum_source_age_s)&&e.maximum_source_age_s>0 ...
    &&finiteScalar(e.maximum_receive_age_s)&&e.maximum_receive_age_s>0;
end
function ok=validClock(c)
fields={'now_sim_time_s','now_wall_time_s','receive_wall_time_s'};
ok=isstruct(c)&&isscalar(c)&&all(isfield(c,fields));if ~ok,return,end
ok=true;for k=1:3,ok=ok&&finiteScalar(c.(fields{k}))&&c.(fields{k})>=0;end
end
function ok=finiteScalar(x)
ok=isnumeric(x)&&isreal(x)&&isscalar(x)&&isfinite(x);
end
function s=latch(s,reason)
s.failed=true;s.failure_reason=reason;
end
function s=initial()
s=struct('failed',false,'failure_reason','','present',false,'bytes',zeros(128,1,'uint8'), ...
    'generation',uint64(0),'missing_generations',uint64(0),'sim_time_s',NaN, ...
    'rotor_thrust_state_n',nan(6,1),'receive_wall_time_s',NaN,'last_call_wall_time_s',-Inf, ...
    'last_call_sim_time_s',-Inf,'context_bound',false,'session_token',uint64(0), ...
    'dll_sha256',zeros(32,1,'uint8'),'maximum_source_age_s',NaN,'maximum_receive_age_s',NaN);
end
function s=emptySample()
s=struct('valid',false,'reason','','accepted_new_sample',false,'duplicate_ignored',false, ...
    'source','COPTERSIM_SAME_CORE_ACCEPTED_ROTOR_LAG_STATE','abi_version',1, ...
    'generation',uint64(0),'session_token',uint64(0),'dll_sha256',zeros(32,1,'uint8'), ...
    'sim_time_s',NaN,'receive_wall_time_s',NaN,'source_age_s',Inf,'receive_age_s',Inf, ...
    'rotor_thrust_state_n',nan(6,1),'position_truth_present',false, ...
    'controller_or_estimator_position_feed_permitted',false,'runtime_origin_attested',false);
end
