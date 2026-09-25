function [state, receipt] = canonicalOuterClock(state, event)
% CANONICALOUTERCLOCK Schedule per-leg effective flight time.
% Initialize paused leg 1 with fields time_s, leg_id, flight_active, reset_leg,
% solver_in_flight and phase_rate. time_s is the inner-sample boundary.
% The prior flight_active flag determines whether the ending interval counts.
% Use a 0.30 s outer period, 0.01 s inner interval and separate 0.28 s solver deadline.
% Record missed slots and dispatch the current due slot once when idle.
% Reset only while paused, with no solve pending, into the next sequential leg.
% The caller owns phase, warm-start and causal-state reset; failures remain terminal.
% Source: gpenmpcRunNativeEnmpcWholeTask.m
% SHA BD9852A0CB4F350727C7A6B40E75004D85EC8AC2168C85EC556F7F93C7DBB7B9.

if isempty(state), state = initialState(); end
receipt = blankReceipt();
if ~validState(state)
    state = initialState(); reject('INVALID_CLOCK_STATE'); return
end
if state.failed
    receipt.reason = state.failure_code; receipt.failed = true; return
end
required = {'time_s','leg_id','flight_active','reset_leg', ...
    'solver_in_flight','phase_rate'};
if ~isstruct(event) || ~isscalar(event) || ~all(isfield(event,required)) || ...
        ~finiteScalar(event.time_s) || event.time_s < 0 || ...
        ~finiteScalar(event.leg_id) || event.leg_id < 1 || ...
        event.leg_id ~= fix(event.leg_id) || event.leg_id > flintmax || ...
        ~finiteScalar(event.phase_rate) || ...
        ~logicalScalar(event.flight_active) || ~logicalScalar(event.reset_leg) || ...
        ~logicalScalar(event.solver_in_flight)
    reject('INVALID_OR_NONFINITE_EVENT'); return
end
if ~state.initialized
    if ~event.reset_leg || event.leg_id ~= 1 || event.flight_active || event.solver_in_flight
        reject('INITIALIZATION_REQUIRES_PAUSED_LEG1_RESET'); return
    end
    state.initialized = true; state.leg_id = 1;
    state.last_time_s = double(event.time_s);
    state.reset_count = 1; markReset(); finish(); return
end

dt = double(event.time_s) - state.last_time_s;
if dt < 0
    reject('TIME_REVERSED'); return
end
ticks = round(dt / state.inner_period_s);
% Use 1 ns tolerance to recognize the floating-point grid.
if ticks > flintmax || abs(dt-ticks*state.inner_period_s) > 1e-9
    reject('TIME_NOT_ON_CANONICAL_INNER_GRID'); return
end
if ticks == 0 && event.flight_active == state.flight_active && ~event.reset_leg
    reject('DUPLICATE_SAMPLE'); return
end
if event.reset_leg
    if state.flight_active || event.flight_active || event.solver_in_flight || ...
            event.leg_id ~= state.leg_id + 1
        reject('LEG_RESET_REQUIRES_PAUSED_SEQUENTIAL_LEG_AND_NO_INFLIGHT'); return
    end
elseif event.leg_id ~= state.leg_id
    reject('LEG_CHANGE_WITHOUT_EXPLICIT_RESET'); return
end

receipt.elapsed_source_ticks = ticks;
receipt.skipped_inner_ticks = max(ticks-1,0);
state.skipped_inner_ticks = state.skipped_inner_ticks + receipt.skipped_inner_ticks;
if state.flight_active
    state.leg_flight_ticks = state.leg_flight_ticks + ticks;
    state.total_flight_ticks = state.total_flight_ticks + ticks;
    receipt.effective_flight_dt_s = ticks*state.inner_period_s;
    receipt.skipped_active_inner_ticks = max(ticks-1,0);
    state.skipped_active_inner_ticks = state.skipped_active_inner_ticks + ...
        receipt.skipped_active_inner_ticks;
end
if event.reset_leg
    state.leg_id = double(event.leg_id); state.leg_flight_ticks = 0;
    state.next_outer_tick = 0; state.reset_count = state.reset_count+1;
    markReset();
end
state.last_time_s = double(event.time_s);
state.flight_active = event.flight_active;

% A pause entry must also account expired slots crossed by a missing flying
% sample; a subsequent leg reset may not erase them. The pause boundary itself
% is excluded because no active interval starts there.
lastDueTick = state.leg_flight_ticks-double(~event.flight_active);
if lastDueTick >= state.next_outer_tick
    due = floor((lastDueTick-state.next_outer_tick)/state.outer_ticks)+1;
    exact = event.flight_active && mod(state.leg_flight_ticks,state.outer_ticks) == 0;
    dispatch = exact && ~event.solver_in_flight;
    receipt.outer_slots_accounted = due;
    receipt.missed_outer_slots = due-double(dispatch);
    receipt.missed_past_outer_slots = due-double(exact);
    receipt.busy_outer_slots = double(exact && event.solver_in_flight);
    receipt.dispatch_solver = dispatch;
    if dispatch
        receipt.dispatch_leg_time_s = state.leg_flight_ticks*state.inner_period_s;
        receipt.dispatch_slot_index = state.leg_flight_ticks/state.outer_ticks;
    end
    state.next_outer_tick = state.next_outer_tick + due*state.outer_ticks;
    state.outer_slots_accounted = state.outer_slots_accounted+due;
    state.dispatch_permissions = state.dispatch_permissions+double(dispatch);
    state.missed_outer_slots = state.missed_outer_slots+receipt.missed_outer_slots;
    state.busy_outer_slots = state.busy_outer_slots+receipt.busy_outer_slots;
end
finish();

    function reject(reason)
        state.failed = true; state.failure_code = reason;
        receipt.failed = true; receipt.reason = reason;
    end
    function markReset()
        receipt.reset_phase = true; receipt.reset_warm_start = true;
        receipt.reset_causal = true;
    end
    function finish()
        receipt.accepted = true; receipt.reason = 'PASS';
        receipt.leg_id = state.leg_id;
        receipt.leg_flight_time_s = state.leg_flight_ticks*state.inner_period_s;
        receipt.total_flight_time_s = state.total_flight_ticks*state.inner_period_s;
        receipt.next_outer_leg_time_s = state.next_outer_tick*state.inner_period_s;
        receipt.phase_rate_diagnostic_only = double(event.phase_rate);
        receipt.accounting_pass = state.outer_slots_accounted == ...
            state.dispatch_permissions+state.missed_outer_slots;
    end
end

function s = initialState()
s = struct('schema','GPENMPC_CANONICAL_OUTER_CLOCK_V1', ...
    'initialized',false,'failed',false,'failure_code','NONE', ...
    'inner_period_s',0.01,'outer_period_s',0.30,'solver_deadline_s',0.28, ...
    'outer_ticks',30,'leg_id',0,'last_time_s',0,'flight_active',false, ...
    'leg_flight_ticks',0,'total_flight_ticks',0,'next_outer_tick',0, ...
    'reset_count',0,'skipped_inner_ticks',0,'skipped_active_inner_ticks',0, ...
    'outer_slots_accounted',0,'dispatch_permissions',0, ...
    'missed_outer_slots',0,'busy_outer_slots',0);
end
function r = blankReceipt()
r = struct('schema','GPENMPC_CANONICAL_OUTER_CLOCK_RECEIPT_V1', ...
    'accepted',false,'failed',false,'reason','UNPROCESSED', ...
    'dispatch_solver',false,'dispatch_leg_time_s',NaN,'dispatch_slot_index',NaN, ...
    'reset_phase',false,'reset_warm_start',false,'reset_causal',false, ...
    'leg_id',0,'leg_flight_time_s',0,'total_flight_time_s',0, ...
    'next_outer_leg_time_s',0,'phase_rate_diagnostic_only',NaN, ...
    'elapsed_source_ticks',0,'effective_flight_dt_s',0, ...
    'skipped_inner_ticks',0,'skipped_active_inner_ticks',0, ...
    'outer_slots_accounted',0,'missed_outer_slots',0, ...
    'missed_past_outer_slots',0,'busy_outer_slots',0, ...
    'accounting_pass',false,'solver_launch_count',0,'hardware_actions',0);
end
function yes = finiteScalar(x)
yes = isnumeric(x) && isreal(x) && isscalar(x) && isfinite(x);
end
function yes = logicalScalar(x)
yes = islogical(x) && isscalar(x);
end
function yes = validState(s)
template = initialState();
yes = isstruct(s) && isscalar(s) && all(isfield(s,fieldnames(template)));
if ~yes,return;end
yes = ischar(s.schema) && strcmp(s.schema,template.schema) && ...
    logicalScalar(s.initialized) && logicalScalar(s.failed) && ...
    logicalScalar(s.flight_active) && ischar(s.failure_code) && ...
    isequal(s.inner_period_s,.01) && isequal(s.outer_period_s,.30) && ...
    isequal(s.solver_deadline_s,.28) && isequal(s.outer_ticks,30) && ...
    finiteScalar(s.last_time_s) && s.last_time_s>=0;
if ~yes,return;end
for name={'leg_id','leg_flight_ticks','total_flight_ticks','next_outer_tick', ...
        'reset_count','skipped_inner_ticks','skipped_active_inner_ticks', ...
        'outer_slots_accounted','dispatch_permissions','missed_outer_slots','busy_outer_slots'}
    x=s.(name{1});
    if ~finiteScalar(x) || x<0 || x~=fix(x) || x>flintmax,yes=false;return;end
end
yes = mod(s.next_outer_tick,30)==0 && ...
    s.outer_slots_accounted==s.dispatch_permissions+s.missed_outer_slots;
end
