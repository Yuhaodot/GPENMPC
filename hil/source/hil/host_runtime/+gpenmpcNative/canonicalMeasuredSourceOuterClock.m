function [state, receipt] = canonicalMeasuredSourceOuterClock(state, event)
% CANONICALMEASUREDSOURCEOUTERCLOCK Schedule outer updates on measured source HRT.
% Events carry sample_timestamp_ns, actual_source_delta_us, leg_id,
% flight_active, reset_leg, solver_in_flight and phase_rate.
% The caller validates source identity and lifecycle.
% Advance fixed wall-time periods, independently of reference phase.
% RSP deltas are between observations; RLS deltas use the last installed
% control sample, which multiple observations may share.
% Dispatch at most the latest due slot; record skipped and busy slots.
% The worker separately enforces its .28 s solve deadline.
% Pause settles the active interval and suppresses dispatch; a coincident slot
% remains available when the same leg resumes.

if isempty(state)
    state=initialState();
    if isstruct(event)&&isscalar(event)&&isfield(event,'board_local_source_bound_us') ...
            &&(isequal(event.board_local_source_bound_us,uint64(20000)) ...
            ||isequal(event.board_local_source_bound_us,uint64(50000)))
        state.maximum_actual_source_delta_us=event.board_local_source_bound_us;
    end
end
receipt=blankReceipt();
if ~validState(state),state=initialState();reject('INVALID_CLOCK_STATE');return;end
if state.failed,receipt.reason=state.failure_code;receipt.failed=true;return;end
required={'sample_timestamp_ns','actual_source_delta_us','leg_id', ...
    'flight_active','reset_leg','solver_in_flight','phase_rate'};
if ~isstruct(event)||~isscalar(event)||~all(isfield(event,required))|| ...
        ~u64(event.sample_timestamp_ns)||event.sample_timestamp_ns==0|| ...
        ~u64(event.actual_source_delta_us)|| ...
        ~finiteScalar(event.leg_id)||event.leg_id<1||event.leg_id~=fix(event.leg_id)|| ...
        event.leg_id>flintmax||~finiteScalar(event.phase_rate)|| ...
        ~logicalScalar(event.flight_active)||~logicalScalar(event.reset_leg)|| ...
        ~logicalScalar(event.solver_in_flight)
    reject('INVALID_OR_NONFINITE_EVENT');return
end
localSource=state.maximum_actual_source_delta_us~=uint64(10000);
if isfield(event,'board_local_source_bound_us')
    if ~localSource||~isequal(event.board_local_source_bound_us,state.maximum_actual_source_delta_us)
        reject('SOURCE_PROFILE_CHANGED');return
    end
elseif localSource
    reject('SOURCE_PROFILE_CHANGED');return
end
if event.actual_source_delta_us>state.maximum_actual_source_delta_us ...
        &&(~localSource||state.flight_active||event.flight_active)
    reject('ACTUAL_SOURCE_DELTA_EXCEEDED');return
end
receipt.original_sample_timestamp_ns=event.sample_timestamp_ns;
receipt.actual_source_delta_us=event.actual_source_delta_us;
if ~state.initialized
    if ~event.reset_leg||event.leg_id~=1||event.flight_active||event.solver_in_flight
        reject('INITIALIZATION_REQUIRES_PAUSED_LEG1_RESET');return
    end
    state.initialized=true;state.leg_id=1;state.reset_count=1;
    state.last_sample_timestamp_ns=event.sample_timestamp_ns;
    state.last_actual_source_delta_us=event.actual_source_delta_us;
    markReset();finish();return
end
if event.sample_timestamp_ns<state.last_sample_timestamp_ns
    reject('TIME_REVERSED');return
end
dt=event.sample_timestamp_ns-state.last_sample_timestamp_ns;
if dt==0
    if event.actual_source_delta_us~=state.last_actual_source_delta_us
        reject('SAME_SOURCE_DELTA_CHANGED');return
    end
    if event.flight_active==state.flight_active&&~event.reset_leg
        reject('DUPLICATE_SAMPLE');return
    end
elseif event.actual_source_delta_us==0|| ...
        (~localSource&&event.actual_source_delta_us*uint64(1000)>dt)
    reject('ACTUAL_SOURCE_DELTA_INCONSISTENT');return
end
if localSource&&dt>0
    % Px4CanonicalLocalIo::capture_next supplies installed_numerics_ as
    % AtomicOdometryAdapter's interval_from. Preserve the reported delta;
    % compare its anchor, not its magnitude with the HOST observation gap.
    deltaNs=event.actual_source_delta_us*uint64(1000);
    priorDeltaNs=state.last_actual_source_delta_us*uint64(1000);
    if deltaNs>=event.sample_timestamp_ns||priorDeltaNs>=state.last_sample_timestamp_ns ...
            ||event.sample_timestamp_ns-deltaNs<state.last_sample_timestamp_ns-priorDeltaNs
        reject('ACTUAL_SOURCE_DELTA_INCONSISTENT');return
    end
end
if event.reset_leg
    if state.flight_active||event.flight_active||event.solver_in_flight||event.leg_id~=state.leg_id+1
        reject('LEG_RESET_REQUIRES_PAUSED_SEQUENTIAL_LEG_AND_NO_INFLIGHT');return
    end
elseif event.leg_id~=state.leg_id
    reject('LEG_CHANGE_WITHOUT_EXPLICIT_RESET');return
end
receipt.observed_source_interval_ns=dt;
% RLS control-anchor deltas cannot establish how many exports were missed.
receipt.host_export_gap=~localSource&&dt>event.actual_source_delta_us*uint64(1000);
if dt>0&&receipt.host_export_gap
    state.host_export_gap_events=state.host_export_gap_events+1;
end
if state.flight_active
    if dt>intmax('uint64')-state.leg_flight_time_ns||dt>intmax('uint64')-state.total_flight_time_ns
        reject('CLOCK_ACCUMULATOR_OVERFLOW');return
    end
    state.leg_flight_time_ns=state.leg_flight_time_ns+dt;
    state.total_flight_time_ns=state.total_flight_time_ns+dt;
    receipt.effective_flight_dt_ns=dt;
end
if event.reset_leg
    state.leg_id=double(event.leg_id);state.leg_flight_time_ns=uint64(0);
    state.next_outer_due_ns=uint64(0);state.reset_count=state.reset_count+1;markReset();
end
state.last_sample_timestamp_ns=event.sample_timestamp_ns;
state.last_actual_source_delta_us=event.actual_source_delta_us;
state.flight_active=event.flight_active;
hasDue=event.flight_active||state.leg_flight_time_ns>0;
if hasDue
    lastDue=state.leg_flight_time_ns;
    if ~event.flight_active,lastDue=lastDue-uint64(1);end
    if lastDue>=state.next_outer_due_ns
        count=idivide(lastDue-state.next_outer_due_ns,state.outer_period_ns)+uint64(1);
        if count>uint64(flintmax)||count>idivide(intmax('uint64')-state.next_outer_due_ns,state.outer_period_ns)
            reject('CLOCK_SLOT_OVERFLOW');return
        end
        latestDue=state.next_outer_due_ns+(count-uint64(1))*state.outer_period_ns;
        dispatch=event.flight_active&&~event.solver_in_flight;
        n=double(count);
        receipt.outer_slots_accounted=n;
        receipt.first_due_leg_time_ns=state.next_outer_due_ns;
        receipt.latest_due_leg_time_ns=latestDue;
        receipt.prior_crossed_slots=n-1;
        receipt.missed_outer_slots=n-double(dispatch);
        receipt.missed_past_outer_slots=n-double(event.flight_active);
        receipt.busy_outer_slots=double(event.flight_active&&event.solver_in_flight);
        receipt.dispatch_solver=dispatch;
        if dispatch
            receipt.dispatch_due_leg_time_ns=latestDue;
            receipt.dispatch_actual_leg_time_ns=state.leg_flight_time_ns;
            receipt.dispatch_delay_ns=state.leg_flight_time_ns-latestDue;
            receipt.dispatch_due_leg_time_s=double(latestDue)*1e-9;
            receipt.dispatch_leg_time_s=double(state.leg_flight_time_ns)*1e-9;
            receipt.dispatch_slot_index=double(idivide(latestDue,state.outer_period_ns));
        end
        state.next_outer_due_ns=state.next_outer_due_ns+count*state.outer_period_ns;
        state.outer_slots_accounted=state.outer_slots_accounted+n;
        state.dispatch_permissions=state.dispatch_permissions+double(dispatch);
        state.missed_outer_slots=state.missed_outer_slots+receipt.missed_outer_slots;
        state.busy_outer_slots=state.busy_outer_slots+receipt.busy_outer_slots;
        if state.outer_slots_accounted>flintmax,reject('CLOCK_SLOT_OVERFLOW');return;end
    end
end
finish();

    function reject(reason)
        state.failed=true;state.failure_code=reason;
        receipt.failed=true;receipt.reason=reason;receipt.dispatch_solver=false;
    end
    function markReset()
        receipt.reset_phase=true;receipt.reset_warm_start=true;receipt.reset_causal=true;
    end
    function finish()
        receipt.accepted=true;receipt.reason='PASS';receipt.leg_id=state.leg_id;
        receipt.leg_flight_time_ns=state.leg_flight_time_ns;
        receipt.total_flight_time_ns=state.total_flight_time_ns;
        receipt.leg_flight_time_s=double(state.leg_flight_time_ns)*1e-9;
        receipt.total_flight_time_s=double(state.total_flight_time_ns)*1e-9;
        receipt.effective_flight_dt_s=double(receipt.effective_flight_dt_ns)*1e-9;
        receipt.next_outer_leg_time_s=double(state.next_outer_due_ns)*1e-9;
        receipt.phase_rate_diagnostic_only=double(event.phase_rate);
        receipt.accounting_pass=state.outer_slots_accounted==state.dispatch_permissions+state.missed_outer_slots;
    end
end

function s=initialState()
s=struct('schema','GPENMPC_MEASURED_SOURCE_OUTER_CLOCK_V1', ...
    'initialized',false,'failed',false,'failure_code','NONE','flight_active',false, ...
    'inner_period_s',.01,'outer_period_s',.30,'solver_deadline_s',.28, ...
    'outer_period_ns',uint64(300000000),'maximum_actual_source_delta_us',uint64(10000), ...
    'leg_id',0,'last_sample_timestamp_ns',uint64(0),'last_actual_source_delta_us',uint64(0), ...
    'leg_flight_time_ns',uint64(0),'total_flight_time_ns',uint64(0),'next_outer_due_ns',uint64(0), ...
    'reset_count',0,'host_export_gap_events',0,'outer_slots_accounted',0, ...
    'dispatch_permissions',0,'missed_outer_slots',0,'busy_outer_slots',0);
end
function r=blankReceipt()
r=struct('schema','GPENMPC_MEASURED_SOURCE_OUTER_CLOCK_RECEIPT_V1', ...
    'accepted',false,'failed',false,'reason','UNPROCESSED','dispatch_solver',false, ...
    'dispatch_slot_index',NaN,'dispatch_due_leg_time_s',NaN,'dispatch_leg_time_s',NaN, ...
    'dispatch_due_leg_time_ns',uint64(0),'dispatch_actual_leg_time_ns',uint64(0), ...
    'dispatch_delay_ns',uint64(0),'first_due_leg_time_ns',uint64(0),'latest_due_leg_time_ns',uint64(0), ...
    'original_sample_timestamp_ns',uint64(0),'actual_source_delta_us',uint64(0), ...
    'observed_source_interval_ns',uint64(0),'effective_flight_dt_ns',uint64(0), ...
    'host_export_gap',false,'prior_crossed_slots',0, ...
    'reset_phase',false,'reset_warm_start',false,'reset_causal',false, ...
    'leg_id',0,'leg_flight_time_ns',uint64(0),'total_flight_time_ns',uint64(0), ...
    'leg_flight_time_s',0,'total_flight_time_s',0,'effective_flight_dt_s',0, ...
    'next_outer_leg_time_s',0,'phase_rate_diagnostic_only',NaN, ...
    'outer_slots_accounted',0,'missed_outer_slots',0,'missed_past_outer_slots',0, ...
    'busy_outer_slots',0,'accounting_pass',false,'solver_launch_count',0, ...
    'hardware_actions',0,'source_time_rewritten',false,'interpolated_samples',0);
end
function yes=u64(x),yes=isa(x,'uint64')&&isscalar(x);end
function yes=finiteScalar(x),yes=isnumeric(x)&&isreal(x)&&isscalar(x)&&isfinite(x);end
function yes=logicalScalar(x),yes=islogical(x)&&isscalar(x);end
function yes=validState(s)
t=initialState();yes=isstruct(s)&&isscalar(s)&&all(isfield(s,fieldnames(t)));
if ~yes,return;end
yes=ischar(s.schema)&&strcmp(s.schema,t.schema)&&ischar(s.failure_code)&& ...
    logicalScalar(s.initialized)&&logicalScalar(s.failed)&&logicalScalar(s.flight_active)&& ...
    isequal(s.inner_period_s,.01)&&isequal(s.outer_period_s,.30)&&isequal(s.solver_deadline_s,.28)&& ...
    isequal(s.outer_period_ns,uint64(300000000))&& ...
    (isequal(s.maximum_actual_source_delta_us,uint64(10000)) ...
    ||isequal(s.maximum_actual_source_delta_us,uint64(20000)) ...
    ||isequal(s.maximum_actual_source_delta_us,uint64(50000)));
if ~yes,return;end
for f={'last_sample_timestamp_ns','last_actual_source_delta_us','leg_flight_time_ns', ...
        'total_flight_time_ns','next_outer_due_ns'}
    if ~u64(s.(f{1})),yes=false;return;end
end
for f={'leg_id','reset_count','host_export_gap_events','outer_slots_accounted', ...
        'dispatch_permissions','missed_outer_slots','busy_outer_slots'}
    x=s.(f{1});if ~finiteScalar(x)||x<0||x~=fix(x)||x>flintmax,yes=false;return;end
end
yes=rem(s.next_outer_due_ns,s.outer_period_ns)==0&& ...
    s.outer_slots_accounted==s.dispatch_permissions+s.missed_outer_slots;
end
