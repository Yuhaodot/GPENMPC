function result = test_canonical_outer_clock()
% Test the outer scheduler.
root = fileparts(fileparts(mfilename('fullpath')));
oldPath = path; guard = onCleanup(@()path(oldPath)); %#ok<NASGU>
addpath(fullfile(root,'host_runtime'),'-begin');
names = {}; checks = false(0,1);
for rate = [0.8,1.0,1.2]
    [s,r] = step([],e(0,1,false,true,false,rate));
    check(sprintf('rate_%.1f_paused_initial_reset',rate),r.accepted && ...
        r.reset_phase && r.reset_warm_start && r.reset_causal && ~r.dispatch_solver);
    scheduled = [];
    for k=0:99
        [s,r] = step(s,e(k*.01,1,true,false,false,rate));
        assert(r.accepted);
        if r.dispatch_solver,scheduled(end+1)=r.dispatch_leg_time_s;end %#ok<AGROW>
    end
    check(sprintf('rate_%.1f_actual_time_not_phase',rate), ...
        isequal(round(scheduled*100),[0,30,60,90]) && ...
        s.outer_slots_accounted==4 && s.dispatch_permissions==4 && ...
        s.missed_outer_slots==0 && r.accounting_pass);
end
[s,~]=step([],e(0,1,false,true));
[s,r]=step(s,e(5,1,false,false));
check('initial_ground_does_not_consume_t0',r.accepted && ...
    r.leg_flight_time_s==0 && s.outer_slots_accounted==0 && ~r.dispatch_solver);
[s,r]=step(s,e(5,1,true,false));
check('first_flight_emits_exact_t0_once',r.dispatch_solver && r.dispatch_slot_index==0);
for k=1:20,[s,~]=step(s,e(5+k*.01,1,true,false));end
[s,r]=step(s,e(5.21,1,false,false));
check('pause_entry_settles_previous_flight_interval', ...
    r.effective_flight_dt_s==.01 && abs(r.leg_flight_time_s-.21)<1e-12);
[s,r]=step(s,e(25.21,1,false,false));
check('land_ground_dwell_outer_time_frozen',r.accepted && ...
    r.effective_flight_dt_s==0 && abs(r.leg_flight_time_s-.21)<1e-12 && ~r.dispatch_solver);
[s,r]=step(s,e(25.21,1,true,false));
check('same_leg_resume_does_not_reset',r.accepted && ~r.reset_phase && ...
    ~r.reset_warm_start && ~r.reset_causal && ~r.dispatch_solver);
for k=1:9,[s,r]=step(s,e(25.21+k*.01,1,true,false));end
check('resume_next_slot_uses_effective_time',r.dispatch_solver && ...
    abs(r.dispatch_leg_time_s-.30)<1e-12);
[s,~]=step(s,e(25.31,1,false,false));
[s,r]=step(s,e(30,2,false,true));
check('new_leg_explicit_three_resets',r.accepted && r.reset_phase && ...
    r.reset_warm_start && r.reset_causal && r.leg_flight_time_s==0 && ~r.dispatch_solver);
[s,r]=step(s,e(30,2,true,false));
check('new_leg_first_flight_zero_slot',r.dispatch_solver && r.dispatch_leg_time_s==0);

s=flying();[s,r]=step(s,e(.31,1,true,false));
check('late_slot_accounted_never_caught_up',r.accepted && ~r.dispatch_solver && ...
    r.skipped_active_inner_ticks==30 && r.outer_slots_accounted==1 && ...
    r.missed_outer_slots==1 && r.missed_past_outer_slots==1);
[s,r]=step(s,e(.32,1,true,false));
check('no_catchup_on_next_tick',r.accepted && ~r.dispatch_solver && ...
    r.outer_slots_accounted==0 && r.accounting_pass);
[s,r]=step(s,e(.90,1,true,false));
check('multi_slot_gap_only_present_slot_may_dispatch',r.accepted && ...
    r.dispatch_solver && r.dispatch_slot_index==3 && r.missed_outer_slots==1 && ...
    s.outer_slots_accounted==4 && s.dispatch_permissions==2 && s.missed_outer_slots==2);
s=flying();[s,r]=step(s,e(.30,1,true,false,true));
check('inflight_blocks_second_solver',r.accepted && ~r.dispatch_solver && ...
    r.busy_outer_slots==1 && r.missed_outer_slots==1);
[s,r]=step(s,e(.31,1,true,false,false));
check('busy_miss_not_dispatched_late',r.accepted && ~r.dispatch_solver);
[s,r]=step(s,e(.60,1,true,false,false));
check('subsequent_fresh_slot_can_dispatch',r.dispatch_solver && r.dispatch_slot_index==2);
s=flying();[s,r]=step(s,e(.61,1,false,false));
check('terminal_pause_gap_does_not_hide_missed_slots',r.accepted && ...
    ~r.dispatch_solver && r.missed_outer_slots==2 && s.outer_slots_accounted==3);
[s,r]=step(s,e(.62,2,false,true));
check('new_leg_keeps_previous_miss_accounting',r.accepted && ...
    s.outer_slots_accounted==3 && s.dispatch_permissions==1 && s.missed_outer_slots==2);

rejectCase('initial_active_forbidden',[],e(0,1,true,true), ...
    'INITIALIZATION_REQUIRES_PAUSED_LEG1_RESET');
rejectCase('initial_leg2_forbidden',[],e(0,2,false,true), ...
    'INITIALIZATION_REQUIRES_PAUSED_LEG1_RESET');
rejectCase('initial_missing_reset_forbidden',[],e(0,1,false,false), ...
    'INITIALIZATION_REQUIRES_PAUSED_LEG1_RESET');
rejectCase('active_new_leg_reset_forbidden',flying(),e(.01,2,false,true), ...
    'LEG_RESET_REQUIRES_PAUSED_SEQUENTIAL_LEG_AND_NO_INFLIGHT');
[paused,~]=step([],e(0,1,false,true));
rejectCase('busy_reset_forbidden',paused,e(.01,2,false,true,true), ...
    'LEG_RESET_REQUIRES_PAUSED_SEQUENTIAL_LEG_AND_NO_INFLIGHT');
rejectCase('nonsequential_leg_reset_forbidden',paused,e(.01,3,false,true), ...
    'LEG_RESET_REQUIRES_PAUSED_SEQUENTIAL_LEG_AND_NO_INFLIGHT');
rejectCase('same_leg_reset_forbidden',paused,e(.01,1,false,true), ...
    'LEG_RESET_REQUIRES_PAUSED_SEQUENTIAL_LEG_AND_NO_INFLIGHT');
rejectCase('unannounced_leg_change_forbidden',paused,e(.01,2,false,false), ...
    'LEG_CHANGE_WITHOUT_EXPLICIT_RESET');
s=flying();[s,~]=step(s,e(.01,1,true,false));
rejectCase('reversed_time_forbidden',s,e(0,1,true,false),'TIME_REVERSED');
rejectCase('duplicate_sample_forbidden',s,e(.01,1,true,false),'DUPLICATE_SAMPLE');
rejectCase('noncanonical_inner_dt_forbidden',s,e(.025,1,true,false), ...
    'TIME_NOT_ON_CANONICAL_INNER_GRID');
rejectCase('nan_time_forbidden',s,e(NaN,1,true,false),'INVALID_OR_NONFINITE_EVENT');
rejectCase('infinite_phase_rate_forbidden',s,e(.02,1,true,false,false,Inf), ...
    'INVALID_OR_NONFINITE_EVENT');
bad=e(.02,1,true,false);bad.flight_active=[true,false];
rejectCase('multiple_flight_flags_forbidden',s,bad,'INVALID_OR_NONFINITE_EVENT');
bad=e(.02,1,true,false);bad.leg_id=[1,2];
rejectCase('multiple_leg_identity_forbidden',s,bad,'INVALID_OR_NONFINITE_EVENT');
[failed,~]=step(s,e(0,1,true,false));
[after,r]=step(failed,e(.02,2,false,true));
check('reset_cannot_clear_failure_latch',r.failed && ~r.accepted && ...
    ~r.dispatch_solver && after.failed && strcmp(after.failure_code,'TIME_REVERSED'));
check('fixed_periods_and_deadline',s.inner_period_s==.01 && ...
    s.outer_period_s==.30 && s.solver_deadline_s==.28);
badState=s;badState.outer_period_s=.2;
rejectCase('changed_outer_period_rejected',badState,e(.02,1,true,false),'INVALID_CLOCK_STATE');
badState=rmfield(s,'next_outer_tick');
rejectCase('incomplete_state_rejected',badState,e(.02,1,true,false),'INVALID_CLOCK_STATE');
result=struct('schema','GPENMPC_CANONICAL_OUTER_CLOCK_TEST_RESULT_V1', ...
    'pass',all(checks),'passed_count',nnz(checks),'test_count',numel(checks), ...
    'names',{names},'checks',checks,'hardware_actions',0,'solver_launch_count',0);
assert(result.pass,'gpenmpcNative:ClockTestFailure','Clock tests failed.');

    function check(name,yes)
        names{end+1}=name;checks(end+1,1)=logical(yes);
        assert(yes,'gpenmpcNative:ClockTestFailure','%s',name);
    end
    function rejectCase(name,state,event,reason)
        [a,b]=step(state,event);
        check(name,a.failed && b.failed && ~b.accepted && ...
            ~b.dispatch_solver && strcmp(b.reason,reason));
    end
end
function [s,r]=step(s,e)
[s,r]=gpenmpcNative.canonicalOuterClock(s,e);
end
function s=flying()
[s,~]=step([],e(0,1,false,true));[s,~]=step(s,e(0,1,true,false));
end
function x=e(t,leg,flight,reset,busy,rate)
if nargin<5,busy=false;end
if nargin<6,rate=1;end
x=struct('time_s',t,'leg_id',leg,'flight_active',logical(flight), ...
    'reset_leg',logical(reset),'solver_in_flight',logical(busy),'phase_rate',rate);
end
