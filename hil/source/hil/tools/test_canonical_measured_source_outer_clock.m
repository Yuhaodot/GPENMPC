function result=test_canonical_measured_source_outer_clock()
% Test measured-source scheduling with synthetic timestamps.
root=fileparts(fileparts(mfilename('fullpath')));old=path;guard=onCleanup(@()path(old)); %#ok<NASGU>
addpath(fullfile(root,'host_runtime'),'-begin');names={};checks=false(0,1);
common={'accepted','dispatch_solver','dispatch_slot_index','reset_phase','reset_warm_start', ...
    'reset_causal','leg_id','outer_slots_accounted','missed_outer_slots','missed_past_outer_slots', ...
    'busy_outer_slots','accounting_pass'};
times={'leg_flight_time_s','total_flight_time_s','effective_flight_dt_s','next_outer_leg_time_s','dispatch_leg_time_s'};
uniformEvents=0;
for rate=[.8,1,1.2]
    a=[];b=[];
    rows=[0 1 0 1 0;0 1 1 0 0;(1:61)' repmat([1 1 0 0],61,1); ...
        62 1 0 0 0; 162 1 0 0 0;162 1 1 0 0; ...
        (163:172)' repmat([1 1 0 0],10,1);173 1 0 0 0;174 2 0 1 0;174 2 1 0 0];
    previousTick=0;actual=uint64(0);
    for k=1:size(rows,1)
        tick=rows(k,1);if tick>previousTick,actual=uint64(10000);end
        ev=e(uint64(tick)*uint64(10000),actual,rows(k,2),rows(k,3),rows(k,4),rows(k,5),rate);
        oldEvent=struct('time_s',double(tick)*.01,'leg_id',rows(k,2), ...
            'flight_active',logical(rows(k,3)),'reset_leg',logical(rows(k,4)), ...
            'solver_in_flight',logical(rows(k,5)),'phase_rate',rate);
        [a,ra]=step(a,ev);[b,rb]=gpenmpcNative.canonicalOuterClock(b,oldEvent);
        assert(ra.accepted&&rb.accepted);
        for j=1:numel(common),assert(isequaln(ra.(common{j}),rb.(common{j})),common{j});end
        for j=1:numel(times)
            x=ra.(times{j});y=rb.(times{j});assert((isnan(x)&&isnan(y))||abs(x-y)<1e-12,times{j});
        end
        previousTick=tick;uniformEvents=uniformEvents+1;
    end
    check(sprintf('uniform_phase_rate_%g_matches_old_actions_and_time',rate), ...
        a.dispatch_permissions==b.dispatch_permissions&&a.missed_outer_slots==b.missed_outer_slots);
end

s=flying();[s,r]=step(s,e(9000,9000,1,true,false));
check('synthetic_private_9ms_accepted_original_time_preserved',r.accepted&& ...
    r.original_sample_timestamp_ns==uint64(1009000000)&&r.effective_flight_dt_ns==uint64(9000000));
for k=2:33,[s,r]=step(s,e(k*9000,9000,1,true,false));assert(r.accepted&&~r.dispatch_solver);end
[s,r]=step(s,e(306000,9000,1,true,false));
check('first_actual_sample_after_due_dispatches_once',r.dispatch_solver&&r.dispatch_slot_index==1&& ...
    r.dispatch_due_leg_time_ns==uint64(300000000)&&r.dispatch_actual_leg_time_ns==uint64(306000000)&& ...
    r.dispatch_delay_ns==uint64(6000000)&&s.next_outer_due_ns==uint64(600000000));
[s,r]=step(s,e(315000,9000,1,true,false));
check('late_dispatch_not_replayed_on_following_sample',r.accepted&&~r.dispatch_solver&&r.outer_slots_accounted==0);

s=flying();[s,r]=step(s,e(910000,10000,1,true,false));
check('missing_exports_account_all_due_slots_dispatch_only_latest',r.accepted&&r.host_export_gap&& ...
    r.prior_crossed_slots==2&&r.outer_slots_accounted==3&&r.missed_outer_slots==2&& ...
    r.dispatch_solver&&r.dispatch_slot_index==3&&r.dispatch_delay_ns==uint64(10000000)&& ...
    s.outer_slots_accounted==4&&s.dispatch_permissions==2&&s.missed_outer_slots==2);
[s,r]=step(s,e(919000,9000,1,true,false));
check('missing_exports_never_synthesize_catchup_samples',~r.dispatch_solver&& ...
    r.interpolated_samples==0&&~r.source_time_rewritten);
s=flying();[s,r]=step(s,e(306000,9000,1,true,false,true));
check('busy_due_slot_missed_single_worker_not_relaunched',r.accepted&&~r.dispatch_solver&& ...
    r.busy_outer_slots==1&&r.missed_outer_slots==1);
[s,r]=step(s,e(315000,9000,1,true,false));
check('worker_free_after_busy_slot_does_not_replay',~r.dispatch_solver&&r.outer_slots_accounted==0);
[s,r]=step(s,e(603000,9000,1,true,false));
check('later_due_slot_is_eligible_with_actual_delay',r.dispatch_solver&& ...
    r.dispatch_slot_index==2&&r.dispatch_delay_ns==uint64(3000000));

s=flying();[s,r]=step(s,e(9000,9000,1,false,false));
check('pause_settles_previous_active_interval',r.accepted&&r.effective_flight_dt_ns==uint64(9000000));
[s,r]=step(s,e(5009000,10000,1,false,false));
check('ground_pause_accumulates_no_flight_or_solver_time',r.accepted&&~r.dispatch_solver&& ...
    r.effective_flight_dt_ns==0&&s.leg_flight_time_ns==uint64(9000000));
[s,r]=step(s,e(5009000,10000,1,true,false));
check('same_sample_explicit_resume_does_not_renew_source_or_reset',r.accepted&& ...
    r.observed_source_interval_ns==0&&~r.reset_phase&&~r.dispatch_solver);
[s,r]=step(s,e(5300000,10000,1,true,false));
check('resume_uses_effective_flight_time_not_ground_time',r.dispatch_solver&& ...
    r.leg_flight_time_ns==uint64(300000000)&&r.dispatch_delay_ns==0);
[s,~]=step(s,e(5309000,9000,1,false,false));
oldTotal=s.total_flight_time_ns;oldSlots=s.outer_slots_accounted;
[s,r]=step(s,e(5318000,9000,2,false,true));
check('sequential_leg_resets_phase_warmstart_causal_only',r.accepted&&r.reset_phase&& ...
    r.reset_warm_start&&r.reset_causal&&s.leg_flight_time_ns==0&& ...
    s.total_flight_time_ns==oldTotal&&s.outer_slots_accounted==oldSlots);
[s,r]=step(s,e(5318000,9000,2,true,false));
check('next_leg_dispatches_zero_slot_once',r.dispatch_solver&&r.dispatch_slot_index==0);
s=flying();[s,r]=step(s,e(300000,10000,1,false,false));
check('exact_due_pause_boundary_does_not_dispatch_or_erase_due',r.accepted&& ...
    ~r.dispatch_solver&&r.outer_slots_accounted==0&&s.next_outer_due_ns==uint64(300000000));
[s,r]=step(s,e(300000,10000,1,true,false));
check('resume_at_exact_pause_boundary_keeps_due_slot',r.dispatch_solver&&r.dispatch_slot_index==1);
s=flying();[s,r]=step(s,e(610000,10000,1,false,false));
check('pause_with_export_gap_keeps_missed_slots_honest',r.accepted&&~r.dispatch_solver&& ...
    r.missed_outer_slots==2&&s.outer_slots_accounted==3);

rejectCase('11ms_source_bound_still_rejected',flying(),e(11000,11000,1,true,false),'ACTUAL_SOURCE_DELTA_EXCEEDED');
rejectCase('10001us_source_bound_still_rejected',flying(),e(10001,10001,1,true,false),'ACTUAL_SOURCE_DELTA_EXCEEDED');
s=flying();[s,r]=step(s,e(10000,10000,1,true,false));
check('exact_original_10000us_bound_accepted',r.accepted);
rejectCase('zero_delta_new_source_rejected',s,e(19000,0,1,true,false),'ACTUAL_SOURCE_DELTA_INCONSISTENT');
rejectCase('source_delta_larger_than_observed_interval_rejected',s,e(19000,10000,1,true,false),'ACTUAL_SOURCE_DELTA_INCONSISTENT');
rejectCase('duplicate_source_rejected',s,e(10000,10000,1,true,false),'DUPLICATE_SAMPLE');
rejectCase('same_source_delta_mutation_rejected',s,e(10000,9000,1,false,false),'SAME_SOURCE_DELTA_CHANGED');
rejectCase('time_regression_rejected',s,e(9000,9000,1,true,false),'TIME_REVERSED');
bad=e(19000,9000,1,true,false);bad.sample_timestamp_ns=double(bad.sample_timestamp_ns);
rejectCase('lossy_double_absolute_time_rejected',s,bad,'INVALID_OR_NONFINITE_EVENT');
bad=e(19000,9000,1,true,false);bad.phase_rate=NaN;
rejectCase('nonfinite_phase_diagnostic_rejected',s,bad,'INVALID_OR_NONFINITE_EVENT');
rejectCase('initial_active_rejected',[],e(0,0,1,true,true),'INITIALIZATION_REQUIRES_PAUSED_LEG1_RESET');
rejectCase('active_leg_reset_rejected',s,e(19000,9000,2,false,true),'LEG_RESET_REQUIRES_PAUSED_SEQUENTIAL_LEG_AND_NO_INFLIGHT');
[paused,~]=step([],e(0,0,1,false,true));
rejectCase('busy_leg_reset_rejected',paused,e(9000,9000,2,false,true,true),'LEG_RESET_REQUIRES_PAUSED_SEQUENTIAL_LEG_AND_NO_INFLIGHT');
rejectCase('nonsequential_leg_reset_rejected',paused,e(9000,9000,3,false,true),'LEG_RESET_REQUIRES_PAUSED_SEQUENTIAL_LEG_AND_NO_INFLIGHT');
rejectCase('leg_change_without_reset_rejected',paused,e(9000,9000,2,false,false),'LEG_CHANGE_WITHOUT_EXPLICIT_RESET');
bad=s;bad.outer_period_ns=uint64(200000000);
rejectCase('outer_period_change_rejected',bad,e(19000,9000,1,true,false),'INVALID_CLOCK_STATE');
bad=s;bad.solver_deadline_s=.4;
rejectCase('solver_deadline_not_extended',bad,e(19000,9000,1,true,false),'INVALID_CLOCK_STATE');
[failed,~]=step(s,e(9000,9000,1,true,false));[failed,r]=step(failed,e(20000,10000,2,false,true));
check('failure_latch_cannot_be_reset',failed.failed&&r.failed&&~r.accepted&&~r.dispatch_solver&&strcmp(r.reason,'TIME_REVERSED'));
huge=bitshift(uint64(1),60);x=e(0,0,1,false,true);x.sample_timestamp_ns=huge;
[s,~]=step([],x);x.flight_active=true;x.reset_leg=false;[s,~]=step(s,x);
x.sample_timestamp_ns=huge+uint64(9000000);x.actual_source_delta_us=uint64(9000);[s,r]=step(s,x);
check('absolute_uint64_above_flintmax_keeps_exact_measured_delta',r.accepted&& ...
    r.observed_source_interval_ns==uint64(9000000)&&s.last_sample_timestamp_ns==x.sample_timestamp_ns);
check('fixed_period_deadline_and_source_bound_unchanged',s.inner_period_s==.01&& ...
    s.outer_period_s==.30&&s.solver_deadline_s==.28&&s.maximum_actual_source_delta_us==uint64(10000));
local=e(0,50000,1,false,true);local.board_local_source_bound_us=uint64(20000);
[ls,lr]=step([],local);
check('local_paused_observation_not_a_control_interval',lr.accepted&& ...
 ~lr.dispatch_solver&&ls.leg_flight_time_ns==0&&lr.actual_source_delta_us==uint64(50000));
local=e(11000,11000,1,true,false);local.board_local_source_bound_us=uint64(20000);
[ls,lr]=step(ls,local);
check('local_active_11ms_matches_existing_board_bound',lr.accepted&&lr.dispatch_solver&&ls.inner_period_s==.01);
local=e(31000,20000,1,true,false);local.board_local_source_bound_us=uint64(20000);
[ls,lr]=step(ls,local);
check('local_existing_20ms_boundary_keeps_original_time',lr.accepted&& ...
 lr.effective_flight_dt_ns==uint64(20000000)&&ls.outer_period_s==.3&&ls.solver_deadline_s==.28);
local=e(51001,20001,1,true,false);local.board_local_source_bound_us=uint64(20000);
rejectCase('local_over_20ms_still_rejected',ls,local,'ACTUAL_SOURCE_DELTA_EXCEEDED');
local.board_local_source_bound_us=uint64(30000);
rejectCase('local_cannot_change_bound_midrun',ls,local,'SOURCE_PROFILE_CHANGED');
local=rmfield(local,'board_local_source_bound_us');
rejectCase('local_cannot_silently_drop_profile',ls,local,'SOURCE_PROFILE_CHANGED');
local=e(0,0,1,false,true);local.board_local_source_bound_us=uint64(50000);
[ls,lr]=step([],local);assert(lr.accepted);
local=e(35866,35866,1,true,false);local.board_local_source_bound_us=uint64(50000);
[ls,lr]=step(ls,local);
check('measured_35866us_delta_uses_selected_board_bound',lr.accepted ...
 &&lr.actual_source_delta_us==uint64(35866)&&~lr.source_time_rewritten);
local=e(85866,50000,1,true,false);local.board_local_source_bound_us=uint64(50000);
[ls,lr]=step(ls,local);
check('exact_50ms_preserves_nominal_and_outer_period',lr.accepted ...
 &&ls.inner_period_s==.01&&ls.outer_period_s==.30&&ls.solver_deadline_s==.28);
local=e(135867,50001,1,true,false);local.board_local_source_bound_us=uint64(50000);
rejectCase('over_50ms_rejected',ls,local,'ACTUAL_SOURCE_DELTA_EXCEEDED');
local.board_local_source_bound_us=uint64(20000);
rejectCase('source_profile_cannot_change_midrun',ls,local,'SOURCE_PROFILE_CHANGED');
% Replay consecutive generations that share installed sample 82736997 us.
% Their 12,520 us observation gap differs from the 35,672 us control interval.
local=e(0,23152,1,false,true);local.sample_timestamp_ns=uint64(82760149000);
local.board_local_source_bound_us=uint64(50000);[ls,lr]=step([],local);assert(lr.accepted);
local.flight_active=true;local.reset_leg=false;[ls,lr]=step(ls,local);assert(lr.accepted);
local.sample_timestamp_ns=uint64(82772669000);local.actual_source_delta_us=uint64(35672);
[ls,lr]=step(ls,local);
check('unexecuted_observation_preserves_shared_control_anchor',lr.accepted ...
 &&lr.observed_source_interval_ns==uint64(12520000) ...
 &&lr.actual_source_delta_us==uint64(35672)&&~lr.source_time_rewritten ...
 &&ls.leg_flight_time_ns==uint64(12520000)&&~lr.host_export_gap);
bad=local;bad.sample_timestamp_ns=bad.sample_timestamp_ns+uint64(1000000);
bad.actual_source_delta_us=uint64(36673);
rejectCase('local_control_anchor_cannot_regress',ls,bad,'ACTUAL_SOURCE_DELTA_INCONSISTENT');
bad.actual_source_delta_us=uint64(50001);
rejectCase('local_overlapping_observation_does_not_extend_50ms',ls,bad,'ACTUAL_SOURCE_DELTA_EXCEEDED');
local.sample_timestamp_ns=uint64(82795149000);local.actual_source_delta_us=uint64(10000);
[ls,lr]=step(ls,local);
check('local_new_actual_control_anchor_advances_without_clock_reset',lr.accepted ...
 &&ls.leg_flight_time_ns==uint64(35000000)&&~lr.reset_phase&&~lr.dispatch_solver);
result=struct('schema','MEASURED_SOURCE_OUTER_CLOCK_HOST_TEST_V1','pass',all(checks), ...
    'passed_count',nnz(checks),'test_count',numel(checks),'uniform_crosschecked_events',uniformEvents, ...
    'names',{names},'checks',checks,'source','SYNTHETIC_PRIVATE_RSP_TIMESTAMPS_NOT_FLIGHT', ...
    'hardware_actions',0,'solver_launch_count',0,'model_runs',0);
assert(result.pass);
    function check(name,yes)
        names{end+1}=name;checks(end+1,1)=logical(yes);
        assert(yes,'gpenmpcNative:MeasuredClockTestFailure','%s',name);
    end
    function rejectCase(name,state,event,reason)
        [a,b]=step(state,event);check(name,a.failed&&b.failed&&~b.accepted&& ...
            ~b.dispatch_solver&&strcmp(b.reason,reason));
    end
end
function [s,r]=step(s,e),[s,r]=gpenmpcNative.canonicalMeasuredSourceOuterClock(s,e);end
function s=flying()
[s,~]=step([],e(0,0,1,false,true));[s,~]=step(s,e(0,0,1,true,false));
end
function x=e(us,delta,leg,flight,reset,busy,rate)
if nargin<6,busy=false;end
if nargin<7,rate=1;end
x=struct('sample_timestamp_ns',uint64(1000000000)+uint64(us)*uint64(1000), ...
    'actual_source_delta_us',uint64(delta),'leg_id',leg,'flight_active',logical(flight), ...
    'reset_leg',logical(reset),'solver_in_flight',logical(busy),'phase_rate',rate);
end
