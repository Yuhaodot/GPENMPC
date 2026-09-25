function report=test_rfly_task_wind_provider(outputRoot)
% Test saved-task wind with private-source fixtures.
build=string(fileparts(fileparts(mfilename('fullpath'))));
addpath(fullfile(build,'host_runtime'));
assert(~isfolder(outputRoot));mkdir(outputRoot);
task=fullfile(build,'task_packages','cambridge_canonical','MU_CAMBRIDGE_MA_02__CANONICAL_PHYSICAL_TASK.mat');
taskSha='B876546B468265F84BD028AC549B06EF8ADDCB25CBA09CF389A08F4B21290A5F';
q=load(task,'physicalTask');r=q.physicalTask.reference;
source=fullfile(gpenmpc_external_path('board_commit_exchange_fixture'));
b1=readbin(fullfile(source,'SOURCE_1.bin'));b2=readbin(fullfile(source,'SOURCE_2.bin'));b3=readbin(fullfile(source,'SOURCE_3.bin'));
rx=uint64(9000000000);s=gpenmpcNative.RflySnapshotDecoder(b1,rx);
e=struct('uid',string(s.observed_uid),'system_id',double(s.source_system),'component_id',double(s.source_component), ...
    'boot_generation',s.observed_boot_generation,'configuration_payload_sha256',hex(s.configuration_sha256), ...
    'maximum_runtime_age_ns',uint64(100000000));
checks=struct('name',{},'pass',{});
make=@()gpenmpcNative.RflyTaskWindProvider(task,taskSha,e);
p=make();[w,a]=p.observe(0,b1,rx,rx);
check('exact_task_and_current_physical_method',strcmpi(p.TaskSha256,taskSha)&&~p.Failed);
check('initial_estimate_exact_not_label_only',isequal(w.estimate_xy_mps,r.wind_estimate_xy_mps(1,:).'));
check('actual_six_rotor_efficiency_from_same_task',isequal(p.RotorEffectiveness, ...
    q.physicalTask.mission_config.plant_mismatch.thrust_effectiveness_by_rotor(:)));
check('original_rx_and_independent_source_generation',w.rx_ns==rx&&w.generation==uint64(s.subscription_generation));
check('no_perfect_wind_or_state_or_authority_claim',~a.actual_plant_wind_used&&~a.plant_truth_state_used ...
    &&~a.online_wind_measurement&&~a.publication_authority&&~a.source_clock_advanced);
[held,h]=p.observe(0,b1,rx,rx+uint64(10));
check('same_source_poll_keeps_original_input',isequal(held,w)&&isequal(h,a));
[w2,a2]=p.observe(.019,b2,rx+uint64(10000000),rx+uint64(10000000));
check('causal_left_hold_not_future_row',a2.selected_row==2&&a2.selected_row_time_s==.01 ...
    &&isequal(w2.estimate_xy_mps,r.wind_estimate_xy_mps(2,:).'));
[w3,a3]=p.observe(.019,b3,rx+uint64(20000000),rx+uint64(20000000));
check('paused_task_new_source_is_input_evaluation_not_measurement',a3.selected_row==a2.selected_row ...
    &&isequal(w3.estimate_xy_mps,w2.estimate_xy_mps)&&w3.generation>w2.generation ...
    &&~a3.online_wind_measurement);
bad=taskSha;bad(1)='0';reject(@()gpenmpcNative.RflyTaskWindProvider(task,bad,e), ...
    'gpenmpcNative:TaskWindIdentity','wrong_task_sha_before_observation');
new=make();new.observe(0,b1,rx,rx);
reject(@()new.observe(0,b1,rx+uint64(1),rx+uint64(1)), ...
    'gpenmpcNative:TaskWindDuplicate','duplicate_cannot_renew_receive_time');
check('fault_latched',new.Failed);
reject(@()new.observe(.01,b2,rx+uint64(10000000),rx+uint64(10000000)), ...
    'gpenmpcNative:TaskWindFailed','new_source_cannot_wash_fault');
new=make();new.observe(.02,b1,rx,rx);
reject(@()new.observe(.01,b2,rx+uint64(10000000),rx+uint64(10000000)), ...
    'gpenmpcNative:TaskWindPhase','task_clock_cannot_reverse');
new=make();new.observe(0,b1,rx,rx);
reject(@()new.observe(.01,b1,rx,rx+uint64(1)), ...
    'gpenmpcNative:TaskWindDuplicate','poll_cannot_advance_task_without_source');
new=make();reject(@()new.observe(0,b1,rx,rx+e.maximum_runtime_age_ns+uint64(1)), ...
    'gpenmpcNative:TaskWindClock','stale_source_not_fresh_task_input');
new=make();reject(@()new.observe(0,b1,rx+uint64(1),rx), ...
    'gpenmpcNative:TaskWindClock','future_source_refused');
new=make();reject(@()new.observe(r.global_time_s(end)+1,b1,rx,rx), ...
    'gpenmpcNative:TaskWindPhase','out_of_task_time_refused');
new=make();new.observe(.01,b2,rx,rx);
reject(@()new.observe(.02,b1,rx+uint64(1),rx+uint64(1)), ...
    'gpenmpcNative:TaskWindSourceOrder','source_generation_reverse');
new=make();[edge,~]=new.observe(0,b1,rx,rx+e.maximum_runtime_age_ns);
check('existing_age_inclusive_boundary',edge.rx_ns==rx&&edge.valid);
new=make();[last,lastReceipt]=new.observe(r.global_time_s(end),b1,rx,rx);
check('last_task_row_without_extension',lastReceipt.selected_row==numel(r.global_time_s) ...
    &&isequal(last.estimate_xy_mps,r.wind_estimate_xy_mps(end,:).'));
report=struct('passed',all([checks.pass]),'checks',checks,'total',numel(checks), ...
    'task_sha256',taskSha,'source_path',char(source),'task_rows',numel(r.global_time_s), ...
    'scope','SAVED_CANONICAL_TASK_ESTIMATOR_INPUT_WITH_ORIGINAL_PRIVATE_SOURCE_FIXTURES', ...
    'COM_open',0,'board_actions',0,'solver_calls',0,'plant_instances',0,'socket_count',0);
save(fullfile(outputRoot,'RAW.mat'),'report','w','a','w2','a2','w3','a3','e');
f=fopen(fullfile(outputRoot,'RESULT.json'),'w');assert(f>=0);c=onCleanup(@()fclose(f));
fprintf(f,'%s\n',jsonencode(report,PrettyPrint=true));clear c
disp(jsonencode(struct('passed',report.passed,'checks',numel(checks),'board_actions',0)));
assert(report.passed);
    function check(name,ok)
        checks(end+1)=struct('name',name,'pass',logical(ok));
        assert(ok,'gpenmpcNative:TaskWindTest','%s',name);
    end
    function reject(fun,identifier,name)
        caught=false;try,fun();catch ex,caught=strcmp(ex.identifier,identifier);end
        check(name,caught);
    end
end
function b=readbin(p)
f=fopen(p,'rb');assert(f>=0);c=onCleanup(@()fclose(f));b=fread(f,Inf,'*uint8');
end
function t=hex(b),t=upper(reshape(dec2hex(b,2).',1,[]));end
