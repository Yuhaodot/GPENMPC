function report=test_local_outer_worker(outputRoot,withPreparation,commandLifetimeNs,withSourceClock,decodeOnly)
% Test asynchronous eNMPC using host C fixture observations.
arguments
    outputRoot (1,1) string
    withPreparation (1,1) logical=false
    commandLifetimeNs (1,1) uint64=uint64(400000000)
    withSourceClock (1,1) logical=false
    decodeOnly (1,1) logical=false
end
build=string(fileparts(fileparts(mfilename('fullpath'))));addpath(fullfile(build,'host_runtime'),'-begin');
if decodeOnly,report=checkRetainedDecodes(build);return;end
addpath(gpenmpc_external_path('native_visual_host_source'),'-end');
a=gpenmpcNative.loadCanonicalAssets();
task=gpenmpcNative.loadRflyCanonicalDeliveryTask(fullfile(build,'task_packages','cambridge_canonical','MU_CAMBRIDGE_MA_02__CANONICAL_PHYSICAL_TASK.mat'), ...
    'B876546B468265F84BD028AC549B06EF8ADDCB25CBA09CF389A08F4B21290A5F');
[trajectory,~]=gpenmpcNative.bindRflyCanonicalInitialTakeoffTrajectory(task.legs{1},zeros(3,1));
if isfolder(outputRoot)
    existing=dir(outputRoot);assert(~any(~[existing.isdir]),'Prior test outputs cannot be overwritten.');
else,mkdir(outputRoot);end
f=fopen(fullfile(gpenmpc_external_path('local_task_input'),'MATCHED_RLS_RLI.bin'),'rb');
assert(f>=0);g=onCleanup(@()fclose(f));b=reshape(fread(f,Inf,'*uint8'),1029,[]);clear g
u=gpenmpcNative.RflyLocalTaskCodec.decode(b(383:end,1));
registered=struct('local_full_inner',true,'identity',u.source.identity,'leg_index',u.leg_index, ...
    'execution_session_sha256',hex(u.execution_session_sha256),'task_sha256',hex(u.task_sha256), ...
    'reference_asset_sha256',hex(u.reference_asset_sha256));
service=gpenmpcNative.RflyLocalOuterService(a,registered,trajectory,uint64(50000000),commandLifetimeNs);
cleanup=onCleanup(@()service.close()); %#ok<NASGU>
% Use synthetic callback arrival times.
t=gpenmpcNative.rflyOriginalHostMonotonicNs();
source=struct('message',b(1:382,1),'original_host_receive_ns',t);
preparationEvents={};preparationState=[];
if withPreparation
    preparationEvents{end+1}=service.beginPreparation(source,b(383:end,1),t);
    preparationTimer=tic;
    while toc(preparationTimer)<60
        r=service.pollPreparation(gpenmpcNative.rflyOriginalHostMonotonicNs());
        preparationEvents{end+1}=r; %#ok<AGROW>
        if ~strcmp(r.status,'PREPARATION_PENDING'),break;end
        pause(.005);
    end
    preparationState=service.status();assert(preparationState.prepared&&~preparationState.failed);
    % Introduce a new fixture callback event.
    t=gpenmpcNative.rflyOriginalHostMonotonicNs();source.original_host_receive_ns=t;
end
first=service.bootstrap(source,b(383:end,1),t,withSourceClock);
events={};timer=tic;
while toc(timer)<60
    now=gpenmpcNative.rflyOriginalHostMonotonicNs();r=service.poll(now);events{end+1,1}=r; %#ok<AGROW>
    s=service.status();if ~s.worker.in_flight,break;end
    pause(.005);
end
firstState=service.status();clockEvidence=struct();
if withSourceClock
    % Existing C fixtures, explicit synthetic HOST callback arrivals only.
    % The scheduling sample is newer; numerical RLS/RLC remains exactly paired.
    f=fopen(fullfile(gpenmpc_external_path('committed_state_reference'),'COMMITTED_RLC1.bin'),'rb');
    assert(f>=0);g=onCleanup(@()fclose(f));cb=reshape(fread(f,Inf,'*uint8'),1398,[]);clear g
    f=fopen(fullfile(build,'rfly_vendor_integration','full_inner_abi','snapshot_wire_fixture','RLS1_SNAPSHOTS.bin'),'rb');
    assert(f>=0);g=onCleanup(@()fclose(f));sb=reshape(fread(f,Inf,'*uint8'),382,[]);clear g
    n=gpenmpcNative.rflyOriginalHostMonotonicNs();
    numerical=struct('message',b(1:382,3),'original_host_receive_ns',n-uint64(20000000));
    closed=struct('message',cb(:,3),'original_host_receive_ns',n-uint64(10000000));
    % Pass the IO owner's verified values to the solver thread.
    numerical.decoded=gpenmpcNative.RflyLocalSnapshotDecoder(numerical.message,numerical.original_host_receive_ns);
    closed.decoded=gpenmpcNative.RflyLocalCommittedDecoder(closed.message,closed.original_host_receive_ns);
    sent=struct('held_input_bytes',b(383:end,3),'original_input_source',numerical, ...
        'held_input_decoded',gpenmpcNative.RflyLocalTaskCodec.decode(b(383:end,3)));
    clockEvidence.retained=service.sample(numerical,closed,sent,true,n,true);
    clockEvidence.before=service.status();
    tick=struct('message',sb(:,34),'original_host_receive_ns',gpenmpcNative.rflyOriginalHostMonotonicNs());
    clockEvidence.not_due=service.serviceSourceClock(tick,true,gpenmpcNative.rflyOriginalHostMonotonicNs());
    tick=struct('message',sb(:,35),'original_host_receive_ns',gpenmpcNative.rflyOriginalHostMonotonicNs());
    clockEvidence.due=service.serviceSourceClock(tick,true,gpenmpcNative.rflyOriginalHostMonotonicNs());
    clockEvidence.after_submit=service.status();
    wait=tic;
    while toc(wait)<60
        r=service.poll(gpenmpcNative.rflyOriginalHostMonotonicNs());events{end+1}=r;
        state=service.status();if ~state.worker.in_flight,break;end
        pause(.005);
    end
    clockEvidence.numeric_anchor=numerical.original_host_receive_ns;
    clockEvidence.numeric_sample_us=uint64(1018000);
    if state.failed||state.worker.errors>0
        save(fullfile(outputRoot,'WORKER_ERROR.mat'),'state','events','clockEvidence');
        error('gpenmpcTest:OriginalWorkerFailure','%s',state.worker.last_error);
    end
    % A new tick must not renew an old numerical observation.
    expiredNow=gpenmpcNative.rflyOriginalHostMonotonicNs()+commandLifetimeNs;
    tick=struct('message',sb(:,36),'original_host_receive_ns',expiredNow);
    clockEvidence.expired=service.serviceSourceClock(tick,true,expiredNow);
    clockEvidence.after_expired=service.status();
    wrong=tick;wrong.original_host_receive_ns=expiredNow+uint64(1);
    rejectedClock=false;
    try,service.serviceSourceClock(wrong,true,expiredNow);catch ex,rejectedClock=strcmp(ex.identifier,'gpenmpcNative:LocalOuterClockSource');end
    clockEvidence.future_rejected=rejectedClock;
end
beforeClose=service.status();service.close();afterClose=service.status();
ageBoundaryChecks=checkSelectedMailboxAgeBoundaries(build);
checks=struct('name',{},'pass',{});
check('selected_mailbox_independent_age_and_work_boundaries',ageBoundaryChecks==8 ...
    &&strcmpi(beforeClose.worker_construction.worker_source, ...
        fullfile(build,'host_runtime','+gpenmpcNative','OuterMailboxWorker.m')));
check('slow_outer_budget_does_not_extend_fast_bootstrap_or_command', ...
    beforeClose.bootstrap_observation_age_ns==uint64(50000000) ...
    &&beforeClose.flight_observation_age_ns==min(uint64(320000000),commandLifetimeNs-uint64(280000000)));
check('actual_original_solver_results',beforeClose.outer_submissions==1+double(withSourceClock)&&beforeClose.completed_original_solver_results==1+double(withSourceClock) ...
    &&beforeClose.worker.maximum_in_flight==1&&beforeClose.worker.errors==0);
check('deadline_never_extended_for_cold_worker',beforeClose.fresh_results+beforeClose.deadline_discards==1+double(withSourceClock));
check('no_HOST_inner_phase_or_plant_step',beforeClose.host_inner_steps==0&&beforeClose.plant_steps==0);
check('original_anchor_and_lease_not_renewed',~isempty(firstState.last_command) ...
    &&firstState.last_command.original_host_source_receive_ns==t ...
    &&firstState.last_command.original_sample_us==u.source.sample_us ...
    &&firstState.last_command.original_expiry_ns==t+commandLifetimeNs);
if withSourceClock
    check('retaining_record_does_not_dispatch_or_advance_clock',~clockEvidence.retained.outer_submitted ...
        &&clockEvidence.before.clock.leg_flight_time_ns==0);
    check('actual_new_source_triggers_original_300ms_slot',~clockEvidence.not_due.outer_submitted ...
        &&clockEvidence.due.outer_submitted&&clockEvidence.due.dispatch_slot_index==1 ...
        &&clockEvidence.due.schedule_source_generation==35&&clockEvidence.due.numerical_source_generation==3);
    check('new_clock_does_not_retimestamp_numerical_input',beforeClose.last_command.generation==2 ...
        &&beforeClose.last_command.original_host_source_receive_ns==clockEvidence.numeric_anchor ...
        &&beforeClose.last_command.original_sample_us==clockEvidence.numeric_sample_us ...
        &&beforeClose.last_command.original_expiry_ns==clockEvidence.numeric_anchor+commandLifetimeNs);
    check('expired_numerical_record_cannot_be_renewed',~clockEvidence.expired.outer_submitted ...
        &&strcmp(clockEvidence.expired.status,'NO_VALID_MATCHED_OBSERVATION') ...
        &&clockEvidence.after_expired.outer_submissions==2);
    check('future_clock_source_rejected',clockEvidence.future_rejected);
end
rejected=false;
try
    gpenmpcNative.RflyLocalOuterService(a,registered,trajectory,uint64(50000000),uint64(700000001));
catch
    rejected=true;
end
check('beyond_prospective_700ms_bound_rejected_before_worker',rejected);
check('no_second_worker_after_close',afterClose.closed);
yes=false;try,service.bootstrap(source,b(383:end,1),gpenmpcNative.rflyOriginalHostMonotonicNs());catch,yes=true;end
check('no_hidden_leg_reset_or_reentry',yes);
if withPreparation
    check('one_same_worker_generation_zero_rehearsal',preparationState.preparation_calls==1 ...
        &&preparationState.outer_submissions==0&&preparationState.prepared);
    check('preparation_cannot_commit_command_or_warm_start',isempty(preparationState.last_command) ...
        &&isempty(preparationState.clock)&&preparationState.preparation.numerical_state_unchanged);
    check('cold_timing_preserved_not_declared_fresh_control',preparationState.preparation.solver_deadline_s==.28 ...
        &&~preparationState.preparation.command_committed);
end
report=struct('passed',all([checks.pass]),'checks',checks,'test_count',numel(checks), ...
    'actual_outer_submissions',beforeClose.outer_submissions,'actual_completed_original_solver_results',beforeClose.completed_original_solver_results, ...
    'fresh_results',beforeClose.fresh_results,'deadline_discards',beforeClose.deadline_discards,'elapsed_s',toc(timer), ...
    'claim','Outer-worker integration and deadline-expiry tests.', ...
    'command_lifetime_ns',commandLifetimeNs,'preparation',preparationState, ...
    'hardware_actions',0,'host_inner_steps',0,'plant_steps',0);
save(fullfile(outputRoot,'RAW.mat'),'first','events','beforeClose','afterClose','report','preparationEvents','clockEvidence');
f=fopen(fullfile(outputRoot,'RESULT.json'),'wt');g=onCleanup(@()fclose(f));fprintf(f,'%s\n',jsonencode(report,PrettyPrint=true));clear g
disp(jsonencode(report));assert(report.passed);
    function check(n,yes),checks(end+1)=struct('name',n,'pass',logical(yes));end
end
function s=hex(b),s=upper(reshape(dec2hex(b,2).',1,[]));end

function count=checkSelectedMailboxAgeBoundaries(build)
% Test production predicates with synthetic boundary times.
% The preceding solver calls exercise the worker mailbox separately.
text=fileread(fullfile(build,'host_runtime','+gpenmpcNative','OuterMailboxWorker.m'));
begin=strfind(text,'expired=nowNs<obj.SubmittedNs');
finish=strfind(text,'if expired && ~obj.ExpiryCounted');
assert(isscalar(begin)&&isscalar(finish));predicate=text(begin:finish-1);
obj=struct('DeadlineNs',280000000,'ObservationResultNs',600000000, ...
    'SubmittedNs',1000000000,'SnapshotNs',878269000);
nowNs=1158270000;eval(predicate);assert(~expired); % 121.731ms age +158.270ms work.
obj.ObservationResultNs=280000000;eval(predicate);assert(expired); % Retained legacy rejection.
obj.ObservationResultNs=600000000;obj.SnapshotNs=680000000;
nowNs=1280000000;eval(predicate);assert(~expired); % Exact320+280ms endpoints.
nowNs=1280000001;eval(predicate);assert(expired); % Work deadline remains strict.
obj.SnapshotNs=679999999;nowNs=1280000000;eval(predicate);assert(expired); % Total age.
nowNs=999999999;eval(predicate);assert(expired); % Regressed HOST time.
begin=strfind(text,'assert(isfield(snapshot,''snapshot_timestamp_ns'')');
finish=strfind(text,'obj.SubmittedNs=nowNs;obj.SnapshotNs=snapshot.snapshot_timestamp_ns;');
assert(isscalar(begin)&&isscalar(finish));admission=text(begin:finish-1);
obj.ObservationAdmissionNs=320000000;nowNs=1000000000;
snapshot=struct('snapshot_timestamp_ns',680000000);eval(admission);
snapshot.snapshot_timestamp_ns=679999999;rejected=false;try,eval(admission);catch,rejected=true;end
assert(rejected);count=8;
end

function report=checkRetainedDecodes(build)
% Execute only the changed production blocks, no worker/socket/board or file
% outputs. Raw-only legacy callers and exact original bytes/time are checked.
text=fileread(fullfile(build,'host_runtime','+gpenmpcNative','RflyLocalOuterService.m'));
begin=strfind(text,'% BEGIN_OUTER_RETAINED_SOURCE');finish=strfind(text,'% END_OUTER_RETAINED_SOURCE');
assert(numel(begin)==2&&numel(finish)==2);
sourceCode=text(begin(1):finish(1)-1);
assert(strcmp(sourceCode,text(begin(2):finish(2)-1)));
begin=strfind(text,'% BEGIN_OUTER_RETAINED_COMMIT');finish=strfind(text,'% END_OUTER_RETAINED_COMMIT');
assert(isscalar(begin)&&isscalar(finish));commitCode=text(begin:finish-1);
f=fopen(fullfile(gpenmpc_external_path('local_task_input'),'MATCHED_RLS_RLI.bin'),'rb');
assert(f>=0);closeFile=onCleanup(@()fclose(f));pairs=reshape(fread(f,Inf,'*uint8'),1029,[]);clear closeFile
f=fopen(fullfile(gpenmpc_external_path('committed_state_reference'),'COMMITTED_RLC1.bin'),'rb');
assert(f>=0);closeFile=onCleanup(@()fclose(f));commits=reshape(fread(f,Inf,'*uint8'),1398,[]);clear closeFile
sources=cell(3,1);records=cell(3,1);checks=0;s=[];c=[];
for k=1:3
    source=struct('message',pairs(1:382,k),'original_host_receive_ns',uint64(1000000000+k*10000000));
    committed=struct('message',commits(:,k),'original_host_receive_ns',source.original_host_receive_ns+uint64(1));
    expectedS=gpenmpcNative.RflyLocalSnapshotDecoder(source.message,source.original_host_receive_ns);
    expectedC=gpenmpcNative.RflyLocalCommittedDecoder(committed.message,committed.original_host_receive_ns);
    eval(sourceCode);eval(commitCode);assert(isequaln(s,expectedS)&&isequaln(c,expectedC));checks=checks+1;
    source.decoded=expectedS;committed.decoded=expectedC;
    eval(sourceCode);eval(commitCode);assert(isequaln(s,expectedS)&&isequaln(c,expectedC));checks=checks+1;
    sources{k}=source;records{k}=committed;
end
for k=1:4
    source=sources{1};committed=records{1};
    if k==1,source.message(132)=bitxor(source.message(132),uint8(1));end
    if k==2,source.original_host_receive_ns=source.original_host_receive_ns+uint64(1);end
    if k==3,committed.message(132)=bitxor(committed.message(132),uint8(1));end
    if k==4,committed.original_host_receive_ns=committed.original_host_receive_ns+uint64(1);end
    rejected=false;
    try,eval(sourceCode);eval(commitCode);catch ex
        rejected=ismember(ex.identifier,{'gpenmpcNative:LocalOuterRetainedSource','gpenmpcNative:LocalOuterRetainedCommit'});
    end
    assert(rejected);checks=checks+1;
end
% Same alternating old-history/new-source pattern; diagnostic, not WCET.
before=zeros(12,1);after=before;
for j=1:12
    k=mod(j-1,3)+1;source=sources{k};committed=records{k};t=tic;
    expectedS=gpenmpcNative.RflyLocalSnapshotDecoder(source.message,source.original_host_receive_ns);
    expectedC=gpenmpcNative.RflyLocalCommittedDecoder(committed.message,committed.original_host_receive_ns);
    before(j)=toc(t);t=tic;eval(sourceCode);eval(commitCode);after(j)=toc(t);
    assert(isequaln(s,expectedS)&&isequaln(c,expectedC));
end
report=struct('passed',true,'affected_checks',checks,'alternating_pairs',12, ...
    'original_decode_median_ms',1000*median(before),'retained_median_ms',1000*median(after), ...
    'includes_eval_overhead',true,'not_live_timing_guarantee',true, ...
    'original_times_and_numerical_fields_unchanged',true,'board_actions',0,'solver_calls',0,'new_workers',0);
disp(jsonencode(report));
end
