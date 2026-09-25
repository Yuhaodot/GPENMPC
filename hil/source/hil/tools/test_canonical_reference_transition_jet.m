function report=test_canonical_reference_transition_jet(outputRoot)
% Test reference jets and jerk-screen boundaries.
arguments
    outputRoot (1,1) string
end
build=string(fileparts(fileparts(mfilename('fullpath'))));
parent=gpenmpc_external_path('native_visual_host_runtime');
addpath(fullfile(parent,'src'),'-begin');
addpath(fullfile(build,'host_runtime'),'-begin');
a=gpenmpcNative.loadCanonicalAssets();
task=fullfile(build,'task_packages','cambridge_canonical', ...
    'MU_CAMBRIDGE_MA_02__CANONICAL_PHYSICAL_TASK.mat');
taskSha='B876546B468265F84BD028AC549B06EF8ADDCB25CBA09CF389A08F4B21290A5F';
b=gpenmpcNative.loadRflyCanonicalDeliveryTask(task,taskSha);
assert(~isfolder(outputRoot));mkdir(outputRoot);
raw={};checks=struct('name',{},'pass',{});exact=0;maximum=0;fractions=[];
for leg=1:5
    tr=b.legs{leg}.trajectory;duration=tr.total_duration_s;
    phases=unique([0,.009,.012345,1.317,duration/2,duration-.007,duration]);
    for k=1:numel(phases)
        q=phases(k);if q<0||q>duration,continue;end
        for scenario=1:7
            rates=[.75,1,1.25,.75,1,1.25,1];
            dts=[.002,.009,.01,.04,0,NaN,-.01];
            prevA=.13;targetA=-.21;prevI=[.01;-.02;.03];targetF=[.3;-.2;.15];
            if scenario==1,prevA=0;targetA=0;prevI=zeros(3,1);targetF=zeros(3,1);end
            compare(tr,q,rates(scenario),prevA,targetA,prevI,targetF, ...
                dts(scenario),a.enmpc.reference_transition_jerk_limit_mps3,leg,scenario);
        end
    end
end
% Evaluate the initial ascent analytically.
[tr,~]=gpenmpcNative.bindRflyCanonicalInitialTakeoffTrajectory(b.legs{1},zeros(3,1));
for q=[0,3.14159,19.99999,20,20.00001,24.99999,25,25.00001]
    compare(tr,q,1,.07,-.1,[.03;-.01;.02],[.2;.05;-.1], ...
        .009,a.enmpc.reference_transition_jerk_limit_mps3,1,8);
end
check('all_original_transition_fields_exact',exact==numel(raw)&&maximum==0);
check('five_actual_legs_and_existing_analytic_prefix', ...
    all(ismember(1:5,cellfun(@(r)r.leg,raw)))&&any(cellfun(@(r)r.scenario==8,raw)));
check('fraction_one_and_limited_and_zero_branches', ...
    any(fractions==1)&&any(fractions>0&fractions<1)&&any(fractions==0));
check('invalid_dt_retains_original_zero_fraction_semantics', ...
    all(cellfun(@(r)isfinite(r.dt_s)&&r.dt_s>0||r.result.fraction==0,raw)));
check('no_source_or_plant_or_hardware_dependency', ...
    ~contains(fileread(which('gpenmpcNative.canonicalReferenceTransitionFromJet')),'mavlinkio') ...
    &&~contains(fileread(which('gpenmpcNative.canonicalReferenceTransitionFromJet')),'serialport'));
save(fullfile(outputRoot,'RAW.mat'),'raw','checks','-v7.3');
report=struct('status','PASS_HOST_ONLY_EXACT_REFERENCE_TRANSITION_FROM_ORIGINAL_JET', ...
    'checks',checks,'test_count',numel(checks),'pass_count',sum([checks.pass]), ...
    'case_count',numel(raw),'all_transition_fields_exact_cases',exact,'maximum_numeric_error',maximum, ...
    'task_sha256',taskSha,'parent_source',which('gpenmpcJerkBoundedReferenceTransition'), ...
    'parent_sha256',gpenmpcNative.fileSha256(which('gpenmpcJerkBoundedReferenceTransition')), ...
    'extracted_source_sha256',gpenmpcNative.fileSha256(which('gpenmpcNative.canonicalReferenceTransitionFromJet')), ...
    'reference_transition_jerk_limit_mps3',a.enmpc.reference_transition_jerk_limit_mps3, ...
    'input_scope','REAL_SAVED_TRAJECTORY_JETS__SYNTHETIC_PHASE_RATE_AND_OUTER_TARGETS', ...
    'new_trajectory_fit',false,'solver_calls',0,'plant_instances',0, ...
    'clock_or_reference_generation_authority',false,'hardware_actions',0,'com_open',0);
save(fullfile(outputRoot,'RAW.mat'),'raw','report','-v7.3');
f=fopen(fullfile(outputRoot,'RESULT.json'),'w','n','UTF-8');assert(f>=0);
c=onCleanup(@()fclose(f));fprintf(f,'%s\n',jsonencode(report,PrettyPrint=true));clear c
disp(jsonencode(report));
    function compare(tr,q,rate,prevA,targetA,prevI,targetF,dt,limit,leg,scenario)
        jet=zeros(3,4);
        for order=0:3,jet(:,order+1)=gpenmpcEvaluateTrajectoryDerivative(tr,q,order);end
        gold=gpenmpcJerkBoundedReferenceTransition(tr,q,rate,prevA,targetA,prevI,targetF,dt,limit);
        got=gpenmpcNative.canonicalReferenceTransitionFromJet(jet,rate,prevA,targetA,prevI,targetF,dt,limit);
        err=compareFields(got,gold);maximum=max(maximum,err);
        yes=isequaln(orderfields(got),orderfields(gold));exact=exact+double(yes);
        raw{end+1}=struct('leg',leg,'scenario',scenario,'phase_s',q,'dt_s',dt, ...
            'jet',jet,'progress_rate',rate,'previous_phase_acceleration',prevA, ...
            'target_phase_acceleration',targetA,'previous_outer_i',prevI,'target_outer_f',targetF, ...
            'result',got,'parent_result',gold,'exact',yes,'maximum_error',err); %#ok<AGROW>
        fractions(end+1)=got.fraction; %#ok<AGROW>
        if ~yes,save(fullfile(outputRoot,'FAILURE.mat'),'raw');end
        assert(yes,'gpenmpcNative:ReferenceJetMismatch','Actual trajectory/transition changed in case %d.',numel(raw));
    end
    function check(name,ok)
        checks(end+1)=struct('name',name,'pass',logical(ok)); %#ok<AGROW>
        assert(ok,'gpenmpcNative:ReferenceJetTest','%s',name);
    end
end
function errorValue=compareFields(a,b)
errorValue=0;names=fieldnames(b);
assert(all(isfield(a,names))&&numel(fieldnames(a))==numel(names));
for k=1:numel(names)
    x=a.(names{k});y=b.(names{k});
    if isstruct(x),err=compareFields(x,y);else,err=max(abs(x(:)-y(:)),[],'omitnan');end
    if isempty(err),err=0;end;errorValue=max(errorValue,err);
end
end
