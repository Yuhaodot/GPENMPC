function report=test_rfly_canonical_initial_takeoff_reference(outputRoot)
% Analyze the saved takeoff reference offline.
build=string(fileparts(fileparts(mfilename('fullpath'))));addpath(fullfile(build,'host_runtime'));
source=fullfile(build,'host_runtime','+gpenmpcNative','bindRflyCanonicalInitialTakeoffTrajectory.m');
sourceBefore=sha(source);testSource=string(mfilename('fullpath'))+".m";testBefore=sha(testSource);
assert(~isfolder(outputRoot));mkdir(outputRoot);
taskPath=fullfile(build,'task_packages','cambridge_canonical','MU_CAMBRIDGE_MA_02__CANONICAL_PHYSICAL_TASK.mat');
taskSha='B876546B468265F84BD028AC549B06EF8ADDCB25CBA09CF389A08F4B21290A5F';
b=gpenmpcNative.loadRflyCanonicalDeliveryTask(taskPath,taskSha);leg=b.legs{1};original=leg;
ground=[0;0;0]; % Explicit HOST fixture in task UP frame, NOT a measured ground anchor.
checks=struct('name',{},'pass',{});negative=struct('name',{},'identifier',{});
[tr,receipt]=gpenmpcNative.bindRflyCanonicalInitialTakeoffTrajectory(leg,ground);
first=receipt.original_first_jet_columns_p_v_a_j;
check('actual_saved_first_zero_velocity_acceleration',isequal(first(:,2:3),zeros(3,2)));
check('actual_saved_first_nonzero_horizontal_jerk',norm(first(1:2,4))>0&&first(3,4)==0);
S=[0,0,0,0,35,-84,70,-20];H=[0,0,0,0,-1/6,1/2,-1/2,1/6];
singleUnit=(first(:,1)-ground)*S+first(:,4)*25^3*H;singleUnit(:,1)=singleUnit(:,1)+ground;
riseUnit=(first(:,1)-ground)*S;riseUnit(:,1)=riseUnit(:,1)+ground;
joinUnit=first(:,4)*5^3*H;joinUnit(:,1)=joinUnit(:,1)+first(:,1);
for k=1:2
    want=riseUnit;if k==2,want=joinUnit;end
    err=max(abs(receipt.coefficients_ascending_unit_time{k}-want),[],'all');
    check(sprintf('segment%d_independent_analytic_seventh_coefficients',k),err<1e-9);
end
single=metrics({singleUnit},25,first(:,1));two=metrics({riseUnit,joinUnit},[20,5],first(:,1));
tau=4/7;shape=tau^4*(1-tau)^3/6;jxy=norm(first(1:2,4));
analytic=struct('terminal_jerk_xy_mps3',jxy,'unit_shape_absolute_maximum',shape, ...
    'unit_time_of_maximum',tau,'single25_lateral_m',jxy*25^3*shape, ...
    'two_stage_lateral_m',jxy*5^3*shape,'two_stage_maximum_phase_s',20+5*tau, ...
    'exact_scale_reduction',125,'formula','H(tau)=-tau^4*(1-tau)^3/6; displacement=j*T^3*H(tau)');
check('analytic_single25_lateral_extremum',abs(single.maximum_horizontal_offset_from_first_m-analytic.single25_lateral_m)<1e-9);
check('analytic_two_stage_lateral_extremum',abs(two.maximum_horizontal_offset_from_first_m-analytic.two_stage_lateral_m)<1e-9);
check('same_jerk_reduction_125_without_changing_total_duration', ...
    abs(single.maximum_horizontal_offset_from_first_m/two.maximum_horizontal_offset_from_first_m-125)<1e-9 ...
    &&receipt.prefix_duration_phase_s==25&&isequal(receipt.durations_phase_s,[20,5]));
boundaryErrors=zeros(3,4);
for order=0:3
    left=zeros(3,1);if order==0,left=ground;end
    rest=zeros(3,1);if order==0,rest=first(:,1);end
    check(sprintf('ground_exact_derivative%d',order),isequal(tr.evaluate_fcn(0,order),left));
    check(sprintf('twenty_exact_derivative%d',order),isequal(tr.evaluate_fcn(20,order),rest));
    check(sprintf('twentyfive_exact_original_derivative%d',order),isequal(tr.evaluate_fcn(25,order),first(:,order+1)));
    % Test actual interior polynomials at their endpoints independently of
    % exact endpoint dispatch, so dispatch cannot conceal a polynomial jump.
    a=evaluateUnit(receipt.coefficients_ascending_unit_time{1},1,order,20);
    b0=evaluateUnit(receipt.coefficients_ascending_unit_time{2},0,order,5);
    b1=evaluateUnit(receipt.coefficients_ascending_unit_time{2},1,order,5);
    boundaryErrors(:,order+1)=[norm(a-rest,Inf);norm(b0-rest,Inf);norm(b1-first(:,order+1),Inf)];
    check(sprintf('all_polynomial_endpoints_C%d',order),max(boundaryErrors(:,order+1))<1e-9);
end
query=0:.002:25;raw=cell(4,1);dense=cell(4,1);
for order=0:3
    raw{order+1}=tr.evaluate_fcn(query,order);want=zeros(3,numel(query));
    for k=1:numel(query)
        if query(k)<=20,want(:,k)=evaluateUnit(riseUnit,query(k)/20,order,20);
        else,want(:,k)=evaluateUnit(joinUnit,(query(k)-20)/5,order,5);end
    end
    check(sprintf('full_prefix_independent_analytic_derivative%d',order),max(abs(raw{order+1}-want),[],'all')<1e-9);
    dense{order+1}=max(vecnorm(raw{order+1},2,1));
end
check('analytic_speed_extremum_bounds_dense_prefix',two.maximum_norm_by_derivative(2)>=dense{2}-1e-9 ...
    &&two.maximum_norm_by_derivative(2)-dense{2}<1e-6);
check('analytic_acceleration_extremum_bounds_dense_prefix',two.maximum_norm_by_derivative(3)>=dense{3}-1e-9 ...
    &&two.maximum_norm_by_derivative(3)-dense{3}<1e-6);
check('analytic_jerk_extremum_bounds_dense_prefix',two.maximum_norm_by_derivative(4)>=dense{4}-1e-9 ...
    &&two.maximum_norm_by_derivative(4)-dense{4}<1e-6);
check('transition_is_not_falsely_labelled_hold', ...
    norm(tr.evaluate_fcn(22.5,0)-first(:,1))>0 ...
    &&norm(tr.evaluate_fcn(22.5,1))>0&&contains(receipt.preparation_segments{2},'NOT_HOLD'));
formalQ=25+leg.local_time_s.';phase=formalQ-25;
formalTimeRoundoff=max(abs(phase-leg.local_time_s.'));
for order=0:3
    actual=tr.evaluate_fcn(formalQ,order);
    want=leg.trajectory.evaluate_fcn(phase,order);
    check(sprintf('every_formal_row_direct_original_evaluator_bits_derivative%d',order), ...
        isequal(typecast(actual(:),'uint64'),typecast(want(:),'uint64')));
    % Report q-25 cancellation error separately while preserving the reference arrays.
end
check('all_formal_arrays_and_original_handle_unchanged',isequaln(leg,original));
check('total_duration_and_phase_names_separate',tr.total_duration_s==25+leg.trajectory.total_duration_s ...
    &&tr.canonical_task_phase_offset_s==25&&contains(receipt.formal_task_phase,'ONLY_WHEN_q_GE_25'));
offset=[.3;-.2;.1];[offsetTr,offsetReceipt]=gpenmpcNative.bindRflyCanonicalInitialTakeoffTrajectory(leg,offset);
check('nonzero_explicit_ground_frame_anchor',isequal(offsetTr.evaluate_fcn(0,0),offset));
for order=0:3
    check(sprintf('nonzero_ground_does_not_move_formal_jet%d',order), ...
        isequal(offsetTr.evaluate_fcn(25,order),first(:,order+1)));
end
check('anchor_is_not_claimed_authenticated',~offsetReceipt.ground_authority_proven&&~offsetReceipt.flight_feasibility_proven);
reject(@()gpenmpcNative.bindRflyCanonicalInitialTakeoffTrajectory(b.legs{2},ground),'gpenmpcNative:InitialTakeoffLeg','not_relaunch_leg');
reject(@()gpenmpcNative.bindRflyCanonicalInitialTakeoffTrajectory(leg,[0;0;10]),'gpenmpcNative:InitialTakeoffAscent','no_ascent_geometry');
reject(@()tr.evaluate_fcn(NaN,0),'gpenmpcNative:InitialTakeoffQuery','nonfinite_phase');
reject(@()tr.evaluate_fcn(0,4),'gpenmpcNative:InitialTakeoffQuery','unsupported_derivative');
bad=leg;bad.trajectory.evaluate_fcn=@(~,~)[NaN;0;0];
reject(@()gpenmpcNative.bindRflyCanonicalInitialTakeoffTrajectory(bad,ground),'gpenmpcNative:InitialTakeoffJet','nonfinite_original_jet');
check('no_replacement_of_canonical_leg_duration_by_wall_time', ...
    isequal(tr.evaluate_fcn(-1,0),ground) ...
    &&isequal(tr.evaluate_fcn(tr.total_duration_s+1,0),leg.trajectory.evaluate_fcn(leg.trajectory.total_duration_s,0)));
% Inspect the PASSPORT-bound effective configuration.
passportPath=fullfile(gpenmpcNative.canonicalAssetRoot(),'binding','execution.json');
passport=jsondecode(fileread(passportPath));
configPath=fullfile(gpenmpcNative.canonicalAssetRoot(),passport.canonical_paths.effective_config_mat);
check('effective_configuration_matches_current_passport',strcmpi(sha(configPath),passport.expected_sha256.effective_configuration_mat));
effective=load(configPath);
fields=constraintFields(effective,'effective');missionFields=constraintFields(b.task.mission_config,'physicalTask.mission_config');
profilePath=fullfile(gpenmpcNative.canonicalAssetRoot(),'method_source','matlab','simulink','assets','M600_PLATFORM_PROFILE.json');
profile=jsondecode(fileread(profilePath));
formalMax=zeros(1,4);formalArrays={leg.position_up_m,leg.velocity_mps,leg.acceleration_mps2,leg.jerk_mps3};
wholeArrays={b.task.reference.position_m,b.task.reference.velocity_mps,b.task.reference.acceleration_mps2,b.task.reference.jerk_mps3};
wholeMax=zeros(1,4);
for order=1:4
    formalMax(order)=max(vecnorm(formalArrays{order},2,2));
    wholeMax(order)=max(vecnorm(wholeArrays{order},2,2));
end
check('helper_and_test_sources_unchanged',strcmpi(sourceBefore,sha(source))&&strcmpi(testBefore,sha(testSource)));
report=struct('passed',all([checks.pass]),'total',numel(checks),'checks',checks,'negative_cases',negative, ...
    'task_path',taskPath,'task_sha256',taskSha,'actual_first_jet',first, ...
    'single25_candidate',single,'selected20_plus5_candidate',two,'analytic_lateral_comparison',analytic, ...
    'coefficient_boundary_errors',boundaryErrors,'formal_source_rows_tested',numel(formalQ), ...
    'formal_time_add_subtract_binary64_max_s',formalTimeRoundoff, ...
    'formal_reference_comparison','BITWISE_ORIGINAL_EVALUATOR_AT_SAME_q_MINUS_25__RAW_TASK_ARRAYS_UNCHANGED', ...
    'first_leg_max_norm_p_v_a_j',formalMax,'whole_task_max_norm_p_v_a_j',wholeMax, ...
    'configuration_path',configPath,'configuration_sha256',sha(configPath), ...
    'configuration_named_limits',{fields},'mission_named_limits',{missionFields}, ...
    'profile_path',profilePath,'profile_sha256',sha(profilePath),'profile_limits',profile.limits, ...
    'profile_limit_scope','PROFILE_MANUFACTURER_AND_SOFTWARE_DOMAIN_VALUES', ...
    'existing_limit_interpretation',struct( ...
        'rollout_source',gpenmpc_external_path('canonical_rollout_source'), ...
        'runtime_chain','CANONICAL_PASSPORT_BOUND_REFERENCE_ROLLOUT', ...
        'velocity_screen_line329','PREDICTED_VELOCITY_TRACKING_ERROR_NOT_REFERENCE_SPEED', ...
        'acceleration_screen_lines317_330','PREDICTED_ACCELERATION_MINUS_EXISTING_RESERVED_AUTHORITY_NOT_REFERENCE_ACCELERATION_ALONE', ...
        'jerk_screen_line331','ROLLOUT_JERK_NOT_A_CERTIFICATE_FOR_UNEXECUTED_REFERENCE', ...
        'transition_jerk_lines127_130','RUNTIME_PHASE_RATE_ACCELERATION_AND_CORRECTION_TRANSITION', ...
        'tilt_airspeed_lines334_335','DESIRED_FORCE_TILT_AND_WIND_RELATIVE_AIRSPEED_REQUIRE_ACTUAL_CONTEXT'), ...
    'reference_safety_admission_evaluated',false, ...
    'constraint_scope','CURRENT_CONFIG_FIELDS_REPORTED_BY_EXACT_PATH__NO_TRUTH_ABORT_OR_OBJECTIVE_SCALE_REPURPOSED_AS_REFERENCE_GATE', ...
    'initial_origin_point5_scope','INITIAL_ESTIMATOR_ORIGIN_CHECK', ...
    'changed_parameter','PREPARATION_REFERENCE_SHAPE_ONLY__TIMES_RETAINED_20_AND_25_SECONDS', ...
    'selection_reason','REDUCE_JERK_INDUCED_HORIZONTAL_PREACTION_125_FOLD_WHILE_RETAINING_TOTAL_25_SECOND_PHASE', ...
    'numerical_tolerances','1e-9_BOUNDARY_ALGEBRA_AND_1e-6_DENSE_EXTREMUM_APPROXIMATION__NOT_FLIGHT_SAFETY_THRESHOLDS', ...
    'binding_receipt',receipt,'ground_anchor_input','EXPLICIT_HOST_FIXTURE_NOT_ACTUAL_EKF_OBSERVATION', ...
    'source_path',source,'source_sha256',sha(source),'test_source',testSource,'test_sha256',sha(testSource), ...
    'sources_unchanged',true,'live_integration_proven',false, ...
    'solver_calls',0,'model_runs',0,'COM_opens',0,'board_actions',0);
save(fullfile(outputRoot,'RAW.mat'),'report','query','raw','singleUnit','riseUnit','joinUnit','receipt');
f=fopen(fullfile(outputRoot,'RESULT.json'),'w');assert(f>=0);c=onCleanup(@()fclose(f));
fprintf(f,'%s\n',jsonencode(report,PrettyPrint=true));clear c
disp(jsonencode(struct('passed',report.passed,'checks',report.total,'single25',single,'two_stage',two)));
    function check(name,ok)
        checks(end+1)=struct('name',name,'pass',logical(ok));
        assert(ok,'gpenmpcNative:InitialTakeoffTest','%s',name);
    end
    function reject(fn,id,name)
        actual='';try,fn();catch ex,actual=ex.identifier;end
        negative(end+1)=struct('name',name,'identifier',actual);check(name,strcmp(actual,id));
    end
end
function result=metrics(units,durations,firstPosition)
peaks=zeros(numel(units),4);times=peaks;xy=zeros(1,numel(units));zmin=xy;zmax=xy;offset=0;
for segment=1:numel(units)
    unit=units{segment};duration=durations(segment);
    for order=0:3
        coeff=derivative(unit,order)/duration^order;
        [peaks(segment,order+1),t]=normMaximum(coeff);times(segment,order+1)=offset+t*duration;
    end
    delta=unit(1:2,:);delta(:,1)=delta(:,1)-firstPosition(1:2);xy(segment)=normMaximum(delta);
    zr=unit(3,:);critical=realRoots01(derivative(zr,1));values=zr*(critical.^(0:numel(zr)-1)).';
    zmin(segment)=min(values);zmax(segment)=max(values);offset=offset+duration;
end
[mx,idx]=max(peaks,[],1);at=zeros(1,4);for k=1:4,at(k)=times(idx(k),k);end
result=struct('maximum_norm_by_derivative',mx,'maximum_phase_by_derivative_s',at, ...
    'maximum_horizontal_offset_from_first_m',max(xy),'minimum_height_up_m',min(zmin), ...
    'maximum_height_up_m',max(zmax),'extrema_method','POLYNOMIAL_SQUARED_NORM_DERIVATIVE_REAL_ROOTS_IN_UNIT_INTERVAL_PLUS_ENDPOINTS');
end
function [value,t]=normMaximum(coeff)
squared=zeros(1,2*size(coeff,2)-1);
for axis=1:size(coeff,1),squared=squared+conv(coeff(axis,:),coeff(axis,:));end
candidates=realRoots01(derivative(squared,1));v=coeff*(candidates.^(0:size(coeff,2)-1)).';
[value,index]=max(vecnorm(v,2,1));t=candidates(index);
end
function values=realRoots01(coeff)
last=find(coeff~=0,1,'last');values=[0;1];if isempty(last)||last==1,return;end
r=roots(fliplr(coeff(1:last)));r=real(r(abs(imag(r))<1e-7&real(r)>0&real(r)<1));
values=unique([values;r(:)]);
end
function value=evaluateUnit(coeff,tau,order,duration)
c=derivative(coeff,order)/duration^order;value=c*(tau.^(0:size(c,2)-1)).';
end
function out=derivative(coeff,order)
out=coeff;for k=1:order,out=out(:,2:end).*(1:size(out,2)-1);end
end
function out=constraintFields(value,path)
out=cell(0,1);if ~isstruct(value)||~isscalar(value),return;end
names=fieldnames(value);
for k=1:numel(names)
    name=names{k};v=value.(name);key=[path,'.',name];
    if isstruct(v)&&isscalar(v),out=[out;constraintFields(v,key)]; %#ok<AGROW>
    elseif isnumeric(v)&&numel(v)<=6&&~isempty(regexp(name,'speed|velocity|tilt|accel|jerk|corridor','once'))
        out{end+1,1}=struct('field',key,'value',v); %#ok<AGROW>
    end
end
end
function value=sha(path)
f=fopen(path,'rb');assert(f>=0);c=onCleanup(@()fclose(f));bytes=fread(f,Inf,'*uint8');clear c
d=java.security.MessageDigest.getInstance('SHA-256');d.update(typecast(bytes,'int8'));
value=upper(reshape(dec2hex(typecast(d.digest(),'uint8'),2).',1,[]));
end
