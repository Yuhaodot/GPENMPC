function [trajectory,receipt]=bindRflyCanonicalInitialTakeoffTrajectory(leg,groundPositionUpM)
% Bind one takeoff trajectory for outer and inner reference queries.
% The caller supplies a verified ground position in the leg's registered UP frame.
% 0..20 s: rest-to-rest ascent; 20..25 s: endpoint-jet transition.
% For q>=25 s, query the canonical leg at q-25.
arguments
    leg (1,1) struct
    groundPositionUpM (3,1) double {mustBeReal,mustBeFinite}
end
assert(isfield(leg,'meta')&&isfield(leg.meta,'leg_index')&&leg.meta.leg_index==1 ...
    &&isfield(leg,'local_time_s')&&leg.local_time_s(1)==0 ...
    &&isfield(leg,'trajectory')&&isfield(leg.trajectory,'evaluate_fcn'), ...
    'gpenmpcNative:InitialTakeoffLeg','Only the actual first leg at original task phase zero is supported.');
nominal=leg.trajectory;
assert(isfinite(nominal.total_duration_s)&&nominal.total_duration_s>0, ...
    'gpenmpcNative:InitialTakeoffDuration','A finite original first-leg duration is required.');
distributionRoot=fileparts(fileparts(fileparts(fileparts(fileparts(mfilename('fullpath'))))));
canonical=fullfile(distributionRoot,'assets','canonical','method_source','matlab','enmpc');
sources={fullfile(canonical,'gpenmpcPrepareTrajectoryDerivatives.m'), ...
    fullfile(canonical,'gpenmpcEvaluateTrajectoryDerivative.m')};
hashes={'D5E2309E110FD2247D4ED2B6DE44BB53D1B9B613531AE91C2243A08824B94650', ...
    '2EB33AFC4DFC300FE8B9C43E96755866613FD784E5928D7ECFB4E8DFEE88FA78'};
for k=1:2
    assert(strcmpi(sha(sources{k}),hashes{k}),'gpenmpcNative:InitialTakeoffSource', ...
        'The retained canonical derivative interface differs.');
end
addpath(canonical,'-begin');
assert(strcmpi(which('gpenmpcPrepareTrajectoryDerivatives'),sources{1}) ...
    &&strcmpi(which('gpenmpcEvaluateTrajectoryDerivative'),sources{2}), ...
    'gpenmpcNative:InitialTakeoffShadow','Canonical derivative source is shadowed.');
first=zeros(3,4);
for order=0:3
    value=nominal.evaluate_fcn(0,order);
    assert(isequal(size(value),[3,1])&&isreal(value)&&all(isfinite(value)), ...
        'gpenmpcNative:InitialTakeoffJet','All four actual first-jet vectors are required.');
    first(:,order+1)=value;
end
assert(first(3,1)>groundPositionUpM(3),'gpenmpcNative:InitialTakeoffAscent', ...
    'The first nominal height must exceed the supplied ground height in the same UP frame.');
ground=[groundPositionUpM,zeros(3,3)];rest=[first(:,1),zeros(3,3)];
durations=[20,5]; % Existing 20-s rise / 25-s formal-start phase locations.
[rise,riseUnit]=hermite(ground,rest,durations(1));
[join,joinUnit]=hermite(rest,first,durations(2));
trajectory=nominal;
trajectory.schema='RFLY_CANONICAL_INITIAL_TAKEOFF_SHARED_REFERENCE_V1';
totalDuration=25+nominal.total_duration_s;
trajectory.total_duration_s=totalDuration;
trajectory.evaluate_fcn=@evaluate;
% Environment service needs the same four derivatives together. Capture only
% the existing prepared coefficients/boundaries, not four generic trajectory
% dispatches inside the receive-to-environment-send deadline. No resampling.
riseDerivatives=rise.prepared_derivative_coefficients_ascending;
joinDerivatives=join.prepared_derivative_coefficients_ascending;
nominalEvaluator=nominal.evaluate_fcn;
trajectory.environment_jet_fcn=@(q)environmentJet(q,totalDuration,ground,rest, ...
    riseDerivatives,joinDerivatives,nominalEvaluator);
trajectory.preparation_prefix_duration_s=25;
trajectory.canonical_task_phase_offset_s=25;
receipt=struct('schema','RFLY_INITIAL_TAKEOFF_REFERENCE_BINDING_V1', ...
    'leg_index',1,'ground_position_up_m',groundPositionUpM.', ...
    'ground_anchor_provenance','CALLER_SUPPLIED_EXISTING_GROUND_OWNER_EKF_POSITION__NOT_AUTHENTICATED_HERE', ...
    'ground_derivatives','PLANNED_ZERO_VELOCITY_ACCELERATION_JERK__NOT_SENSOR_MEASUREMENTS', ...
    'original_first_jet_columns_p_v_a_j',first,'durations_phase_s',durations, ...
    'prefix_duration_phase_s',25,'original_leg_duration_s',nominal.total_duration_s, ...
    'controller_trajectory_phase','q__CANONICAL_CAUSAL_FLIGHT_PHASE_NOT_WALL_TIME', ...
    'formal_task_phase','q_minus_25_ONLY_WHEN_q_GE_25__NO_FORMAL_TASK_PROGRESS_IN_PREFIX', ...
    'preparation_segments',{{'SEVENTH_ORDER_REST_TO_REST_ASCENT','SEVENTH_ORDER_EXACT_FIRST_JET_TRANSITION_NOT_HOLD'}}, ...
    'coefficients_ascending_seconds',{{rise.coefficients_ascending,join.coefficients_ascending}}, ...
    'coefficients_ascending_unit_time',{{riseUnit,joinUnit}}, ...
    'canonical_derivative_sources',{sources},'canonical_derivative_sha256',{hashes}, ...
    'nominal_reference_arrays_modified',false,'outer_inner_shared_evaluator',true, ...
    'derivative_endpoint_dispatch','EXACT_SUPPLIED_BOUNDARIES__INTERIOR_BINARY64_POLYNOMIAL', ...
    'formal_phase_arithmetic','DIRECT_NOMINAL_EVALUATION_AT_BINARY64_q_MINUS_25__NO_RESAMPLING_OR_SAMPLE_SNAPPING', ...
    'flight_feasibility_proven',false,'ground_authority_proven',false, ...
    'solver_calls',0,'hardware_actions',0);
    function value=evaluate(progress,order)
        assert(isnumeric(progress)&&isreal(progress)&&all(isfinite(progress),'all') ...
            &&isscalar(order)&&isreal(order)&&any(order==0:3), ...
            'gpenmpcNative:InitialTakeoffQuery','Finite phase and derivative order zero through three are required.');
        q=min(max(double(progress(:).'),0),totalDuration);
        value=zeros(3,numel(q));
        for column=1:numel(q)
            if q(column)==0
                value(:,column)=ground(:,order+1);
            elseif q(column)<20
                value(:,column)=gpenmpcEvaluateTrajectoryDerivative(rise,q(column),order);
            elseif q(column)==20
                value(:,column)=rest(:,order+1);
            elseif q(column)<25
                value(:,column)=gpenmpcEvaluateTrajectoryDerivative(join,q(column)-20,order);
            else
                value(:,column)=nominal.evaluate_fcn(q(column)-25,order);
            end
        end
    end
end
function jet=environmentJet(progress,totalDuration,ground,rest,rise,join,nominal)
assert(isnumeric(progress)&&isreal(progress)&&isscalar(progress)&&isfinite(progress), ...
    'gpenmpcNative:InitialTakeoffQuery','Finite scalar actual phase is required.');
q=min(max(double(progress),0),totalDuration);
if q==0,jet=ground;return,end
if q==20,jet=rest;return,end
jet=zeros(3,4);
if q<25
    if q<20,coefficients=rise;else,coefficients=join;q=q-20;end
    for k=1:4
        c=coefficients{k};
        % Exact arithmetic/order from gpenmpcEvaluateTrajectoryDerivative.
        jet(:,k)=c*(q.^(0:size(c,2)-1)).';
    end
else
    for k=1:4,jet(:,k)=nominal(q-25,k-1);end
end
assert(all(isfinite(jet),'all'),'gpenmpcNative:InitialTakeoffJet','Nonfinite original reference.');
end
function [polynomial,unit]=hermite(left,right,duration)
% Eight p/v/a/j endpoint equations, solved in unit time. No fitted samples.
unit=zeros(3,8);
unit(:,1:4)=left.*[1,duration,duration^2/2,duration^3/6];
residual=[right(:,1)-sum(unit(:,1:4),2), ...
    duration*right(:,2)-unit(:,2)-2*unit(:,3)-3*unit(:,4), ...
    duration^2*right(:,3)-2*unit(:,3)-6*unit(:,4), ...
    duration^3*right(:,4)-6*unit(:,4)];
boundary=[1,1,1,1;4,5,6,7;12,20,30,42;24,60,120,210];
unit(:,5:8)=(boundary\residual.').';
polynomial=struct('total_duration_s',duration, ...
    'coefficients_ascending',unit./(duration.^(0:7)));
polynomial=gpenmpcPrepareTrajectoryDerivatives(polynomial,3);
end
function value=sha(path)
f=fopen(path,'rb');assert(f>=0,'gpenmpcNative:InitialTakeoffInput','Missing source %s',path);
c=onCleanup(@()fclose(f));bytes=fread(f,Inf,'*uint8');clear c
d=java.security.MessageDigest.getInstance('SHA-256');d.update(typecast(bytes,'int8'));
value=upper(reshape(dec2hex(typecast(d.digest(),'uint8'),2).',1,[]));
end
