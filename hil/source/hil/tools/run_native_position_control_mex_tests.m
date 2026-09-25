function report=run_native_position_control_mex_tests(newOutputDir,mexDir)
% Test the PositionControl MEX with independent numerical cases.
arguments
    newOutputDir (1,1) string
    mexDir (1,1) string
end
assert(~isfolder(newOutputDir)&&~isfile(newOutputDir),'gpenmpc:ExistingOutput','Preserve earlier results.');
toolDir=string(fileparts(mfilename('fullpath')));
restoreSearchPath=pathGuard(path); %#ok<NASGU>
addpath(toolDir,mexDir,'-begin');
binary=fullfile(mexDir,'gpenmpc_px4_native_position_control_mex.mexw64');
assert(strcmpi(which('gpenmpc_px4_native_position_control_mex'),binary),'gpenmpc:PositionMexShadowing');
gateway=@gpenmpc_px4_native_position_control_mex;
releaseMex=kernelGuard(gateway); %#ok<NASGU>
[cfg,metadata]=native_position_control_explicit_fixture();
plan=struct('name',{},'fn',{});cases=struct('name',{},'passed',{},'failure',{},'calls',{},'evidence',{});
calls={};
add('ZERO_ERROR_GRAVITY_THRUST_AND_LEVEL_QUATERNION',@equilibrium);
for axis=1:3
    for signValue=[-1,1]
        add(sprintf('POSITION_P_AXIS%d_SIGN%+d',axis,signValue),@()positionResponse(axis,signValue));
        add(sprintf('VELOCITY_PID_AXIS%d_SIGN%+d',axis,signValue),@()velocityResponse(axis,signValue));
    end
end
add('VELOCITY_PID_THREE_STEP_INTEGRAL_ORACLE',@integralOracle);
add('PAIRED_NAN_POSITION_NATIVE_VELOCITY_MODE',@nanVelocityMode);
add('PAIRED_NAN_POSITION_VELOCITY_NATIVE_ACCELERATION_MODE',@nanAccelerationMode);
add('NAN_YAW_AND_YAWSPEED_NATIVE_FALLBACK',@nanYaw);
add('YAW_POSITIVE_NEGATIVE_AND_THRUST_DIRECTION',@yawAndDirection);
add('VERTICAL_I_ZERO_GAIN_DOES_NOT_ACCUMULATE',@zeroIntegralGain);
add('VERTICAL_I_GAIN_CHANGE_PRESERVES_ACCUMULATED_STATE',@changeIntegralGain);
add('TERMINAL_NEGATIVE_I_INHIBIT_IS_DIRECTIONAL',@terminalInhibit);
add('VELOCITY_HORIZONTAL_UP_DOWN_LIMITS',@velocityLimits);
add('TILT_AND_COLLECTIVE_THRUST_LIMITS',@thrustAndTiltLimits);
add('NATIVE_MINIMUM_THRUST_FLOOR_REPORTED',@minimumThrustFloor);
add('DECOUPLED_AND_COUPLED_ACCELERATION_ORACLE',@decoupleOracle);
add('RESET_CLEARS_INTEGRAL_AND_SOURCE_CLOCK_REINIT',@resetCase);
add('CLEAR_REQUIRES_NEW_INIT',@clearCase);
add('ALL_NAN_SETPOINT_NATIVE_INVALID_CLEARS_INSTANCE',@()invalidSetpoint('ALL'));
add('UNPAIRED_POSITION_NAN_NATIVE_INVALID_CLEARS_INSTANCE',@()invalidSetpoint('POSITION'));
add('UNPAIRED_VELOCITY_NAN_NATIVE_INVALID_CLEARS_INSTANCE',@()invalidSetpoint('VELOCITY'));
add('UNPAIRED_ACCELERATION_NAN_NATIVE_INVALID_CLEARS_INSTANCE',@()invalidSetpoint('ACCELERATION'));
add('UNCONTROLLED_Z_AXIS_NATIVE_INVALID_CLEARS_INSTANCE',@()invalidSetpoint('Z'));
add('MISSING_CONFIGURATION_FIELD_REJECTS_AND_CLEARS',@missingConfiguration);
add('UNKNOWN_CONFIGURATION_FIELD_REJECTS_AND_CLEARS',@unknownConfiguration);
add('UNKNOWN_PROVENANCE_REJECTS_AND_CLEARS',@()badConfiguration('parameter_provenance','UNKNOWN'));
add('CONFIGURATION_INF_REJECTS_AND_CLEARS',@()badConfiguration('hover_thrust',Inf));
add('CONFIGURATION_NAN_REJECTS_AND_CLEARS',@()badConfiguration('velocity_i',[.5;NaN;.5]));
add('CONFIGURATION_WRONG_VECTOR_SHAPE_REJECTS_AND_CLEARS',@()badConfiguration('position_p',[1,1,1]));
add('CONFIGURATION_NEGATIVE_GAIN_REJECTS_AND_CLEARS',@()badConfiguration('velocity_i',[-1;.5;.5]));
add('ZERO_ARW_DIVISOR_REJECTS_AND_CLEARS',@()badConfiguration('velocity_p',[0;2;2]));
add('HOVER_OUT_OF_NATIVE_DOMAIN_REJECTS_AND_CLEARS',@()badConfiguration('hover_thrust',.99));
add('THRUST_MARGIN_OUT_OF_DOMAIN_REJECTS_AND_CLEARS',@()badConfiguration('horizontal_thrust_margin',1));
add('MISSING_STEP_FIELD_REJECTS_AND_CLEARS',@missingStep);
add('UNKNOWN_STEP_FIELD_REJECTS_AND_CLEARS',@unknownStep);
add('STATE_NAN_REJECTS_AND_CLEARS',@()badStep('state_position_ned',[0;NaN;0]));
add('STATE_INF_REJECTS_AND_CLEARS',@()badStep('state_velocity_ned',[Inf;0;0]));
add('STATE_WRONG_SHAPE_REJECTS_AND_CLEARS',@()badStep('state_acceleration_ned',[0,0,0]));
add('SETPOINT_INF_REJECTS_AND_CLEARS',@()badStep('trajectory_acceleration_ned',[0;0;Inf]));
add('SETPOINT_FLOAT_OVERFLOW_REJECTS_AND_CLEARS',@()badStep('trajectory_position_ned',[1e100;0;0]));
add('PHASE_I_NAN_REJECTS_AND_CLEARS',@()badStep('vertical_i_gain',NaN));
add('PHASE_I_NEGATIVE_REJECTS_AND_CLEARS',@()badStep('vertical_i_gain',-.1));
add('PHASE_LOGICAL_TYPE_REJECTS_AND_CLEARS',@()badStep('terminal_negative_integral_update_inhibit',0));
add('ZERO_DT_REJECTS_AND_CLEARS',@()badStep('dt_s',0));
add('NEGATIVE_DT_REJECTS_AND_CLEARS',@()badStep('dt_s',-.01));
add('NAN_TIME_REJECTS_AND_CLEARS',@()badStep('timestamp_sample_s',NaN));
add('DUPLICATE_SOURCE_TIME_REJECTS_AND_CLEARS',@()badTime(0,.01));
add('REVERSED_SOURCE_TIME_REJECTS_AND_CLEARS',@()badTime(-.01,.01));
add('DT_SOURCE_DELTA_MISMATCH_REJECTS_AND_CLEARS',@()badTime(.02,.01));
add('UNKNOWN_COMMAND_REJECTS_AND_CLEARS',@unknownCommand);
add('RESET_WITHOUT_INSTANCE_REJECTS',@resetWithoutInstance);
mkdir(newOutputDir);
for k=1:numel(plan)
    calls={};evidence=struct();failure='';passed=false;
    try
        invoke('clear');
        evidence=plan(k).fn();passed=true;
    catch problem
        failure=getReport(problem,'extended','hyperlinks','off');
    end
    try,invoke('clear');catch cleanupProblem,passed=false;failure=[failure newline getReport(cleanupProblem,'extended','hyperlinks','off')];end %#ok<AGROW>
    cases(end+1)=struct('name',plan(k).name,'passed',passed,'failure',failure,'calls',{calls},'evidence',evidence); %#ok<AGROW>
    fprintf('HOST PositionControl %s: %d\n',plan(k).name,passed);
end
report=struct('schema','HOST_NATIVE_POSITION_CONTROL_MEX_FUNCTIONAL_TESTS_V1', ...
    'classification','HOST_NUMERICAL_KERNEL_FUNCTIONAL_CONTROLS', ...
    'passed',numel(cases)==numel(plan)&&all([cases.passed]),'cases_planned',numel(plan), ...
    'cases_executed',numel(cases),'cases_passed',nnz([cases.passed]), ...
    'planned_names',{{plan.name}},'cases',cases,'fixture',cfg,'fixture_metadata',metadata, ...
    'binary',identity(binary),'script',identity([mfilename('fullpath') '.m']), ...
    'fixture_source',identity(fullfile(toolDir,'native_position_control_explicit_fixture.m')), ...
    'hardware_actions',0,'COM_UDP_actions',0,'live_admission',false, ...
    'current_position_parameters_verified',false,'numerical_tolerance_provenance', ...
    'Float arithmetic agreement uses 2e-5 absolute/relative tolerance.', ...
    'nonfinite_json_encoding','Numeric arrays containing NaN/Inf are tagged MATLAB_IEEE754_HEX_V1 with full class, shape and num2hex values; no NaN/Inf is silently replaced by null.', ...
    'limitations',{{'Tests use explicit HOST parameter fixtures.', ...
        'PositionControl includes GPENMPC modifications.', ...
        'No M600 plant, estimator, HTE, takeoff, landing, transport, controller deployment or flight is exercised.', ...
        'Independent arithmetic checks cover unsaturated small cases; qualitative limit tests cover constrained behavior.', ...
        'Position-kernel tests use their own parameter fixture.'}});
fid=fopen(fullfile(newOutputDir,'RESULT.json'),'w','n','UTF-8');assert(fid>=0);
fileCleanup=fileGuard(fid); %#ok<NASGU>
fprintf(fid,'%s',jsonencode(losslessJson(report),PrettyPrint=true));
fprintf('HOST PositionControl actual cases: %d/%d; hardware=0\n',report.cases_passed,report.cases_planned);
assert(report.passed,'gpenmpc:PositionKernelTestsFailed','See retained RESULT.json.');

    function add(name,fn),plan(end+1)=struct('name',name,'fn',fn);end
    function out=invoke(command,varargin)
        entry=struct('command',command,'inputs',{varargin},'returned',false,'output',[], ...
            'error_identifier','','error_message','');
        try
            if strcmp(command,'clear'),gateway('clear');out=[];else,out=gateway(command,varargin{:});end
            entry.returned=true;entry.output=out;calls{end+1}=entry;
        catch problem
            entry.error_identifier=problem.identifier;entry.error_message=problem.message;calls{end+1}=entry;rethrow(problem);
        end
    end
    function begin(c),assert(isequal(invoke('init',c),true));end
    function out=step(s)
        out=invoke('step',s);
        assert(out.update_valid&&out.outputs_usable_for_host_numerical_chain ...
            &&~out.authority_granted&&~out.instance_cleared&&~out.dt_clamped);
        assert(all(isfinite([out.q_d;out.thrust_body;out.acceleration_setpoint_ned;out.thrust_setpoint_ned])));
    end
    function errorReceipt=reject(fn)
        caught=false;errorReceipt=struct();
        try,fn();catch problem,caught=true;errorReceipt=struct('identifier',problem.identifier,'message',problem.message); ...
                assert(strcmp(problem.identifier,'gpenmpc:NativePositionKernel'));end
        assert(caught,'gpenmpc:MissingExpectedRejection','Expected actual MEX rejection.');
    end
    function cleared()
        r=reject(@()invoke('step',sample()));assert(contains(r.message,'not initialized'));
    end
    function e=equilibrium()
        begin(cfg);s=sample();o=step(s);near(o.acceleration_setpoint_ned,zeros(3,1));
        near(o.thrust_setpoint_ned,[0;0;-.5]);near(o.q_d,[1;0;0;0]);near(o.vertical_integral_z_after,0);
        e=struct('expected_thrust',[0;0;-.5],'expected_quaternion',[1;0;0;0]);
    end
    function e=positionResponse(axis,signValue)
        begin(cfg);s=sample();s.trajectory_position_ned(axis)=signValue*.1;
        o=step(s);expected=oracle(cfg,s,zeros(3,1));compareOracle(o,expected);
        e=struct('axis',axis,'sign',signValue,'oracle',expected);
    end
    function e=velocityResponse(axis,signValue)
        begin(cfg);s=sample();s.trajectory_position_ned(:)=NaN;
        s.state_velocity_ned(axis)=-signValue*.1;s.state_acceleration_ned(axis)=signValue*.2;
        s.trajectory_velocity_ned(axis)=signValue*.2;s.trajectory_acceleration_ned(axis)=signValue*.03;
        o=step(s);expected=oracle(cfg,s,zeros(3,1));compareOracle(o,expected);
        e=struct('axis',axis,'sign',signValue,'oracle',expected);
    end
    function e=integralOracle()
        begin(cfg);s=sample();s.trajectory_position_ned(:)=NaN;s.state_velocity_ned=[-.2;.15;.3];
        s.state_acceleration_ned=[.1;-.2;.25];s.trajectory_velocity_ned=[.15;-.05;.1];
        s.trajectory_acceleration_ned=[.03;-.02;.04];I=zeros(3,1);expected=cell(3,1);
        for n=1:3,s.timestamp_sample_s=(n-1)*s.dt_s;o=step(s);expected{n}=oracle(cfg,s,I); ...
                compareOracle(o,expected{n});I=expected{n}.integral_after;end
        e=struct('three_independent_oracle_steps',{expected});
    end
    function e=nanVelocityMode()
        begin(cfg);s=sample();s.trajectory_position_ned(:)=NaN;s.trajectory_velocity_ned=[.1;-.1;0];
        o=step(s);assert(all(isnan(o.position_setpoint_ned)));near(o.velocity_setpoint_ned,s.trajectory_velocity_ned);
        e=struct('native_nan_position_retained',true);
    end
    function e=nanAccelerationMode()
        begin(cfg);s=sample();s.trajectory_position_ned(:)=NaN;s.trajectory_velocity_ned(:)=NaN;
        s.trajectory_acceleration_ned=[.1;-.1;.05];o=step(s);
        assert(all(isnan(o.position_setpoint_ned))&&all(isnan(o.velocity_setpoint_ned)));
        near(o.acceleration_setpoint_ned,s.trajectory_acceleration_ned);
        near(o.thrust_setpoint_ned,thrustOracle(s.trajectory_acceleration_ned,cfg));
        near(o.vertical_integral_z_after,0);e=struct('acceleration_only_native_mode',true);
    end
    function e=nanYaw()
        begin(cfg);s=sample();s.state_yaw=.37;s.trajectory_yaw=NaN;s.trajectory_yawspeed=NaN;o=step(s);
        near(o.yaw_setpoint,s.state_yaw);near(o.yawspeed_setpoint,0);near(o.q_d,[cos(.37/2);0;0;sin(.37/2)]);
        e=struct('expected_yaw',.37,'expected_yawspeed',0);
    end
    function e=yawAndDirection()
        expected=cell(2,1);n=0;
        for y=[-.3,.3]
            n=n+1;begin(cfg);s=sample();s.trajectory_yaw=y;s.trajectory_acceleration_ned=[.25;-.15;0];
            o=step(s);thr=thrustOracle(s.trajectory_acceleration_ned,cfg);bodyZ=rotateQuaternion(o.q_d,[0;0;1]);
            near(bodyZ,-thr/norm(thr));near(o.thrust_body,[0;0;-norm(thr)]);near(norm(o.q_d),1);
            expected{n}=struct('yaw',y,'desired_body_z_ned',-thr/norm(thr));
        end
        e=struct('independent_quaternion_direction_checks',{expected});
    end
    function e=zeroIntegralGain()
        begin(cfg);s=sample();s.trajectory_position_ned(3)=.1;s.vertical_i_gain=0;
        for n=1:8,s.timestamp_sample_s=(n-1)*s.dt_s;o=step(s);near(o.vertical_integral_z_after,0); ...
                assert(~o.vertical_diagnostics.integral_updated);end
        e=struct('explicit_zero_phase_gain_steps',8);
    end
    function e=changeIntegralGain()
        begin(cfg);s=sample();s.trajectory_position_ned(3)=.1;o1=step(s);
        s.timestamp_sample_s=.01;s.vertical_i_gain=0;o2=step(s);
        near(o2.vertical_integral_z_before,o1.vertical_integral_z_after);near(o2.vertical_integral_z_after,o1.vertical_integral_z_after);
        s.timestamp_sample_s=.02;s.vertical_i_gain=cfg.velocity_i(3);o3=step(s);
        near(o3.vertical_integral_z_after,o2.vertical_integral_z_after+.1*.5*.01);
        e=struct('initial_I',o1.vertical_integral_z_after,'held_I',o2.vertical_integral_z_after,'resumed_I',o3.vertical_integral_z_after);
    end
    function e=terminalInhibit()
        begin(cfg);s=sample();s.trajectory_position_ned(3)=.1;first=step(s);
        s.timestamp_sample_s=.01;s.trajectory_position_ned(3)=-.1;s.terminal_negative_integral_update_inhibit=true;
        blocked=step(s);near(blocked.vertical_integral_z_after,first.vertical_integral_z_after);
        assert(blocked.vertical_diagnostics.terminal_negative_integral_update_blocked);
        s.timestamp_sample_s=.02;s.trajectory_position_ned(3)=.1;positive=step(s);
        assert(positive.vertical_integral_z_after>blocked.vertical_integral_z_after ...
            &&~positive.vertical_diagnostics.terminal_negative_integral_update_blocked);
        s.timestamp_sample_s=.03;s.trajectory_position_ned(3)=-.1;s.terminal_negative_integral_update_inhibit=false;
        negative=step(s);assert(negative.vertical_integral_z_after<positive.vertical_integral_z_after);
        e=struct('negative_blocked',true,'positive_still_integrates',true,'disabled_negative_integrates',true);
    end
    function e=velocityLimits()
        values=cell(3,1);targets={[100;100;0],[0;0;-100],[0;0;100]};
        for n=1:3,begin(cfg);s=sample();s.trajectory_position_ned=targets{n};values{n}=step(s);end
        near(norm(values{1}.velocity_setpoint_ned(1:2)),cfg.velocity_limits_mps(1));
        near(values{2}.velocity_setpoint_ned(3),-cfg.velocity_limits_mps(2));
        near(values{3}.velocity_setpoint_ned(3),cfg.velocity_limits_mps(3));
        e=struct('outputs',{values});
    end
    function e=thrustAndTiltLimits()
        begin(cfg);s=sample();s.trajectory_position_ned=[100;100;-100];o=step(s);
        z=rotateQuaternion(o.q_d,[0;0;1]);tilt=acos(max(-1,min(1,z(3))));
        assert(tilt<=cfg.tilt_limit_rad+2e-5&&norm(o.thrust_setpoint_ned)<=cfg.thrust_limits(2)+2e-5);
        e=struct('tilt_rad',tilt,'thrust_norm',norm(o.thrust_setpoint_ned));
    end
    function e=minimumThrustFloor()
        c=cfg;c.thrust_limits(1)=0;begin(c);s=sample();s.trajectory_position_ned(:)=NaN;s.trajectory_velocity_ned(:)=NaN;
        s.trajectory_acceleration_ned=[0;0;20];o=step(s);near(o.effective_minimum_thrust,.001);
        near(norm(o.thrust_setpoint_ned),.001);e=struct('native_floor',.001);
    end
    function e=decoupleOracle()
        values=cell(2,1);
        for n=1:2,c=cfg;c.decouple_horizontal_vertical=logical(n==1);begin(c);s=sample(); ...
                s.trajectory_acceleration_ned=[.3;-.2;.4];o=step(s);values{n}=thrustOracle(s.trajectory_acceleration_ned,c); ...
                near(o.thrust_setpoint_ned,values{n});end
        assert(norm(values{1}-values{2})>1e-5);e=struct('independent_thrust_vectors',{values});
    end
    function e=resetCase()
        begin(cfg);s=sample();s.trajectory_position_ned(3)=.1;o=step(s);assert(o.vertical_integral_z_after>0);
        assert(isequal(invoke('reset'),true));reset=step(sample());near(reset.vertical_integral_z_before,0);near(reset.vertical_integral_z_after,0);
        begin(cfg);again=step(sample());near(again.q_d,reset.q_d);e=struct('reset_and_reinit_match',true);
    end
    function e=clearCase(),begin(cfg);step(sample());invoke('clear');cleared();e=struct('explicit_clear_verified',true);end
    function e=invalidSetpoint(kind)
        begin(cfg);step(sample());s=sample();s.timestamp_sample_s=.01;
        switch kind
            case 'ALL',s.trajectory_position_ned(:)=NaN;s.trajectory_velocity_ned(:)=NaN;s.trajectory_acceleration_ned(:)=NaN;
            case 'POSITION',s.trajectory_position_ned(1)=NaN;
            case 'VELOCITY',s.trajectory_velocity_ned(1)=NaN;
            case 'ACCELERATION',s.trajectory_acceleration_ned(1)=NaN;
            case 'Z',s.trajectory_position_ned(3)=NaN;s.trajectory_velocity_ned(3)=NaN;s.trajectory_acceleration_ned(3)=NaN;
        end
        o=invoke('step',s);assert(~o.update_valid&&~o.outputs_usable_for_host_numerical_chain ...
            &&~o.authority_granted&&o.instance_cleared&&all(isnan([o.q_d;o.thrust_body;o.thrust_setpoint_ned])));
        assert(~o.vertical_diagnostics.current_update_valid&&isnan(o.vertical_diagnostics.position));cleared();
        e=struct('native_invalid_result',o,'old_successful_outputs_not_reused',true);
    end
    function e=missingConfiguration(),begin(cfg);bad=rmfield(cfg,'hover_thrust');e=reject(@()invoke('init',bad));cleared();end
    function e=unknownConfiguration(),begin(cfg);bad=cfg;bad.unknown=0;e=reject(@()invoke('init',bad));cleared();end
    function e=badConfiguration(key,value),begin(cfg);bad=cfg;bad.(key)=value;e=reject(@()invoke('init',bad));cleared();end
    function e=missingStep(),begin(cfg);s=rmfield(sample(),'vertical_i_gain');e=reject(@()invoke('step',s));cleared();end
    function e=unknownStep(),begin(cfg);s=sample();s.unknown=0;e=reject(@()invoke('step',s));cleared();end
    function e=badStep(key,value),begin(cfg);s=sample();s.(key)=value;e=reject(@()invoke('step',s));cleared();end
    function e=badTime(timestamp,dt),begin(cfg);step(sample());s=sample();s.timestamp_sample_s=timestamp;s.dt_s=dt; ...
            e=reject(@()invoke('step',s));cleared();end
    function e=unknownCommand(),begin(cfg);e=reject(@()invoke('UNKNOWN',sample()));cleared();end
    function e=resetWithoutInstance(),invoke('clear');e=reject(@()invoke('reset'));cleared();end
end

function s=sample()
s=struct('timestamp_sample_s',0,'dt_s',.01,'state_position_ned',zeros(3,1), ...
    'state_velocity_ned',zeros(3,1),'state_acceleration_ned',zeros(3,1),'state_yaw',0, ...
    'trajectory_position_ned',zeros(3,1),'trajectory_velocity_ned',zeros(3,1), ...
    'trajectory_acceleration_ned',zeros(3,1),'trajectory_yaw',0,'trajectory_yawspeed',0, ...
    'vertical_i_gain',.5,'terminal_negative_integral_update_inhibit',false);
end
function e=oracle(c,s,I)
% Compute an independent linear unsaturated cascade.
positionTerm=c.position_p.*(s.trajectory_position_ned-s.state_position_ned);
positionTerm(isnan(positionTerm))=0;
velocity=s.trajectory_velocity_ned+positionTerm;
error=velocity-s.state_velocity_ned;
acceleration=s.trajectory_acceleration_ned+c.velocity_p.*error+I-c.velocity_d.*s.state_acceleration_ned;
gain=c.velocity_i;gain(3)=s.vertical_i_gain;
if s.terminal_negative_integral_update_inhibit,error(3)=max(error(3),0);end
e=struct('position_term',positionTerm,'velocity',velocity,'acceleration',acceleration, ...
    'thrust_ned',thrustOracle(acceleration,c),'integral_after',I+gain.*error*s.dt_s);
end
function compareOracle(o,e)
near(o.velocity_setpoint_ned,e.velocity);near(o.acceleration_setpoint_ned,e.acceleration);
near(o.thrust_setpoint_ned,e.thrust_ned);near(o.vertical_integral_z_after,e.integral_after(3));
near(rotateQuaternion(o.q_d,[0;0;1]),-e.thrust_ned/norm(e.thrust_ned));
near(o.thrust_body,[0;0;-norm(e.thrust_ned)]);
end
function t=thrustOracle(acc,c)
g=double(single(9.80665));upSpecific=g;
if ~c.decouple_horizontal_vertical,upSpecific=g-acc(3);end
z=[-acc(1);-acc(2);upSpecific];z=z/norm(z);
assert(acos(z(3))<c.tilt_limit_rad,'gpenmpc:OracleOutsideDomain','Small-case oracle excludes tilt clipping.');
vertical=acc(3)*c.hover_thrust/g-c.hover_thrust;t=z*(vertical/z(3));
assert(norm(t)<c.thrust_limits(2)&&norm(t)>max(c.thrust_limits(1),.001), ...
    'gpenmpc:OracleOutsideDomain','Small-case oracle excludes collective saturation.');
end
function v=rotateQuaternion(q,v)
q=q(:);v=v(:);u=q(2:4);v=v+2*q(1)*cross(u,v)+2*cross(u,cross(u,v));
end
function near(a,b)
assert(isequal(size(a),size(b))&&all(isfinite(a),'all')&&all(isfinite(b),'all'));
assert(all(abs(double(a)-double(b))<=2e-5*(1+abs(double(b))),'all'), ...
    'gpenmpc:PositionOracleMismatch','Independent float-arithmetic oracle differs.');
end
function out=losslessJson(value)
if isnumeric(value)&&any(~isfinite(value(:)))
    assert(isreal(value)&&any(strcmp(class(value),{'double','single'})));
    out=struct('numeric_encoding','MATLAB_IEEE754_HEX_V1','matlab_class',class(value), ...
        'shape',size(value),'column_major_hex',{cellstr(num2hex(value(:)))});
elseif isstruct(value)
    out=value;keys=fieldnames(value);
    for k=1:numel(value),for j=1:numel(keys),out(k).(keys{j})=losslessJson(value(k).(keys{j}));end,end
elseif iscell(value)
    out=cell(size(value));for k=1:numel(value),out{k}=losslessJson(value{k});end
else
    out=value;
end
end
function guard=pathGuard(capturedPath),guard=onCleanup(@()path(capturedPath));end
function guard=kernelGuard(capturedGateway),guard=onCleanup(@()capturedGateway('clear'));end
function guard=fileGuard(capturedFid),guard=onCleanup(@()fclose(capturedFid));end
function r=identity(file)
fid=fopen(file,'rb');assert(fid>=0);guard=fileGuard(fid); %#ok<NASGU>
bytes=fread(fid,Inf,'*uint8');md=java.security.MessageDigest.getInstance('SHA-256');md.update(bytes);
r=struct('path',char(file),'bytes',numel(bytes), ...
    'sha256',upper(reshape(dec2hex(typecast(md.digest(),'uint8'),2).',1,[])));
end
