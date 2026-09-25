function report=run_native_position_runtime_mex_tests(newOutputDir,mexDir)
% Test configure_runtime on the host.
arguments
    newOutputDir (1,1) string
    mexDir (1,1) string
end
assert(~isfolder(newOutputDir)&&~isfile(newOutputDir),'gpenmpc:ExistingOutput');
toolsDir=string(fileparts(mfilename('fullpath')));pathCleanup=guardPath(path); %#ok<NASGU>
addpath(toolsDir,mexDir,'-begin');
binary=fullfile(mexDir,'gpenmpc_px4_native_position_control_mex.mexw64');
assert(strcmpi(which('gpenmpc_px4_native_position_control_mex'),binary));
gateway=@gpenmpc_px4_native_position_control_mex;kernelCleanup=guardKernel(gateway); %#ok<NASGU>
[cfg,meta]=native_position_control_explicit_fixture();
tests=struct('name',{},'fn',{});cases=struct('name',{},'passed',{},'failure',{},'calls',{},'evidence',{});calls={};
add('KEEP_PRESERVES_INTEGRAL_AND_SOURCE_HISTORY',@keepHistory);
add('SET_HOVER_PRESERVES_I_WITHOUT_BUMPLESS_COMPENSATION',@setHover);
add('UPDATE_HOVER_BUMPLESS_AT_EQUILIBRIUM',@()updateHover(0));
add('UPDATE_HOVER_USES_PRIOR_NONZERO_ACCELERATION',@()updateHover(.4));
add('RESET_ALL_ORIGINAL_SETTER_ZEROES_ALL_I_WITHOUT_TIME_RESET',@resetAll);
add('RESET_XY_PRESERVES_VERTICAL_I',@resetXY);
add('RUNTIME_VELOCITY_AND_THRUST_LIMITS_TAKE_EFFECT',@limits);
add('NEGATIVE_UP_LIMIT_PRESERVES_NATIVE_RAMPUP_SEMANTICS',@negativeUp);
add('RUNTIME_TILT_TAKES_EFFECT',@tilt);
add('ORDERED_HTE_UPDATE_THEN_BEFORE_FLIGHT_SET_RESET',@orderedCalls);
add('CONFIGURE_DOES_NOT_RESET_SOURCE_TIME',@duplicateTime);
add('EXPLICIT_RESET_RETAINS_LAST_RUNTIME_CONFIG',@resetRetainsConfig);
add('REPEATED_KEEP_CONFIGURE_RETAINS_SINGLE_INSTANCE',@repeatedKeep);
add('UPDATE_BEFORE_FIRST_VALID_CONTROL_REJECTS_AND_CLEARS',@updateBeforeStep);
add('MISSING_RUNTIME_FIELD_REJECTS_AND_CLEARS',@missingField);
add('UNKNOWN_RUNTIME_FIELD_REJECTS_AND_CLEARS',@unknownField);
add('UNKNOWN_HOVER_ACTION_REJECTS_AND_CLEARS',@()bad('hover_action','UNKNOWN'));
add('KEEP_WITH_DIFFERENT_VALUE_REJECTS_AND_CLEARS',@()bad('hover_thrust',.6));
add('WRONG_VECTOR_SHAPE_REJECTS_AND_CLEARS',@()bad('velocity_limits_mps',[5,3,2]));
add('NONFINITE_LIMIT_REJECTS_AND_CLEARS',@()bad('velocity_limits_mps',[Inf;3;2]));
add('NAN_HOVER_REJECTS_AND_CLEARS',@()bad('hover_thrust',NaN));
add('NEGATIVE_XY_LIMIT_REJECTS_AND_CLEARS',@()bad('velocity_limits_mps',[-1;3;2]));
add('NEGATIVE_DOWN_LIMIT_REJECTS_AND_CLEARS',@()bad('velocity_limits_mps',[5;3;-1]));
add('REVERSED_THRUST_LIMITS_REJECT_AND_CLEAR',@()bad('thrust_limits',[.8;.7]));
add('TILT_SINGULARITY_REJECTS_AND_CLEARS',@()bad('tilt_limit_rad',pi/2));
add('THRUST_MAX_LESS_THAN_RETAINED_MARGIN_REJECTS',@()bad('thrust_limits',[.1;.2]));
add('NONLOGICAL_RESET_REJECTS_AND_CLEARS',@()bad('reset_integral',1));
add('HOVER_OUTSIDE_NATIVE_DOMAIN_REJECTS_AND_CLEARS',@()bad('hover_thrust',1));
add('CONFIGURE_UNINITIALIZED_REJECTS',@uninitialized);
mkdir(newOutputDir);
for n=1:numel(tests)
    calls={};passed=false;failure='';evidence=struct();
    try,invoke('clear');evidence=tests(n).fn();passed=true;catch err,failure=getReport(err,'extended','hyperlinks','off');end
    try,invoke('clear');catch err,passed=false;failure=[failure newline getReport(err,'extended','hyperlinks','off')];end %#ok<AGROW>
    cases(end+1)=struct('name',tests(n).name,'passed',passed,'failure',failure,'calls',{calls},'evidence',evidence); %#ok<AGROW>
    fprintf('HOST runtime setter %s: %d\n',tests(n).name,passed);
end
report=struct('schema','HOST_NATIVE_POSITION_RUNTIME_SETTER_MEX_TESTS_V1', ...
    'passed',all([cases.passed]),'cases_planned',numel(tests),'cases_executed',numel(cases), ...
    'cases_passed',nnz([cases.passed]),'cases',cases,'fixture',cfg,'fixture_metadata',meta, ...
    'binary',identity(binary),'test_source',identity([mfilename('fullpath') '.m']), ...
    'hardware_actions',0,'COM_UDP_actions',0,'flight_admission',false, ...
    'classification','HOST_SETTER_FUNCTIONAL_TESTS', ...
    'nonfinite_encoding','MATLAB_IEEE754_HEX_V1 retains original NaN/Inf bits and shape in call arguments.', ...
    'limitations',{{'Explicit position-profile fixture values.', ...
        'The scalar HTE compensation oracle checks original updateHoverThrust arithmetic, not an HTE estimator.', ...
        'No module phase, failsafe, takeoff, payload or landing decisions are made here.', ...
        'All configure calls retain the existing instance/time; explicit reset is tested separately.', ...
        'Existing 60 init/step/reset/clear cases must also be rerun on the extended binary.'}});
fid=fopen(fullfile(newOutputDir,'RESULT.json'),'w','n','UTF-8');assert(fid>=0);fileCleanup=guardFile(fid); %#ok<NASGU>
fprintf(fid,'%s',jsonencode(encodeNonfinite(report),PrettyPrint=true));
assert(report.passed,'gpenmpc:RuntimeSetterTestsFailed','See retained RESULT.json.');
    function add(name,fn),tests(end+1)=struct('name',name,'fn',fn);end
    function out=invoke(command,varargin)
        item=struct('command',command,'inputs',{varargin},'returned',false,'output',[],'error','');
        try
            if strcmp(command,'clear'),gateway('clear');out=[];else,out=gateway(command,varargin{:});end
            item.returned=true;item.output=out;calls{end+1}=item;
        catch err,item.error=[err.identifier ': ' err.message];calls{end+1}=item;rethrow(err);end
    end
    function begin(),assert(invoke('init',cfg));end
    function o=step(s),o=invoke('step',s);assert(o.update_valid&&~o.authority_granted);end
    function r=configure(c)
        r=invoke('configure_runtime',c);assert(r.configured&&~r.authority_granted ...
            &&~r.instance_reinitialized&&r.source_clock_unchanged);
    end
    function r=reject(fn)
        caught=false;r=struct();try,fn();catch err,caught=true;assert(strcmp(err.identifier,'gpenmpc:NativePositionKernel')); ...
                r=struct('identifier',err.identifier,'message',err.message);end
        assert(caught,'gpenmpc:ExpectedRuntimeRejection');
    end
    function cleared(),r=reject(@()invoke('step',sample()));assert(contains(r.message,'not initialized'));end
    function e=keepHistory()
        begin();s=sample();s.trajectory_position_ned(3)=.1;o=step(s);r=configure(runtime(cfg));
        near(r.vertical_integral_z_after,o.vertical_integral_z_after);assert(r.has_prior_valid_step&&r.last_source_time_s==0);
        s.timestamp_sample_s=.01;next=step(s);near(next.vertical_integral_z_before,o.vertical_integral_z_after);
        e=struct('first_I',o.vertical_integral_z_after,'configuration',r);
    end
    function e=setHover()
        begin();s=sample();s.trajectory_position_ned(3)=1;o=step(s);assert(o.vertical_integral_z_after>0);
        c=runtime(cfg);c.hover_action='SET';c.hover_thrust=.6;r=configure(c);
        near(r.vertical_integral_z_after,o.vertical_integral_z_after);s=sample();s.timestamp_sample_s=.01;next=step(s);
        expected=(o.vertical_integral_z_after/double(single(9.80665))-1)*double(single(.6));
        near(next.thrust_setpoint_ned(3),expected);e=struct('receipt',r,'new_thrust',next.thrust_setpoint_ned,'expected_thrust_z',expected);
    end
    function e=updateHover(acceleration)
        begin();s=sample();s.trajectory_acceleration_ned(3)=acceleration;before=step(s);
        c=runtime(cfg);c.hover_action='UPDATE';c.hover_thrust=.6;r=configure(c);
        g=double(single(9.80665));old=.5;new=double(single(.6));
        expected=before.vertical_integral_z_after+(before.acceleration_setpoint_ned(3)-g)*old/new+g-before.acceleration_setpoint_ned(3);
        near(r.vertical_integral_z_after_hover_action,expected);near(r.vertical_integral_z_after,expected);
        s.timestamp_sample_s=.01;after=step(s);near(after.thrust_setpoint_ned(3),before.thrust_setpoint_ned(3));
        e=struct('independent_expected_I',expected,'receipt',r,'before_thrust',before.thrust_setpoint_ned,'after_thrust',after.thrust_setpoint_ned);
    end
    function e=resetAll()
        begin();s=sample();s.trajectory_position_ned=[.1;.1;.1];step(s);
        c=runtime(cfg);c.reset_integral=true;r=configure(c);near(r.vertical_integral_z_after,0);
        s=sample();s.timestamp_sample_s=.01;o=step(s);near(o.acceleration_setpoint_ned,zeros(3,1));
        e=struct('receipt',r,'all_axes_integral_effect_zero',true);
    end
    function e=resetXY()
        begin();s=sample();s.trajectory_position_ned=[.1;.1;.1];before=step(s);
        c=runtime(cfg);c.reset_integral_xy=true;r=configure(c);near(r.vertical_integral_z_after,before.vertical_integral_z_after);
        s=sample();s.timestamp_sample_s=.01;o=step(s);near(o.acceleration_setpoint_ned(1:2),zeros(2,1));
        near(o.acceleration_setpoint_ned(3),before.vertical_integral_z_after);e=struct('receipt',r,'vertical_I_retained',true);
    end
    function e=limits()
        begin();c=runtime(cfg);c.velocity_limits_mps=[.2;.1;.15];c.thrust_limits=[0;.6];r=configure(c);
        s=sample();s.trajectory_position_ned=[10;10;-10];o=step(s);near(norm(o.velocity_setpoint_ned(1:2)),.2);near(o.velocity_setpoint_ned(3),-.1);
        s.timestamp_sample_s=.01;s.trajectory_position_ned(3)=10;next=step(s);near(next.velocity_setpoint_ned(3),.15);
        assert(norm(next.thrust_setpoint_ned)<=.6+2e-5);e=struct('receipt',r);
    end
    function e=negativeUp()
        begin();c=runtime(cfg);c.velocity_limits_mps=[5;-.2;.5];r=configure(c);o=step(sample());
        near(o.velocity_setpoint_ned(3),.2);e=struct('receipt',r,'native_lower_vertical_velocity',o.velocity_setpoint_ned(3));
    end
    function e=tilt()
        begin();c=runtime(cfg);c.tilt_limit_rad=.05;r=configure(c);s=sample();s.trajectory_acceleration_ned(1)=1;o=step(s);
        z=rotate(o.q_d,[0;0;1]);angle=acos(max(-1,min(1,z(3))));assert(angle<=.05+2e-5);
        e=struct('receipt',r,'actual_tilt_rad',angle);
    end
    function e=orderedCalls()
        begin();step(sample());early=runtime(cfg);early.hover_action='UPDATE';early.hover_thrust=.6;first=configure(early);
        late=runtime(cfg);late.hover_action='SET';late.reset_integral=true;late.reset_integral_xy=true;second=configure(late);
        assert(first.runtime_configuration_sequence==1&&second.runtime_configuration_sequence==2);
        assert(strcmp(second.original_setter_call_order,'setHoverThrust -> resetIntegral -> setTiltLimit -> setThrustLimits -> setVelocityLimits -> resetIntegralXY'));
        near(second.vertical_integral_z_after_hover_action,first.vertical_integral_z_after);near(second.vertical_integral_z_after,0);
        s=sample();s.timestamp_sample_s=.01;o=step(s);near(o.thrust_setpoint_ned(3),-.5);
        e=struct('early_HTE',first,'late_preflight',second);
    end
    function e=duplicateTime()
        begin();step(sample());configure(runtime(cfg));e=reject(@()invoke('step',sample()));cleared();
    end
    function e=resetRetainsConfig()
        begin();step(sample());c=runtime(cfg);c.hover_action='SET';c.hover_thrust=.6;c.velocity_limits_mps=[.2;.1;.15];configure(c);
        assert(invoke('reset'));o=step(sample());near(o.thrust_setpoint_ned(3),-.6);near(o.vertical_integral_z_before,0);
        s=sample();s.timestamp_sample_s=.01;s.trajectory_position_ned(3)=-10;next=step(s);near(next.velocity_setpoint_ned(3),-.1);
        e=struct('runtime_configuration_survived_explicit_reset',true);
    end
    function e=repeatedKeep()
        begin();step(sample());last=struct();for repeatIndex=1:3,last=configure(runtime(cfg));assert(last.runtime_configuration_sequence==repeatIndex);end
        s=sample();s.timestamp_sample_s=.01;step(s);e=struct('last_receipt',last);
    end
    function e=updateBeforeStep()
        begin();c=runtime(cfg);c.hover_action='UPDATE';c.hover_thrust=.6;e=reject(@()invoke('configure_runtime',c));cleared();
    end
    function e=missingField(),begin();c=rmfield(runtime(cfg),'reset_integral_xy');e=reject(@()invoke('configure_runtime',c));cleared();end
    function e=unknownField(),begin();c=runtime(cfg);c.extra=1;e=reject(@()invoke('configure_runtime',c));cleared();end
    function e=bad(key,value),begin();step(sample());c=runtime(cfg);c.(key)=value;e=reject(@()invoke('configure_runtime',c));cleared();end
    function e=uninitialized(),invoke('clear');e=reject(@()invoke('configure_runtime',runtime(cfg)));cleared();end
end
function c=runtime(cfg)
c=struct('velocity_limits_mps',cfg.velocity_limits_mps,'thrust_limits',cfg.thrust_limits, ...
    'tilt_limit_rad',cfg.tilt_limit_rad,'hover_action','KEEP','hover_thrust',cfg.hover_thrust, ...
    'reset_integral',false,'reset_integral_xy',false);
end
function s=sample()
s=struct('timestamp_sample_s',0,'dt_s',.01,'state_position_ned',zeros(3,1), ...
    'state_velocity_ned',zeros(3,1),'state_acceleration_ned',zeros(3,1),'state_yaw',0, ...
    'trajectory_position_ned',zeros(3,1),'trajectory_velocity_ned',zeros(3,1), ...
    'trajectory_acceleration_ned',zeros(3,1),'trajectory_yaw',0,'trajectory_yawspeed',0, ...
    'vertical_i_gain',.5,'terminal_negative_integral_update_inhibit',false);
end
function near(a,b),assert(isequal(size(a),size(b))&&all(isfinite(a),'all')&&all(isfinite(b),'all') ...
    &&all(abs(a-b)<=2e-5*(1+abs(b)),'all'),'gpenmpc:RuntimeOracleMismatch');end
function v=rotate(q,v),q=q(:);v=v(:);u=q(2:4);v=v+2*q(1)*cross(u,v)+2*cross(u,cross(u,v));end
function value=encodeNonfinite(value)
if isnumeric(value)&&any(~isfinite(value(:)))
    value=struct('numeric_encoding','MATLAB_IEEE754_HEX_V1','matlab_class',class(value), ...
        'shape',size(value),'column_major_hex',{cellstr(num2hex(value(:)))});
elseif isstruct(value)
    fields=fieldnames(value);for n=1:numel(value),for k=1:numel(fields),value(n).(fields{k})=encodeNonfinite(value(n).(fields{k}));end,end
elseif iscell(value)
    for k=1:numel(value),value{k}=encodeNonfinite(value{k});end
end
end
function g=guardPath(v),g=onCleanup(@()path(v));end
function g=guardKernel(v),g=onCleanup(@()v('clear'));end
function g=guardFile(v),g=onCleanup(@()fclose(v));end
function r=identity(file)
fid=fopen(file,'rb');assert(fid>=0);g=guardFile(fid); %#ok<NASGU>
b=fread(fid,Inf,'*uint8');m=java.security.MessageDigest.getInstance('SHA-256');m.update(b);
r=struct('path',char(file),'bytes',numel(b),'sha256',upper(reshape(dec2hex(typecast(m.digest(),'uint8'),2).',1,[])));
end
