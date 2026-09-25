function report=run_native_lower_loop_mex_tests(mexDir,outputDir)
% Test the native lower-loop MEX on the host.
arguments,mexDir (1,1) string,outputDir (1,1) string,end
assert(~isfolder(outputDir),'gpenmpc:ExistingOutput','Use a new test directory.');
binary=fullfile(mexDir,['gpenmpc_px4_native_lower_loop_mex.' mexext]);assert(isfile(binary),'gpenmpc:MexMissing');
oldPath=path;pathGuard=onCleanup(@()path(oldPath));addpath(mexDir,'-begin'); %#ok<NASGU>
assert(strcmp(which('gpenmpc_px4_native_lower_loop_mex'),char(binary)),'gpenmpc:MexResolution','Unexpected MEX on path.');
fn=@gpenmpc_px4_native_lower_loop_mex;
invokeCount=struct('init',0,'step',0,'clear',0,'unknown',0);
cleanup=onCleanup(@()clearKernel()); %#ok<NASGU>
[cfg,metadata]=native_lower_loop_explicit_fixture();base=state();
cases=struct('name',{},'passed',{},'evidence',{},'error',{});
run('current_typed_and_explicit_fixture_identity',@fixtureIdentity);
run('actual_init_zero_error_symmetric_bounded_allocation',@zeroError);
for axis=1:3
    for signValue=[-1,1]
        aa=axis;ss=signValue;
        run(sprintf('reference_axis_%d_sign_%+d',aa,ss),@()polarity(aa,ss,false));
        run(sprintf('estimate_error_axis_%d_sign_%+d',aa,ss),@()polarity(aa,ss,true));
    end
end
run('attitude_update_false_holds_last_reference',@holdAttitude);
run('dt_nominal_no_clamp',@()dtCheck(.01,NaN,.01,false));
run('first_dt_lower_clamp',@()dtCheck(.00001,NaN,.000125,true));
run('first_dt_upper_clamp',@()dtCheck(1,NaN,.02,true));
run('source_delta_lower_clamp',@()dtCheck(.01,.00001,.000125,true));
run('source_delta_upper_clamp',@()dtCheck(.01,.5,.02,true));
run('landed_integral_frozen_zero',@()integralCase('landed'));
run('maybe_landed_integral_frozen_zero',@()integralCase('maybe'));
run('airborne_integral_actually_accumulates',@()integralCase('airborne'));
run('airborne_then_landed_integral_holds',@()integralCase('hold'));
run('disarmed_landed_resets_integral',@()integralCase('disarmed'));
run('nonrotary_landed_resets_integral',@()integralCase('nonrotary'));
run('reinit_resets_previous_integral_state',@()integralCase('reinit'));
run('finite_large_demand_allocator_clipped',@largeDemand);
run('explicit_yaw_filter_branch',@yawFilter);
run('explicit_battery_scale_branch',@batteryScale);
run('explicit_slew_fixture_branch',@slew);
run('clear_is_idempotent_then_step_rejects',@clearAndReinit);
bad=rmfield(cfg,'rate_p');run('missing_config_field_rejected_clears_instance',@()reject('init',bad,'rate_p'));
bad=cfg;bad.rate_p(1)=NaN;run('nan_config_rejected_clears_instance',@()reject('init',bad,'Nonfinite'));
bad=cfg;bad.rate_p(1)=realmax;run('float_overflow_config_rejected',@()reject('init',bad,'Nonfinite'));
bad=cfg;bad.rate_p=[.1;.1];run('wrong_gain_size_rejected',@()reject('init',bad,'exact size'));
bad=cfg;bad.effectiveness=zeros(5,6);run('wrong_effectiveness_dimensions_rejected',@()reject('init',bad,'6x6'));
bad=cfg;bad.effectiveness(1,1)=NaN;run('nonfinite_effectiveness_rejected',@()reject('init',bad,'Nonfinite'));
bad=cfg;bad.parameter_provenance='UNKNOWN';run('unknown_provenance_rejected',@()reject('init',bad,'provenance'));
bad=cfg;bad.parameter_provenance='HOST_FIXTURE_NOT_LIVE';run('unsupported_design_draft_provenance_literal_rejected',@()reject('init',bad,'provenance'));
bad=cfg;bad.mc_airmode=1;run('unsupported_airmode_rejected',@()reject('init',bad,'MC_AIRMODE'));
bad=cfg;bad.yaw_weight=1.1;run('invalid_yaw_weight_rejected',@()reject('init',bad,'Yaw weight'));
bad=cfg;bad.rate_limits_rad_s(1)=0;run('zero_rate_limit_rejected',@()reject('init',bad,'gains/limits'));
bad=cfg;bad.integral_limits(1)=-1;run('negative_integral_limit_rejected',@()reject('init',bad,'gains/limits'));
bad=cfg;bad.battery_scale_enabled=0;run('nonlogical_config_boolean_rejected',@()reject('init',bad,'logical'));
bad=cfg;bad.initial_motor_commands(1)=1.1;run('invalid_initial_motor_command_rejected',@()reject('init',bad,'Slew/initial'));
bad=cfg;bad.slew_limits_s(1)=-1;run('negative_slew_rejected',@()reject('init',bad,'Slew/initial'));
bad=rmfield(base,'body_rate');run('missing_step_field_rejected_clears_instance',@()reject('step',bad,'body_rate'));
bad=base;bad.body_rate(1)=NaN;run('nan_step_rejected_clears_instance',@()reject('step',bad,'Nonfinite'));
bad=base;bad.angular_acceleration(1)=Inf;run('inf_step_rejected_clears_instance',@()reject('step',bad,'Nonfinite'));
bad=base;bad.body_rate=[0;0];run('wrong_step_vector_dimensions_rejected',@()reject('step',bad,'exact size'));
bad=base;bad.q_est=[2;0;0;0];run('nonunit_quaternion_rejected',@()reject('step',bad,'Quaternion'));
bad=base;bad.q_reference=[1;0;0];run('wrong_quaternion_dimensions_rejected',@()reject('step',bad,'exact size'));
bad=base;bad.armed=1;run('nonlogical_step_boolean_rejected',@()reject('step',bad,'logical'));
bad=base;bad.rates_enabled=false;run('disabled_rate_path_rejected',@()reject('step',bad,'rates_enabled'));
bad=base;bad.attitude_updated=false;run('missing_initial_attitude_update_rejected',@()reject('step',bad,'First step'));
bad=base;bad.initial_dt_s=0;run('zero_initial_dt_rejected',@()reject('step',bad,'initial_dt_s'));
bad=base;bad.battery_scale=0;run('zero_battery_scale_rejected',@()reject('step',bad,'battery_scale'));
run('duplicate_timestamp_rejected_clears_instance',@()timeReject(0));
run('reversed_timestamp_rejected_clears_instance',@()timeReject(-.01));
run('unknown_command_rejected_clears_instance',@()reject('unknown',base,'Unknown command'));
run('final_reinit_zero_state_and_explicit_clear',@clearAndReinit);
clearKernel();clear cleanup
report=struct('schema','HOST_NATIVE_LOWER_LOOP_MEX_FUNCTION_NEGATIVE_TESTS_V1', ...
    'passed',numel(cases)==62&&all([cases.passed]),'planned_case_count',62,'case_count',numel(cases),'cases_passed',nnz([cases.passed]), ...
    'cases',cases,'actual_mex_invocations',invokeCount,'binary',identity(binary), ...
    'test_source',identity(mfilename('fullpath')+string('.m')), ...
    'fixture_source',identity(fullfile(fileparts(mfilename('fullpath')),'native_lower_loop_explicit_fixture.m')), ...
    'configuration',cfg,'configuration_provenance',metadata,'initial_step_fixture',base, ...
    'numerical_test_tolerance',struct('float_output_absolute',1e-6,'dt_absolute',1e-8, ...
        'provenance','HOST_FLOAT_ARITHMETIC_FUNCTION_TEST_TOLERANCE_NOT_LIVE_PERFORMANCE_GATE'), ...
    'all_current_parameters_known',false,'hardware_actions',0,'COM_UDP_board_process_actions',0, ...
    'closed_loop_stability_verified',false,'full_PX4_SITL_verified',false,'flight_admission',false, ...
    'claim','Native MEX functional and invalid-input tests.');
mkdir(outputDir);f=fopen(fullfile(outputDir,'RESULT.json'),'w');assert(f>=0);c=onCleanup(@()fclose(f));
fprintf(f,'%s\n',jsonencode(report,PrettyPrint=true));clear c
disp(struct('passed',report.passed,'case_count',report.case_count,'cases_passed',report.cases_passed));
assert(report.passed,'gpenmpc:NativeMexTestsFailed','See complete case rows in RESULT.json.');
    function run(name,action)
        clearKernel();ev=struct();failure='';
        try,ev=action();passed=true;catch err,passed=false;failure=getReport(err,'extended','hyperlinks','off');end
        clearKernel();cases(end+1)=struct('name',name,'passed',passed,'evidence',ev,'error',failure);
    end
    function out=call(command,input)
        if isfield(invokeCount,command),invokeCount.(command)=invokeCount.(command)+1;else,invokeCount.unknown=invokeCount.unknown+1;end
        out=fn(command,input);
    end
    function clearKernel(),invokeCount.clear=invokeCount.clear+1;fn('clear');end
    function ev=fixtureIdentity()
        assert(strcmp(cfg.parameter_provenance,'CURRENT_TYPED_AND_EXPLICIT_FIXTURE'));
        assert(~metadata.current_all_parameters_known&&numel(metadata.missing_fields_explicit_fixtures)==7);
        ok=call('init',cfg);assert(islogical(ok)&&isscalar(ok)&&ok);
        ev=struct('current_parameter_rows',numel(metadata.current_typed_fields),'explicit_fixture_fields',7);
    end
    function y=zeroError()
        assert(call('init',cfg));y=call('step',base);bounded(y);
        assert(norm(y.rate_setpoint)<1e-6&&norm(y.torque_raw)<1e-6&&norm(y.integral)<1e-6);
        assert(max(y.motor_commands)-min(y.motor_commands)<1e-6&&all(y.motor_commands>0));
    end
    function y=polarity(axis,signValue,estimateError)
        assert(call('init',cfg));s=base;q=[cos(pi/720);zeros(3,1)];q(axis+1)=signValue*sin(pi/720);
        if estimateError,s.q_est=q;expected=-signValue;else,s.q_reference=q;expected=signValue;end
        y=call('step',s);bounded(y);
        assert(expected*y.rate_setpoint(axis)>0&&expected*y.torque_raw(axis)>0);
        physical=cfg.effectiveness*y.motor_commands;assert(expected*physical(axis)>0);
        assert(all(abs(y.rate_setpoint(setdiff(1:3,axis)))<1e-5));
    end
    function ev=holdAttitude()
        assert(call('init',cfg));s=base;s.q_reference=[cos(.005);sin(.005);0;0];a=call('step',s);
        s.timestamp_sample_s=s.timestamp_sample_s+.01;s.attitude_updated=false;s.q_reference=[cos(.01);0;sin(.01);0];
        z=call('step',s);assert(isequal(a.rate_setpoint,z.rate_setpoint));ev=struct('first',a,'held',z);
    end
    function y=dtCheck(first,delta,expected,clamped)
        assert(call('init',cfg));s=base;s.initial_dt_s=first;y=call('step',s);
        if isfinite(delta),s.timestamp_sample_s=s.timestamp_sample_s+delta;y=call('step',s);end
        assert(abs(y.dt_s-expected)<1e-8&&y.dt_clamped==clamped);bounded(y);
    end
    function ev=integralCase(mode)
        assert(call('init',cfg));s=base;s.body_rate=[-.05;0;0];s.landed=false;s.maybe_landed=false;
        if strcmp(mode,'landed'),s.landed=true;elseif strcmp(mode,'maybe'),s.maybe_landed=true;end
        sequence=zeros(20,3);
        for j=1:20,s.timestamp_sample_s=10+j*.01;y=call('step',s);sequence(j,:)=y.integral.';end
        if ismember(mode,{'landed','maybe'})
            assert(all(sequence==0,'all')&&~y.integral_updates_enabled);
        else
            assert(sequence(end,1)>sequence(1,1)&&sequence(1,1)>0&&y.integral_updates_enabled);
        end
        initial=y;
        if ismember(mode,{'hold','disarmed','nonrotary'})
            s.timestamp_sample_s=s.timestamp_sample_s+.01;s.landed=true;
            if strcmp(mode,'disarmed'),s.armed=false;elseif strcmp(mode,'nonrotary'),s.rotary_wing=false;end
            y=call('step',s);
            if strcmp(mode,'hold'),assert(isequal(y.integral,initial.integral));else,assert(all(y.integral==0));end
        elseif strcmp(mode,'reinit')
            assert(call('init',cfg));y=call('step',base);assert(all(y.integral==0));
        end
        ev=struct('sequence',sequence,'before_transition',initial,'after',y);
    end
    function y=largeDemand()
        assert(call('init',cfg));s=base;s.body_rate=[-100;100;-100];s.thrust_body_normalized=[0;0;-1];
        y=call('step',s);bounded(y);assert(y.motor_saturated);
    end
    function y=yawFilter()
        c=cfg;c.yaw_torque_cutoff_hz=20;assert(call('init',c));s=base;s.body_rate=[0;0;-.1];
        y=call('step',s);assert(y.torque_applied(3)>0&&y.torque_applied(3)<y.torque_raw(3));bounded(y);
    end
    function y=batteryScale()
        c=cfg;c.battery_scale_enabled=true;assert(call('init',c));s=base;s.body_rate=[-.1;0;0];s.battery_scale=.5;
        y=call('step',s);assert(abs(y.torque_applied(1)-.5*y.torque_raw(1))<1e-6);bounded(y);
    end
    function y=slew()
        c=cfg;c.slew_limits_s=10*ones(6,1);assert(call('init',c));y=call('step',base);
        bounded(y);assert(max(y.motor_commands)<=.001+1e-6&&any(y.motor_commands>0));
    end
    function ev=clearAndReinit()
        assert(call('init',cfg));call('step',base);clearKernel();clearKernel();
        failure=expectNativeError(@()call('step',base),'not initialized');
        assert(call('init',cfg));z=call('step',base);assert(all(z.integral==0));clearKernel();
        ev=struct('cleared_step_error',failure,'reinitialized',z);
    end
    function ev=reject(command,input,message)
        assert(call('init',cfg));failure=expectNativeError(@()call(command,input),message);
        cleared=expectNativeError(@()call('step',base),'not initialized');
        assert(call('init',cfg));z=call('step',base);bounded(z);
        ev=struct('first_error',failure,'after_error_instance_cleared',cleared,'reinitialized',true);
    end
    function ev=timeReject(delta)
        assert(call('init',cfg));call('step',base);s=base;s.timestamp_sample_s=s.timestamp_sample_s+delta;
        failure=expectNativeError(@()call('step',s),'timestamp');
        cleared=expectNativeError(@()call('step',base),'not initialized');
        assert(call('init',cfg));z=call('step',base);bounded(z);ev=struct('first_error',failure,'cleared',cleared);
    end
end
function r=state()
r=struct('timestamp_sample_s',10.0,'initial_dt_s',.01,'armed',true,'rotary_wing',true, ...
    'landed',true,'maybe_landed',false,'rates_enabled',true,'q_est',[1;0;0;0], ...
    'q_reference',[1;0;0;0],'body_rate',zeros(3,1),'angular_acceleration',zeros(3,1), ...
    'thrust_body_normalized',[0;0;-.5],'yaw_rate_ff',0.0,'attitude_updated',true,'battery_scale',1.0);
end
function bounded(y)
for f={'rate_setpoint','torque_raw','torque_applied','motor_commands','allocated_control','unallocated_control','integral'}
    assert(all(isfinite(y.(f{1}))),'gpenmpc:NonfiniteOutput');
end
assert(isequal(size(y.motor_commands),[6,1])&&all(y.motor_commands>=0&y.motor_commands<=1));
assert(contains(y.claim,'PX4_ATTITUDE_RATE_SEQUENTIAL_ALLOCATION_KERNEL')&&contains(y.claim,'CALLER_OWNED_ARM_STATE'));
end
function r=expectNativeError(action,message)
failed=false;r=struct('identifier','','message','');
try,unused=action();catch e,failed=true;r=struct('identifier',e.identifier,'message',e.message);end %#ok<NASGU>
assert(failed&&strcmp(r.identifier,'gpenmpc:NativeKernel')&&contains(lower(r.message),lower(message)), ...
    'gpenmpc:ExpectedNativeReject','Expected NativeKernel rejection containing %s; actual %s',message,r.message);
end
function r=identity(path)
path=char(path);f=fopen(path,'rb');assert(f>=0);c=onCleanup(@()fclose(f));v=fread(f,Inf,'*uint8');
m=java.security.MessageDigest.getInstance('SHA-256');m.update(v);
r=struct('path',path,'bytes',numel(v),'sha256',upper(reshape(dec2hex(typecast(m.digest(),'uint8'),2).',1,[])));clear c
end
