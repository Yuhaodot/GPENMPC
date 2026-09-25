function result=run_native_takeoff_mex_tests(mexDir,outputDir)
% Test takeoff state and ramp handling with explicit clocks.
arguments
    mexDir (1,1) string
    outputDir (1,1) string
end
assert(~isfolder(outputDir)&&~isfile(outputDir));oldPath=path;addpath(mexDir,'-begin');
name='gpenmpc_px4_native_takeoff_mex';assert(strcmpi(which(name),fullfile(mexDir,name+".mexw64")));
f=str2func(name);guard=onCleanup(@()finish(f,oldPath)); %#ok<NASGU>
[~,m,meta]=native_position_control_current_config();
cfg=struct('spoolup_time_s',m.COM_SPOOLUP_TIME,'ramp_time_s',m.MPC_TKO_RAMP_T, ...
    'vertical_velocity_p',m.MPC_Z_VEL_P_ACC,'parameter_provenance','CURRENT_TYPED_WITH_EXPLICIT_MODULE_PHASE');
assert(cfg.spoolup_time_s==1&&cfg.ramp_time_s==3&&cfg.vertical_velocity_p==4);
tests=struct('name',{},'passed',{});trace=struct([]);saved=struct();
assert(f('init',cfg));a=sample(uint64(1000000));o=f('step',a);append(o);
expectedInitial=double(-single(9.80665)/single(4));
add('initial_disarmed_native_negative_up_limit',o.state_before==1&&o.state_after==1&&o.not_taken_off&&~o.flying&&o.upward_velocity_limit_mps==expectedInitial);
add('first_dt_explicit_not_boot_default',o.initial_dt_used&&o.dt_s==double(single(.01))&&~o.dt_clamped);
add('no_authority_or_private_state_claim',~o.authority_granted&&~o.private_ramp_progress_observed);
a.source_timestamp_us=uint64(1010000);a.armed=true;a.want_takeoff=true;o=f('step',a);append(o);
add('armed_enters_spoolup',o.state_after==2&&o.upward_velocity_limit_mps==expectedInitial);
for k=1:99,a.source_timestamp_us=uint64(1010000+k*10000);o=f('step',a);append(o);end
add('spoolup_not_complete_at_990ms',o.state_after==2);
a.source_timestamp_us=uint64(2010000);o=f('step',a);append(o);
firstRamp=single(expectedInitial)+(single(.01)/single(3))*(single(3)-single(expectedInitial));
add('spoolup_exact_1s_enters_ramp',o.state_after==4&&abs(o.upward_velocity_limit_mps-double(firstRamp))<1e-6);
add('ramp_can_return_negative_up_limit',o.upward_velocity_limit_mps<0&&~o.not_taken_off&&~o.flying);
a.landed=false;firstFlight=[];
for k=1:305
    a.source_timestamp_us=uint64(2010000+k*10000);o=f('step',a);append(o);
    if isempty(firstFlight)&&o.state_after==5,firstFlight=double(a.source_timestamp_us-uint64(2010000))*1e-6;end
end
add('original_float_ramp_reaches_flight',~isempty(firstFlight)&&firstFlight>=2.98&&firstFlight<=3.03&&o.state_after==5);
add('flight_limit_exact_desired',o.upward_velocity_limit_mps==3&&o.flying);
a.source_timestamp_us=a.source_timestamp_us+uint64(10000);a.landed=true;a.want_takeoff=false;o=f('step',a);append(o);
add('native_landed_returns_ready',o.state_after==3&&o.not_taken_off&&o.upward_velocity_limit_mps==expectedInitial);
a.source_timestamp_us=a.source_timestamp_us+uint64(10000);a.armed=false;o=f('step',a);append(o);
add('disarm_returns_disarmed',o.state_after==1&&~o.armed_input);
saved.full_ramp_last=o;
assert(f('reset'));a=sample(uint64(1000000));o=f('step',a);
add('explicit_reset_restarts_source_and_hysteresis',o.state_before==1&&o.state_after==1&&o.sample_count==1&&o.initial_dt_used);
assert(f('reset'));a=sample(uint64(1000000));a.armed=true;a.skip_takeoff=true;o=f('step',a);
add('explicit_skip_sets_flight_even_landed',o.state_after==5&&o.landed_input&&o.upward_velocity_limit_mps==3);
assert(f('reset'));a=sample(uint64(1000000));a.armed=true;o=f('step',a);a.source_timestamp_us=uint64(2000000);o=f('step',a);
add('no_want_takeoff_stays_ready',o.state_after==3&&o.upward_velocity_limit_mps==expectedInitial);
add('large_source_gap_clips_ramp_dt_not_hysteresis_time',o.source_delta_s==1&&o.dt_clamped&&o.dt_s==double(single(.04)));
assert(f('reset'));a=sample(uint64(1000000));a.initial_dt_s=.0001;o=f('step',a);
add('explicit_first_dt_lower_clip',o.dt_clamped&&o.dt_s==double(single(.002))&&o.source_delta_s==.0001);
a.source_timestamp_us=uint64(1000100);o=f('step',a);
add('source_sub2ms_dt_lower_clip',~o.initial_dt_used&&o.dt_clamped&&o.dt_s==double(single(.002)));
c=cfg;c.vertical_velocity_p=0;assert(f('init',c));o=f('step',sample(uint64(1000000)));
add('native_P_floor_preserved',o.native_p_floor_applied&&o.upward_velocity_limit_mps==double(-single(9.80665)/single(.01)));
c=cfg;c.ramp_time_s=0;c.spoolup_time_s=0;assert(f('init',c));a=sample(uint64(1000000));a.armed=true;a.want_takeoff=true;a.landed=false;o=f('step',a);
add('zero_ramp_uses_original_one_update_completion',o.state_after==4&&o.upward_velocity_limit_mps==3);
a.source_timestamp_us=a.source_timestamp_us+uint64(10000);o=f('step',a);add('flight_transition_follows_ramp_update',o.state_before==4&&o.state_after==5);
bad=cfg;bad=rmfield(bad,'ramp_time_s');rejectInit('missing_config',bad);
bad=cfg;bad.unknown=1;rejectInit('unknown_config',bad);
bad=cfg;bad.ramp_time_s=NaN;rejectInit('NaN_config',bad);
bad=cfg;bad.spoolup_time_s=-1;rejectInit('negative_spoolup',bad);
bad=cfg;bad.vertical_velocity_p=-1;rejectInit('negative_P',bad);
bad=cfg;bad.parameter_provenance='ASSUMED_DEFAULT';rejectInit('unknown_provenance',bad);
bad=sample(uint64(1000000));bad=rmfield(bad,'want_takeoff');rejectStep('missing_step_field',bad);
bad=sample(uint64(1000000));bad.extra=0;rejectStep('unknown_step_field',bad);
bad=sample(uint64(1000000));bad.armed=1;rejectStep('numeric_boolean_rejected',bad);
bad=sample(uint64(1000000));bad.source_timestamp_us=1000000;rejectStep('double_timestamp_rejected',bad);
bad=sample(uint64(0));rejectStep('zero_timestamp_rejected',bad);
bad=sample(intmax('uint64'));rejectStep('hysteresis_timestamp_overflow_rejected',bad);
bad=sample(uint64(1000000));bad.initial_dt_s=0;rejectStep('zero_initial_dt_rejected',bad);
bad=sample(uint64(1000000));bad.initial_dt_s=Inf;rejectStep('infinite_dt_rejected',bad);
bad=sample(uint64(1000000));bad.takeoff_desired_velocity_up_mps=NaN;rejectStep('nonfinite_desired_speed_rejected',bad);
bad=sample(uint64(1000000));bad.takeoff_desired_velocity_up_mps=-1;rejectStep('negative_desired_magnitude_rejected',bad);
for mode=1:2
    assert(f('init',cfg));a=sample(uint64(1000000));o=f('step',a); %#ok<NASGU>
    if mode==1,label='duplicate_clock_clears_instance';else,label='reverse_clock_clears_instance';a.source_timestamp_us=uint64(999999);end
    caught=throws(@()f('step',a));after=throws(@()f('step',sample(uint64(2000000))));add(label,caught&&after);
end
f('clear');add('clear_prevents_step',throws(@()f('step',sample(uint64(1000000)))));
add('reset_requires_existing_instance',throws(@()f('reset')));
result=struct('schema','HOST_ORIGINAL_NATIVE_TAKEOFF_MEX_TESTS_V1','passed',all([tests.passed]), ...
    'case_count',numel(tests),'cases_passed',sum([tests.passed]),'tests',tests, ...
    'current_configuration',cfg,'current_parameter_evidence',meta.current138, ...
    'first_flight_after_ramp_start_s',firstFlight,'saved',saved,'hardware_actions',0,'COM_UDP_actions',0, ...
    'claim','Takeoff-class functional and invalid-input tests.');
mkdir(outputDir);save(fullfile(outputDir,'RAW_TESTS.mat'),'result','trace');
out=fopen(fullfile(outputDir,'HOST_RESULT.json'),'w','n','UTF-8');assert(out>=0);fg=onCleanup(@()fclose(out)); %#ok<NASGU>
fprintf(out,'%s\n',jsonencode(result,PrettyPrint=true));assert(result.passed,'gpenmpc:NativeTakeoffTestsFailed');
    function add(label,value),tests(end+1)=struct('name',label,'passed',logical(value));end %#ok<AGROW>
    function append(value),if isempty(trace),trace=value;else,trace(end+1)=value;end,end %#ok<AGROW>
    function rejectInit(label,bad)
        caught=throws(@()f('init',bad));cleared=throws(@()f('step',sample(uint64(1000000))));add(label,caught&&cleared);
    end
    function rejectStep(label,bad)
        assert(f('init',cfg));caught=throws(@()f('step',bad));cleared=throws(@()f('step',sample(uint64(2000000))));add(label,caught&&cleared);
    end
end
function s=sample(t)
s=struct('source_timestamp_us',t,'initial_dt_s',.01,'armed',false,'landed',true, ...
    'want_takeoff',false,'skip_takeoff',false,'takeoff_desired_velocity_up_mps',3);
end
function yes=throws(call)
yes=false;try,value=call();catch e,yes=strcmp(e.identifier,'gpenmpc:NativeTakeoffKernel');end %#ok<NASGU>
end
function finish(f,p)
try,f('clear');catch,end
path(p);
end
