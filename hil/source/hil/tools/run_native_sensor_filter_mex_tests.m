function report = run_native_sensor_filter_mex_tests(outputDir)
% Compare the filter MEX with an independent arithmetic oracle.
arguments
    outputDir (1,1) string
end
assert(~isfolder(outputDir),'gpenmpc:ExistingOutput','Use a new test output.');
here=string(fileparts(mfilename('fullpath')));
mexDir=gpenmpc_external_path('native_sensor_filter_mex');
build=jsondecode(fileread(fullfile(mexDir,'BUILD_RESULT.json')));assert(build.passed);
binary=identity(fullfile(mexDir,'gpenmpc_px4_sensor_filter_mex.mexw64'));
assert(strcmp(binary.sha256,build.binary.sha256)&&binary.bytes==build.binary.bytes);
for k=1:numel(build.bindings)
    actual=identity(build.bindings(k).path);
    assert(strcmp(actual.sha256,build.bindings(k).sha256)&&actual.bytes==build.bindings(k).bytes,'gpenmpc:FilterInputChanged');
end
oldPath=path;addpath(here,mexDir);
resolvedFilter=@gpenmpc_px4_sensor_filter_mex;
cleanupGuard=onCleanup(@()finishFilter(resolvedFilter,oldPath)); %#ok<NASGU>
assert(strcmp(string(which('gpenmpc_px4_sensor_filter_mex')),string(binary.path)));
gpenmpc_px4_sensor_filter_mex('clear');mkdir(outputDir);
[cfg,provenance]=native_sensor_filter_explicit_fixture(500);
cases=struct('name',{},'passed',{},'detail',{},'error',{});
runCase('typed_current_40_30_and_explicit_fs',@()checkFixture(cfg,provenance));
runCase('zero_input_64_samples',@()zeroInput(cfg));
runCase('constant_input_warm_initialization',@()constantInput(cfg));
runCase('proper_rotation_offset_and_body_bias',@()calibrationDirection(cfg));
for a=1:3,runCase(sprintf('unit_sensor_axis_%d_direction',a),@()axisDirection(cfg,a));end
runCase('NF_LP_D_explicit_bypass',@()allBypass(cfg));
runCase('LP_D_single_formula_oracle',@()oracleCase(cfg,false));
runCase('static_NF0_NF1_action_and_oracle',@()oracleCase(cfg,true));
runCase('Nyquist_static_filters_bypass',@()nyquist(cfg));
runCase('negative_cutoffs_native_bypass',@()negativeCutoffs(cfg));
runCase('D_uses_backward_filtered_difference_not_true_alpha',@()differenceNotTruth(cfg));
runCase('first_sample_native_predecessor',@()firstSample(cfg));
runCase('native_20us_min_and_20ms_max_dt',@()clippedDt(cfg));
runCase('filter_only_reset_preserves_clock',@()resetClock(cfg,false));
runCase('explicit_reset_restarts_clock',@()resetClock(cfg,true));
runCase('warm_uncorrect_reset_directions',@()warmUncorrect(cfg));
runCase('duplicate_must_stop_and_explicit_reset',@()sequenceReset(cfg,false));
runCase('reversal_must_stop_and_explicit_reset',@()sequenceReset(cfg,true));
runCase('step_after_sequence_fault_rejects_and_clears',@()sequenceReject(cfg,false));
runCase('filter_only_reset_cannot_clear_sequence_fault',@()sequenceReject(cfg,true));
bad=rmfield(cfg,'gyro_cutoff_hz');runCase('missing_cfg_clears_object',@()invalidInit(cfg,bad));
bad=cfg;bad.gyro_cutoff_hz=NaN;runCase('NaN_cfg_clears_object',@()invalidInit(cfg,bad));
bad=cfg;bad.dynamic_notch_enable=1;runCase('DNF_unknown_dynamic_state_rejected',@()invalidInit(cfg,bad));
bad=cfg;bad.scale(1)=1.01;runCase('nonunity_scale_rejected',@()invalidInit(cfg,bad));
bad=cfg;bad.mount_rotation=ones(3);runCase('nonorthogonal_matrix_rejected',@()invalidInit(cfg,bad));
bad=cfg;bad.mount_rotation=diag([-1,1,1]);runCase('reflection_matrix_rejected',@()invalidInit(cfg,bad));
bad=cfg;bad.mount_rotation=eye(2);runCase('unknown_matrix_shape_rejected',@()invalidInit(cfg,bad));
bad=cfg;bad.mount_rotation(1)=NaN;runCase('nonfinite_matrix_rejected',@()invalidInit(cfg,bad));
bad=cfg;bad.sample_rate_hz=10;runCase('fs_lower_boundary_rejected',@()invalidInit(cfg,bad));
bad=cfg;bad.sample_rate_hz=10000;runCase('fs_upper_boundary_rejected',@()invalidInit(cfg,bad));
bad=cfg;bad.parameter_provenance='ASSUMED_LIVE';runCase('unknown_provenance_rejected',@()invalidInit(cfg,bad));
s=sample([NaN;0;0],1000000);runCase('NaN_sample_clears_object',@()invalidStep(cfg,s));
s=sample([0;0],1000000);runCase('wrong_sample_vector_clears_object',@()invalidStep(cfg,s));
s=sample(zeros(3,1),1000000);s=rmfield(s,'raw_gyro');runCase('missing_sample_field_clears_object',@()invalidStep(cfg,s));
s=sample(zeros(3,1),1000000);s.timestamp_sample_us=1000000;runCase('double_timestamp_rejected',@()invalidStep(cfg,s));
s=sample(zeros(3,1),1);runCase('first_timestamp_underflow_rejected',@()invalidStep(cfg,s));
s=sample(zeros(3,1),1000000);s.timestamp_sample_us=uint64(9007199254740992)+uint64(1);runCase('timestamp_reporting_range_rejected',@()invalidStep(cfg,s));
runCase('unknown_command_clears_object',@()unknownCommand(cfg));
runCase('nonfinite_reset_clears_object',@()badReset(cfg));
gpenmpc_px4_sensor_filter_mex('clear');
cleared=throws(@()gpenmpc_px4_sensor_filter_mex('status'));
report=struct('schema','HOST_ORIGINAL_PX4_SENSOR_FILTER_MEX_TESTS_V1',...
    'passed',all([cases.passed])&&cleared,'case_count',numel(cases),...
    'cases_passed',sum([cases.passed]),'cases',cases,'binary',binary,...
    'explicit_fixture',provenance,'MEX_object_cleared',cleared,...
    'oracle','MATLAB single direct-form-II LP, direct-form-I static notch, backward difference and first-order D LP',...
    'oracle_tolerance',struct('rate_abs',2e-5,'derivative_abs',2e-3,'relative',4e-5),...
    'COM_actions',0,...
    'board_actions',0,'actuator_outputs',0,'claim','Host sensor-filter interface tests.');
f=fopen(fullfile(outputDir,'CASES.json'),'w','n','UTF-8');assert(f>=0);
c=onCleanup(@()fclose(f));fprintf(f,'%s\n',jsonencode(report,PrettyPrint=true));clear c
disp(jsonencode(report,PrettyPrint=true));
assert(report.passed,'gpenmpc:SensorFilterTests','%d/%d tests passed; inspect CASES.json',report.cases_passed,report.case_count);
    function runCase(name,fn)
        gpenmpc_px4_sensor_filter_mex('clear');detail=struct();error='';passed=false;
        try,detail=fn();passed=true;catch e,error=getReport(e,'extended','hyperlinks','off');end
        gpenmpc_px4_sensor_filter_mex('clear');
        cases(end+1)=struct('name',name,'passed',passed,'detail',detail,'error',error); %#ok<AGROW>
    end
end

function finishFilter(resolvedFilter,originalPath)
% Capture cleanup values rather than referencing an exiting shared workspace.
try,resolvedFilter('clear');catch,end
clear resolvedFilter
clear gpenmpc_px4_sensor_filter_mex
path(originalPath);
end

function d=checkFixture(c,p)
assert(c.gyro_cutoff_hz==40&&c.dgyro_cutoff_hz==30&&c.dynamic_notch_enable==0);
assert(c.sample_rate_hz==500&&~p.actual_selected_gyro_raw_rate_known&&~p.ratemax_used_as_fs);
s=gpenmpc_px4_sensor_filter_mex('init',c);assert(s.initialized&&s.sample_count==0&&s.reset_count==1);d=s;
end
function d=zeroInput(c)
init(c);largest=0;for k=1:64,y=step(zeros(3,1),1000000+2000*(k-1));largest=max(largest,max(abs([y.rate_body;y.derivative_body])));end
assert(largest==0);d=struct('samples',64,'max_absolute_output',largest);
end
function d=constantInput(c)
c.initial_gyro_uncalibrated=[.25;-.5;1];init(c);peak=0;
for k=1:32,y=step(c.initial_gyro_uncalibrated,1000000+2000*(k-1));close(y.rate_body,c.initial_gyro_uncalibrated,2e-5);peak=max(peak,max(abs(y.derivative_body)));end
assert(peak<2e-3);d=struct('samples',32,'derivative_peak',peak);
end
function d=calibrationDirection(c)
c=bypass(c);c.mount_rotation=[0,-1,0;1,0,0;0,0,1];c.offset_sensor=[.125;-.25;.5];c.bias_body=[.25;.5;-.125];init(c);
y=step([1;2;3],1000000);expected=c.mount_rotation*([1;2;3]-c.offset_sensor)-c.bias_body;
close(y.rate_body,expected,1e-6);close(y.derivative_body,c.mount_rotation*y.filtered_derivative_sensor,1e-5);d=struct('rate_body',y.rate_body,'expected',expected);
end
function d=axisDirection(c,a)
c=bypass(c);c.mount_rotation=[0,-1,0;1,0,0;0,0,1];init(c);u=zeros(3,1);u(a)=1;y=step(u,1000000);
close(y.rate_body,c.mount_rotation*u,1e-6);d=struct('axis',a,'body',y.rate_body);
end
function d=allBypass(c)
c=bypass(c);s=init(c);assert(~s.gyro_lowpass_enabled&&~s.derivative_lowpass_enabled&&~s.notch0_enabled&&~s.notch1_enabled);
u=[.125;-.25;.5];y=step(u,1000000);close(y.filtered_gyro_sensor,u,0);close(y.filtered_derivative_sensor,y.difference_derivative_sensor,0);d=s;
end
function d=nyquist(c)
c.gyro_cutoff_hz=c.sample_rate_hz/2;c.dgyro_cutoff_hz=c.sample_rate_hz/2;
c.notch0_frequency_hz=c.sample_rate_hz/2;c.notch1_frequency_hz=c.sample_rate_hz;d=allBypassLike(c);
end
function d=negativeCutoffs(c)
c.gyro_cutoff_hz=-1;c.dgyro_cutoff_hz=-1;c.notch0_frequency_hz=-1;c.notch1_frequency_hz=-1;d=allBypassLike(c);
end
function d=allBypassLike(c)
s=init(c);assert(~s.gyro_lowpass_enabled&&~s.derivative_lowpass_enabled&&~s.notch0_enabled&&~s.notch1_enabled);
y=step([.25;-.5;1],1000000);close(y.rate_body,[.25;-.5;1],0);close(y.filtered_derivative_sensor,y.difference_derivative_sensor,0);d=s;
end
function d=differenceNotTruth(c)
c.dgyro_cutoff_hz=0;init(c);a=step(zeros(3,1),1000000);b=step([1;0;0],1002000);
expected=double(single(single(b.filtered_gyro_sensor)-single(a.filtered_gyro_sensor)).*single(1/single(b.native_effective_dt_s)));
close(b.difference_derivative_sensor,expected,1e-5);assert(abs(b.derivative_body(1))>1);
d=struct('expected_backward_difference',expected,'output',b.derivative_body,'wrong_analytic_alpha_fixture',zeros(3,1),'wrong_analytic_alpha_rejected',true);
end
function d=firstSample(c)
c=bypass(c);init(c);y=step([.1;0;0],1000000);
previous=uint64(fix(single(uint64(1000000))-single(1e6)/single(c.sample_rate_hz)));
assert(y.native_dt_backfilled&&~y.had_previous_sample&&y.raw_observed_dt_s==0&&y.effective_previous_timestamp_us==previous);
assert(y.timestamp_sample_us==uint64(1000000)&&y.sample_count==1);d=y;
end
function d=clippedDt(c)
c=bypass(c);init(c);step(zeros(3,1),1000000);a=step([1;0;0],1000001);b=step([2;0;0],1040001);
close(a.native_effective_dt_s,double(single(.00002)),0);close(b.native_effective_dt_s,double(single(.02)),0);
assert(a.native_dt_clamped&&b.native_dt_clamped);close(a.derivative_body,[50000;0;0],.01);close(b.derivative_body,[50;0;0],1e-5);
d=struct('minimum_dt_s',a.native_effective_dt_s,'maximum_dt_s',b.native_effective_dt_s,'min_raw',a.raw_observed_dt_s,'max_raw',b.raw_observed_dt_s);
end
function d=resetClock(c,resetSource)
c=bypass(c);init(c);step([.5;0;0],1000000);r=resetStruct([.5;0;0],zeros(3,1),resetSource);s=gpenmpc_px4_sensor_filter_mex('reset',r);
if resetSource,assert(s.sample_count==0&&s.previous_timestamp_sample_us==0);else,assert(s.sample_count==1&&s.previous_timestamp_sample_us==1000000);end
y=step([.5;0;0],1002000);close(y.derivative_body,zeros(3,1),0);assert(y.native_dt_backfilled==resetSource);d=s;
end
function d=warmUncorrect(c)
c.mount_rotation=[0,-1,0;1,0,0;0,0,1];c.offset_sensor=[.1;.2;.3];c.bias_body=[.02;-.01;.03];init(c);
for k=1:8,y=step([.4;.7;.9],1000000+2000*k);end
gyro=c.mount_rotation.'*(y.rate_body+c.bias_body)+c.offset_sensor;alpha=c.mount_rotation.'*y.derivative_body;
s=gpenmpc_px4_sensor_filter_mex('reset',resetStruct(gyro,alpha,false));close(s.previous_filtered_gyro_sensor,y.filtered_gyro_sensor,2e-6);
assert(s.previous_timestamp_sample_us==y.timestamp_sample_us);d=struct('uncorrected_gyro',gyro,'uncorrected_derivative',alpha,'reset_count',s.reset_count);
end
function d=sequenceReset(c,reversed)
c=bypass(c);init(c);step([.1;0;0],1004000);t=1004000;if reversed,t=1002000;end
y=step([.2;0;0],t);s=gpenmpc_px4_sensor_filter_mex('status');assert(y.must_stop&&~y.source_progressed&&s.source_fault_latched&&y.native_dt_backfilled);
r=gpenmpc_px4_sensor_filter_mex('reset',resetStruct(zeros(3,1),zeros(3,1),true));assert(~r.source_fault_latched&&r.sample_count==0);
z=step(zeros(3,1),2000000);assert(~z.must_stop&&z.native_dt_backfilled);d=struct('failure_row',y,'explicit_reset',r);
end
function d=sequenceReject(c,badResetOnly)
init(c);step(zeros(3,1),1004000);y=step(zeros(3,1),1004000);assert(y.must_stop);
if badResetOnly,ok=throws(@()gpenmpc_px4_sensor_filter_mex('reset',resetStruct(zeros(3,1),zeros(3,1),false)));
else,ok=throws(@()gpenmpc_px4_sensor_filter_mex('step',sample(zeros(3,1),1006000)));end
assert(ok&&throws(@()gpenmpc_px4_sensor_filter_mex('status')));d=struct('rejected',ok,'object_cleared',true,'requires_explicit_init_after_invalid_api_call',true);
end
function d=invalidInit(c,bad)
init(c);ok=throws(@()gpenmpc_px4_sensor_filter_mex('init',bad));assert(ok&&throws(@()gpenmpc_px4_sensor_filter_mex('status')));d=struct('rejected',true,'object_cleared',true);
end
function d=invalidStep(c,bad)
init(c);ok=throws(@()gpenmpc_px4_sensor_filter_mex('step',bad));assert(ok&&throws(@()gpenmpc_px4_sensor_filter_mex('status')));d=struct('rejected',true,'object_cleared',true);
end
function d=unknownCommand(c)
init(c);assert(throws(@()gpenmpc_px4_sensor_filter_mex('unknown',struct()))&&throws(@()gpenmpc_px4_sensor_filter_mex('status')));d=struct('rejected',true,'object_cleared',true);
end
function d=badReset(c)
init(c);assert(throws(@()gpenmpc_px4_sensor_filter_mex('reset',resetStruct([NaN;0;0],zeros(3,1),true)))&&throws(@()gpenmpc_px4_sensor_filter_mex('status')));d=struct('rejected',true,'object_cleared',true);
end

function d=oracleCase(c,notchEnabled)
if notchEnabled,c.notch0_frequency_hz=60;c.notch0_bandwidth_hz=15;c.notch1_frequency_hz=90;c.notch1_bandwidth_hz=20;end
init(c);state=oracleInit(c);maxRate=0;maxD=0;notchDelta=0;n=256;
if notchEnabled,bypassCfg=c;bypassCfg.notch0_frequency_hz=0;bypassCfg.notch1_frequency_hz=0;bypassState=oracleInit(bypassCfg);end
for k=1:n
    u=double(single([.2*sin(.754*k)+.01*k/n;.15*cos(1.13*k);.1*sin(.18*k)]));
    y=step(u,1000000+2000*(k-1));[expected,state]=oracleStep(u,c,state,single(y.native_effective_dt_s));
    close(y.filtered_gyro_sensor,double(expected.rate),2e-5);close(y.filtered_derivative_sensor,double(expected.derivative),2e-3);
    maxRate=max(maxRate,max(abs(y.filtered_gyro_sensor-double(expected.rate))));maxD=max(maxD,max(abs(y.filtered_derivative_sensor-double(expected.derivative))));
    if notchEnabled,[other,bypassState]=oracleStep(u,bypassCfg,bypassState,single(y.native_effective_dt_s));notchDelta=max(notchDelta,max(abs(double(expected.rate-other.rate))));end
end
if notchEnabled,assert(notchDelta>1e-3);end
d=struct('samples',n,'max_rate_difference',maxRate,'max_derivative_difference',maxD,'static_notch_action_vs_disabled',notchDelta,'arithmetic','single direct source formulas');
end
function s=oracleInit(c)
s.lp=lpCoefficients(single(c.sample_rate_hz),single(c.gyro_cutoff_hz));
s.d1=zeros(3,1,'single');s.d2=s.d1;g=single(c.initial_gyro_uncalibrated);
den=single(single(1)+s.lp.a1+s.lp.a2);if abs(den)>eps('single'),s.d1=g./den;s.d2=s.d1;else,s.d1=g;s.d2=g;end
[~,s]=lpApply(g,s);s.previous=g;s.derivative=single(c.initial_acceleration_uncalibrated);
fs=single(c.sample_rate_hz);fc=single(c.dgyro_cutoff_hz);
if fc>0&&fc<fs/2,dt=single(1)/fs;tau=single(1)/(single(6.28318531)*fc);s.alpha=dt/(tau+dt);else,s.alpha=single(1);end
s.n0=notchCoefficients(fs,single(c.notch0_frequency_hz),single(c.notch0_bandwidth_hz));
s.n1=notchCoefficients(fs,single(c.notch1_frequency_hz),single(c.notch1_bandwidth_hz));
end
function [y,s]=oracleStep(raw,c,s,dt) %#ok<INUSD>
u=single(raw);[u,s.n0]=notchApply(u,s.n0);[u,s.n1]=notchApply(u,s.n1);[rate,s]=lpApply(u,s);
diff=single(rate-s.previous).*single(1/dt);s.derivative=single(s.derivative+single(s.alpha.*single(diff-s.derivative)));s.previous=rate;
y=struct('rate',rate,'derivative',s.derivative);
end
function p=lpCoefficients(fs,fc)
p=struct('a1',single(0),'a2',single(0),'b0',single(1),'b1',single(0),'b2',single(0));
if fc<=0||fc>=fs/2,return;end
fc=max(fc,fs*single(.001));ohm=tan(single(3.14159265)/(fs/fc));cc=cos(single(3.14159265)/single(4));
c=single(single(1)+single(single(2)*cc*ohm)+single(ohm*ohm));
p.b0=single(ohm*ohm)/c;p.b1=single(2)*p.b0;p.b2=p.b0;
p.a1=single(2)*single(ohm*ohm-single(1))/c;p.a2=single(single(1)-single(single(2)*cc*ohm)+single(ohm*ohm))/c;
end
function [out,s]=lpApply(u,s)
p=s.lp;v=single(single(u-single(s.d1*p.a1))-single(s.d2*p.a2));
out=single(single(v*p.b0+s.d1*p.b1)+s.d2*p.b2);s.d2=s.d1;s.d1=v;
end
function p=notchCoefficients(fs,f,bw)
p=struct('enabled',false,'initialized',false,'a1',single(0),'a2',single(0),...
    'b0',single(1),'b1',single(0),'b2',single(0),'x1',zeros(3,1,'single'),'x2',zeros(3,1,'single'),'y1',zeros(3,1,'single'),'y2',zeros(3,1,'single'));
if f<=0||bw<=0||f>=fs/2,return;end
f=max(f,fs*single(.001));bw=max(bw,fs*single(.001));alpha=tan(single(3.14159265)*bw/fs);beta=-cos(single(2)*single(3.14159265)*f/fs);a0=single(1)/single(alpha+single(1));
p.b0=a0;p.b1=single(2)*beta*a0;p.b2=a0;p.a1=p.b1;p.a2=single(single(1)-alpha)*a0;p.enabled=true;
end
function [out,p]=notchApply(u,p)
if ~p.enabled,out=u;return;end
if ~p.initialized,p.x1=u;p.x2=u;p.y1=single(u*single(p.b0+p.b1+p.b2)/single(single(1)+p.a1+p.a2));p.y2=p.y1;p.initialized=true;end
out=single(single(single(single(p.b0*u+p.b1*p.x1)+p.b2*p.x2)-p.a1*p.y1)-p.a2*p.y2);
p.x2=p.x1;p.x1=u;p.y2=p.y1;p.y1=out;
end
function c=bypass(c),c.gyro_cutoff_hz=0;c.dgyro_cutoff_hz=0;c.notch0_frequency_hz=0;c.notch1_frequency_hz=0;end
function s=init(c),s=gpenmpc_px4_sensor_filter_mex('init',c);end
function s=sample(u,t),s=struct('raw_gyro',double(u),'timestamp_sample_us',uint64(t));end
function y=step(u,t),y=gpenmpc_px4_sensor_filter_mex('step',sample(u,t));end
function r=resetStruct(g,a,b),r=struct('gyro_uncalibrated',double(g),'acceleration_uncalibrated',double(a),'reset_source_timestamp',logical(b));end
function close(a,b,tol),assert(isequal(size(a),size(b))&&all(isfinite(a(:)))&&all(isfinite(b(:)))&&all(abs(a(:)-b(:))<=tol+4e-5*abs(b(:))),'gpenmpc:FilterOracleMismatch');end
function ok=throws(fn)
ok=false;try,ignored=fn();catch e,ok=strcmp(e.identifier,'gpenmpc:NativeSensorFilter');end %#ok<NASGU>
end
function result=identity(path)
path=char(path);assert(isfile(path));f=fopen(path,'rb');assert(f>=0);c=onCleanup(@()fclose(f)); %#ok<NASGU>
b=fread(f,Inf,'*uint8');d=java.security.MessageDigest.getInstance('SHA-256');d.update(b);
result=struct('path',path,'bytes',numel(b),'sha256',upper(reshape(dec2hex(typecast(d.digest(),'uint8'),2).',1,[])));
end
