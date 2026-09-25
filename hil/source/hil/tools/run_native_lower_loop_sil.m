function report=run_native_lower_loop_sil(mexDir,outputDir,configurationMode,filterSampleRateHz,candidateFile)
% Run a host small-signal experiment with the PX4 lower loop and
% 19-state M600 RK4 plant using ideal state feedback.
arguments
    mexDir (1,1) string
    outputDir (1,1) string
    configurationMode (1,1) string = "EXPLICIT_HOST_FIXTURE"
    filterSampleRateHz (1,1) double = 0
    candidateFile (1,1) string = ""
end
assert(~isfolder(outputDir)&&~isfile(outputDir),'gpenmpc:ExistingOutput','No overwrite.');
assert(ismember(filterSampleRateHz,[0,100,1000]),'gpenmpc:FilterSampleRate','Only 0/100/1000 are prospective choices.');
assert(filterSampleRateHz==0||configurationMode=="CURRENT_TYPED_LOWER_LOOP", ...
    'gpenmpc:FilterConfiguration','Positive filter fs requires CURRENT_TYPED_LOWER_LOOP.');
build=string(fileparts(fileparts(mfilename('fullpath'))));
restorePath=path;kernelFunction=[];filterFunction=[];
try
addpath(mexDir,fullfile(build,'tools'),fullfile(build,'matlab_validation'), ...
    fullfile(build,'m600_coptersim','matlab_validation'),'-begin');
binary=fullfile(mexDir,'gpenmpc_px4_native_lower_loop_mex.mexw64');
assert(strcmpi(which('gpenmpc_px4_native_lower_loop_mex'),binary));
kernelFunction=@gpenmpc_px4_native_lower_loop_mex;
filterEnabled=filterSampleRateHz>0;filterCfg=struct();filterMetadata=struct();filterBinding=struct();
if filterEnabled
    filterDir=gpenmpc_external_path('native_sensor_filter_mex');
    filterBuild=jsondecode(fileread(fullfile(filterDir,'BUILD_RESULT.json')));assert(filterBuild.passed);
    filterBinary=fullfile(filterDir,'gpenmpc_px4_sensor_filter_mex.mexw64');
    filterBinding=verifyIdentity(filterBinary,filterBuild.binary);
    for j=1:numel(filterBuild.bindings),verifyIdentity(filterBuild.bindings(j).path,filterBuild.bindings(j));end
    addpath(filterDir,'-begin');assert(strcmpi(which('gpenmpc_px4_sensor_filter_mex'),filterBinary));
    filterFunction=@gpenmpc_px4_sensor_filter_mex;
    [filterCfg,filterMetadata]=native_sensor_filter_explicit_fixture(filterSampleRateHz);
    filterMetadata.fixture_source_sha256=m600check.fileSha256(which('native_sensor_filter_explicit_fixture'));
    filterMetadata.filter_build_receipt_sha256=m600check.fileSha256(fullfile(filterDir,'BUILD_RESULT.json'));
end
catch setupError
    finishKernels(kernelFunction,filterFunction,restorePath);
    rethrow(setupError);
end
% Capture cleanup values and clear both objects before releasing MEX files or paths.
cleanup=onCleanup(@()finishKernels(kernelFunction,filterFunction,restorePath)); %#ok<NASGU>
% loadFixture only installs and verifies the original numerical dependencies.
% Its default mission parameters are discarded; exact model009 MAT is used.
m600check.loadFixture();
parameterFile=fullfile(gpenmpc_external_path('flat_terrain_model'),'M600_CORE_PARAMETERS.mat');
assert(strcmpi(m600check.fileSha256(parameterFile), ...
    '969D347CE041715CAED49BE0B7897486E34FA4E0B22EB82E50E9752B8F4F19E4'));
loaded=load(parameterFile,'parameters','environment');p=loaded.parameters;env=loaded.environment;
assert(any(configurationMode==["EXPLICIT_HOST_FIXTURE","CURRENT_TYPED_LOWER_LOOP"]));
if configurationMode=="CURRENT_TYPED_LOWER_LOOP"
    [cfg,metadata]=native_lower_loop_current_config();
else
    [cfg,metadata]=native_lower_loop_explicit_fixture();
end
candidateBinding=struct();parameterChanges=0;
if strlength(candidateFile)>0
    assert(configurationMode=="CURRENT_TYPED_LOWER_LOOP",'gpenmpc:CandidateCurrentConfigRequired');
    assert(strcmpi(m600check.fileSha256(candidateFile), ...
        'A6C93CC99FF39A6B84CD132E9DB0A4471E60D125781AE7BFD7677A260CD29C19'), ...
        'gpenmpc:CandidateIdentityChanged');
    candidate=jsondecode(fileread(candidateFile));
    assert(candidate.passed&&~candidate.SIL_outcomes_read&&~candidate.live_admission ...
        &&candidate.parameter_writes==0&&candidate.hardware_actions==0);
    assert(isequaln(candidate.original_configuration,cfg),'gpenmpc:CandidateOriginalMismatch');
    restored=candidate.candidate_configuration;restored.attitude_p=cfg.attitude_p;
    assert(isequaln(restored,cfg)&&numel(candidate.selection.parameter_changes)==2 ...
        &&strcmp(candidate.selection.common_REAL32_bits,'4026CCBE') ...
        &&all(candidate.candidate_configuration.attitude_p(1:2)==double(typecast(uint32(hex2dec('4026CCBE')),'single'))), ...
        'gpenmpc:CandidateChangeSurface');
    cfg=candidate.candidate_configuration;parameterChanges=2;
    candidateBinding=struct('path',candidateFile,'sha256',m600check.fileSha256(candidateFile), ...
        'selection',candidate.selection,'no_live_parameter_apply',true);
end
payload=2.21;mass=p.profile.mass_properties.base_mass_kg+payload+p.mission.plant_mismatch.mass_bias_kg;
assert(abs(mass-11.71)<1e-12);
upper=p.calibration.rotor_allocation.per_rotor_thrust_upper_n;
hoverN=mass*9.80665/6;hover=hoverN/upper;
cfg.initial_motor_commands=repmat(hover,6,1);
fixture=struct('dt_s',.01,'duration_s',8,'initial_height_m',10, ...
    'perturbation_deg',.05,'payload_kg',payload,'mass_kg',mass, ...
    'hover_per_rotor_N',hoverN,'collective_normalized',hover, ...
    'controller_parameter_changes',parameterChanges,'wind_xy_mps',[0;0], ...
    'position_loop_present',false,'ekf_present',false, ...
    'source','Exact neutral model009; original thrust lag/contact/rigid-body equations', ...
    'delay_interpretation','Fixed 0/10ms rate+alpha sample delay; explicit held equilibrium prehistory at -0.01s, native MEX timestamp follows that source; not an identified driver delay', ...
    'maybe_landed_interpretation','Paired I-enabled/I-frozen numerical inputs.');
fixture.sensor_filter=struct('enabled',filterEnabled,'sample_rate_hz',filterSampleRateHz, ...
    'actual_gyro_sample_rate_observed',false,'IMU_GYRO_RATEMAX_used_as_rate',false, ...
    'source_clock_epoch_us',uint64(1000000),'subsamples_per_10ms_plant_step',max(1,filterSampleRateHz/100), ...
    'interpretation','PROSPECTIVE_SOURCE_CLOCK_ZOH_FIXTURE__NOT_OBSERVED_GYRO_RATE_OR_WORKQUEUE_REPLAY', ...
    'raw_gyro_hold','One selected delayed plant-grid gyro value is held across each 10ms frame.', ...
    'attitude_update','Current ideal quaternion consumed only on first substep; rate setpoint held on later substeps.', ...
    'plant_actuation','All substep commands are recorded; the final command drives one 10 ms RK4 plant step.', ...
    'D_input','When enabled, angular acceleration uses the native filtered-gyro backward difference and D lowpass.', ...
    'calibration','Fixture calibration: sensor/body identity, zero offset/bias and unity scale.');
if filterEnabled
    fixture.delay_interpretation='Fixture delay: 0/10 ms on the raw-gyro plant grid with held equilibrium prehistory and matching source timestamps.';
end
% Use fixed diagnostic cases.
specs=struct('name',{},'axis',{},'angle_deg',{},'integral_enabled',{},'rate_delay_steps',{});
specs(end+1)=spec('ZERO_ERROR_EQUILIBRIUM',1,0,true,0);
for axis=1:2
    axisNames={'ROLL','PITCH'};
    for iEnabled=[false true]
        iNames={'I_FROZEN','I_ENABLED'};
        for delay=[0 1]
            specs(end+1)=spec(sprintf('%s_%s_RATE_DELAY_%dMS',axisNames{axis},iNames{1+iEnabled},delay*10), ...
                axis,fixture.perturbation_deg,iEnabled,delay); %#ok<AGROW>
        end
    end
end
specs(end+1)=spec('ROLL_I_ENABLED_NEGATIVE',1,-fixture.perturbation_deg,true,0);
specs(end+1)=spec('PITCH_I_ENABLED_NEGATIVE',2,-fixture.perturbation_deg,true,0);
mkdir(outputDir);allData=cell(numel(specs),1);results=struct([]);checks=struct('name',{},'passed',{});
failure='';
try
    for c=1:numel(specs)
        s=specs(c);ok=gpenmpc_px4_native_lower_loop_mex('init',cfg);assert(ok);
        if filterEnabled,filterInitial=filterFunction('init',filterCfg);assert(filterInitial.initialized);end
        x=zeros(19,1);x(3)=fixture.initial_height_m;x(7)=cosd(s.angle_deg/2);
        qNed=[cosd(s.angle_deg/2);0;0;0];qNed(1+s.axis)=sind(s.angle_deg/2);
        x(7:10)=qNed.*[1;-1;-1;1];x(14:19)=hoverN;
        memory=m600check.initialStepMemory();memory.airborne_observed=true;
        memory.ground_confirmed=false;memory.ground_support_dwell_s=0;memory.previous_contact_active=false;
        jet=zeros(12,1);jet(3)=fixture.initial_height_m;
        controls=zeros(16,1);controls(1:6)=hover;
        n=round(fixture.duration_s/fixture.dt_s);
        X=zeros(n+1,19);X(1,:)=x.';U=zeros(n,6);Y=zeros(n,3);Q=Y;Torque=Y;Integral=Y;
        limits=false(n,3);contacts=false(n,1);dtClamped=false(n,1);
        sourceTimes=zeros(n,1);minimumRateTime=zeros(n,1);accepted=false(n,1);
        rateHistory=zeros(n,3);alphaHistory=zeros(n,3);appliedRate=zeros(n,3);appliedAlpha=zeros(n,3);postEuler=zeros(n,3);
        subsamples=struct();subRow=0;filterDtClamped=false(n,1);substeps=max(1,filterSampleRateHz/100);
        if filterEnabled,subsamples=allocateSubsamples(n*substeps);end
        rowCount=0;termination='FULL_DIAGNOSTIC_WINDOW';
        for k=1:n
            t=(k-1)*fixture.dt_s;
            % Command order changes only inside the existing RK4 adapter.
            rotor=controls([5;1;4;6;2;3])*upper;
            [dx,d,~]=m600check.derivativeSoftware(x,rotor,jet,payload,[0;0],t,p);
            obs=m600check.copterSimOutputs(x,d.actual_acceleration_mps2,[0;0],dx(11:13),mass);
            rateHistory(k,:)=obs.body_rate_frd_rad_s.';alphaHistory(k,:)=obs.angular_acceleration_body_frd_rad_s2.';
            delayed=max(1,k-s.rate_delay_steps);
            rateSourceTime=t-s.rate_delay_steps*fixture.dt_s;
            sample=struct('timestamp_sample_s',rateSourceTime,'initial_dt_s',fixture.dt_s, ...
                'q_est',obs.quaternion_wxyz_body_to_ned,'q_reference',[1;0;0;0], ...
                'body_rate',rateHistory(delayed,:).','angular_acceleration',alphaHistory(delayed,:).', ...
                'thrust_body_normalized',[0;0;-hover],'yaw_rate_ff',0,'battery_scale',1, ...
                'armed',true,'rotary_wing',true,'landed',false,'maybe_landed',~s.integral_enabled, ...
                'rates_enabled',true,'attitude_updated',true);
            if filterEnabled
                % Hold the raw gyro input within each plant frame.
                frameLimits=false(1,3);frameDtClamped=false;frameFilterDtClamped=false;
                heldRaw=rateHistory(delayed,:).';
                for sub=1:substeps
                    sourceUs=uint64(1000000+round(rateSourceTime*1e6)+(sub-1)*(1e6/filterSampleRateHz));
                    filtered=filterFunction('step',struct('raw_gyro',heldRaw,'timestamp_sample_us',sourceUs));
                    assert(~filtered.must_stop&&filtered.source_progressed,'gpenmpc:FilterSourceFault','Source did not progress; no reset/restart permitted.');
                    sample.timestamp_sample_s=double(sourceUs)*1e-6;
                    sample.initial_dt_s=1/filterSampleRateHz;
                    sample.body_rate=filtered.rate_body;sample.angular_acceleration=filtered.derivative_body;
                    sample.attitude_updated=(sub==1);
                    out=gpenmpc_px4_native_lower_loop_mex('step',sample);
                    subRow=subRow+1;
                    subsamples=recordSubsample(subsamples,subRow,k,sub,t,rateSourceTime,sourceUs,heldRaw,filtered,sample,out);
                    frameLimits=frameLimits|[out.motor_saturated,out.rate_limit_reached,out.integral_limit_reached];
                    frameDtClamped=frameDtClamped||out.dt_clamped;
                    frameFilterDtClamped=frameFilterDtClamped||filtered.native_dt_clamped;
                end
            else
                % Legacy path: same analytic-alpha, one-controller-step and
                % timestamp semantics as the original filter0 diagnostic.
                out=gpenmpc_px4_native_lower_loop_mex('step',sample);
                frameLimits=[out.motor_saturated,out.rate_limit_reached,out.integral_limit_reached];
                frameDtClamped=out.dt_clamped;frameFilterDtClamped=false;
            end
            appliedRate(k,:)=sample.body_rate.';appliedAlpha(k,:)=sample.angular_acceleration.';
            controls=zeros(16,1);controls(1:6)=out.motor_commands;
            [next,~,contact,nextMemory,status]=m600check.stepPx4Rk4(x,controls,jet,payload,[0;0],t,p,memory);
            rowCount=k;X(k+1,:)=next.';U(k,:)=out.motor_commands.';
            postObs=m600check.copterSimOutputs(next,[0;0;0],[0;0],[0;0;0],mass);
            postEuler(k,:)=rad2deg(postObs.euler_rpy_rad).';
            Y(k,:)=rad2deg(obs.euler_rpy_rad).';Q(k,:)=out.rate_setpoint.';
            Torque(k,:)=out.torque_applied.';Integral(k,:)=out.integral.';
            limits(k,:)=frameLimits;
            contacts(k)=contact.contact_active;dtClamped(k)=frameDtClamped;
            filterDtClamped(k)=frameFilterDtClamped;
            sourceTimes(k)=t;minimumRateTime(k)=rateSourceTime;accepted(k)=status.accepted;
            if ~status.accepted,termination=sprintf('PLANT_REJECT_%d',status.failure_code);break;end
            x=next;memory=nextMemory;
            if contact.contact_active,termination='CONTACT_ENTERED_FREE_FLIGHT_FIXTURE_ENDED';break;end
            if max(abs(postEuler(k,1:2)))>=1,termination='OUTSIDE_PREDECLARED_1DEG_SMALL_SIGNAL_DOMAIN';break;end
        end
        clearKernel();if filterEnabled,filterFunction('clear');end;nUsed=rowCount;
        tr=table(sourceTimes(1:nUsed),minimumRateTime(1:nUsed),Y(1:nUsed,1),Y(1:nUsed,2),Y(1:nUsed,3), ...
            X(1:nUsed,3),accepted(1:nUsed),contacts(1:nUsed),dtClamped(1:nUsed), ...
            'VariableNames',{'time_s','rate_sample_source_time_s','roll_deg','pitch_deg','yaw_deg', ...
            'height_up_m','plant_accepted','plant_contact','controller_dt_clamped'});
        for a=1:6,tr.(sprintf('board_motor%d',a))=U(1:nUsed,a);end
        for a=1:3
            tr.(sprintf('torque_axis%d',a))=Torque(1:nUsed,a);tr.(sprintf('integral_axis%d',a))=Integral(1:nUsed,a);
            tr.(sprintf('post_step_euler_axis%d_deg',a))=postEuler(1:nUsed,a);
            tr.(sprintf('actual_rate_feedback_axis%d_rad_s',a))=appliedRate(1:nUsed,a);
            tr.(sprintf('actual_alpha_feedback_axis%d_rad_s2',a))=appliedAlpha(1:nUsed,a);
        end
        tr.post_step_time_s=sourceTimes(1:nUsed)+fixture.dt_s;
        if filterEnabled
            tr.sensor_filter_dt_clamped=filterDtClamped(1:nUsed);
            tr.any_subsample_dt_clamped=dtClamped(1:nUsed)|filterDtClamped(1:nUsed);
            tr.sensor_filter_subsamples=repmat(substeps,nUsed,1);
        end
        writetable(tr,fullfile(outputDir,string(s.name)+'.csv'));
        peaks=envelope([sourceTimes(1:nUsed);sourceTimes(nUsed)+fixture.dt_s],[Y(1:nUsed,s.axis);postEuler(nUsed,s.axis)]);
        r=struct('name',s.name,'specification',s,'rows',nUsed,'termination',termination, ...
            'max_attitude_deg',max(abs([Y(1:nUsed,:);postEuler(nUsed,:)]),[],1), ...
            'min_height_m',min(X(1:nUsed+1,3)),'max_height_m',max(X(1:nUsed+1,3)), ...
            'motor_saturated_rows',nnz(limits(1:nUsed,1)),'rate_limited_rows',nnz(limits(1:nUsed,2)), ...
            'integral_limited_rows',nnz(limits(1:nUsed,3)),'contact_rows',nnz(contacts(1:nUsed)), ...
            'dt_clamped_rows',nnz(dtClamped(1:nUsed)),'finite',all(isfinite(X(1:nUsed+1,:)),'all'), ...
            'envelope',peaks,'controller_gains_unchanged',parameterChanges==0,'hardware_actions',0);
        if filterEnabled
            r.filter_sample_rate_hz=filterSampleRateHz;r.sensor_subsamples=subRow;
            r.filter_dt_clamped_rows=nnz(filterDtClamped(1:nUsed));
            r.any_subsample_dt_clamped_rows=nnz(dtClamped(1:nUsed)|filterDtClamped(1:nUsed));
            r.sensor_source_clock_progressed=all(subsamples.source_timestamp_us(2:subRow)>subsamples.source_timestamp_us(1:subRow-1));
            r.feedback_derivative_basis='SAME_SOURCE_FILTERED_GYRO_BACKWARD_DIFFERENCE_THEN_NATIVE_D_LOWPASS';
        end
        if isempty(results),results=r;else,results(end+1)=r;end %#ok<AGROW>
        allData{c}=struct('stateUp',X(1:nUsed+1,:),'controlsBoard',U(1:nUsed,:), ...
            'rateSetpoint',Q(1:nUsed,:),'currentRate',rateHistory(1:nUsed,:), ...
            'currentAlpha',alphaHistory(1:nUsed,:),'appliedRateFeedback',appliedRate(1:nUsed,:), ...
            'appliedAlphaFeedback',appliedAlpha(1:nUsed,:),'limits',limits(1:nUsed,:), 'trace',tr);
        if filterEnabled
            allData{c}.sensorFilterSubsamples=trimSubsamples(subsamples,subRow);
            allData{c}.sensorFilterConfiguration=filterCfg;
        end
        check([s.name '_FINITE_AND_PLANT_ACCEPTED'],r.finite&&all(accepted(1:nUsed)));
        check([s.name '_NO_GROUND_CONTACT'],r.contact_rows==0);
        check([s.name '_NO_LIMIT_ACTIVATION'],~any(limits(1:nUsed,:),'all'));
        check([s.name '_TIME_STEP_UNCLAMPED'],r.dt_clamped_rows==0);
        if filterEnabled
            check([s.name '_FILTER_TIME_STEP_UNCLAMPED'],r.filter_dt_clamped_rows==0);
            check([s.name '_SAME_SOURCE_CLOCK_STRICT_PROGRESS'],r.sensor_source_clock_progressed&&subRow==nUsed*substeps);
            check([s.name '_ONE_ATTITUDE_UPDATE_PER_PLANT_FRAME'],nnz(subsamples.attitude_updated(1:subRow))==nUsed);
            check([s.name '_SENSOR_OUTPUT_FINITE'],all(isfinite(subsamples.rate_body(1:subRow,:)),'all')&& ...
                all(isfinite(subsamples.derivative_body(1:subRow,:)),'all'));
        end
        if ~s.integral_enabled,check([s.name '_INTEGRAL_STAYS_ZERO'],all(Integral(1:nUsed,:)==0,'all'));end
        fprintf('HOST lower-loop SIL %s: %s; max roll/pitch %.6f/%.6f deg\n',s.name,termination,r.max_attitude_deg(1:2));
    end
    zero=results(1);check('ZERO_ERROR_EQUILIBRIUM_FULL_8S',zero.rows==800&&max(zero.max_attitude_deg)<.001&& ...
        max(abs([zero.min_height_m,zero.max_height_m]-10))<.001);
catch err
    failure=getReport(err,'extended','hyperlinks','off');
end
report=struct('classification','HOST_PX4_LOWER_LOOP_M600_SIL_DIAGNOSTIC', ...
    'execution_completed',isempty(failure),'test_checks_passed',~isempty(checks)&&all([checks.passed]), ...
    'failure',failure,'fixture',fixture,'configuration_mode',configurationMode, ...
    'parameter_metadata',metadata,'configuration',cfg, ...
    'candidate_binding',candidateBinding,'controller_parameter_changes',parameterChanges, ...
    'filter_sample_rate_hz',filterSampleRateHz,'filter_configuration',filterCfg, ...
    'filter_parameter_metadata',filterMetadata,'filter_binary',filterBinding, ...
    'cases_planned',numel(specs),'cases_completed',numel(results),'cases',results,'checks',checks, ...
    'source_sha256',m600check.fileSha256([mfilename('fullpath') '.m']), ...
    'mex_sha256',m600check.fileSha256(binary),'parameters_sha256',m600check.fileSha256(parameterFile), ...
    'original_model_environment',env,'hardware_actions',0,'COM_actions',0,'live_admission',false, ...
    'terrain_source_cause_identified',false, ...
    'limitations',{{'The replay covers the native attitude/rate/allocator kernels and the numerical plant.', ...
    'filter0 uses analytic alpha; positive filter fs enables native static filter classes with a source-clock/ZOH fixture.', ...
    'Sensor sampling, zero calibration and initial states are explicit fixture inputs.', ...
    'The final substep command drives one 10 ms RK4 plant step.', ...
    'Rate/alpha delay is varied as a diagnostic contrast.', ...
    'Ideal plant state feedback provides the numerical SIL reference.', ...
    'Frozen-integral case uses a fixed integral state for paired comparison.'}});
save(fullfile(outputDir,'RAW_LOWER_LOOP_SIL.mat'),'report','allData','-v7.3');
f=fopen(fullfile(outputDir,'RESULT.json'),'w','n','UTF-8');assert(f>=0);fcloseGuard=onCleanup(@()fclose(f)); %#ok<NASGU>
fprintf(f,'%s\n',jsonencode(report,PrettyPrint=true));
assert(isempty(failure),'gpenmpc:SilExecutionFailed','%s',failure);
    function check(name,passed)
        checks(end+1)=struct('name',name,'passed',logical(passed)); %#ok<AGROW>
    end
end
function s=spec(name,axis,angle,integral,delay)
s=struct('name',name,'axis',axis,'angle_deg',angle,'integral_enabled',logical(integral),'rate_delay_steps',delay);
end
function r=envelope(t,y)
% All raw samples remain in CSV/MAT. Local extrema used only for diagnostics.
ind=find((y(2:end-1)-y(1:end-2)).*(y(3:end)-y(2:end-1))<0)+1;
ind=ind(abs(y(ind))>1e-6);growth=NaN;period=NaN;
if numel(ind)>=4
    fit=polyfit(t(ind),log(abs(y(ind))),1);growth=fit(1);
    if numel(ind)>=3,period=median(t(ind(3:end))-t(ind(1:end-2)));end
end
r=struct('peak_time_s',t(ind),'signed_peak_deg',y(ind),'log_amplitude_growth_per_s',growth, ...
    'median_same_sign_period_s',period,'nan_means','Insufficient extrema; not applicable');
end
function clearKernel()
gpenmpc_px4_native_lower_loop_mex('clear');
end
function finishKernels(kernelFunction,filterFunction,restorePath)
% Capture cleanup values locally.
if ~isempty(kernelFunction),try,kernelFunction('clear');catch,end,end
if ~isempty(filterFunction),try,filterFunction('clear');catch,end,end
kernelFunction=[];filterFunction=[]; %#ok<NASGU>
clear gpenmpc_px4_native_lower_loop_mex gpenmpc_px4_sensor_filter_mex
path(restorePath);
end
function actual=verifyIdentity(file,expected)
file=char(file);info=dir(file);assert(isscalar(info)&&~info.isdir,'gpenmpc:FilterBindingMissing');
f=fopen(file,'rb');assert(f>=0);c=onCleanup(@()fclose(f)); %#ok<NASGU>
bytes=fread(f,Inf,'*uint8');d=java.security.MessageDigest.getInstance('SHA-256');d.update(bytes);
actual=struct('path',file,'bytes',numel(bytes),'sha256',upper(reshape(dec2hex(typecast(d.digest(),'uint8'),2).',1,[])));
assert(actual.bytes==expected.bytes&&strcmpi(actual.sha256,expected.sha256),'gpenmpc:FilterBindingChanged');
end
function s=allocateSubsamples(n)
s=struct('plant_step',zeros(n,1),'substep',zeros(n,1),'plant_frame_time_s',zeros(n,1), ...
    'held_raw_plant_source_time_s',zeros(n,1),'source_timestamp_us',zeros(n,1,'uint64'), ...
    'controller_source_time_s',zeros(n,1),'previous_source_timestamp_us',zeros(n,1,'uint64'), ...
    'effective_previous_timestamp_us',zeros(n,1,'uint64'),'filter_observed_dt_s',zeros(n,1), ...
    'filter_effective_dt_s',zeros(n,1),'controller_dt_s',zeros(n,1), ...
    'filter_dt_clamped',false(n,1),'controller_dt_clamped',false(n,1),'filter_dt_backfilled',false(n,1), ...
    'source_progressed',false(n,1),'must_stop',false(n,1),'attitude_updated',false(n,1), ...
    'integral_updates_enabled',false(n,1),'limits',false(n,3), ...
    'raw_gyro_sensor',zeros(n,3),'filtered_gyro_sensor',zeros(n,3),'difference_derivative_sensor',zeros(n,3), ...
    'filtered_derivative_sensor',zeros(n,3),'rate_body',zeros(n,3),'derivative_body',zeros(n,3), ...
    'rate_setpoint',zeros(n,3),'torque_raw',zeros(n,3),'torque_applied',zeros(n,3), ...
    'motor_commands',zeros(n,6),'integral',zeros(n,3),'unallocated_control',zeros(n,6), ...
    'q_est',zeros(n,4),'filter_state_reason',strings(n,1));
end
function s=recordSubsample(s,j,k,sub,t,heldRawTime,sourceUs,raw,f,input,o)
s.plant_step(j)=k;s.substep(j)=sub;s.plant_frame_time_s(j)=t;s.held_raw_plant_source_time_s(j)=heldRawTime;
s.source_timestamp_us(j)=sourceUs;s.controller_source_time_s(j)=input.timestamp_sample_s;
assert(f.timestamp_sample_us==sourceUs&&input.timestamp_sample_s==double(f.timestamp_sample_us)*1e-6, ...
    'gpenmpc:SensorControllerClockMismatch');
s.previous_source_timestamp_us(j)=f.previous_timestamp_sample_us;s.effective_previous_timestamp_us(j)=f.effective_previous_timestamp_us;
s.filter_observed_dt_s(j)=f.raw_observed_dt_s;s.filter_effective_dt_s(j)=f.native_effective_dt_s;s.controller_dt_s(j)=o.dt_s;
s.filter_dt_clamped(j)=f.native_dt_clamped;s.controller_dt_clamped(j)=o.dt_clamped;s.filter_dt_backfilled(j)=f.native_dt_backfilled;
s.source_progressed(j)=f.source_progressed;s.must_stop(j)=f.must_stop;s.attitude_updated(j)=input.attitude_updated;
s.integral_updates_enabled(j)=o.integral_updates_enabled;s.limits(j,:)=[o.motor_saturated,o.rate_limit_reached,o.integral_limit_reached];
s.raw_gyro_sensor(j,:)=raw.';s.filtered_gyro_sensor(j,:)=f.filtered_gyro_sensor.';
s.difference_derivative_sensor(j,:)=f.difference_derivative_sensor.';s.filtered_derivative_sensor(j,:)=f.filtered_derivative_sensor.';
s.rate_body(j,:)=input.body_rate.';s.derivative_body(j,:)=input.angular_acceleration.';
s.rate_setpoint(j,:)=o.rate_setpoint.';s.torque_raw(j,:)=o.torque_raw.';s.torque_applied(j,:)=o.torque_applied.';
s.motor_commands(j,:)=o.motor_commands.';s.integral(j,:)=o.integral.';s.unallocated_control(j,:)=o.unallocated_control.';
s.q_est(j,:)=input.q_est.';s.filter_state_reason(j)=string(f.filter_state_reason);
end
function s=trimSubsamples(s,n)
names=fieldnames(s);for k=1:numel(names),s.(names{k})=s.(names{k})(1:n,:);end
end
