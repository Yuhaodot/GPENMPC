function report=run_native_position_attitude_sil(outputDir)
% Test PositionControl, attitude, rate and allocation with the M600 plant.
arguments
    outputDir (1,1) string
end
assert(~isfolder(outputDir)&&~isfile(outputDir),'gpenmpc:ExistingOutput');
build=string(fileparts(fileparts(mfilename('fullpath'))));
runtime=string(gpenmpc_external_path('native_control_runtime_root'));
oldPath=path;handles={};
try
    addpath(fullfile(build,'tools'),fullfile(build,'matlab_validation'), ...
        fullfile(build,'m600_coptersim','matlab_validation'));
    dirs=[fullfile(gpenmpc_external_path('host_native_position_control_mex')), ...
        fullfile(gpenmpc_external_path('host_native_lower_loop_mex')), ...
        fullfile(gpenmpc_external_path('host_native_sensor_filter_mex'))];
    names=["gpenmpc_px4_native_position_control_mex","gpenmpc_px4_native_lower_loop_mex","gpenmpc_px4_sensor_filter_mex"];
    expected=["A2D4191A92EFB4DCB65BE98CA3C9B39B9629202D32C96AD54349B57069D78C88", ...
        "62B8C112B32909269BA6FB7F7C31D980B0D6759BD556DFF5A7AB7454D81BDAE5", ...
        "C4D4F05F491538967916A6769F62D92102E073645FE952B396829E56900BAD39"];
    bindings=struct('path',{},'bytes',{},'sha256',{});
    for j=1:3
        addpath(dirs(j),'-begin');binary=fullfile(dirs(j),names(j)+".mexw64");
        assert(strcmpi(which(names(j)),binary));id=fileIdentity(binary);
        assert(string(id.sha256)==expected(j),'gpenmpc:NumericalBinaryChanged');
        bindings(j)=id;handles{j}=str2func(names(j)); %#ok<AGROW>
    end
catch e
    cleanup(handles,oldPath);rethrow(e);
end
cleanupGuard=onCleanup(@()cleanup(handles,oldPath)); %#ok<NASGU>
pos=handles{1};low=handles{2};filter=handles{3};
m600check.loadFixture();
parameterFile=fullfile(gpenmpc_external_path('flat_terrain_model'),'M600_CORE_PARAMETERS.mat');
assert(strcmpi(m600check.fileSha256(parameterFile),'969D347CE041715CAED49BE0B7897486E34FA4E0B22EB82E50E9752B8F4F19E4'));
loaded=load(parameterFile,'parameters');p=loaded.parameters;
[pc,moduleCfg,positionMeta]=native_position_control_current_config();
[lc,lowerMeta]=native_lower_loop_current_config();
[fc,filterMeta]=native_sensor_filter_explicit_fixture(100);
rawPath=string(gpenmpc_external_path('native_position_readback_receipt'));
assert(strcmpi(m600check.fileSha256(rawPath),'BF31EEF276ADC9756371E5FEF282AC10465056B5710E3A87D64CD422F5F1E0B4'));
pr=jsondecode(fileread(rawPath));assert(pr.passed&&pr.COM_closed&&pr.parameter_writes==0);
velLP=observed(pr,'MPC_VEL_LP');velNF=observed(pr,'MPC_VEL_NF_FRQ');velD=observed(pr,'MPC_VELD_LP');
assert(velLP==0&&velNF==0&&velD==5,'gpenmpc:CurrentVelocityFilterChanged');
candidateFile=fullfile(gpenmpc_external_path('host_native_attitude_candidate'),'CANDIDATE.json');
assert(strcmpi(m600check.fileSha256(candidateFile),'A6C93CC99FF39A6B84CD132E9DB0A4471E60D125781AE7BFD7677A260CD29C19'));
candidate=jsondecode(fileread(candidateFile));assert(candidate.passed&&~candidate.SIL_outcomes_read&&~candidate.live_admission);
assert(isequaln(candidate.original_configuration,lc));
unmodified=candidate.candidate_configuration;unmodified.attitude_p=lc.attitude_p;assert(isequaln(unmodified,lc));
dt=.01;duration=20;n=round(duration/dt);payload=2.21;
mass=p.profile.mass_properties.base_mass_kg+payload+p.mission.plant_mismatch.mass_bias_kg;
assert(abs(mass-11.71)<1e-12);upper=p.calibration.rotor_allocation.per_rotor_thrust_upper_n;
hoverN=mass*9.80665/6;hover=hoverN/upper;
specs=struct('name',{},'candidate',{},'effective_vertical_i',{},'phase_interpretation',{});
for c=[false true]
    for useNominal=[false true]
        if c,tag="ANALYTIC_ATTITUDE_CANDIDATE";else,tag="ORIGINAL_ATTITUDE_GAINS";end
        if useNominal,phase="NOMINAL_I_COUNTERFACTUAL_NOT_HOVER_RUNTIME";gain=pc.velocity_i(3);
        else,phase="NO_ROUTE_HOLD_I0_HTE_UNAVAILABLE_FIXTURE";gain=0;end
        specs(end+1)=struct('name',tag+"__"+phase,'candidate',c, ...
            'effective_vertical_i',gain,'phase_interpretation',phase); %#ok<AGROW>
    end
end
fixture=struct('duration_s',duration,'plant_and_position_rate_hz',100,'sensor_rate_hz',100, ...
    'rates_are_HOST_fixture_not_observed_workqueue',true,'initial_height_m',10, ...
    'initial_x_offset_m',.02,'initial_roll_deg',.05,'payload_kg',payload,'mass_kg',mass, ...
    'initial_rotor_thrust_N',hoverN,'physical_equilibrium_collective',hover, ...
    'controller_hover_thrust',pc.hover_thrust,'HTE_updates_enabled',false, ...
    'ideal_state_feedback_only_in_HOST',true,'ground_initialization_or_takeoff_test',false, ...
    'position_velocity_D','Original AlphaFilter float arithmetic, finite difference of current velocity; LP and notch disabled by actual parameters.', ...
    'diagnostic_stop_attitude_deg',15,'diagnostic_stop_position_error_m',3, ...
    'stop_bounds_provenance','Bounded numerical diagnostic domain.', ...
    'all_cases_preserved',true);
mkdir(outputDir);results=struct([]);allData=cell(numel(specs),1);failure='';
try
    for j=1:numel(specs)
        s=specs(j);controlCfg=lc;if s.candidate,controlCfg=candidate.candidate_configuration;end
        controlCfg.initial_motor_commands=repmat(hover,6,1);
        assert(pos('init',pc)&&low('init',controlCfg));f=filter('init',fc);assert(f.initialized);
        x=zeros(19,1);x(1)=.02;x(3)=10;x(7:10)=[cosd(.025);-sind(.025);0;0];x(14:19)=hoverN;
        memory=m600check.initialStepMemory();memory.airborne_observed=true;memory.ground_confirmed=false;
        memory.ground_support_dwell_s=0;memory.previous_contact_active=false;
        controls=zeros(16,1);controls(1:6)=hover;jet=zeros(12,1);jet(3)=10;
        previousVelocity=zeros(3,1,'single');accelFiltered=zeros(3,1,'single');
        % Exact AlphaFilter<float> coefficient expression for fixed 100 Hz.
        sf=single(100);cutoff=single(velD);dtFloat=single(1)/sf;
        tau=single(1)/(single(2*pi)*cutoff);alpha=dtFloat/(tau+dtFloat);
        X=zeros(n+1,19);X(1,:)=x.';R=zeros(n,26);positionOutputs=cell(n,1);lowerOutputs=cell(n,1);
        sensorOutputs=cell(n,1);plantStatuses=cell(n,1);positionInputs=cell(n,1);
        rows=0;termination="FULL_PREDECLARED_DIAGNOSTIC_WINDOW";
        for k=1:n
            t=(k-1)*dt;rotor=controls([5;1;4;6;2;3])*upper;
            [dx,d,~]=m600check.derivativeSoftware(x,rotor,jet,payload,[0;0],t,p);
            obs=m600check.copterSimOutputs(x,d.actual_acceleration_mps2,[0;0],dx(11:13),mass);
            velocity=single(obs.velocity_ned_mps);
            derivative=(velocity-previousVelocity)/dtFloat;
            accelFiltered=accelFiltered+alpha*(derivative-accelFiltered);previousVelocity=velocity;
            ps=struct('timestamp_sample_s',t,'dt_s',dt,'state_position_ned',obs.position_ned_m, ...
                'state_velocity_ned',double(velocity),'state_acceleration_ned',double(accelFiltered),'state_yaw',obs.euler_rpy_rad(3), ...
                'trajectory_position_ned',[0;0;-10],'trajectory_velocity_ned',zeros(3,1), ...
                'trajectory_acceleration_ned',zeros(3,1),'trajectory_yaw',0,'trajectory_yawspeed',0, ...
                'vertical_i_gain',s.effective_vertical_i,'terminal_negative_integral_update_inhibit',false);
            po=pos('step',ps);assert(po.update_valid&&po.outputs_usable_for_host_numerical_chain&&~po.authority_granted);
            fo=filter('step',struct('raw_gyro',obs.body_rate_frd_rad_s,'timestamp_sample_us',uint64(1000000+round(t*1e6))));
            assert(~fo.must_stop&&fo.source_progressed);
            ls=struct('timestamp_sample_s',1+t,'initial_dt_s',dt,'q_est',obs.quaternion_wxyz_body_to_ned, ...
                'q_reference',po.q_d,'body_rate',fo.rate_body,'angular_acceleration',fo.derivative_body, ...
                'thrust_body_normalized',po.thrust_body,'yaw_rate_ff',po.yawspeed_setpoint,'battery_scale',1, ...
                'armed',true,'rotary_wing',true,'landed',false,'maybe_landed',false,'rates_enabled',true,'attitude_updated',true);
            lo=low('step',ls);controls=zeros(16,1);controls(1:6)=lo.motor_commands;
            [next,~,contact,nextMemory,status]=m600check.stepPx4Rk4(x,controls,jet,payload,[0;0],t,p,memory);
            plantStatuses{k}=status;positionInputs{k}=ps;
            after=m600check.copterSimOutputs(next,[0;0;0],[0;0],[0;0;0],mass);
            rows=k;X(k+1,:)=next.';positionOutputs{k}=po;lowerOutputs{k}=lo;sensorOutputs{k}=fo;
            error=obs.position_ned_m-[0;0;-10];
            R(k,:)=[t,obs.position_ned_m.',error.',rad2deg(obs.euler_rpy_rad).',po.vertical_integral_z_after, ...
                lo.motor_commands.',po.thrust_body(3),lo.motor_saturated,lo.rate_limit_reached,lo.integral_limit_reached, ...
                contact.contact_active,status.accepted,po.dt_clamped,lo.dt_clamped,fo.native_dt_clamped];
            if ~status.accepted,termination="PLANT_REJECT_"+status.failure_code;break;end
            if contact.contact_active,termination="CONTACT_ENTERED_FREE_FLIGHT_FIXTURE_STOP";break;end
            if max(abs(rad2deg(after.euler_rpy_rad(1:2))))>=15,termination="PREDECLARED_ATTITUDE_DIAGNOSTIC_DOMAIN_EXIT";break;end
            if norm(after.position_ned_m-[0;0;-10])>=3,termination="PREDECLARED_POSITION_DIAGNOSTIC_DOMAIN_EXIT";break;end
            x=next;memory=nextMemory;
        end
        pos('clear');low('clear');filter('clear');
        columns={'time_s','north_m','east_m','down_m','error_north_m','error_east_m','error_down_m', ...
            'roll_deg','pitch_deg','yaw_deg','vertical_integral_z','motor1','motor2','motor3','motor4','motor5','motor6', ...
            'body_thrust_z','motor_saturated','rate_limited','integral_limited','contact_active','plant_accepted', ...
            'position_dt_clamped','lower_dt_clamped','sensor_dt_clamped'};
        trace=array2table(R(1:rows,:),'VariableNames',columns);writetable(trace,fullfile(outputDir,s.name+".csv"));
        errorAll=[R(1:rows,5:7);next(1:3).'.*[1,1,-1]-[0,0,-10]];
        angles=[R(1:rows,8:10);rad2deg(after.euler_rpy_rad).'];
        r=struct('specification',s,'rows',rows,'termination',termination,'full_diagnostic_window',rows==n, ...
            'max_position_error_m',max(vecnorm(errorAll,2,2)),'max_attitude_deg',max(abs(angles),[],1), ...
            'final_position_error_ned_m',errorAll(end,:),'final_attitude_deg',angles(end,:), ...
            'max_abs_vertical_integral',max(abs(R(1:rows,11))),'motor_saturated_rows',nnz(R(1:rows,19)), ...
            'rate_limited_rows',nnz(R(1:rows,20)),'integral_limited_rows',nnz(R(1:rows,21)), ...
            'plant_accepted_all',all(R(1:rows,23)==1),'dt_clamped_rows',nnz(any(R(1:rows,24:26),2)), ...
            'all_retained_accepted_states_finite',all(isfinite(X(1:rows+1,:)),'all'), ...
            'rejected_candidate_endpoint_is_in_plant_status',true,'live_admission',false);
        if isempty(results),results=r;else,results(end+1)=r;end %#ok<AGROW>
        allData{j}=struct('trace',trace,'states_up',X(1:rows+1,:), ...
            'position_inputs',{positionInputs(1:rows)},'position_outputs',{positionOutputs(1:rows)}, ...
            'lower_outputs',{lowerOutputs(1:rows)},'sensor_outputs',{sensorOutputs(1:rows)}, ...
            'plant_statuses',{plantStatuses(1:rows)},'velocity_D_alpha_float',alpha);
        fprintf('HOST coupled %s: %s rows=%d max_error=%.6f max_roll/pitch=%.6f/%.6f\n',s.name,termination,rows,r.max_position_error_m,r.max_attitude_deg(1:2));
    end
catch problem
    failure=getReport(problem,'extended','hyperlinks','off');
end
report=struct('classification','HOST_POSITION_ATTITUDE_RATE_M600_COUPLING_DIAGNOSTIC', ...
    'execution_completed',isempty(failure),'cases_planned',numel(specs),'cases_completed',numel(results), ...
    'cases',results,'fixture',fixture,'position_configuration',pc,'module_parameters_not_automatically_executed',moduleCfg, ...
    'position_metadata',positionMeta,'lower_metadata',lowerMeta,'filter_metadata',filterMeta,'binaries',bindings, ...
    'candidate',fileIdentity(candidateFile),'source',fileIdentity([mfilename('fullpath') '.m']), ...
    'hardware_actions',0,'COM_UDP_actions',0,'live_admission',false,'failure',failure, ...
    'limitations',{{'Ideal plant state provides HOST feedback.', ...
    'Position and attitude kernel replay with explicit HOST fixtures.', ...
    'Hover thrust is fixed at the nominal value; HTE is unavailable in this fixture.', ...
    'I=nominal is a diagnostic counterfactual.', ...
    'Synchronous numerical source timing is 100 Hz.', ...
    'The attitude and position domain bounds stop the simulation; stopped runs retain their actual endpoint.'}});
save(fullfile(outputDir,'RAW_COUPLED_SIL.mat'),'report','allData','-v7.3');
f=fopen(fullfile(outputDir,'RESULT.json'),'w','n','UTF-8');assert(f>=0);fg=onCleanup(@()fclose(f)); %#ok<NASGU>
fprintf(f,'%s\n',jsonencode(report,PrettyPrint=true));assert(isempty(failure),'gpenmpc:CoupledSilExecutionFailed','%s',failure);
end
function v=observed(r,name)
rows=r.additional_validation.rows;k=find(strcmp({rows.name},name));assert(isscalar(k)&&rows(k).passed);v=rows(k).observation.typed_value.decoded;
end
function cleanup(handles,capturedPath)
for j=1:numel(handles),try,handles{j}('clear');catch,end,end
path(capturedPath);
end
function id=fileIdentity(file)
f=fopen(file,'rb');assert(f>=0);g=onCleanup(@()fclose(f)); %#ok<NASGU>
b=fread(f,Inf,'*uint8');h=java.security.MessageDigest.getInstance('SHA-256');h.update(b);
id=struct('path',char(file),'bytes',numel(b),'sha256',upper(reshape(dec2hex(typecast(h.digest(),'uint8'),2).',1,[])));
end
