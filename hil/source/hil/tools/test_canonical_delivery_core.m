function result=test_canonical_delivery_core(outputDir)
% Test the RK4/flat-terrain core offline, one fixture instance at a time.
% Explicit resets separate the named fixtures.
root=fileparts(fileparts(mfilename('fullpath')));addpath(fullfile(root,'matlab_validation'));
assert(~isfolder(outputDir)&&~isfile(outputDir),'m600check:DeliveryOutput','Fresh output directory required.');
mkdir(outputDir);m600check.loadFixture();
parameterPath=fullfile(gpenmpc_external_path('diagnostic_terrain_model'),'M600_CORE_PARAMETERS.mat');
loaded=load(parameterPath,'parameters','environment');p=loaded.parameters;env=loaded.environment;
policy=struct('initial_payload_kg',env.payload_kg,'initial_wind_ned_xy_mps',env.wind_xy_mps, ...
    'initial_reference_jet_ned',env.reference_jet_ned,'expected_session_token',1234567, ...
    'max_env_age_s',.5,'max_board_age_s',.25,'max_future_skew_s',.001, ...
    'max_ground_sample_gap_s',.025,'unload_dwell_s',8.0,'max_commit_delay_s',.02, ...
    'base_mass_kg',p.profile.mass_properties.base_mass_kg,'mass_bias_kg',p.mission.plant_mismatch.mass_bias_kg, ...
    'mass_tolerance_kg',0,'service_payload_targets_kg',env.payload_kg*[.75;.5;.25;0]);
u=zeros(16,1);terrain=zeros(15,1);pos=zeros(3,1);angles=zeros(3,1);
checks=struct('name',{},'pass',{});firstError=struct('identifier','','message','');
rows=zeros(7000,12);rowCount=0;wrapperCalls=0;referenceCalls=0;explicitResets=0;
sourceNames={'m600check.copterSimDeliveryCore','m600check.copterSimFlatTerrainCore','m600check.copterSimIoCore','m600check.derivativeSoftware', ...
    'gpenmpcTaskIo.initPlantEnvironmentV2','gpenmpcTaskIo.acceptPlantEnvironmentV2','gpenmpcTaskIo.commitPlantEnvironmentV2'};
sources=struct('name',{},'path',{},'sha256',{});
for k=1:numel(sourceNames),path=which(sourceNames{k});sources(end+1)=struct('name',sourceNames{k},'path',path,'sha256',gpenmpcNative.fileSha256(path));end %#ok<AGROW>
try
    % Evolve the plant for 50 s before BEGIN; zero ENV must not start the timeout.
    [y,d,a,r]=call(true,zeros(28,1),0,pos,p,policy);
    check('physical_reset_t0_unbound_observation',d.reset_applied&&d.sim_time_s==0&&a.session_token==0&&~a.valid&&r.environment_status_code==16&&~r.task_may_continue);
    check('initial_actual_mass_matches_core_rawbits',typecast(y.mass_kg,'uint64')==typecast((policy.base_mass_kg+env.payload_kg)+policy.mass_bias_kg,'uint64'));
    for k=1:5000
        [y,d,a,r]=call(false,zeros(28,1),k*.01,pos,p,policy);
        assert(~d.failed&&d.step_accepted&&r.environment_status_code==16&&~r.task_env_failed,'m600check:DeliveryFixture','Pre-BEGIN actual plant became invalid.');
        if mod(k,1000)==0,fprintf('PRE_BEGIN actual plant %.3f s; rows=%d\n',d.sim_time_s,rowCount);end
    end
    beforeBegin=d.sim_time_s;
    check('50s_pre_BEGIN_no_environment_deadline_or_permission',abs(d.sim_time_s-50)<1e-8&&~r.begin_attempted&&~r.session_begun&&~a.valid&&a.session_token==0);
    check('pre_BEGIN_one_core_step_per_call',d.plant_step_count==uint64(5000)&&r.core_calls_this_invocation==1);
    begin=frame(1,50.01,env.payload_kg,0);begin(4)=0;
    [~,d,a,r]=call(false,begin,50.01,pos,p,policy);
    check('late_BEGIN_commits_without_reset',r.session_begun&&r.begin_count==1&&a.new_credit&&a.valid&&~d.reset_applied&&abs(d.sim_time_s-beforeBegin-.01)<1e-9);
    check('late_BEGIN_retains_monotonic_nonzero_core_time',a.applied_core_time_s>50&&d.plant_step_count==uint64(5001));
    for k=1:800
        t=50.01+k*.01;f=frame(k+1,t,env.payload_kg,0);
        [~,d,a,r]=call(false,f,t,pos,p,policy);
        assert(~d.failed&&a.new_credit&&~r.task_env_failed,'m600check:DeliveryFixture','Actual eight-second ground proof failed.');
    end
    check('actual_same_plant_ground_8s_ready',d.ground_confirmed&&r.continuous_ground_dwell_s>=8&&a.unload_count==0);
    beforeUnloadTime=d.sim_time_s;release=frame(802,58.02,policy.service_payload_targets_kg(1),1);release(28)=1;
    [y,d,a,r]=call(false,release,58.02,pos,p,policy);
    check('actual_mass_change_commits_after_8s',a.new_credit&&a.valid&&a.applied_payload_generation==1&&a.unload_count==1&&a.actual_payload_kg==policy.service_payload_targets_kg(1));
    check('ACK_mass_is_actual_y_mass',typecast(a.actual_total_mass_kg,'uint64')==typecast(y.mass_kg,'uint64'));
    check('unload_does_not_reset_same_plant_clock',~d.reset_applied&&abs(d.sim_time_s-beforeUnloadTime-.01)<1e-9&&r.core_calls_this_invocation==1);
    [~,d,a,r]=call(false,release,58.03,pos,p,policy);
    check('same_release_is_idempotent_with_physics_advancing',~a.new_credit&&a.unload_count==1&&~r.pending&&d.step_accepted);
    acceptedMass=y.mass_kg;
    bad=frame(803,58.04,policy.service_payload_targets_kg(2),2);bad(28)=2;
    [y,d,a,r]=call(false,bad,58.04,pos,p,policy);
    check('second_unload_without_new_8s_denied',r.task_env_failed&&r.failure_code==10&&a.unload_count==1&&y.mass_kg==acceptedMass&&d.step_accepted);
    failedTime=d.sim_time_s;
    [y,d,a,r]=call(false,begin,58.05,pos,p,policy);
    check('failed_environment_cannot_reBEGIN_or_freeze_HIL',r.task_env_failed&&r.failure_code==10&&r.begin_count==1&&~a.valid&&y.mass_kg==acceptedMass&&d.sim_time_s>failedTime&&d.step_accepted);
    % BEGIN during physical reset must not count as a core step.
    f=frame(1,0,env.payload_kg,0);f(4)=0;
    [~,d,a,r]=call(true,f,0,pos,p,policy);
    check('BEGIN_on_reset_no_commit_credit',d.reset_applied&&~d.step_accepted&&~a.new_credit&&~a.valid&&r.pending);
    [~,d,a,r]=call(false,f,.01,pos,p,policy);
    check('first_real_step_after_reset_commits_pending_once',d.step_accepted&&a.new_credit&&a.valid&&~r.pending&&d.sim_time_s==.01);
    % Invalid first BEGIN variants: every subsequent proper BEGIN remains
    % rejected while the physical core keeps taking steps with initial env.
    names={'generation_not_one','payload_generation_not_zero','not_paused','task_time_nonzero', ...
        'armed','PX4_not_ground','release_nonzero','wrong_session','wrong_schema','nonfinite','short_shape','initial_payload_changed'};
    for testIndex=1:numel(names)
        [~,~,~,~]=call(true,zeros(28,1),0,pos,p,policy);bad=frame(1,.01,env.payload_kg,0);bad(4)=0;expected=5;
        switch testIndex
            case 1,bad(2)=2;
            case 2,bad(21)=1;
            case 3,bad(22)=0;
            case 4,bad(4)=.01;
            case 5,bad(25)=31;
            case 6,bad(26)=2;
            case 7,bad(28)=1;
            case 8,bad(23)=1234568;expected=12;
            case 9,bad(1)=1;
            case 10,bad(6)=NaN;expected=4;
            case 11,bad=bad(1:27);expected=4;
            case 12,bad(5)=env.payload_kg-.1;
        end
        [y,d,a,r]=call(false,bad,.01,pos,p,policy);
        check(['invalid_first_BEGIN_' names{testIndex}],r.task_env_failed&&r.failure_code==expected&&r.begin_count==1&&~a.valid&&d.step_accepted);
        firstMass=y.mass_kg;firstTime=d.sim_time_s;good=frame(1,.02,env.payload_kg,0);good(4)=0;
        [y,d,a,r]=call(false,good,.02,pos,p,policy);
        check(['no_reBEGIN_wash_' names{testIndex}],r.task_env_failed&&r.failure_code==expected&&r.begin_count==1&&~a.new_credit&&y.mass_kg==firstMass&&d.sim_time_s>firstTime);
    end
    % Actual core mass readback is independent of caller-proposed policy.
    wrongMassPolicy=policy;wrongMassPolicy.mass_bias_kg=policy.mass_bias_kg+.001;
    [~,~,~,~]=call(true,zeros(28,1),0,pos,p,wrongMassPolicy);
    good=frame(1,.01,env.payload_kg,0);good(4)=0;
    [y,d,a,r]=call(false,good,.01,pos,p,wrongMassPolicy);
    check('actual_core_mass_mismatch_prevents_ACK',r.task_env_failed&&r.failure_code==15&&~a.valid&&~a.new_credit&&d.step_accepted);
    actualMass=y.mass_kg;clockAtFailure=d.sim_time_s;
    [y,d,a,r]=call(false,good,.02,pos,p,wrongMassPolicy);
    check('wrong_mass_failure_keeps_initial_env_and_advancing_core',r.failure_code==15&&y.mass_kg==actualMass&&d.sim_time_s>clockAtFailure&&~a.new_credit);
    % An environment clock rollback must not reverse the physical clock.
    [~,~,~,~]=call(true,zeros(28,1),0,pos,p,policy);
    [~,~,~,~]=call(false,zeros(28,1),.02,pos,p,policy);
    [~,d,a,r]=call(false,zeros(28,1),.01,pos,p,policy);
    check('I_O_clock_reversal_latches_but_core_advances',r.failure_code==1&&r.task_env_failed&&d.sim_time_s==.02&&~d.reset_applied&&~a.valid);
    % After expiry, a new BEGIN must not clear the recorded gap.
    f=frame(1,0,env.payload_kg,0);f(4)=0;
    [~,~,~,~]=call(true,f,0,pos,p,policy);[~,~,~,~]=call(false,f,.01,pos,p,policy);
    [~,d,a,r]=call(false,zeros(28,1),.51,pos,p,policy);
    check('environment_expiry_not_plant_failure',r.failure_code==3&&r.task_env_failed&&~d.failed&&d.step_accepted&&~a.valid);
    expiredTime=d.sim_time_s;f=frame(1,.52,env.payload_kg,0);f(4)=0;
    [~,d,a,r]=call(false,f,.52,pos,p,policy);
    check('post_expiry_BEGIN_does_not_reset_or_refresh',r.failure_code==3&&r.begin_count==1&&d.sim_time_s>expiredTime&&~a.valid);
    % Source equivalence in free flight BEFORE session BEGIN. Compare every
    % actual y/d field against a separate explicit-reset reference sequence.
    flightPos=[1;2;-10];reference=cell(21,1);
    for k=0:20
        [yy,dd]=m600check.copterSimFlatTerrainCore(u,k==0,flightPos,angles,env,terrain,p);referenceCalls=referenceCalls+1;
        reference{k+1}=struct('y',yy,'d',dd);
    end
    exact=true;
    for k=0:20
        [yy,dd,aa,rr]=call(k==0,zeros(28,1),k*.01,flightPos,p,policy);
        exact=exact&&isequaln(yy,reference{k+1}.y)&&isequaln(dd,reference{k+1}.d)&&~aa.valid&&rr.environment_status_code==16;
    end
    check('pre_session_free_flight_all_original_outputs_diagnostics_exact',exact);
    check('free_flight_does_not_claim_ground_or_task_authority',~dd.ground_confirmed&&~rr.task_may_continue&&aa.session_token==0);
    sourceStable=true;for k=1:numel(sources),sourceStable=sourceStable&&strcmp(gpenmpcNative.fileSha256(sources(k).path),sources(k).sha256);end
    check('bound_sources_not_changed_during_actual_test',sourceStable);
catch err
    firstError=struct('identifier',err.identifier,'message',getReport(err,'extended','hyperlinks','off'));
end
columns={'wrapper_call','explicit_reset','io_time_s','core_time_s','actual_mass_kg','env_status','applied_frame_gen','applied_payload_gen','new_ACK','ground','core_failed','task_allowed'};
writetable(array2table(rows(1:rowCount,:),'VariableNames',columns),fullfile(outputDir,'CORE_ROWS.csv'));
result=struct('status','HOST_ONLY_SAME_CANONICAL_RK4_DELIVERY_WRAPPER','pass',isempty(firstError.identifier)&&all([checks.pass]), ...
    'checks_total',numel(checks),'checks_passed',sum([checks.pass]),'checks',checks,'first_error',firstError, ...
    'wrapper_calls',wrapperCalls,'reference_core_calls',referenceCalls,'fixture_explicit_new_instance_resets',explicitResets, ...
    'parameter_path',parameterPath,'parameter_sha256',gpenmpcNative.fileSha256(parameterPath),'sources',sources, ...
    'fixture_policy_not_live_task_parameters',policy,'nominal_50s_pre_BEGIN_test',true, ...
    'hardware_actions',0,'socket_open',0,'COM_open',0,'models_built',0,'DLLs_built',0, ...
    'code_generation_executed',false, ...
    'limitations','MATLAB plant arithmetic with synthetic independent board fields.');
fid=fopen(fullfile(outputDir,'RESULT.json'),'w','n','UTF-8');assert(fid>=0);closer=onCleanup(@()fclose(fid));fprintf(fid,'%s\n',jsonencode(result,PrettyPrint=true));clear closer;
disp(jsonencode(struct('pass',result.pass,'checks_total',numel(checks),'checks_passed',sum([checks.pass]),'wrapper_calls',wrapperCalls,'first_error',firstError)));
assert(result.pass,'m600check:DeliveryTests','See preserved RESULT.json and CORE_ROWS.csv.');
    function check(name,ok)
        checks(end+1)=struct('name',name,'pass',islogical(ok)&&isscalar(ok)&&ok);
        assert(checks(end).pass,'m600check:DeliveryCheck','Failed: %s',name);
    end
    function f=frame(g,t,payload,pg)
        value=struct('schema_version',2,'generation',g,'source_io_time_s',t,'task_reference_time_s',t, ...
            'payload_kg',payload,'wind_ned_xy_mps',env.wind_xy_mps,'mission_phase',2,'reference_jet_ned',env.reference_jet_ned, ...
            'payload_generation',pg,'task_clock_paused',true,'session_token',policy.expected_session_token, ...
            'board_min_rx_io_time_s',t,'board_valid_flags',15,'landed_state',1,'continuity_epoch',1,'service_release_generation',0);
        [~,f]=gpenmpcTaskIo.encodePlantEnvironmentV2(value,1,env.payload_kg);
    end
    function [yy,dd,aa,rr]=call(doReset,f,t,position,parameters,pp)
        [yy,dd,aa,rr]=m600check.copterSimDeliveryCore(u,doReset,position,angles,terrain,f,t,env,parameters,pp);
        wrapperCalls=wrapperCalls+1;explicitResets=explicitResets+double(doReset);rowCount=rowCount+1;
        assert(rowCount<=size(rows,1),'m600check:DeliveryRows','Bounded trace buffer exhausted.');
        rows(rowCount,:)=[wrapperCalls,double(doReset),t,dd.sim_time_s,yy.mass_kg,double(rr.environment_status_code), ...
            aa.applied_frame_generation,aa.applied_payload_generation,double(aa.new_credit),double(dd.ground_confirmed),double(dd.failed),double(rr.task_may_continue)];
    end
end
