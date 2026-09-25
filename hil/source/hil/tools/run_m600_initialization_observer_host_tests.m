function result=run_m600_initialization_observer_host_tests(outputDir)
deviceFixture=gpenmpc_test_device_config(); %#ok<NASGU>
% Test the initialization observer with in-memory transports.
assert(~isfolder(outputDir)&&~isfile(outputDir),'m600check:HostTestOutputExists','Use a new output path.');
mkdir(outputDir);cfg=make_m600_native_hover_config('MOCK_M600','MOCK_OUTER');
buildRoot=fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(buildRoot,'matlab_validation'),fullfile(buildRoot,'m600_coptersim','matlab_validation'));
cfg.write_artifacts=false;cfg.expected_uid='1234605616436508552';
cfg.expected_board_version=56;cfg.expected_flight_custom_version_hex='1234567890ABCDEF';
testNames={};testPass=[];caseResults={};mode='';t=0;calls=[];mockSnapshots={};snapCount=0;diagnosticState=[];
r=runCase('normal');
check('normal_full_45_s',r.window_completed&&r.elapsed_observation_s==45);
check('normal_not_flight_or_initialization_pass',~r.flight_result&&~r.initialization_pass_claim&&r.formal_rows==0);
check('normal_records_all_snapshots',numel(r.snapshots)==snapCount&&numel(r.rows)==snapCount&&snapCount>=899);
check('heartbeat_1_hz_not_control',r.counts.heartbeat_sent>=44&&r.counts.heartbeat_sent<=45&&calls.heartbeat==r.counts.heartbeat_sent);
check('normal_close_exactly_once',calls.close==1&&r.matlab_sockets_closed);
check('normal_raw_evidence_preserved',numel(r.transport_evidence.raw_snapshots)==snapCount);
check('normal_no_fatal',isempty(r.first_fatal)&&isempty(r.failure));
check('normal_expected_identity',all([r.rows.identity_matches_expected]));
check('normal_zero_forbidden_calls',calls.authority==0&&allZeroAuthority(r));
r=runCase('source_reset');
check('source_reset_observed_without_clearing',r.window_completed&&~isempty(r.first_fatal)&&~r.fault_latch_cleared&&~r.source_reset_performed);
check('first_source_fault_immutable',strcmp(r.first_fatal.reason,'TRUTH_TIME_REVERSED')&&r.first_fatal.time_s>=20&&r.first_fatal.time_s<20.1);
check('exact_adapter_first_fatal_preserved',isstruct(r.first_fatal.adapter_first_fatal)&& ...
    strcmp(r.first_fatal.adapter_first_fatal.reason,'TRUTH_TIME_REVERSED')&&r.first_fatal.adapter_first_fatal.time_s==20);
check('source_reset_columns_separate',any([r.rows.truth_source_reversed])&&any([r.rows.raw_model_source_reversed])&&~any([r.rows.estimate_source_reversed]));
check('accepted_model_time_not_mislabelled_as_raw',~any([r.rows.model_source_reversed])&&any([r.rows.raw_model_source_reversed]));
check('later_model_fault_not_relabel_first',any(strcmp({r.events.domain},'MODEL_FAULT'))&&strcmp(r.first_fatal.domain,'ADAPTER_FATAL'));
check('source_reset_remains_diagnostic_failure',strcmp(r.status,'OBSERVATION_ENDED_WITH_RECORDED_FAULTS'));
check('source_reset_does_not_construct_again',r.counts.adapter_creations==0&&calls.close==1&&calls.authority==0);
r=runCase('px4_reset');
check('px4_reset_source_independent',any([r.rows.estimate_source_reversed])&&~any([r.rows.truth_source_reversed]));
check('px4_reset_retained_without_adapter_reset',~r.source_reset_performed&&strcmp(r.first_fatal.domain,'PX4_SOURCE_REVERSED'));
r=runCase('px4_attitude_reset');
check('attitude_reset_without_local_position_detected',all(isnan([r.rows.estimate_source_s]))&&any([r.rows.attitude_source_reversed]));
check('attitude_reset_distinct_domain',strcmp(r.first_fatal.domain,'PX4_ATTITUDE_SOURCE_REVERSED'));
r=runCase('heartbeat_stale');
check('heartbeat_after_fresh_stops_at_3s_age',strcmp(r.stop_reason,'HEARTBEAT_STALE_RETURN_TO_OUTER')&&r.rows(end).heartbeat_age_s>3&&r.rows(end).heartbeat_age_s<3.051);
check('heartbeat_stale_does_not_wait_45s',~r.window_completed&&r.elapsed_observation_s<5.1);
check('heartbeat_stale_closes_returns_outer',r.matlab_sockets_closed&&r.outer_final_safety_still_required&&calls.close==1);
r=runCase('initial_heartbeat_missing');
check('initial_missing_waits_declared_deadline',strcmp(r.stop_reason,'INITIAL_HEARTBEAT_ACQUISITION_TIMEOUT')&&r.elapsed_observation_s>=30&&r.elapsed_observation_s<30.051);
check('initial_missing_never_claims_disarmed',~r.ever_fresh_heartbeat&&~r.initialization_pass_claim);
r=runCase('snapshot_exception');
check('snapshot_exception_captured',contains(r.failure,'mock:SnapshotFailure'));
check('snapshot_exception_keeps_prior_raw',numel(r.snapshots)>0&&numel(r.transport_evidence.raw_snapshots)==numel(r.snapshots));
check('snapshot_exception_finally_close',calls.close==1&&r.matlab_sockets_closed&&r.outer_final_safety_still_required);
r=runCase('heartbeat_exception');
check('heartbeat_attempt_success_distinct',r.counts.heartbeat_attempts==r.counts.heartbeat_sent+1&&contains(r.failure,'mock:HeartbeatFailure'));
check('heartbeat_exception_finally_close',calls.close==1&&r.matlab_sockets_closed);
r=runCase('close_false');
check('close_false_not_safe',~r.matlab_sockets_closed&&~isempty(r.close_failure)&&strcmp(r.status,'OBSERVATION_STOPPED__OUTER_SAFETY_REQUIRED'));
check('close_false_still_exact_attempt',r.close_attempted&&calls.close==1);
r=runCase('close_exception');
check('close_exception_retained',contains(r.close_failure,'mock:CloseFailure')&&~r.matlab_sockets_closed&&calls.close==1);
r=runCase('evidence_exception');
check('evidence_exception_does_not_block_close',contains(r.evidence_capture_failure,'mock:EvidenceFailure')&&calls.close==1&&r.matlab_sockets_closed);
check('evidence_exception_keeps_local_snapshots',numel(r.snapshots)>800&&strcmp(r.first_fatal.domain,'EVIDENCE_CAPTURE'));
r=runCase('identity_change');
check('identity_change_recorded_and_stops',strcmp(r.stop_reason,'IDENTITY_MISMATCH_RETURN_TO_OUTER')&&r.rows(end).identity_changed&&r.elapsed_observation_s<6);
check('identity_change_no_authority',calls.authority==0&&allZeroAuthority(r));
r=runCase('identity_absent');
check('identity_absent_not_inferred',~any([r.rows.identity_present])&&isempty(r.initial_observed_identity)&&~r.initialization_pass_claim);
r=runCase('unexpected_arm');
check('unexpected_arm_returns_outer_no_disarm_write',strcmp(r.stop_reason,'UNEXPECTED_ARM_STATE_RETURN_TO_OUTER')&&r.rows(end).armed==1&&calls.authority==0);
r=runCase('model_fatal_only');
check('model_fatal_distinct',strcmp(r.first_fatal.domain,'MODEL_FAULT')&&~any([r.rows.truth_source_reversed]));
check('all_cases_zero_authority',all(cellfun(@allZeroAuthority,caseResults)));
check('all_cases_close_attempted_once',all(cellfun(@(x)x.close_attempted,caseResults)));

blocked=false;try,observe_m600_copter_initialization('',cfg);catch e,blocked=strcmp(e.identifier,'m600check:ObserverNoLivePreflight');end
check('default_live_without_preflight_rejected_before_adapter',blocked);
cfg2=cfg;cfg2.heartbeat_max_age_s=4;blocked=false;
try,observe_m600_copter_initialization('',cfg2,makeIo());catch e,blocked=strcmp(e.identifier,'m600check:ObserverHeartbeatContract');end
check('caller_cannot_weaken_existing_heartbeat_bound',blocked);
cfg2=cfg;cfg2.write_artifacts=true;blocked=false;
try,observe_m600_copter_initialization(outputDir,cfg2,makeIo());catch e,blocked=strcmp(e.identifier,'m600check:ObserverOutputExists');end
check('existing_output_not_overwritten',blocked);

% Exercise genuine MAT/CSV/JSON serialization of the same loop with uint64
% AUTOPILOT_VERSION and byte arrays still in the in-memory mock evidence.
cfg.write_artifacts=true;mode='normal';resetMock();
artifactResult=observe_m600_copter_initialization(fullfile(outputDir,'SERIALIZATION_CASE'),cfg,makeIo());
compact=jsondecode(fileread(fullfile(outputDir,'SERIALIZATION_CASE','RESULT.json')));
saved=load(fullfile(outputDir,'SERIALIZATION_CASE','RAW_INITIALIZATION_OBSERVATION.mat'),'result');
check('actual_json_result_created',compact.window_completed&&isempty(compact.snapshots));
check('actual_mat_preserves_uint64',isa(saved.result.snapshots{1}.autopilot_version.uid,'uint64')&& ...
    saved.result.snapshots{1}.autopilot_version.uid==uint64(3473490377)*uint64(1000000000)+uint64(90611257));
check('actual_mat_preserves_bytes',isa(saved.result.transport_evidence.bytes,'uint8')&&isequal(saved.result.transport_evidence.bytes,uint8([0,255,128])));
check('actual_csv_has_all_rows',height(readtable(fullfile(outputDir,'SERIALIZATION_CASE','OBSERVATION_ROWS.csv')))==numel(artifactResult.rows));
result=struct('status','HOST_ONLY_OBSERVATION_RUNNER_TESTS','passed',all(testPass), ...
    'checks_total',numel(testPass),'checks_passed',sum(testPass), ...
    'checks',struct('name',testNames,'passed',num2cell(testPass)), ...
    'cases',numel(caseResults)+1,'COM_open',0,'UDP_open',0,'board_actions',0, ...
    'simulator_started',0,'plant_creations',0,'flight_claim',false);
save(fullfile(outputDir,'RAW_HOST_CASES.mat'),'caseResults','result','-v7.3');
fid=fopen(fullfile(outputDir,'RESULT.json'),'w','n','UTF-8');assert(fid>=0);
f=onCleanup(@()fclose(fid));fprintf(fid,'%s\n',jsonencode(result,PrettyPrint=true));clear f
disp(jsonencode(result));assert(result.passed,'m600check:ObserverHostTestsFailed','Observer HOST tests failed.');

    function check(n,v),testNames{end+1}=n;testPass(end+1)=logical(v);end
    function r=runCase(name)
        mode=name;resetMock();r=observe_m600_copter_initialization('',cfg,makeIo());
        caseResults{end+1}=r; %#ok<AGROW>
    end
    function resetMock()
        t=0;snapCount=0;mockSnapshots={};diagnosticState=[];calls=struct('close',0,'heartbeat',0,'authority',0,'evidence',0);
    end
    function io=makeIo()
        io=struct('now',@nowMock,'sleep',@advance,'snapshot',@snapshot,'sendHeartbeat',@heartbeat, ...
            'evidence',@evidence,'close',@closeIo,'readParameter',@authority, ...
            'setIntegerParameter',@authority,'sendSetpoint',@authority,'requestCommand',@authority);
    end
    function value=nowMock(),value=t;end
    function advance(dt),t=t+dt;end
    function heartbeat()
        if strcmp(mode,'heartbeat_exception')&&t>=3,error('mock:HeartbeatFailure','Injected send exception.');end
        calls.heartbeat=calls.heartbeat+1;
    end
    function s=snapshot()
        if strcmp(mode,'snapshot_exception')&&t>=5,error('mock:SnapshotFailure','Injected receive exception.');end
        snapCount=snapCount+1;custom=uint8(sscanf(cfg.expected_flight_custom_version_hex,'%2x').');
        uid=uint64(3473490377)*uint64(1000000000)+uint64(90611257);
        s=struct('now_s',t,'armed',0,'landed_state',1,'heartbeat_rx_s',t,'extended_rx_s',t, ...
            'clock_valid',true,'clock_uncertainty_s',.001,'clock_utc_drift_s',.001, ...
            'model_ready',true,'fatal','','first_fatal',[],'attitude',struct('time_boot_ms',uint32(round((100+t)*1000))), ...
            'estimate',struct('raw_source_time_s',100+t),'truth',struct('raw_source_time_s',t), ...
            'autopilot_version',struct('uid',uid,'board_version',uint32(56),'flight_custom_version',custom));
        modelTime=t;failed=0;failureCode=0;
        switch mode
            case 'source_reset'
                if t>=20
                    s.truth.raw_source_time_s=t-20;modelTime=t-20;s.clock_valid=false;
                    if t<21,s.fatal='TRUTH_TIME_REVERSED';s.first_fatal=struct('time_s',20,'reason','TRUTH_TIME_REVERSED');end
                    if t>=25,failed=1;failureCode=2;end
                end
            case 'px4_reset',if t>=20,s.estimate.raw_source_time_s=t-20;end
            case 'px4_attitude_reset'
                s.estimate=[];if t>=20,s.attitude.time_boot_ms=uint32(round((t-20)*1000));end
            case 'heartbeat_stale',if t>=2,s.heartbeat_rx_s=2;end
            case 'initial_heartbeat_missing',s.heartbeat_rx_s=-Inf;s.armed=NaN;
            case 'identity_change',if t>=5,s.autopilot_version.uid=uid+1;end
            case 'identity_absent',s.autopilot_version=[];
            case 'unexpected_arm',if t>=5,s.armed=1;end
            case 'model_fatal_only'
                if t>=5,failed=1;failureCode=2;end
        end
        % Test production decoding and latching with ii32d packets.
        header=typecast(int32([1234567890,1]),'uint8');
        payload=typecast(double([failed,failureCode,modelTime,114.8358715,1,0,1,zeros(1,25)]),'uint8');
        diagnosticState=m600check.updateCopterSimDiagnostic(diagnosticState,[header,payload],t,1);
        s.model_diagnostic=m600check.copterSimDiagnosticSnapshot(diagnosticState,t,cfg.state_max_age_s);
        s.model_ready=s.model_diagnostic.model_ready;
        mockSnapshots{end+1}=s; %#ok<AGROW>
    end
    function r=evidence()
        calls.evidence=calls.evidence+1;
        if strcmp(mode,'evidence_exception'),error('mock:EvidenceFailure','Injected evidence exception.');end
        r=struct('raw_snapshots',{mockSnapshots},'bytes',uint8([0,255,128]));
    end
    function yes=closeIo()
        calls.close=calls.close+1;
        if strcmp(mode,'close_exception'),error('mock:CloseFailure','Injected close exception.');end
        yes=~strcmp(mode,'close_false');
    end
    function varargout=authority(varargin) %#ok<STOUT,INUSD>
        calls.authority=calls.authority+1;error('mock:ForbiddenAuthority','Observer invoked an authority method.');
    end
end
function yes=allZeroAuthority(r)
names={'parameter_read_requests','parameter_writes','mapping_writes','mode_requests','arm_requests', ...
    'disarm_requests','land_requests','task_requests','setpoint_requests','reboot_requests', ...
    'COM_open','plant_creations','physical_output_actions'};
yes=true;for k=1:numel(names),yes=yes&&r.counts.(names{k})==0;end
end
