function result = run_m600_matlab_hil(outputDir,cfg,io)
% RUN_M600_MATLAB_HIL Run native PX4 hover with the CopterSim M600 plant.
% The outer owner manages identity, firmware, output isolation, COM and simulator lifetime.
% This runner owns UDP, temporary virtual mappings and normal LAND/disarm.
% Restore allocator and tuning contracts independently and verify baseline values.
% Missing or stale heartbeat leaves disarm status unknown; delegate unresolved recovery.
% Timing bounds come from cfg. Inject IO and disable artifact writing for host tests.

validateConfig(cfg);
geometryCheck=[];geometryApply=[];geometryRestore=[];geometryTouched=false;
geometryRestored=true;
if isfield(cfg,'temporary_allocator_geometry')
    geometryCheck=m600check.validateTemporaryAllocatorGeometry(cfg.temporary_allocator_geometry);
    assert(geometryCheck.passed,'m600check:AllocatorGeometryContract','%s',geometryCheck.failure);
end
tuningCheck=[];tuningApply=[];tuningRestore=[];tuningTouched=false;tuningRestored=true;
parameterUnionOriginal=[];parameterUnionApplied=[];parameterUnionRestored=[];parameterUnionSafe=true;
if isfield(cfg,'native_hover_tuning')
    tuningCheck=m600check.validateNativeHoverTuningContract(cfg.native_hover_tuning);
    assert(tuningCheck.passed,'m600check:NativeHoverTuningContract','%s',tuningCheck.failure);
    assert(~isempty(geometryCheck),'m600check:NativeHoverTuningGeometryRequired', ...
        'The three-field tuning is allowed only with the separately verified twelve-field geometry contract.');
    parameterUnionSafe=false;
end
hoverPreparation=[];
if isfield(cfg,'virtual_sensor_mount_contract')&&strcmp(cfg.virtual_sensor_mount_contract.schema, ...
        'VIRTUAL_SENSOR_MOUNT_CONTRACT_V2_AUTOCAL_OBSERVED')
    assert(isfield(cfg,'native_hover_preparation'),'m600check:ObservationContractOnly', ...
        'Hover preparation requires a separately bound observation receipt.');
    hoverPreparation=m600check.verifySensorAlignedHoverPreparation( ...
        cfg.native_hover_preparation,cfg.virtual_sensor_mount_contract);
    assert(hoverPreparation.passed,'m600check:HoverPreparationRejected','%s',hoverPreparation.failure);
end
if cfg.write_artifacts
    assert(~isfolder(outputDir),'m600check:OutputExists','Refusing to reuse an output directory.');
end
if nargin<3
    assert(cfg.live_enabled&&cfg.outer_preflight_pass, 'm600check:NoLivePreflight', ...
        'An explicit live configuration and outer identity/safety preflight are required.');
    io=m600check.makeM600CopterSimIo(cfg);
end
clockId='MATLAB_MONOTONIC_MAPPED_SOURCE_TIME';
events=repmat(struct('time_s',0,'kind','','detail',''),0,1);
parameterActions=repmat(struct('name','','target',0,'purpose','','attempted',false, ...
    'write_returned',false,'readback_verified',false,'raw_bits_hex','','error',''),0,1);
mapped=strings(0,1);log=[];phase='PRE_ARM';formalStart=NaN;formalEnd=NaN;
latest=[];finalized=false;socketsClosed=false;safeFinal=false;hardwareFinal=false;failure='';
firstModelFailure=[];
firstFailedSnapshot=[];firstFailedStateEvidence=[];
sensorMountCheck=[];
normalLandRequested=false;disarmRequested=false;armRequested=false;
landDeadline=NaN;
offboardRequested=false;lastHeartbeat=-Inf;lastRefTime=-Inf;
lastEstimateSource=-Inf;lastTruthSource=-Inf;
lastConfirmedArmed=NaN;
counts=struct('arm_requests',0,'offboard_requests',0,'land_requests',0, ...
    'observed_arm_transitions',0,'observed_disarm_transitions',0, ...
    'standard_disarm_requests',0,'force_disarm_requests',0, ...
    'mapping_write_attempts',0,'mapping_verified_writes',0, ...
    'mapping_rollback_attempts',0,'mapping_verified_rollbacks',0, ...
    'tuning_apply_setter_attempts',0,'tuning_apply_setter_returns',0, ...
    'tuning_apply_ack_readback_verified',0,'tuning_apply_no_op',0, ...
    'tuning_restore_setter_attempts',0,'tuning_restore_setter_returns',0, ...
    'tuning_restore_ack_readback_verified',0,'tuning_restore_no_op',0, ...
    'ra_ctrl_mode_writes',0,'physical_output_actions',0,'com_open_count',0);
target=[NaN,NaN,NaN];truthTarget=target;coordinateOffset=target;
guard=onCleanup(@finish); %#ok<NASGU>
try
    t0=io.now();
    while io.now()-t0<cfg.preflight_deadline_s
        observe(false);sendHeartbeatIfDue();
        if modelFatal(latest),assertModelReady('m600check:PrearmModelDiagnosticNotReady');end
        if ready(latest),break;end
        io.sleep(cfg.poll_period_s);
    end
    assertModelReady('m600check:PrearmModelDiagnosticNotReady');
    assertSnapshotCondition(plantGround(latest),'m600check:PrearmPlantGroundNotConfirmed', ...
        'M600 model contact-plane confirmation is required in addition to PX4 landed status.');
    assertSnapshotCondition(ready(latest),'m600check:PrearmTelemetryOrClock', ...
        'Fresh finite estimate/truth, bounded clock mapping and disarmed/landed were not established.');
    guards={'SYS_HITL',1;'SYS_AUTOSTART',6001;'MAV_TYPE',13;'CA_ROTOR_COUNT',6;'RA_CTRL_MODE',0};
    for i=1:size(guards,1)
        p=io.readParameter(guards{i,1});
        assert(p.mav_type==6&&p.decoded==guards{i,2},'m600check:ProfileMismatch','%s type/value differs.',guards{i,1});
    end
    for i=1:16,p=io.readParameter(sprintf('HIL_ACT_FUNC%d',i));assert(p.mav_type==6&&p.decoded==0,'m600check:HILMappingNonzero','HIL mapping type/value not INT32 zero.');end
    for i=1:8,p=io.readParameter(sprintf('PWM_MAIN_FUNC%d',i));assert(p.mav_type==6&&p.decoded==0,'m600check:PhysicalMappingNonzero','Physical mapping type/value not INT32 zero.');end
    if isfield(cfg,'virtual_sensor_mount_contract')
        sensorMountCheck=m600check.verifyVirtualSensorMountContract(io.readParameter,cfg.virtual_sensor_mount_contract);
        assert(sensorMountCheck.passed,'m600check:SensorFrameIdentityMismatch', ...
            'Installed raw-sensor adapter and live calibration/mount identity disagree; no mapping or arm.');
    end
    observe(false);assertModelReady('m600check:PrewriteModelDiagnosticNotReady');
    assertSnapshotCondition(ready(latest),'m600check:PrewriteStateLost','Fresh safe state lost before writes.');
    % Measure a constant origin translation while disarmed and grounded.
    coordinateOffset=latest.truth.position_ned_m-latest.estimate.position_ned_m;
    assert(norm(coordinateOffset)<=cfg.maximum_initial_origin_offset_m, ...
        'm600check:InitialOriginMismatch','Initial PX4/plant origins exceed the caller bound.');
    target=latest.estimate.position_ned_m+[0,0,-cfg.hover_altitude_m];
    truthTarget=target+coordinateOffset;
    if ~isempty(tuningCheck)
        parameterUnionOriginal=m600check.verifyNativeHoverParameterUnion(io.readParameter, ...
            cfg.native_hover_tuning,cfg.temporary_allocator_geometry,'ORIGINAL');
        assert(parameterUnionOriginal.passed,'m600check:NativeHoverOriginal138', ...
            'Original typed138 differ before geometry/tuning/mapping mutation: %s',parameterUnionOriginal.failure);
    end
    if ~isempty(geometryCheck)
        % Read the 67-field identity before geometry or tuning changes.
        g=cfg.temporary_allocator_geometry.unchanged_guard_entries;
        for k=1:numel(g)
            p=io.readParameter(g(k).name);
            assert(p.mav_type==g(k).mav_type&&strcmpi(p.raw_bits_hex,g(k).raw_bits_hex), ...
                'm600check:AllocatorUnchangedIdentity','Unchanged control/profile field differs: %s',g(k).name);
        end
        geometryTouched=true;geometryRestored=false;
        geometryApply=m600check.transferNativeAllocatorGeometry(io, ...
            cfg.temporary_allocator_geometry.entries,'APPLY',@geometryApplyGuard);
        event('ALLOCATOR_GEOMETRY_APPLY',sprintf('attempts=%d verified=%d pass=%d', ...
            geometryApply.setter_invocation_attempt_count,geometryApply.ack_and_readback_verified_count,geometryApply.passed));
        assert(geometryApply.passed,'m600check:AllocatorGeometryApply', ...
            'Temporary geometry apply failed; retain exact partial actions and restore before any arm.');
    end
    if ~isempty(tuningCheck)
        tuningTouched=true;tuningRestored=false;
        tuningApply=m600check.transferNativeHoverTuning(io,cfg.native_hover_tuning.entries,'APPLY',@geometryApplyGuard);
        recordTuningCounts('apply',tuningApply);
        event('NATIVE_HOVER_TUNING_APPLY',sprintf('attempts=%d verified=%d pass=%d', ...
            tuningApply.setter_invocation_attempt_count,tuningApply.ack_and_readback_verified_count,tuningApply.passed));
        assert(tuningApply.passed&&tuningApply.final_identity_verified,'m600check:NativeHoverTuningApply', ...
            'Exact three-field apply was not verified; retain partial actions and recover without mapping/arm.');
        parameterUnionApplied=m600check.verifyNativeHoverParameterUnion(io.readParameter, ...
            cfg.native_hover_tuning,cfg.temporary_allocator_geometry,'APPLIED');
        assert(parameterUnionApplied.passed,'m600check:NativeHoverApplied138', ...
            'Combined geometry12+tuning3 targets or unchanged123 differ: %s',parameterUnionApplied.failure);
    end
    for i=1:6
        observe(false);assertSnapshotCondition(ready(latest),'m600check:PrewriteStateLost','Fresh safe state lost during mapping apply.');
        name=sprintf('HIL_ACT_FUNC%d',i);mapped(end+1,1)=string(name); %#ok<AGROW>
        writeMapping(name,100+i,'APPLY');
    end
    phase='PRESTREAM';t0=io.now();
    while io.now()-t0<cfg.prestream_s
        holdReference();observe(true);io.sleep(cfg.poll_period_s);
    end
    observe(true);assertModelReady('m600check:PreOffboardModelDiagnosticNotReady');
    assertSnapshotCondition(ready(latest),'m600check:PreOffboardStateLost','Fresh disarmed/landed state and current sensor validity lost before Offboard.');
    counts.offboard_requests=counts.offboard_requests+1;offboardRequested=true;
    request(176,[1,6,0,0,0,0,0],'OFFBOARD');
    assertSnapshotCondition(waitFor(@(s)stateFresh(s)&&s.main_mode==6,cfg.mode_timeout_s,true), ...
        'm600check:OffboardStateTimeout','Offboard heartbeat was not confirmed; arm not sent.');
    observe(true);assertModelReady('m600check:PreArmModelDiagnosticNotReady');
    assertSnapshotCondition(ready(latest),'m600check:PreArmStateLost','Fresh disarmed/landed state and current sensor validity lost before logical arm.');
    assertSnapshotCondition(latest.main_mode==6,'m600check:PreArmOffboardLost','Offboard state was lost before logical arm.');
    counts.arm_requests=counts.arm_requests+1;armRequested=true;
    request(400,[1,0,0,0,0,0,0],'LOGICAL_ARM');
    assertSnapshotCondition(waitFor(@(s)stateFresh(s)&&s.armed==1,cfg.arm_timeout_s,true), ...
        'm600check:ArmStateTimeout','Fresh armed heartbeat was not observed.');
    phase='M600_HOVER';formalStart=io.now();event('FORMAL_BEGIN','');
    while io.now()-formalStart<cfg.hover_duration_s
        holdReference();observe(true);io.sleep(cfg.poll_period_s);
    end
    formalEnd=io.now();event('FORMAL_END','');
    normalLand();
catch problem
    failure=[problem.identifier ': ' problem.message];event('EXCEPTION',failure);
end
finish();clear guard
if isempty(failure)&&~isempty(firstModelFailure)
    failure=['m600check:ModelDiagnosticNotReady: ' firstModelFailure.reason];
end

evaluation=struct('passed',false,'status','FORMAL_WINDOW_NOT_COMPLETED');
if isfinite(formalStart)&&isfinite(formalEnd)&&~isempty(log)
    policy=cfg.evaluation_policy;
    policy.clock_id=clockId;
    policy.clock_mapping_evidence='RECORDED_PX4_TIMESYNC_INTERVAL_AND_COPTERSIM_SYSTEM_START_UTC';
    policy.window_s=[formalStart,formalEnd];
    evaluation=m600check.evaluateThreeStream(log,policy);
end
actual=io.evidence();
result=struct('schema','M600_MATLAB_COPTERSIM_NATIVE_HOVER_V1', ...
    'status','VALID_COMPLETED_WITH_PERFORMANCE_NOT_MET', ...
    'formal_entered',isfinite(formalStart),'formal_completed',isfinite(formalEnd), ...
    'failure',failure,'first_model_failure',firstModelFailure, ...
    'first_failed_snapshot',firstFailedSnapshot,'first_failed_state_evidence',firstFailedStateEvidence, ...
    'first_failed_snapshot_encoded',encodeSnapshotLosslessly(firstFailedSnapshot), ...
    'final_model_diagnostic',diagnosticValue(latest), ...
    'evaluation',evaluation,'counts',counts,'events',events, ...
    'parameter_actions',parameterActions,'three_stream_log',log, ...
    'allocator_geometry_contract',geometryCheck,'allocator_geometry_apply',geometryApply, ...
    'allocator_geometry_restore',geometryRestore,'allocator_geometry_restored',geometryRestored, ...
    'native_hover_tuning_contract',tuningCheck,'native_hover_tuning_apply',tuningApply, ...
    'native_hover_tuning_restore',tuningRestore,'native_hover_tuning_restored',tuningRestored, ...
    'native_hover_parameter_union_original',parameterUnionOriginal, ...
    'native_hover_parameter_union_applied',parameterUnionApplied, ...
    'native_hover_parameter_union_restored',parameterUnionRestored, ...
    'native_hover_original138_restored',parameterUnionSafe, ...
    'px4_target_ned_m',target,'truth_target_ned_m',truthTarget, ...
    'fixed_prearm_origin_translation_m',coordinateOffset, ...
    'transport_evidence',actual,'fresh_final_safe_state',safeFinal, ...
    'fresh_final_hardware_safe_state',hardwareFinal,'final_plant_ground_confirmed',plantGround(latest), ...
    'matlab_sockets_closed',socketsClosed,'outer_final_safety_still_required',true, ...
    'board_identity_source',cfg.outer_preflight_receipt, ...
    'plant_identity',cfg.plant_identity, 'config',cfg,'sensor_mount_check',sensorMountCheck, ...
    'hover_preparation_evidence',hoverPreparation, ...
    'claim','NATIVE_PX4_AND_EFFECTIVE_M600_COPTERSIM_SHORT_HOVER_ONLY', ...
    'truth_to_controller_or_estimator',false,'python_hil_driver',false);
if ~isempty(failure)||~safeFinal||~socketsClosed
    result.status='INFRASTRUCTURE_OR_TERMINATION_NOT_COMPLETED';
elseif ~isfield(evaluation,'data_admission_pass')||~evaluation.data_admission_pass
    result.status='FORMAL_COMPLETED_BUT_THREE_STREAM_DATA_INVALID';
elseif evaluation.passed
    result.status='PASS_CALLER_DEFINED_M600_NATIVE_HOVER';
end
if cfg.write_artifacts
    assert(~isfolder(outputDir),'m600check:OutputExists','Refusing to overwrite output directory.');
    mkdir(outputDir);
    % MAT preserves all uint64/raw values; JSON is the compact review entry.
    save(fullfile(outputDir,'RAW_MATLAB_HIL.mat'),'result','-v7.3');
    if isfield(evaluation,'rows')&&~isempty(evaluation.rows)
        writetable(struct2table(evaluation.rows),fullfile(outputDir,'THREE_STREAM_EVALUATION.csv'));
    end
    compact=result;compact.three_stream_log=[];compact.transport_evidence=[];
    compact.first_failed_snapshot=[]; % Exact original in MAT; typed raw bits in JSON below.
    if isfield(compact.evaluation,'raw_streams'),compact.evaluation.raw_streams=[];end
    if isfield(compact.evaluation,'rows'),compact.evaluation.rows=[];end
    writeJson(fullfile(outputDir,'RESULT.json'),compact);
end

    function event(kind,detail)
        events(end+1,1)=struct('time_s',io.now(),'kind',kind,'detail',detail); %#ok<AGROW>
    end
    function sendHeartbeatIfDue()
        if io.now()-lastHeartbeat>=cfg.heartbeat_period_s
            io.sendHeartbeat();lastHeartbeat=io.now();
        end
    end
    function holdReference()
        io.sendSetpoint(target);
        now=io.now();
        if now>lastRefTime
            sample=struct('clock_id',clockId,'time_s',now,'received_at_s',now, ...
                'position_ned_m',truthTarget,'velocity_ned_mps',[0,0,0]);
            log=m600check.appendThreeStreamSample(log,'reference',sample);lastRefTime=now;
        end
        sendHeartbeatIfDue();
    end
    function observe(requireFlight)
        latest=io.snapshot();
        % Check plant health independently of clock validity; a failed model may
        % continue sending timestamped frozen truth.
        if ~finalized&&~modelReady(latest)&&(requireFlight||offboardRequested||modelFatal(latest))
            rememberModelFailure();
        end
        if stateFresh(latest)
            if lastConfirmedArmed==0&&latest.armed==1
                counts.observed_arm_transitions=counts.observed_arm_transitions+1;
            elseif lastConfirmedArmed==1&&latest.armed==0
                counts.observed_disarm_transitions=counts.observed_disarm_transitions+1;
            end
            lastConfirmedArmed=latest.armed;
        end
        if latest.clock_valid&&all(isfinite(coordinateOffset))
            if ~isempty(latest.estimate)&&latest.estimate.raw_source_time_s>lastEstimateSource
                sample=latest.estimate.sample;sample.clock_id=clockId;
                sample.position_ned_m=sample.position_ned_m+coordinateOffset;
                log=m600check.appendThreeStreamSample(log,'estimate',sample);
                lastEstimateSource=latest.estimate.raw_source_time_s;
            end
            if ~isempty(latest.truth)&&latest.truth.raw_source_time_s>lastTruthSource
                sample=latest.truth.sample;sample.clock_id=clockId;
                log=m600check.appendThreeStreamSample(log,'truth',sample);
                lastTruthSource=latest.truth.raw_source_time_s;
            end
        end
        if requireFlight
            assertModelReady('m600check:ModelDiagnosticNotReady');
            assertSnapshotCondition(stateFresh(latest),'m600check:HeartbeatUnknown','Heartbeat/land state lost; armed status unknown.');
            assertSnapshotCondition(latest.clock_valid&&streamFresh(latest),'m600check:TelemetryInvalid','Clock or state streams became stale/invalid.');
            assertSnapshotCondition(nativeTelemetryFresh(latest),'m600check:NativeTelemetryInvalid', ...
                'Current ATTITUDE/LOCAL_POSITION/ESTIMATOR source, receive time or validity failed.');
            assert(norm(latest.truth.velocity_ned_mps)<=cfg.abort_truth_speed_mps, ...
                'm600check:TruthSpeedAbort','Truth speed crossed the caller abort bound.');
            assert(norm(latest.estimate.position_ned_m+coordinateOffset-latest.truth.position_ned_m)<=cfg.abort_estimator_gap_m, ...
                'm600check:EstimatorGapAbort','Estimator/plant gap crossed the caller abort bound.');
            if strcmp(phase,'M600_HOVER')
                assertSnapshotCondition(latest.armed==1,'m600check:FormalArmedStateLost','Unexpected disarm interrupted the formal hover.');
                assertSnapshotCondition(latest.main_mode==6,'m600check:FormalOffboardStateLost','Unexpected mode exit interrupted the formal hover.');
            end
        end
    end
    function yes=stateFresh(s)
        yes=~isempty(s)&&all(isfinite([s.armed,s.landed_state,s.heartbeat_rx_s,s.extended_rx_s]))&& ...
            ismember(s.armed,[0,1])&&io.now()>=s.heartbeat_rx_s&&io.now()>=s.extended_rx_s&& ...
            io.now()-s.heartbeat_rx_s<=cfg.heartbeat_max_age_s&& ...
            io.now()-s.extended_rx_s<=cfg.landed_max_age_s;
    end
    function yes=streamFresh(s)
        yes=~isempty(s.estimate)&&~isempty(s.truth)&& ...
            all(isfinite([s.estimate.position_ned_m,s.estimate.velocity_ned_mps, ...
            s.truth.position_ned_m,s.truth.velocity_ned_mps]))&& ...
            all(isfinite([s.estimate.rx_s,s.truth.rx_s]))&& ...
            io.now()>=s.estimate.rx_s&&io.now()>=s.truth.rx_s&& ...
            io.now()-s.estimate.rx_s<=cfg.state_max_age_s&&io.now()-s.truth.rx_s<=cfg.state_max_age_s;
    end
    function yes=ready(s)
        yes=modelReady(s)&&plantGround(s)&&stateFresh(s)&&s.armed==0&&s.landed_state==1&&s.clock_valid&&streamFresh(s)&&nativeTelemetryFresh(s);
    end
    function yes=nativeTelemetryFresh(s)
        yes=true;
        if isfield(cfg,'require_current_native_telemetry')&&cfg.require_current_native_telemetry
            yes=isstruct(s)&&isfield(s,'native_hover_telemetry')&& ...
                isstruct(s.native_hover_telemetry)&&isscalar(s.native_hover_telemetry)&& ...
                isfield(s.native_hover_telemetry,'passed')&&isequal(s.native_hover_telemetry.passed,true);
        end
    end
    function yes=modelReady(s)
        yes=~isempty(s)&&isfield(s,'model_ready')&&isscalar(s.model_ready)&& ...
            (islogical(s.model_ready)||isnumeric(s.model_ready))&&s.model_ready==1;
    end
    function d=diagnosticValue(s)
        d=[];if ~isempty(s)&&isfield(s,'model_diagnostic'),d=s.model_diagnostic;end
    end
    function yes=plantGround(s)
        d=diagnosticValue(s);yes=modelReady(s)&&isstruct(d)&&isfield(d,'decoded')&& ...
            isstruct(d.decoded)&&isfield(d.decoded,'ground_confirmed')&& ...
            isscalar(d.decoded.ground_confirmed)&&d.decoded.ground_confirmed==1;
    end
    function reason=modelReason(s)
        reason='MODEL_DIAGNOSTIC_MISSING';d=diagnosticValue(s);
        if isstruct(d)&&isscalar(d)&&isfield(d,'status'),reason=char(d.status);end
    end
    function yes=modelFatal(s)
        d=diagnosticValue(s);yes=isstruct(d)&&isscalar(d)&& ...
            isfield(d,'fatal_reason')&&~isempty(d.fatal_reason);
    end
    function rememberModelFailure()
        if isempty(firstModelFailure)
            firstModelFailure=struct('time_s',io.now(),'phase',phase, ...
                'reason',modelReason(latest),'diagnostic',diagnosticValue(latest));
            event('MODEL_DIAGNOSTIC_NOT_READY',firstModelFailure.reason);
        end
    end
    function assertModelReady(identifier)
        if ~modelReady(latest)
            rememberModelFailure();
            rememberStateFailure(identifier,['Model diagnostic is not healthy/fresh/advancing: ' modelReason(latest)]);
            error(identifier,'Model diagnostic is not healthy/fresh/advancing: %s.',modelReason(latest));
        end
    end
    function assertSnapshotCondition(condition,identifier,message)
        % Same condition, error and action ordering as the original assertion.
        if ~condition,rememberStateFailure(identifier,message);end
        assert(condition,identifier,message);
    end
    function rememberStateFailure(identifier,message)
        if ~isempty(firstFailedStateEvidence),return;end
        % Use the snapshot acquired by observe without draining the adapter again.
        firstFailedSnapshot=latest;
        firstFailedStateEvidence=struct('schema','M600_FIRST_FAILED_STATE_V1', ...
            'trigger',identifier,'message',message,'phase',phase,'captured_at_s',io.now(), ...
            'during_finalization',finalized,'checks',struct(),'diagnostic_error','', ...
            'evidence_basis','EXACT_EXISTING_SNAPSHOT_NO_ADAPTER_REPOLL__FULL_RAW_IN_TRANSPORT_EVIDENCE');
        try
            nowValue=firstFailedStateEvidence.captured_at_s;
            s=latest;
            hb=fieldOr(s,'heartbeat_rx_s',NaN);ex=fieldOr(s,'extended_rx_s',NaN);
            arm=fieldOr(s,'armed',NaN);land=fieldOr(s,'landed_state',NaN);
            f=struct('model_ready',modelReady(s),'plant_ground',plantGround(s), ...
                'clock_valid',isequal(fieldOr(s,'clock_valid',false),true), ...
                'heartbeat_fresh',finiteScalar(hb)&&nowValue>=hb&&nowValue-hb<=cfg.heartbeat_max_age_s, ...
                'extended_state_fresh',finiteScalar(ex)&&nowValue>=ex&&nowValue-ex<=cfg.landed_max_age_s, ...
                'armed_value_valid',finiteScalar(arm)&&ismember(arm,[0,1]), ...
                'disarmed',isequal(arm,0),'landed',isequal(land,1), ...
                'state_fresh',stateFresh(s),'streams_fresh',streamFresh(s), ...
                'native_telemetry_required',isfield(cfg,'require_current_native_telemetry')&&cfg.require_current_native_telemetry, ...
                'native_telemetry_passed',nativeTelemetryFresh(s));
            for name={'estimate','truth'}
                x=fieldOr(s,name{1},[]);has=isstruct(x)&&isscalar(x);
                pos=fieldOr(x,'position_ned_m',NaN);vel=fieldOr(x,'velocity_ned_mps',NaN);rx=fieldOr(x,'rx_s',NaN);
                f.([name{1} '_present'])=has;
                f.([name{1} '_finite'])=has&&all(isfinite([pos(:);vel(:)]));
                f.([name{1} '_receive_fresh'])=has&&finiteScalar(rx)&&nowValue>=rx&&nowValue-rx<=cfg.state_max_age_s;
            end
            firstFailedStateEvidence.checks=f;
            firstFailedStateEvidence.observed_values=struct('snapshot_time_s',fieldOr(s,'now_s',NaN), ...
                'armed',arm,'landed_state',land,'main_mode',fieldOr(s,'main_mode',NaN), ...
                'heartbeat_age_s',nowValue-hb,'extended_state_age_s',nowValue-ex);
            firstFailedStateEvidence.clock_diagnostic=fieldOr(s,'clock_diagnostic',[]);
            firstFailedStateEvidence.native_hover_telemetry=fieldOr(s,'native_hover_telemetry',[]);
        catch problem
            % Preserve the original failure if diagnostic enrichment fails.
            firstFailedStateEvidence.diagnostic_error=[problem.identifier ': ' problem.message];
        end
        event('FIRST_FAILED_STATE_CAPTURED',identifier);
    end
    function yes=waitFor(predicate,deadline,sendHold)
        t=io.now();yes=false;
        while io.now()-t<deadline
            if sendHold,holdReference();else,sendHeartbeatIfDue();end
            observe(sendHold);
            if predicate(latest),yes=true;return;end
            io.sleep(cfg.poll_period_s);
        end
    end
    function request(command,params,label)
        sent=io.now();io.requestCommand(command,params);event([label '_REQUEST'],'');
        t=io.now();ack=[];
        while io.now()-t<cfg.command_timeout_s
            if offboardRequested&&~normalLandRequested&&all(isfinite(target)),holdReference();end
            observe(strcmp(label,'OFFBOARD')||strcmp(label,'LOGICAL_ARM'));
            ack=io.commandAck(command,sent);
            if ~isempty(ack),break;end
            io.sleep(cfg.poll_period_s);
        end
        assert(~isempty(ack),'m600check:CommandAckMissing','Missing ACK for %s.',label);
        event([label '_ACK'],sprintf('result=%g',ack.result));
        assert(ack.result==0,'m600check:CommandRejected','%s rejected with result %g.',label,ack.result);
    end
    function normalLand()
        phase='NATIVE_LAND'; %#ok<NASGU>
        if ~normalLandRequested
            normalLandRequested=true;counts.land_requests=counts.land_requests+1;
            landDeadline=io.now()+cfg.land_timeout_s;
            request(176,[1,4,6,0,0,0,0],'AUTO_LAND');
        end
        assert(waitFor(@(s)stateFresh(s)&&s.main_mode==4&&s.sub_mode==6, ...
            cfg.mode_timeout_s,false),'m600check:LandModeTimeout','LAND mode not confirmed.');
        assert(waitFor(@(s)stateFresh(s)&&s.landed_state==1,max(0,landDeadline-io.now()),false), ...
            'm600check:LandingTimeout','Native landed state not reached.');
        if latest.armed==1&&~disarmRequested
            disarmRequested=true;counts.standard_disarm_requests=counts.standard_disarm_requests+1;
            request(400,[0,0,0,0,0,0,0],'STANDARD_DISARM');
        end
        assert(waitFor(@(s)stateFresh(s)&&s.armed==0&&s.landed_state==1,cfg.disarm_timeout_s,false), ...
            'm600check:DisarmTimeout','Fresh disarmed/landed state not confirmed.');
        if ~finalized
            assert(plantGround(latest),'m600check:NativeLandingPlantGroundNotConfirmed', ...
                'Native termination needs both PX4 landed and healthy M600 ground-contact evidence.');
        end
    end
    function writeMapping(name,targetValue,purpose)
        a=struct('name',name,'target',targetValue,'purpose',purpose,'attempted',true, ...
            'write_returned',false,'readback_verified',false,'raw_bits_hex','','error','');
        parameterActions(end+1,1)=a;idx=numel(parameterActions);
        if strcmp(purpose,'APPLY'),counts.mapping_write_attempts=counts.mapping_write_attempts+1;
        else,counts.mapping_rollback_attempts=counts.mapping_rollback_attempts+1;end
        try
            p=io.setIntegerParameter(name,targetValue);parameterActions(idx).write_returned=true;
            assert(p.mav_type==6&&p.decoded==targetValue,'m600check:MappingReadback','Typed mapping mismatch.');
            parameterActions(idx).readback_verified=true;parameterActions(idx).raw_bits_hex=p.raw_bits_hex;
            if strcmp(purpose,'APPLY'),counts.mapping_verified_writes=counts.mapping_verified_writes+1;
            else,counts.mapping_verified_rollbacks=counts.mapping_verified_rollbacks+1;end
        catch problem
            parameterActions(idx).error=problem.message;rethrow(problem);
        end
    end
    function finish()
        if finalized,return;end
        finalized=true;
        try
            observe(false);
            if stateFresh(latest)&&latest.armed==1
                normalLand();
            elseif ~stateFresh(latest)
                event('FINAL_ARMED_STATE_UNKNOWN','No blind arm/disarm command; outer recovery required.');
            end
        catch problem,event('FINAL_NATIVE_TERMINATION_ERROR',problem.message);end
        for k=numel(mapped):-1:1
            try
                observe(false);
                assert(stateFresh(latest)&&latest.armed==0&&latest.landed_state==1, ...
                    'm600check:RecoverySafeStateUnknown', ...
                    'Do not remove virtual control while armed; outer recovery must first establish disarmed/landed.');
                writeMapping(char(mapped(k)),0,'ROLLBACK');
            catch problem,event('MAPPING_ROLLBACK_ERROR',problem.message);end
        end
        if geometryTouched
            try
                geometryRestore=m600check.transferNativeAllocatorGeometry(io, ...
                    cfg.temporary_allocator_geometry.entries,'RESTORE',@geometryRestoreGuard);
                geometryRestored=geometryRestore.passed;
                event('ALLOCATOR_GEOMETRY_RESTORE',sprintf('attempts=%d verified=%d pass=%d', ...
                    geometryRestore.setter_invocation_attempt_count,geometryRestore.ack_and_readback_verified_count,geometryRestore.passed));
            catch problem
                geometryRestored=false;event('ALLOCATOR_GEOMETRY_RESTORE_ERROR',[problem.identifier ': ' problem.message]);
            end
        end
        % Attempt geometry and tuning restoration independently.
        if tuningTouched
            try
                tuningRestore=m600check.transferNativeHoverTuning(io,cfg.native_hover_tuning.entries,'RESTORE',@geometryRestoreGuard);
                recordTuningCounts('restore',tuningRestore);
                tuningRestored=tuningRestore.passed&&tuningRestore.final_identity_verified;
                event('NATIVE_HOVER_TUNING_RESTORE',sprintf('attempts=%d verified=%d pass=%d', ...
                    tuningRestore.setter_invocation_attempt_count,tuningRestore.ack_and_readback_verified_count,tuningRestore.passed));
            catch problem
                tuningRestored=false;event('NATIVE_HOVER_TUNING_RESTORE_ERROR',[problem.identifier ': ' problem.message]);
            end
        end
        if ~isempty(tuningCheck)
            try
                parameterUnionRestored=m600check.verifyNativeHoverParameterUnion(io.readParameter, ...
                    cfg.native_hover_tuning,cfg.temporary_allocator_geometry,'RESTORED');
                parameterUnionSafe=parameterUnionRestored.passed;
                if ~parameterUnionSafe,event('NATIVE_HOVER_RESTORED138_MISMATCH',parameterUnionRestored.failure);end
            catch problem
                parameterUnionSafe=false;event('NATIVE_HOVER_RESTORED138_ERROR',[problem.identifier ': ' problem.message]);
            end
        end
        try
            observe(false);hardwareFinal=stateFresh(latest)&&latest.armed==0&&latest.landed_state==1;
            for k=1:16,p=io.readParameter(sprintf('HIL_ACT_FUNC%d',k));hardwareFinal=hardwareFinal&&(p.mav_type==6&&p.decoded==0);end
            for k=1:8,p=io.readParameter(sprintf('PWM_MAIN_FUNC%d',k));hardwareFinal=hardwareFinal&&(p.mav_type==6&&p.decoded==0);end
            observe(false);hardwareFinal=hardwareFinal&&stateFresh(latest)&&latest.armed==0&&latest.landed_state==1;
            safeFinal=hardwareFinal&&plantGround(latest)&&geometryRestored&&tuningRestored&&parameterUnionSafe;
        catch problem,safeFinal=false;hardwareFinal=false;event('FINAL_READBACK_ERROR',problem.message);end
        try,socketsClosed=io.close();catch problem,event('SOCKET_CLOSE_ERROR',problem.message);end
    end
    function yes=geometryApplyGuard()
        yes=geometryZeroOutputGuard();
        observe(false);yes=logical(yes&&ready(latest));
    end
    function recordTuningCounts(purpose,r)
        counts.(['tuning_' purpose '_setter_attempts'])=r.setter_invocation_attempt_count;
        counts.(['tuning_' purpose '_setter_returns'])=r.setter_return_count;
        counts.(['tuning_' purpose '_ack_readback_verified'])=r.ack_and_readback_verified_count;
        counts.(['tuning_' purpose '_no_op'])=r.no_op_count;
    end
    function yes=geometryRestoreGuard()
        % Continue safe hardware restoration after model failure.
        yes=geometryZeroOutputGuard();
    end
    function yes=geometryZeroOutputGuard()
        observe(false);yes=stateFresh(latest)&&latest.armed==0&&latest.landed_state==1;
        if ~yes,yes=false;return;end
        for j=1:16
            p=io.readParameter(sprintf('HIL_ACT_FUNC%d',j));
            yes=yes&&p.mav_type==6&&p.decoded==0&&strcmpi(p.raw_bits_hex,'00000000');
        end
        for j=1:8
            p=io.readParameter(sprintf('PWM_MAIN_FUNC%d',j));
            yes=yes&&p.mav_type==6&&p.decoded==0&&strcmpi(p.raw_bits_hex,'00000000');
        end
        p=io.readParameter('RA_CTRL_MODE');yes=yes&&p.mav_type==6&&p.decoded==0;
        observe(false);yes=logical(yes&&stateFresh(latest)&&latest.armed==0&&latest.landed_state==1);
    end
end

function validateConfig(c)
required={'live_enabled','outer_preflight_pass','outer_preflight_receipt','plant_identity', ...
    'write_artifacts','hover_altitude_m','hover_duration_s','prestream_s','preflight_deadline_s', ...
    'mode_timeout_s','arm_timeout_s','land_timeout_s','disarm_timeout_s','command_timeout_s', ...
    'poll_period_s','heartbeat_period_s','heartbeat_max_age_s','landed_max_age_s','state_max_age_s', ...
    'maximum_initial_origin_offset_m','abort_truth_speed_mps','abort_estimator_gap_m','evaluation_policy'};
assert(isstruct(c)&&isscalar(c)&&all(isfield(c,required)),'m600check:RunnerConfig', ...
    'Every runner bound/provenance field must be explicitly supplied.');
for k=6:22
    if ismember(required{k},{'write_artifacts'}),continue;end
    v=c.(required{k});assert(isnumeric(v)&&isreal(v)&&isscalar(v)&&isfinite(v)&&v>0, ...
        'm600check:RunnerConfig','%s must be a positive finite caller value.',required{k});
end
% Validate the exact evaluator schema before constructing any live adapter.
% An empty log is intentionally invalid data, but exercises all policy checks.
p=c.evaluation_policy;p.clock_id='PRELAUNCH_POLICY_VALIDATION';
p.clock_mapping_evidence='PRELAUNCH_POLICY_VALIDATION';p.window_s=[0,c.hover_duration_s];
m600check.evaluateThreeStream(struct('reference',struct([]),'estimate',struct([]),'truth',struct([])),p);
end

function writeJson(path,value)
temporary=string(path)+".tmp";fid=fopen(temporary,'w','n','UTF-8');
assert(fid>=0,'m600check:ResultOpen','Cannot write result.');guard=onCleanup(@()fclose(fid));
fprintf(fid,'%s\n',jsonencode(value,PrettyPrint=true));clear guard
movefile(temporary,path,'f');
end

function value=fieldOr(s,name,fallback)
value=fallback;if isstruct(s)&&isscalar(s)&&isfield(s,name),value=s.(name);end
end
function yes=finiteScalar(v)
yes=isnumeric(v)&&isreal(v)&&isscalar(v)&&isfinite(v);
end
function encoded=encodeSnapshotLosslessly(snapshot)
if isempty(snapshot),encoded=[];return;end
[~,~,byteOrder]=computer;
encoded=struct('encoding','MATLAB_TYPED_RECURSIVE_RAW_BITS_HEX_V1', ...
    'byte_order',byteOrder,'value',typedValue(snapshot));
end
function out=typedValue(value)
% Preserve NaN payloads, infinities, signed zero and integer bits in JSON and MAT.
out=struct('matlab_class',class(value),'shape',size(value));
if isnumeric(value)
    out.real_hex=hexBytes(typecast(real(value(:)),'uint8'));
    out.imag_hex='';if ~isreal(value),out.imag_hex=hexBytes(typecast(imag(value(:)),'uint8'));end
elseif islogical(value)
    out.bytes_hex=hexBytes(uint8(value(:)));
elseif ischar(value)
    out.utf16_hex=hexBytes(typecast(uint16(value(:)),'uint8'));
elseif isstruct(value)
    out.field_names=fieldnames(value);out.elements=cell(numel(value),1);
    for k=1:numel(value)
        entry=struct();for j=1:numel(out.field_names)
            name=out.field_names{j};entry.(name)=typedValue(value(k).(name));
        end
        out.elements{k}=entry;
    end
elseif iscell(value)
    out.elements=cell(numel(value),1);for k=1:numel(value),out.elements{k}=typedValue(value{k});end
elseif isstring(value)
    out.elements=cell(numel(value),1);out.missing=ismissing(value(:));
    for k=1:numel(value),if ~out.missing(k),out.elements{k}=typedValue(char(value(k)));end,end
else
    % Preserve future MATLAB-only classes without string conversion.
    out.matlab_byte_stream_hex=hexBytes(getByteStreamFromArray(value));
end
end
function text=hexBytes(bytes)
text=upper(reshape(dec2hex(bytes,2).',1,[]));
end
