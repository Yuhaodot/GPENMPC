function result=run_m600_matlab_delivery_hil(outputDir,cfg,io)
%RUN_M600_MATLAB_DELIVERY_HIL MATLAB-owned PX4/M600 physical-delivery loop.
% The host supplies C3 p/v/a/yaw references and a 28-D plant environment;
% PX4 supplies the native position/attitude/rate/allocation controller.
% Each delivery is native LAND -> dual ground -> standard disarm -> 8 s
% service -> measured mass ACK -> Offboard/rearm, with no airborne unload.
arguments
    outputDir (1,1) string
    cfg (1,1) struct
    io = []
end
buildRoot=fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(buildRoot,'tools'),fullfile(buildRoot,'matlab_validation'), ...
    fullfile(buildRoot,'m600_coptersim','matlab_validation'),'-begin');
[task,timeline,deliveryConfig,relaunchConfig]=validateAndLoad(cfg);
geometryCheck=[];geometryApply=[];geometryRestore=[];geometryTouched=false;geometryRestored=true;
if isfield(cfg,'temporary_allocator_geometry')
    geometryCheck=m600check.validateTemporaryAllocatorGeometry(cfg.temporary_allocator_geometry);
    assert(geometryCheck.passed,'m600delivery:AllocatorGeometryContract','%s',geometryCheck.failure);
end
tuningCheck=[];tuningApply=[];tuningRestore=[];tuningTouched=false;tuningRestored=true;
parameterUnionOriginal=[];parameterUnionApplied=[];parameterUnionRestored=[];parameterUnionSafe=true;
if isfield(cfg,'native_hover_tuning')
    if strcmp(cfg.native_hover_tuning.schema,'TEMPORARY_M600_CANONICAL_DELIVERY_NATIVE_TUNING_V1')
        tuningCheck=validate_m600_delivery_native_tuning_contract(cfg.native_hover_tuning);
    else
        tuningCheck=m600check.validateNativeHoverTuningContract(cfg.native_hover_tuning);
    end
    assert(tuningCheck.passed,'m600delivery:NativeHoverTuningContract','%s',tuningCheck.failure);
    assert(~isempty(geometryCheck),'m600delivery:TuningNeedsGeometry', ...
        'Native hover tuning requires the validated allocator-geometry contract.');
    parameterUnionSafe=false;
end
parent=char(cfg.delivery_mission.gpenmpc_parent_matlab_root);
oldPath=path;pathGuard=onCleanup(@()path(oldPath)); %#ok<NASGU>
addpath(parent,'-begin');
if cfg.write_artifacts
    assert(~isfolder(outputDir)&&~isfile(outputDir),'m600delivery:OutputExists', ...
        'The requested output directory already exists.');
end
if isempty(io)
    assert(cfg.live_enabled&&cfg.outer_preflight_pass,'m600delivery:LivePreflight', ...
        'A fresh outer live preflight is required before constructing live I/O.');
    io=m600check.makeM600CopterSimIo(cfg);
end

hostState=gpenmpcHil.newThreeMethodHostAdapterState( ...
    "PX4_NATIVE_CONTROL",task.reference.payload_kg(1),4,"LIVE",false,true);
outerConfig=struct('outer_period_s',.30,'time_tolerance_s',1e-9);
events=struct('time_s',{},'task_time_s',{},'kind',{},'detail',{});
parameterActions=struct('name',{},'target',{},'purpose',{},'attempted',{}, ...
    'write_returned',{},'readback_verified',{},'raw_bits_hex',{},'error',{});
counts=struct('arm_requests',0,'arm_transitions',0,'offboard_requests',0, ...
    'land_requests',0,'standard_disarm_requests',0,'force_disarm_requests',0, ...
    'payload_update_requests',0,'payload_update_acks',0, ...
    'mapping_apply_attempts',0,'mapping_apply_verified',0, ...
    'mapping_rollback_attempts',0,'mapping_rollback_verified',0, ...
    'environment_frames',0,'environment_board_time_clamps',0, ...
    'environment_board_time_max_raw_regression_s',0, ...
    'paused_command_clock_hold_updates',0,'paused_command_waits',0, ...
    'paused_command_max_wait_s',0, ...
    'safety_only_command_waits',0,'safety_only_command_observations',0, ...
    'setpoint_frames',0,'physical_output_actions',0);
mapped=strings(0,1);latest=[];lastArmed=NaN;lastHeartbeat=-Inf;
lastEstimate=-Inf;lastTruth=-Inf;threeStream=[];lastReferenceTime=-Inf;
boardOrigin=[NaN NaN NaN];truthOrigin=[NaN NaN NaN];coordinateOffset=[NaN NaN NaN];
currentRelativeReference=groundReference(timeline);currentBoardReference=[];
payloadGeneration=0;releaseGeneration=0;payloadKg=cfg.delivery_environment_contract.initial_payload_kg;
initialPrearmAdmissionLatched=false;
environmentGeneration=0;lastEnvironmentSource=-Inf;lastEnvironmentReceipt=[];
lastEnvironmentBoardSource=-Inf;
landAck=false;landAckRejected=false;lastLandRequest=-Inf;
formalStart=NaN;formalEnd=NaN;failure='';phase='PREARM';
finalized=false;socketsClosed=false;safeFinal=false;hardwareFinal=false;
firstFailure=[];sequence=uint64(0);guard=onCleanup(@finish); %#ok<NASGU>
finalizerMissionClockBefore=[];finalizerMissionClockAfter=[];

try
    % Establish the current model/clock/board state without beginning the
    % delivery-environment session. Parameter reads and typed, reversible
    % setup operations are blocking serial transactions and can legitimately
    % exceed the model's 250 ms ACTIVE environment freshness bound. The first
    % nonzero BEGIN frame is therefore sent only after all disarmed setup is
    % complete, immediately before the continuously serviced prestream.
    t0=io.now();
    while io.now()-t0<cfg.preflight_deadline_s
        observe(false);sendHeartbeatIfDue();
        if ready(latest),break;end
        io.sleep(cfg.poll_period_s);
    end
    assert(ready(latest),'m600delivery:Preflight', ...
        'Fresh model/clock/PX4 estimate and disarmed dual-ground state were not established.');
    boardOrigin=latest.estimate.position_ned_m;
    truthOrigin=latest.truth.position_ned_m;
    coordinateOffset=truthOrigin-boardOrigin;
    assert(norm(coordinateOffset)<=cfg.maximum_initial_origin_offset_m, ...
        'm600delivery:Origin','Initial PX4/plant origin translation exceeds the frozen bound.');
    guards={'SYS_HITL',1;'SYS_AUTOSTART',6001;'MAV_TYPE',13;'CA_ROTOR_COUNT',6;'RA_CTRL_MODE',0};
    for k=1:size(guards,1)
        p=io.readParameter(guards{k,1});
        assert(p.mav_type==6&&p.decoded==guards{k,2},'m600delivery:Profile',guards{k,1});
    end
    for k=1:16
        p=io.readParameter(sprintf('HIL_ACT_FUNC%d',k));
        assert(p.mav_type==6&&p.decoded==0,'m600delivery:PreMapping', ...
            'A virtual HIL mapping was nonzero before setup.');
    end
    for k=1:8
        p=io.readParameter(sprintf('PWM_MAIN_FUNC%d',k));
        assert(p.mav_type==6&&p.decoded==0,'m600delivery:PhysicalMapping', ...
            'A physical PWM mapping was nonzero before setup.');
    end
    if ~isempty(tuningCheck)
        parameterUnionOriginal=verifyDeliveryUnion(io.readParameter, ...
            cfg.native_hover_tuning,cfg.temporary_allocator_geometry,'ORIGINAL');
        assert(parameterUnionOriginal.passed,'m600delivery:OriginalParameterUnion','%s', ...
            parameterUnionOriginal.failure);
    end
    if ~isempty(geometryCheck)
        g=cfg.temporary_allocator_geometry.unchanged_guard_entries;
        for k=1:numel(g)
            p=io.readParameter(g(k).name);
            assert(p.mav_type==g(k).mav_type&&strcmpi(p.raw_bits_hex,g(k).raw_bits_hex), ...
                'm600delivery:AllocatorGuard','%s',g(k).name);
        end
        geometryTouched=true;geometryRestored=false;
        geometryApply=m600check.transferNativeAllocatorGeometry(io, ...
            cfg.temporary_allocator_geometry.entries,'APPLY',@geometryMutationGuard);
        event('ALLOCATOR_GEOMETRY_APPLY',sprintf('attempts=%d verified=%d pass=%d', ...
            geometryApply.setter_invocation_attempt_count, ...
            geometryApply.ack_and_readback_verified_count,geometryApply.passed));
        assert(geometryApply.passed,'m600delivery:AllocatorGeometryApply', ...
            'Temporary allocator geometry apply/readback did not pass.');
    end
    if ~isempty(tuningCheck)
        tuningTouched=true;tuningRestored=false;
        tuningApply=m600check.transferNativeHoverTuning(io, ...
            cfg.native_hover_tuning.entries,'APPLY',@geometryMutationGuard);
        event('NATIVE_HOVER_TUNING_APPLY',sprintf('attempts=%d verified=%d pass=%d', ...
            tuningApply.setter_invocation_attempt_count, ...
            tuningApply.ack_and_readback_verified_count,tuningApply.passed));
        assert(tuningApply.passed&&tuningApply.final_identity_verified, ...
            'm600delivery:NativeHoverTuningApply');
        parameterUnionApplied=verifyDeliveryUnion(io.readParameter, ...
            cfg.native_hover_tuning,cfg.temporary_allocator_geometry,'APPLIED');
        assert(parameterUnionApplied.passed,'m600delivery:AppliedParameterUnion','%s', ...
            parameterUnionApplied.failure);
    end
    for k=1:6
        observe(false);assert(ready(latest),'m600delivery:PrewriteState', ...
            'Board/model state was not safe before a virtual mapping write.');
        name=sprintf('HIL_ACT_FUNC%d',k);mapped(end+1,1)=string(name); %#ok<AGROW>
        writeMapping(name,100+k,'APPLY');
    end

    % Prestream ground reference and environment before requesting Offboard.
    % Service the environment during every subsequent blocking command wait.
    event('DELIVERY_ENVIRONMENT_BEGIN_PENDING','first nonzero frame follows');
    t0=io.now();
    while io.now()-t0<cfg.prestream_s
        observe(false);sendReferenceAndEnvironment(true);io.sleep(cfg.poll_period_s);
    end
    assert(ready(latest),'m600delivery:PreOffboardState', ...
        'Board/model state was not safe after the continuous prestream.');
    requestAccepted(176,[1,6,0,0,0,0,0],'OFFBOARD_INITIAL');
    assert(waitFor(@(s)stateFresh(s)&&s.main_mode==6,cfg.mode_timeout_s), ...
        'm600delivery:InitialOffboard','Initial Offboard state not confirmed.');

    missionStart=io.now();
    while io.now()-missionStart<cfg.delivery_mission.mission_timeout_s
        observe(false);
        assertOperational();
        observation=makeObservation();
        request=struct('schema','GPENMPC_THREE_METHOD_HOST_STEP_REQUEST_V1', ...
            'sequence',sequence,'simulation_time_s',io.now(), ...
            'method_id','PX4_NATIVE_CONTROL','observation',observation);
        [hostState,response,wrapperAudit]=canonical_delivery_host_step_checked( ...
            hostState,request,deliveryConfig,relaunchConfig,outerConfig,timeline);
        if wrapperAudit.precondition_override
            error('m600delivery:SafetyWrapper','%s',wrapperAudit.reason);
        end
        if response.fail_closed
            error('m600delivery:Lifecycle','%s',string(response.failure_code));
        end
        phase=char(response.lifecycle_diagnostic.delivery_state);
        if string(response.lifecycle_event)~="NONE"
            event(char(response.lifecycle_event),'');
        end
        currentRelativeReference=response.board_reference;
        if string(response.lifecycle_event)=="ARMED_FOR_TAKEOFF" && ~isfinite(formalStart)
            formalStart=io.now();event('FORMAL_BEGIN','initial arm transition');
        end

        c=response.lifecycle_command;
        if c.allow_payload_update
            next=payloadGeneration+1;
            assert(next<=4,'m600delivery:PayloadGeneration', ...
                'The payload generation exceeded the four frozen services.');
            payloadGeneration=next;releaseGeneration=next;
            payloadKg=cfg.delivery_environment_contract.payload_by_generation_kg(next+1);
            counts.payload_update_requests=counts.payload_update_requests+1;
            event('PAYLOAD_UPDATE_FRAME_ISSUED',sprintf('generation=%d payload=%.9g',next,payloadKg));
        end
        sendReferenceAndEnvironment(~c.advance_reference);
        if c.request_land
            landAck=false;landAckRejected=false;lastLandRequest=io.now();
            counts.land_requests=counts.land_requests+1;
            ack=requestRaw(176,[1,4,6,0,0,0,0],'AUTO_LAND');
            landAck=ack.result==0;landAckRejected=~landAck;
        end
        if c.request_disarm
            counts.standard_disarm_requests=counts.standard_disarm_requests+1;
            requestAccepted(400,[0,0,0,0,0,0,0],'STANDARD_DISARM');
        end
        if c.request_offboard
            counts.offboard_requests=counts.offboard_requests+1;
            requestAccepted(176,[1,6,0,0,0,0,0],'OFFBOARD_REENTRY');
        end
        if c.request_arm
            if string(hostState.coordinator.delivery.name)=="PREARM"
                assert(ready(latest),'m600delivery:InitialArmWithoutAdmission', ...
                    'The initial logical arm request lacked fresh admission evidence.');
                initialPrearmAdmissionLatched=true;
            end
            counts.arm_requests=counts.arm_requests+1;
            requestAccepted(400,[1,0,0,0,0,0,0],'LOGICAL_ARM');
        end
        if response.task_complete
            formalEnd=io.now();event('FORMAL_END','task complete after final native landing');break
        end
        sequence=sequence+1;io.sleep(cfg.poll_period_s);
    end
    assert(isfinite(formalStart)&&isfinite(formalEnd)&&hostState.coordinator.delivery.task_complete, ...
        'm600delivery:MissionTimeout','Canonical delivery mission did not complete in its frozen wall bound.');
catch problem
    failure=[problem.identifier ': ' problem.message];
    firstFailure=struct('time_s',io.now(),'phase',phase,'identifier',problem.identifier, ...
        'message',problem.message,'snapshot',latest);
    event('EXCEPTION',failure);
end
finish();clear guard

evaluation=struct('passed',false,'status','FORMAL_WINDOW_NOT_COMPLETED');
if isfinite(formalStart)&&isfinite(formalEnd)&&~isempty(threeStream)
    policy=cfg.evaluation_policy;policy.clock_id='MATLAB_MONOTONIC_MAPPED_SOURCE_TIME';
    policy.clock_mapping_evidence='PX4_TIMESYNC_AND_COPTERSIM_SOURCE_CLOCK';
    policy.window_s=[formalStart formalEnd];
    evaluation=m600check.evaluateThreeStream(threeStream,policy);
end
delivery=hostState.coordinator.delivery;
completed=isfinite(formalEnd)&&delivery.task_complete&&delivery.completed_ground_services==4&& ...
    delivery.completed_relaunches==4&&counts.payload_update_acks==4;
status='INFRASTRUCTURE_OR_SAFETY_TERMINATION_NOT_COMPLETED';
if completed&&safeFinal&&isempty(failure)
    status='VALID_COMPLETED_CANONICAL_CAMBRIDGE_FOUR_GROUND_DELIVERY_HIL';
end
result=struct('schema','M600_MATLAB_CANONICAL_DELIVERY_HIL_V1','status',status, ...
    'task_completed',completed,'formal_entered',isfinite(formalStart), ...
    'formal_completed',isfinite(formalEnd),'failure',failure,'first_failure',firstFailure, ...
    'delivery_state',delivery,'counts',counts,'events',events,'evaluation',evaluation, ...
    'parameter_actions',parameterActions,'three_stream_log',threeStream, ...
    'allocator_geometry_contract',geometryCheck,'allocator_geometry_apply',geometryApply, ...
    'allocator_geometry_restore',geometryRestore,'allocator_geometry_restored',geometryRestored, ...
    'native_hover_tuning_contract',tuningCheck,'native_hover_tuning_apply',tuningApply, ...
    'native_hover_tuning_restore',tuningRestore,'native_hover_tuning_restored',tuningRestored, ...
    'native_parameter_union_original',parameterUnionOriginal, ...
    'native_parameter_union_applied',parameterUnionApplied, ...
    'native_parameter_union_restored',parameterUnionRestored, ...
    'native_original138_restored',parameterUnionSafe, ...
    'task_path',cfg.delivery_mission.task_path,'task_sha256',cfg.delivery_mission.task_sha256, ...
    'fixed_prearm_origin_translation_m',coordinateOffset, ...
    'transport_evidence',io.evidence(),'fresh_final_safe_state',safeFinal, ...
    'fresh_final_hardware_safe_state',hardwareFinal,'matlab_sockets_closed',socketsClosed, ...
    'physical_output_actions',counts.physical_output_actions, ...
    'safety_finalizer_mission_clock_unchanged', ...
        isequaln(finalizerMissionClockBefore,finalizerMissionClockAfter), ...
    'python_hil_driver',false,'controller','PX4_NATIVE_POSITION_ATTITUDE_RATE_ALLOCATOR', ...
    'delivery_semantics','NATIVE_LAND_DUAL_GROUND_STANDARD_DISARM_8S_UNLOAD_REARM');
if cfg.write_artifacts
    mkdir(outputDir);save(fullfile(outputDir,'RAW_MATLAB_DELIVERY_HIL.mat'),'result','-v7.3');
    compact=result;compact.three_stream_log=[];compact.transport_evidence=[];
    if isfield(compact.evaluation,'rows'),compact.evaluation.rows=[];end
    if isfield(compact.evaluation,'raw_streams'),compact.evaluation.raw_streams=[];end
    writeJson(fullfile(outputDir,'RESULT.json'),compact);
end

    function observe(requireOperational)
        latest=io.snapshot();
        if stateFresh(latest)
            if lastArmed==0&&latest.armed==1,counts.arm_transitions=counts.arm_transitions+1;end
            lastArmed=latest.armed;
        end
        if ~isempty(latest.estimate)&&latest.estimate.raw_source_time_s>lastEstimate&&all(isfinite(boardOrigin))
            s=latest.estimate.sample;s.position_ned_m=s.position_ned_m+coordinateOffset;
            threeStream=m600check.appendThreeStreamSample(threeStream,'estimate',s);
            lastEstimate=latest.estimate.raw_source_time_s;
        end
        if ~isempty(latest.truth)&&latest.truth.raw_source_time_s>lastTruth
            s=latest.truth.sample;threeStream=m600check.appendThreeStreamSample(threeStream,'truth',s);
            lastTruth=latest.truth.raw_source_time_s;
        end
        if requireOperational,assertOperational();end
    end
    function yes=originsAvailable()
        yes=stateFresh(latest)&&streamFresh(latest)&&latest.clock_valid;
        if yes&&~all(isfinite(boardOrigin))
            boardOrigin=latest.estimate.position_ned_m;truthOrigin=latest.truth.position_ned_m;
            coordinateOffset=truthOrigin-boardOrigin;
        end
    end
    function yes=modelClockAvailable(s)
        yes=isstruct(s)&&isfield(s,'model_diagnostic')&&isstruct(s.model_diagnostic)&& ...
            isfield(s.model_diagnostic,'decoded')&&isstruct(s.model_diagnostic.decoded)&& ...
            isfield(s.model_diagnostic.decoded,'sim_time_s')&&isfinite(s.model_diagnostic.decoded.sim_time_s);
    end
    function yes=stateFresh(s)
        yes=isstruct(s)&&all(isfield(s,{'armed','landed_state','heartbeat_rx_s','extended_rx_s'}))&& ...
            all(isfinite([s.armed,s.landed_state,s.heartbeat_rx_s,s.extended_rx_s]))&& ...
            io.now()>=s.heartbeat_rx_s&&io.now()>=s.extended_rx_s&& ...
            io.now()-s.heartbeat_rx_s<=cfg.heartbeat_max_age_s&& ...
            io.now()-s.extended_rx_s<=cfg.landed_max_age_s;
    end
    function yes=streamFresh(s)
        yes=isstruct(s)&&~isempty(s.estimate)&&~isempty(s.truth)&& ...
            all(isfinite([s.estimate.position_ned_m,s.estimate.velocity_ned_mps, ...
            s.truth.position_ned_m,s.truth.velocity_ned_mps,s.estimate.rx_s,s.truth.rx_s]))&& ...
            io.now()-s.estimate.rx_s<=cfg.state_max_age_s&&io.now()-s.truth.rx_s<=cfg.state_max_age_s;
    end
    function yes=modelReady(s)
        yes=isstruct(s)&&isfield(s,'model_ready')&&isequal(s.model_ready,true);
    end
    function yes=plantGround(s)
        yes=modelReady(s)&&isfield(s.model_diagnostic,'decoded')&& ...
            isfield(s.model_diagnostic.decoded,'ground_confirmed')&& ...
            isequal(s.model_diagnostic.decoded.ground_confirmed,true);
    end
    function yes=nativeFresh(s)
        yes=~isfield(cfg,'require_current_native_telemetry')||~cfg.require_current_native_telemetry|| ...
            (isfield(s,'native_hover_telemetry')&&isstruct(s.native_hover_telemetry)&& ...
            isfield(s.native_hover_telemetry,'passed')&&s.native_hover_telemetry.passed);
    end
    function yes=ready(s)
        yes=modelReady(s)&&plantGround(s)&&stateFresh(s)&&streamFresh(s)&& ...
            s.clock_valid&&nativeFresh(s)&&s.armed==0&&s.landed_state==1;
    end
    function assertOperational()
        assert(modelReady(latest),'m600delivery:Model','M600 delivery model not ready.');
        assert(stateFresh(latest)&&streamFresh(latest)&&latest.clock_valid, ...
            'm600delivery:Telemetry','PX4/model telemetry or clock stale.');
        assert(nativeFresh(latest),'m600delivery:NativeTelemetry','Native estimator stream invalid.');
        assert(norm(latest.truth.velocity_ned_mps)<=cfg.abort_truth_speed_mps, ...
            'm600delivery:TruthSpeedAbort', ...
            'Plant speed exceeded the frozen safety-interface abort bound.');
        assert(norm(latest.estimate.position_ned_m+coordinateOffset-latest.truth.position_ned_m)<= ...
            cfg.abort_estimator_gap_m,'m600delivery:EstimatorGapAbort', ...
            'Estimator-to-plant separation exceeded the frozen abort bound.');
    end
    function observation=makeObservation()
        [nominal,~]=gpenmpcHil.cambridgeMissionReference(hostState.mission_clock,timeline,io.now());
        estimateRel=latest.estimate.position_ned_m-boardOrigin;
        plantRel=latest.truth.position_ned_m-truthOrigin;
        payloadAck=false;payloadAfter=payloadKg;
        if isfield(latest,'delivery_environment')&&latest.delivery_environment.present&& ...
                latest.delivery_environment.mass_ack_valid
            q=latest.delivery_environment.decoded.environment_extension;
            payloadAck=q.applied_payload_generation==payloadGeneration&& ...
                abs(q.actual_payload_kg-payloadKg)<1e-12;
            if payloadAck,payloadAfter=q.actual_payload_kg;end
        end
        payloadAckForLifecycle=payloadAck&& ...
            hostState.coordinator.delivery.payload_update_requested;
        if payloadAckForLifecycle
            counts.payload_update_acks=max(counts.payload_update_acks,payloadGeneration);
        end
        prearmReady=ready(latest);
        if string(hostState.coordinator.delivery.name)=="PREARM" && latest.armed==1
            prearmReady=initialPrearmAdmissionLatched;
        end
        observation=struct('estimated_position_ned_m',estimateRel, ...
            'estimated_velocity_ned_mps',latest.estimate.velocity_ned_mps, ...
            'plant_position_ned_m',plantRel,'prearm_ready',prearmReady, ...
            'armed',latest.armed==1,'px4_landed',latest.landed_state==1, ...
            'plant_contact',plantGround(latest), ...
            'altitude_agl_m',max(-plantRel(3),0), ...
            'horizontal_error_m',norm(estimateRel(1:2)-nominal.position_ned_m(1:2)), ...
            'land_ack',landAck&&lastLandRequest>=0,'land_ack_rejected',landAckRejected, ...
            'px4_auto_land',latest.main_mode==4&&latest.sub_mode==6, ...
            'px4_offboard',latest.main_mode==6, ...
            'payload_update_ack',payloadAckForLifecycle,'payload_after_service_kg',payloadAfter, ...
            'at_return_base',false);
    end
    function sendReferenceAndEnvironment(paused)
        if ~originsAvailable()||~modelClockAvailable(latest),return;end
        boardRef=shiftReference(currentRelativeReference,boardOrigin);
        io.sendSetpoint(boardRef);counts.setpoint_frames=counts.setpoint_frames+1;
        currentBoardReference=boardRef;
        now=io.now();if now>lastReferenceTime
            truthRef=shiftReference(currentRelativeReference,truthOrigin);
            s=struct('clock_id','MATLAB_MONOTONIC_MAPPED_SOURCE_TIME','time_s',now, ...
                'received_at_s',now,'position_ned_m',truthRef.position_ned_m, ...
                'velocity_ned_mps',truthRef.velocity_ned_mps);
            threeStream=m600check.appendThreeStreamSample(threeStream,'reference',s);
            lastReferenceTime=now;
        end
        simS=latest.model_diagnostic.decoded.sim_time_s;
        if simS<=lastEnvironmentSource,sendHeartbeatIfDue();return;end
        snapshotNow=double(latest.now_s);
        heartbeatFresh=isfinite(latest.heartbeat_rx_s)&&snapshotNow>=latest.heartbeat_rx_s&& ...
            snapshotNow-latest.heartbeat_rx_s<=cfg.heartbeat_max_age_s;
        extendedStateFresh=isfinite(latest.extended_rx_s)&&snapshotNow>=latest.extended_rx_s&& ...
            snapshotNow-latest.extended_rx_s<=cfg.landed_max_age_s;
        assert(heartbeatFresh&&extendedStateFresh,'m600delivery:EnvironmentBoardEvidenceStale', ...
            'Environment frame refused stale board evidence (heartbeat=%d extended=%d).', ...
            heartbeatFresh,extendedStateFresh);
        environmentGeneration=environmentGeneration+1;
        v=gpenmpcTaskIo.makePlantEnvironmentFrameFromSnapshot(latest,currentRelativeReference, ...
            payloadKg,payloadGeneration,releaseGeneration,environmentGeneration, ...
            max(0,hostState.mission_clock.task_time_s),logical(paused), ...
            string(hostState.coordinator.delivery.name), ...
            cfg.delivery_environment_contract.expected_session_token,true, ...
            heartbeatFresh,extendedStateFresh);
        % Map heartbeat and landed-state receive times into the simulator clock.
        % Asynchronous sampling can move a derived bound backwards; retain the
        % largest observed causal lower bound while checking freshness and identity.
        rawBoardSource=v.board_min_rx_io_time_s;
        if isfinite(lastEnvironmentBoardSource)&&rawBoardSource<lastEnvironmentBoardSource
            counts.environment_board_time_clamps=counts.environment_board_time_clamps+1;
            counts.environment_board_time_max_raw_regression_s=max( ...
                counts.environment_board_time_max_raw_regression_s, ...
                lastEnvironmentBoardSource-rawBoardSource);
            v.board_min_rx_io_time_s=lastEnvironmentBoardSource;
        end
        lastEnvironmentBoardSource=v.board_min_rx_io_time_s;
        lastEnvironmentReceipt=io.sendPlantEnvironment(v); %#ok<NASGU>
        lastEnvironmentSource=simS;counts.environment_frames=counts.environment_frames+1;
        sendHeartbeatIfDue();
    end
    function sendHeartbeatIfDue()
        if io.now()-lastHeartbeat>=cfg.heartbeat_period_s
            io.sendHeartbeat();lastHeartbeat=io.now();
        end
    end
    function a=requestRaw(command,params,label)
        sent=io.now();io.requestCommand(command,params);event([label '_REQUEST'],'');
        a=[];t=io.now();
        while io.now()-t<cfg.command_timeout_s
            observe(false);sendReferenceAndEnvironment(true);
            if finalized
                % Keep task phase and clock frozen during native LAND/disarm recovery.
                counts.safety_only_command_observations=counts.safety_only_command_observations+1;
            else
                holdMissionClockDuringCommand(label);
            end
            a=io.commandAck(command,sent);if ~isempty(a),break;end
            io.sleep(cfg.poll_period_s);
        end
        if finalized
            counts.safety_only_command_waits=counts.safety_only_command_waits+1;
            event('SAFETY_ONLY_COMMAND_WAIT_NO_TASK_CLOCK_UPDATE', ...
                sprintf('%s wait=%.9g',label,io.now()-sent));
        elseif isfinite(hostState.mission_clock.last_simulation_time_s)
            counts.paused_command_waits=counts.paused_command_waits+1;
            counts.paused_command_max_wait_s=max(counts.paused_command_max_wait_s,io.now()-sent);
            event('PAUSED_COMMAND_WAIT_ACCOUNTED',sprintf('%s wait=%.9g',label,io.now()-sent));
        end
        assert(~isempty(a),'m600delivery:CommandAckMissing','Missing ACK for %s',label);
        event([label '_ACK'],sprintf('result=%g',a.result));
    end
    function holdMissionClockDuringCommand(label)
        if ~isfinite(hostState.mission_clock.last_simulation_time_s),return;end
        pausedStates=["PREARM","NATIVE_LAND","GROUND_CONFIRM", ...
            "SERVICE_GROUNDED_DISARMED","WAIT_OFFBOARD","REARM_ADMISSION", ...
            "FINAL_NATIVE_LAND","COMPLETE","FAIL_CLOSED"];
        stateName=string(hostState.coordinator.delivery.name);
        assert(any(stateName==pausedStates),'m600delivery:ActiveFlightCommandWait', ...
            'Blocking command wait %s occurred while the task reference was active.',label);
        % Record observations during requestRaw as held mission-clock samples
        % so ACK latency is excluded from the next active reference step.
        hostState.mission_clock.last_simulation_time_s=io.now();
        counts.paused_command_clock_hold_updates=counts.paused_command_clock_hold_updates+1;
    end
    function requestAccepted(command,params,label)
        a=requestRaw(command,params,label);
        assert(a.result==0,'m600delivery:CommandRejected','%s result %g',label,a.result);
    end
    function yes=waitFor(predicate,deadline)
        yes=false;t=io.now();
        while io.now()-t<deadline
            observe(false);sendReferenceAndEnvironment(true);
            if predicate(latest),yes=true;return;end
            io.sleep(cfg.poll_period_s);
        end
    end
    function writeMapping(name,target,purpose)
        a=struct('name',name,'target',target,'purpose',purpose,'attempted',true, ...
            'write_returned',false,'readback_verified',false,'raw_bits_hex','','error','');
        parameterActions(end+1)=a;idx=numel(parameterActions);
        if strcmp(purpose,'APPLY'),counts.mapping_apply_attempts=counts.mapping_apply_attempts+1;
        else,counts.mapping_rollback_attempts=counts.mapping_rollback_attempts+1;end
        try
            p=io.setIntegerParameter(name,target);parameterActions(idx).write_returned=true;
            assert(p.mav_type==6&&p.decoded==target,'m600delivery:MappingReadback', ...
                'Virtual mapping typed readback did not match the requested value.');
            parameterActions(idx).readback_verified=true;parameterActions(idx).raw_bits_hex=p.raw_bits_hex;
            if strcmp(purpose,'APPLY'),counts.mapping_apply_verified=counts.mapping_apply_verified+1;
            else,counts.mapping_rollback_verified=counts.mapping_rollback_verified+1;end
        catch problem
            parameterActions(idx).error=problem.message;rethrow(problem);
        end
    end
    function event(kind,detail)
        t=NaN;if isstruct(hostState)&&isfield(hostState,'mission_clock'),t=hostState.mission_clock.task_time_s;end
        events(end+1)=struct('time_s',io.now(),'task_time_s',t,'kind',kind,'detail',detail); %#ok<AGROW>
    end
    function finish()
        if finalized,return;end
        finalized=true;
        finalizerMissionClockBefore=hostState.mission_clock;
        try
            observe(false);
            if stateFresh(latest)&&latest.armed==1
                if ~(latest.main_mode==4&&latest.sub_mode==6)
                    counts.land_requests=counts.land_requests+1;
                    requestAccepted(176,[1,4,6,0,0,0,0],'FINALIZER_AUTO_LAND');
                end
                waitFor(@(s)stateFresh(s)&&s.landed_state==1,cfg.land_timeout_s);
                observe(false);
                if stateFresh(latest)&&latest.landed_state==1&&latest.armed==1
                    counts.standard_disarm_requests=counts.standard_disarm_requests+1;
                    requestAccepted(400,[0,0,0,0,0,0,0],'FINALIZER_STANDARD_DISARM');
                end
                waitFor(@(s)stateFresh(s)&&s.armed==0&&s.landed_state==1,cfg.disarm_timeout_s);
            end
        catch problem,event('FINALIZER_TERMINATION_ERROR',problem.message);end
        for k=numel(mapped):-1:1
            try
                observe(false);assert(stateFresh(latest)&&latest.armed==0&&latest.landed_state==1, ...
                    'm600delivery:UnsafeRollback', ...
                    'Virtual mappings may only be rolled back while freshly disarmed and landed.');
                writeMapping(char(mapped(k)),0,'ROLLBACK');
            catch problem,event('MAPPING_ROLLBACK_ERROR',problem.message);end
        end
        if geometryTouched
            try
                geometryRestore=m600check.transferNativeAllocatorGeometry(io, ...
                    cfg.temporary_allocator_geometry.entries,'RESTORE',@geometryMutationGuard);
                geometryRestored=geometryRestore.passed;
                event('ALLOCATOR_GEOMETRY_RESTORE',sprintf('attempts=%d verified=%d pass=%d', ...
                    geometryRestore.setter_invocation_attempt_count, ...
                    geometryRestore.ack_and_readback_verified_count,geometryRestore.passed));
            catch problem
                geometryRestored=false;event('ALLOCATOR_GEOMETRY_RESTORE_ERROR',problem.message);
            end
        end
        if tuningTouched
            try
                tuningRestore=m600check.transferNativeHoverTuning(io, ...
                    cfg.native_hover_tuning.entries,'RESTORE',@geometryMutationGuard);
                tuningRestored=tuningRestore.passed&&tuningRestore.final_identity_verified;
                event('NATIVE_HOVER_TUNING_RESTORE',sprintf('attempts=%d verified=%d pass=%d', ...
                    tuningRestore.setter_invocation_attempt_count, ...
                    tuningRestore.ack_and_readback_verified_count,tuningRestore.passed));
            catch problem
                tuningRestored=false;event('NATIVE_HOVER_TUNING_RESTORE_ERROR',problem.message);
            end
        end
        if ~isempty(tuningCheck)
            try
                parameterUnionRestored=verifyDeliveryUnion(io.readParameter, ...
                    cfg.native_hover_tuning,cfg.temporary_allocator_geometry,'RESTORED');
                parameterUnionSafe=parameterUnionRestored.passed;
                if ~parameterUnionSafe,event('NATIVE_PARAMETER_UNION_RESTORE_MISMATCH', ...
                        parameterUnionRestored.failure);end
            catch problem
                parameterUnionSafe=false;event('NATIVE_PARAMETER_UNION_RESTORE_ERROR',problem.message);
            end
        end
        try
            observe(false);hardwareFinal=stateFresh(latest)&&latest.armed==0&&latest.landed_state==1;
            for k=1:16,p=io.readParameter(sprintf('HIL_ACT_FUNC%d',k));hardwareFinal=hardwareFinal&&p.mav_type==6&&p.decoded==0;end
            for k=1:8,p=io.readParameter(sprintf('PWM_MAIN_FUNC%d',k));hardwareFinal=hardwareFinal&&p.mav_type==6&&p.decoded==0;end
            safeFinal=hardwareFinal&&plantGround(latest)&&geometryRestored&& ...
                tuningRestored&&parameterUnionSafe;
        catch problem,event('FINAL_READBACK_ERROR',problem.message);end
        try,socketsClosed=io.close();catch problem,event('SOCKET_CLOSE_ERROR',problem.message);end
        finalizerMissionClockAfter=hostState.mission_clock;
    end
    function yes=geometryMutationGuard()
        observe(false);yes=stateFresh(latest)&&latest.armed==0&&latest.landed_state==1;
        if ~yes,return;end
        for q=1:16
            p=io.readParameter(sprintf('HIL_ACT_FUNC%d',q));
            yes=yes&&p.mav_type==6&&p.decoded==0&&strcmpi(p.raw_bits_hex,'00000000');
        end
        for q=1:8
            p=io.readParameter(sprintf('PWM_MAIN_FUNC%d',q));
            yes=yes&&p.mav_type==6&&p.decoded==0&&strcmpi(p.raw_bits_hex,'00000000');
        end
        p=io.readParameter('RA_CTRL_MODE');yes=yes&&p.mav_type==6&&p.decoded==0;
        observe(false);yes=logical(yes&&stateFresh(latest)&&latest.armed==0&&latest.landed_state==1);
    end
    function receipt=verifyDeliveryUnion(reader,tuning,geometry,unionPhase)
        if strcmp(tuning.schema,'TEMPORARY_M600_CANONICAL_DELIVERY_NATIVE_TUNING_V1')
            receipt=verify_m600_delivery_parameter_union(reader,tuning,geometry,unionPhase);
        else
            receipt=m600check.verifyNativeHoverParameterUnion(reader,tuning,geometry,unionPhase);
        end
    end
end

function [task,timeline,d,r]=validateAndLoad(c)
assert(isfield(c,'delivery_mission')&&isfield(c,'delivery_environment_contract'), ...
    'm600delivery:Config','The canonical delivery configuration is incomplete.');m=c.delivery_mission;
assert(strcmpi(m600check.fileSha256(m.task_path),m.task_sha256), ...
    'm600delivery:TaskIdentity','The canonical task content hash does not match.');
assert(strcmpi(m600check.fileSha256(m.delivery_lifecycle_config),m.delivery_lifecycle_sha256), ...
    'm600delivery:LifecycleIdentity','The delivery lifecycle content hash does not match.');
assert(strcmpi(m600check.fileSha256(m.relaunch_config),m.relaunch_config_sha256), ...
    'm600delivery:RelaunchIdentity','The relaunch configuration content hash does not match.');
addpath(m.gpenmpc_parent_matlab_root,'-begin');
x=load(m.task_path,'physicalTask','timeline');task=x.physicalTask;timeline=x.timeline;
d=jsondecode(fileread(m.delivery_lifecycle_config));r=jsondecode(fileread(m.relaunch_config));
assert(d.required_delivery_count==4&&d.service_dwell_s==8&& ...
    numel(c.delivery_environment_contract.payload_by_generation_kg)==5, ...
    'm600delivery:DeliveryContract','The frozen four-delivery lifecycle contract is inconsistent.');
end

function r=groundReference(timeline)
r=gpenmpcHil.sampleCambridgePhysicalMission(timeline,0);r.position_ned_m(3)=0;
r.velocity_ned_mps=zeros(1,3);r.acceleration_ned_mps2=zeros(1,3);
r.jerk_ned_mps3=zeros(1,3);r.yaw_rad=0;r.estimated_wind_ned_mps=zeros(1,3);
end

function r=shiftReference(r,origin)
r.position_ned_m=double(r.position_ned_m)+double(origin);
end

function writeJson(pathValue,value)
temporary=string(pathValue)+'.tmp';fid=fopen(temporary,'w','n','UTF-8');assert(fid>=0);
guard=onCleanup(@()fclose(fid));fprintf(fid,'%s\n',jsonencode(value,PrettyPrint=true));clear guard
movefile(temporary,pathValue,'f');
end
