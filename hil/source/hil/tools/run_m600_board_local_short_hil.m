function [result,safetyOwner]=run_m600_board_local_short_hil(outputDir,cfg,io,resources)
% Run a board-local control session with externally prepared IO resources.
% Supports a bounded trajectory trial or a user-ended manual reference.
% The caller owns simulator/getter startup, shutdown and parameter recovery.
arguments
    outputDir (1,1) string
    cfg (1,1) struct
    io (1,1) struct
    resources (1,1) struct
end
build=fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(build,'host_runtime'),fullfile(build,'matlab_validation'), ...
    fullfile(build,'m600_coptersim','matlab_validation'),fullfile(build,'tools'));
assert(~isfolder(outputDir)&&~isfile(outputDir),'gpenmpcShort:OutputExists','Choose an unused output path.');
assert(cfg.live_enabled&&cfg.outer_preflight_pass ...
    &&strcmp(cfg.canonical_exchange.runtime,'BOARD_LOCAL_FULL_INNER'),'gpenmpcShort:LocalSelection','Explicit local full-inner selection and outer preflight are required.');
needed={'assets','bundle','task_source','authorization','getter_source', ...
    'getter_open_receipt','owned_simulator_pid','challenge','applied_parameters','short_objects'};
assert(all(isfield(resources,needed))&&all(isfield(cfg,{'local_short','outer_preflight_receipt'})), ...
    'gpenmpcShort:Resources','Real existing process/getter/asset resources are required.');
c=cfg.local_short;
saveFullRaw=~isfield(c,'save_full_raw')||c.save_full_raw;
assert(saveFullRaw||isfield(c.service_cfg,'operator_reference'),'gpenmpcShort:RecordingScope');
assert(all(isfield(c,{'duration_s','poll_period_s','heartbeat_period_s', ...
    'preparation_timeout_s','getter_capacity','environment_ledger_capacity','raw_capacity','service_cfg'})) ...
    &&(ismember(c.duration_s,[20 40])||(ismember(c.duration_s,[0 120]) ...
       &&isfield(c,'component_initialization')&&c.component_initialization ...
       &&isfield(c.service_cfg,'operator_reference'))) ...
    &&c.poll_period_s>0&&c.heartbeat_period_s>0 ...
    &&c.poll_period_s<=c.heartbeat_period_s&&c.heartbeat_period_s<.1 ...
    &&c.preparation_timeout_s>0&&c.raw_capacity>=1&&c.raw_capacity==fix(c.raw_capacity) ...
    &&c.environment_ledger_capacity>=4&&c.environment_ledger_capacity<=8192 ...
    &&c.environment_ledger_capacity==fix(c.environment_ledger_capacity), ...
    'gpenmpcShort:Bounds','Use a 20/40 s trial or the USB operator component. Duration 0 ends on user request; heartbeat validity is 100 ms.');
assert(isa(c.service_cfg.source_max_age_ns,'uint64') ...
    &&isequal(c.service_cfg.source_max_age_ns,cfg.canonical_exchange.local_input_host_max_age_ns) ...
    &&isa(c.service_cfg.command_lifetime_ns,'uint64'), ...
    'gpenmpcShort:Lease','Use the configured source and command limits.');
pre=jsondecode(fileread(cfg.outer_preflight_receipt));
assert(pre.passed&&pre.COM_closed,'gpenmpcShort:SerialPreflight','Original serial preflight must pass and release COM.');
uid=exactUid(pre.uid);assert(uid>0);
assert(isfield(pre,'board_id')&&isfield(pre,'raw_identity') ...
    &&isfield(pre.raw_identity,'autopilot_version'),'gpenmpcShort:PreflightIdentity','Original handoff identity fields are required.');
preVersion=pre.raw_identity.autopilot_version;
assert(preVersion.board_version==pre.board_id&&numel(preVersion.flight_custom_version)==8, ...
    'gpenmpcShort:PreflightVersion','Original AUTOPILOT_VERSION must agree with the handoff board ID.');
expectedFlightVersion=upper(reshape(dec2hex(uint8(preVersion.flight_custom_version(:)),2).',1,[]));
a=resources.assets;bundle=resources.bundle;task=resources.task_source;
assert(numel(bundle.legs)==5&&strcmpi(task.sha256,bundle.receipt.task_sha256) ...
    &&strcmpi(task.configuration_sha256,a.binding.effective_configuration_payload_sha256), ...
    'gpenmpcShort:CanonicalAssets','Loaded task and method configuration identities differ.');
assert(isa(resources.challenge,'uint64')&&numel(resources.challenge)==2&&any(resources.challenge));
auth=resources.authorization;
assert(strcmp(auth.source,'OperatorUsbIsolationDeclaration') ...
    &&isfield(auth,'original_record')&&~isempty(auth.original_record) ...
    &&strcmpi(bytesSha(unicode2native(char(auth.original_record),'UTF-8')),auth.physical_setup_record_sha256), ...
    'gpenmpcShort:AuthorizationRecord','The physical setup record must match its declared digest.');
% Reuse the session-independent objects prepared before simulator startup.
objects=resources.short_objects;taskIdentity=task;
if isfield(taskIdentity,'cached_asset'),taskIdentity=rmfield(taskIdentity,'cached_asset');end
assert(isstruct(objects)&&isscalar(objects)&&all(isfield(objects, ...
    {'schema','local_short','environment_contract','task_identity','getter_source', ...
    'getter','environment','task_asset','dialect','getter_mex','receipt', ...
     'environment_service','environment_io','environment_service_config','method_backends'})) ...
    &&strcmp(objects.schema,'GPENMPC_LOCAL_SHORT_PURE_OBJECTS_V1') ...
    &&isequaln(objects.local_short,c)&&isequaln(objects.environment_contract,cfg.delivery_environment_contract) ...
    &&isequaln(objects.task_identity,taskIdentity)&&isequaln(objects.getter_source,resources.getter_source), ...
    'gpenmpcShort:PreparedObjects','Use the PREPARE objects with matching configuration, task and getter bindings.');
assert(isa(objects.getter,'gpenmpcNative.RflyLocalOriginalGetterBuffer') ...
    &&isa(objects.environment,'gpenmpcNative.RflyLocalEnvironmentLedger') ...
    &&isa(objects.task_asset,'gpenmpcNative.RflyLocalTaskAsset') ...
    &&isa(objects.dialect,'mavlinkdialect')&&isscalar(objects.dialect) ...
    &&isa(objects.getter_mex,'function_handle') ...
    &&isa(objects.environment_service,'gpenmpcNative.RflyCanonicalEnvironmentService'), ...
    'gpenmpcShort:PreparedObjectTypes','Use the preconstructed runtime objects.');
getter=objects.getter;environment=objects.environment;d=objects.dialect;getterMex=objects.getter_mex;
environmentConfig=struct('target_system',cfg.target_system,'target_component',cfg.target_component, ...
    'heartbeat_max_age_s',cfg.heartbeat_max_age_s,'landed_max_age_s',cfg.landed_max_age_s, ...
    'delivery_environment_contract',cfg.delivery_environment_contract);
assert(isequal(objects.environment_io,io)&&isequaln(objects.environment_service_config,environmentConfig), ...
    'gpenmpcShort:PreparedEnvironmentOwner','Reuse the exact actual IO and original environment configuration; do not substitute callbacks.');
gs=getter.status();es=environment.status();
assert(~gs.failed&&gs.accepted_getters==0&&gs.matched_sources==0&&gs.retained_getters==0 ...
    &&~es.failed&&es.sent==0&&es.received==0, ...
    'gpenmpcShort:PreparedObjectsConsumed','PREPARE buffers must be empty.');
preparedEnvStatus=objects.environment_service.status();
assert(~preparedEnvStatus.failed&&~preparedEnvStatus.closed ...
    &&preparedEnvStatus.send_attempt_count==0&&preparedEnvStatus.send_return_count==0 ...
    &&preparedEnvStatus.frame_generation==0&&strcmpi(preparedEnvStatus.canonical_task_sha256,task.sha256), ...
    'gpenmpcShort:PreparedEnvironmentConsumed','Borrow the untouched original environment service; no reset or renewed source is allowed.');
task.cached_asset=objects.task_asset;originalTask=task.cached_asset.read(task);
opened=resources.getter_open_receipt;
assert(isstruct(opened)&&all(isfield(opened,{'ring_name','status_name','peer_nonce','consumer_pid'})) ...
    &&resources.owned_simulator_pid>0,'gpenmpcShort:GetterOwner','Actual getter receipt and owned simulator PID are required.');
% Borrow the simulator resources owned by the caller.
process=System.Diagnostics.Process.GetProcessById(int32(resources.owned_simulator_pid));
assert(~process.HasExited&&strcmpi(char(process.ProcessName),'CopterSimNoUI'), ...
    'gpenmpcShort:OwnedSimulator','The outer owner must supply its running official NoUI process.');
mkdir(outputDir);
events={};getterRaw={};methodRaw={};commandRaw={};sessionRaw=struct();parameterRaw={};
service=[];stream=[];envService=[];environmentRefreshActive=false;
association=[];registered=[];identityReceipt=[];expectedIdentity=[];
latest=[];trajectory=[];referenceBinding=[];initialView=[];lastView=[];lastHeartbeat=-Inf;
lastRefreshTiming=struct();
coordinateOffset=[];
worldGround=zeros(3,1);
if isfield(cfg,'initial_world_ground_ned_m')
    worldGround=double(cfg.initial_world_ground_ned_m(:));
    assert(numel(worldGround)==3&&all(isfinite(worldGround)),'gpenmpcShort:TerrainOrigin');
end
mode='NONE';phase='PREFLIGHT';firstCommit=NaN;lastCommit=NaN;formalEnd=NaN;
pendingRecovery=struct('required',true,'completed_here',false,'source',resources.applied_parameters, ...
    'original_parameters',[],'writes',[],'receipt_validated',false, ...
    'owner','OUTER_SINGLE_SERIAL_OWNER_AFTER_NOUI_STOP_AND_GETTER_CLOSE','obligation_cleared',false);
failure=[];cleanupFailures={};finalized=false;ioClosed=false;safeGround=false;release=[];
outerMustContinueSafety=false;safetyOwner=[];
manualResetRequested=false;nextResetCheckS=-Inf;
counts=struct('arm_requests',0,'offboard_requests',0,'land_requests',0,'disarm_requests',0, ...
    'commits',0,'gp_replies',0,'control_setpoints_sent',0,'payload_updates',0);
guard=onCleanup(@finish);
try
    % The outer's single serial owner applied and independently reread the
    % exact 21 temporary recovery/mapping values BEFORE starting NoUI. Never
    % repeat those long transactions while its 256-record ring is producing.
    parameterRaw{end+1}=appliedParameters(resources.applied_parameters,cfg,uid);
    pendingRecovery.original_parameters=parameterRaw{end}.before_parameters;
    pendingRecovery.writes=parameterRaw{end}.writes;pendingRecovery.receipt_validated=true;
    preflightStarted=io.now();
    waitUntil(@ready,cfg.preflight_deadline_s,'PREFLIGHT');
    % Disable unused telemetry streams. RLS supplies atomic controller state;
    % ATTITUDE and LOCAL_POSITION support monitoring, and the model getter supplies rotor lag.
    assert(~isfield(cfg,'canonical_px4_observation'),'gpenmpcShort:StreamConsumer', ...
        'Do not disable ODOMETRY while a legacy atomic observer consumes it.');
    sessionRaw.unused_telemetry_message_ids=[31 33 74 105 141 331];
    sessionRaw.telemetry_restore='REFERENCE_APPLICATION_APPLICATION_REBOOT_RESTORES_DEFAULT_STREAM_PROFILE';
    for messageId=sessionRaw.unused_telemetry_message_ids
        command(511,[messageId -1 0 0 0 0 0],cfg.command_timeout_s,@()true, ...
            sprintf('DISABLE_UNUSED_OUTPUT_STREAM_%d',messageId));
    end
    % Keep auxiliary ATTITUDE telemetry at 20 Hz.
    % An application reboot restores the default stream profile.
    sessionRaw.attitude_telemetry_interval_us=50000;
    command(511,[30 sessionRaw.attitude_telemetry_interval_us 0 0 0 0 0], ...
        cfg.command_timeout_s,@()true,'SET_AUXILIARY_ATTITUDE_TELEMETRY_20HZ');
    waitUntil(@ready,cfg.preflight_deadline_s,'PREFLIGHT_AFTER_TELEMETRY_CONFIGURATION');
    % Reinitialize the estimator once after sensors are available and before registration.
    restartPrearmEstimator();
    % Check native estimator arming health within the preflight acquisition deadline.
    phase='PREFLIGHT_ESTIMATOR_HEALTH';
    healthRequest=[];healthNextRequest=io.now();healthNextRead=io.now();
    while true
        pump();
        assert(io.now()-preflightStarted<cfg.preflight_deadline_s, ...
            'gpenmpcShort:EstimatorHealthNotReady','Native estimator arming health did not clear within the original acquisition cap.');
        if ready()&&isempty(healthRequest)&&io.now()>=healthNextRequest
            healthRequest=io.requestPrearmHealthReport();healthNextRead=io.now()+.1;
        end
        if ~isempty(healthRequest)&&io.now()>=healthNextRead
            report=io.healthReport(healthRequest);sessionRaw.prearm_estimator_health=report;
            healthNextRead=io.now()+.25;
            if report.complete
                % Native health_component_t local_position_estimate = 1<<17.
                % Offboard availability is not expected before the local OCM starts.
                estimatorClear=bitand(uint32(report.arming_summary.error),uint32(131072))==0;
                if estimatorClear&&ready(),break;end
                healthRequest=[];healthNextRequest=io.now()+1;
            end
        end
    end
    phase='PREPARE';
    origin=double(latest.estimate.position_ned_m(:));
    assert(all(isfinite(origin))&&numel(origin)==3,'gpenmpcShort:Origin','Finite original estimator origin is required.');
    coordinateOffset=double(latest.truth.position_ned_m(:))-origin;
    assert(norm(coordinateOffset-worldGround)<=cfg.maximum_initial_origin_offset_m, ...
        'gpenmpcShort:OriginOffset','Estimator/plant residual after independent ground-frame translation exceeds the configured bound.');
    % Explicit registered estimator frame: p_task = p_NED - this origin.
    groundUp=(double(latest.estimate.position_ned_m(:))-origin).*[1;1;-1];
    % Prepare numerical coordinates before simulator startup.
    % Acquire estimator origin and plant offset during live ground checks.
    assert(isfield(resources,'initial_trajectory')&&isfield(resources,'initial_reference_binding') ...
        &&isequal(groundUp,zeros(3,1)) ...
        &&isequal(resources.initial_reference_binding.ground_position_up_m,groundUp.') ...
        &&resources.initial_reference_binding.leg_index==1 ...
        &&~resources.initial_reference_binding.ground_authority_proven, ...
        'gpenmpcShort:PreparedInitialReference','Prepared planned coordinates must match the actually registered zero-origin frame.');
    trajectory=resources.initial_trajectory;referenceBinding=resources.initial_reference_binding;
    phase='REGISTER';pump();
    request=struct('original_host_challenge',resources.challenge(:),'uid',uid, ...
        'system',uint8(cfg.target_system),'component',uint8(cfg.target_component), ...
        'host_system',uint8(255),'host_component',uint8(190), ...
        'configuration_payload_sha256',task.configuration_sha256, ...
        'local_full_inner',true,'leg_index',uint8(1),'task_sha256',task.sha256);
    prepareAction='prepare_local';referenceSha=task.sha256;
    if isfield(c.service_cfg,'operator_reference')
        assert(c.component_initialization&&c.service_cfg.component_initialization);
        prepareAction='prepare_local_rc';request.operator_reference=true;
        referenceSha=c.service_cfg.operator_reference_sha256;
    end
    tx=io.sendCanonicalSession(prepareAction,struct('challenge',request.original_host_challenge, ...
        'origin_ned_m',origin,'host_system',request.host_system,'host_component',request.host_component,'leg_index',uint8(1)));
    request.original_prepare_submit_ns=tx.original_host_submit_ns(1);sessionRaw.prepare_send=tx;
    [prepared,prepareFrames]=awaitSession(request,[],[],auth,false);
    tx=io.sendCanonicalSession('confirm',struct('prepared_receipt',prepared, ...
        'physical_setup_record_sha256',auth.physical_setup_record_sha256));
    request.original_confirm_submit_ns=tx.original_host_submit_ns(1);
    request.original_confirm_session_sha256=prepared.execution_session_sha256;
    request.confirmed_physical_setup_record_sha256=auth.physical_setup_record_sha256;sessionRaw.confirm_send=tx;
    [association,confirmFrames]=awaitSession(request,prepareFrames,[],auth,true);
    sessionRaw.association=association;sessionRaw.confirm_frames=confirmFrames;
    e=association.echo;
    expected=struct('source_system',e.system,'source_component',e.component, ...
        'target_system',uint8(255),'target_component',uint8(190),'uid',e.uid, ...
        'session_generation',e.process_session_generation,'link_lifecycle_generation',e.link_lifecycle_generation, ...
        'confirmed_host_rx_ns',association.original_host_receive_ns, ...
        'execution_session_sha256',association.execution_session_sha256, ...
        'configuration_sha256',e.configuration_payload_sha256,'local_full_inner',true, ...
        'leg_index',uint8(1),'task_sha256',task.sha256,'reference_asset_sha256',referenceSha);
    registered=struct('local_full_inner',true,'identity',struct('uid',e.uid, ...
        'boot_generation',e.process_session_generation,'system',e.system,'component',e.component), ...
        'leg_index',uint8(1),'task_sha256',task.sha256,'execution_session_sha256',association.execution_session_sha256, ...
        'configuration_sha256',e.configuration_payload_sha256,'reference_asset_sha256',referenceSha);
    io.bindCanonicalSession(expected,association);verified=io.evidence();
    assert(isequaln(verified.canonical_exchange_association,association),'gpenmpcShort:ActualAssociation','Same IO must retain the exact registered association.');
    % Derive caller verification from the IO command/receipt ledger.
    identityReceipt=struct('schema','RFLY_CALLER_VERIFIED_SAME_IO_IDENTITY_V1','verified',true, ...
        'uid',e.uid,'system_id',e.system,'component_id',e.component,'boot_generation',e.process_session_generation, ...
        'configuration_payload_sha256',e.configuration_payload_sha256, ...
        'execution_session_sha256',association.execution_session_sha256, ...
        'original_observed_io_time_s',io.now(),'original_observed_host_ns',association.original_host_receive_ns, ...
        'provenance','ACTUAL_SAME_IO_PREPARE_CONFIRM_AND_BIND_CANONICAL_SESSION_LEDGER');
    expectedIdentity=rmfield(identityReceipt,{'schema','verified','original_observed_io_time_s', ...
        'original_observed_host_ns','provenance','execution_session_sha256'});
    expectedIdentity.rfly_board_commit=struct('execution_session_sha256',association.execution_session_sha256);
    initialPhase=gpenmpcNative.RflyLocalPhaseView(bundle,trajectory,registered,association);initialView=initialPhase.view();
    % Enable snapshot transmission before starting the board task so its first
    % snapshot does not wait behind the synchronous startup response.
    stream=gpenmpcNative.RflyLocalStreamLifecycle(io,registered,association);sessionRaw.stream_enable=stream.enable();
    sessionRaw.getter_drain_after_stream_enable=startupDrainObservationOnly('AFTER_STREAM_ENABLE_BEFORE_METHOD_SERVICE');
    sc=c.service_cfg;sc.stream_lifecycle=stream;
    if isfield(c,'runtime_state_only')&&c.runtime_state_only
        getter.useRuntimeStateOnly();
        sc.runtime_state_only=true;
    end
    if isfield(objects,'outer')&&~isempty(objects.outer),sc.prepared_outer=objects.outer;end
    assert(isstruct(objects.method_backends)&&strcmp(objects.method_backends.schema,'GPENMPC_PREPARED_LOCAL_METHOD_BACKENDS_V1'), ...
        'gpenmpcShort:PreparedMethodBackends','Session-independent backends must be prepared before CopterSim starts.');
    sc.prepared_input_codec_backend=objects.method_backends.input_codec;
    sc.prepared_gp_backend=objects.method_backends.gp;
    constructionStarted=io.now();
    service=gpenmpcNative.RflyLocalMethodService(a,bundle,trajectory,referenceBinding,registered,association,io,getter,environment,task,sc);
    sessionRaw.method_service_construction=struct('started_io_time_s',constructionStarted, ...
        'finished_io_time_s',io.now(),'session_independent_backends_prepared_before_getter_producer',true, ...
        'constructor_file_hash_or_mex_load_checks',0,'authority_changed',false);
    sessionRaw.getter_drain_after_method_service=startupDrainObservationOnly('AFTER_METHOD_SERVICE_BEFORE_ENVIRONMENT_PRIME');
    % Enable method dispatch after the module-start observation so the
    % 244-fragment window is consumed by the 16-slot ingress queue.
    mode='PREPARE';pump();
    startupDrainObservationOnly('AFTER_COLD_POLL_BEFORE_ENVIRONMENT');
    envService=objects.environment_service;
    environmentRefreshActive=true;
    activationStatus=envService.status();
    sessionRaw.environment_activation=struct('io_time_s',io.now(), ...
        'after_stream_enable',true,'before_module_start',true,'after_method_service_construction',true, ...
        'send_attempt_count_before_activation',activationStatus.send_attempt_count, ...
        'scope','HOST_LIFECYCLE_ORDER_ONLY__UNCHANGED_250MS_DEADLINE');
    assert(sessionRaw.environment_activation.send_attempt_count_before_activation==0, ...
        'gpenmpcShort:PrematureEnvironmentActivation','Environment lease was consumed before blocking construction completed.');
    note('ENVIRONMENT_REFRESH_ACTIVATED','After stream enable and method-service construction; before the single module start and PREPARE polling.');
    % Establish the environment lease before starting the board module.
    mode='PREPARE';phase='ENVIRONMENT_PRIME';
    sessionRaw.environment_prime=primeEnvironment();
    % Refresh the observation after cold frame construction, then update the environment.
    primeEnvironment();
    % Start source matching at the module-launch boundary and retain all
    % rows that can supply its first snapshot.
    sessionRaw.start_send=io.sendCanonicalSession('start',struct());
    % Send a heartbeat after the synchronous start response and before matching.
    heartbeat(true);
    getter.beginSourceMatching();
    matchingStatus=getter.status();
    sessionRaw.getter_source_matching_begin=struct('io_time_s',io.now(), ...
        'source_matching_started',matchingStatus.source_matching_started, ...
        'source_matching_closed',matchingStatus.source_matching_closed, ...
        'retained_getters',matchingStatus.retained_getters, ...
        'scope','IMMEDIATELY_AFTER_SINGLE_START_SEND_RETURN__BEFORE_FIRST_RECEIPT_WAIT_PUMP__NO_CONTROL_OR_BOARD_AUTHORITY');
    phase='MODULE_START';
    sessionRaw.start_observation=awaitLocalStart(sessionRaw.start_send.original_host_submit_ns(1));
    mode='PREPARE';phase='PREPARE';
    if isfield(c,'component_initialization')&&c.component_initialization,mode='COMPONENT';phase='COMPONENT_INITIALIZATION';end
    waitUntil(@preparedWithWindow,c.preparation_timeout_s,'PREPARATION_AND_ORIGINAL_WINDOW');
    % Prestream through the board-local OCM producer.
    t=io.now();while io.now()-t<cfg.prestream_s,pump();io.sleep(c.poll_period_s);end
    counts.offboard_requests=counts.offboard_requests+1;phase='OFFBOARD';
    command(176,[1 6 0 0 0 0 0],cfg.mode_timeout_s,@offboardDisarmed,'OFFBOARD');
    % Component mode supplies its pre-arm input; full-runtime solving begins
    % after the armed state is observed so command lifetime excludes ARM latency.
    afterArmBootstrap=~c.component_initialization&&c.runtime_state_only;
    if ~afterArmBootstrap
        if ~strcmp(mode,'COMPONENT'),mode='BOOTSTRAP';end
        phase='BOOTSTRAP';waitUntil(@bootstrapReady,c.preparation_timeout_s,'FRESH_BOOTSTRAP');
    end
    % Require a fresh state and a successful input from the same bootstrap poll.
    if isfield(c.service_cfg,'operator_reference')&&service.OperatorFinishRequested
        error('gpenmpcShort:OperatorCancelled','User requested finish before ARM.');
    end
    counts.arm_requests=counts.arm_requests+1;phase='ARM_REQUESTED';
    command(400,[1 0 0 0 0 0 0],cfg.arm_timeout_s,@isArmed,'LOGICAL_ARM');
    if afterArmBootstrap
        mode='BOOTSTRAP';phase='BOOTSTRAP_AFTER_ARM';
        waitUntil(@bootstrapReady,c.preparation_timeout_s,'FRESH_ORIGINAL_SOLVE_AFTER_ARM');
        mode='FLIGHT';
    end
    phase='FLIGHT';waitUntil(@hasFirstCommit,cfg.arm_timeout_s,'FIRST_REAL_COMMIT');
    if isfield(c.service_cfg,'operator_reference')
        if c.duration_s==0
    fprintf('USB_RC_ACTIVE: Ready. Sticks are live. End flight: Ctrl+Shift+L.\n');
        else
            fprintf('USB_RC_ACTIVE: %.0f-second TEST window; timer end WILL LAND. Right stick: aircraft-heading forward/right; left vertical: up/down; left horizontal: yaw. Ctrl+Shift+L ends early.\n',c.duration_s);
        end
    end
    while c.duration_s==0||io.now()-firstCommit<c.duration_s
        cycleStart=io.now();pump();assertOperational();
        if isfield(c.service_cfg,'operator_reference')&&service.OperatorFinishRequested,break;end
        remaining=c.poll_period_s-(io.now()-cycleStart);
        if remaining>0,io.sleep(remaining);end
    end
    formalEnd=io.now();
    if isfield(c.service_cfg,'operator_reference')&&service.OperatorFinishRequested
        note('USER_REQUESTED_MANUAL_LAND','Ctrl+Shift+L requested normal LAND.');
        fprintf('USB_RC_STOP: user requested LAND and recovery.\n');
    else
        note('PLANNED_SHORT_STOP',sprintf('Timed session complete: %g s from the first joint commit.',c.duration_s));
        if isfield(c.service_cfg,'operator_reference'),fprintf('USB_RC_STOP: declared TEST timer ended; native LAND and recovery now.\n');end
    end
catch ex
    if strcmp(ex.identifier,'gpenmpcShort:ManualReset')
        note('USER_REQUESTED_SIMULATION_RESET','Explicit reset; outer owner will stop this model and reset the HIL application.');
        fprintf('USB_RC_RESETTING: Resetting simulation...\n');
    else
    if ~isempty(service),retain('method',service.LastEvent);end
    failure=struct('identifier',ex.identifier,'message',ex.message,'stack',ex.stack, ...
        'phase',phase,'io_time_s',io.now(),'snapshot',latest,'environment_refresh_timing',lastRefreshTiming);
    % Retain a diagnostic shell/command suffix before recovery.
    try
        failure.session_diagnostic=capture_gpenmpc_failure_diagnostic(sessionRaw,io.evidence());
    catch captureError
        failure.session_diagnostic_capture_error=captureError.message;
    end
    if ~saveFullRaw&&~isempty(service)
        % Preserve the failed poll's timings before recovery updates the service.
        failure.last_method_event=service.LastEvent;
    end
    note('FIRST_FAILURE',[ex.identifier ': ' ex.message]);
    if isfield(c.service_cfg,'operator_reference')
        if counts.arm_requests==0
            fprintf(2,'USB_RC_STOP: Preparation incomplete; returning to standby. %s: %s\n',ex.identifier,ex.message);
        else
            fprintf(2,'USB_RC_STOP: Control stopped; landing and recovering. %s: %s\n',ex.identifier,ex.message);
        end
    end
    end
end
finish();clear guard
state=[];if ~isempty(service),state=service.status();end
if ~isempty(state),counts.gp_replies=state.counts.gp_replies;end % Native actual sends survive deferred callback history.
result=struct('schema','M600_BOARD_LOCAL_SHORT_HIL_V1','planned_active_s',c.duration_s, ...
    'window_completed',isfinite(formalEnd),'task_completed',false,'safe_ground',safeGround, ...
    'io_closed',ioClosed,'counts',counts,'first_commit_io_s',firstCommit,'last_commit_io_s',lastCommit, ...
    'formal_end_io_s',formalEnd,'failure',failure,'cleanup_failures',{cleanupFailures}, ...
    'method',state,'session_release',release,'geometry_restored',false,'tuning_restored',false, ...
    'getter_buffer',getter.status(), ...
    'pending_parameter_recovery',pendingRecovery,'parameter_recovery_complete',false, ...
    'outer_must_continue_native_safety',outerMustContinueSafety, ...
    'manual_reset_requested',manualResetRequested, ...
    'host_inner_steps',0,'new_plant_instances',0,'getter_closed_here',false, ...
    'outer_must_stop_simulator_then_drain_and_close_getter',~outerMustContinueSafety, ...
    'control_source','ACTUAL_BOARD_LOCAL_FULL_INNER_ONLY', ...
    'pure_object_preparation',objects.receipt, ...
    'environment_ledger_capacity',c.environment_ledger_capacity,'full_raw_capacity',c.raw_capacity, ...
    'full_raw_recording_enabled',saveFullRaw,'raw_storage_pending',saveFullRaw, ...
    'energy_scope','Any reconstructed energy is model-calculated, not battery measurement', ...
    'duration_scope',sprintf('%g s session; 25 phase-second takeoff reference followed by the Cambridge leg',c.duration_s));
if ~saveFullRaw
    recent=io.evidence();
    result.recording=struct('full_flight_file_saved',false,'recent_history_capacity',recent.recent_history_capacity, ...
        'retired_history_records',recent.retired_history_records, ...
        'retained_mavlink_records',numel(recent.raw_mavlink),'retained_tx_records',numel(recent.raw_transmit_messages));
    % Retain native error lines and the first-fault record.
    try
        result.board_stop_lines=existingBoardStopLines(recent.raw_mavlink);
        if ~isempty(failure)
            % Retain the first IO failure alongside later wrapper errors.
            result.failure.io_first_fatal=recent.first_fatal;
            result.failure.io_exchange_failure=recent.canonical_exchange_failure;
            % Retain the input batch preceding the fault, not only the final poll event.
            result.failure.input_send_tail={};
            for txIndex=numel(recent.raw_transmit_messages):-1:1
                tx=recent.raw_transmit_messages{txIndex};
                if isfield(tx,'canonical_local_task_inputs')&&tx.canonical_local_task_inputs ...
                        &&isfield(tx,'sent_s')&&isfinite(tx.sent_s)&&tx.sent_s<=failure.io_time_s
                    result.failure.input_send_tail=[{tx};result.failure.input_send_tail];
                    if numel(result.failure.input_send_tail)==36,break,end
                end
            end
            result.failure.heartbeat_send_tail={};
            for txIndex=numel(recent.raw_transmit_messages):-1:1
                tx=recent.raw_transmit_messages{txIndex};
                if isfield(tx,'message')&&isfield(tx.message,'MsgID')&&tx.message.MsgID==0 ...
                        &&isfinite(tx.sent_s)&&tx.sent_s<=failure.io_time_s
                    result.failure.heartbeat_send_tail=[{tx};result.failure.heartbeat_send_tail];
                    if numel(result.failure.heartbeat_send_tail)==12,break,end
                end
            end
        end
    catch captureError
        % Preserve the flight failure and recovery path if text extraction fails.
        result.board_stop_lines={};
        result.board_stop_text_capture_error=captureError.message;
    end
    clear recent
end
if isfield(c.service_cfg,'operator_reference')
    result.reference_source='USB_OPERATOR_BODY_HEADING_VELOCITY_AND_YAW_RATE';
    result.duration_scope=sprintf('%g s manual-reference SE(3) session',c.duration_s);
    if c.duration_s==0
        result.duration_scope='USER_ENDED_OPERATOR_COMPONENT';
    end
    result.user_requested_land=~isempty(service)&&service.OperatorFinishRequested;
    result.complete_enmpc_gp_method=false;
    result.operator_reference_sha256=c.service_cfg.operator_reference_sha256;
    result.yaw_reference_enabled=true;
    result.yaw_rate_limit_rad_s=1.0; % Current manual profile; not a measured vehicle limit.
    result.yaw_motion_requires_raw_trace_confirmation=true;
end
if outerMustContinueSafety
    % Return promptly while safety observation remains active.
    safetyOwner=struct('pump',@serviceSafety,'finishAfterSafe',@completeSafety,'persistRaw',@persistRaw, ...
        'raw',@retainedRaw,'formal_control_resume_allowed',false);
    if ~saveFullRaw,safetyOwner=rmfield(safetyOwner,{'persistRaw','raw'});end
    result.raw_save_deferred_for_native_safety=true;
    return
end
result.raw_save_deferred_for_native_safety=false;
result.raw_storage_pending=saveFullRaw;
result.full_raw_recording_enabled=saveFullRaw;
% Retain this workspace and raw rows until outer recovery completes.
safetyOwner=struct('persistRaw',@persistRaw,'raw',@retainedRaw);
if ~saveFullRaw,safetyOwner=[];end
writeJson(fullfile(outputDir,'RESULT.json'),result);

    function pump()
        checkManualReset();
        % Send due heartbeats before potentially blocking host operations.
        heartbeat(false);
        runtimeReceiveAtInput=isfield(c,'runtime_state_only')&&c.runtime_state_only ...
            &&~isempty(service)&&~service.Closed&&~finalized ...
            &&isfield(sessionRaw,'start_observation')&&~isempty(sessionRaw.start_observation) ...
            &&sessionRaw.start_observation.task_created;
        latest=io.snapshot(@()heartbeat(false),~runtimeReceiveAtInput);
        if ~finalized&&~isempty(latest.fatal)
            note('ORIGINAL_IO_FATAL',jsonencode(latest.first_fatal));
            error('gpenmpcShort:OriginalIoFatal','Original IO failure: %s',latest.fatal);
        end
        heartbeat(false);
        % Refresh the environment immediately after the IO snapshot and before method work.
        refreshEnvironment();
        heartbeat(false);
        b=getterMex('drain');
        assert(strcmp(b.ring_name,opened.ring_name)&&strcmp(b.status_name,opened.status_name) ...
            &&isequal(b.peer_nonce,opened.peer_nonce)&&isequal(b.consumer_pid,opened.consumer_pid), ...
            'gpenmpcShort:GetterOwnerChanged','Original getter mapping ownership changed.');
        if size(b.records,2)>0
            producer=readLE(b.ring_header(21:24),'uint32');
            assert(double(producer)==double(resources.owned_simulator_pid),'gpenmpcShort:GetterProducer','Original ring producer is not the owned NoUI process.');
        end
        if size(b.records,2)>0||b.failed,retain('getter',b);end
        heartbeat(false);
        if ~isempty(service)&&~service.Closed&&~finalized&&isfield(sessionRaw,'start_observation') ...
                &&~isempty(sessionRaw.start_observation)&&sessionRaw.start_observation.task_created
            if isfield(c,'component_initialization')&&c.component_initialization,mode='COMPONENT';end
            historicalMatching=~(isfield(c,'runtime_state_only')&&c.runtime_state_only);
            if historicalMatching
                beforeGetter=getter.status();beforeMethod=service.status();
            end
            if historicalMatching&&beforeGetter.source_matching_started&&beforeMethod.counts.snapshots_taken==0 ...
                    &&beforeGetter.retained_getters+size(b.records,2)>c.getter_capacity
                % Preserve the unconsumed batch and query first-snapshot liveness once.
                sessionRaw.first_snapshot_liveness=struct('io_time_s',io.now(), ...
                    'retained_before',beforeGetter.retained_getters, ...
                    'incoming_records',size(b.records,2),'capacity',c.getter_capacity, ...
                    'snapshots_taken',beforeMethod.counts.snapshots_taken, ...
                    'classification','BOARD_FIRST_RLS1_ABSENT_BEFORE_NO_EVICTION_CAPACITY');
                try
                    sessionRaw.first_snapshot_liveness.module_status_send= ...
                        io.sendCanonicalSession('module_status',struct());
                catch statusError
                    sessionRaw.first_snapshot_liveness.module_status_error= ...
                        struct('identifier',statusError.identifier,'message',statusError.message);
                end
                error('gpenmpcShort:FirstSnapshotLiveness', ...
                    'The first RLS1 snapshot was absent when the getter buffer reached capacity.');
            end
            if strcmp(mode,'BOOTSTRAP')&&latest.armed==1&&~strcmp(phase,'BOOTSTRAP_AFTER_ARM'),mode='FLIGHT';end
            % Service the heartbeat before each method batch.
            heartbeat(true);
            % One forced heartbeat immediately above starts this bounded
            % method batch. Between its sections use the existing due-time
            % check, rather than queueing redundant writes at every helper.
            event=service.poll(b,@methodModeAfterReceive,@serviceRuntimeIo,latest.armed==0,@()heartbeat(false));retain('method',event);
            % Refresh the same IO's validated cache after the method receive pass.
            latest=io.snapshot(@()heartbeat(false),false);
            if ~isempty(latest.fatal)
                note('ORIGINAL_IO_FATAL',jsonencode(latest.first_fatal));
                error('gpenmpcShort:OriginalIoFatal','Original IO failure: %s',latest.fatal);
            end
            if strcmp(phase,'BOOTSTRAP_AFTER_ARM')&&strcmp(event.mode,'FLIGHT'),mode='FLIGHT';end
            counts.gp_replies=counts.gp_replies+numel(event.gp);
            if isfield(c,'component_initialization')&&c.component_initialization
                % Count board-reported joint installations; component telemetry is decimated.
                counts.commits=double(service.Phase.CommitCount);
            else
                counts.commits=counts.commits+numel(event.committed);
            end
            if ~isempty(event.committed)
                lastCommit=io.now();if ~isfinite(firstCommit),firstCommit=lastCommit;end
            end
            heartbeat(false);
        else
            % Before registration (and after the method is suspended), the
            % simulator still publishes diagnostics. Drain the same queue
            % without granting an ENV/source binding; raw IO retains bytes.
            startupEnvironment=io.takeCanonicalEnvironmentRecords();
            if ~isempty(startupEnvironment)
                retain('environment_observation',struct('phase',phase, ...
                    'records',{startupEnvironment},'source_binding_granted',false));
            end
            gs=getter.status();
            if gs.source_matching_started&&~gs.source_matching_closed,getter.ingest(b);
            else,getter.observeOnly(b);end
            heartbeat(false);
        end
    end
    function serviceRuntimeIo()
        % Yield between bounded batches; the next refresh uses a new IO snapshot.
        heartbeat(true);
        if isfield(c,'runtime_state_only')&&c.runtime_state_only&&~finalized
            % Refresh model/time and environment before receiving another MAVLink batch.
            latest=io.snapshot(@()heartbeat(false),false);
            if ~isempty(latest.fatal)
                error('gpenmpcShort:OriginalIoFatal','Original IO failure: %s',latest.fatal);
            end
            refreshEnvironment();heartbeat(false);
        end
        fullRuntime=isfield(c,'runtime_state_only')&&c.runtime_state_only ...
            &&~c.component_initialization&&~finalized;
        if ~fullRuntime
            latest=io.snapshot(@()heartbeat(false));
            if ~finalized&&~isempty(latest.fatal)
                error('gpenmpcShort:OriginalIoFatal','Original IO failure: %s',latest.fatal);
            end
            heartbeat(false);refreshEnvironment();heartbeat(false);
        end
        % Refresh the getter before input binding and between bounded history slices.
        % MethodService owns the MAVLink receive; archive observations before validation.
        b=getterMex('drain');
        assert(strcmp(b.ring_name,opened.ring_name)&&isequal(b.peer_nonce,opened.peer_nonce) ...
            &&isequal(b.consumer_pid,opened.consumer_pid),'gpenmpcShort:GetterOwnerChanged');
        if size(b.records,2)>0||b.failed,retain('getter',b);end
        gs=getter.status();
        if gs.source_matching_started&&~gs.source_matching_closed,getter.ingest(b);
        else,getter.observeOnly(b);end
        % Send a heartbeat at the end of each cooperative service slice.
        heartbeat(true);
    end
    function receipt=primeEnvironment()
        % Publish the initial model/board environment snapshot immediately before
        % module start; defer getter validation and method polling until afterwards.
        latest=io.snapshot();
        if ~isempty(latest.fatal)
            error('gpenmpcShort:OriginalIoFatal','Original IO failure before environment prime: %s',latest.fatal);
        end
        heartbeat(false);
        before=envService.status();outcome=refreshEnvironment();after=envService.status();
        assert(after.send_return_count==before.send_return_count+uint64(1), ...
            'gpenmpcShort:EnvironmentPrime','The fresh pre-start environment frame was not sent exactly once.');
        receipt=struct('snapshot_io_time_s',latest.now_s,'send_outcome',outcome, ...
            'send_count_before',before.send_return_count,'send_count_after',after.send_return_count, ...
            'getter_drains_between_send_and_start',0,'method_polls_between_send_and_start',0, ...
            'deadline_changed',false,'board_authority',false);
    end
    function outcome=refreshEnvironment()
        outcome=[];
        if ~environmentRefreshActive||isempty(envService)||(envService.Failed&&~finalized),return,end
        lastRefreshTiming=struct('clock','EXISTING_IO_MONOTONIC_SECONDS','snapshot_s',latest.now_s,'entry_s',io.now());
        if ~isempty(service),v=service.Phase.view();else,v=initialView;end
        lastRefreshTiming.phase_return_s=io.now();
        lastView=v;q=v.controller_phase_s;
        jet=trajectory.environment_jet_fcn(q);
        lastRefreshTiming.reference_return_s=io.now();
        jet=jet.*[1;1;-1];
        [wind,~,~]=gpenmpcNative.canonicalSavedTaskWindAt(task.cached_asset,v.saved_task_time_s,1);
        lastRefreshTiming.wind_return_s=io.now();
        ref=struct('position_ned_m',jet(:,1)+worldGround,'velocity_ned_mps',jet(:,2), ...
            'acceleration_ned_mps2',jet(:,3),'jerk_ned_mps3',jet(:,4), ...
            'actual_wind_ned_xy_mps',wind,'environment_task_sha256',task.sha256, ...
            'reference_scope','NOMINAL_JET_AT_CURRENT_PHASE_FOR_ENVIRONMENT');
        life='PREARM';paused=true;
        if latest.armed==1,life='INITIAL_TAKEOFF';paused=false;end
        if finalized,life='NATIVE_LAND';paused=true;end
        intent=struct('payload_kg',bundle.legs{1}.meta.payload_kg,'payload_generation',0, ...
            'release_generation',0,'task_clock_paused',paused,'lifecycle_phase',life,'payload_update_requested',false);
        lastRefreshTiming.environment_call_s=io.now();
        outcome=envService.stepCanonical(latest,v,ref,intent,identityReceipt,expectedIdentity);
        lastRefreshTiming.environment_return_s=io.now();
    end
    function receipt=startupDrainObservationOnly(label)
        % Drain startup diagnostics at lifecycle boundaries before source matching begins.
        for drainIndex=1:32 % Existing 8192-slot HOST pending bound / 256 batch.
        b=getterMex('drain');
        assert(strcmp(b.ring_name,opened.ring_name)&&strcmp(b.status_name,opened.status_name) ...
            &&isequal(b.peer_nonce,opened.peer_nonce)&&isequal(b.consumer_pid,opened.consumer_pid), ...
            'gpenmpcShort:GetterOwnerChanged','Original getter mapping ownership changed.');
        if size(b.records,2)>0
            producer=readLE(b.ring_header(21:24),'uint32');
            assert(double(producer)==double(resources.owned_simulator_pid), ...
                'gpenmpcShort:GetterProducer','Original ring producer is not the owned NoUI process.');
        end
        if size(b.records,2)>0||b.failed,retain('getter',b);end
        getter.observeOnly(b);
        if size(b.records,2)<256,break,end
        end
        pendingEnvironment=io.takeCanonicalEnvironmentRecords();
        if ~isempty(pendingEnvironment)
            retain('environment_observation',struct('phase',label, ...
                'records',{pendingEnvironment},'source_binding_granted',false));
        end
        receipt=struct('label',label,'io_time_s',io.now(),'records',size(b.records,2), ...
            'failed',logical(b.failed),'total_read',b.total_read,'board_authority',logical(b.board_authority), ...
            'source_binding_granted',false,'environment_records',numel(pendingEnvironment));
    end
    function heartbeat(force)
        if nargin<1,force=false;end
        % Coalesce forced heartbeats within 10 ms; periodic service uses its configured interval.
        duePeriod=c.heartbeat_period_s;
        if force,duePeriod=min(duePeriod,.010);end
        if isstruct(latest)&&isscalar(latest)&&isfield(latest,'automatic_mavlink_peer_ready') ...
                &&latest.automatic_mavlink_peer_ready ...
                &&io.now()-lastHeartbeat>=duePeriod
            io.sendHeartbeat();lastHeartbeat=io.now();
        end
    end
    function yes=stateFresh()
        now=io.now();yes=isstruct(latest)&&all(isfield(latest,{'armed','landed_state','heartbeat_rx_s','extended_rx_s'})) ...
            &&all(isfinite([latest.armed latest.landed_state latest.heartbeat_rx_s latest.extended_rx_s])) ...
            &&now>=latest.heartbeat_rx_s&&now>=latest.extended_rx_s ...
            &&now-latest.heartbeat_rx_s<=cfg.heartbeat_max_age_s&&now-latest.extended_rx_s<=cfg.landed_max_age_s;
    end
    % Named nested callbacks read the current shared state after each pump;
    % anonymous expressions that directly mention latest capture an old value.
    function yes=offboardDisarmed(),yes=stateFresh()&&latest.main_mode==6&&latest.armed==0;end
    function yes=isArmed(),yes=stateFresh()&&latest.armed==1;end
    function yes=isDisarmed(),yes=stateFresh()&&latest.armed==0;end
    function yes=nativeLandMode(),yes=stateFresh()&&latest.main_mode==4&&latest.sub_mode==6;end
    function yes=hasFirstCommit(),yes=isfinite(firstCommit);end
    function yes=ground()
        yes=isstruct(latest)&&isfield(latest,'model_ready')&&latest.model_ready ...
            &&isfield(latest,'model_diagnostic')&&isfield(latest.model_diagnostic,'decoded') ...
            &&isfield(latest.model_diagnostic.decoded,'ground_confirmed')&&latest.model_diagnostic.decoded.ground_confirmed;
    end
    function yes=ready()
        yes=stateFresh()&&ground()&&latest.armed==0&&latest.landed_state==1&&latest.clock_valid ...
            &&~isempty(latest.estimate)&&~isempty(latest.truth)&&isempty(latest.fatal);
        if yes
            % Require valid estimator outputs and clear error flags before registration.
            preflightTelemetry=latest.native_hover_telemetry;
            yes=isfield(preflightTelemetry.streams,'estimator_status')&&preflightTelemetry.streams.estimator_status.passed ...
                &&isfield(preflightTelemetry.checks,'estimator_required_outputs_valid') ...
                &&preflightTelemetry.checks.estimator_required_outputs_valid&&preflightTelemetry.checks.estimator_error_flags_clear;
        end
        if yes
            now=io.now();
            yes=all(isfinite([latest.estimate.position_ned_m(:);latest.truth.position_ned_m(:); ...
                latest.estimate.velocity_ned_mps(:);latest.truth.velocity_ned_mps(:); ...
                latest.estimate.rx_s;latest.truth.rx_s])) ...
                &&now>=latest.estimate.rx_s&&now>=latest.truth.rx_s ...
                &&now-latest.estimate.rx_s<=cfg.state_max_age_s&&now-latest.truth.rx_s<=cfg.state_max_age_s;
            % Apply the origin-offset check within the bounded readiness wait.
            if yes
                yes=norm(double(latest.truth.position_ned_m(:))-worldGround- ...
                    double(latest.estimate.position_ned_m(:)))<=cfg.maximum_initial_origin_offset_m;
            end
        end
        if yes
            yes=isfield(latest,'autopilot_version')&&isstruct(latest.autopilot_version) ...
                &&all(isfield(latest.autopilot_version,{'uid','board_version','flight_custom_version'}));
            if yes
                av=latest.autopilot_version;
                assert(isa(av.uid,'uint64')&&av.uid==uid&&av.board_version==pre.board_id ...
                    &&strcmpi(upper(reshape(dec2hex(av.flight_custom_version(:),2).',1,[])),expectedFlightVersion), ...
                    'gpenmpcShort:ObservedApplication','Actual same-IO UID/application differs from fresh serial preflight.');
            end
        end
    end
    function assertOperational()
        assert(stateFresh()&&latest.armed==1&&latest.main_mode==6&&latest.clock_valid ...
            &&isempty(latest.fatal)&&~isempty(latest.estimate)&&~isempty(latest.truth), ...
            'gpenmpcShort:Operational','Original active state or clock is no longer valid.');
        assert(io.now()-latest.estimate.rx_s<=cfg.state_max_age_s ...
            &&io.now()-latest.truth.rx_s<=cfg.state_max_age_s,'gpenmpcShort:StateAge','Original estimator or plant observation expired.');
        % Apply the caller-selected diagnostic abort values.
        assert(norm(latest.truth.velocity_ned_mps)<=cfg.abort_truth_speed_mps,'gpenmpcShort:TruthSpeed','Original truth-speed abort bound was exceeded.');
        assert(norm(double(latest.estimate.position_ned_m(:))+coordinateOffset- ...
            double(latest.truth.position_ned_m(:)))<=cfg.abort_estimator_gap_m,'gpenmpcShort:EstimatorGap','Original estimator/plant gap abort bound was exceeded.');
    end
    function yes=preparedWithWindow()
        s=service.status();yes=(s.outer.prepared||s.component_initialization)&&~isempty(s.last_window_tx)&&s.last_window_tx.send_complete;
    end
    function yes=bootstrapReady()
        expectedArmed=double(strcmp(phase,'BOOTSTRAP_AFTER_ARM'));
        yes=stateFresh()&&latest.armed==expectedArmed&&latest.main_mode==6;
        if ~yes,return;end
        if expectedArmed==1
            % Solver acceptance alone does not confirm input transmission.
            % Use this batch's send result in the deadline-sensitive loop.
            last=service.LastEvent;
            yes=strcmp(last.mode,'FLIGHT')&&isfield(last,'input_send') ...
                &&last.input_send.messages_send_returned==6 ...
                &&all(last.input_send.returned_ns>0) ...
                &&all(last.input_send.returned_ns<=last.input_send.binding.valid_until_host_ns) ...
                &&gpenmpcNative.rflyOriginalHostMonotonicNs()<=last.input_send.binding.valid_until_host_ns;
            return
        end
        s=service.status();cmd=s.outer.last_command;
        if s.component_initialization,cmd=s.nominal_initialization_command;end
        yes=~isempty(cmd)&&gpenmpcNative.rflyOriginalHostMonotonicNs()<=cmd.original_expiry_ns;
        if yes&&s.component_initialization
            if isfield(c.service_cfg,'operator_reference')&&counts.arm_requests==0
                yes=all(abs(cmd.target4)<=.05);
            end
            last=service.LastEvent;
            yes=yes&&isfield(cmd,'input_send')&&cmd.input_send.messages_send_returned==6 ...
                &&isfield(last,'input_send')&&last.input_send.messages_send_returned==6 ...
                &&last.input_send.binding.original_source_generation==cmd.source_generation ...
                &&cmd.input_send.binding.original_source_generation==cmd.source_generation ...
                &&cmd.generation==cmd.source_generation ...
                &&numel(cmd.input_send.returned_ns)==6 ...
                &&all(cmd.input_send.returned_ns>0) ...
                &&all(cmd.input_send.returned_ns<=cmd.input_send.binding.valid_until_host_ns) ...
                &&gpenmpcNative.rflyOriginalHostMonotonicNs()<=cmd.input_send.binding.valid_until_host_ns;
        end
    end
    function waitUntil(predicate,timeout,label)
        t=io.now();identityRequestTime=-Inf;identityRequestIndex=0;
        while io.now()-t<timeout
            cycleStart=io.now();pump();
            % Request AUTOPILOT_VERSION if it was not published; validate the received identity.
            if strcmp(label,'PREFLIGHT')&&stateFresh()&&latest.armed==0 ...
                    &&latest.automatic_mavlink_peer_ready
                if identityRequestIndex>0
                    identityAck=io.commandAck(512,identityRequestTime,false);
                    if ~isempty(identityAck)
                        commandRaw{identityRequestIndex}.ack=identityAck;
                        assert(ismember(identityAck.result,[0 5]), ...
                            'gpenmpcShort:IdentityRequestRejected','Read-only AUTOPILOT_VERSION request rejected.');
                    end
                end
                if isempty(latest.autopilot_version)&&io.now()-identityRequestTime>=cfg.command_timeout_s
                    identityRequestTime=io.now();
                    commandRaw{end+1}=struct('command',512,'parameters',[148 0 0 0 0 0 0], ...
                        'label','READONLY_AUTOPILOT_VERSION','original_submit_io_s',identityRequestTime,'ack',[]);
                    identityRequestIndex=numel(commandRaw);
                    io.requestCommand(512,[148 0 0 0 0 0 0]);
                end
            end
            if predicate(),return,end
            remaining=c.poll_period_s-(io.now()-cycleStart);
            if remaining>0,io.sleep(remaining);end
        end
        error('gpenmpcShort:WaitTimeout','%s timed out without changing its original source/lease.',label);
    end
    function value=methodModeAfterReceive()
        value=mode;
        if ~strcmp(phase,'ARM_REQUESTED')||~strcmp(mode,'BOOTSTRAP'),return,end
        request=commandRaw{end};
        assert(request.command==400&&request.parameters(1)==1&&strcmp(request.label,'LOGICAL_ARM'), ...
            'gpenmpcShort:ArmTransitionRequest','Current single ARM request required.');
        ack=io.commandAck(400,request.original_submit_io_s,false);
        if isempty(ack)||ack.result==5,value='ARM_WAIT';return,end
        assert(ack.result==0,'gpenmpcShort:CommandRejected','LOGICAL_ARM rejected: %d',ack.result);
        % An accepted ARM request permits input service; fresh telemetry still confirms armed state.
        commandRaw{end}.ack=ack;mode='FLIGHT';value=mode;
    end
    function command(id,parameters,timeout,predicate,label)
        sent=io.now();io.requestCommand(id,parameters);ack=[];
        row=struct('command',id,'parameters',parameters,'label',label,'original_submit_io_s',sent,'ack',[]);
        commandRaw{end+1}=row;index=numel(commandRaw);
        while io.now()-sent<timeout
            cycleStart=io.now();
            if finalized,safetyPump();else,pump();end
            % Read ACKs accepted by the last pump; receive new bytes in the next bounded pump.
            candidate=io.commandAck(id,sent,false);
            if ~isempty(candidate)
                ack=candidate;commandRaw{index}.ack=ack;
                assert(ismember(ack.result,[0 5]),'gpenmpcShort:CommandRejected','%s rejected: %d',label,ack.result);
            end
            if ~isempty(ack)&&ack.result==0&&predicate(),return,end
            remaining=c.poll_period_s-(io.now()-cycleStart);
            if remaining>0,io.sleep(remaining);end
        end
        error('gpenmpcShort:CommandTimeout','%s did not receive acceptance and matching observed state.',label);
    end
    function [decoded,frames]=awaitSession(request,prepareFrames,unused,authorization,confirming) %#ok<INUSD>
        t=io.now();frames=[];decoded=[];
        if confirming,submit=request.original_confirm_submit_ns;else,submit=request.original_prepare_submit_ns;end
        while io.now()-t<cfg.command_timeout_s
            pump();frames=framesAfter(submit);
            if ~isempty(frames)
                try
                    if confirming,decoded=gpenmpcNative.RflySessionAssociationDecoder(prepareFrames,frames,request,authorization,d);
                    else,decoded=gpenmpcNative.RflySessionAssociationDecoder(frames,[],request,[],d);end
                    return
                catch ex
                    if ~strcmp(ex.identifier,'gpenmpcNative:SessionLine'),rethrow(ex);end
                end
            end
            io.sleep(c.poll_period_s);
        end
        error('gpenmpcShort:SessionTimeout','No complete original session receipt.');
    end
    function restartPrearmEstimator()
        phase='PREFLIGHT_ESTIMATOR_INITIALIZATION';
        assert(isempty(service)&&isempty(registered)&&counts.commits==0&&counts.arm_requests==0, ...
            'gpenmpcShort:EstimatorRestartPhase','Only untouched pre-arm startup can initialize the estimator.');
        sessionRaw.prearm_ekf_initialization=struct();
        for action=["prearm_ekf_stop","prearm_ekf_start","prearm_ekf_status"]
            pump();
            assert(stateFresh()&&ground()&&latest.armed==0&&latest.landed_state==1 ...
                &&io.now()-preflightStarted<cfg.preflight_deadline_s, ...
                'gpenmpcShort:EstimatorRestartState','Fresh disarmed/ground and original acquisition time required.');
            sent=io.sendCanonicalSession(action,struct());
            sessionRaw.prearm_ekf_initialization.(action)=struct('send',sent,'text','','completed',false);
            begin=io.now();completed=false;
            while io.now()-begin<cfg.command_timeout_s&&io.now()-preflightStarted<cfg.preflight_deadline_s
                pump();
                assert(stateFresh()&&ground()&&latest.armed==0&&latest.landed_state==1, ...
                    'gpenmpcShort:EstimatorRestartState','Pre-arm safety changed during native initialization.');
                frames=io.canonicalSessionReceipts(sent.original_host_submit_ns(1),false,true);
                bytes=uint8([]);
                for f=1:numel(frames)
                    p=frames{f}.decoded_message.Payload;
                    assert(p.device==10&&p.count<=70,'gpenmpcShort:EstimatorShellSource');
                    bytes=[bytes;reshape(uint8(p.data(1:double(p.count))),[],1)]; %#ok<AGROW>
                end
                text=char(bytes.');text=regexprep(text,[char(27) '\[[0-9;]*[A-Za-z]'],'');
                sessionRaw.prearm_ekf_initialization.(action).text=text;
                echo=strfind(text,sent.encoding.original_command);
                if ~isempty(echo)
                    tail=text(echo(end)+length(sent.encoding.original_command):end);
                    completed=contains(tail,'nsh>');
                    if completed
                        assert(~contains(lower(tail),'error')&&~contains(lower(tail),'failed') ...
                            &&~contains(lower(tail),'module locked'), ...
                            'gpenmpcShort:EstimatorCommandRejected','Native command rejected: %s',tail);
                        if action=="prearm_ekf_stop"
                            assert(contains(tail,'stopping ekf2 instance')||contains(tail,'not running'), ...
                                'gpenmpcShort:EstimatorStopUnconfirmed','Stop must finish before start.');
                        elseif action=="prearm_ekf_status"
                            assert(~isempty(regexp(tail,'ekf2:\d+ EKF dt:','once')), ...
                                'gpenmpcShort:EstimatorStartUnconfirmed','Native status must report an actual EKF instance.');
                        end
                        sessionRaw.prearm_ekf_initialization.(action).completed=true;
                        break
                    end
                end
                io.sleep(c.poll_period_s);
            end
            assert(completed,'gpenmpcShort:EstimatorCommandTimeout','Native pre-arm initialization command did not complete; no retry/arm.');
        end
    end
    function frames=framesAfter(submit)
        % Use the last pump's receive batch and service the heartbeat before decoding more.
        selected=io.canonicalSessionReceipts(submit,false);
        frames=[];if ~isempty(selected),frames=vertcat(selected{:});end
    end
    function observed=awaitLocalStart(submit)
        % Wait for task-created confirmation while servicing the getter.
        begin=io.now();observed=[];
        while io.now()-begin<cfg.command_timeout_s
            pump();heartbeat(true);
            selected=io.canonicalSessionReceipts(submit,false,true);
            frames=[];if ~isempty(selected),frames=vertcat(selected{:});end
            if ~isempty(frames)
                observed=gpenmpcNative.RflyLocalStartReceiptDecoder(frames,request,d,@serviceRuntimeIo);
                if ~isempty(observed)
                    sessionRaw.start_observation=observed;
                    assert(observed.task_created&&observed.start_return==0 ...
                        &&observed.acquire_attempted&&observed.acquire_result==0, ...
                        'gpenmpcShort:ModuleStartRejected','Actual module start failed: %s',observed.original_line);
                    return
                end
            end
            io.sleep(c.poll_period_s);
        end
        error('gpenmpcShort:ModuleStartTimeout','No actual task-created start receipt; arm and method solve remain forbidden.');
    end
    function retain(kind,value)
        if ~saveFullRaw,return;end % Display packets and control validation are independent.
        if strcmp(kind,'getter')
            assert(numel(getterRaw)<c.raw_capacity,'gpenmpcShort:RawCapacity','Original raw evidence capacity reached; entries cannot be evicted.');getterRaw{end+1}=value;
        else
            assert(numel(methodRaw)<c.raw_capacity,'gpenmpcShort:RawCapacity','Original raw evidence capacity reached; entries cannot be evicted.');methodRaw{end+1}=value;
        end
    end
    function note(kind,detail)
        if numel(events)<c.raw_capacity,events{end+1}=struct('time_s',io.now(),'kind',kind,'detail',detail);end
    end
    function safetyPump()
        % Service is suspended. Continue getter draining and heartbeats even when
        % the environment sender has failed; pump() skips that sender.
        try,pump();catch ex
            if strcmp(ex.identifier,'gpenmpcShort:ManualReset'),rethrow(ex);end
            cleanError('environment_or_getter_during_land',ex);
        end
    end
    function checkManualReset()
        % Check the session-local request at 10 Hz without changing control timestamps.
        if isfield(c,'manual_reset_path')&&isfield(c.service_cfg,'operator_reference')
            now=io.now();
            if now>=nextResetCheckS
                nextResetCheckS=now+.1;
                manualResetRequested=manualResetRequested||isfile(c.manual_reset_path);
            end
            if manualResetRequested
                error('gpenmpcShort:ManualReset','Simulation reset requested.');
            end
        end
    end
    function finish()
        if finalized,return,end;finalized=true;phase='NATIVE_LAND';
        if ~isempty(service)
            try,service.suspendForNativeLand();catch ex
                cleanError('suspend',ex);
                try,service.close();catch closeError,cleanError('service_close',closeError);end
            end
        end
        try,getter.closeSourceMatching();catch ex,cleanError('getter_matching_close',ex);end
        try
            safetyPump();
            if counts.arm_requests>0
                % Require a post-ARM state observation before confirming disarm.
                armRows=find(cellfun(@(x)x.command==400&&x.parameters(1)==1,commandRaw));
                armSubmit=commandRaw{armRows(end)}.original_submit_io_s;
                waitStarted=io.now();postArmState=false;
                while io.now()-waitStarted<cfg.command_timeout_s
                    armAck=io.commandAck(400,armSubmit,false);
                    postArmState=~isempty(armAck)&&armAck.result~=5&&stateFresh() ...
                        &&latest.heartbeat_rx_s>armAck.received_at_s;
                    if postArmState,break,end
                    safetyPump();io.sleep(c.poll_period_s);
                end
                assert(postArmState,'gpenmpcShort:PostArmSafetyStateUnavailable', ...
                    'Need an actual heartbeat after the current ARM response; outer safety owner remains responsible.');
            end
            if stateFresh()&&latest.armed==1
                % When both PX4 and model report ground, ordinary disarm does not require LAND.
                if ~(latest.landed_state==1&&ground())
                    counts.land_requests=counts.land_requests+1;
                    command(176,[1 4 6 0 0 0 0],cfg.mode_timeout_s, ...
                        @nativeLandMode,'FINAL_NATIVE_LAND');
                    t=io.now();while io.now()-t<cfg.land_timeout_s
                        safetyPump();if stateFresh()&&latest.landed_state==1&&ground(),break,end;io.sleep(c.poll_period_s);
                    end
                end
                assert(stateFresh()&&latest.landed_state==1&&ground(),'gpenmpcShort:LandingUnconfirmed','Both original PX4 and plant ground evidence are required.');
                if latest.armed==1
                    counts.disarm_requests=counts.disarm_requests+1;
                    command(400,[0 0 0 0 0 0 0],cfg.disarm_timeout_s,@isDisarmed,'STANDARD_DISARM');
                end
            end
            safeGround=stateFresh()&&latest.armed==0&&latest.landed_state==1&&ground();
        catch ex,cleanError('land_disarm',ex);end
        if ~safeGround
            outerMustContinueSafety=true;
            pendingRecovery.before_outer_close_snapshot=latest;
            pendingRecovery.safe_ground_observed=false;
            pendingRecovery.module_release=release;
            note('NATIVE_SAFETY_OWNER_RETAINED','Ground/disarm unconfirmed: no stop, disable, release, IO close or simulator stop authorized here.');
            return
        end
        retireAndClose();
    end
    function retireAndClose()
        if ~isempty(association)
            stopTx=[];
            try,stopTx=io.sendCanonicalSession('stop',struct());sessionRaw.stop_send=stopTx;
            catch ex,cleanError('session_stop',ex);end
            % Attempt stream disable and IO close independently of task-stop success.
            if ~isempty(stream)
                try,sessionRaw.stream_disable=stream.disable();catch ex,cleanError('stream_disable',ex);end
            end
            try
                releaseTx=io.sendCanonicalSession('release',struct());sessionRaw.release_send=releaseTx;
                assert(~isempty(stopTx),'gpenmpcShort:StopReceiptUnavailable', ...
                    'Release was attempted; the stop-send receipt is unavailable.');
                rq=struct('registered_association',association, ...
                    'original_stop_submit_ns',stopTx.original_host_submit_ns(1), ...
                    'original_release_submit_ns',releaseTx.original_host_submit_ns(1));
                t=io.now();while io.now()-t<cfg.command_timeout_s
                    safetyPump();frames=framesAfter(rq.original_release_submit_ns);
                    if ~isempty(frames)
                        try,release=gpenmpcNative.RflySessionReleaseDecoder(frames,rq,d);break
                        catch ex,if ~strcmp(ex.identifier,'gpenmpcNative:ReleaseLine'),rethrow(ex);end,end
                    end
                    io.sleep(c.poll_period_s);
                end
                assert(~isempty(release),'gpenmpcShort:ReleaseUnconfirmed','Original module release was not confirmed.');
                % A release receipt does not confirm a zero plant input cache.
                % Query the retained post-release first-fault bundle before closing the COM owner.
                evidenceTx=io.sendCanonicalSession('evidence_local',struct());
                sessionRaw.post_release_evidence_send=evidenceTx;
                evidenceBegin=io.now();
                while io.now()-evidenceBegin<min(1.0,cfg.command_timeout_s)
                    safetyPump();io.sleep(c.poll_period_s);
                end
                sessionRaw.post_release_evidence_frames=framesAfter(evidenceTx.original_host_submit_ns(1));
            catch ex,cleanError('session_release',ex);end
        elseif ~isempty(stream)
            try,sessionRaw.stream_disable=stream.disable();catch ex,cleanError('stream_disable',ex);end
        end
        % Stop, drain and close the simulator owner before lengthy serial parameter recovery.
        pendingRecovery.before_outer_close_snapshot=latest;
        pendingRecovery.safe_ground_observed=safeGround;
        pendingRecovery.module_release=release;
        if ~isempty(envService)
            try,envService.close();catch ex,cleanError('environment_close',ex);end
        end
        try,ioClosed=io.close();catch ex,cleanError('io_close',ex);end
    end
    function snapshot=serviceSafety()
        assert(outerMustContinueSafety&&~ioClosed,'gpenmpcShort:SafetyOwnerClosed','Only the retained native safety owner may continue here.');
        safetyPump();snapshot=latest;
    end
    function completion=completeSafety()
        assert(outerMustContinueSafety&&~ioClosed,'gpenmpcShort:SafetyOwnerClosed','The retained native safety owner is unavailable.');
        safetyPump();safeGround=stateFresh()&&latest.armed==0&&latest.landed_state==1&&ground();
        assert(safeGround,'gpenmpcShort:SafetyStillUnconfirmed','Fresh original disarmed PX4 and plant ground evidence are required before stopping/closing.');
        retireAndClose();outerMustContinueSafety=~ioClosed;
        result.safe_ground=safeGround;result.io_closed=ioClosed;result.session_release=release;
        result.counts=counts;result.cleanup_failures=cleanupFailures;
        result.pending_parameter_recovery=pendingRecovery;
        result.outer_must_continue_native_safety=outerMustContinueSafety;
        result.outer_must_stop_simulator_then_drain_and_close_getter=~outerMustContinueSafety;
        result.native_safety_continuation_formal_credit=false;
        if ioClosed
            result.raw_save_deferred_for_native_safety=false;
            result.raw_storage_pending=saveFullRaw;
            writeJson(fullfile(outputDir,'RESULT.json'),result);
        end
        completion=result;
    end
    function raw=retainedRaw()
        raw=struct('events',{events},'getterRaw',{getterRaw},'methodRaw',{methodRaw}, ...
            'commandRaw',{commandRaw},'sessionRaw',sessionRaw,'parameterRaw',{parameterRaw}, ...
            'rawIo',io.evidence(),'cfg',cfg,'referenceBinding',referenceBinding);
    end
    function completion=persistRaw()
        assert(ioClosed&&~outerMustContinueSafety,'gpenmpcShort:RawBeforeSafety','Complete safety service before serializing raw data.');
        raw=retainedRaw();raw.result=result;
        fprintf('USB_RC_STORAGE: Saving complete raw data after board recovery.\n');
        result.raw_storage=save_m600_short_raw(fullfile(outputDir,'RAW_BOARD_LOCAL_SHORT_HIL.mat'),raw);
        result.raw_storage_pending=false;
        writeJson(fullfile(outputDir,'RESULT.json'),result);completion=result;
        fprintf('USB_RC_STORAGE_DONE: %.3f s, %.1f MB; all variables retained.\n',result.raw_storage.elapsed_s,result.raw_storage.bytes/1e6);
    end
    function cleanError(stage,ex)
        cleanupFailures{end+1}=struct('stage',stage,'identifier',ex.identifier,'message',ex.message);
    end
end
function receipt=appliedParameters(source,cfg,uid)
% Read typed setup values from the retained serial setup file.
assert(isstruct(source)&&isscalar(source)&&all(isfield(source,{'source_path','source_sha256'})) ...
    &&isfile(source.source_path)&&strcmpi(fileSha(source.source_path),source.source_sha256), ...
    'gpenmpcShort:SetupSource','Original completed serial setup file and exact SHA required.');
receipt=jsondecode(fileread(source.source_path));
assert(strcmp(receipt.schema,'GPENMPC_LOCAL_SHORT_PARAMETER_SETUP_V1') ...
    &&receipt.passed&&receipt.COM_closed&&exactUid(receipt.uid)==uid ...
    &&all(isfield(receipt,{'before_parameters','after_parameters','writes','aux_parameters','guard_parameters'})) ...
    &&numel(receipt.before_parameters)==167&&numel(receipt.after_parameters)==167 ...
    &&numel(receipt.writes)==21&&numel(receipt.aux_parameters)==8, ...
    'gpenmpcShort:SetupReceipt','The same-board original 167-before/after, 21 writes and AUX8 are required.');
assert(all(isfield(cfg,{'temporary_allocator_geometry','native_hover_tuning'})), ...
    'gpenmpcShort:SetupContracts','The actual existing geometry12/tuning3 recovery contracts are required.');
[originalContracts,originalProof]=load_m600_recovery_contracts();
assert(isequaln(cfg.temporary_allocator_geometry,originalContracts.temporary_allocator_geometry) ...
    &&isequaln(cfg.native_hover_tuning,originalContracts.native_hover_tuning), ...
    'gpenmpcShort:OriginalRecoveryContracts','Recovery contracts must match the bound recovery plan.');
receipt.original_recovery_contract_source=originalProof;
entries=cfg.temporary_allocator_geometry.entries(:);
entries=[entries;cfg.native_hover_tuning.entries(:)];
assert(numel(entries)==15,'gpenmpcShort:SetupContractCount','Exactly fifteen original recovery contract entries required.');
for k=1:6
    entries(end+1)=struct('name',sprintf('HIL_ACT_FUNC%d',k),'mav_type',6, ...
        'original_raw_bits_hex','00000000','target_raw_bits_hex',upper(dec2hex(uint32(100+k),8))); %#ok<AGROW>
end
before=receipt.before_parameters;after=receipt.after_parameters;writes=receipt.writes;
assert(numel(unique(string({before.name})))==167&&numel(unique(string({after.name})))==167 ...
    &&numel(unique(string({writes.name})))==21 ...
    &&isequal(sort(string({before.name})),sort(string({after.name}))) ...
    &&isequal(sort(string({writes.name})),sort(string({entries.name}))), ...
    'gpenmpcShort:SetupNames','No missing, duplicated or extra setup fields.');
for k=1:167
    b=before(k);name=char(b.name);a=after(strcmp({after.name},name));
    [bt,bb]=typedParameter(b,name);[at,ab]=typedParameter(a,name);
    assert(bt==at,'gpenmpcShort:SetupTypeChange','Original parameter type changed during setup.');i=find(strcmp({entries.name},name));
    if isempty(i)
        assert(strcmp(bb,ab),'gpenmpcShort:UnexpectedSetupMutation','Unexpected changed parameter: %s',name);
    else
        e=entries(i);w=writes(strcmp({writes.name},name));
        assert(bt==e.mav_type&&strcmpi(bb,e.original_raw_bits_hex)&&strcmpi(ab,e.target_raw_bits_hex) ...
            &&w.mav_type==bt&&strcmpi(w.before_raw_bits_hex,bb)&&strcmpi(w.target_raw_bits_hex,ab), ...
            'gpenmpcShort:SetupContractMismatch','Setup differs from exact original/target contract: %s',name);
        [ot,ob]=typedParameter(w.original,name);[et,eb]=typedParameter(w.echo,name);
        [rt,rb]=typedParameter(w.independent_readback,name);
        assert(ot==bt&&et==bt&&rt==bt&&strcmp(ob,bb)&&strcmp(eb,ab)&&strcmp(rb,ab), ...
            'gpenmpcShort:SetupTypedEcho','Original/echo/independent readback mismatch: %s',name);
    end
end
for k=1:15
    assert(any(strcmp({before.name},entries(k).name)),'gpenmpcShort:SetupMissingContract','A recovery contract entry is absent from the original parameter union.');
end
guards={'SYS_HITL',1;'SYS_AUTOSTART',6001;'MAV_TYPE',13;'CA_ROTOR_COUNT',6;'RA_CTRL_MODE',0};
for k=1:size(guards,1),checkInt(after,guards{k,1},guards{k,2});end
checkInt(receipt.guard_parameters,'COM_OBL_RC_ACT',4);
for k=1:8
    checkInt(after,sprintf('PWM_MAIN_FUNC%d',k),0);
    checkInt(receipt.aux_parameters,sprintf('PWM_AUX_FUNC%d',k),0);
end
for k=1:16
    checkInt(before,sprintf('HIL_ACT_FUNC%d',k),0);
    target=0;if k<=6,target=100+k;end
    checkInt(after,sprintf('HIL_ACT_FUNC%d',k),target);
end
receipt.source_path=source.source_path;receipt.source_sha256=source.source_sha256;
receipt.consumed_without_post_NoUI_parameter_setup=true;
end
function checkInt(rows,name,value)
p=rows(strcmp({rows.name},name));[t,b]=typedParameter(p,name);
assert(t==6&&strcmp(b,upper(dec2hex(typecast(int32(value),'uint32'),8))), ...
    'gpenmpcShort:SetupGuard','Original typed guard mismatch: %s',name);
end
function [t,b]=typedParameter(p,name)
assert(isstruct(p)&&isscalar(p)&&all(isfield(p,{'name','mav_type','raw_bits_hex'})) ...
    &&strcmp(char(p.name),name)&&isnumeric(p.mav_type)&&isscalar(p.mav_type) ...
    &&ismember(p.mav_type,[5 6 9]),'gpenmpcShort:SetupTypedRow','Original typed parameter row required: %s',name);
t=double(p.mav_type);b=upper(char(p.raw_bits_hex));
assert(~isempty(regexp(b,'^[0-9A-F]{8}$','once')),'gpenmpcShort:SetupBits','Original raw parameter identity must contain exactly eight hexadecimal digits.');
if t==9,assert(isfinite(typecast(uint32(hex2dec(b)),'single')),'gpenmpcShort:SetupFinite','Nonfinite REAL32 recovery values are not accepted.');end
end
function n=exactUid(value)
if isa(value,'uint64'),n=value;assert(isscalar(n));return,end
s=char(string(value));assert(~isempty(regexp(s,'^[0-9]+$','once')));n=uint64(0);
for c=s,d=uint64(c-'0');assert(n<=idivide(intmax('uint64')-d,uint64(10),'floor'));n=n*uint64(10)+d;end
end
function h=fileSha(path)
f=fopen(path,'rb');assert(f>=0);c=onCleanup(@()fclose(f));b=fread(f,Inf,'*uint8');clear c;h=bytesSha(b);
end
function h=bytesSha(bytes)
m=java.security.MessageDigest.getInstance('SHA-256');m.update(typecast(uint8(bytes(:)),'int8'));
h=upper(reshape(dec2hex(typecast(m.digest(),'uint8'),2).',1,[]));
end
function x=readLE(bytes,type)
x=typecast(uint8(bytes(:)),type);[~,~,e]=computer;if e=='B',x=swapbytes(x);end
end
function writeJson(path,value)
f=fopen(path,'w');assert(f>=0);c=onCleanup(@()fclose(f)); %#ok<NASGU>
fwrite(f,jsonencode(value,PrettyPrint=true),'char');
end
function lines=existingBoardStopLines(records)
text='';
for k=1:numel(records)
 r=records{k};if ~strcmp(r.topic,'SERIAL_CONTROL'),continue;end
 p=r.message.Payload;
 if p.device==10&&p.flags==1&&p.count<=70
  text=[text char(reshape(uint8(p.data(1:double(p.count))),1,[]))]; %#ok<AGROW>
 end
end
lines=regexp(text,'(?:LOCAL_CTX|LOCAL_AUTH|RFLY_LOCAL_RETAINED|RFLY_LOCAL_NESTED)[^\r\n]*','match');
lines=unique(lines,'stable');if numel(lines)>12,lines=lines(end-11:end);end
end
