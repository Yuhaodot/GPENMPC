function receipt=launch_m600_canonical_hil(planPath,operation,outputRoot)
% LAUNCH_M600_CANONICAL_HIL Stage helper for PowerShell orchestration.
% OFFLINE_VALIDATE opens no endpoint. Live operations require the plan,
% stage token and fresh serial preflight.
plan=jsondecode(fileread(planPath));operation=upper(string(operation));
assert(strcmp(plan.schema,'M600_CANONICAL_OUTER_PLAN_V1'),'m600check:OuterPlanSchema','Unexpected plan schema.');
isDelivery=strcmp(plan.execution_kind,'CANONICAL_DELIVERY');
root=fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(root,'host_runtime'),fullfile(root,'matlab_validation'), ...
    fullfile(root,'m600_coptersim','matlab_validation'),fullfile(root,'tools'));
assert(isfolder(outputRoot),'m600check:OuterOutputMissing','Fresh output directory is missing.');
for k=1:numel(plan.references)
    r=plan.references(k);
    info=dir(r.path);
    assert(isfile(r.path)&&isscalar(info)&&info.bytes==r.bytes&& ...
        strcmpi(m600check.fileSha256(r.path),r.sha256),'m600check:OuterReferenceMismatch','%s',r.label);
end
cfg=plan.runner_config;validateBounds(cfg);
hoverPreparation=[];
if isfield(plan,'virtual_sensor_mount_contract')&&strcmp(plan.virtual_sensor_mount_contract.schema, ...
        'VIRTUAL_SENSOR_MOUNT_CONTRACT_V2_AUTOCAL_OBSERVED')
    if strcmp(plan.execution_kind,'NATIVE_HOVER')||isDelivery
        assert(isfield(plan,'native_hover_preparation'),'m600check:ObservationContractOnly', ...
            'The calibration receipt alone cannot prepare a hover.');
        hoverPreparation=m600check.verifySensorAlignedHoverPreparation( ...
            plan.native_hover_preparation,plan.virtual_sensor_mount_contract);
        assert(hoverPreparation.passed,'m600check:HoverPreparationRejected','%s',hoverPreparation.failure);
    else
        assert(strcmp(plan.execution_kind,'INITIALIZATION_OBSERVATION_ONLY')&&operation~="RUN", ...
            'm600check:ObservationContractOnly','A flight-execution plan is required by this runner.');
    end
end
configRef=plan.references(strcmp({plan.references.label},'runner_config_source'));
expectedConfigSource=fullfile(root,'tools','make_m600_native_hover_config.m');
if isDelivery,expectedConfigSource=fullfile(root,'tools','make_m600_canonical_delivery_config.m');end
assert(isscalar(configRef)&&strcmp(configRef.path,expectedConfigSource), ...
    'm600check:RunnerConfigSourceMismatch','Current configuration source was not bound.');
if isDelivery
    expectedCfg=make_m600_canonical_delivery_config(cfg.plant_identity,cfg.outer_preflight_receipt);
else
    expectedCfg=make_m600_native_hover_config(cfg.plant_identity,cfg.outer_preflight_receipt);
end
if isfield(plan,'virtual_sensor_mount_contract'),expectedCfg.virtual_sensor_mount_contract=plan.virtual_sensor_mount_contract;end
if isfield(plan,'native_hover_preparation'),expectedCfg.native_hover_preparation=plan.native_hover_preparation;end
terrainCheck=[];
if isfield(plan,'terrain_diagnostic_runtime_contract')
    assert(isfield(plan,'require_terrain_diagnostic_extension')&& ...
        islogical(plan.require_terrain_diagnostic_extension)&&isscalar(plan.require_terrain_diagnostic_extension)&& ...
        plan.require_terrain_diagnostic_extension,'m600check:TerrainRuntimeFlag','Explicit diagnostic consumer required.');
    consumer=struct();
    for name={'model_dll','source_model','core_parameters'}
        entry=plan.references(strcmp({plan.references.label},name{1}));
        assert(isscalar(entry),'m600check:TerrainConsumerReference','Exact consumer reference missing.');
        consumer.(name{1})=rmfield(entry,'label');
    end
    consumer.require_terrain_diagnostic_extension=true;
    terrainCheck=m600check.validateTerrainDiagnosticRuntimeContract(plan.terrain_diagnostic_runtime_contract,consumer);
    assert(terrainCheck.passed&&terrainCheck.consumer_verified,'m600check:TerrainRuntimeContract','%s',terrainCheck.failure);
    assert(isfield(cfg,'require_terrain_diagnostic_extension')&& ...
        islogical(cfg.require_terrain_diagnostic_extension)&&isscalar(cfg.require_terrain_diagnostic_extension)&& ...
        cfg.require_terrain_diagnostic_extension&&isfield(cfg,'terrain_diagnostic_runtime_contract'), ...
        'm600check:TerrainRuntimeConfigFlag','Runner must explicitly require the typed diagnostic extension.');
    cfgTerrain=m600check.validateTerrainDiagnosticRuntimeContract(cfg.terrain_diagnostic_runtime_contract,consumer);
    assert(cfgTerrain.passed&&cfgTerrain.consumer_verified,'m600check:TerrainRuntimeConfigContract','%s',cfgTerrain.failure);
    expectedCfg.terrain_diagnostic_runtime_contract=plan.terrain_diagnostic_runtime_contract;
    expectedCfg.require_terrain_diagnostic_extension=true;
end
geometryCheck=[];
if isfield(plan,'temporary_allocator_geometry')
    geometryCheck=m600check.validateTemporaryAllocatorGeometry(plan.temporary_allocator_geometry);
    assert(geometryCheck.passed,'m600check:AllocatorGeometryContract','%s',geometryCheck.failure);
    expectedCfg.temporary_allocator_geometry=plan.temporary_allocator_geometry;
end
tuningCheck=[];
if isfield(plan,'native_hover_tuning')
    if isDelivery
        tuningCheck=validate_m600_delivery_native_tuning_contract(plan.native_hover_tuning);
    else
        tuningCheck=m600check.validateNativeHoverTuningContract(plan.native_hover_tuning);
    end
    assert(tuningCheck.passed,'m600check:NativeHoverTuningContract','%s',tuningCheck.failure);
    assert(~isempty(geometryCheck),'m600check:NativeHoverTuningGeometryRequired', ...
        'The exact tuning3 requires the separately bound geometry12 contract.');
    assert(isfield(cfg,'native_hover_tuning'),'m600check:NativeHoverTuningConfigMissing','Runner tuning contract missing.');
    if isDelivery
        cfgTuning=validate_m600_delivery_native_tuning_contract(cfg.native_hover_tuning);
    else
        cfgTuning=m600check.validateNativeHoverTuningContract(cfg.native_hover_tuning);
    end
    assert(cfgTuning.passed,'m600check:NativeHoverTuningConfigContract','%s',cfgTuning.failure);
    expectedCfg.native_hover_tuning=plan.native_hover_tuning;
end
if isDelivery
    % Compare ordered JSON/MATLAB service entries independently of array orientation.
    cfg.delivery_mission.service_ids=cfg.delivery_mission.service_ids(:);
    expectedCfg.delivery_mission.service_ids=expectedCfg.delivery_mission.service_ids(:);
end
assert(isequaln(orderfields(cfg),orderfields(expectedCfg)), ...
    'm600check:RunnerConfigValueMismatch','Use the declared native-hover configuration.');
receipt=struct('operation',char(operation),'passed',false,'hardware_actions',0,'hover_preparation_evidence',hoverPreparation);
receipt.terrain_runtime_derivation=terrainCheck;
if operation=="OFFLINE_VALIDATE"
    assert(exist('mavlinkdialect','file')~=0&&exist('udpport','file')~=0, ...
        'm600check:MatlabToolboxUnavailable','Required MATLAB toolboxes are unavailable.');
    policy=cfg.evaluation_policy;policy.clock_id='PRELAUNCH_POLICY_VALIDATION';
    policy.clock_mapping_evidence='PRELAUNCH_POLICY_VALIDATION_ONLY_NO_CLOCK_OBSERVED';
    policy.window_s=[0,cfg.hover_duration_s];
    m600check.evaluateThreeStream(struct('reference',struct([]),'estimate',struct([]),'truth',struct([])),policy);
    assert(~isempty(which('m600check.decodeCopterSimDiagnostics'))&& ...
        ~isempty(which('recover_m600_canonical_udp')),'m600check:OuterDependencyMissing','A required dependency is missing.');
    receipt.status='PASS_NO_NETWORK_NO_COM_CANONICAL_OUTER_MATLAB_PREFLIGHT';receipt.passed=true;
    write(fullfile(outputRoot,'MATLAB_OFFLINE_PREFLIGHT.json'),receipt);return
end
assert(plan.live_enabled,'m600check:NoExplicitLivePlan','This plan does not enable live execution.');
token=jsondecode(fileread(fullfile(outputRoot,'OUTER_STAGE_TOKEN.json')));
assert(strcmp(token.operation,operation)&&strcmpi(token.plan_sha256,m600check.fileSha256(planPath)), ...
    'm600check:OuterStageTokenMismatch','Outer operation or plan hash differs.');
expected=struct('uid',plan.board.uid,'board_version',plan.board.board_version,'commit',plan.board.commit);
if isfield(plan,'virtual_sensor_mount_contract'),expected.virtual_sensor_mount_contract=plan.virtual_sensor_mount_contract;end
if ~isempty(geometryCheck),expected.temporary_allocator_geometry=plan.temporary_allocator_geometry;end
if ~isempty(tuningCheck)
    expected.native_hover_tuning=plan.native_hover_tuning;
    expected.native_hover_tuning_phase='ORIGINAL';
    if operation=="SERIAL_POSTFLIGHT",expected.native_hover_tuning_phase='RESTORED';end
end
switch operation
    case "SERIAL_PREFLIGHT"
        assert(token.coptersim_owned_pid==0,'m600check:SerialWhileCopterSimOwnsCOM','CopterSim still owns COM.');
        receipt=m600_canonical_serial_preflight(fullfile(outputRoot,'SERIAL_PREFLIGHT.json'),expected);
    case "SERIAL_POSTFLIGHT"
        assert(token.coptersim_owned_pid==0,'m600check:SerialWhileCopterSimOwnsCOM','CopterSim still owns COM.');
        receipt=m600_canonical_serial_preflight(fullfile(outputRoot,'SERIAL_POSTFLIGHT.json'),expected);
    case {"RUN","INITIALIZATION_OBSERVE","UDP_RECOVERY"}
        assert(token.coptersim_owned_pid>0,'m600check:MissingOwnedSensorProcess','Owned sensor process is not recorded.');
        pre=jsondecode(fileread(fullfile(outputRoot,'SERIAL_PREFLIGHT.json')));
        assert(pre.passed&&pre.COM_closed&&strcmp(pre.uid,plan.board.uid)&& ...
            pre.board_version==plan.board.board_version&&strcmpi(pre.commit,plan.board.commit), ...
            'm600check:FreshSerialPreflightNotPassed','Fresh serial identity/safety preflight did not pass.');
        cfg.live_enabled=true;cfg.outer_preflight_pass=true;cfg.write_artifacts=true;
        cfg.outer_preflight_receipt=fullfile(outputRoot,'SERIAL_PREFLIGHT.json');
        cfg.plant_identity=plan.plant_identity;
        cfg.expected_uid=pre.uid;cfg.expected_board_version=pre.board_version;
        cfg.expected_flight_custom_version_hex=pre.flight_custom_version_hex;
        if operation=="RUN"
            if isDelivery
                receipt=run_m600_matlab_delivery_hil(fullfile(outputRoot,'INNER_MATLAB'),cfg);
            else
                assert(strcmp(plan.execution_kind,'NATIVE_HOVER'),'m600check:ExecutionKindMismatch');
                receipt=run_m600_matlab_hil(fullfile(outputRoot,'INNER_MATLAB'),cfg);
            end
        elseif operation=="INITIALIZATION_OBSERVE"
            assert(strcmp(plan.execution_kind,'INITIALIZATION_OBSERVATION_ONLY'), ...
                'm600check:ExecutionKindMismatch');
            receipt=observe_m600_copter_initialization(fullfile(outputRoot,'INITIALIZATION_MATLAB'),cfg);
        else
            cfg.allow_emergency_logical_force_disarm=logical(plan.safety.allow_emergency_logical_force_disarm);
            if ~isempty(geometryCheck)||~isempty(tuningCheck)
                assert(isfield(token,'independent_recovery_previous_owner_released')&& ...
                    isequal(token.independent_recovery_previous_owner_released,true), ...
                    'm600check:PreviousOwnerNotReleased','Independent recovery requires outer Job/UDP release proof.');
                cfg.independent_recovery_previous_owner_released=true;
            end
            receipt=recover_m600_canonical_udp(cfg);
            save(fullfile(outputRoot,'UDP_RECOVERY_RAW.mat'),'receipt','-v7.3');
            compact=receipt;compact.evidence=[];
            write(fullfile(outputRoot,'UDP_RECOVERY.json'),compact);
        end
    otherwise,error('m600check:UnknownOuterOperation','Unknown outer operation.');
end
end

function validateBounds(cfg)
% Use timings from the live configuration.
fields={'hover_altitude_m','hover_duration_s','prestream_s','preflight_deadline_s', ...
    'mode_timeout_s','arm_timeout_s','land_timeout_s','disarm_timeout_s','command_timeout_s', ...
    'poll_period_s','heartbeat_period_s','heartbeat_max_age_s','landed_max_age_s','state_max_age_s', ...
    'maximum_initial_origin_offset_m','abort_truth_speed_mps','abort_estimator_gap_m', ...
    'local_mavlink_port','remote_mavlink_port','truth_port','coptersim_time_port', ...
    'target_system','target_component','clock_max_rtt_s','clock_sync_samples', ...
    'clock_sync_period_s','clock_max_age_s','clock_max_uncertainty_s','clock_max_utc_drift_s', ...
    'clock_max_time_heartbeat_age_s','clock_max_time_heartbeat_lag_s', ...
    'maximum_truth_lag_s','maximum_raw_records'};
assert(all(isfield(cfg,fields))&&isfield(cfg,'evaluation_policy'),'m600check:LiveBoundMissing','Required live configuration field is missing.');
assert(isfield(cfg,'bound_provenance')&&isstruct(cfg.bound_provenance)&& ...
    isfield(cfg,'source_provenance')&&isstruct(cfg.source_provenance), ...
    'm600check:LiveBoundProvenanceMissing','Numerical provenance is missing.');
for k=1:numel(fields)
    value=cfg.(fields{k});assert(isnumeric(value)&&isreal(value)&&isscalar(value)&&isfinite(value)&&value>0, ...
        'm600check:LiveBoundInvalid','%s',fields{k});
end
assert(cfg.local_mavlink_port==14550&&cfg.remote_mavlink_port==18570&&cfg.truth_port==30101&& ...
    cfg.coptersim_time_port==20005&&cfg.target_system==1&&cfg.target_component==1, ...
    'm600check:OfficialEndpointIdentityChanged','Official live endpoint assignment changed.');
assert(isfield(cfg,'live_enabled')&&isfield(cfg,'outer_preflight_pass')&&isfield(cfg,'write_artifacts'));
end
function write(path,value)
assert(~isfile(path),'m600check:OuterArtifactExists','Existing evidence must not be overwritten.');
fid=fopen(path,'w','n','UTF-8');assert(fid>=0);guard=onCleanup(@()fclose(fid));
fprintf(fid,'%s\n',jsonencode(value,PrettyPrint=true));clear guard
end
