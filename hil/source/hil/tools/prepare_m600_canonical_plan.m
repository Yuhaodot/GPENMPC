function plan=prepare_m600_canonical_plan(input,outputPath)
% Prepare launch files from the supplied compiled, sensor and source records.
assert(~isfile(outputPath),'m600check:PlanExists','Choose an unused output path.');
root=fileparts(fileparts(mfilename('fullpath')));io=fullfile(root,'m600_coptersim','matlab_validation');
addpath(io,fullfile(root,'matlab_validation'),fullfile(root,'tools'));
required={'runtime_path','dll_build','dll_probe','dll_source_comparison','actual_sensor_receipt', ...
    'actual_forwarding_receipt','terrain_map','firmware','hil_profile','recovery_host_tests','plant_identity', ...
    'owned_job_host_tests'};
assert(all(isfield(input,required)),'m600check:PlanInputMissing','Final proof/runtime path is missing.');
build=jsondecode(fileread(input.dll_build));comparison=jsondecode(fileread(input.dll_source_comparison));
if isfield(input,'actual_api_receipt')
    apiReceipt=input.actual_api_receipt;
else
    apiReceipt=gpenmpc_external_path('host_api_receipt');
end
paths={
    'model_dll',build.dll;'dll_build',input.dll_build;'dll_probe',input.dll_probe;
    'dll_source_comparison',input.dll_source_comparison;'core_parameters',comparison.parameter_path;
    'source_model',build.source_model;'actual_sensor_receipt',input.actual_sensor_receipt;
    'actual_forwarding_receipt',input.actual_forwarding_receipt;
    'terrain_map',input.terrain_map;'firmware',input.firmware;'hil_profile',input.hil_profile;
    'terrain_heightfield',fullfile(fileparts(input.terrain_map),'LowGPU.png');
    'recovery_host_tests',input.recovery_host_tests;
    'owned_job_host_tests',input.owned_job_host_tests;
    'owned_job_source',fullfile(root,'tools','OwnedMatlabJob.cs');
    'host_load_source',fullfile(root,'tools','HostLoadSnapshot.cs');
    'copter_exe',fullfile(input.runtime_path,'CopterSim.exe');
    'matlab_exe',fullfile(matlabroot,'bin','matlab.exe');
    'outer_source',fullfile(root,'tools','run_m600_canonical_hil.ps1');
    'runner',fullfile(root,'tools','run_m600_matlab_hil.m');
    'recovery',fullfile(root,'tools','recover_m600_canonical_udp.m');
    'matlab_launcher',fullfile(root,'tools','launch_m600_canonical_hil.m');
    'serial_preflight',fullfile(root,'tools','m600_canonical_serial_preflight.m');
    'serial_link',fullfile(root,'host_runtime','+gpenmpcNative','MavlinkSerialLink.m');
    'sd_listing_inspector',fullfile(io,'+m600check','inspectSdDiagnosticListing.m');
    'runner_config_source',fullfile(root,'tools','make_m600_native_hover_config.m');
    'plan_builder',mfilename('fullpath')+".m";
    'sensor_aligned_plan_builder',fullfile(root,'tools','prepare_m600_sensor_aligned_hover.m');
    'adapter',fullfile(io,'+m600check','makeM600CopterSimIo.m');
    'current_native_telemetry_checker',fullfile(io,'+m600check','evaluateNativeHoverTelemetry.m');
    'utc_pair_sampler',fullfile(io,'+m600check','sampleUtcMonotonicPair.m');
    'utc_drift_evaluator',fullfile(io,'+m600check','evaluateUtcDrift.m');
    'utc_pair_host_tests',gpenmpc_external_path('utc_pair_validation_receipt');
    'copter_init_observer_source',fullfile(root,'tools','OwnedCopterInitObserver.cs');
    'initialization_observer',fullfile(root,'tools','observe_m600_copter_initialization.m');
    'diagnostic_decoder',fullfile(root,'matlab_validation','+m600check','decodeCopterSimDiagnostics.m');
    'diagnostic_observer',fullfile(io,'+m600check','updateCopterSimDiagnostic.m');
    'diagnostic_snapshot',fullfile(io,'+m600check','copterSimDiagnosticSnapshot.m');
    'truth_decoder',fullfile(io,'+m600check','decodeTruthPacket.m');
    'three_stream_evaluator',fullfile(io,'+m600check','evaluateThreeStream.m');
    'three_stream_accumulator',fullfile(io,'+m600check','appendThreeStreamSample.m');
    'hash_helper',fullfile(io,'+m600check','fileSha256.m');
    'output_evidence_helper',fullfile(io,'+m600check','evaluateVirtualOutputEvidence.m');
    'actual_api_receipt',apiReceipt};
if isfield(input,'virtual_sensor_mount_contract')
    assert(all(isfield(input,{'sensor_mount_host_tests','sensor_frame_model_receipt'})),'m600check:MountEvidenceMissing');
    paths(end+1,:)={'sensor_mount_checker',fullfile(io,'+m600check','verifyVirtualSensorMountContract.m')};
    paths(end+1,:)={'sensor_frame_encoder',fullfile(io,'+m600check','encodeHilSensorLevelFrame.m')};
    paths(end+1,:)={'sensor_mount_host_tests',input.sensor_mount_host_tests};
    paths(end+1,:)={'sensor_frame_model_receipt',input.sensor_frame_model_receipt};
    if isfield(input,'sensor_calibration_readonly')
        paths(end+1,:)={'sensor_calibration_readonly',input.sensor_calibration_readonly};
    end
end
if isfield(input,'copter_init_observer_tests')
    paths(end+1,:)={'copter_init_observer_tests',input.copter_init_observer_tests};
end
if isfield(input,'initialization_observer_tests')
    paths(end+1,:)={'initialization_observer_tests',input.initialization_observer_tests};
end
if isfield(input,'copter_barrier_tests')
    paths(end+1,:)={'copter_barrier_source',fullfile(root,'tools','CopterInitializationBarrier.cs')};
    paths(end+1,:)={'copter_barrier_tests',input.copter_barrier_tests};
end
if isfield(input,'copter_barrier_outer_tests')
    paths(end+1,:)={'copter_barrier_outer_tests',input.copter_barrier_outer_tests};
end
if isfield(input,'native_hover_preparation')
    assert(all(isfield(input,{'hover_preparation_tests','native_telemetry_tests'})),'m600check:HoverPreparationTestsMissing');
    ht=jsondecode(fileread(input.hover_preparation_tests));
    assert(ht.passed&&ht.case_count==ht.cases_passed&&ht.case_count>=26&& ...
        ht.COM_UDP_board_model_process_actions==0&&~ht.flight_admission&& ...
        strcmpi(ht.sources.helper.sha256,m600check.fileSha256(fullfile(io,'+m600check','verifySensorAlignedHoverPreparation.m'))), ...
        'm600check:HoverPreparationTestsFailed','Complete current evidence helper negative tests required.');
    paths(end+1,:)={'hover_preparation_checker',fullfile(io,'+m600check','verifySensorAlignedHoverPreparation.m')};
    paths(end+1,:)={'hover_preparation_tests',input.hover_preparation_tests};
    paths(end+1,:)={'current_native_telemetry_tests',input.native_telemetry_tests};
    for hi=1:numel(input.native_hover_preparation.references)
        hp=input.native_hover_preparation.references(hi);
        paths(end+1,:)={['hover_preparation_' hp.label],hp.path}; %#ok<AGROW>
    end
end
if isfield(input,'temporary_allocator_geometry')
    gc=m600check.validateTemporaryAllocatorGeometry(input.temporary_allocator_geometry);
    assert(gc.passed,'m600check:AllocatorGeometryContract','%s',gc.failure);
    assert(all(isfield(input,{'geometry_host_tests','geometry_api_tests'})),'m600check:GeometryTestsMissing');
    gt=jsondecode(fileread(input.geometry_host_tests));ga=jsondecode(fileread(input.geometry_api_tests));
    assert(gt.passed&&gt.checks_total==gt.checks_passed&&gt.hardware_actions==0&& ...
        ga.passed&&ga.checks_total==ga.checks_passed&&ga.COM_open==0&& ...
        strcmpi(ga.adapter_sha256,m600check.fileSha256(fullfile(io,'+m600check','makeM600CopterSimIo.m'))), ...
        'm600check:GeometryHostTestsFailed');
    for si=1:numel(gt.sources)
        source=gt.sources(si);
        assert(strcmpi(source.sha256,m600check.fileSha256(source.path)),'m600check:GeometryHostSourceChanged');
        paths(end+1,:)={['geometry_host_source_' num2str(si)],source.path}; %#ok<AGROW>
    end
    paths(end+1,:)={'geometry_host_tests',input.geometry_host_tests};
    paths(end+1,:)={'geometry_api_tests',input.geometry_api_tests};
    fields=fieldnames(input.temporary_allocator_geometry.bindings);
    for gi=1:numel(fields)
        binding=input.temporary_allocator_geometry.bindings.(fields{gi});
        paths(end+1,:)={['geometry_' fields{gi}],binding.path}; %#ok<AGROW>
    end
end
terrainCheck=[];
if isfield(input,'terrain_diagnostic_runtime_contract')
    assert(all(isfield(input,{'require_terrain_diagnostic_extension','terrain_runtime_host_tests', ...
        'native_hover_preparation','virtual_sensor_mount_contract'}))&& ...
        logicalTrue(input.require_terrain_diagnostic_extension),'m600check:TerrainExtensionFlagRequired', ...
        'Terrain derivation requires scalar logical true, completed HOST tests and the unchanged observation/mount context.');
    terrainCheck=m600check.validateTerrainDiagnosticRuntimeContract(input.terrain_diagnostic_runtime_contract);
    assert(terrainCheck.passed,'m600check:TerrainRuntimeContract','%s',terrainCheck.failure);
    tr=jsondecode(fileread(input.terrain_runtime_host_tests));
    assert(strcmp(tr.schema,'HOST_TERRAIN_DIAGNOSTIC_RUNTIME_CONTRACT_TESTS_V1')&& ...
        logicalTrue(tr.passed)&&tr.checks_total>0&&tr.checks_total==tr.checks_passed&&all([tr.checks.passed])&& ...
        isempty(tr.failure)&&tr.COM_open==0&&tr.UDP_open==0&&tr.board_actions==0&& ...
        ~tr.model_loaded&&~tr.DLL_loaded&&~tr.runtime_installed&&~tr.live_or_flight_claim&& ...
        strcmpi(tr.tested_sources.builder_sha256,m600check.fileSha256(which('m600check.buildTerrainDiagnosticRuntimeContract')))&& ...
        strcmpi(tr.tested_sources.checker_sha256,m600check.fileSha256(which('m600check.validateTerrainDiagnosticRuntimeContract')))&& ...
        strcmpi(tr.tested_sources.tests_sha256,m600check.fileSha256(fullfile(root,'tools','run_terrain_diagnostic_runtime_contract_tests.m'))), ...
        'm600check:TerrainRuntimeHostTests','Terrain runtime tests are incomplete, failed or claim non-HOST actions.');
    tested=m600check.validateTerrainDiagnosticRuntimeContract(tr.contract);
    assert(tested.passed,'m600check:TerrainRuntimeTestContract','%s',tested.failure);
    paths(end+1,:)={'terrain_runtime_host_tests',input.terrain_runtime_host_tests};
    terrainSources={ ...
        'terrain_runtime_builder',fullfile(io,'+m600check','buildTerrainDiagnosticRuntimeContract.m'); ...
        'terrain_runtime_checker',fullfile(io,'+m600check','validateTerrainDiagnosticRuntimeContract.m'); ...
        'terrain_runtime_tests_source',fullfile(root,'tools','run_terrain_diagnostic_runtime_contract_tests.m'); ...
        'terrain_model_prepare_source',fullfile(root,'tools','prepare_terrain_diagnostic_model.m'); ...
        'terrain_source_replay_source',fullfile(root,'tools','run_terrain_diagnostic_source_replay.m'); ...
        'terrain_dll_probe_source',fullfile(root,'tools','probe_terrain_diagnostic_dll.cpp'); ...
        'terrain_dll_probe_wrapper',fullfile(root,'tools','run_terrain_diagnostic_dll_probe.m'); ...
        'terrain_extension_tests_source',fullfile(root,'tools','run_m600_terrain_diagnostic_extension_tests.m'); ...
        'terrain_fault_tests_source',fullfile(root,'tools','run_m600_terrain_fault_reason_tests.m'); ...
        'terrain_observer_tests_source',fullfile(root,'tools','run_terrain_observer_tests.m')};
    paths=[paths;terrainSources];
    for ti=1:numel(input.terrain_diagnostic_runtime_contract.bindings)
        binding=input.terrain_diagnostic_runtime_contract.bindings(ti);
        paths(end+1,:)={['terrain_runtime_' binding.label],binding.path}; %#ok<AGROW>
    end
else
    assert(~any(isfield(input,{'require_terrain_diagnostic_extension','terrain_runtime_host_tests'})), ...
        'm600check:TerrainRuntimeContractRequired','Terrain flags and tests cannot appear without the explicit derivation contract.');
end
if isfield(input,'native_hover_tuning')
    assert(isfield(input,'temporary_allocator_geometry')&& ...
        all(isfield(input,{'tuning_contract_tests','tuning_transfer_tests','tuning_union_tests','tuning_api_tests'})), ...
        'm600check:TuningPlanEvidenceMissing','Tuning requires geometry12 and all four completed HOST/API test receipts.');
    tuningCheck=m600check.validateNativeHoverTuningContract(input.native_hover_tuning);
    assert(tuningCheck.passed,'m600check:TuningPlanContract','%s',tuningCheck.failure);
    tc=jsondecode(fileread(input.tuning_contract_tests));tt=jsondecode(fileread(input.tuning_transfer_tests));
    tu=jsondecode(fileread(input.tuning_union_tests));ta=jsondecode(fileread(input.tuning_api_tests));
    assert(strcmp(tc.schema,'HOST_NATIVE_HOVER_TUNING_CONTRACT_TESTS_V1')&&logicalTrue(tc.passed)&& ...
        tc.case_count>0&&tc.case_count==tc.cases_passed&&all([tc.tests.passed])&& ...
        tc.hardware_actions==0&&tc.COM_UDP_actions==0&&tc.parameter_writes==0&&~tc.authority_granted, ...
        'm600check:TuningContractHostTests','Tuning contract tests failed or have incomplete/non-HOST evidence.');
    tested=m600check.validateNativeHoverTuningContract(tc.contract);
    assert(tested.passed,'m600check:TuningTestContract','%s',tested.failure);
    assert(strcmp(tt.schema,'HOST_NATIVE_HOVER_TUNING_TRANSFER_TESTS_V1')&&logicalTrue(tt.passed)&& ...
        tt.checks_total>0&&tt.checks_total==tt.checks_passed&&all([tt.cases.passed])&& ...
        tt.COM_open==0&&tt.UDP_open==0&&tt.board_actions==0&&~tt.real_adapter_constructed&& ...
        strcmpi(tt.source_sha256,m600check.fileSha256(fullfile(io,'+m600check','transferNativeHoverTuning.m')))&& ...
        strcmpi(tt.test_sha256,m600check.fileSha256(fullfile(root,'tools','run_native_hover_tuning_transfer_tests.m'))), ...
        'm600check:TuningTransferHostTests','Tuning transfer tests or their current source hashes are invalid.');
    assert(strcmp(tu.schema,'HOST_NATIVE_HOVER_PARAMETER_UNION_TESTS_V1')&&logicalTrue(tu.passed)&& ...
        tu.case_count>0&&tu.case_count==tu.cases_passed&&all([tu.tests.passed])&& ...
        tu.hardware_actions==0&&tu.COM_UDP_actions==0&&tu.parameter_writes==0&&~tu.real_adapter_constructed&& ...
        strcmpi(tu.source_sha256,m600check.fileSha256(fullfile(io,'+m600check','verifyNativeHoverParameterUnion.m')))&& ...
        strcmpi(tu.test_sha256,m600check.fileSha256(fullfile(root,'tools','run_native_hover_parameter_union_tests.m'))), ...
        'm600check:TuningUnionHostTests','The full parameter-union tests or their current source hashes are invalid.');
    assert(strcmp(ta.classification,'HOST_SYNTHETIC_THREE_PARAMETER_API')&&logicalTrue(ta.passed)&& ...
        ta.checks_total>0&&ta.checks_total==ta.checks_passed&&all([ta.checks.passed])&& ...
        isempty(ta.failure)&&ta.hardware_actions==0&&ta.COM_open==0&&ta.PX4_access==0&&ta.real_parameter_writes==0&& ...
        ta.cleanup.ports_rebound&&isempty(ta.cleanup.errors)&& ...
        strcmpi(ta.adapter_sha256,m600check.fileSha256(fullfile(io,'+m600check','makeM600CopterSimIo.m')))&& ...
        strcmpi(ta.fixture_sha256,m600check.fileSha256(fullfile(root,'tools','run_native_hover_tuning_loopback_tests.m'))), ...
        'm600check:TuningActualApiTests','Tuning API evidence, endpoint cleanup or current adapter/fixture identity is invalid.');
    % Use the shared union checker to verify the 138 baseline contract entries.
    originalRows=input.native_hover_tuning.unchanged_guard_entries(:);
    for ti=1:numel(input.native_hover_tuning.entries)
        e=input.native_hover_tuning.entries(ti);
        originalRows(end+1)=struct('name',e.name,'mav_type',e.mav_type,'raw_bits_hex',e.original_raw_bits_hex); %#ok<AGROW>
    end
    unionCheck=m600check.verifyNativeHoverParameterUnion(@(name) originalTypedRow(originalRows,name), ...
        input.native_hover_tuning,input.temporary_allocator_geometry,'ORIGINAL');
    assert(unionCheck.passed,'m600check:TuningGeometryUnion','%s',unionCheck.failure);
    tuningSources={ ...
        'tuning_builder',fullfile(io,'+m600check','buildNativeHoverTuningContract.m'); ...
        'tuning_checker',fullfile(io,'+m600check','validateNativeHoverTuningContract.m'); ...
        'tuning_transfer',fullfile(io,'+m600check','transferNativeHoverTuning.m'); ...
        'tuning_union_checker',fullfile(io,'+m600check','verifyNativeHoverParameterUnion.m'); ...
        'tuning_contract_tests_source',fullfile(root,'tools','run_native_hover_tuning_contract_tests.m'); ...
        'tuning_transfer_tests_source',fullfile(root,'tools','run_native_hover_tuning_transfer_tests.m'); ...
        'tuning_union_tests_source',fullfile(root,'tools','run_native_hover_parameter_union_tests.m'); ...
        'tuning_api_tests_source',fullfile(root,'tools','run_native_hover_tuning_loopback_tests.m')};
    paths=[paths;tuningSources];
    for name={'tuning_contract_tests','tuning_transfer_tests','tuning_union_tests','tuning_api_tests'}
        paths(end+1,:)={name{1},input.(name{1})}; %#ok<AGROW>
    end
    names=fieldnames(input.native_hover_tuning.bindings);
    for ti=1:numel(names)
        binding=input.native_hover_tuning.bindings.(names{ti});
        paths(end+1,:)={['tuning_' names{ti}],binding.path}; %#ok<AGROW>
    end
    for ti=1:numel(input.native_hover_tuning.source_provenance)
        paths(end+1,:)={['tuning_provenance_' num2str(ti)],input.native_hover_tuning.source_provenance(ti).path}; %#ok<AGROW>
    end
else
    assert(~any(isfield(input,{'tuning_contract_tests','tuning_transfer_tests','tuning_union_tests','tuning_api_tests'})), ...
        'm600check:TuningPlanContractRequired','Tuning test receipts cannot appear without a tuning contract.');
end
refs=struct('label',{},'path',{},'bytes',{},'sha256',{});
for k=1:size(paths,1)
    path=char(paths{k,2});info=dir(path);
    assert(isfile(path)&&isscalar(info),'m600check:PlanReferenceMissing','%s',path);
    refs(end+1)=struct('label',paths{k,1},'path',path,'bytes',info.bytes,'sha256',m600check.fileSha256(path)); %#ok<AGROW>
end
assert(numel(unique({refs.label}))==numel(refs),'m600check:DuplicatePlanLabel', ...
    'Every plan reference label must be unique.');
if ~isempty(terrainCheck)
    consumer=struct('model_dll',unlabel(refs,'model_dll'),'source_model',unlabel(refs,'source_model'), ...
        'core_parameters',unlabel(refs,'core_parameters'),'require_terrain_diagnostic_extension',true);
    terrainCheck=m600check.validateTerrainDiagnosticRuntimeContract(input.terrain_diagnostic_runtime_contract,consumer);
    assert(terrainCheck.passed&&terrainCheck.consumer_verified&&~terrainCheck.new_live_credit&&~terrainCheck.flight_admission, ...
        'm600check:TerrainRuntimeConsumer','%s',terrainCheck.failure);
    oldCheck=m600check.verifySensorAlignedHoverPreparation(input.native_hover_preparation,input.virtual_sensor_mount_contract);
    assert(oldCheck.passed,'m600check:TerrainOldObservation','%s',oldCheck.failure);
    rb=input.native_hover_preparation.runtime_binding;
    for name={'sensor_frame_encoder','hil_profile','firmware'}
        r=unlabel(refs,name{1});
        assert(strcmpi(r.sha256,rb.([name{1} '_sha256'])), ...
            'm600check:TerrainNonDiagnosticIdentityChanged','%s is not the old observation identity.',name{1});
    end
    for pair={'model_dll','source_model';'old_dll','old_model'}
        parent=input.terrain_diagnostic_runtime_contract.bindings;
        parent=parent(strcmp({parent.label},pair{2}));
        assert(isscalar(parent)&&strcmpi(parent.sha256,rb.([pair{1} '_sha256'])), ...
            'm600check:TerrainObservationParentMismatch','%s parent differs from the reference observation.',pair{1});
    end
end
% Set process-liveness limits.
bounds=struct('copter_start_timeout_s',30,'matlab_startup_timeout_s',120, ...
    'serial_child_timeout_s',300,'matlab_child_timeout_s',420,'recovery_child_timeout_s',300, ...
    'process_exit_timeout_s',15);
field=fieldnames(bounds);basis={
    '30s: existing 15s no-board probe already starts stably; bounded startup margin';
    '120s: measured current MATLAB startup seconds, with license/process-launch allowance';
    '300s: MATLAB startup 120s + 29 typed parameter reads at 3s + identity/status observations';
    '420s: startup 120s + pre30 + stream2.5 + mode8 + arm12 + hover20 + land30 + disarm8 + finally allowance';
    '300 s startup and identity/state/mapping recovery bound; bridge stop requires verified safety.';
    '15s: owned no-board process and endpoints have been observed to release; bounded exit wait'};
provenance=struct('field',{},'reference_label',{},'basis',{});
for k=1:numel(field),provenance(end+1)=struct('field',['outer.' field{k}], ...
        'reference_label','plan_builder','basis',['HOST_PROCESS_ENGINEERING_BOUND: ' basis{k}]);end %#ok<AGROW>
live=false;if isfield(input,'live_enabled'),live=logical(input.live_enabled);end
cfg=make_m600_native_hover_config(input.plant_identity,'ASSIGNED_FROM_FRESH_OUTER_SERIAL_PREFLIGHT');
if isfield(input,'virtual_sensor_mount_contract'),cfg.virtual_sensor_mount_contract=input.virtual_sensor_mount_contract;end
if isfield(input,'native_hover_preparation'),cfg.native_hover_preparation=input.native_hover_preparation;end
if isfield(input,'temporary_allocator_geometry'),cfg.temporary_allocator_geometry=input.temporary_allocator_geometry;end
if isfield(input,'native_hover_tuning'),cfg.native_hover_tuning=input.native_hover_tuning;end
if ~isempty(terrainCheck)
    cfg.terrain_diagnostic_runtime_contract=input.terrain_diagnostic_runtime_contract;
    cfg.require_terrain_diagnostic_extension=true;
end
kind='NATIVE_HOVER';if isfield(input,'execution_kind'),kind=char(input.execution_kind);end
assert(any(strcmp(kind,{'NATIVE_HOVER','INITIALIZATION_OBSERVATION_ONLY'})), ...
    'm600check:PlanKind','Unknown execution purpose.');
if isfield(input,'virtual_sensor_mount_contract')&&strcmp(input.virtual_sensor_mount_contract.schema, ...
        'VIRTUAL_SENSOR_MOUNT_CONTRACT_V2_AUTOCAL_OBSERVED')
    if strcmp(kind,'NATIVE_HOVER')
        assert(isfield(input,'native_hover_preparation'),'m600check:ObservationContractOnly', ...
            'Observed calibration requires separate completed sensor/clock/safety evidence before hover preparation.');
        hp=m600check.verifySensorAlignedHoverPreparation(input.native_hover_preparation,input.virtual_sensor_mount_contract);
        assert(hp.passed,'m600check:HoverPreparationRejected','%s',hp.failure);
        rb=input.native_hover_preparation.runtime_binding;
        for binding={'model_dll','source_model','sensor_frame_encoder','hil_profile','firmware'}
            rr=refs(strcmp({refs.label},binding{1}));
            if ~isempty(terrainCheck)&&any(strcmp(binding{1},{'model_dll','source_model'}))
                % Validate the diagnostic derivative separately from the sensor-frame observation.
                oldLabel='old_dll';if strcmp(binding{1},'source_model'),oldLabel='old_model';end
                old=input.terrain_diagnostic_runtime_contract.bindings;
                old=old(strcmp({old.label},oldLabel));
                assert(isscalar(old)&&strcmpi(old.sha256,rb.([binding{1} '_sha256'])), ...
                    'm600check:TerrainObservationParentMismatch','%s parent differs from the reference observation.',binding{1});
                continue
            end
            assert(isscalar(rr)&&strcmpi(rr.sha256,rb.([binding{1} '_sha256'])), ...
                'm600check:HoverModelBindingMismatch','%s differs from the completed observation.',binding{1});
        end
    else
        assert(strcmp(kind,'INITIALIZATION_OBSERVATION_ONLY'),'m600check:ObservationContractOnly');
    end
end
plan=struct('schema','M600_CANONICAL_OUTER_PLAN_V1','live_enabled',live,'scene_name','LowGPU', ...
    'execution_kind',kind, ...
    'runtime_path',input.runtime_path,'references',refs,'plant_identity',input.plant_identity, ...
    'runner_config',cfg,'outer_bounds',bounds,'bound_provenance',provenance, ...
    'board',struct('uid',gpenmpc_device_identity('uid'),'board_version',56,'commit', ...
        '6ea3539157ca358c70a515878b77077af7d4611d','endpoint','COM3','baud',921600,'RA_CTRL_MODE',0), ...
    'safety',struct('usb_only',true,'propulsion_and_actuators_isolated',true, ...
        'allow_emergency_logical_force_disarm',true, ...
        'physical_output_actions',0,'flash_reboot_bootloader_actions',0, ...
        'reboot_count_scope','HOST_REQUESTS_WITH_SEPARATE_VENDOR_LIFECYCLE_COUNTS', ...
        'vendor_lifecycle_note','CopterSim may restart PX4 during HITL initialization; retain each source epoch and apply admission checks to the current epoch.'));
if isfield(input,'virtual_sensor_mount_contract'),plan.virtual_sensor_mount_contract=input.virtual_sensor_mount_contract;end
if isfield(input,'native_hover_preparation'),plan.native_hover_preparation=input.native_hover_preparation;end
if isfield(input,'temporary_allocator_geometry'),plan.temporary_allocator_geometry=input.temporary_allocator_geometry;end
if isfield(input,'native_hover_tuning'),plan.native_hover_tuning=input.native_hover_tuning;end
if ~isempty(terrainCheck)
    plan.terrain_diagnostic_runtime_contract=input.terrain_diagnostic_runtime_contract;
    plan.require_terrain_diagnostic_extension=true;
end
if isfield(input,'copter_barrier_tests')
    assert(strcmp(kind,'INITIALIZATION_OBSERVATION_ONLY')||isfield(input,'native_hover_preparation'), ...
        'm600check:BarrierScope','Hover requires independently bound completed ground observation.');
    plan.copter_initialization_barrier=struct('enabled',true,'deadline_s',60, ...
        'maximum_vendor_reboots',1,'scope','DISARMED_OBSERVER_START', ...
        'initialization_prefix_scientific_credit',0,'subsequent_observation_duration_s',45, ...
        'provenance','Vendor lifecycle startup limit: 60 s, including a 20 s engineering allowance. Retain the full initialization and health log.');
    if strcmp(kind,'NATIVE_HOVER')
        plan.copter_initialization_barrier.scope='VENDOR_PREFIX_BEFORE_NATIVE_HOVER_PREFLIGHT';
        plan.copter_initialization_barrier.subsequent_observation_duration_s=0;
        plan.copter_initialization_barrier.fresh_hover_preflight_required=true;
    end
end
folder=fileparts(outputPath);if ~isfolder(folder),mkdir(folder);end
fid=fopen(outputPath,'w','n','UTF-8');assert(fid>=0);guard=onCleanup(@()fclose(fid));
fprintf(fid,'%s\n',jsonencode(plan,PrettyPrint=true));clear guard
end
function yes=logicalTrue(value),yes=islogical(value)&&isscalar(value)&&value;end
function r=unlabel(refs,label)
r=refs(strcmp({refs.label},label));assert(isscalar(r),'m600check:PlanBindingMissing','%s',label);
r=rmfield(r,'label');
end
function q=originalTypedRow(rows,name)
q=rows(strcmp({rows.name},name));assert(isscalar(q),'m600check:TuningOriginalMissing','%s',name);
bits=uint32(hex2dec(q.raw_bits_hex));
if q.mav_type==9,q.decoded=double(typecast(bits,'single'));
elseif q.mav_type==6,q.decoded=double(typecast(bits,'int32'));
else,error('m600check:TuningOriginalType','Unsupported original type.');end
end
