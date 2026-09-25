[CmdletBinding()]
param(
    [Parameter(Mandatory=$true)][string]$PlanPath,
    [string]$OutputRoot,
    [switch]$Live
)
# Default to offline validation; live execution requires the current plan.
# Emergency force-disarm is available only at the end of UDP recovery.
# Keep the sensor process running when safety is unknown.
$ErrorActionPreference='Stop'
$buildRoot=Split-Path -Parent $PSScriptRoot
$device=Get-Content -LiteralPath $(if($env:GPENMPC_DEVICE_CONFIG){$env:GPENMPC_DEVICE_CONFIG}else{Join-Path (Split-Path (Split-Path $buildRoot -Parent) -Parent) 'local/device.json'}) -Raw | ConvertFrom-Json
if($device.uid -isnot [string] -or $device.uid -notmatch '^[1-9][0-9]{0,19}$'){throw 'Configure the exact device UID in GPENMPC_DEVICE_CONFIG or local/device.json.'}
$parsedUid=[uint64]::Parse($device.uid)
if (-not $OutputRoot) {
    $OutputRoot=Join-Path $buildRoot ('evidence\CANONICAL_HIL_OUTER_' + (Get-Date -Format 'yyyyMMdd_HHmmss_fff'))
}
$OutputRoot=[IO.Path]::GetFullPath($OutputRoot)
if (Test-Path -LiteralPath $OutputRoot) { throw "Fresh output path required: $OutputRoot" }
$plan=Get-Content -LiteralPath $PlanPath -Raw | ConvertFrom-Json
if ($plan.schema -ne 'M600_CANONICAL_OUTER_PLAN_V1') {throw 'Unexpected plan schema.'}
if($plan.execution_kind -notin @('NATIVE_HOVER','INITIALIZATION_OBSERVATION_ONLY','CANONICAL_DELIVERY')){throw 'Unknown explicit execution purpose.'}
$hasDelivery=$plan.execution_kind -eq 'CANONICAL_DELIVERY'
$hasHoverPreparation=$plan.execution_kind -eq 'NATIVE_HOVER' -and
    $null -ne $plan.native_hover_preparation -and
    $plan.native_hover_preparation.schema -eq 'M600_SENSOR_ALIGNED_NATIVE_HOVER_PREPARATION_V1'
$hasTerrainDerivation=$null -ne $plan.terrain_diagnostic_runtime_contract
$hasHoverTuning=$null -ne $plan.native_hover_tuning
if($hasTerrainDerivation -and (-not $hasHoverPreparation -or
    $plan.require_terrain_diagnostic_extension -isnot [bool] -or -not $plan.require_terrain_diagnostic_extension -or
    $plan.terrain_diagnostic_runtime_contract.require_terrain_diagnostic_extension -isnot [bool] -or
    -not $plan.terrain_diagnostic_runtime_contract.require_terrain_diagnostic_extension)) {
    throw 'New diagnostic DLL requires an explicit typed consumer and parent observation provenance.'
}
if($hasHoverTuning -and ((-not $hasHoverPreparation -and -not $hasDelivery) -or $null -eq $plan.temporary_allocator_geometry)) {
    throw 'Exact tuning3 requires native hover plus the separate geometry12 contract.'
}
if($plan.virtual_sensor_mount_contract.schema -eq 'VIRTUAL_SENSOR_MOUNT_CONTRACT_V2_AUTOCAL_OBSERVED' -and
   $plan.execution_kind -ne 'INITIALIZATION_OBSERVATION_ONLY' -and -not $hasHoverPreparation -and -not $hasDelivery) {
    throw 'Calibration observation alone cannot authorize the flight runner.'
}
$planHash=(Get-FileHash -LiteralPath $PlanPath -Algorithm SHA256).Hash
$refs=@{}
foreach($ref in $plan.references) {
    if ($refs.ContainsKey([string]$ref.label)) {throw "Duplicate reference label: $($ref.label)"}
    $item=Get-Item -LiteralPath $ref.path -ErrorAction Stop
    if($item.PSIsContainer -or $item.Length -ne [long]$ref.bytes -or
       (Get-FileHash -LiteralPath $item.FullName -Algorithm SHA256).Hash -ne $ref.sha256) {
        throw "Byte/SHA binding mismatch: $($ref.label)"
    }
    $refs[[string]$ref.label]=$ref
}
$required=@('model_dll','dll_build','dll_probe','dll_source_comparison','core_parameters',
    'runner','adapter','diagnostic_decoder','diagnostic_observer','diagnostic_snapshot',
    'recovery','matlab_launcher','serial_preflight','copter_exe','matlab_exe','firmware',
    'hil_profile','recovery_host_tests','runner_config_source','terrain_map','outer_source','hash_helper',
    'output_evidence_helper','actual_api_receipt','actual_sensor_receipt','serial_link',
    'truth_decoder','three_stream_evaluator','three_stream_accumulator','actual_forwarding_receipt','terrain_heightfield',
    'owned_job_source','owned_job_host_tests','host_load_source',
    'utc_pair_sampler','utc_drift_evaluator','utc_pair_host_tests','copter_init_observer_source','initialization_observer','sd_listing_inspector')
foreach($label in $required) {if(-not $refs.ContainsKey($label)){throw "Missing binding: $label"}}
$terrainRefs=@{}
if($hasTerrainDerivation) {
    if($plan.terrain_diagnostic_runtime_contract.schema -ne 'M600_TERRAIN_DIAGNOSTIC_RUNTIME_CONTRACT_V1') {
        throw 'Unexpected diagnostic derivation schema.'
    }
    foreach($e in $plan.terrain_diagnostic_runtime_contract.bindings) {
        if($terrainRefs.ContainsKey([string]$e.label)){throw 'Duplicate terrain derivation binding.'}
        $same=@($plan.references|Where-Object {$_.path -eq $e.path -and $_.bytes -eq $e.bytes -and $_.sha256 -eq $e.sha256})
        if($same.Count -eq 0){throw "Unbound terrain derivation source: $($e.label)"}
        $terrainRefs[[string]$e.label]=$e
    }
    foreach($pair in @(@('model_dll','new_dll'),@('source_model','new_model'),@('core_parameters','new_parameters'))) {
        if($refs[$pair[0]].sha256 -ne $terrainRefs[$pair[1]].sha256){throw 'Current diagnostic consumer identity differs.'}
    }
    foreach($label in @('terrain_runtime_host_tests','terrain_runtime_builder','terrain_runtime_checker','terrain_runtime_tests_source')) {
        if(-not $refs.ContainsKey($label)){throw "Missing current terrain HOST tests: $label"}
    }
    $tr=Get-Content -LiteralPath $refs.terrain_runtime_host_tests.path -Raw|ConvertFrom-Json
    if($tr.schema -ne 'HOST_TERRAIN_DIAGNOSTIC_RUNTIME_CONTRACT_TESTS_V1' -or -not $tr.passed -or
        $tr.checks_total -le 0 -or $tr.checks_total -ne $tr.checks_passed -or
        $tr.COM_open -ne 0 -or $tr.UDP_open -ne 0 -or $tr.board_actions -ne 0 -or
        $tr.model_loaded -or $tr.DLL_loaded -or $tr.live_or_flight_claim -or
        $tr.tested_sources.builder_sha256 -ne $refs.terrain_runtime_builder.sha256 -or
        $tr.tested_sources.checker_sha256 -ne $refs.terrain_runtime_checker.sha256 -or
        $tr.tested_sources.tests_sha256 -ne $refs.terrain_runtime_tests_source.sha256) {
        throw 'Diagnostic runtime tests are incomplete or not bound to the consumed source.'
    }
    # Semantic validation and all actual HOST evidence are evaluated in the
    # MATLAB OFFLINE_VALIDATE child, before any serial operation.
}
if($hasHoverTuning -and -not $hasDelivery) {
    foreach($label in @('tuning_contract_tests','tuning_transfer_tests','tuning_union_tests','tuning_api_tests',
        'tuning_builder','tuning_checker','tuning_transfer','tuning_union_checker',
        'tuning_transfer_tests_source','tuning_union_tests_source','tuning_api_tests_source')) {
        if(-not $refs.ContainsKey($label)){throw "Missing exact tuning HOST proof: $label"}
    }
    $tc=Get-Content -LiteralPath $refs.tuning_contract_tests.path -Raw|ConvertFrom-Json
    $tt=Get-Content -LiteralPath $refs.tuning_transfer_tests.path -Raw|ConvertFrom-Json
    $tu=Get-Content -LiteralPath $refs.tuning_union_tests.path -Raw|ConvertFrom-Json
    $ta=Get-Content -LiteralPath $refs.tuning_api_tests.path -Raw|ConvertFrom-Json
    if(-not $tc.passed -or $tc.case_count -le 0 -or $tc.case_count -ne $tc.cases_passed -or
        $tc.hardware_actions -ne 0 -or $tc.COM_UDP_actions -ne 0 -or $tc.parameter_writes -ne 0 -or
        -not $tt.passed -or $tt.checks_total -le 0 -or $tt.checks_total -ne $tt.checks_passed -or
        $tt.COM_open -ne 0 -or $tt.UDP_open -ne 0 -or $tt.board_actions -ne 0 -or
        $tt.source_sha256 -ne $refs.tuning_transfer.sha256 -or $tt.test_sha256 -ne $refs.tuning_transfer_tests_source.sha256 -or
        -not $tu.passed -or $tu.case_count -le 0 -or $tu.case_count -ne $tu.cases_passed -or
        $tu.hardware_actions -ne 0 -or $tu.COM_UDP_actions -ne 0 -or $tu.parameter_writes -ne 0 -or
        $tu.source_sha256 -ne $refs.tuning_union_checker.sha256 -or $tu.test_sha256 -ne $refs.tuning_union_tests_source.sha256 -or
        -not $ta.passed -or $ta.checks_total -le 0 -or $ta.checks_total -ne $ta.checks_passed -or
        $ta.hardware_actions -ne 0 -or $ta.COM_open -ne 0 -or $ta.PX4_access -ne 0 -or $ta.real_parameter_writes -ne 0 -or
        -not $ta.cleanup.ports_rebound -or @($ta.cleanup.errors).Count -ne 0 -or
        $ta.adapter_sha256 -ne $refs.adapter.sha256 -or $ta.fixture_sha256 -ne $refs.tuning_api_tests_source.sha256) {
        throw 'Three-parameter transfer/union/local API evidence is stale or failed.'
    }
}
if($hasHoverPreparation) {
    foreach($label in @('hover_preparation_checker','hover_preparation_tests','copter_barrier_outer_tests',
        'current_native_telemetry_checker','current_native_telemetry_tests')) {
        if(-not $refs.ContainsKey($label)){throw "Missing hover evidence checker: $label"}
    }
    $ot=Get-Content -LiteralPath $refs.copter_barrier_outer_tests.path -Raw|ConvertFrom-Json
    if(-not $ot.pass -or $ot.checks_total -ne $ot.checks_passed -or
       $ot.outer_source_sha256 -ne $refs.outer_source.sha256 -or $ot.COM_open -ne 0 -or
       $ot.UDP_open -ne 0 -or $ot.hardware_actions -ne 0) {throw 'Current hover lifecycle wrapper HOST tests not bound.'}
    $ht=Get-Content -LiteralPath $refs.hover_preparation_tests.path -Raw|ConvertFrom-Json
    if(-not $ht.passed -or $ht.case_count -lt 26 -or $ht.cases_passed -ne $ht.case_count -or
       $ht.COM_UDP_board_model_process_actions -ne 0 -or $ht.flight_admission -or
       $ht.sources.helper.sha256 -ne $refs.hover_preparation_checker.sha256) {throw 'Current hover preparation negative tests not bound.'}
    $nt=Get-Content -LiteralPath $refs.current_native_telemetry_tests.path -Raw|ConvertFrom-Json
    if(-not $nt.passed -or $nt.checks_total -ne $nt.checks_passed -or
       $nt.source_sha256 -ne $refs.current_native_telemetry_checker.sha256 -or
       $nt.COM_open -ne 0 -or $nt.UDP_open -ne 0 -or $nt.hardware_actions -ne 0) {
        throw 'Current ATT/LP/EST freshness and validity negatives not bound.'
    }
    foreach($e in $plan.native_hover_preparation.references) {
        $key='hover_preparation_'+$e.label
        if(-not $refs.ContainsKey($key) -or $refs[$key].path -ne $e.path -or
           $refs[$key].bytes -ne $e.bytes -or $refs[$key].sha256 -ne $e.sha256) {
            throw "Hover evidence must be included in the exact plan: $key"
        }
    }
    foreach($label in @('model_dll','source_model','sensor_frame_encoder','hil_profile','firmware')) {
        if($hasTerrainDerivation -and $label -in @('model_dll','source_model')) {
            $oldLabel=if($label -eq 'model_dll'){'old_dll'}else{'old_model'}
            if($terrainRefs[$oldLabel].sha256 -ne $plan.native_hover_preparation.runtime_binding.($label+'_sha256')) {
                throw 'Diagnostic derivation does not originate from the reference observation input.'
            }
            continue
        }
        if($refs[$label].sha256 -ne $plan.native_hover_preparation.runtime_binding.($label+'_sha256')) {
            throw "Hover input differs from the completed observation: $label"
        }
    }
    # Validate semantics offline before serial preflight.
}
foreach($p in $plan.bound_provenance) {
    if (-not $refs.ContainsKey([string]$p.reference_label) -or [string]::IsNullOrWhiteSpace($p.basis)) {
        throw "Unbound numerical provenance: $($p.field)"
    }
}
if($plan.board.uid -ne $device.uid -or $plan.board.board_version -ne 56 -or
    $plan.board.commit -ne '6ea3539157ca358c70a515878b77077af7d4611d' -or
    $plan.board.endpoint -ne 'COM3' -or $plan.board.baud -ne 921600 -or
    $plan.board.RA_CTRL_MODE -ne 0) {throw 'Native Pixhawk target/firmware/profile differs.'}
if($refs.firmware.sha256 -ne '7722616157AF96E3493D1827F01A0713D92373946C669B854B8F043552892FC7') {
    throw 'This native-controller run requires the accepted application identity without reflashing.'
}
if(-not $plan.safety.usb_only -or -not $plan.safety.propulsion_and_actuators_isolated -or
    $plan.safety.physical_output_actions -ne 0 -or $plan.safety.flash_reboot_bootloader_actions -ne 0) {
    throw 'Explicit zero physical authority/no-flash envelope is required.'
}
$compiled=Get-Content -LiteralPath $refs.dll_build.path -Raw | ConvertFrom-Json
$probe=Get-Content -LiteralPath $refs.dll_probe.path -Raw | ConvertFrom-Json
$equal=Get-Content -LiteralPath $refs.dll_source_comparison.path -Raw | ConvertFrom-Json
if($hasDelivery) {
    foreach($label in @('delivery_runner_tests','delivery_adapter_tests','delivery_host_replay','delivery_model_preparation',
        'delivery_environment_inflight_budget_test')) {
        if(-not $refs.ContainsKey($label)){throw "Missing delivery HOST proof: $label"}
    }
    $dr=Get-Content -LiteralPath $refs.delivery_runner_tests.path -Raw|ConvertFrom-Json
    $da=Get-Content -LiteralPath $refs.delivery_adapter_tests.path -Raw|ConvertFrom-Json
    $dh=Get-Content -LiteralPath $refs.delivery_host_replay.path -Raw|ConvertFrom-Json
    $dm=Get-Content -LiteralPath $refs.delivery_model_preparation.path -Raw|ConvertFrom-Json
    $db=Get-Content -LiteralPath $refs.delivery_environment_inflight_budget_test.path -Raw|ConvertFrom-Json
    if($compiled.status -ne 'PASS_SIMULINK_CPP_DLL_BUILD_ONLY' -or
       $compiled.dll_sha256 -ne $refs.model_dll.sha256 -or
       $compiled.source_model_sha256 -ne $refs.source_model.sha256 -or
       -not $probe.pass -or $probe.checks_total -ne $probe.checks_passed -or $probe.board_actions -ne 0 -or
       -not $dm.passed -or $dm.model_sha256 -ne $refs.source_model.sha256 -or $dm.parameters_sha256 -ne $refs.core_parameters.sha256 -or
       -not $db.passed -or $db.checks_total -ne 5 -or $db.checks_passed -ne 5 -or
           $db.host_heartbeat_freshness_s -ne 3.0 -or $db.host_extended_state_freshness_s -ne 2.0 -or
           $db.environment_inflight_budget_s -ne 0.25 -or $db.derived_board_evidence_bound_s -ne 3.25 -or
           $db.COM_open -ne 0 -or $db.board_actions -ne 0 -or
       -not $dr.passed -or $dr.checks_total -ne 18 -or $dr.checks_passed -ne 18 -or
           -not $dr.checks.environment_begin_after_disarmed_setup -or
           -not $dr.checks.environment_board_time_monotonic_under_mapping_jitter -or
           -not $dr.checks.blocking_command_waits_pause_mission_clock -or
           $dr.hardware_actions -ne 0 -or
       -not $da.pass -or $da.checks_total -ne 6 -or $da.checks_passed -ne 6 -or $da.board_actions -ne 0 -or
       -not $dh.passed -or $dh.checks_total -ne $dh.checks_passed -or $dh.hardware_actions -ne 0) {
        throw 'Current delivery DLL/model/runner/adapter/replay HOST evidence is incomplete or stale.'
    }
} elseif($compiled.status -ne 'PASS_SIMULINK_CPP_DLL_BUILD_ONLY' -or
    $probe.status -ne 'PASS_DLL_ABI_AND_DIAGNOSTIC_FAULT_LATCH_ONLY' -or
    $compiled.dll_sha256 -ne $refs.model_dll.sha256 -or $probe.dll_sha256 -ne $refs.model_dll.sha256 -or
    $equal.status -ne 'PASS_DLL_RAW_IO_VS_CANONICAL_MATLAB_CORE' -or $equal.cases -ne 6 -or
    $equal.passed_cases -ne $equal.cases -or $probe.cases -ne $equal.cases -or
    $equal.raw_rows -ne 18000 -or $equal.compared_rows -ne $equal.raw_rows -or $probe.rows -ne $equal.raw_rows -or
    $equal.csv_sha256 -ne $probe.trace_sha256 -or $equal.parameter_sha256 -ne $refs.core_parameters.sha256) {
    throw 'The exact compiled DLL, complete raw probe, and source-equivalence receipt are not mutually bound.'
}
if(-not $hasDelivery -and ((Get-FileHash -LiteralPath $compiled.source_model -Algorithm SHA256).Hash -ne $compiled.source_model_sha256 -or
    @($equal.case_results | Where-Object {-not $_.passed}).Count -ne 0 -or
    @($equal.case_results | Where-Object name -eq 'ELEVATED_GROUND_PROFILE').Count -ne 1 -or
    @($equal.case_results | Where-Object name -eq 'ELEVATED_FREE_AIR_PROFILE').Count -ne 1)) {
    throw 'Current model/independent-terrain equivalence coverage is missing or changed.'
}
if(-not $hasDelivery -and (Get-FileHash -LiteralPath $probe.trace -Algorithm SHA256).Hash -ne $probe.trace_sha256) {
    throw 'DLL/source probe raw trace changed.'
}
$api=Get-Content -LiteralPath $refs.actual_api_receipt.path -Raw | ConvertFrom-Json
if(-not $hasDelivery -and ($api.classification -ne 'HOST_API_LOOPBACK_FIXTURE' -or -not $api.pass -or
    $api.checks_total -lt 37 -or $api.checks_passed -ne $api.checks_total -or
    $api.adapter_sha256 -ne $refs.adapter.sha256 -or -not $api.cleanup.ports_rebound)) {
    throw 'Current adapter actual MATLAB socket/API fixture did not pass or changed.'
}
$clockTest=Get-Content -LiteralPath $refs.utc_pair_host_tests.path -Raw|ConvertFrom-Json
if(-not $clockTest.passed -or $clockTest.checks_total -ne 23 -or $clockTest.checks_passed -ne 23 -or
   $clockTest.timing_gates_changed -or $clockTest.COM_open -ne 0 -or $clockTest.UDP_open -ne 0){throw 'Measured UTC calibration negative/real clock tests missing.'}
if($plan.execution_kind -eq 'INITIALIZATION_OBSERVATION_ONLY') {
    foreach($requiredDiagnostic in @('copter_init_observer_tests','initialization_observer_tests')) {
        if(-not $refs.ContainsKey($requiredDiagnostic)){throw "Missing actual initialization-observer HOST test: $requiredDiagnostic"}
    }
    $initTest=Get-Content -LiteralPath $refs.copter_init_observer_tests.path -Raw|ConvertFrom-Json
    if(-not $initTest.pass -or $initTest.checks_total -ne 28 -or $initTest.checks_passed -ne 28 -or
        $initTest.source_sha256 -ne $refs.copter_init_observer_source.sha256 -or
        $initTest.COM_open -ne 0 -or $initTest.CopterSim_started -ne 0){throw 'Current passive observer actual HOST fixture failed or source changed.'}
    $observationTest=Get-Content -LiteralPath $refs.initialization_observer_tests.path -Raw|ConvertFrom-Json
    if(-not $observationTest.passed -or $observationTest.checks_total -ne 50 -or $observationTest.checks_passed -ne 50 -or
        $observationTest.cases -ne 16 -or $observationTest.COM_open -ne 0 -or $observationTest.UDP_open -ne 0 -or
        $observationTest.board_actions -ne 0 -or $observationTest.flight_claim){throw 'No-authority initialization observer HOST tests failed or incomplete.'}
}
$sensor=Get-Content -LiteralPath $refs.actual_sensor_receipt.path -Raw | ConvertFrom-Json
$sensorProofDllSha=$refs.model_dll.sha256
$sensorProofModelSha=$compiled.source_model_sha256
if($hasTerrainDerivation) {
    # Validate the diagnostic derivative and sensor-frame bindings separately.
    $sensorProofDllSha=$terrainRefs.old_dll.sha256
    $sensorProofModelSha=$terrainRefs.old_model.sha256
}
if($null -ne $plan.virtual_sensor_mount_contract -and -not $hasDelivery) {
    foreach($label in @('sensor_mount_checker','sensor_frame_encoder','sensor_mount_host_tests','sensor_frame_model_receipt')) {
        if(-not $refs.ContainsKey($label)){throw "Missing aligned sensor binding: $label"}
    }
    $mt=Get-Content -LiteralPath $refs.sensor_mount_host_tests.path -Raw|ConvertFrom-Json
    $mr=Get-Content -LiteralPath $refs.sensor_frame_model_receipt.path -Raw|ConvertFrom-Json
    if(-not $sensor.passed -or $sensor.status -ne 'PASS_OFFLINE_VIRTUAL_SENSOR_LEVEL_FRAME_PAIR' -or
       $sensor.checks_total -ne 16 -or $sensor.checks_passed -ne 16 -or $sensor.rows -ne 6100 -or
       $sensor.parent_sensor_physics_checks -ne 23 -or -not $sensor.parent_sensor_physics_passed -or
       $sensor.dll_sha256 -ne $sensorProofDllSha -or
       (Get-FileHash -LiteralPath $sensor.csv_path -Algorithm SHA256).Hash -ne $sensor.csv_sha256 -or
       (Get-FileHash -LiteralPath $sensor.parent_csv -Algorithm SHA256).Hash -ne $sensor.parent_csv_sha256 -or
       (Get-FileHash -LiteralPath $sensor.parent_sensor_receipt -Algorithm SHA256).Hash -ne $sensor.parent_sensor_receipt_sha256 -or
       -not $mr.plant_truth_kinematics_gravity_control_core_unchanged -or
       $mr.model_sha256 -ne $sensorProofModelSha -or $mr.helper_sha256 -ne $refs.sensor_frame_encoder.sha256) {
       throw 'Paired raw-sensor and plant-truth checks must bind to this generated DLL.'
    }
    if($plan.virtual_sensor_mount_contract.schema -eq 'VIRTUAL_SENSOR_MOUNT_CONTRACT_V2_AUTOCAL_OBSERVED') {
        if(-not $mt.passed -or $mt.legacy_case_count -ne 34 -or $mt.legacy_cases_passed -ne 34 -or
           $mt.v2_case_count -le 0 -or $mt.v2_cases_passed -ne $mt.v2_case_count -or
           $mt.case_count -ne (34+$mt.v2_case_count) -or $mt.cases_passed -ne $mt.case_count -or
           $mt.source_sha256 -ne $refs.sensor_mount_checker.sha256 -or
           -not $refs.ContainsKey('sensor_calibration_readonly')) {throw 'Current automatic calibration observation HOST tests or source evidence failed.'}
    } elseif(-not $mt.passed -or $mt.case_count -ne 34 -or $mt.cases_passed -ne 34) {throw 'Exact live mount/calibration checker HOST cases failed.'}
} elseif(-not $hasDelivery -and ($sensor.status -ne 'PASS_OFFLINE_OFFICIAL_SENSOR_FIELD_AND_SINGLE_GRAVITY_CONTRACT' -or
    -not $sensor.passed -or $sensor.checks_total -ne 23 -or $sensor.checks_passed -ne 23 -or
    $sensor.rows -ne 6100 -or $sensor.dll_sha256 -ne $refs.model_dll.sha256 -or
    @($sensor.cases | Where-Object {-not $_.passed -or $_.raw_bus_acceleration_contract -ne 'KINEMATIC_BODY_ACCELERATION'}).Count -ne 0 -or
    (Get-FileHash -LiteralPath $sensor.csv_path -Algorithm SHA256).Hash -ne $sensor.csv_sha256 -or
    (Get-FileHash -LiteralPath $sensor.generated_cpp_path -Algorithm SHA256).Hash -ne $sensor.generated_cpp_sha256)) {
    throw 'Actual official sensor fields/single-gravity/raw evidence do not match the exact current DLL.'
}
$forward=Get-Content -LiteralPath $refs.actual_forwarding_receipt.path -Raw | ConvertFrom-Json
if(-not $hasDelivery -and ($forward.status -ne 'PASS_COPTERSIM_NO_BOARD_FORWARDING_ONLY' -or
    $forward.dll_sha256 -ne $refs.model_dll.sha256 -or
    @($forward.checks.PSObject.Properties | Where-Object {-not $_.Value}).Count -ne 0 -or
    $forward.COM_open -ne 0 -or $forward.board_actions -ne 0 -or $forward.HIL)) {
    throw 'Current actual no-board forwarding/owned-process release did not pass.'
}
$hostTests=Get-Content -LiteralPath $refs.recovery_host_tests.path -Raw | ConvertFrom-Json
if(-not $hasDelivery -and ($hostTests.status -ne 'PASS_HOST_ONLY_NEW_OUTER_AND_RECOVERY' -or
    $hostTests.checks_total -ne $hostTests.checks_passed -or $hostTests.recovery.passed -ne $hostTests.recovery.cases -or
    $hostTests.COM_open -ne 0 -or $hostTests.UDP_open -ne 0 -or $hostTests.board_actions -ne 0)) {
    throw 'Current independent recovery and outer HOST-only regression result is missing.'
}
foreach($source in @($hostTests.sources)) {
    if($hasDelivery){break}
    if((Get-FileHash -LiteralPath $source.path -Algorithm SHA256).Hash -ne $source.sha256) {
        throw 'A directly tested outer/runner source changed after the HOST-only regression.'
    }
}
$jobTest=Get-Content -LiteralPath $refs.owned_job_host_tests.path -Raw | ConvertFrom-Json
if(-not $jobTest.pass -or $jobTest.checks_passed -ne $jobTest.checks_total -or
    $jobTest.source_sha256 -ne $refs.owned_job_source.sha256 -or
    @($jobTest.remaining_owned_process_ids).Count -ne 0 -or $jobTest.unrelated_existing_processes_terminated -ne 0) {
    throw 'Current suspended-start Windows Job ownership/release fixture did not pass.'
}
$expectedRunner=if($hasDelivery){Join-Path $PSScriptRoot 'run_m600_matlab_delivery_hil.m'}else{Join-Path $PSScriptRoot 'run_m600_matlab_hil.m'}
$expectedConfig=if($hasDelivery){Join-Path $PSScriptRoot 'make_m600_canonical_delivery_config.m'}else{Join-Path $PSScriptRoot 'make_m600_native_hover_config.m'}
if($refs.runner.path -ne $expectedRunner -or
    $refs.recovery.path -ne (Join-Path $PSScriptRoot 'recover_m600_canonical_udp.m') -or
    $refs.matlab_launcher.path -ne (Join-Path $PSScriptRoot 'launch_m600_canonical_hil.m') -or
    $refs.runner_config_source.path -ne $expectedConfig -or
    $refs.outer_source.path -ne $PSCommandPath) {
    throw 'Do not route this entry back to the obsolete smoke runner/finalizer.'
}
foreach($name in @('copter_start_timeout_s','matlab_child_timeout_s','recovery_child_timeout_s',
    'serial_child_timeout_s','matlab_startup_timeout_s','process_exit_timeout_s')) {
    $v=$plan.outer_bounds.$name
    if($null -eq $v -or [double]$v -le 0 -or [double]::IsNaN([double]$v) -or [double]::IsInfinity([double]$v)) {throw "Missing outer bound: $name"}
    $p=@($plan.bound_provenance | Where-Object field -eq ('outer.'+$name))
    if($p.Count -ne 1){throw "Missing/duplicate outer bound provenance: $name"}
}
$runtime=[IO.Path]::GetFullPath([string]$plan.runtime_path)
if($plan.scene_name -ne 'LowGPU'){throw 'This neutral physical ground experiment requires the measured LowGPU map.'}
$runtimeDll=Join-Path $runtime 'external\model\GPENMPC_M600_Canonical.dll'
if((Get-FileHash -LiteralPath $runtimeDll -Algorithm SHA256).Hash -ne $refs.model_dll.sha256 -or
    (Get-FileHash -LiteralPath (Join-Path $runtime 'CopterSim.exe') -Algorithm SHA256).Hash -ne $refs.copter_exe.sha256 -or
    (Get-FileHash -LiteralPath (Join-Path $runtime 'external\map\LowGPU.txt') -Algorithm SHA256).Hash -ne $refs.terrain_map.sha256 -or
    (Get-FileHash -LiteralPath (Join-Path $runtime 'external\map\LowGPU.png') -Algorithm SHA256).Hash -ne $refs.terrain_heightfield.sha256) {
    throw 'Prepared isolated CopterSim runtime does not match model/executable bindings.'
}
if($Live -and -not $plan.live_enabled){throw 'The supplied plan is HOST-ONLY; live is not enabled.'}
$useInitBarrier=$null -ne $plan.copter_initialization_barrier
if($useInitBarrier) {
    $b=$plan.copter_initialization_barrier
    $observerScope=$plan.execution_kind -eq 'INITIALIZATION_OBSERVATION_ONLY' -and
        $b.scope -eq 'DISARMED_OBSERVER_START' -and $b.subsequent_observation_duration_s -eq 45
    $hoverScope=($hasHoverPreparation -or $hasDelivery) -and
        $b.scope -eq 'VENDOR_PREFIX_BEFORE_NATIVE_HOVER_PREFLIGHT' -and
        $b.subsequent_observation_duration_s -eq 0 -and $b.fresh_hover_preflight_required
    if(-not $b.enabled -or -not ($observerScope -or $hoverScope) -or
       $b.deadline_s -ne 60 -or $b.maximum_vendor_reboots -ne 1 -or
       $b.initialization_prefix_scientific_credit -ne 0 -or
       -not $refs.ContainsKey('copter_barrier_source') -or -not $refs.ContainsKey('copter_barrier_tests')) {
        throw 'Vendor prefix must precede either the full diagnostic or a separately bound fresh hover preflight.'
    }
    $bt=Get-Content -LiteralPath $refs.copter_barrier_tests.path -Raw|ConvertFrom-Json
    if(-not $bt.pass -or $bt.checks_total -ne 49 -or $bt.checks_passed -ne 49 -or
       $bt.source_sha256 -ne $refs.copter_barrier_source.sha256 -or
       $bt.COM_open -ne 0 -or $bt.UDP_open -ne 0 -or $bt.hardware_actions -ne 0 -or $bt.CopterSim_start -ne 0) {
        throw 'Current vendor-prefix actual HOST tests did not pass.'
    }
}

New-Item -ItemType Directory -Path $OutputRoot | Out-Null
$journal=Join-Path $OutputRoot 'OUTER_JOURNAL.jsonl'
$records=[Collections.Generic.List[object]]::new()
$loadRows=[Collections.Generic.List[object]]::new()
$script:matlabTreeReleaseProved=$true
Add-Type -Path $refs.owned_job_source.path
Add-Type -Path $refs.host_load_source.path
$loadSampler=[GPENMPC.HostDiagnostics.HostLoadSampler]::new()
$ownedCopter=$null;$inner=$null;$udpRecovery=$null;$serialPost=$null;$failure=$null;$safeStop=$false
$startedCopter=$false;$copterStopped=$false;$offlinePassed=$false;$finalSafe=$false
$initObserver=$null;$initObservation=$null;$initEvidenceSaved=$false;$initPortReleased=$false;$initEvidenceComplete=$false
$initBarrier=$null;$barrierStartReceipt=$null;$barrierFinalReceipt=$null;$copterLaunchWatch=$null
$script:barrierLogPrefix='';$copterLogFreshBeforeLaunch=$false
$script:lastOwnedModuleProof=$null;$script:lastModuleObservationKey=''

function Record([string]$kind,$details) {
    $row=[ordered]@{utc=[DateTime]::UtcNow.ToString('o');kind=$kind;details=$details}
    $records.Add($row);[IO.File]::AppendAllText($journal,($row|ConvertTo-Json -Depth 12 -Compress)+[Environment]::NewLine)
}
function QuoteMat([string]$s) {return $s.Replace("'","''")}
function AssertOwnedProcess {
    if($null -eq $ownedCopter){throw 'No CopterSim PID was created by this outer.'}
    $p=Get-Process -Id $ownedCopter.Id -ErrorAction Stop
    if($p.StartTime.ToUniversalTime().Ticks -ne $ownedCopter.StartTime.ToUniversalTime().Ticks){throw 'Owned PID was reused.'}
    if([IO.Path]::GetFullPath($p.Path) -ne [IO.Path]::GetFullPath((Join-Path $runtime 'CopterSim.exe'))){throw 'Owned process executable changed.'}
    return $p
}
function AssertOwnedCopter([switch]$AllowUnloaded) {
    $p=AssertOwnedProcess
    $modelModules=@($p.Modules | Where-Object ModuleName -eq 'GPENMPC_M600_Canonical.dll')
    $proof=[ordered]@{pid=$p.Id;utc=[DateTime]::UtcNow.ToString('o');matching_module_count=$modelModules.Count;
        observed_path=$null;observed_sha256=$null;expected_path=[IO.Path]::GetFullPath($runtimeDll);
        expected_sha256=$refs.model_dll.sha256;present=$modelModules.Count -eq 1;matched=$false}
    if($modelModules.Count -eq 1) {
        $proof.observed_path=[IO.Path]::GetFullPath($modelModules[0].FileName)
        $proof.observed_sha256=(Get-FileHash -LiteralPath $proof.observed_path -Algorithm SHA256).Hash
        $proof.matched=$proof.observed_path -eq $proof.expected_path -and $proof.observed_sha256 -eq $proof.expected_sha256
    }
    $key=([string]$proof.matching_module_count)+'|'+$proof.observed_path+'|'+$proof.observed_sha256
    if($key -ne $script:lastModuleObservationKey){Record 'OWNED_MODEL_MODULE_OBSERVATION' $proof;$script:lastModuleObservationKey=$key}
    if($modelModules.Count -eq 0) {if($AllowUnloaded){return $null};throw 'OWNED_CANONICAL_MODULE_NOT_PRESENT_YET'}
    if(-not $proof.matched){throw ('OWNED_CANONICAL_MODULE_IDENTITY_MISMATCH: '+($proof|ConvertTo-Json -Compress))}
    $script:lastOwnedModuleProof=$proof
    return $p
}
function AssertControllerEndpointsReleased {
    if(-not $script:matlabTreeReleaseProved){throw 'Owned MATLAB Job ActiveProcesses=0 has not been proved.'}
    # Remote port 18570 belongs to CopterSim.
    $busy=@(Get-NetUDPEndpoint -ErrorAction Stop | Where-Object {$_.LocalPort -in @(14550,30101,20005)})
    Record 'CONTROLLER_UDP_RELEASE_PRE_RECOVERY' @{remaining=$busy;coptersim_not_stopped=$true}
    if($busy.Count -ne 0){throw 'Controller UDP handles remain owned; no competing recovery socket may open.'}
}
function UpdateInitializationPrefix {
    # Allow concurrent writing while reading the producer's stderr log.
    # File.ReadAllText does not provide the required Windows sharing mode.
    $stream=$null;$reader=$null
    try {
        $stream=[IO.FileStream]::new((Join-Path $OutputRoot 'COPTERSIM_STDERR.log'),
            [IO.FileMode]::Open,[IO.FileAccess]::Read,[IO.FileShare]::ReadWrite)
        $reader=[IO.StreamReader]::new($stream,[Text.Encoding]::UTF8,$true)
        $text=$reader.ReadToEnd()
    } finally {
        if($null -ne $reader){$reader.Dispose()}elseif($null -ne $stream){$stream.Dispose()}
    }
    if(-not $text.StartsWith($script:barrierLogPrefix,[StringComparison]::Ordinal)) {
        throw 'The vendor log was truncated or rewritten; establish readiness from the current log.'
    }
    $initBarrier.Append($text.Substring($script:barrierLogPrefix.Length),$copterLaunchWatch.Elapsed.TotalSeconds)
    $script:barrierLogPrefix=$text
    return $initBarrier.Snapshot($copterLaunchWatch.Elapsed.TotalSeconds)
}
function WaitForVendorInitializationPrefix {
    # Retain vendor initialization records separately from the observation interval.
    $lastLoadSample=-1.0
    while($true) {
        $null=AssertOwnedProcess
        $e=UpdateInitializationPrefix
        if($e.status -eq 'REJECTED_VENDOR_INITIALIZATION') {return $e}
        if($e.can_start_disarmed_observer) {
            # Verify the currently loaded module at handover.
            $p=AssertOwnedCopter -AllowUnloaded
            if($null -ne $p){return $e}
            if($copterLaunchWatch.Elapsed.TotalSeconds -ge 60){throw 'MODEL_NOT_LOADED_AT_READY_WITHIN_INITIALIZATION_BOUND'}
        }
        if($copterLaunchWatch.Elapsed.TotalSeconds-$lastLoadSample -ge 1.0) {
            try {$loadRows.Add($loadSampler.Sample([int[]]@($ownedCopter.Id)))}
            catch {$loadRows.Add([ordered]@{utc=[DateTime]::UtcNow.ToString('o');sampling_error=$_.Exception.Message})}
            $lastLoadSample=$copterLaunchWatch.Elapsed.TotalSeconds
        }
        Start-Sleep -Milliseconds 200
    }
}
function Child([string]$operation,[double]$timeout) {
    $copterPid=0;if($null -ne $ownedCopter -and -not $copterStopped){$copterPid=$ownedCopter.Id}
    $token=[ordered]@{operation=$operation;plan_sha256=$planHash;coptersim_owned_pid=$copterPid}
    if($operation -eq 'UDP_RECOVERY') {
        AssertControllerEndpointsReleased
        $token.independent_recovery_previous_owner_released=[bool]$script:matlabTreeReleaseProved
    }
    [IO.File]::WriteAllText((Join-Path $OutputRoot 'OUTER_STAGE_TOKEN.json'),($token|ConvertTo-Json))
    $stdout=Join-Path $OutputRoot ($operation+'_STDOUT.log');$stderr=Join-Path $OutputRoot ($operation+'_STDERR.log')
    $expr="try; addpath('$(QuoteMat $PSScriptRoot)'); r=launch_m600_canonical_hil('$(QuoteMat $PlanPath)','$operation','$(QuoteMat $OutputRoot)'); disp(struct('operation','$operation','result_saved',true)); catch e; disp(getReport(e,'extended','hyperlinks','off')); exit(1); end;"
    $ownedJob=$null
    try {
        $ownedJob=[OwnedMatlabJob]::Start($refs.matlab_exe.path,('-singleCompThread -batch "'+$expr+'"'),$buildRoot,$stdout,$stderr)
        $script:matlabTreeReleaseProved=$false
        Record 'MATLAB_CHILD_JOB_STARTED' @{operation=$operation;pid=$ownedJob.ProcessId;timeout_s=$timeout;
            assigned_before_resume=$ownedJob.AssignedBeforeResume;stdout=$stdout;stderr=$stderr;coptersim_outside_job=$true}
        $watch=[Diagnostics.Stopwatch]::StartNew();$exited=$false;$vendorAborted=$false;$vendorAbortReason=$null
        while($watch.Elapsed.TotalSeconds -lt $timeout) {
            if($operation -in @('INITIALIZATION_OBSERVE','RUN') -and $null -ne $barrierStartReceipt) {
                try {
                    $livePrefix=UpdateInitializationPrefix
                    if($livePrefix.status -eq 'REJECTED_VENDOR_INITIALIZATION' -or
                       $livePrefix.vendor_reboot_count -ne $barrierStartReceipt.vendor_reboot_count -or
                       $livePrefix.model_stop_count -ne $barrierStartReceipt.model_stop_count -or
                       $livePrefix.sim_start_count -ne $barrierStartReceipt.sim_start_count) {
                        $vendorAborted=$true;$vendorAbortReason='VENDOR_LIFECYCLE_CHANGED_AFTER_OBSERVER_HANDOVER'
                    }
                } catch {$vendorAborted=$true;$vendorAbortReason=$_.Exception.Message}
                if($vendorAborted) {
                    Record 'VENDOR_AFTER_HANDOVER_ABORT_PRESERVE_EXISTING_OBSERVER' @{reason=$vendorAbortReason;
                        operation=$operation;current_prefix=$livePrefix;never_restart_observer=$true;
                        never_restart_flight=$true;coptersim_retained_for_safety=$true}
                    break
                }
            }
            $ownedPids=[Collections.Generic.List[int]]::new()
            foreach($ownedPid in $ownedJob.ProcessIds){$ownedPids.Add($ownedPid)}
            if($null -ne $ownedCopter -and -not $copterStopped){$ownedPids.Add($ownedCopter.Id)}
            try {$loadRows.Add($loadSampler.Sample($ownedPids.ToArray()))}
            catch {$loadRows.Add([ordered]@{utc=[DateTime]::UtcNow.ToString('o');sampling_error=$_.Exception.Message})}
            $remaining=[Math]::Max(1,[Math]::Min(1000,[Math]::Ceiling(($timeout-$watch.Elapsed.TotalSeconds)*1000)))
            if($ownedJob.WaitForTreeExit([int]$remaining)){$exited=$true;break}
        }
        $expired=-not $exited -and -not $vendorAborted
        if(-not $exited) {
            # Only this suspended-start Job tree is terminated. CopterSim
            # remains alive and independent UDP safety recovery must follow.
            $endKind=if($vendorAborted){'MATLAB_CHILD_JOB_VENDOR_LIFECYCLE_ABORT'}else{'MATLAB_CHILD_JOB_WALL_TIMEOUT'}
            Record $endKind @{operation=$operation;pid=$ownedJob.ProcessId;
                active_processes=$ownedJob.ActiveProcesses;partial_logs_preserved=$true;vendor_abort_reason=$vendorAbortReason}
            $ownedJob.Terminate([uint32]124)
            $exited=$ownedJob.WaitForTreeExit([int][Math]::Ceiling($plan.outer_bounds.process_exit_timeout_s*1000))
        }
        $script:matlabTreeReleaseProved=$exited -and $ownedJob.ActiveProcesses -eq 0
        if(-not $script:matlabTreeReleaseProved){throw 'Owned MATLAB Job tree has not fully exited; recovery endpoint acquisition is forbidden.'}
        $result=[ordered]@{operation=$operation;pid=$ownedJob.ProcessId;exit_code=$ownedJob.ExitCode;
            timed_out=$expired;vendor_lifecycle_aborted=$vendorAborted;vendor_abort_reason=$vendorAbortReason;
            exited=$true;active_processes=0;complete_owned_tree_released=$true}
        Record 'MATLAB_CHILD_JOB_EXITED' $result
        return $result
    } finally {if($null -ne $ownedJob){$ownedJob.Dispose()}}
}

try {
    Record 'INPUTS_VERIFIED_NO_BOARD_CONTACT' @{plan_sha256=$planHash;references=$plan.references.Count;live=[bool]$Live}
    $probeExit=Child 'OFFLINE_VALIDATE' $plan.outer_bounds.matlab_startup_timeout_s
    $probeResult=Get-Content -LiteralPath (Join-Path $OutputRoot 'MATLAB_OFFLINE_PREFLIGHT.json') -Raw | ConvertFrom-Json
    if($probeExit.exit_code -ne 0 -or -not $probeResult.passed){throw 'MATLAB offline preflight failed before CopterSim/COM.'}
    $offlinePassed=$true
    if($Live) {
        $competitors=@(Get-Process -Name CopterSim,QGroundControl -ErrorAction SilentlyContinue)
        if($competitors.Count -ne 0){throw 'Existing simulator/COM-capable UI owner must not be interrupted or duplicated.'}
        # This is endpoint exclusivity only. Other MATLAB/Python/B tasks are
        # allowed and are neither stopped nor treated as generic blockers.
        $ports=@(14550,18570,30101,20005,20009)
        $udp=@(Get-NetUDPEndpoint -ErrorAction Stop | Where-Object {$_.LocalPort -in $ports})
        Record 'SHARED_ENDPOINT_PRECHECK' @{busy_endpoints=$udp;other_software_jobs_not_stopped=$true}
        if($udp.Count -ne 0){throw 'A required non-shareable endpoint is already owned.'}
        $preExit=Child 'SERIAL_PREFLIGHT' $plan.outer_bounds.serial_child_timeout_s
        $pre=Get-Content -LiteralPath (Join-Path $OutputRoot 'SERIAL_PREFLIGHT.json') -Raw | ConvertFrom-Json
        if($preExit.exit_code -ne 0 -or -not $pre.passed -or -not $pre.COM_closed){throw 'Fresh exact serial identity/safety/close not proved.'}
        Add-Type -Path $refs.copter_init_observer_source.path
        $initObserver=[GPENMPC.HostDiagnostics.OwnedCopterInitObserver]::Start(20009,1,8192)
        Record 'PASSIVE_VENDOR_INIT_OBSERVER_STARTED_BEFORE_COPTER' @{port=20009;receive_only=$true;capacity=8192;COM_open=0}
        $args=@('1','1','-1','GPENMPC_M600_Canonical','0','LowGPU','0','0','0','0','3:921600','2')
        $oldQtStderr=[Environment]::GetEnvironmentVariable('QT_FORCE_STDERR_LOGGING','Process')
        $oldQtPattern=[Environment]::GetEnvironmentVariable('QT_MESSAGE_PATTERN','Process')
        $copterLogFreshBeforeLaunch=-not (Test-Path -LiteralPath (Join-Path $OutputRoot 'COPTERSIM_STDERR.log'))
        if(-not $copterLogFreshBeforeLaunch){throw 'Vendor log must be absent in this fresh output before launch.'}
        $copterLaunchWatch=[Diagnostics.Stopwatch]::StartNew()
        try {
            [Environment]::SetEnvironmentVariable('QT_FORCE_STDERR_LOGGING','1','Process')
            [Environment]::SetEnvironmentVariable('QT_MESSAGE_PATTERN','[%{time process} %{type} pid=%{pid} tid=%{threadid}] %{message}','Process')
            $ownedCopter=Start-Process -FilePath (Join-Path $runtime 'CopterSim.exe') -ArgumentList $args `
                -WorkingDirectory $runtime -WindowStyle Hidden -PassThru `
                -RedirectStandardOutput (Join-Path $OutputRoot 'COPTERSIM_STDOUT.log') `
                -RedirectStandardError (Join-Path $OutputRoot 'COPTERSIM_STDERR.log')
        } finally {
            [Environment]::SetEnvironmentVariable('QT_FORCE_STDERR_LOGGING',$oldQtStderr,'Process')
            [Environment]::SetEnvironmentVariable('QT_MESSAGE_PATTERN',$oldQtPattern,'Process')
        }
        $startedCopter=$true;Record 'OWNED_COPTERSIM_STARTED' @{pid=$ownedCopter.Id;arguments=$args;runtime=$runtime}
        $watch=[Diagnostics.Stopwatch]::StartNew();$loaded=$false
        while($watch.Elapsed.TotalSeconds -lt $plan.outer_bounds.copter_start_timeout_s) {
            try {$null=AssertOwnedCopter;$loaded=$true;break} catch {
                if($_.Exception.Message -ne 'OWNED_CANONICAL_MODULE_NOT_PRESENT_YET'){throw}
                Start-Sleep -Milliseconds 100
            }
        }
        if(-not $loaded){throw 'Canonical module was not loaded within the declared startup bound.'}
        if($useInitBarrier) {
            Add-Type -Path $refs.copter_barrier_source.path
            # Reuse the module observation captured by the bounded load loop.
            $moduleProof=$script:lastOwnedModuleProof
            if($null -eq $moduleProof -or -not $moduleProof.matched){throw 'Initial module identity record is missing.'}
            $bo=[GPENMPC.HostDiagnostics.CopterInitializationBarrier+Options]::new()
            $bo.source_log_identity=$planHash+'::'+$ownedCopter.Id+'::'+(Join-Path $OutputRoot 'COPTERSIM_STDERR.log')
            $bo.expected_process_id=$ownedCopter.Id
            $bo.log_created_exclusively_for_this_launch=$copterLogFreshBeforeLaunch
            $bo.log_empty_before_launch=$copterLogFreshBeforeLaunch
            $bo.expected_dll_path=[IO.Path]::GetFullPath($runtimeDll)
            $bo.observed_dll_path=$moduleProof.observed_path
            $bo.expected_dll_sha256=$refs.model_dll.sha256
            $bo.observed_dll_sha256=$moduleProof.observed_sha256
            $initBarrier=[GPENMPC.HostDiagnostics.CopterInitializationBarrier]::new($bo)
            Record 'VENDOR_PREFIX_BEFORE_NEW_NO_ARM_OBSERVER' @{bound_s=60;maximum_vendor_initialization_reboots=1;
                execution_kind=$plan.execution_kind;fresh_runner_preflight_required=$true;
                prefix_scientific_credit=0;subsequent_complete_observation_s=$plan.copter_initialization_barrier.subsequent_observation_duration_s;
                flight_admission=$false}
            $barrierStartReceipt=WaitForVendorInitializationPrefix
            [IO.File]::WriteAllText((Join-Path $OutputRoot 'VENDOR_INITIALIZATION_PREFIX.json'),($barrierStartReceipt|ConvertTo-Json -Depth 12))
            Record 'VENDOR_PREFIX_OUTCOME_NOT_HEALTH_OR_FLIGHT_AUTHORITY' @{status=$barrierStartReceipt.status;
                vendor_reboots=$barrierStartReceipt.vendor_reboot_count;health_notes=$barrierStartReceipt.health_notes.Count;
                observation_start_allowed=$barrierStartReceipt.can_start_disarmed_observer;flight_admission=$false}
            if(-not $barrierStartReceipt.can_start_disarmed_observer){throw ('Vendor startup observation boundary: '+$barrierStartReceipt.first_reject)}
        }
        $operation=if($plan.execution_kind -eq 'INITIALIZATION_OBSERVATION_ONLY'){'INITIALIZATION_OBSERVE'}else{'RUN'}
        $runExit=Child $operation $plan.outer_bounds.matlab_child_timeout_s
        $innerPath=if($operation -eq 'RUN'){Join-Path $OutputRoot 'INNER_MATLAB\RESULT.json'}else{Join-Path $OutputRoot 'INITIALIZATION_MATLAB\RESULT.json'}
        if(Test-Path -LiteralPath $innerPath){$inner=Get-Content -LiteralPath $innerPath -Raw|ConvertFrom-Json}
        Record 'INNER_OUTCOME_NOT_SAFETY_AUTHORITY' @{process=$runExit;result=$inner}
    }
} catch {$failure=$_.Exception.Message;Record 'OUTER_EXCEPTION' @{message=$failure}}
finally {
    if($startedCopter) {
        try {
            # Keep the owned bridge available for recovery after model-load failure.
            $null=AssertOwnedProcess
            AssertControllerEndpointsReleased
            $recoveryExit=Child 'UDP_RECOVERY' $plan.outer_bounds.recovery_child_timeout_s
            $rp=Join-Path $OutputRoot 'UDP_RECOVERY.json'
            if(Test-Path -LiteralPath $rp){$udpRecovery=Get-Content -LiteralPath $rp -Raw|ConvertFrom-Json}
            $safeStop=$recoveryExit.exited -and -not $recoveryExit.timed_out -and
                $null -ne $udpRecovery -and $udpRecovery.safe_bridge_shutdown_allowed -and $udpRecovery.udp_closed
            if($hasHoverTuning) {
                $safeStop=$safeStop -and $udpRecovery.native_hover_original138_restored -and
                    $udpRecovery.tuning_original_identity_verified
            }
            if($safeStop) {
                $liveOwned=AssertOwnedProcess
                Stop-Process -Id $liveOwned.Id -Force
                if(-not $liveOwned.WaitForExit([int][Math]::Ceiling($plan.outer_bounds.process_exit_timeout_s*1000))){throw 'Safely stopped owned CopterSim has not exited.'}
                $copterStopped=$true;Record 'OWNED_COPTERSIM_STOPPED_AFTER_FRESH_SAFE_PROOF' @{pid=$liveOwned.Id}
                $postExit=Child 'SERIAL_POSTFLIGHT' $plan.outer_bounds.serial_child_timeout_s
                $serialPost=Get-Content -LiteralPath (Join-Path $OutputRoot 'SERIAL_POSTFLIGHT.json') -Raw|ConvertFrom-Json
                $finalSafe=$postExit.exit_code -eq 0 -and $serialPost.passed -and $serialPost.COM_closed
                if($hasHoverTuning){$finalSafe=$finalSafe -and $serialPost.native_hover_original138_verified}
            } else {Record 'KEEP_COPTERSIM_ALIVE_SAFETY_NOT_PROVED' @{pid=$ownedCopter.Id;receipt=$udpRecovery}}
        } catch {
            Record 'OUTER_RECOVERY_EXCEPTION_KEEP_OWNER_IF_NOT_ALREADY_SAFELY_STOPPED' @{message=$_.Exception.Message;pid=$ownedCopter.Id}
        }
    }
    if($null -ne $initObserver) {
        try {
            $initObserver.Dispose();$initObservation=$initObserver.Snapshot()
            [IO.File]::WriteAllText((Join-Path $OutputRoot 'COPTERSIM_INIT_OBSERVATIONS.json'),($initObservation|ConvertTo-Json -Depth 12))
            $initEvidenceSaved=$true
            $initRemaining=@(Get-NetUDPEndpoint -ErrorAction Stop | Where-Object LocalPort -eq 20009)
            $initPortReleased=$initObservation.socket_closed -and $initObservation.thread_exited -and $initRemaining.Count -eq 0
            $initEvidenceComplete=$initObservation.evidence_complete -and $initEvidenceSaved -and $initPortReleased
            Record 'PASSIVE_VENDOR_INIT_OBSERVER_CLOSED' @{socket_closed=$initObservation.socket_closed;thread_exited=$initObservation.thread_exited;
                raw_count=$initObservation.received_datagrams;complete=$initObservation.evidence_complete}
        } catch {Record 'PASSIVE_INIT_OBSERVER_FINALIZATION_ERROR' @{error=$_.Exception.Message}}
    }
    if($null -ne $initBarrier) {
        try {
            $barrierFinalReceipt=UpdateInitializationPrefix
            [IO.File]::WriteAllText((Join-Path $OutputRoot 'VENDOR_INITIALIZATION_FULL_LOG.json'),($barrierFinalReceipt|ConvertTo-Json -Depth 12))
        } catch {Record 'VENDOR_PREFIX_FINAL_EVIDENCE_ERROR' @{message=$_.Exception.Message}}
    }
    $result=[ordered]@{
        schema='M600_CANONICAL_OWNED_OUTER_RESULT_V1';plan_sha256=$planHash;offline_preflight_passed=$offlinePassed
        live_requested=[bool]$Live;execution_kind=$plan.execution_kind;coptersim_started=$startedCopter;coptersim_stopped=$copterStopped
        safe_to_stop_proved=$safeStop;final_serial_safety_passed=$finalSafe;failure=$failure
        inner_result=$inner;independent_udp_recovery=$udpRecovery;serial_postflight=$serialPost
        emergency_logical_force_disarm_count=if($null -ne $udpRecovery){$udpRecovery.counts.force_disarm_requests}else{0}
        normal_termination_accepted=($null -ne $udpRecovery -and $udpRecovery.normal_termination_pass -and
            $null -ne $inner -and $inner.fresh_final_safe_state)
        safety_recovery_remaining=$startedCopter -and -not $finalSafe
        user_physical_action_proven_required=$false
        owned_coptersim_pid=if($null -ne $ownedCopter){$ownedCopter.Id}else{0}
        other_software_tasks_stopped=0;RESOURCE_HOLD_created=$false;host_explicit_flash_reboot_bootloader_requests=0
        vendor_reboot_count=$null;vendor_reboot_count_status='NOT_INFERRED_FROM_ZERO_HOST_REQUESTS__INSPECT_VENDOR_AND_SOURCE_EPOCH_RECORDS'
        passive_vendor_initialization_observed=($null -ne $initObservation)
        passive_observer_evidence_saved=$initEvidenceSaved;passive_observer_port_released=$initPortReleased
        passive_observer_evidence_complete=$initEvidenceComplete
        vendor_initialization_barrier_used=$useInitBarrier
        vendor_initialization_prefix=$barrierStartReceipt;vendor_initialization_full_log=$barrierFinalReceipt
        physical_output_actions=0;inner_performance_and_outer_safety_are_separate=$true
        status=if(-not $Live -and $offlinePassed){'PREPARED_HOST_ONLY_NOT_EXECUTED'}elseif($finalSafe -and $initEvidenceComplete){'LIVE_OUTCOME_RECORDED_AND_BOARD_SAFE'}elseif($finalSafe){'BOARD_SAFE__PASSIVE_DIAGNOSTIC_EVIDENCE_INCOMPLETE'}else{'NOT_COMPLETED_SAFETY_OR_INFRASTRUCTURE_BOUNDARY'}
    }
    [IO.File]::WriteAllText((Join-Path $OutputRoot 'OUTER_RESULT.json'),($result|ConvertTo-Json -Depth 40))
    [IO.File]::WriteAllText((Join-Path $OutputRoot 'HOST_LOAD_1HZ.json'),(@($loadRows.ToArray())|ConvertTo-Json -Depth 12))
    [ordered]@{result_path=(Join-Path $OutputRoot 'OUTER_RESULT.json');
        failure=$failure;final_safety_passed=$finalSafe;coptersim_stopped=$copterStopped;
        full_evidence_preserved=$true}|ConvertTo-Json -Depth 4
}
if((-not $Live -and -not $offlinePassed) -or ($Live -and -not $finalSafe)){exit 2}
