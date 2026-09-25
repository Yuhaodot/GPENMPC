function plan=prepare_m600_delivery_live_plan(outputPath)
% PREPARE_M600_DELIVERY_LIVE_PLAN Prepare a content-addressed delivery plan.
arguments
    outputPath (1,1) string
end
assert(~isfile(outputPath)&&~isfolder(outputPath),'m600delivery:PlanExists', ...
    'The requested delivery plan output already exists.');
root=fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(root,'tools'),fullfile(root,'matlab_validation'), ...
    fullfile(root,'m600_coptersim','matlab_validation'));
parentPath=fullfile(gpenmpc_external_path('native_tuned_hover_preparation'),'PLAN.json');
plan=jsondecode(fileread(parentPath));
runtime=gpenmpc_external_path('delivery_coptersim_runtime');
plan.execution_kind='CANONICAL_DELIVERY';
plan.runtime_path=runtime;
plan.plant_identity='M600_19STATE_RK4_10MS__CANONICAL_CAMBRIDGE_FOUR_NATIVE_GROUND_DELIVERIES__STATEFUL_MASS_ACK';
plan.outer_bounds.matlab_child_timeout_s=1300;
plan.native_hover_tuning=make_m600_delivery_native_tuning_contract();
cfg=make_m600_canonical_delivery_config(plan.plant_identity,'FRESH_SERIAL_PREFLIGHT_REQUIRED');
cfg.virtual_sensor_mount_contract=plan.virtual_sensor_mount_contract;
cfg.native_hover_preparation=plan.native_hover_preparation;
cfg.temporary_allocator_geometry=plan.temporary_allocator_geometry;
cfg.native_hover_tuning=plan.native_hover_tuning;
plan.runner_config=cfg;
% Bind the delivery diagnostic to its ground and environment extension.
if isfield(plan,'terrain_diagnostic_runtime_contract'),plan=rmfield(plan,'terrain_diagnostic_runtime_contract');end
if isfield(plan,'require_terrain_diagnostic_extension'),plan=rmfield(plan,'require_terrain_diagnostic_extension');end

replace('model_dll',fullfile(runtime,'external','model','GPENMPC_M600_Canonical.dll'));
replace('dll_build',fullfile(gpenmpc_external_path('canonical_delivery_dll_build'),'DLL_BUILD_RESULT.json'));
replace('dll_probe',fullfile(gpenmpc_external_path('host_canonical_delivery_dll'),'RESULT.json'));
replace('dll_source_comparison',fullfile(gpenmpc_external_path('environment_delivery_model'),'MODEL_PREPARATION_RESULT.json'));
replace('core_parameters',fullfile(gpenmpc_external_path('environment_delivery_model'),'M600_CORE_PARAMETERS.mat'));
replace('source_model',fullfile(gpenmpc_external_path('environment_delivery_model'),'GPENMPC_M600_Canonical.slx'));
replace('actual_api_receipt',fullfile(gpenmpc_external_path('host_m600_delivery_adapter_loopback'),'RESULT.json'));
replace('runner',fullfile(root,'tools','run_m600_matlab_delivery_hil.m'));
replace('runner_config_source',fullfile(root,'tools','make_m600_canonical_delivery_config.m'));
replace('matlab_launcher',fullfile(root,'tools','launch_m600_canonical_hil.m'));
replace('outer_source',fullfile(root,'tools','run_m600_canonical_hil.ps1'));
replace('recovery',fullfile(root,'tools','recover_m600_canonical_udp.m'));
replace('serial_preflight',fullfile(root,'tools','m600_canonical_serial_preflight.m'));
replace('adapter',fullfile(root,'m600_coptersim','matlab_validation','+m600check','makeM600CopterSimIo.m'));
replace('diagnostic_observer',fullfile(root,'matlab_validation','+m600check','updateCopterSimDeliveryDiagnostic.m'));
replace('terrain_map',fullfile(runtime,'external','map','LowGPU.txt'));
replace('terrain_heightfield',fullfile(runtime,'external','map','LowGPU.png'));
replace('copter_exe',fullfile(runtime,'CopterSim.exe'));
replace('plan_builder',[mfilename('fullpath') '.m']);
append('delivery_runner_tests',fullfile(gpenmpc_external_path('host_m600_delivery_runner'),'RESULT.json'));
append('delivery_adapter_tests',fullfile(gpenmpc_external_path('host_m600_delivery_adapter_loopback'),'RESULT.json'));
append('delivery_host_replay',fullfile(gpenmpc_external_path('host_canonical_cambridge_delivery_replay'),'RESULT.json'));
append('delivery_model_preparation',fullfile(gpenmpc_external_path('environment_delivery_model'),'MODEL_PREPARATION_RESULT.json'));
append('delivery_environment_inflight_budget_test',fullfile(gpenmpc_external_path('host_delivery_environment_inflight_budget'),'RESULT.json'));
append('delivery_environment_inflight_budget_test_source',fullfile(root,'tools', ...
    'test_delivery_environment_inflight_budget.m'));
append('delivery_task',cfg.delivery_mission.task_path);
append('delivery_lifecycle',cfg.delivery_mission.delivery_lifecycle_config);
append('delivery_relaunch',cfg.delivery_mission.relaunch_config);
append('delivery_runner_test_source',fullfile(root,'tools','run_m600_delivery_runner_host_tests.m'));
append('delivery_adapter_test_source',fullfile(root,'tools','run_m600_delivery_adapter_loopback.m'));
append('delivery_config_source',fullfile(root,'tools','make_m600_canonical_delivery_config.m'));
append('delivery_tuning_builder',fullfile(root,'tools','make_m600_delivery_native_tuning_contract.m'));
append('delivery_tuning_validator',fullfile(root,'tools','validate_m600_delivery_native_tuning_contract.m'));
append('delivery_union_checker',fullfile(root,'tools','verify_m600_delivery_parameter_union.m'));
append('delivery_environment_frame_builder',fullfile(root,'matlab_validation', ...
    '+gpenmpcTaskIo','makePlantEnvironmentFrameFromSnapshot.m'));
append('delivery_host_safety_wrapper',fullfile(root,'tools','canonical_delivery_host_step_checked.m'));
plan.delivery_contract=struct('task_sha256',cfg.delivery_mission.task_sha256, ...
    'services',4,'ground_dwell_min_s',8,'native_land_count',5, ...
    'standard_disarm_count',5,'arm_transition_count',5, ...
    'payload_kg',cfg.delivery_environment_contract.payload_by_generation_kg, ...
    'physical_output_actions',0,'python_hil_driver',false);
plan.delivery_prelaunch_evidence=struct( ...
    'runner_tests','18/18 PASS INCLUDING 160MS BLOCKING COMMAND CLOCK-HOLD ACCOUNTING', ...
    'environment_cross_process_budget','5/5 PASS; host freshness remains 3.0/2.0 s and only the 0.25 s in-flight allowance is derived', ...
    'adapter_loopback','6/6 PASS', ...
    'host_task_replay','14/14 PASS','claim','Host replay.');
bp=find(strcmp({plan.bound_provenance.field},'outer.matlab_child_timeout_s'));
assert(isscalar(bp),'m600delivery:MatlabChildBoundProvenance', ...
    'The MATLAB child bound provenance entry is missing or ambiguous.');
plan.bound_provenance(bp).basis= ...
    '1300 s wall-liveness bound covers the 980 s mission plus MATLAB startup and finalization.';
plan.bound_provenance(bp).reference_label='runner';

% Resolve inherited paths against current delivery-specific sources.
for ri=1:numel(plan.references)
    info=dir(plan.references(ri).path);
    assert(isfile(plan.references(ri).path)&&isscalar(info), ...
        'm600delivery:InheritedReferenceMissing','%s',plan.references(ri).path);
    plan.references(ri).bytes=info.bytes;
    plan.references(ri).sha256=m600check.fileSha256(plan.references(ri).path);
end

folder=fileparts(outputPath);if ~isfolder(folder),mkdir(folder);end
fid=fopen(outputPath,'w','n','UTF-8');assert(fid>=0);guard=onCleanup(@()fclose(fid));
fprintf(fid,'%s\n',jsonencode(plan,PrettyPrint=true));clear guard

    function replace(label,path)
        idx=find(strcmp({plan.references.label},label));assert(isscalar(idx), ...
            'm600delivery:PlanLabel','The requested plan reference label is missing or ambiguous.');
        plan.references(idx)=entry(label,path);
    end
    function append(label,path)
        assert(~any(strcmp({plan.references.label},label)),'m600delivery:DuplicateLabel', ...
            'The requested plan reference label already exists.');
        plan.references(end+1)=entry(label,path);
    end
    function e=entry(label,path)
        info=dir(path);assert(isfile(path)&&isscalar(info),'m600delivery:MissingReference','%s',path);
        e=struct('label',label,'path',char(path),'bytes',info.bytes,'sha256',m600check.fileSha256(path));
    end
end
