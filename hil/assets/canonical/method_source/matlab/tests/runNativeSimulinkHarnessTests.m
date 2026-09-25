function result = runNativeSimulinkHarnessTests(projectRoot, outputRoot)
%RUNNATIVESIMULINKHARNESSTESTS Build, update and simulate the native loop.

arguments
    projectRoot (1,1) string
    outputRoot (1,1) string
end
addpath(fullfile(projectRoot, "matlab", "simulink"));
addpath(fullfile(projectRoot, "matlab", "enmpc"));
addpath(fullfile(projectRoot, "matlab", "inference"));
addpath(fullfile(projectRoot, "matlab", "dynamics"));
if ~isfolder(outputRoot)
    mkdir(outputRoot);
end
modelPath = buildGPENMPCNativeSixDofClosedLoop(outputRoot);
[~, modelName] = fileparts(modelPath);
load_system(modelPath);
set_param(modelName, "SimulationCommand", "update");
blocks = find_system(modelName, "Type", "Block");
sfunctions = find_system(modelName, "BlockType", "M-S-Function");
outports = find_system(modelName, "BlockType", "Outport");
simOut = sim(modelName, "StopTime", "1.2", "SimulationMode", "normal", ...
    "ReturnWorkspaceOutputs", "on");
logs = simOut.yout;
required = ["fixed_reference", "environment", "selected_reference", "state", ...
    "gp_diagnostics", "enmpc_decision", "solver_status", ...
    "robust_se3_diagnostics", "desired_force", "desired_wrench", ...
    "rotor_commands", "modeled_power", "plant_acceleration"];
checks = struct;
for name = required
    element = logs.getElement(name);
    values = element.Values.Data;
    checks.(name) = ~isempty(values) && all(isfinite(double(values(:))));
end
state = logs.getElement("state").Values.Data;
fixedReference = logs.getElement("fixed_reference").Values.Data;
rotor = logs.getElement("rotor_commands").Values.Data;
power = logs.getElement("modeled_power").Values.Data;
solver = logs.getElement("solver_status").Values.Data;
gp = logs.getElement("gp_diagnostics").Values.Data;
checks.state_dimension = size(state, 2) == 19;
checks.rotor_dimension = size(rotor, 2) == 6;
checks.positive_power = all(power(:) > 0.0);
checks.solver_updated = any(solver(:, 5) > 0.0);
checks.native_m_sfunctions = numel(sfunctions) == 3;
checks.all_public_outports = numel(outports) == 13;
checks.state_evolves = norm(double(state(end, :) - state(1, :))) > 1.0e-6;
checks.reference_evolves = norm(double(fixedReference(end, :) ...
    - fixedReference(1, :))) > 1.0e-6;
checks.rotor_commands_nonnegative = all(rotor(:) >= 0.0);
quaternionNorm = sqrt(sum(double(state(:, 7:10)).^2, 2));
checks.unit_quaternion = max(abs(quaternionNorm - 1.0)) <= 1.0e-10;
checks.gp_uncertainty_computed = any(gp(:, 4:6) > 0.0, "all");
checks.no_hardware_blocks = isempty(find_system(modelName, ...
    "RegExp", "on", "Name", ".*(PX4|COM|Serial|HIL|UDP|MAVLink).*"));
close_system(modelName, 0);

result = struct;
result.schema = "GPENMPC_NATIVE_SIMULINK_6DOF_HARNESS_TEST_V1";
result.status = "PASS_EXECUTABLE_NATIVE_SIMULINK_6DOF_HARNESS";
result.block_count = numel(blocks);
result.sfunction_count = numel(sfunctions);
result.outport_count = numel(outports);
result.logged_signal_count = numel(required);
result.checks = checks;
result.all_checks_pass = all(structfun(@logical, checks));
result.stop_time_s = 1.2;
result.normal_mode = true;
result.model_path = string(modelPath);
result.metrics = struct( ...
    "state_change_norm", norm(double(state(end, :) - state(1, :))), ...
    "quaternion_norm_max_error", max(abs(quaternionNorm - 1.0)), ...
    "rotor_command_min_n", min(double(rotor(:))), ...
    "rotor_command_max_n", max(double(rotor(:))), ...
    "modeled_power_min_w", min(double(power(:))), ...
    "modeled_power_mean_w", mean(double(power(:))), ...
    "modeled_power_max_w", max(double(power(:))), ...
    "solver_iteration_max", max(double(solver(:, 5))), ...
    "gp_trust_max", max(double(gp(:, 7))), ...
    "gp_applied_mean_norm_max_mps2", max(double(gp(:, 10))));
if ~result.all_checks_pass
    error("runNativeSimulinkHarnessTests:Check", ...
        "Native Simulink harness did not pass all structural/runtime checks.");
end
jsonPath = fullfile(outputRoot, "NATIVE_SIMULINK_6DOF_HARNESS_TEST.json");
file = fopen(jsonPath, "w", "n", "UTF-8");
cleanup = onCleanup(@() fclose(file));
fwrite(file, jsonencode(result, PrettyPrint=true), "char");
harnessData = struct;
harnessData.schema = "GPENMPC_NATIVE_SIMULINK_6DOF_SHORT_RUN_SIGNALS_V1";
harnessData.sample_time_s = 0.01;
harnessData.stop_time_s = result.stop_time_s;
for name = required
    element = logs.getElement(name);
    harnessData.(name) = struct("time_s", element.Values.Time, ...
        "data", element.Values.Data);
end
save(fullfile(outputRoot, "NATIVE_SIMULINK_6DOF_SHORT_RUN_SIGNALS.mat"), ...
    "harnessData", "-v7");
fprintf("%s blocks=%d sfunctions=%d outports=%d signals=%d\n", ...
    result.status, result.block_count, result.sfunction_count, ...
    result.outport_count, result.logged_signal_count);
end
