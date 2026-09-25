function summary = runNativeFixedC3FocusedClosedLoop()
%RUNNATIVEFIXEDC3FOCUSEDCLOSEDLOOP Compare R0 and GP-robust tracking on one task-route unit.

testRoot = fileparts(mfilename("fullpath"));
projectRoot = fileparts(testRoot);
paths = [fullfile(projectRoot,"matlab","comparison"), ...
    fullfile(projectRoot,"matlab","dynamics"), ...
    fullfile(projectRoot,"matlab","inference"), ...
    fullfile(projectRoot,"matlab","simulink"), ...
    fullfile(projectRoot,"matlab","enmpc")];
for path = paths
    addpath(path);
end
cleanup = onCleanup(@() removePaths(paths));

multiCityRoot = ...
    gpenmpcExternalPath("comparison_fixed_reference_cases");
ordinaryB2Root = ...
    gpenmpcExternalPath("comparison_enmpc_cases");
units = gpenmpcDiscoverNativeComparisonUnits(multiCityRoot, ordinaryB2Root);
unit = units(1);
task = gpenmpcLoadNativeComparisonUnit(unit, ...
    fullfile(projectRoot,"matlab","data"));
assets = gpenmpcNativeHarnessAssets();

outputRoot = fullfile(projectRoot,"runtime", ...
    "native_fixed_c3_comparison", task.mission_id, task.planner_id);
if ~isfolder(outputRoot)
    mkdir(outputRoot);
end
methods = ["R0_ROBUST_SE3", "GR0_GP_ROBUST_SE3"];
traces = cell(2,1);
results = cell(2,1);
for methodIndex = 1:numel(methods)
    methodRoot = fullfile(outputRoot, methods(methodIndex));
    tracePath = fullfile(methodRoot,"MATLAB_NATIVE_FIXED_C3_TRACE.mat");
    resultPath = fullfile(methodRoot,"MATLAB_NATIVE_FIXED_C3_RESULT.json");
    if isfile(tracePath) && isfile(resultPath)
        loaded = load(tracePath,"trace");
        traces{methodIndex} = loaded.trace;
        results{methodIndex} = jsondecode(fileread(resultPath));
        generatedNow = false;
    else
        [traces{methodIndex}, results{methodIndex}] = ...
            gpenmpcRunFixedC3WholeTask(task, methods(methodIndex), assets);
        generatedNow = true;
    end
    assert(results{methodIndex}.reference_complete);
    assert(results{methodIndex}.task_complete_under_fixed_reference);
    assert(results{methodIndex}.leg_count == 5);
    assert(results{methodIndex}.service_phase_count == 4);
    if ~isfolder(methodRoot)
        mkdir(methodRoot);
    end
    if generatedNow
        trace = traces{methodIndex};
        save(tracePath, "trace", "-v7.3");
        writeJson(resultPath, results{methodIndex});
    end
end
assert(results{2}.method_runtime_diagnostics.valid_gp_observations > 0);
assert(results{2}.mean_trust_weight > 0.0);

prefixCount = 1001;
[r0FallbackTrace, r0FallbackResult] = gpenmpcRunFixedC3WholeTask( ...
    task, "R0_ROBUST_SE3", assets, maximum_samples=prefixCount);
[gpFallbackTrace, gpFallbackResult] = gpenmpcRunFixedC3WholeTask( ...
    task, "GR0_GP_ROBUST_SE3", assets, maximum_samples=prefixCount, ...
    force_hard_invalid=true);
fallback = struct;
fallback.sample_count = prefixCount;
fallback.maximum_position_difference_m = maxAbs( ...
    r0FallbackTrace.position_m, gpFallbackTrace.position_m);
fallback.maximum_velocity_difference_mps = maxAbs( ...
    r0FallbackTrace.velocity_mps, gpFallbackTrace.velocity_mps);
fallback.maximum_quaternion_difference = maxAbs( ...
    r0FallbackTrace.quaternion_wxyz, gpFallbackTrace.quaternion_wxyz);
fallback.maximum_rotor_thrust_difference_n = maxAbs( ...
    r0FallbackTrace.per_rotor_thrust_n, gpFallbackTrace.per_rotor_thrust_n);
fallback.maximum_rotor_command_difference_n = maxAbs( ...
    r0FallbackTrace.rotor_command_n, gpFallbackTrace.rotor_command_n);
fallback.maximum_augmentation_difference_mps2 = maxAbs( ...
    r0FallbackTrace.augmentation_acceleration_i_mps2, ...
    gpFallbackTrace.augmentation_acceleration_i_mps2);
fallback.controller_sensitive_energy_difference_j = abs( ...
    r0FallbackResult.mission_controller_sensitive_energy_j ...
    - gpFallbackResult.mission_controller_sensitive_energy_j);
fallback.tolerance = 1.0e-12;
fallback.pass = max([fallback.maximum_position_difference_m, ...
    fallback.maximum_velocity_difference_mps, ...
    fallback.maximum_quaternion_difference, ...
    fallback.maximum_rotor_thrust_difference_n, ...
    fallback.maximum_rotor_command_difference_n, ...
    fallback.maximum_augmentation_difference_mps2, ...
    fallback.controller_sensitive_energy_difference_j]) ...
    <= fallback.tolerance;
assert(fallback.pass);

b1 = gpenmpcFixedC3MethodContract( ...
    "B1_ENMPC_TOTAL_ROBUST_TUBE_NO_GP");
b2 = gpenmpcFixedC3MethodContract( ...
    "B2_ENMPC_GP_MEAN_TOTAL_TUBE");
assert(b1.external_reference_provider_required);
assert(b2.external_reference_provider_required);

summary = struct;
summary.schema = "GPENMPC_MATLAB_NATIVE_FIXED_C3_FOCUSED_RESULT_V1";
summary.status = "PASS_FOCUSED_NATIVE_FIXED_C3_IMPLEMENTATION";
summary.mission_id = task.mission_id;
summary.planner_id = task.planner_id;
summary.executed_method_cases = 2;
summary.r0 = results{1};
summary.gr0 = results{2};
summary.gr0_minus_r0 = pairedEffects(results{1}, results{2});
summary.forced_hard_invalid_exact_r0 = fallback;
summary.service_time_included = true;
summary.service_public_energy_included = true;
summary.controller_sensitive_correction_included = true;
summary.parent_control_outputs_injected = false;
summary.b1_b2_external_reference_provider_interface_reserved = true;
writeJson(fullfile(projectRoot,"reports", ...
    "NATIVE_FIXED_C3_FOCUSED_RESULT.json"), summary);

fprintf(char("Native fixed-C3 focused closure PASS: %s/%s, " ...
    + "R0 E=%.6f J, GR0 E=%.6f J, R0 p95=%.9f m, " ...
    + "GR0 p95=%.9f m, hard-invalid max diff=%.3g.\n"), ...
    task.mission_id, task.planner_id, ...
    results{1}.mission_controller_sensitive_energy_j, ...
    results{2}.mission_controller_sensitive_energy_j, ...
    results{1}.tracking_position_p95_m, ...
    results{2}.tracking_position_p95_m, ...
    fallback.maximum_position_difference_m);
end


function effects = pairedEffects(r0, gr0)
effects = struct;
names = ["mission_controller_sensitive_energy_j", "mission_time_s", ...
    "tracking_position_p95_m", "tracking_attitude_p95_rad", ...
    "maximum_acceleration_mps2", "maximum_jerk_mps3", ...
    "rotor_command_total_variation_n", ...
    "minimum_rotor_thrust_margin_n", ...
    "robust_compensation_p95_mps2"];
for name = names
    effects.(name + "__gr0_minus_r0") = gr0.(name) - r0.(name);
end
end


function value = maxAbs(left, right)
value = max(abs(double(left) - double(right)), [], "all");
end


function writeJson(path, payload)
text = jsonencode(payload, PrettyPrint=true);
fid = fopen(path, "w", "n", "UTF-8");
if fid < 0
    error("runNativeFixedC3FocusedClosedLoop:Write", ...
        "Cannot open %s for writing.", path);
end
cleanup = onCleanup(@() fclose(fid));
fwrite(fid, text, "char");
end


function removePaths(paths)
for path = paths
    rmpath(path);
end
end
