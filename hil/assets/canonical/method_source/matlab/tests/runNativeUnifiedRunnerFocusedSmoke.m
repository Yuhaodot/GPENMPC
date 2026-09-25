function result = runNativeUnifiedRunnerFocusedSmoke(projectRoot, outputRoot)
%RUNNATIVEUNIFIEDRUNNERFOCUSEDSMOKE Test B1/B2 interfaces on an 80-sample task prefix.

arguments
    projectRoot (1,1) string
    outputRoot (1,1) string = fullfile(projectRoot, "reports")
end
addpath(fullfile(projectRoot, "matlab", "comparison"), ...
    fullfile(projectRoot, "matlab", "data"), ...
    fullfile(projectRoot, "matlab", "dynamics"), ...
    fullfile(projectRoot, "matlab", "enmpc"), ...
    fullfile(projectRoot, "matlab", "inference"));
multi = gpenmpcExternalPath("comparison_fixed_reference_cases");
parent = gpenmpcExternalPath("comparison_enmpc_cases");
units = gpenmpcDiscoverNativeComparisonUnits(multi, parent);
task = gpenmpcLoadNativeComparisonUnit(units(1), ...
    fullfile(projectRoot, "matlab", "data"));
assets = gpenmpcLoadNativeEnmpcComparisonAssets(projectRoot, 0.30);
methods = [assets.enmpc.b1_fallback_method, assets.enmpc.method];
rows = cell(2, 1);
for index = 1:2
    [trace, row] = gpenmpcRunNativeEnmpcWholeTask(task, methods(index), ...
        assets, MaximumInnerSamples=80, FocusedSmoke=true);
    assert(row.focused_smoke && ~row.full_task_mode);
    assert(row.sample_count == 80);
    assert(allFinite(trace));
    rows{index} = row;
end
result = struct( ...
    "schema", "GPENMPC_MATLAB_NATIVE_UNIFIED_RUNNER_FOCUSED_SMOKE_V1", ...
    "status", "PASS", ...
    "mission_id", task.mission_id, ...
    "planner_id", task.planner_id, ...
    "outer_period_s", 0.30, ...
    "methods", methods, ...
    "sample_count_per_method", 80, ...
    "b1_solver_updates", rows{1}.solver.update_count, ...
    "b2_solver_updates", rows{2}.solver.update_count, ...
    "b2_gp_hard_invalid_count", rows{2}.solver.gp_hard_invalid_count);
if ~isfolder(outputRoot), mkdir(outputRoot); end
reportPath = fullfile(outputRoot, ...
    "NATIVE_UNIFIED_RUNNER_FOCUSED_SMOKE_RESULT.json");
writelines(jsonencode(result, PrettyPrint=true), reportPath, Encoding="UTF-8");
fprintf("MATLAB_NATIVE_UNIFIED_RUNNER_FOCUSED_SMOKE_PASS samples=%d methods=2\n", 80);
end

function pass = allFinite(trace)
pass = true;
names = fieldnames(trace);
for index = 1:numel(names)
    value = trace.(names{index});
    if isnumeric(value) && any(~isfinite(double(value)), "all")
        pass = false;
        return
    end
end
end
