function result = runB2SustainedPrefix(projectRoot, outputPath, options)
%RUNB2SUSTAINEDPREFIX Run a sample-limited B2 diagnostic with zero disturbance.

arguments
    projectRoot (1,1) string
    outputPath (1,1) string = ""
    options.MaximumInnerSamples (1,1) double {mustBeInteger,mustBePositive} = 5000
end
addpath(genpath(fullfile(projectRoot, "matlab")));
multiCityRoot = ...
    gpenmpcExternalPath("comparison_fixed_reference_cases");
ordinaryB2Root = ...
    gpenmpcExternalPath("comparison_enmpc_cases");
units = gpenmpcDiscoverNativeComparisonUnits(multiCityRoot, ordinaryB2Root);
task = gpenmpcLoadNativeComparisonUnit(units(1), ...
    fullfile(projectRoot, "matlab", "data"));
task = gpenmpcMakeNoDisturbanceDiagnosticTask(task);
assets = gpenmpcLoadNativeEnmpcComparisonAssets(projectRoot, 0.30);
[trace, row] = gpenmpcRunNativeEnmpcWholeTask(task, assets.enmpc.method, ...
    assets, MaximumInnerSamples=options.MaximumInnerSamples, ...
    FocusedSmoke=true, AttitudeContinuityEnabled=true, ...
    AttitudeContinuityTimeConstantS=0.08, ...
    AttitudeContinuityMaximumAngularVelocityRadS=1.5, ...
    AttitudeContinuityMaximumAngularAccelerationRadS2=8.0, ...
    AttitudeContinuityMaximumContinuousDtS=0.05);
assert(numel(trace.global_time_s) == options.MaximumInnerSamples);
assert(row.stopped_by_maximum_inner_samples);
result = struct;
result.schema = "GPENMPC_B2_SUSTAINED_PREFIX_RESULT_V1";
result.status = "PASS_SAMPLE_LIMITED_DIAGNOSTIC";
result.maximum_inner_samples = options.MaximumInnerSamples;
result.samples_observed = numel(trace.global_time_s);
result.solver_update_count = row.solver.update_count;
result.solver_deadline_miss_count = row.solver.deadline_miss_count;
result.solver_elapsed_p95_s = row.solver.elapsed_p95_s;
result.solver_elapsed_max_s = row.solver.elapsed_max_s;
result.gp_adopted_count = row.solver.gp_adopted_count;
result.gp_downweighted_count = row.solver.gp_downweighted_count;
result.gp_hard_invalid_count = row.solver.gp_hard_invalid_count;
result.task_complete = row.task_complete;
if strlength(outputPath) > 0
    parent = fileparts(outputPath);
    if ~isfolder(parent)
        mkdir(parent);
    end
    writelines(jsonencode(result, PrettyPrint=true), outputPath, ...
        Encoding="UTF-8");
end
fprintf("%s\n", jsonencode(result));
end
