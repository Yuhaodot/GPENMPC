function result = runPreparedTrajectoryEquivalence(projectRoot, stagingRoot, outputPath)
%RUNPREPAREDTRAJECTORYEQUIVALENCE Verify exact derivative preparation.

arguments
    projectRoot (1,1) string
    stagingRoot (1,1) string
    outputPath (1,1) string = ""
end
addpath(genpath(fullfile(projectRoot, "matlab")));
addpath(genpath(fullfile(stagingRoot, "matlab")), "-begin");
fixture = jsondecode(fileread(fullfile(projectRoot, "tests", "fixtures", ...
    "ordinary_b2_rollout_python_reference.json")));
trajectory = struct( ...
    "total_duration_s", double(fixture.trajectory.total_duration_s), ...
    "coefficients_ascending", ...
        double(fixture.trajectory.coefficients_ascending));
prepared = gpenmpcPrepareTrajectoryDerivatives(trajectory, 3);

rng(20260902, "twister");
progresses = [0.0; trajectory.total_duration_s; ...
    trajectory.total_duration_s .* rand(1024, 1)];
maximumDifference = 0.0;
for progress = progresses.'
    for order = 0:3
        legacy = legacyDerivative(trajectory, progress, order);
        current = gpenmpcEvaluateTrajectoryDerivative(prepared, progress, order);
        maximumDifference = max(maximumDifference, ...
            max(abs(legacy - current), [], "all"));
    end
end
assert(maximumDifference <= 1.0e-12);

repeatCount = 500;
queryProgress = trajectory.total_duration_s .* 0.613;
legacyStarted = tic;
for repeat = 1:repeatCount
    for order = 0:3
        gpenmpcEvaluateTrajectoryDerivative(trajectory, queryProgress, order);
    end
end
legacyElapsedS = toc(legacyStarted);
preparedStarted = tic;
for repeat = 1:repeatCount
    gpenmpcEvaluatePreparedTrajectoryJet(prepared, queryProgress);
end
preparedElapsedS = toc(preparedStarted);

result = struct;
result.schema = "GPENMPC_PREPARED_TRAJECTORY_EQUIVALENCE_V1";
result.status = "PASS";
result.progress_count = numel(progresses);
result.derivative_orders = 0:3;
result.maximum_absolute_difference = maximumDifference;
result.timing_repeats = repeatCount;
result.legacy_elapsed_s = legacyElapsedS;
result.prepared_elapsed_s = preparedElapsedS;
result.elapsed_ratio_prepared_over_legacy = ...
    preparedElapsedS ./ legacyElapsedS;
result.candidate_order_changed = false;
result.objective_or_constraint_changed = false;
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


function value = legacyDerivative(trajectory, progressS, derivativeOrder)
progress = min(max(double(progressS), 0.0), ...
    double(trajectory.total_duration_s));
coefficients = double(trajectory.coefficients_ascending);
for order = 1:derivativeOrder
    powers = 1:(size(coefficients, 2) - 1);
    if isempty(powers)
        coefficients = zeros(3, 1);
        break
    end
    coefficients = coefficients(:, 2:end) .* powers;
end
powers = progress .^ (0:(size(coefficients, 2) - 1));
value = coefficients * powers.';
value = double(value(:));
end
