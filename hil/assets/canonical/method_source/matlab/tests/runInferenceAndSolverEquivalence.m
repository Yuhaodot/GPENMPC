function result = runInferenceAndSolverEquivalence(projectRoot, outputPath)
%RUNINFERENCEANDSOLVEREQUIVALENCE Compare cached GP inference and outer-solver decisions.

arguments
    projectRoot (1,1) string
    outputPath (1,1) string = ""
end
addpath(genpath(fullfile(projectRoot, "matlab")));

modelPath = fullfile(projectRoot, "matlab", "native_training", ...
    "raw", "MATLAB_NATIVE_SPARSE_GP_MODEL.mat");
stored = load(modelPath, "nativeModel");
rawModel = stored.nativeModel;
preparedModel = gpenmpcPrepareSparseGpModel(rawModel);
priorPreparedModel = rmfield(preparedModel, ...
    "prepared_posterior_covariance_white_stack");

offsets = reshape(linspace(-0.35, 0.35, 96 * numel(rawModel.input_mean)), ...
    96, []);
inputs = rawModel.input_mean + offsets .* rawModel.input_scale;
rawPrediction = gpenmpcSparseGpPredict(rawModel, inputs);
preparedPrediction = gpenmpcSparseGpPredict(preparedModel, inputs);
priorPreparedPrediction = gpenmpcSparseGpPredict(priorPreparedModel, inputs);
predictionFields = ["mean_mps2", "raw_std_mps2", ...
    "calibrated_half_width_mps2", "latent_variance_standardized", ...
    "support_distance", "trust", "applied_mean_f_mps2"];
maximumPredictionDifference = 0.0;
for fieldName = predictionFields
    difference = max(abs(double(rawPrediction.(fieldName)) ...
        - double(preparedPrediction.(fieldName))), [], "all");
    maximumPredictionDifference = max(maximumPredictionDifference, difference);
    priorDifference = max(abs(double(priorPreparedPrediction.(fieldName)) ...
        - double(preparedPrediction.(fieldName))), [], "all");
    maximumPredictionDifference = max(maximumPredictionDifference, priorDifference);
end
assert(maximumPredictionDifference <= 1.0e-12);
assert(isequal(rawPrediction.hard_invalid, preparedPrediction.hard_invalid));

invalidInput = rawModel.input_mean + 1.0e6 .* rawModel.input_scale;
invalidRaw = gpenmpcSparseGpPredict(rawModel, invalidInput);
invalidPrepared = gpenmpcSparseGpPredict(preparedModel, invalidInput);
assert(invalidRaw.hard_invalid && invalidPrepared.hard_invalid);
assert(all(invalidRaw.applied_mean_f_mps2 == 0.0, "all"));
assert(all(invalidPrepared.applied_mean_f_mps2 == 0.0, "all"));

repeatCount = 20;
startedRaw = tic;
for index = 1:repeatCount
    gpenmpcSparseGpPredict(rawModel, inputs);
end
rawElapsedS = toc(startedRaw);
startedPrepared = tic;
for index = 1:repeatCount
    gpenmpcSparseGpPredict(preparedModel, inputs);
end
preparedElapsedS = toc(startedPrepared);

singleInput = inputs(37, :);
singleRepeatCount = 400;
gpenmpcSparseGpPredict(priorPreparedModel, singleInput);
gpenmpcSparseGpPredict(preparedModel, singleInput);
startedPriorPreparedSingle = tic;
for index = 1:singleRepeatCount
    gpenmpcSparseGpPredict(priorPreparedModel, singleInput);
end
priorPreparedSingleElapsedS = toc(startedPriorPreparedSingle);
startedPreparedSingle = tic;
for index = 1:singleRepeatCount
    gpenmpcSparseGpPredict(preparedModel, singleInput);
end
preparedSingleElapsedS = toc(startedPreparedSingle);

runtimePath = fullfile(projectRoot, "matlab", "simulink", "assets", ...
    "enmpc_runtime_configuration.json");
config = gpenmpcLoadOrdinaryB2Config(runtimePath, 0.30);
config.solver_deadline_s = 10.0;
warmStart = zeros(1, config.decision_dimension);
gpMeans = zeros(config.horizon_steps, 3);

legacyCount = 0;
legacyEvaluate = @legacyMock;
legacyInitial = legacyEvaluate(warmStart);
[legacyDecision, legacyWarm, legacyAudit] = ...
    gpenmpcSolveOrdinaryB2FiniteShooting(warmStart, gpMeans, ...
    legacyEvaluate, @b1Mock, config, tic);

reusedCount = 0;
reusedEvaluate = @reusedEvaluationMock;
reusedInitial = reusedEvaluate(warmStart);
[reusedDecision, reusedWarm, reusedAudit] = ...
    gpenmpcSolveOrdinaryB2FiniteShooting(warmStart, gpMeans, ...
    reusedEvaluate, @b1Mock, config, tic, reusedInitial);

assert(isequaln(legacyInitial, reusedInitial));
legacyComparableDecision = rmfield(legacyDecision, "elapsed_seconds");
reusedComparableDecision = rmfield(reusedDecision, "elapsed_seconds");
assert(isequaln(legacyComparableDecision, reusedComparableDecision));
assert(isequaln(legacyWarm, reusedWarm));
assert(legacyCount == reusedCount + 1);
assert(~legacyAudit.warm_evaluation_reused);
assert(reusedAudit.warm_evaluation_reused);
assert(legacyAudit.rollout_evaluation_count ...
    == reusedAudit.rollout_evaluation_count + 1);

result = struct;
result.schema = "GPENMPC_INFERENCE_AND_SOLVER_EQUIVALENCE_V1";
result.status = "PASS";
result.maximum_prediction_difference = maximumPredictionDifference;
result.hard_invalid_exact_zero_fallback = true;
result.gp_batch_rows = size(inputs, 1);
result.gp_timing_repeats = repeatCount;
result.raw_gp_elapsed_s = rawElapsedS;
result.prepared_gp_elapsed_s = preparedElapsedS;
result.gp_elapsed_ratio_prepared_over_raw = preparedElapsedS / rawElapsedS;
result.gp_single_row_timing_repeats = singleRepeatCount;
result.prior_prepared_single_row_elapsed_s = priorPreparedSingleElapsedS;
result.prepared_single_row_elapsed_s = preparedSingleElapsedS;
result.gp_single_row_elapsed_ratio_new_over_prior = ...
    preparedSingleElapsedS / priorPreparedSingleElapsedS;
result.legacy_outer_rollout_calls = legacyCount;
result.supervisor_rollout_calls = reusedCount;
result.warm_start_rollouts_removed_per_outer_update = legacyCount - reusedCount;
result.decision_equivalent_excluding_wall_clock = ...
    isequaln(legacyComparableDecision, reusedComparableDecision);
result.next_warm_start_byte_equivalent = isequaln(legacyWarm, reusedWarm);

if strlength(outputPath) > 0
    parent = fileparts(outputPath);
    if ~isfolder(parent)
        mkdir(parent);
    end
    handle = fopen(outputPath, "w");
    cleaner = onCleanup(@() fclose(handle));
    fprintf(handle, "%s\n", jsonencode(result, PrettyPrint=true));
end
fprintf("%s\n", jsonencode(result));

    function item = legacyMock(candidate)
        legacyCount = legacyCount + 1;
        item = mockEvaluation(candidate);
    end

    function item = reusedEvaluationMock(candidate)
        reusedCount = reusedCount + 1;
        item = mockEvaluation(candidate);
    end

    function item = b1Mock(candidate)
        item = mockEvaluation(candidate);
    end
end


function item = mockEvaluation(candidate)
item = struct;
item.objective = sum(double(candidate).^2);
item.constraints = ones(121, 1);
item.hard_constraints = item.constraints;
item.risk_violation = 0.0;
item.tracking_zone_violation = 0.0;
item.predicted_energy_j = 100.0;
item.terminal_energy_to_go_j = 20.0;
item.terminal_progress_s = 1.0;
item.mean_trust = 0.5;
item.hard_invalid = false;
item.current_hard_invalid = false;
end
