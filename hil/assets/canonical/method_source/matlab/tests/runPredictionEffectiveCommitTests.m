function result = runPredictionEffectiveCommitTests(projectRoot, implementationRoot, outputPath)
%RUNPREDICTIONEFFECTIVECOMMITTESTS Test the prediction-effective commit policy.

arguments
    projectRoot (1,1) string
    implementationRoot (1,1) string
    outputPath (1,1) string = ""
end

originalPath = path;
cleanup = onCleanup(@() path(originalPath)); %#ok<NASGU>
addpath(genpath(fullfile(projectRoot, "matlab")), "-end");
addpath(genpath(implementationRoot), "-begin");
rehash path;

runtimePath = fullfile(projectRoot, "matlab", "simulink", "assets", ...
    "enmpc_runtime_configuration.json");
config = gpenmpcLoadOrdinaryB2Config(runtimePath, 0.30);
assert(~config.prediction_effective_commit_enabled);
assert(~config.prediction_effective_direct_b2_enabled);

config.prediction_effective_commit_enabled = true;
config.prediction_effective_direct_b2_enabled = true;
config.prediction_effective_commit_base_fraction = 0.005;
config.prediction_effective_commit_uncertainty_fraction = 0.005;
config.prediction_effective_commit_switch_fraction = 0.005;
config.prediction_effective_execution_nonworse_tolerance = 1.0e-12;

warm = zeros(1, config.decision_dimension);
candidate = [0.02, 0.02, 0.02, 0.02, 0.01, -0.01, 0.00];
candidates = [warm; candidate];

% A material, trusted, execution-aligned improvement is committed.
evaluations = [evaluation(100.0, 0.0, 0.0, 1.0, 1.0, 1.0); ...
    evaluation(98.0, 0.0, 0.0, 1.0, 0.9, 0.8)];
selection = gpenmpcSelectEnmpcCandidate(candidates, evaluations, config);
assert(selection.index == 2);
assert(selection.raw_selected_index == 2);
assert(selection.prediction_effective_commit_guard_active);
assert(selection.prediction_effective_execution_nonworse);

% A lower scalar objective cannot hide worse predicted execution activity.
evaluations(2) = evaluation(98.0, 0.0, 0.0, 1.0, 1.1, 0.8);
selection = gpenmpcSelectEnmpcCandidate(candidates, evaluations, config);
assert(selection.index == 1);
assert(selection.raw_selected_index == 2);
assert(selection.profile_switch_suppressed);
assert(~selection.prediction_effective_execution_nonworse);

% Low trust raises the evidence threshold above the legacy 0.5 percent rule.
evaluations(2) = evaluation(99.3, 0.0, 0.0, 0.0, 0.9, 0.8);
selection = gpenmpcSelectEnmpcCandidate(candidates, evaluations, config);
assert(selection.index == 1);
assert(selection.relative_objective_improvement > 0.005);
assert(selection.prediction_effective_required_relative_improvement > 0.005);

% A genuine primary risk improvement retains lexicographic priority.
evaluations = [evaluation(100.0, 0.20, 0.0, 0.3, 1.0, 1.0); ...
    evaluation(101.0, 0.10, 0.0, 0.3, 1.2, 1.2)];
selection = gpenmpcSelectEnmpcCandidate(candidates, evaluations, config);
assert(selection.index == 2);
assert(selection.primary_improvement);

% Opt-out reproduces the prior selector semantics.
config.prediction_effective_commit_enabled = false;
evaluations = [evaluation(100.0, 0.0, 0.0, 0.0, 1.0, 1.0); ...
    evaluation(99.3, 0.0, 0.0, 0.0, 1.2, 1.2)];
selection = gpenmpcSelectEnmpcCandidate(candidates, evaluations, config);
assert(selection.index == 2);

runnerText = fileread(fullfile(implementationRoot, "matlab", ...
    "comparison", "gpenmpcRunNativeEnmpcWholeTask.m"));
assert(contains(runnerText, "prediction_effective_direct_b2_enabled"));
assert(contains(runnerText, "gpenmpcOrdinaryB2OuterStep"));
assert(contains(runnerText, "gp_b1_fallback_active"));

result = struct( ...
    "schema", "GPENMPC_PREDICTION_EFFECTIVE_COMMIT_TEST_RESULT_V1", ...
    "status", "PASS", ...
    "test_count", 7, ...
    "outer_period_s", config.outer_period_s, ...
    "prediction_step_s", config.prediction_step_s, ...
    "horizon_steps", config.horizon_steps, ...
    "solver_deadline_s", config.solver_deadline_s, ...
    "formal_cases", 0);
if strlength(outputPath) > 0
    folder = fileparts(outputPath);
    if ~isfolder(folder), mkdir(folder); end
    fid = fopen(outputPath, "w");
    assert(fid >= 0);
    fileCleanup = onCleanup(@() fclose(fid)); %#ok<NASGU>
    fwrite(fid, jsonencode(result, PrettyPrint=true), "char");
end
fprintf("PREDICTION_EFFECTIVE_COMMIT_TESTS_PASS count=%d\n", result.test_count);
end


function item = evaluation(objective, risk, tracking, trust, jerk, force)
item = struct( ...
    "objective", objective, ...
    "constraints", ones(121, 1), ...
    "hard_invalid", false, ...
    "risk_violation", risk, ...
    "tracking_zone_violation", tracking, ...
    "mean_trust", trust, ...
    "objective_terms", struct("jerk", jerk, "force_variation", force));
end
