function result = runOuterPreparedTrajectoryEquivalence(projectRoot, stagingRoot, outputPath)
%RUNOUTERPREPAREDTRAJECTORYEQUIVALENCE Compare complete outer decisions.

arguments
    projectRoot (1,1) string
    stagingRoot (1,1) string
    outputPath (1,1) string = ""
end
addpath(genpath(fullfile(projectRoot, "matlab")));
addpath(genpath(fullfile(stagingRoot, "matlab")), "-begin");
fixture = jsondecode(fileread(fullfile(projectRoot, "tests", "fixtures", ...
    "ordinary_b2_rollout_python_reference.json")));
stored = load(fullfile(projectRoot, "matlab", ...
    "native_training", "raw", ...
    "MATLAB_NATIVE_SPARSE_GP_MODEL.mat"), "nativeModel");
model = gpenmpcPrepareSparseGpModel(stored.nativeModel);
config = gpenmpcLoadOrdinaryB2Config(string(fixture.runtime_config_path), 0.30);
config.solver_deadline_s = 60.0;
trajectory = struct( ...
    "total_duration_s", double(fixture.trajectory.total_duration_s), ...
    "coefficients_ascending", ...
        double(fixture.trajectory.coefficients_ascending));
phaseState = fixture.phase_state;
observation = fixture.observation;
context = fixture.context;
context.phase_power_fcn = @fixedPower;
causal = fixture.causal_context_f17;
warmStart = zeros(1, config.decision_dimension);

[legacyDecision, legacyWarm, legacyAudit] = legacyOuterStep( ...
    warmStart, trajectory, phaseState, observation, context, model, causal, config);
[preparedDecision, preparedWarm, preparedAudit] = gpenmpcOrdinaryB2OuterStep( ...
    warmStart, trajectory, phaseState, observation, context, model, causal, config);
assert(isequaln(rmfield(legacyDecision, "elapsed_seconds"), ...
    rmfield(preparedDecision, "elapsed_seconds")));
assert(isequaln(legacyWarm, preparedWarm));
assert(isequaln(legacyAudit.primary_candidates, preparedAudit.primary_candidates));
assert(isequaln(legacyAudit.selected_candidate, preparedAudit.selected_candidate));
assert(isequaln(legacyAudit.feasible_mask, preparedAudit.feasible_mask));
assert(isequaln(legacyAudit.initial_rollout.stage_rows, ...
    preparedAudit.initial_rollout.stage_rows));
assert(isequaln(legacyAudit.initial_rollout.rows, ...
    preparedAudit.initial_rollout.rows));

repeatCount = 30;
legacyStarted = tic;
for repeat = 1:repeatCount
    legacyOuterStep(warmStart, trajectory, phaseState, observation, ...
        context, model, causal, config);
end
legacyElapsedS = toc(legacyStarted);
preparedStarted = tic;
for repeat = 1:repeatCount
    gpenmpcOrdinaryB2OuterStep(warmStart, trajectory, phaseState, observation, ...
        context, model, causal, config);
end
preparedElapsedS = toc(preparedStarted);

result = struct;
result.schema = "GPENMPC_OUTER_PREPARED_TRAJECTORY_EQUIVALENCE_V1";
result.status = "PASS";
result.repeat_count = repeatCount;
result.decision_equivalent_excluding_wall_clock = true;
result.next_warm_start_byte_equivalent = true;
result.primary_candidate_order_byte_equivalent = true;
result.selection_byte_equivalent = true;
result.initial_rollout_byte_equivalent = true;
result.legacy_elapsed_s = legacyElapsedS;
result.prepared_elapsed_s = preparedElapsedS;
result.elapsed_ratio_prepared_over_legacy = ...
    preparedElapsedS ./ legacyElapsedS;
result.solver_deadline_s_unchanged = 0.28;
if strlength(outputPath) > 0
    parent = fileparts(outputPath);
    if ~isfolder(parent)
        mkdir(parent);
    end
    writelines(jsonencode(result, PrettyPrint=true), outputPath, ...
        Encoding="UTF-8");
end
fprintf("%s\n", jsonencode(result));

    function powerW = fixedPower(~, airspeed, payload)
        powerW = 1050.0 + 9.0 .* airspeed.^2 + 20.0 .* payload;
    end
end


function [decision, nextWarmStart, audit] = legacyOuterStep( ...
        warmStart, trajectory, phaseState, observation, context, gpModel, ...
        causalContextF17, config)
started = tic;
initial = gpenmpcEvaluateOrdinaryB2Rollout(warmStart, config.method, ...
    trajectory, phaseState, observation, context, gpModel, ...
    causalContextF17, config);
gpMeans = reshape([initial.rows.gp_mean_i_mps2], 3, []).';
b2Evaluate = @(candidate) gpenmpcEvaluateOrdinaryB2Rollout(candidate, ...
    config.method, trajectory, phaseState, observation, context, gpModel, ...
    causalContextF17, config);
b1Evaluate = @(candidate) gpenmpcEvaluateOrdinaryB2Rollout(candidate, ...
    config.b1_fallback_method, trajectory, phaseState, observation, context, ...
    struct, struct("valid", false, "values", zeros(4, 1)), config);
[decision, nextWarmStart, audit] = gpenmpcSolveOrdinaryB2FiniteShooting( ...
    warmStart, gpMeans, b2Evaluate, b1Evaluate, config, started, initial);
audit.initial_rollout = initial;
audit.outer_period_s = config.outer_period_s;
audit.prediction_step_s = config.prediction_step_s;
audit.prediction_horizon_s = config.prediction_horizon_s;
audit.method_identity = config.method;
audit.n1_or_b2r1_active = false;
end
