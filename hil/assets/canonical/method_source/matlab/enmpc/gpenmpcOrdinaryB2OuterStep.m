function [decision, nextWarmStart, audit] = gpenmpcOrdinaryB2OuterStep(warmStart, trajectory, phaseState, observation, context, gpModel, causalContextF17, config)
%GPENMPCORDINARYB2OUTERSTEP Stable host-callable ordinary-B2 outer update.
%
% B1 and B2 share the same physical candidate library. GP mean appears only
% inside the B2 predictive dynamics. A current-sample hard-invalid result
% invokes the exact same-start B1 solve.

arguments
    warmStart (1,:) double
    trajectory (1,1) struct
    phaseState (1,1) struct
    observation (1,1) struct
    context (1,1) struct
    gpModel (1,1) struct
    causalContextF17 (1,1) struct
    config (1,1) struct
end
started = tic;
preparedTrajectory = gpenmpcPrepareTrajectoryDerivatives(trajectory, 3);
initial = gpenmpcEvaluateOrdinaryB2Rollout(warmStart, config.method, ...
    preparedTrajectory, phaseState, observation, context, gpModel, ...
    causalContextF17, config);
gpMeans = reshape([initial.rows.gp_mean_i_mps2], 3, []).';
b2Evaluate = @(candidate) gpenmpcEvaluateOrdinaryB2Rollout(candidate, ...
    config.method, preparedTrajectory, phaseState, observation, context, gpModel, ...
    causalContextF17, config);
b1Evaluate = @(candidate) gpenmpcEvaluateOrdinaryB2Rollout(candidate, ...
    config.b1_fallback_method, preparedTrajectory, phaseState, observation, context, ...
    struct, struct("valid", false, "values", zeros(4, 1)), config);
[decision, nextWarmStart, audit] = gpenmpcSolveOrdinaryB2FiniteShooting( ...
    warmStart, gpMeans, b2Evaluate, b1Evaluate, config, started, initial);
audit.initial_rollout = initial;
audit.outer_period_s = config.outer_period_s;
audit.prediction_step_s = config.prediction_step_s;
audit.prediction_horizon_s = config.prediction_horizon_s;
audit.method_identity = config.method;
audit.parent_comparator_identity = config.b1_fallback_method;
audit.fallback_identity = "FRESH_FULL_B1_OUTER_DEADLINE";
if isfield(config, "raw") && isfield(config.raw, "selected_gp_model_identity")
    audit.gp_model_identity = string(config.raw.selected_gp_model_identity);
else
    audit.gp_model_identity = "UNAVAILABLE";
end
audit.n1_or_b2r1_active = false;
end
