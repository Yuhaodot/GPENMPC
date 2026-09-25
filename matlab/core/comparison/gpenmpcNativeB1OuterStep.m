function [decision, nextWarmStart, audit] = gpenmpcNativeB1OuterStep( ...
        warmStart, trajectory, phaseState, observation, context, config)
%GPENMPCNATIVEB1OUTERSTEP Native no-GP eNMPC solve with deterministic candidates.

arguments
    warmStart (1,:) double
    trajectory (1,1) struct
    phaseState (1,1) struct
    observation (1,1) struct
    context (1,1) struct
    config (1,1) struct
end
started = tic;
evaluate = @(candidate) gpenmpcEvaluateOrdinaryB2Rollout(candidate, ...
    config.b1_fallback_method, trajectory, phaseState, observation, context, ...
    struct, struct("valid", false, "values", zeros(4, 1)), config);
library = gpenmpcBuildEnmpcCandidateLibrary(config.b1_fallback_method, ...
    warmStart, zeros(0, 3), config);
primary = evaluateSet(library.values, evaluate);
selection = gpenmpcSelectEnmpcCandidate(library.values, primary, config);
candidates = library.values;
evaluations = primary;
refinement = struct("labels", strings(0,1), "values", zeros(0,7), "count", 0);
if selection.has_feasible
    refinement = gpenmpcBuildEnmpcRefinementCandidates(selection.candidate, config);
    refined = evaluateSet(refinement.values, evaluate);
    candidates = [candidates; refinement.values];
    evaluations = [evaluations; refined];
    selection = gpenmpcSelectEnmpcCandidate(candidates, evaluations, config);
end
elapsed = toc(started);
if selection.has_feasible && elapsed <= config.solver_deadline_s
    item = selection.evaluation;
    candidate = selection.candidate;
    decision = commonDecision(item, elapsed, size(candidates, 1), config);
    decision.success = true;
    decision.fallback_active = false;
    decision.fallback_reason = "";
    decision.phase_acceleration_s_inv = candidate(1);
    decision.decision_blocks_s_inv = candidate(1:config.control_blocks);
    decision.outer_acceleration_correction_f_mps2 = ...
        candidate(config.control_blocks + (1:3));
    nextWarmStart = gpenmpcShiftEnmpcWarmStart(candidate, true, config);
elseif ~selection.has_feasible
    decision = commonDecision(primary(1), elapsed, size(candidates, 1), config);
    decision.fallback_reason = "NO_FEASIBLE_SHOOTING_PROFILE";
    nextWarmStart = zeros(1, config.decision_dimension);
else
    decision = commonDecision(selection.evaluation, elapsed, ...
        size(candidates, 1), config);
    decision.fallback_reason = "SOLVER_DEADLINE_MISSED";
    nextWarmStart = zeros(1, config.decision_dimension);
end
diagnostics = bestConstraintDiagnostics(evaluations, selection, config);
audit = struct( ...
    "schema", "GPENMPC_MATLAB_NATIVE_B1_OUTER_AUDIT_V1", ...
    "method", config.b1_fallback_method, ...
    "primary_count", library.count, ...
    "refinement_count", refinement.count, ...
    "evaluated_count", size(candidates, 1), ...
    "selected_index", selection.index, ...
    "selected_candidate", selection.candidate, ...
    "minimum_constraint_margins", selection.minimum_constraint_margins, ...
    "best_candidate_index_by_margin", diagnostics.candidate_index, ...
    "best_minimum_constraint_margin", diagnostics.minimum_margin, ...
    "argmin_constraint_name", diagnostics.constraint_name, ...
    "argmin_horizon_step", diagnostics.horizon_step, ...
    "primary_improvement", selection.primary_improvement, ...
    "primary_equivalent", selection.primary_equivalent, ...
    "risk_equivalence_tolerance", selection.risk_equivalence_tolerance, ...
    "tracking_equivalence_tolerance", ...
        selection.tracking_equivalence_tolerance, ...
    "candidate_commit_hysteresis_fraction", ...
        selection.candidate_commit_hysteresis_fraction, ...
    "candidate_relative_objective_improvement", ...
        selection.relative_objective_improvement, ...
    "failure_warm_start_preserved", false, ...
    "no_feasible_profile", ~selection.has_feasible, ...
    "b1_fallback_active", false, ...
    "b1_fallback_trigger", "", ...
    "outer_period_s", config.outer_period_s, ...
    "prediction_step_s", config.prediction_step_s, ...
    "prediction_horizon_s", config.prediction_horizon_s);
end

function result = bestConstraintDiagnostics(evaluations, selection, config)
margins = double(selection.minimum_constraint_margins(:));
finite = find(isfinite(margins));
result = struct("candidate_index", 0, "minimum_margin", NaN, ...
    "constraint_name", "UNAVAILABLE", "horizon_step", 0);
if isempty(finite), return; end
[result.minimum_margin, localIndex] = max(margins(finite));
result.candidate_index = finite(localIndex);
constraints = double(evaluations(result.candidate_index).constraints(:));
[~, constraintIndex] = min(constraints);
names = ["PHASE_RATE_MIN", "PHASE_RATE_MAX", "PHASE_MIN", ...
    "PHASE_MAX", "CORRIDOR_POSITION", "POSITION_SCREEN", ...
    "VELOCITY_SCREEN", "PREDICTED_ACCELERATION", ...
    "REFERENCE_JERK", "TOTAL_THRUST", "PER_ROTOR_THRUST", ...
    "TILT", "AIRSPEED", "POWER_DOMAIN", "OUTER_CORRECTION"];
stageCount = config.horizon_steps * config.hard_constraints_per_step;
if constraintIndex > stageCount
    result.constraint_name = "GP_VALIDITY_SENTINEL";
    return;
end
result.horizon_step = ceil(constraintIndex / config.hard_constraints_per_step);
index = mod(constraintIndex - 1, config.hard_constraints_per_step) + 1;
if index <= numel(names)
    result.constraint_name = names(index);
else
    result.constraint_name = "HARD_CONSTRAINT_" + string(index);
end
end

function evaluations = evaluateSet(candidates, evaluate)
count = size(candidates, 1);
evaluations = repmat(evaluate(candidates(1,:)), count, 1);
for index = 2:count
    evaluations(index) = evaluate(candidates(index,:));
end
evaluations = evaluations(:);
end

function decision = commonDecision(item, elapsed, iterations, config)
constraints = double(item.constraints(:));
decision = struct;
decision.schema = "GPENMPC_MATLAB_NATIVE_NMPC_DECISION_V1";
decision.method = config.b1_fallback_method;
decision.phase_acceleration_s_inv = 0.0;
decision.objective = double(item.objective);
decision.success = false;
decision.fallback_active = true;
decision.fallback_reason = "";
decision.elapsed_seconds = elapsed;
decision.solver_iterations = iterations;
decision.minimum_constraint_margin = min(constraints);
decision.predicted_energy_j = double(item.predicted_energy_j);
decision.predicted_progress_s = double(item.terminal_progress_s);
decision.mean_trust = 0.0;
decision.hard_invalid = false;
decision.decision_blocks_s_inv = zeros(1, config.control_blocks);
decision.outer_acceleration_correction_f_mps2 = zeros(1, 3);
decision.deadline_guard_truncated = false;
decision.risk_violation = double(item.risk_violation);
decision.tracking_zone_violation = double(item.tracking_zone_violation);
decision.terminal_energy_to_go_j = double(item.terminal_energy_to_go_j);
decision.gp_b1_fallback_active = false;
end
