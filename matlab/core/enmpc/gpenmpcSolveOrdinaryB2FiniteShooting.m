function [decision, nextWarmStart, audit] = gpenmpcSolveOrdinaryB2FiniteShooting( ...
        warmStart, gpMeansIMps2, b2EvaluateFcn, b1EvaluateFcn, config, started, ...
        precomputedB2WarmEvaluation)
%GPENMPCSOLVEORDINARYB2FINITESHOOTING Bounded MATLAB-native ordinary-B2 core.
%
% The callbacks each accept one 1-by-7 decision and return the bookkeeping
% struct produced by a native rollout. This function owns deterministic
% candidate order, one refinement pass, feasibility-first selection, deadline
% handling, warm-start update and B1 fallback state semantics.

arguments
    warmStart (1,:) double
    gpMeansIMps2 (:,3) double
    b2EvaluateFcn (1,1) function_handle
    b1EvaluateFcn (1,1) function_handle
    config (1,1) struct
    started = []
    precomputedB2WarmEvaluation = []
end
if isempty(started)
    started = tic;
end
[b2Decision, b2Warm, b2Audit] = solveOne( ...
    config.method, warmStart, gpMeansIMps2, b2EvaluateFcn, config, started, ...
    precomputedB2WarmEvaluation);
if b2Audit.current_hard_invalid
    b1Started = tic;
    [b1Decision, nextWarmStart, b1Audit] = solveOne( ...
        config.b1_fallback_method, warmStart, zeros(0, 3), ...
        b1EvaluateFcn, config, b1Started, []);
    decision = gpenmpcLiftB1FallbackDecision( ...
        b1Decision, b2Audit.initial_mean_trust, true, ...
        "GP_HARD_INVALID", config);
    audit = fallbackAudit(b2Audit, b1Audit, "GP_HARD_INVALID");
    return;
end
if b2Audit.no_feasible_profile
    b1Started = tic;
    [b1Decision, nextWarmStart, b1Audit] = solveOne( ...
        config.b1_fallback_method, warmStart, zeros(0, 3), ...
        b1EvaluateFcn, config, b1Started, []);
    decision = gpenmpcLiftB1FallbackDecision( ...
        b1Decision, b2Audit.initial_mean_trust, b2Audit.initial_hard_invalid, ...
        "GP_NO_FEASIBLE_PROFILE", config);
    audit = fallbackAudit(b2Audit, b1Audit, "GP_NO_FEASIBLE_PROFILE");
    return;
end
decision = b2Decision;
nextWarmStart = b2Warm;
audit = b2Audit;
end


function [decision, nextWarmStart, audit] = solveOne( ...
        method, warmStart, gpMeans, evaluateFcn, config, started, ...
        precomputedWarmEvaluation)
library = gpenmpcBuildEnmpcCandidateLibrary(method, warmStart, gpMeans, config);
[primaryEvaluations, reusedWarmEvaluation] = evaluateSet( ...
    library.values, evaluateFcn, precomputedWarmEvaluation, warmStart);
initial = primaryEvaluations(1);
currentHardInvalid = logicalField(initial, "current_hard_invalid", false);
initialHardInvalid = logicalField(initial, "hard_invalid", false);
initialMeanTrust = numericField(initial, "mean_trust", 0.0);

audit = struct;
audit.schema = "GPENMPC_MATLAB_NATIVE_ENMPC_FINITE_SHOOTING_AUDIT_V1";
audit.method = method;
audit.primary_labels = library.labels;
audit.primary_candidates = library.values;
audit.primary_count = library.count;
audit.refinement_labels = strings(0, 1);
audit.refinement_candidates = zeros(0, config.decision_dimension);
audit.refinement_count = 0;
audit.evaluated_count = library.count;
audit.rollout_evaluation_count = library.count - double(reusedWarmEvaluation);
audit.warm_evaluation_reused = reusedWarmEvaluation;
audit.current_hard_invalid = currentHardInvalid;
audit.initial_hard_invalid = initialHardInvalid;
audit.initial_mean_trust = initialMeanTrust;
audit.no_feasible_profile = false;
audit.deadline_guard_truncated = false;
audit.b1_fallback_active = false;
audit.b1_fallback_trigger = "";

if currentHardInvalid && method == config.method
    decision = failureDecision(method, initial, "GP_HARD_INVALID", ...
        toc(started), library.count, false, config);
    nextWarmStart = zeros(1, config.decision_dimension);
    audit.no_feasible_profile = true;
    audit.failure_warm_start_preserved = false;
    return;
end

primarySelection = gpenmpcSelectEnmpcCandidate( ...
    library.values, primaryEvaluations, config);
allCandidates = library.values;
allEvaluations = primaryEvaluations;
if primarySelection.has_feasible
    refinement = gpenmpcBuildEnmpcRefinementCandidates( ...
        primarySelection.candidate, config);
    refinementEvaluations = evaluateSet( ...
        refinement.values, evaluateFcn, [], zeros(1, 0));
    allCandidates = [allCandidates; refinement.values];
    allEvaluations = [allEvaluations; refinementEvaluations];
    audit.refinement_labels = refinement.labels;
    audit.refinement_candidates = refinement.values;
    audit.refinement_count = refinement.count;
    audit.evaluated_count = size(allCandidates, 1);
    audit.rollout_evaluation_count = size(allCandidates, 1) ...
        - double(reusedWarmEvaluation);
end
selection = gpenmpcSelectEnmpcCandidate(allCandidates, allEvaluations, config);
audit.feasible_mask = selection.feasible_mask;
audit.selected_index = selection.index;
audit.selected_candidate = selection.candidate;
audit.raw_selected_index = selection.raw_selected_index;
audit.profile_switch_suppressed = selection.profile_switch_suppressed;
audit.primary_improvement = selection.primary_improvement;
audit.primary_equivalent = selection.primary_equivalent;
audit.risk_equivalence_tolerance = ...
    selection.risk_equivalence_tolerance;
audit.tracking_equivalence_tolerance = ...
    selection.tracking_equivalence_tolerance;
audit.candidate_commit_hysteresis_fraction = ...
    selection.candidate_commit_hysteresis_fraction;
audit.candidate_relative_objective_improvement = ...
    selection.relative_objective_improvement;
audit.prediction_effective_commit_guard_active = ...
    selection.prediction_effective_commit_guard_active;
audit.prediction_effective_execution_nonworse = ...
    selection.prediction_effective_execution_nonworse;
audit.prediction_effective_required_relative_improvement = ...
    selection.prediction_effective_required_relative_improvement;
audit.prediction_effective_uncertainty_penalty_fraction = ...
    selection.prediction_effective_uncertainty_penalty_fraction;
audit.prediction_effective_switch_penalty_fraction = ...
    selection.prediction_effective_switch_penalty_fraction;
audit.prediction_effective_normalized_candidate_change = ...
    selection.prediction_effective_normalized_candidate_change;
audit.prediction_effective_selected_jerk_term = ...
    selection.prediction_effective_selected_jerk_term;
audit.prediction_effective_warm_jerk_term = ...
    selection.prediction_effective_warm_jerk_term;
audit.prediction_effective_selected_force_variation_term = ...
    selection.prediction_effective_selected_force_variation_term;
audit.prediction_effective_warm_force_variation_term = ...
    selection.prediction_effective_warm_force_variation_term;
audit.rank_order = selection.rank_order;
audit.minimum_constraint_margins = selection.minimum_constraint_margins;
audit.candidate_labels = [audit.primary_labels; audit.refinement_labels];
audit.candidate_objective = reshape([allEvaluations.objective], [], 1);
audit.candidate_objective_vector = vertcat(allEvaluations.objective_vector);
audit.candidate_predicted_energy_j = reshape( ...
    [allEvaluations.predicted_energy_j], [], 1);
audit.candidate_terminal_energy_to_go_j = reshape( ...
    [allEvaluations.terminal_energy_to_go_j], [], 1);
audit.candidate_risk_violation = reshape( ...
    [allEvaluations.risk_violation], [], 1);
audit.candidate_tracking_zone_violation = reshape( ...
    [allEvaluations.tracking_zone_violation], [], 1);
[noEnergyIndex, energyInfluenced] = energyCounterfactualSelection( ...
    allCandidates, allEvaluations, selection.index, config);
audit.selected_index_without_energy_term = noEnergyIndex;
audit.energy_term_changed_selected_candidate = energyInfluenced;
audit = addBestInfeasibleDiagnostics(audit, allEvaluations, selection, config);

elapsedS = toc(started);
if ~selection.has_feasible
    audit.no_feasible_profile = true;
    decision = failureDecision(method, initial, ...
        "NO_FEASIBLE_SHOOTING_PROFILE", elapsedS, ...
        size(allCandidates, 1), false, config);
    nextWarmStart = zeros(1, config.decision_dimension);
    audit.failure_warm_start_preserved = false;
    return;
end

withinDeadline = elapsedS <= config.solver_deadline_s;
if withinDeadline
    decision = successDecision(method, selection.candidate, ...
        selection.evaluation, elapsedS, size(allCandidates, 1), config);
    decision.profile_switch_suppressed = ...
        selection.profile_switch_suppressed;
    decision.raw_selected_index = selection.raw_selected_index;
    decision.committed_selected_index = selection.index;
    decision.prediction_effective_commit_guard_active = ...
        selection.prediction_effective_commit_guard_active;
    decision.prediction_effective_execution_nonworse = ...
        selection.prediction_effective_execution_nonworse;
    decision.prediction_effective_required_relative_improvement = ...
        selection.prediction_effective_required_relative_improvement;
    nextWarmStart = gpenmpcShiftEnmpcWarmStart( ...
        selection.candidate, true, config);
else
    decision = failureDecision(method, selection.evaluation, ...
        "SOLVER_DEADLINE_MISSED", elapsedS, size(allCandidates, 1), ...
        false, config);
    nextWarmStart = zeros(1, config.decision_dimension);
    audit.failure_warm_start_preserved = false;
end
end


function [index, changed] = energyCounterfactualSelection( ...
        candidates, evaluations, selectedIndex, config)
counterfactual = evaluations;
for cursor = 1:numel(counterfactual)
    if isfield(counterfactual(cursor), "objective_terms") ...
            && isfield(counterfactual(cursor).objective_terms, "energy")
        energy = double(counterfactual(cursor).objective_terms.energy);
        counterfactual(cursor).objective = ...
            double(counterfactual(cursor).objective) - energy;
        counterfactual(cursor).objective_terms.energy = 0.0;
        if isfield(counterfactual(cursor), "objective_vector") ...
                && ~isempty(counterfactual(cursor).objective_vector)
            counterfactual(cursor).objective_vector(1) = 0.0;
        end
    end
end
selection = gpenmpcSelectEnmpcCandidate(candidates, counterfactual, config);
index = double(selection.index);
changed = selection.has_feasible && selectedIndex > 0 && index ~= selectedIndex;
end


function audit = addBestInfeasibleDiagnostics(audit, evaluations, selection, config)
margins = double(selection.minimum_constraint_margins(:));
finite = find(isfinite(margins));
if isempty(finite)
    audit.best_candidate_index_by_margin = 0;
    audit.best_minimum_constraint_margin = NaN;
    audit.argmin_constraint_name = "UNAVAILABLE";
    audit.argmin_horizon_step = 0;
    return;
end
[bestMargin, localIndex] = max(margins(finite));
candidateIndex = finite(localIndex);
constraints = double(evaluations(candidateIndex).constraints(:));
[~, constraintIndex] = min(constraints);
[name, horizonStep] = decodeConstraint(constraintIndex, config);
audit.best_candidate_index_by_margin = candidateIndex;
audit.best_minimum_constraint_margin = bestMargin;
audit.argmin_constraint_name = name;
audit.argmin_horizon_step = horizonStep;
end


function [name, horizonStep] = decodeConstraint(index, config)
names = ["PHASE_RATE_MIN", "PHASE_RATE_MAX", "PHASE_MIN", ...
    "PHASE_MAX", "CORRIDOR_POSITION", "POSITION_SCREEN", ...
    "VELOCITY_SCREEN", "PREDICTED_ACCELERATION", ...
    "REFERENCE_JERK", "TOTAL_THRUST", "PER_ROTOR_THRUST", ...
    "TILT", "AIRSPEED", "POWER_DOMAIN", "OUTER_CORRECTION"];
stageCount = config.horizon_steps * config.hard_constraints_per_step;
if index > stageCount
    name = "GP_VALIDITY_SENTINEL";
    horizonStep = 0;
    return;
end
horizonStep = ceil(index / config.hard_constraints_per_step);
localIndex = mod(index - 1, config.hard_constraints_per_step) + 1;
if localIndex <= numel(names)
    name = names(localIndex);
else
    name = "HARD_CONSTRAINT_" + string(localIndex);
end
end


function [evaluations, reusedFirst] = evaluateSet( ...
        candidates, evaluateFcn, precomputedFirst, expectedFirstCandidate)
count = size(candidates, 1);
reusedFirst = ~isempty(precomputedFirst) ...
    && isequal(candidates(1, :), double(expectedFirstCandidate(:).'));
if reusedFirst
    firstEvaluation = precomputedFirst;
else
    firstEvaluation = evaluateFcn(candidates(1, :));
end
evaluations = repmat(firstEvaluation, count, 1);
for index = 2:count
    evaluations(index) = evaluateFcn(candidates(index, :));
end
evaluations = evaluations(:);
end


function decision = successDecision(method, candidate, item, elapsedS, iterations, config)
decision = commonDecision(method, item, elapsedS, iterations, config);
decision.phase_acceleration_s_inv = candidate(1);
decision.success = true;
decision.fallback_active = false;
decision.fallback_reason = "";
decision.decision_blocks_s_inv = candidate(1:config.control_blocks);
decision.outer_acceleration_correction_f_mps2 = ...
    candidate(config.control_blocks + 1:end);
end


function decision = failureDecision(method, item, reason, elapsedS, iterations, truncated, config)
decision = commonDecision(method, item, elapsedS, iterations, config);
decision.phase_acceleration_s_inv = 0.0;
decision.success = false;
decision.fallback_active = true;
decision.fallback_reason = string(reason);
decision.decision_blocks_s_inv = zeros(1, config.control_blocks);
decision.outer_acceleration_correction_f_mps2 = zeros(1, 3);
decision.deadline_guard_truncated = logical(truncated);
end


function decision = commonDecision(method, item, elapsedS, iterations, config)
constraints = double(item.constraints(:));
decision = struct;
decision.schema = "GPENMPC_MATLAB_NATIVE_NMPC_DECISION_V1";
decision.method = method;
decision.phase_acceleration_s_inv = 0.0;
decision.objective = numericField(item, "objective", Inf);
decision.success = false;
decision.fallback_active = true;
decision.fallback_reason = "";
decision.elapsed_seconds = elapsedS;
decision.solver_iterations = iterations;
decision.minimum_constraint_margin = min(constraints);
decision.predicted_energy_j = numericField(item, "predicted_energy_j", NaN);
decision.predicted_progress_s = numericField(item, "terminal_progress_s", NaN);
decision.mean_trust = numericField(item, "mean_trust", 0.0);
decision.hard_invalid = logicalField(item, "hard_invalid", false);
decision.decision_blocks_s_inv = zeros(1, config.control_blocks);
decision.outer_acceleration_correction_f_mps2 = zeros(1, 3);
decision.deadline_guard_truncated = false;
decision.risk_violation = numericField(item, "risk_violation", 0.0);
decision.tracking_zone_violation = ...
    numericField(item, "tracking_zone_violation", 0.0);
decision.terminal_energy_to_go_j = ...
    numericField(item, "terminal_energy_to_go_j", 0.0);
decision.gp_b1_fallback_active = false;
end


function value = numericField(item, fieldName, defaultValue)
if isfield(item, fieldName)
    value = double(item.(fieldName));
else
    value = double(defaultValue);
end
end


function value = logicalField(item, fieldName, defaultValue)
if isfield(item, fieldName)
    value = logical(item.(fieldName));
else
    value = logical(defaultValue);
end
end


function audit = fallbackAudit(b2Audit, b1Audit, trigger)
audit = b2Audit;
audit.b1_fallback_active = true;
audit.b1_fallback_trigger = trigger;
audit.b1_fallback_budget_identity = "FRESH_FULL_B1_OUTER_DEADLINE";
audit.b1_fallback_full_budget = true;
audit.b1_fallback_parent_method = b2Audit.method;
audit.b1_fallback_method = b1Audit.method;
audit.b1 = b1Audit;
end
