function selection = gpenmpcSelectEnmpcCandidate(candidates, evaluations, config)
%GPENMPCSELECTENMPCCANDIDATE Apply deterministic feasibility-first ordering.
%
% Rank is lexicographic: calibrated risk violation, tracking-zone violation,
% economic objective, then the seven candidate coordinates.

arguments
    candidates (:,:) double
    evaluations (:,1) struct
    config (1,1) struct
end
if size(candidates, 2) ~= config.decision_dimension ...
        || size(candidates, 1) ~= numel(evaluations)
    error("gpenmpcSelectEnmpcCandidate:Shape", ...
        "Candidate and evaluation counts or decision dimension differ.");
end
count = size(candidates, 1);
feasible = false(count, 1);
rankMatrix = inf(count, 3 + config.decision_dimension);
minimumMargins = nan(count, 1);
for index = 1:count
    item = evaluations(index);
    required = ["objective", "constraints", "hard_invalid", ...
        "risk_violation", "tracking_zone_violation"];
    for name = required
        if ~isfield(item, name)
            error("gpenmpcSelectEnmpcCandidate:Evaluation", ...
                "Evaluation %d is missing %s.", index, name);
        end
    end
    margins = double(item.constraints(:));
    if ~isempty(margins) && all(isfinite(margins))
        minimumMargins(index) = min(margins);
    end
    objectiveFinite = isfinite(double(item.objective));
    feasible(index) = ~logical(item.hard_invalid) ...
        && ~isempty(margins) ...
        && all(isfinite(margins)) ...
        && minimumMargins(index) >= -config.feasibility_tolerance ...
        && objectiveFinite;
    if feasible(index)
        rankMatrix(index, :) = [double(item.risk_violation), ...
            double(item.tracking_zone_violation), double(item.objective), ...
            candidates(index, :)];
    end
end

feasibleIndices = find(feasible);
selection = struct;
selection.schema = "GPENMPC_MATLAB_NATIVE_ENMPC_SELECTION_V1";
selection.has_feasible = ~isempty(feasibleIndices);
selection.feasible_mask = feasible;
selection.feasible_indices = feasibleIndices;
selection.minimum_constraint_margins = minimumMargins;
selection.rank_matrix = rankMatrix;
selection.rank_order = ["CALIBRATED_RISK_VIOLATION", ...
    "TRACKING_ZONE_VIOLATION", "ECONOMIC_OBJECTIVE", "DECISION_VECTOR"];
selection.index = 0;
selection.candidate = zeros(1, config.decision_dimension);
selection.evaluation = struct;
selection.raw_selected_index = 0;
selection.warm_candidate_feasible = feasible(1);
selection.profile_switch_suppressed = false;
selection.primary_improvement = false;
selection.primary_equivalent = false;
selection.relative_objective_improvement = 0.0;
legacyTolerance = optionalScalar(config, ...
    "candidate_primary_equivalence_tolerance", 0.0);
selection.risk_equivalence_tolerance = optionalScalar(config, ...
    "coordinated_risk_equivalence_tolerance", legacyTolerance);
selection.tracking_equivalence_tolerance = optionalScalar(config, ...
    "coordinated_tracking_equivalence_tolerance", legacyTolerance);
selection.candidate_commit_hysteresis_fraction = optionalScalar(config, ...
    "coordinated_objective_relative_switch_improvement", optionalScalar( ...
    config, "candidate_commit_hysteresis_fraction", 0.0));
selection.prediction_effective_commit_guard_active = false;
selection.prediction_effective_execution_nonworse = true;
selection.prediction_effective_required_relative_improvement = ...
    selection.candidate_commit_hysteresis_fraction;
selection.prediction_effective_uncertainty_penalty_fraction = 0.0;
selection.prediction_effective_switch_penalty_fraction = 0.0;
selection.prediction_effective_normalized_candidate_change = 0.0;
selection.prediction_effective_selected_jerk_term = 0.0;
selection.prediction_effective_warm_jerk_term = 0.0;
selection.prediction_effective_selected_force_variation_term = 0.0;
selection.prediction_effective_warm_force_variation_term = 0.0;
if isempty(feasibleIndices)
    return;
end
[~, order] = sortrows(rankMatrix(feasibleIndices, :), ...
    1:size(rankMatrix, 2));
selectedIndex = feasibleIndices(order(1));
rawSelectedIndex = selectedIndex;
switchSuppressed = false;
relativeObjectiveImprovement = 0.0;
primaryImprovement = false;
primaryEquivalent = false;
if selectedIndex ~= 1 && feasible(1)
    riskTolerance = selection.risk_equivalence_tolerance;
    trackingTolerance = selection.tracking_equivalence_tolerance;
    hysteresis = selection.candidate_commit_hysteresis_fraction;
    selectedEvaluation = evaluations(selectedIndex);
    warmEvaluation = evaluations(1);
    selectedRisk = double(selectedEvaluation.risk_violation);
    warmRisk = double(warmEvaluation.risk_violation);
    selectedTracking = double(selectedEvaluation.tracking_zone_violation);
    warmTracking = double(warmEvaluation.tracking_zone_violation);
    riskEquivalent = abs(selectedRisk - warmRisk) <= riskTolerance;
    trackingEquivalent = ...
        abs(selectedTracking - warmTracking) <= trackingTolerance;
    primaryImprovement = selectedRisk < warmRisk - riskTolerance ...
        || (riskEquivalent ...
            && selectedTracking < warmTracking - trackingTolerance);
    primaryEquivalent = riskEquivalent && trackingEquivalent;
    selectedObjective = double(selectedEvaluation.objective);
    warmObjective = double(warmEvaluation.objective);
    relativeObjectiveImprovement = (warmObjective - selectedObjective) ...
        ./ max(abs(warmObjective), 1.0);
    guardActive = optionalLogical(config, ...
        "prediction_effective_commit_enabled", false);
    executionNonworse = true;
    requiredImprovement = hysteresis;
    uncertaintyPenalty = 0.0;
    switchPenalty = 0.0;
    normalizedChange = 0.0;
    selectedJerk = objectiveTerm(selectedEvaluation, "jerk");
    warmJerk = objectiveTerm(warmEvaluation, "jerk");
    selectedForce = objectiveTerm(selectedEvaluation, "force_variation");
    warmForce = objectiveTerm(warmEvaluation, "force_variation");
    if guardActive
        meanTrust = min(max(optionalEvaluationScalar( ...
            selectedEvaluation, "mean_trust", 0.0), 0.0), 1.0);
        baseFraction = optionalScalar(config, ...
            "prediction_effective_commit_base_fraction", hysteresis);
        uncertaintyScale = optionalScalar(config, ...
            "prediction_effective_commit_uncertainty_fraction", hysteresis);
        switchScale = optionalScalar(config, ...
            "prediction_effective_commit_switch_fraction", hysteresis);
        executionTolerance = optionalScalar(config, ...
            "prediction_effective_execution_nonworse_tolerance", 1.0e-12);
        normalizedChange = normalizedDecisionChange( ...
            candidates(selectedIndex,:), candidates(1,:), config);
        uncertaintyPenalty = uncertaintyScale .* (1.0 - meanTrust);
        switchPenalty = switchScale .* normalizedChange;
        requiredImprovement = baseFraction + uncertaintyPenalty + switchPenalty;
        executionNonworse = selectedJerk <= warmJerk + executionTolerance ...
            && selectedForce <= warmForce + executionTolerance;
    end
    selection.prediction_effective_commit_guard_active = guardActive;
    selection.prediction_effective_execution_nonworse = executionNonworse;
    selection.prediction_effective_required_relative_improvement = ...
        requiredImprovement;
    selection.prediction_effective_uncertainty_penalty_fraction = ...
        uncertaintyPenalty;
    selection.prediction_effective_switch_penalty_fraction = switchPenalty;
    selection.prediction_effective_normalized_candidate_change = normalizedChange;
    selection.prediction_effective_selected_jerk_term = selectedJerk;
    selection.prediction_effective_warm_jerk_term = warmJerk;
    selection.prediction_effective_selected_force_variation_term = selectedForce;
    selection.prediction_effective_warm_force_variation_term = warmForce;
    if ~primaryImprovement && (~primaryEquivalent ...
            || relativeObjectiveImprovement < requiredImprovement ...
            || ~executionNonworse)
        selectedIndex = 1;
        switchSuppressed = true;
    end
end
selection.index = selectedIndex;
selection.candidate = candidates(selectedIndex, :);
selection.evaluation = evaluations(selectedIndex);
selection.raw_selected_index = rawSelectedIndex;
selection.warm_candidate_feasible = feasible(1);
selection.profile_switch_suppressed = switchSuppressed;
selection.primary_improvement = primaryImprovement;
selection.primary_equivalent = primaryEquivalent;
selection.relative_objective_improvement = relativeObjectiveImprovement;
selection.risk_equivalence_tolerance = optionalScalar(config, ...
    "coordinated_risk_equivalence_tolerance", legacyTolerance);
selection.tracking_equivalence_tolerance = optionalScalar(config, ...
    "coordinated_tracking_equivalence_tolerance", legacyTolerance);
selection.candidate_commit_hysteresis_fraction = optionalScalar(config, ...
    "coordinated_objective_relative_switch_improvement", optionalScalar( ...
    config, "candidate_commit_hysteresis_fraction", 0.0));
end


function value = optionalLogical(source, name, fallback)
if isfield(source, name)
    value = logical(source.(name));
else
    value = logical(fallback);
end
if ~isscalar(value)
    error("gpenmpcSelectEnmpcCandidate:Config", ...
        "Optional selector value %s must be a logical scalar.", name);
end
end


function value = optionalEvaluationScalar(source, name, fallback)
if isfield(source, name)
    value = double(source.(name));
else
    value = double(fallback);
end
if ~isscalar(value) || ~isfinite(value)
    value = double(fallback);
end
end


function value = objectiveTerm(item, name)
value = 0.0;
if isfield(item, "objective_terms") && isfield(item.objective_terms, name)
    candidate = double(item.objective_terms.(name));
    if isscalar(candidate) && isfinite(candidate)
        value = candidate;
    end
end
end


function value = normalizedDecisionChange(candidate, warm, config)
phaseBound = optionalScalar(config, "phase_acceleration_max_s_inv", 1.0);
correctionBound = optionalScalar( ...
    config, "outer_acceleration_correction_max_mps2", 1.0);
blockCount = min(optionalScalar(config, "control_blocks", 0.0), ...
    numel(candidate));
scale = correctionBound .* ones(size(candidate));
if blockCount > 0
    scale(1:blockCount) = phaseBound;
end
scale = max(scale, eps);
value = norm((double(candidate) - double(warm)) ./ scale, 2) ...
    ./ sqrt(numel(candidate));
value = min(max(value, 0.0), 1.0);
end


function value = optionalScalar(source, name, fallback)
if isfield(source, name)
    value = double(source.(name));
else
    value = double(fallback);
end
if ~isscalar(value) || ~isfinite(value) || value < 0.0
    error("gpenmpcSelectEnmpcCandidate:Config", ...
        "Optional selector value %s must be a finite nonnegative scalar.", ...
        name);
end
end
