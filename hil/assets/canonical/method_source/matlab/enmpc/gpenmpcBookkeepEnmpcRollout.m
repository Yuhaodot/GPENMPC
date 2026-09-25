function result = gpenmpcBookkeepEnmpcRollout(stageRows, terminalEnergyToGoJ, hardInvalid, currentHardInvalid, config)
%GPENMPCBOOKKEEPENMPCROLLOUT Aggregate the configured objective and constraints.
%
% This function aggregates scalar terms from exactly eight rollout stages,
% each providing fifteen hard margins and six normalized risk margins.
% Positive margins are feasible; the final hard margin is the invariant
% GP-validity sentinel.

arguments
    stageRows (:,1) struct
    terminalEnergyToGoJ (1,1) double
    hardInvalid (1,1) logical
    currentHardInvalid (1,1) logical
    config (1,1) struct
end
if numel(stageRows) ~= config.horizon_steps
    error("gpenmpcBookkeepEnmpcRollout:Horizon", ...
        "Bookkeeping requires exactly %d stage rows.", config.horizon_steps);
end
if ~isfinite(terminalEnergyToGoJ) || terminalEnergyToGoJ < 0.0
    error("gpenmpcBookkeepEnmpcRollout:TerminalEnergy", ...
        "Terminal energy-to-go must be finite and nonnegative.");
end

objectiveTerms = struct( ...
    "energy", 0.0, ...
    "position", 0.0, ...
    "velocity", 0.0, ...
    "force_variation", 0.0, ...
    "jerk", 0.0, ...
    "phase_acceleration", 0.0, ...
    "phase_acceleration_variation", 0.0, ...
    "progress", 0.0, ...
    "terminal_energy_to_go", 0.0);
hardConstraints = zeros(1, config.expected_hard_constraint_count);
riskConstraints = zeros(1, config.expected_risk_constraint_count);
predictedEnergyJ = 0.0;
trackingZoneViolation = 0.0;
trusts = zeros(config.horizon_steps, 1);
hardCursor = 1;
riskCursor = 1;

for index = 1:config.horizon_steps
    row = stageRows(index);
    requiredFiniteScalar(row, "stage_energy_j");
    requiredFiniteScalar(row, "position_error_norm_m");
    requiredFiniteScalar(row, "velocity_error_norm_mps");
    requiredFiniteScalar(row, "force_variation_norm_n");
    if executionConsistentCostEnabled(config)
        requiredFiniteScalar(row, "execution_rotor_variation_n");
    end
    requiredFiniteScalar(row, "reference_jerk_norm_mps3");
    requiredFiniteScalar(row, "phase_acceleration_s_inv");
    requiredFiniteScalar(row, "phase_jerk_s_inv2");
    requiredFiniteScalar(row, "progress_deficit_s");
    requiredFiniteScalar(row, "predicted_next_position_error_m");
    requiredFiniteScalar(row, "predicted_next_velocity_error_mps");
    requiredFiniteScalar(row, "trust");
    hard = requiredRow(row, "hard_constraints", config.hard_constraints_per_step);
    risk = requiredRow(row, "risk_constraints_normalized", config.risk_constraints_per_step);
    if row.stage_energy_j < 0.0
        error("gpenmpcBookkeepEnmpcRollout:StageEnergy", ...
            "Stage energy must be nonnegative.");
    end

    predictedEnergyJ = predictedEnergyJ + row.stage_energy_j;
    objectiveTerms.energy = objectiveTerms.energy + ...
        config.weights.energy * row.stage_energy_j / config.energy_scale_j;
    objectiveTerms.position = objectiveTerms.position + ...
        config.weights.position * (row.position_error_norm_m / config.position_scale_m)^2;
    objectiveTerms.velocity = objectiveTerms.velocity + ...
        config.weights.velocity * (row.velocity_error_norm_mps / config.velocity_scale_mps)^2;
    variationValue = row.force_variation_norm_n;
    variationScale = config.force_variation_scale_n;
    if executionConsistentCostEnabled(config)
        variationValue = row.execution_rotor_variation_n;
        variationScale = config.execution_rotor_variation_scale_n;
    end
    objectiveTerms.force_variation = objectiveTerms.force_variation + ...
        config.weights.force_variation * ...
        (variationValue / variationScale)^2;
    objectiveTerms.jerk = objectiveTerms.jerk + ...
        config.weights.jerk * (row.reference_jerk_norm_mps3 / config.jerk_scale_mps3)^2;
    objectiveTerms.phase_acceleration = objectiveTerms.phase_acceleration + ...
        config.weights.phase_acceleration * ...
        (row.phase_acceleration_s_inv / config.phase_acceleration_scale_s_inv)^2;
    objectiveTerms.phase_acceleration_variation = ...
        objectiveTerms.phase_acceleration_variation + ...
        config.weights.phase_acceleration_variation * ...
        (row.phase_jerk_s_inv2 * config.prediction_step_s ...
        / config.phase_acceleration_scale_s_inv)^2;
    objectiveTerms.progress = objectiveTerms.progress + ...
        config.weights.progress * (row.progress_deficit_s / config.progress_scale_s)^2;

    hardConstraints(hardCursor:hardCursor + config.hard_constraints_per_step - 1) = hard;
    riskConstraints(riskCursor:riskCursor + config.risk_constraints_per_step - 1) = risk;
    hardCursor = hardCursor + config.hard_constraints_per_step;
    riskCursor = riskCursor + config.risk_constraints_per_step;
    trackingZoneViolation = trackingZoneViolation + ...
        (max(0.0, row.predicted_next_position_error_m ...
        - config.tracking_position_zone_m) / config.tracking_position_zone_m)^2;
    trackingZoneViolation = trackingZoneViolation + ...
        (max(0.0, row.predicted_next_velocity_error_mps ...
        - config.tracking_velocity_zone_mps) / config.tracking_velocity_zone_mps)^2;
    trusts(index) = row.trust;
end

objectiveTerms.terminal_energy_to_go = config.terminal_energy_to_go_weight ...
    * terminalEnergyToGoJ / config.energy_scale_j;
objectiveVector = [objectiveTerms.energy, objectiveTerms.position, ...
    objectiveTerms.velocity, objectiveTerms.force_variation, ...
    objectiveTerms.jerk, objectiveTerms.phase_acceleration, ...
    objectiveTerms.phase_acceleration_variation, objectiveTerms.progress, ...
    objectiveTerms.terminal_energy_to_go];
hardConstraints(end) = 1.0;
if hardInvalid
    hardConstraints(end) = -1.0;
end
riskViolation = sum(max(0.0, -riskConstraints).^2);

result = struct;
result.schema = "GPENMPC_MATLAB_NATIVE_ENMPC_ROLLOUT_BOOKKEEPING_V1";
result.objective = sum(objectiveVector);
result.objective_terms = objectiveTerms;
result.objective_vector = objectiveVector;
result.constraints = hardConstraints;
result.hard_constraints = hardConstraints;
result.risk_constraints_normalized = riskConstraints;
result.risk_violation = riskViolation;
result.tracking_zone_violation = trackingZoneViolation;
result.predicted_energy_j = predictedEnergyJ;
result.terminal_energy_to_go_j = terminalEnergyToGoJ;
result.mean_trust = mean(trusts);
result.hard_invalid = hardInvalid;
result.current_hard_invalid = currentHardInvalid;
result.hard_constraint_count = numel(hardConstraints);
result.risk_constraint_count = numel(riskConstraints);
if executionConsistentCostEnabled(config)
    result.control_activity_cost_signal = "PREDICTED_SIX_ROTOR_TOTAL_VARIATION";
else
    result.control_activity_cost_signal = "DESIRED_FORCE_EUCLIDEAN_VARIATION";
end
end


function enabled = executionConsistentCostEnabled(config)
enabled = isfield(config, "execution_consistent_cost_enabled") ...
    && logical(config.execution_consistent_cost_enabled);
end


function requiredFiniteScalar(row, fieldName)
if ~isfield(row, fieldName) || ~isscalar(row.(fieldName)) ...
        || ~isfinite(double(row.(fieldName)))
    error("gpenmpcBookkeepEnmpcRollout:StageSchema", ...
        "Stage row is missing finite scalar %s.", fieldName);
end
end


function value = requiredRow(row, fieldName, width)
if ~isfield(row, fieldName)
    error("gpenmpcBookkeepEnmpcRollout:StageSchema", ...
        "Stage row is missing %s.", fieldName);
end
value = double(row.(fieldName)(:).');
if numel(value) ~= width || any(~isfinite(value))
    error("gpenmpcBookkeepEnmpcRollout:StageSchema", ...
        "Stage field %s must contain %d finite margins.", fieldName, width);
end
end
