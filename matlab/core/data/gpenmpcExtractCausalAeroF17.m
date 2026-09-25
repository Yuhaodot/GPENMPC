function extracted = gpenmpcExtractCausalAeroF17(trace)
%GPENMPCEXTRACTCAUSALAEROF17 Build the causal F17 GP feature and label data.
%
% Label k is formed only after velocity observation k+1 arrives:
%   r(k) = (v_hat(k+1)-v_hat(k))/dt(k) - a_nominal_known(k).
% The construction uses controller-observable samples available by k+1 and
% excludes latent plant states, future windows and task outcomes.

required = ["global_time_s", "velocity_mps", "nominal_acceleration_mps2", ...
    "leg_index", "plan_version", "from_roster_index", "to_roster_index", ...
    "reference_velocity_mps", "estimated_air_velocity_i_mps", ...
    "reference_acceleration_mps2", "payload_kg", "tilt_rad", ...
    "rotor_command_n", "phase_code"];
for name = required
    if ~isfield(trace, name)
        error("gpenmpcExtractCausalAeroF17:Field", "Missing raw trace field %s", name);
    end
end

gravity = 9.80665;
maximumPayloadKg = 4.54;
maximumDtS = 0.0100001;
samplePeriodS = 0.01;
historyTimeConstantS = 0.50;

timeS = double(trace.global_time_s(:));
count = numel(timeS);
velocity = requireShape(trace.velocity_mps, count, 3, "velocity_mps");
nominalAcceleration = requireShape(trace.nominal_acceleration_mps2, count, 3, ...
    "nominal_acceleration_mps2");
referenceVelocity = requireShape(trace.reference_velocity_mps, count, 3, ...
    "reference_velocity_mps");
referenceAcceleration = requireShape(trace.reference_acceleration_mps2, count, 3, ...
    "reference_acceleration_mps2");
estimatedAir = requireShape(trace.estimated_air_velocity_i_mps, count, 3, ...
    "estimated_air_velocity_i_mps");
rotorCommand = requireShape(trace.rotor_command_n, count, 6, "rotor_command_n");
payloadKg = requireVector(trace.payload_kg, count, "payload_kg");
tilt = requireVector(trace.tilt_rad, count, "tilt_rad");
phaseCode = requireVector(trace.phase_code, count, "phase_code");

dtS = nan(count, 1);
observedAcceleration = nan(count, 3);
residualI = nan(count, 3);
if count > 1
    dtS(1:end-1) = diff(timeS);
    observedAcceleration(1:end-1, :) = diff(velocity, 1, 1) ./ dtS(1:end-1);
    residualI(1:end-1, :) = observedAcceleration(1:end-1, :) ...
        - nominalAcceleration(1:end-1, :);
end

sameContext = true(max(count - 1, 0), 1);
contextNames = ["leg_index", "plan_version", "from_roster_index", "to_roster_index"];
for name = contextNames
    values = requireVector(trace.(name), count, name);
    sameContext = sameContext & values(1:end-1) == values(2:end);
end
causalValid = false(count, 1);
if count > 1
    finiteRows = isfinite(dtS(1:end-1)) ...
        & all(isfinite(velocity(1:end-1, :)), 2) ...
        & all(isfinite(velocity(2:end, :)), 2) ...
        & all(isfinite(nominalAcceleration(1:end-1, :)), 2) ...
        & all(isfinite(residualI(1:end-1, :)), 2);
    causalValid(1:end-1) = finiteRows & sameContext ...
        & dtS(1:end-1) > 1.0e-9 & dtS(1:end-1) <= maximumDtS;
end

horizontal = referenceVelocity(:, 1:2);
horizontalSpeed = vecnorm(horizontal, 2, 2);
tangentValid = horizontalSpeed >= 0.75;
tangent = zeros(count, 2);
lastTangent = [1.0, 0.0];
for index = 1:count
    if tangentValid(index)
        lastTangent = horizontal(index, :) ./ horizontalSpeed(index);
    end
    tangent(index, :) = lastTangent;
end

estimatedAirF = toFrenet(tangent, estimatedAir);
referenceAccelerationF = toFrenet(tangent, referenceAcceleration);
labelsF = toFrenet(tangent, residualI);

previousCommand = ones(count, 1);
if count > 1
    for index = 2:count
        previousCommand(index) = gpenmpcNormalizedTotalRotorCommand( ...
            rotorCommand(index - 1, :), payloadKg(index - 1));
    end
end
previousCommand = min(max(previousCommand, 0.0), 2.0);

history = zeros(count, 3);
state = zeros(1, 3);
for index = 1:count
    history(index, :) = state;
    nextIndex = min(index + 1, count);
    step = samplePeriodS;
    if causalValid(index)
        step = dtS(index);
    end
    frame = [tangent(index, 1), -tangent(index, 2), 0; ...
        tangent(index, 2), tangent(index, 1), 0; 0, 0, 1];
    [nextState, ~, ~] = gpenmpcAdvanceCausalResidualHistory( ...
        state.', velocity(index, :).', velocity(nextIndex, :).', ...
        nominalAcceleration(index, :).', frame, step, causalValid(index), ...
        historyTimeConstantS);
    state = nextState.';
end

numerator = abs(referenceVelocity(:, 1) .* referenceAcceleration(:, 2) ...
    - referenceVelocity(:, 2) .* referenceAcceleration(:, 1));
curvature = zeros(count, 1);
curvature(tangentValid) = numerator(tangentValid) ...
    ./ max(horizontalSpeed(tangentValid).^3, 1.0e-12);

payloadFraction = payloadKg ./ maximumPayloadKg;
airSpeed = vecnorm(estimatedAirF, 2, 2);
tiltFactor = 1.0 + 0.35 .* sin(tilt).^2;
dragT = payloadFraction .* tiltFactor .* estimatedAirF(:, 1) ...
    .* abs(estimatedAirF(:, 1));
dragN = payloadFraction .* tiltFactor .* estimatedAirF(:, 2) ...
    .* abs(estimatedAirF(:, 2));
turnCrossflow = curvature .* estimatedAirF(:, 1) .* abs(estimatedAirF(:, 1)) ...
    + estimatedAirF(:, 2) .* abs(estimatedAirF(:, 1));
verticalDemand = max(0.0, 1.0 + referenceAccelerationF(:, 3) ./ gravity);
thrustPayloadDemand = previousCommand .* (1.0 + payloadFraction) .* verticalDemand;

features = [payloadFraction, estimatedAirF, airSpeed, dragT, dragN, ...
    turnCrossflow, referenceAccelerationF, tilt, previousCommand, ...
    thrustPayloadDemand, history];
featureNames = { ...
    'payload_fraction', 'v_air_est_t_mps', 'v_air_est_n_mps', ...
    'v_air_est_z_mps', 'v_air_est_norm_mps', ...
    'payload_tilt_drag_basis_t_mps2', 'payload_tilt_drag_basis_n_mps2', ...
    'turn_crossflow_basis_mps2', 'a_ref_t_mps2', 'a_ref_n_mps2', ...
    'a_ref_z_mps2', 'tilt_rad', ...
    'previous_normalized_total_rotor_command', ...
    'thrust_payload_vertical_demand_basis', ...
    'previous_residual_ewma_t_mps2', ...
    'previous_residual_ewma_n_mps2', ...
    'previous_residual_ewma_z_mps2'};
finiteRows = all(isfinite(features), 2) & all(isfinite(labelsF), 2);
transit = phaseCode == 1;
payloadValid = payloadKg >= -1.0e-12 & payloadKg <= maximumPayloadKg + 1.0e-12;
valid = causalValid & tangentValid & transit & payloadValid & finiteRows;

extracted = struct;
extracted.schema = "GPENMPC_MATLAB_NATIVE_CAUSAL_AERO_F17_V1";
extracted.features = features;
extracted.labels_f = labelsF;
extracted.valid_mask = valid;
extracted.feature_names = featureNames;
extracted.tangent_i_xy = tangent;
extracted.dt_s = dtS;
extracted.causal_valid_mask = causalValid;
extracted.audit = struct( ...
    "label_formula", "(v_hat[k+1]-v_hat[k])/dt[k]-a_nominal_known[k]", ...
    "feature_time", "k", "label_available_time", "k+1", ...
    "maximum_dt_s", maximumDtS, ...
    "row_count", count, ...
    "causal_valid_count", sum(causalValid), ...
    "fit_valid_count", sum(valid), ...
    "invalid_context_or_time_count", sum(~causalValid), ...
    "low_reference_speed_count", sum(~tangentValid), ...
    "non_transit_count", sum(~transit), ...
    "payload_outside_domain_count", sum(~payloadValid));
end


function matrix = requireShape(value, rows, columns, name)
matrix = double(value);
if ~isequal(size(matrix), [rows, columns])
    error("gpenmpcExtractCausalAeroF17:Shape", ...
        "%s must be %d by %d, received %s", name, rows, columns, mat2str(size(matrix)));
end
end


function vector = requireVector(value, rows, name)
vector = double(value(:));
if numel(vector) ~= rows
    error("gpenmpcExtractCausalAeroF17:Shape", ...
        "%s must contain %d rows, received %d", name, rows, numel(vector));
end
end


function projected = toFrenet(tangent, inertial)
projected = [ ...
    inertial(:, 1) .* tangent(:, 1) + inertial(:, 2) .* tangent(:, 2), ...
   -inertial(:, 1) .* tangent(:, 2) + inertial(:, 2) .* tangent(:, 1), ...
    inertial(:, 3)];
end
