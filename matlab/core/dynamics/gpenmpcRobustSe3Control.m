function [rotorCommandN, diagnostic] = gpenmpcRobustSe3Control( ...
        plantState, reference, payloadKg, windEstimateXyMps, ...
        augmentationIMps2, calibration, profile, attitudeCommand)
%GPENMPCROBUSTSE3CONTROL Common geometric SE(3) inner loop and allocation.
%
% The caller supplies the reference.  Consequently B1 and ordinary B2 use
% identical gains, projection, allocation and actuator authority.

continuityEnabled = nargin >= 8 && ~isempty(attitudeCommand) ...
    && isfield(attitudeCommand, "enabled") ...
    && logical(attitudeCommand.enabled);
x = double(plantState(:));
allocation = gpenmpcM600Allocation(calibration);
controller = calibration.controller;
kr = double(controller.attitude_moment_gain_nm_per_rad(:));
kw = double(controller.body_rate_moment_gain_nm_s_per_rad(:));

rotation = gpenmpcQuaternionRotation(x(7:10));
omega = x(11:13);
rawCommand = gpenmpcDesiredSe3Command(plantState, reference, payloadKg, ...
    windEstimateXyMps, augmentationIMps2, calibration, profile);
desiredForceRaw = rawCommand.desired_force_raw_n;
desiredForce = rawCommand.desired_force_projected_n;
projectionMismatch = rawCommand.force_projection_norm_mismatch_n;
if continuityEnabled
    desiredRotation = double(attitudeCommand.desired_rotation);
    desiredOmega = double( ...
        attitudeCommand.desired_angular_velocity_body_rad_s(:));
    desiredOmegaDot = double( ...
        attitudeCommand.desired_angular_acceleration_body_rad_s2(:));
else
    desiredRotation = rawCommand.desired_rotation;
    desiredOmega = zeros(3,1);
    desiredOmegaDot = zeros(3,1);
end
attitudeError = 0.5 .* gpenmpcVee( ...
    desiredRotation.' * rotation - rotation.' * desiredRotation);
gyroscopicMomentNm = cross(omega, allocation.inertia_kg_m2 * omega);
if continuityEnabled
    desiredToCurrent = rotation.' * desiredRotation;
    desiredOmegaInCurrent = desiredToCurrent * desiredOmega;
    bodyRateError = omega - desiredOmegaInCurrent;
    geometricFeedforwardNm = -allocation.inertia_kg_m2 * ( ...
        cross(omega, desiredOmegaInCurrent) ...
        - desiredToCurrent * desiredOmegaDot);
    momentNm = -kr .* attitudeError - kw .* bodyRateError ...
        + gyroscopicMomentNm + geometricFeedforwardNm;
else
    desiredOmegaInCurrent = zeros(3,1);
    bodyRateError = omega;
    geometricFeedforwardNm = zeros(3,1);
    % Use the baseline geometric-control operation order when attitude
    % continuity is disabled.
    momentNm = -kr .* attitudeError - kw .* omega + gyroscopicMomentNm;
end
desiredThrustN = max(dot(desiredForce, rotation(:, 3)), 0.0);
rawCommandN = allocation.pseudoinverse * [desiredThrustN; momentNm];
rotorCommandN = min(max(rawCommandN, 0.0), ...
    allocation.per_rotor_upper_n);

diagnostic = struct;
diagnostic.desired_force_raw_n = desiredForceRaw;
diagnostic.desired_force_projected_n = desiredForce;
diagnostic.desired_rotation = desiredRotation;
diagnostic.raw_desired_rotation = rawCommand.desired_rotation;
diagnostic.attitude_error = attitudeError;
diagnostic.desired_angular_velocity_body_rad_s = desiredOmega;
diagnostic.desired_angular_acceleration_body_rad_s2 = desiredOmegaDot;
diagnostic.desired_angular_velocity_in_current_body_rad_s = ...
    desiredOmegaInCurrent;
diagnostic.body_rate_error_rad_s = bodyRateError;
diagnostic.geometric_feedforward_moment_nm = geometricFeedforwardNm;
diagnostic.attitude_continuity_enabled = continuityEnabled;
diagnostic.attitude_continuity_reset_active = continuityEnabled ...
    && logical(getFieldOr(attitudeCommand, "reset_active", false));
diagnostic.attitude_continuity_time_discontinuity = continuityEnabled ...
    && logical(getFieldOr(attitudeCommand, "time_discontinuity", false));
diagnostic.desired_moment_nm = momentNm;
diagnostic.desired_thrust_n = desiredThrustN;
diagnostic.raw_rotor_command_n = rawCommandN;
diagnostic.rotor_command_n = rotorCommandN;
diagnostic.rotor_saturated = any(abs(rotorCommandN - rawCommandN) > 1e-10);
diagnostic.force_projection_norm_mismatch_n = projectionMismatch;
diagnostic.tilt_rad = acos(min(max(rotation(3, 3), -1.0), 1.0));
end

function value = getFieldOr(record, name, fallback)
if isfield(record, name)
    value = record.(name);
else
    value = fallback;
end
end
