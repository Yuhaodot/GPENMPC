function command = gpenmpcDesiredSe3Command( ...
        plantState, reference, payloadKg, windEstimateXyMps, ...
        augmentationIMps2, calibration, profile)
%GPENMPCDESIREDSE3COMMAND Build the common desired force and raw attitude.
%
% This shared desired-command construction is used by the base controller
% and the optional attitude-continuity layer while preserving the
% configured force and attitude authority.

gravity = 9.80665;
x = double(plantState(:));
allocation = gpenmpcM600Allocation(calibration);
controller = calibration.controller;
kp = double(controller.position_gain_s2(:));
kd = double(controller.velocity_gain_s(:));

p = x(1:3);
v = x(4:6);
nominalMassKg = double(profile.mass_properties.base_mass_kg) + double(payloadKg);
referenceAir = double(reference.velocity_mps(:)) ...
    - [double(windEstimateXyMps(:)); 0];
dragFeedforward = allocation.nominal_drag_n_per_mps2 ...
    .* referenceAir .* abs(referenceAir);
desiredForceRaw = nominalMassKg .* (double(reference.acceleration_mps2(:)) ...
    + kp .* (double(reference.position_m(:)) - p) ...
    + kd .* (double(reference.velocity_mps(:)) - v) ...
    + [0; 0; gravity] + double(augmentationIMps2(:))) ...
    + dragFeedforward;
[desiredForce, projectionMismatch] = gpenmpcProjectForce( ...
    desiredForceRaw, allocation.total_thrust_upper_n, ...
    allocation.maximum_tilt_rad);

b3 = desiredForce ./ max(norm(desiredForce), 1e-15);
b1Command = [1; 0; 0];
b2 = cross(b3, b1Command);
if norm(b2) < 1e-9
    b1Command = [0; 1; 0];
    b2 = cross(b3, b1Command);
end
b2 = b2 ./ max(norm(b2), 1e-15);
b1 = cross(b2, b3);

command = struct;
command.desired_force_raw_n = desiredForceRaw;
command.desired_force_projected_n = desiredForce;
command.force_projection_norm_mismatch_n = projectionMismatch;
command.desired_rotation = [b1, b2, b3];
end
