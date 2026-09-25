function allocation = gpenmpcM600Allocation(calibration)
%GPENMPCM600ALLOCATION Build the common M600 six-rotor allocation identity.
%
% The returned geometry and limits are applied identically to B1 and ordinary
% B2, providing the common six-rotor wrench allocation map.

rotor = calibration.rotor_allocation;
angles = deg2rad(double(rotor.angles_deg(:)));
spin = double(rotor.spin_sign(:));
arm = double(rotor.arm_radius_m);
yawArm = double(rotor.yaw_moment_arm_nominal_m);

matrix = [ones(1, 6); ...
    arm .* sin(angles).'; ...
    -arm .* cos(angles).'; ...
    yawArm .* spin.'];

gram = matrix * matrix.';
if rank(gram) ~= 4
    error("gpenmpcM600Allocation:Rank", ...
        "The six-rotor allocation matrix must have full wrench rank.");
end

allocation = struct;
allocation.matrix = matrix;
allocation.pseudoinverse = matrix.' / gram;
allocation.per_rotor_upper_n = double(rotor.per_rotor_thrust_upper_n);
allocation.total_thrust_upper_n = 6.0 .* allocation.per_rotor_upper_n;
allocation.actuator_time_constant_s = ...
    double(rotor.actuator_time_constant_nominal_s);
allocation.inertia_kg_m2 = ...
    diag(double(calibration.mass_inertia.inertia_nominal_kg_m2(:)));
allocation.nominal_drag_n_per_mps2 = ...
    double(calibration.aerodynamics.frame_drag_nominal_n_per_mps2(:));
allocation.maximum_tilt_rad = ...
    deg2rad(double(calibration.development_limits.tilt_deg));
end
