function [derivative, diagnostic] = gpenmpcM600SixDofPlantDerivative( ...
        plantState, rotorCommandN, reference, payloadKg, actualWindXyMps, ...
        globalTimeS, mission, calibration, profile)
%GPENMPCM600SIXDOFPLANTDERIVATIVE Common 19-state M600 software plant.
%
% State order is [position(3); velocity(3); quaternion_wxyz(4);
% body_rate(3); per_rotor_thrust(6)].  The plant includes actuator lag,
% six-rotor effectiveness, drag, plant mismatch and structured residuals.

gravity = 9.80665;
x = double(plantState(:));
command = double(rotorCommandN(:));
allocation = gpenmpcM600Allocation(calibration);

position = x(1:3); %#ok<NASGU>
velocity = x(4:6);
quaternion = x(7:10) ./ max(norm(x(7:10)), 1e-15);
bodyRate = x(11:13);
rotorThrust = x(14:19);
rotation = gpenmpcQuaternionRotation(quaternion);

mismatch = mission.plant_mismatch;
nominalMassKg = double(profile.mass_properties.base_mass_kg) + double(payloadKg);
trueMassKg = nominalMassKg + double(mismatch.mass_bias_kg);
actualAir = velocity - [double(actualWindXyMps(:)); 0];
trueDragBasis = actualAir .* abs(actualAir);
dragScale = double(mismatch.drag_scale_xyz(:));
crossDrag = double(mismatch.cross_drag_matrix_n_per_mps2);
trueDrag = allocation.nominal_drag_n_per_mps2 .* dragScale ...
    .* trueDragBasis + crossDrag * trueDragBasis;

effectiveness = double(mismatch.thrust_effectiveness_by_rotor(:));
trueWrench = allocation.matrix * (rotorThrust .* effectiveness);
baseAcceleration = (rotation * [0; 0; trueWrench(1)] - trueDrag) ...
    ./ trueMassKg - [0; 0; gravity];
structured = gpenmpcM600StructuredResidual(x, reference, payloadKg, ...
    actualAir, trueMassKg, mission, profile);
frequency = double(mismatch.external_acceleration_frequencies_hz(:));
phase = double(mismatch.external_acceleration_phases_rad(:));
fastAcceleration = double(mismatch.external_acceleration_amplitude_mps2) ...
    .* sin(2 .* pi .* frequency .* double(globalTimeS) + phase);
acceleration = baseAcceleration ...
    + double(mismatch.acceleration_bias_inertial_mps2(:)) ...
    + fastAcceleration + structured.drag + structured.turn + structured.thrust;

angularAcceleration = allocation.inertia_kg_m2 \ ...
    (trueWrench(2:4) ...
    - cross(bodyRate, allocation.inertia_kg_m2 * bodyRate));
quaternionDerivative = 0.5 ...
    .* gpenmpcQuaternionDerivativeMatrix(bodyRate) * quaternion;
rotorDerivative = (command - rotorThrust) ...
    ./ allocation.actuator_time_constant_s;
derivative = [velocity; acceleration; quaternionDerivative; ...
    angularAcceleration; rotorDerivative];

diagnostic = struct;
diagnostic.actual_acceleration_mps2 = acceleration;
diagnostic.actual_airspeed_mps = norm(actualAir);
diagnostic.actual_air_velocity_mps = actualAir;
diagnostic.true_drag_n = trueDrag;
diagnostic.true_wrench = trueWrench;
diagnostic.structured_acceleration_mps2 = ...
    structured.drag + structured.turn + structured.thrust;
diagnostic.fast_acceleration_mps2 = fastAcceleration;
diagnostic.true_mass_kg = trueMassKg;
end
