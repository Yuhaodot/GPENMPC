function [rotorCommandN, diagnostic] = gpenmpcCascadedPidControl( ...
        plantState, reference, payloadKg, windEstimateXyMps, ...
        calibration, profile, configuration, state, dt)
%GPENMPCCASCADEDPIDCONTROL Four-loop software PID with common authority.
%
% Position P produces a velocity command, velocity PID produces desired
% acceleration, attitude P produces a body-rate command, and body-rate PID
% produces moment.  Force projection, six-rotor allocation, plant and
% actuator limits are exactly the common GPENMPC implementation.

gravity = 9.80665;
x = double(plantState(:));
allocation = gpenmpcM600Allocation(calibration);
rotation = gpenmpcQuaternionRotation(x(7:10));
position = x(1:3);
velocity = x(4:6);
omega = x(11:13);

positionError = double(reference.position_m(:)) - position;
positionGain = double(configuration.position_loop. ...
    proportional_velocity_gain_s_inv(:));
maximumVelocityCorrection = double(configuration.position_loop. ...
    maximum_velocity_correction_mps(:));
velocityCorrection = clampVector(positionGain .* positionError, ...
    maximumVelocityCorrection);
velocityCommand = double(reference.velocity_mps(:)) + velocityCorrection;
velocityError = velocityCommand - velocity;
if state.previous_error_valid
    velocityErrorDerivative = (velocityError ...
        - state.previous_velocity_error_mps) ./ double(dt);
else
    velocityErrorDerivative = zeros(3,1);
end
velocityLoop = configuration.velocity_loop;
desiredAcceleration = double(reference.acceleration_mps2(:)) ...
    + double(velocityLoop.proportional_acceleration_gain_s_inv(:)) ...
        .* velocityError ...
    + double(velocityLoop.integral_acceleration_gain_s2(:)) ...
        .* state.velocity_integral_error_m ...
    + double(velocityLoop.derivative_acceleration_gain(:)) ...
        .* velocityErrorDerivative;

nominalMassKg = double(profile.mass_properties.base_mass_kg) ...
    + double(payloadKg);
referenceAir = double(reference.velocity_mps(:)) ...
    - [double(windEstimateXyMps(:)); 0];
dragFeedforward = allocation.nominal_drag_n_per_mps2 ...
    .* referenceAir .* abs(referenceAir);
desiredForceRaw = nominalMassKg .* (desiredAcceleration ...
    + [0;0;gravity]) + dragFeedforward;
[desiredForce, projectionMismatch] = gpenmpcProjectForce(desiredForceRaw, ...
    allocation.total_thrust_upper_n, allocation.maximum_tilt_rad);
desiredRotation = forceToRotation(desiredForce);
attitudeError = 0.5 .* gpenmpcVee(desiredRotation.' * rotation ...
    - rotation.' * desiredRotation);

attitudeLoop = configuration.attitude_loop;
desiredBodyRate = -double(attitudeLoop. ...
    proportional_body_rate_gain_s_inv(:)) .* attitudeError;
desiredBodyRate = clampVector(desiredBodyRate, double(attitudeLoop. ...
    maximum_body_rate_command_rad_s(:)));
bodyRateError = omega - desiredBodyRate;
if state.previous_error_valid
    bodyRateErrorDerivative = (bodyRateError ...
        - state.previous_body_rate_error_rad_s) ./ double(dt);
else
    bodyRateErrorDerivative = zeros(3,1);
end
rateLoop = configuration.body_rate_loop;
gyroscopicMomentNm = cross(omega, allocation.inertia_kg_m2 * omega);
momentNm = -double(rateLoop.proportional_moment_gain_nm_s_per_rad(:)) ...
        .* bodyRateError ...
    - double(rateLoop.integral_moment_gain_nm_per_rad(:)) ...
        .* state.body_rate_integral_error_rad ...
    - double(rateLoop.derivative_moment_gain_nm_s2_per_rad(:)) ...
        .* bodyRateErrorDerivative ...
    + gyroscopicMomentNm;
desiredThrustN = max(dot(desiredForce, rotation(:,3)), 0.0);
rawCommandN = allocation.pseudoinverse * [desiredThrustN; momentNm];
rotorCommandN = min(max(rawCommandN,0.0), ...
    allocation.per_rotor_upper_n);

diagnostic = struct;
diagnostic.position_error_m = positionError;
diagnostic.velocity_command_mps = velocityCommand;
diagnostic.velocity_error_mps = velocityError;
diagnostic.velocity_error_derivative_mps2 = velocityErrorDerivative;
diagnostic.desired_acceleration_mps2 = desiredAcceleration;
diagnostic.desired_force_raw_n = desiredForceRaw;
diagnostic.desired_force_projected_n = desiredForce;
diagnostic.desired_rotation = desiredRotation;
diagnostic.raw_desired_rotation = desiredRotation;
diagnostic.attitude_error = attitudeError;
diagnostic.desired_angular_velocity_body_rad_s = desiredBodyRate;
diagnostic.desired_angular_acceleration_body_rad_s2 = zeros(3,1);
diagnostic.desired_angular_velocity_in_current_body_rad_s = desiredBodyRate;
diagnostic.body_rate_error_rad_s = bodyRateError;
diagnostic.body_rate_error_derivative_rad_s2 = bodyRateErrorDerivative;
diagnostic.geometric_feedforward_moment_nm = zeros(3,1);
diagnostic.attitude_continuity_enabled = false;
diagnostic.attitude_continuity_reset_active = false;
diagnostic.attitude_continuity_time_discontinuity = false;
diagnostic.desired_moment_nm = momentNm;
diagnostic.desired_thrust_n = desiredThrustN;
diagnostic.raw_rotor_command_n = rawCommandN;
diagnostic.rotor_command_n = rotorCommandN;
diagnostic.rotor_saturated = any(abs(rotorCommandN-rawCommandN)>1e-10);
diagnostic.force_projection_norm_mismatch_n = projectionMismatch;
diagnostic.tilt_rad = acos(min(max(rotation(3,3),-1.0),1.0));
end

function rotation = forceToRotation(force)
b3 = double(force(:)) ./ max(norm(force),1e-15);
b1Command = [1;0;0];
b2 = cross(b3,b1Command);
if norm(b2)<1e-9
    b1Command=[0;1;0];
    b2=cross(b3,b1Command);
end
b2=b2./max(norm(b2),1e-15);
b1=cross(b2,b3);
rotation=[b1,b2,b3];
end

function value = clampVector(value, limit)
value = min(max(double(value(:)),-double(limit(:))),double(limit(:)));
end
