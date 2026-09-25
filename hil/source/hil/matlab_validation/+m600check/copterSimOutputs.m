function y = copterSimOutputs(stateUp,accelerationUp,windXyMps,angularAccelerationUp,massKg)
%#codegen
%COPTERSIMOUTPUTS Pure current-plant observation to official 6DOF I/O fields.
% World velocities are NED; body velocities/rates/specific force are FRD.
% Specific force excludes gravitational acceleration, matching the current
% LiveHilPlantService snapshot; inertial body acceleration is separate.
assert(isequal(size(stateUp),[19,1]) &&all(isfinite(stateUp)));
assert(isequal(size(accelerationUp),[3,1]) &&all(isfinite(accelerationUp)));
assert(isequal(size(windXyMps),[2,1]) &&all(isfinite(windXyMps)));
assert(isequal(size(angularAccelerationUp),[3,1]) &&all(isfinite(angularAccelerationUp)));
assert(isfinite(massKg) &&massKg>0);
qUp=stateUp(7:10)/max(norm(stateUp(7:10)),1e-15);
qNed=qUp.*[1;-1;-1;1];
R=gpenmpcQuaternionRotation(qNed);
position=stateUp(1:3).*[1;1;-1];
velocity=stateUp(4:6).*[1;1;-1];
acceleration=accelerationUp.*[1;1;-1];
omega=stateUp(11:13).*[-1;-1;1];
euler=[atan2(R(3,2),R(3,3));asin(min(max(-R(3,1),-1),1));atan2(R(2,1),R(1,1))];
windNed=[windXyMps;0];
airVelocityBody=R.'*(velocity-windNed);
bodyVelocity=R.'*velocity;
specificForce=R.'*(acceleration-[0;0;9.80665]);
y=struct('position_ned_m',position,'velocity_ned_mps',velocity, ...
    'quaternion_wxyz_body_to_ned',qNed,'euler_rpy_rad',euler, ...
    'body_rate_frd_rad_s',omega,'specific_force_body_frd_mps2',specificForce, ...
    'air_velocity_body_frd_mps',airVelocityBody, ...
    'velocity_body_frd_mps',bodyVelocity,'acceleration_ned_mps2',acceleration, ...
    'acceleration_body_frd_mps2',R.'*acceleration, ...
    'rotation_ned_from_body',R,'rotation_body_from_ned',R.', ...
    'angular_acceleration_body_frd_rad_s2',angularAccelerationUp.*[-1;-1;1], ...
    'wind_body_frd_mps',R.'*windNed,'airspeed_mps',norm(airVelocityBody), ...
    'rotor_thrust_software_order_n',stateUp(14:19), ...
    'mass_kg',massKg,'gravity_mps2',9.80665,'sample_period_s',0.01);
end
