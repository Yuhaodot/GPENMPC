function allocation = generatedAllocation(calibration)
%#codegen
% Same arithmetic as immutable gpenmpcM600Allocation, SHA 8888EFB8...2D00F.
% Only change: construct every field before reading the structure (Coder).
rotor=calibration.rotor_allocation;
angles=deg2rad(double(rotor.angles_deg(:)));spin=double(rotor.spin_sign(:));
arm=double(rotor.arm_radius_m);yawArm=double(rotor.yaw_moment_arm_nominal_m);
matrix=[ones(1,6);arm.*sin(angles).';-arm.*cos(angles).';yawArm.*spin.'];
gram=matrix*matrix.';
if rank(gram)~=4
    error('gpenmpcM600Allocation:Rank','The six-rotor allocation matrix must have full wrench rank.');
end
upper=double(rotor.per_rotor_thrust_upper_n);
allocation=struct('matrix',matrix,'pseudoinverse',matrix.'/gram,...
    'per_rotor_upper_n',upper,'total_thrust_upper_n',6.0.*upper,...
    'actuator_time_constant_s',double(rotor.actuator_time_constant_nominal_s),...
    'inertia_kg_m2',diag(double(calibration.mass_inertia.inertia_nominal_kg_m2(:))),...
    'nominal_drag_n_per_mps2',double(calibration.aerodynamics.frame_drag_nominal_n_per_mps2(:)),...
    'maximum_tilt_rad',deg2rad(double(calibration.development_limits.tilt_deg)));
end
