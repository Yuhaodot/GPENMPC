function reference = gpenmpcPhaseReference(trajectory, progressS, progressRate, phaseAccelerationSInv, phaseJerkSInv2)
%GPENMPCPHASEREFERENCE Exact time-reparameterization chain rule.

arguments
    trajectory (1,1) struct
    progressS (1,1) double
    progressRate (1,1) double
    phaseAccelerationSInv (1,1) double
    phaseJerkSInv2 (1,1) double = 0.0
end
r0 = gpenmpcEvaluateTrajectoryDerivative(trajectory, progressS, 0);
r1 = gpenmpcEvaluateTrajectoryDerivative(trajectory, progressS, 1);
r2 = gpenmpcEvaluateTrajectoryDerivative(trajectory, progressS, 2);
r3 = gpenmpcEvaluateTrajectoryDerivative(trajectory, progressS, 3);
rate = double(progressRate);
acceleration = double(phaseAccelerationSInv);
phaseJerk = double(phaseJerkSInv2);
reference = struct;
reference.position_m = r0;
reference.velocity_mps = r1 .* rate;
reference.acceleration_mps2 = r2 .* rate.^2 + r1 .* acceleration;
reference.jerk_mps3 = r3 .* rate.^3 ...
    + 3.0 .* r2 .* rate .* acceleration + r1 .* phaseJerk;
end
