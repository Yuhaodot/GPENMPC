function transition = gpenmpcJerkBoundedReferenceTransition(trajectory, progressS, progressRate, previousPhaseAcceleration, targetPhaseAcceleration, previousOuterCorrectionI, targetOuterCorrectionF, dtS, jerkLimitMps3)
%GPENMPCJERKBOUNDEDREFERENCETRANSITION Apply both outer targets causally.
%
% Phase acceleration and inertial acceleration correction advance with the
% same scalar fraction. The exact chain-rule reference jerk plus correction
% transition jerk must remain within the configured jerk bound.

arguments
    trajectory (1,1) struct
    progressS (1,1) double
    progressRate (1,1) double
    previousPhaseAcceleration (1,1) double
    targetPhaseAcceleration (1,1) double
    previousOuterCorrectionI (3,1) double
    targetOuterCorrectionF (3,1) double
    dtS (1,1) double
    jerkLimitMps3 (1,1) double
end
if isfield(trajectory, "prepared_derivative_coefficients_ascending")
    trajectoryJet = gpenmpcEvaluatePreparedTrajectoryJet(trajectory, progressS);
else
    trajectoryJet = cell(4, 1);
    for derivativeOrder = 0:3
        trajectoryJet{derivativeOrder + 1} = ...
            gpenmpcEvaluateTrajectoryDerivative( ...
                trajectory, progressS, derivativeOrder);
    end
end
base = referenceFromJet(progressRate, 0.0, 0.0);
[frameIFromF, ~, ~] = gpenmpcFrenetFrame(base);
targetOuterI = frameIFromF * targetOuterCorrectionF;
phaseDelta = targetPhaseAcceleration - previousPhaseAcceleration;
outerDelta = targetOuterI - previousOuterCorrectionI;

if ~isfinite(dtS) || dtS <= 0.0
    fraction = 0.0;
else
    [referenceZero, ~, ~, ~] = evaluateAt(0.0);
    [referenceOne, ~, ~, ~] = evaluateAt(1.0);
    jerkZero = referenceZero.jerk_mps3;
    jerkDelta = referenceOne.jerk_mps3 - jerkZero;
    if norm(referenceOne.jerk_mps3, 2) <= jerkLimitMps3 + 1.0e-12
        fraction = 1.0;
    elseif norm(jerkZero, 2) > jerkLimitMps3 + 1.0e-12
        fraction = 0.0;
    else
        quadratic = sum(jerkDelta(:) .* jerkDelta(:));
        linear = 2.0 .* sum(jerkZero(:) .* jerkDelta(:));
        constant = sum(jerkZero(:) .* jerkZero(:)) - jerkLimitMps3.^2;
        if quadratic <= 1.0e-24
            if all(linear(:) <= 0.0)
                fraction = 0.0;
            else
                fraction = -constant ./ linear;
            end
        else
            discriminant = max(0.0, linear.^2 - 4.0 .* quadratic .* constant);
            fraction = (-linear + sqrt(discriminant)) ./ (2.0 .* quadratic);
        end
        fraction = min(max(fraction, 0.0), 1.0);
    end
end
[reference, phaseAcceleration, phaseJerk, outerCorrectionI, outerJerkI] = ...
    evaluateAt(fraction);
transition = struct;
transition.reference = reference;
transition.phase_acceleration_s_inv = phaseAcceleration;
transition.phase_jerk_s_inv2 = phaseJerk;
transition.outer_correction_i_mps2 = outerCorrectionI;
transition.outer_correction_jerk_i_mps3 = outerJerkI;
transition.fraction = fraction;
transition.frame_i_from_f = frameIFromF;
[transition.reference_frame_i_from_f, transition.reference_curvature, ...
    transition.reference_signed_yaw_rate] = gpenmpcFrenetFrame(reference);

    function [reference, phaseAcceleration, phaseJerk, outerCorrectionI, outerJerkI] = evaluateAt(alpha)
        phaseAcceleration = previousPhaseAcceleration + alpha .* phaseDelta;
        outerCorrectionI = previousOuterCorrectionI + alpha .* outerDelta;
        if isfinite(dtS) && dtS > 0.0
            phaseJerk = (phaseAcceleration - previousPhaseAcceleration) ./ dtS;
            outerJerkI = (outerCorrectionI - previousOuterCorrectionI) ./ dtS;
        else
            phaseJerk = 0.0;
            outerJerkI = zeros(3, 1);
        end
        reference = referenceFromJet(progressRate, ...
            phaseAcceleration, phaseJerk);
        reference.acceleration_mps2 = reference.acceleration_mps2 + outerCorrectionI;
        reference.jerk_mps3 = reference.jerk_mps3 + outerJerkI;
    end

    function reference = referenceFromJet(rate, acceleration, phaseJerk)
        reference = struct;
        reference.position_m = trajectoryJet{1};
        reference.velocity_mps = trajectoryJet{2} .* rate;
        reference.acceleration_mps2 = trajectoryJet{3} .* rate.^2 ...
            + trajectoryJet{2} .* acceleration;
        reference.jerk_mps3 = trajectoryJet{4} .* rate.^3 ...
            + 3.0 .* trajectoryJet{3} .* rate .* acceleration ...
            + trajectoryJet{2} .* phaseJerk;
    end
end
