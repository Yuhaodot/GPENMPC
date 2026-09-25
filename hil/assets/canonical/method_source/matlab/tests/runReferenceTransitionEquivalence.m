function result = runReferenceTransitionEquivalence(projectRoot, stagingRoot, outputPath)
%RUNREFERENCETRANSITIONEQUIVALENCE Verify trajectory-jet reuse exactly.

arguments
    projectRoot (1,1) string
    stagingRoot (1,1) string
    outputPath (1,1) string = ""
end
addpath(genpath(fullfile(projectRoot, "matlab")));
addpath(genpath(fullfile(stagingRoot, "matlab")), "-begin");
fixture = jsondecode(fileread(fullfile(projectRoot, "tests", "fixtures", ...
    "ordinary_b2_rollout_python_reference.json")));
trajectory = struct( ...
    "total_duration_s", double(fixture.trajectory.total_duration_s), ...
    "coefficients_ascending", ...
        double(fixture.trajectory.coefficients_ascending));
trajectory = gpenmpcPrepareTrajectoryDerivatives(trajectory, 3);
rng(20260902, "twister");
caseCount = 240;
maximumDifference = 0.0;
for index = 1:caseCount
    progress = trajectory.total_duration_s .* rand;
    rate = 0.25 + 1.25 .* rand;
    previousPhaseAcceleration = -0.2 + 0.4 .* rand;
    targetPhaseAcceleration = -0.2 + 0.4 .* rand;
    previousOuter = -0.1 + 0.2 .* rand(3, 1);
    targetOuter = -0.1 + 0.2 .* rand(3, 1);
    dt = 0.005 + 0.295 .* rand;
    jerkLimit = 0.5 + 4.0 .* rand;
    legacy = legacyTransition(trajectory, progress, rate, ...
        previousPhaseAcceleration, targetPhaseAcceleration, previousOuter, ...
        targetOuter, dt, jerkLimit);
    current = gpenmpcJerkBoundedReferenceTransition(trajectory, progress, rate, ...
        previousPhaseAcceleration, targetPhaseAcceleration, previousOuter, ...
        targetOuter, dt, jerkLimit);
    fields = ["phase_acceleration_s_inv", "phase_jerk_s_inv2", ...
        "outer_correction_i_mps2", "outer_correction_jerk_i_mps3", ...
        "fraction", "frame_i_from_f"];
    for fieldName = fields
        maximumDifference = max(maximumDifference, max(abs( ...
            double(legacy.(fieldName)) - double(current.(fieldName))), [], "all"));
    end
    referenceFields = ["position_m", "velocity_mps", ...
        "acceleration_mps2", "jerk_mps3"];
    for fieldName = referenceFields
        maximumDifference = max(maximumDifference, max(abs( ...
            double(legacy.reference.(fieldName)) ...
            - double(current.reference.(fieldName))), [], "all"));
    end
end
assert(maximumDifference <= 1.0e-12);

result = struct;
result.schema = "GPENMPC_REFERENCE_TRANSITION_EQUIVALENCE_V1";
result.status = "PASS";
result.case_count = caseCount;
result.maximum_absolute_difference = maximumDifference;
result.trajectory_derivative_calls_per_transition_prior = 16;
result.trajectory_derivative_calls_per_transition_current = 4;
if strlength(outputPath) > 0
    parent = fileparts(outputPath);
    if ~isfolder(parent)
        mkdir(parent);
    end
    writelines(jsonencode(result, PrettyPrint=true), outputPath, ...
        Encoding="UTF-8");
end
fprintf("%s\n", jsonencode(result));
end


function transition = legacyTransition(trajectory, progressS, progressRate, ...
        previousPhaseAcceleration, targetPhaseAcceleration, ...
        previousOuterCorrectionI, targetOuterCorrectionF, dtS, jerkLimitMps3)
base = gpenmpcPhaseReference(trajectory, progressS, progressRate, 0.0, 0.0);
[frameIFromF, ~, ~] = gpenmpcFrenetFrame(base);
targetOuterI = frameIFromF * targetOuterCorrectionF;
phaseDelta = targetPhaseAcceleration - previousPhaseAcceleration;
outerDelta = targetOuterI - previousOuterCorrectionI;
if ~isfinite(dtS) || dtS <= 0.0
    fraction = 0.0;
else
    referenceZero = evaluateAt(0.0);
    referenceOne = evaluateAt(1.0);
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

    function [reference, phaseAcceleration, phaseJerk, ...
            outerCorrectionI, outerJerkI] = evaluateAt(alpha)
        phaseAcceleration = previousPhaseAcceleration + alpha .* phaseDelta;
        outerCorrectionI = previousOuterCorrectionI + alpha .* outerDelta;
        if isfinite(dtS) && dtS > 0.0
            phaseJerk = (phaseAcceleration - previousPhaseAcceleration) ./ dtS;
            outerJerkI = (outerCorrectionI - previousOuterCorrectionI) ./ dtS;
        else
            phaseJerk = 0.0;
            outerJerkI = zeros(3, 1);
        end
        reference = gpenmpcPhaseReference(trajectory, progressS, progressRate, ...
            phaseAcceleration, phaseJerk);
        reference.acceleration_mps2 = reference.acceleration_mps2 + outerCorrectionI;
        reference.jerk_mps3 = reference.jerk_mps3 + outerJerkI;
    end
end
