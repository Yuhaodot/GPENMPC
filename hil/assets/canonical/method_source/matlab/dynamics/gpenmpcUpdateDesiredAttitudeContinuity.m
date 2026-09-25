function [command, nextState] = gpenmpcUpdateDesiredAttitudeContinuity( ...
        state, rawDesiredRotation, actualDtS, resetRequested, config)
%GPENMPCUPDATEDESIREDATTITUDECONTINUITY Causal geodesic SO(3) filtering.
%
% Only the current raw desired rotation, past filter state and actual elapsed
% time are used. A reset adopts the current raw rotation without injecting
% stale angular velocity or angular acceleration.

arguments
    state (1,1) struct
    rawDesiredRotation (3,3) double
    actualDtS (1,1) double
    resetRequested (1,1) logical
    config (1,1) struct
end

enabled = logical(getFieldOr(config, "enabled", false));
tau = double(getFieldOr(config, "time_constant_s", 0.0));
maximumOmega = double(getFieldOr(config, ...
    "maximum_angular_velocity_rad_s", Inf));
maximumOmegaDot = double(getFieldOr(config, ...
    "maximum_angular_acceleration_rad_s2", Inf));
maximumOmegaDoubleDot = double(getFieldOr(config, ...
    "maximum_angular_jerk_rad_s3", Inf));
maximumContinuousDt = double(getFieldOr(config, ...
    "maximum_continuous_dt_s", Inf));
if tau < 0.0 || maximumOmega <= 0.0 || maximumOmegaDot <= 0.0 ...
        || maximumOmegaDoubleDot <= 0.0 ...
        || maximumContinuousDt <= 0.0
    error("gpenmpcUpdateDesiredAttitudeContinuity:Config", ...
        "Limits must be positive and time constant nonnegative.");
end
if any(~isfinite(rawDesiredRotation), "all")
    error("gpenmpcUpdateDesiredAttitudeContinuity:Rotation", ...
        "Desired rotation must be finite.");
end

timeDiscontinuity = ~isfinite(actualDtS) || actualDtS <= 0.0 ...
    || actualDtS > maximumContinuousDt;
memoryInvalid = ~isfield(state, "initialized") || ~logical(state.initialized);
resetActive = resetRequested || timeDiscontinuity || memoryInvalid;

if ~enabled
    command = outputCommand(rawDesiredRotation, zeros(3,1), zeros(3,1), ...
        false, false, ...
        false, timeDiscontinuity, 1.0, 0.0, false);
    nextState = state;
    return
end
raw = projectToSo3(rawDesiredRotation);

if resetActive
    filtered = raw;
    desiredOmega = zeros(3,1);
    desiredOmegaDot = zeros(3,1);
    velocityValid = false;
    accelerationValid = false;
    geodesicGain = 1.0;
    rawErrorNorm = 0.0;
else
    previousRotation = projectToSo3(double(state.filtered_rotation));
    rawError = gpenmpcSo3LogVee(previousRotation.' * raw);
    rawErrorNorm = norm(rawError);
    if tau <= 0.0
        geodesicGain = 1.0;
    else
        geodesicGain = 1.0 - exp(-actualDtS ./ tau);
    end
    increment = clipNorm(geodesicGain .* rawError, ...
        maximumOmega .* actualDtS);
    filtered = projectToSo3(previousRotation * gpenmpcSo3Exp(increment));
    desiredOmega = gpenmpcSo3LogVee(previousRotation.' * filtered) ./ actualDtS;
    desiredOmega = clipNorm(desiredOmega, maximumOmega);
    velocityValid = true;

    previousOmega = filtered.' * previousRotation * double( ...
        state.desired_angular_velocity_body_rad_s(:));
    previousVelocityValid = isfield(state, "angular_velocity_valid") ...
        && logical(state.angular_velocity_valid);
    if previousVelocityValid
        rawOmegaDot = clipNorm((desiredOmega - previousOmega) ./ actualDtS, ...
            maximumOmegaDot);
        previousOmegaDot = filtered.' * previousRotation * double( ...
            state.desired_angular_acceleration_body_rad_s2(:));
        if tau <= 0.0
            accelerationGain = 1.0;
        else
            accelerationGain = 1.0 - exp(-actualDtS ./ tau);
        end
        lowPassOmegaDot = previousOmegaDot ...
            + accelerationGain .* (rawOmegaDot - previousOmegaDot);
        omegaDotDelta = clipNorm(lowPassOmegaDot - previousOmegaDot, ...
            maximumOmegaDoubleDot .* actualDtS);
        desiredOmegaDot = clipNorm(previousOmegaDot + omegaDotDelta, ...
            maximumOmegaDot);
        accelerationValid = true;
    else
        desiredOmegaDot = zeros(3,1);
        accelerationValid = false;
    end
end

if any(~isfinite(filtered), "all") || any(~isfinite(desiredOmega)) ...
        || any(~isfinite(desiredOmegaDot))
    error("gpenmpcUpdateDesiredAttitudeContinuity:Finite", ...
        "Filtered attitude command must remain finite.");
end

command = outputCommand(filtered, desiredOmega, desiredOmegaDot, ...
    velocityValid, accelerationValid, resetActive, timeDiscontinuity, ...
    geodesicGain, rawErrorNorm, true);
nextState = struct( ...
    "initialized", true, ...
    "angular_velocity_valid", velocityValid, ...
    "angular_acceleration_valid", accelerationValid, ...
    "filtered_rotation", filtered, ...
    "desired_angular_velocity_body_rad_s", desiredOmega, ...
    "desired_angular_acceleration_body_rad_s2", desiredOmegaDot, ...
    "update_count", double(getFieldOr(state, "update_count", 0)) + 1, ...
    "reset_count", double(getFieldOr(state, "reset_count", 0)) ...
        + double(resetActive));
end

function command = outputCommand(rotation, omega, omegaDot, omegaValid, ...
        omegaDotValid, resetActive, timeDiscontinuity, gain, errorNorm, enabled)
command = struct( ...
    "enabled", enabled, ...
    "desired_rotation", rotation, ...
    "desired_angular_velocity_body_rad_s", omega, ...
    "desired_angular_acceleration_body_rad_s2", omegaDot, ...
    "angular_velocity_valid", omegaValid, ...
    "angular_acceleration_valid", omegaDotValid, ...
    "reset_active", resetActive, ...
    "time_discontinuity", timeDiscontinuity, ...
    "geodesic_filter_gain", gain, ...
    "raw_geodesic_error_rad", errorNorm);
end

function rotation = projectToSo3(input)
if any(~isfinite(input), "all")
    error("gpenmpcUpdateDesiredAttitudeContinuity:Rotation", ...
        "Desired rotation must be finite.");
end
[left, ~, right] = svd(double(input));
correction = eye(3);
correction(3,3) = sign(det(left * right.'));
if correction(3,3) == 0.0
    correction(3,3) = 1.0;
end
rotation = left * correction * right.';
end

function value = clipNorm(value, upper)
value = double(value(:));
lengthValue = norm(value);
if isfinite(upper) && lengthValue > upper
    value = value .* (upper ./ max(lengthValue, 1.0e-15));
end
end

function value = getFieldOr(record, name, fallback)
if isfield(record, name)
    value = record.(name);
else
    value = fallback;
end
end
