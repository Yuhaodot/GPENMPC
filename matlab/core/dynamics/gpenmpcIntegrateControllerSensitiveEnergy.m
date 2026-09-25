function [energyJ, cumulativeJ] = gpenmpcIntegrateControllerSensitiveEnergy( ...
        powerW, timeS, segmentId)
%GPENMPCINTEGRATECONTROLLERSENSITIVEENERGY Segment-aware trapezoidal integral.

power = double(powerW(:));
time = double(timeS(:));
segment = double(segmentId(:));
if numel(power) ~= numel(time) || numel(power) ~= numel(segment)
    error("gpenmpcIntegrateControllerSensitiveEnergy:Size", ...
        "Power, time and segment arrays must have the same length.");
end
if isempty(power)
    energyJ = 0.0;
    cumulativeJ = zeros(0, 1);
    return
end
if any(~isfinite(power)) || any(~isfinite(time)) || any(diff(time) < -1e-12)
    error("gpenmpcIntegrateControllerSensitiveEnergy:Finite", ...
        "Power and time must be finite and time must be monotone.");
end

cumulativeJ = zeros(size(power));
for sample = 2:numel(power)
    cumulativeJ(sample) = cumulativeJ(sample - 1);
    if segment(sample) == segment(sample - 1)
        dt = time(sample) - time(sample - 1);
        cumulativeJ(sample) = cumulativeJ(sample) ...
            + 0.5 .* dt .* (power(sample) + power(sample - 1));
    end
end
energyJ = cumulativeJ(end);
end
