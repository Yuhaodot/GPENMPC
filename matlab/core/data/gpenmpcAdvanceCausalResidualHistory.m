function [nextHistoryF, residualF, labelValid] = ...
        gpenmpcAdvanceCausalResidualHistory(historyF, velocityAtK, ...
        velocityAtKp1, nominalAccelerationAtK, frameIFromFAtK, dt, ...
        labelValid, timeConstantS)
%GPENMPCADVANCECAUSALRESIDUALHISTORY Advance the strictly causal F17 history.
%
% The interval-k label becomes available only after velocity k+1 arrives:
%   r(k) = (v_hat(k+1)-v_hat(k))/dt(k) - a_nominal_known(k).
% The stored frame and nominal acceleration must therefore both belong to k.

arguments
    historyF (3,1) double
    velocityAtK (3,1) double
    velocityAtKp1 (3,1) double
    nominalAccelerationAtK (3,1) double
    frameIFromFAtK (3,3) double
    dt (1,1) double
    labelValid (1,1) logical
    timeConstantS (1,1) double {mustBePositive} = 0.50
end

labelValid = logical(labelValid) && isfinite(dt) && dt > 1.0e-9 ...
    && all(isfinite(velocityAtK)) && all(isfinite(velocityAtKp1)) ...
    && all(isfinite(nominalAccelerationAtK), "all") ...
    && all(isfinite(frameIFromFAtK), "all");
if labelValid
    observedAcceleration = (velocityAtKp1 - velocityAtK) ./ dt;
    residualF = frameIFromFAtK.' * ...
        (observedAcceleration - nominalAccelerationAtK);
else
    % Match the offline history convention at context boundaries: invalid
    % labels are excluded and the retained history decays toward zero.
    residualF = zeros(3, 1);
end
gain = 1.0 - exp(-max(dt, 0.0) ./ timeConstantS);
if ~isfinite(gain)
    gain = 0.0;
end
nextHistoryF = historyF + gain .* (residualF - historyF);
end
