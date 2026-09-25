function responsibility = gpenmpcComputeGpResponsibility( ...
        responsibilityBlend, gpMeanScale, trust, agreementF, meanF, ...
        gpValid, minimumTrust, zeroTolerance)
%GPENMPCCOMPUTEGPRESPONSIBILITY One causal, axis-wise GP authority rule.
%
% This pure function is shared by the outer decision path, the eNMPC
% rollout and the runtime residual composition. It combines the supplied GP
% mean and trust with axis-wise agreement and a causal responsibility blend
% formed from completed innovation observations.

arguments
    responsibilityBlend (1,1) double
    gpMeanScale (1,1) double
    trust (1,1) double
    agreementF (3,1) double
    meanF (3,1) double
    gpValid (1,1) logical
    minimumTrust (1,1) double
    zeroTolerance (1,1) double {mustBeNonnegative}
end

finiteInputs = isfinite(responsibilityBlend) && isfinite(gpMeanScale) ...
    && isfinite(trust) && all(isfinite(agreementF)) && all(isfinite(meanF));
valid = gpValid && finiteInputs && trust >= minimumTrust;
if valid
    alphaF = min(max(responsibilityBlend .* gpMeanScale .* trust ...
        .* agreementF, 0.0), 1.0);
    weightedMeanF = alphaF .* meanF;
else
    alphaF = zeros(3,1);
    weightedMeanF = zeros(3,1);
end

exactB1 = ~valid || max(abs(alphaF)) <= zeroTolerance ...
    || norm(weightedMeanF, 2) <= zeroTolerance;
if exactB1
    alphaF = zeros(3,1);
    weightedMeanF = zeros(3,1);
end

if ~finiteInputs
    reason = "NONFINITE_GP_STATE";
elseif ~gpValid
    reason = "INVALID_OR_STALE_GP";
elseif trust < minimumTrust
    reason = "TRUST_BELOW_MINIMUM";
elseif max(abs(alphaF)) <= zeroTolerance
    reason = "ZERO_EFFECTIVE_AUTHORITY";
elseif norm(weightedMeanF, 2) <= zeroTolerance
    reason = "ZERO_EFFECTIVE_MEAN";
else
    reason = "ACTIVE_CAUSAL_GP";
end

responsibility = struct( ...
    "valid", valid, ...
    "exact_b1_required", exactB1, ...
    "reason", reason, ...
    "blend", min(max(responsibilityBlend, 0.0), 1.0), ...
    "trust", trust, ...
    "agreement_f", min(max(agreementF, 0.0), 1.0), ...
    "alpha_effective_f", alphaF, ...
    "alpha_effective_mean", mean(alphaF), ...
    "mean_f_mps2", meanF, ...
    "weighted_mean_f_mps2", weightedMeanF);
end
