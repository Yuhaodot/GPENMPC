function [nextWeightF, diagnostic] = gpenmpcUpdateGpAgreementWeight( ...
        previousWeightF, evidence, actualDtS, config)
%GPENMPCUPDATEGPAGREEMENTWEIGHT Causal per-axis GP responsibility update.
%
% Each GP axis receives a continuous score from its normalized innovation.
% The score is filtered with actual dt and the configured 0.50 s causal
% memory using current and past samples.

arguments
    previousWeightF (3,1) double
    evidence (1,1) struct
    actualDtS (1,1) double {mustBePositive}
    config (1,1) struct
end

timeConstantS = consistencyTimeConstant(config);
minimumTrust = double(config.minimum_soft_trust);
rawWeightF = zeros(3, 1);
valid = logicalField(evidence, "available", false) ...
    && logicalField(evidence, "prediction_sample_closed", false) ...
    && logicalField(evidence, "observed_innovation_available", false) ...
    && ~logicalField(evidence, "hard_invalid", true) ...
    && numericField(evidence, "trust", 0.0) >= minimumTrust;
normalizedErrorF = inf(3, 1);
if valid
    predicted = double(evidence.predicted_mean_f_mps2(:));
    observed = double(evidence.observed_innovation_f_mps2(:));
    halfWidth = double(evidence.calibrated_half_width_f_mps2(:));
    if numel(predicted) ~= 3 || numel(observed) ~= 3 || numel(halfWidth) ~= 3 ...
            || any(~isfinite([predicted; observed; halfWidth])) ...
            || any(halfWidth <= 0.0)
        error("gpenmpcUpdateGpAgreementWeight:Evidence", ...
            "Closed prediction, innovation and half-width must be valid three-axis vectors.");
    end
    normalizedErrorF = abs(observed - predicted) ./ max(halfWidth, 1.0e-12);
    rawWeightF = 1.0 ./ (1.0 + normalizedErrorF .^ 2);
end

alpha = 1.0 - exp(-actualDtS ./ timeConstantS);
if valid
    nextWeightF = previousWeightF + alpha .* (rawWeightF - previousWeightF);
else
    % Stale, low-trust, or hard-invalid evidence sets GP authority to zero for
    % the current update; subsequent closed observations update it through the
    % causal filter.
    nextWeightF = zeros(3, 1);
end
nextWeightF = min(max(nextWeightF, 0.0), 1.0);

diagnostic = struct;
diagnostic.schema = "GPENMPC_GP_CAUSAL_AGREEMENT_WEIGHT_V1";
diagnostic.valid_closed_evidence = valid;
diagnostic.raw_weight_f = rawWeightF;
diagnostic.normalized_innovation_error_f = normalizedErrorF;
diagnostic.instantaneous_consistency_score_f = rawWeightF;
diagnostic.previous_weight_f = previousWeightF;
diagnostic.next_weight_f = nextWeightF;
diagnostic.aggregate_consistency_score = mean(nextWeightF);
diagnostic.enter_threshold = consistencyEnterThreshold(config);
diagnostic.exit_threshold = consistencyExitThreshold(config);
diagnostic.enter_eligible = diagnostic.aggregate_consistency_score ...
    >= diagnostic.enter_threshold;
diagnostic.exit_low = diagnostic.aggregate_consistency_score ...
    < diagnostic.exit_threshold;
diagnostic.actual_dt_s = actualDtS;
diagnostic.time_constant_s = timeConstantS;
end


function value = consistencyTimeConstant(config)
if isfield(config, "innovation_consistency_time_constant_s")
    value = double(config.innovation_consistency_time_constant_s);
else
    value = 0.50;
end
if ~isscalar(value) || ~isfinite(value) || value <= 0.0
    error("gpenmpcUpdateGpAgreementWeight:TimeConstant", ...
        "The consistency time constant must be positive and finite.");
end
end


function value = consistencyEnterThreshold(config)
if isfield(config, "innovation_consistency_enter_threshold")
    value = double(config.innovation_consistency_enter_threshold);
else
    value = 0.50;
end
end


function value = consistencyExitThreshold(config)
if isfield(config, "innovation_consistency_exit_threshold")
    value = double(config.innovation_consistency_exit_threshold);
else
    value = 0.25;
end
end


function value = logicalField(source, name, fallback)
if isfield(source, name), value = logical(source.(name)); else, value = logical(fallback); end
end


function value = numericField(source, name, fallback)
if isfield(source, name), value = double(source.(name)); else, value = double(fallback); end
end
