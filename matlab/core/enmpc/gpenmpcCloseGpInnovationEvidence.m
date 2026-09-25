function evidence = gpenmpcCloseGpInnovationEvidence( ...
        pending, observedInnovationI, observedAvailable, config)
%GPENMPCCLOSEGPINNOVATIONEVIDENCE Pair k prediction with the k+1 label.

arguments
    pending (1,1) struct
    observedInnovationI (3,1) double
    observedAvailable (1,1) logical
    config (1,1) struct
end

evidence = pending;
evidence.schema = "GPENMPC_CLOSED_SAME_SAMPLE_GP_EVIDENCE_V1";
evidence.prediction_sample_closed = true;
evidence.observed_innovation_available = observedAvailable ...
    && all(isfinite(observedInnovationI));
if ~logicalField(evidence, "available", false) ...
        || ~evidence.observed_innovation_available
    evidence.observed_innovation_consistent = false;
    evidence.eligible_for_b1_dwell = false;
    return
end

frameIFromF = double(evidence.gp_frame_i_from_f);
if ~isequal(size(frameIFromF), [3, 3]) || any(~isfinite(frameIFromF), "all")
    error("gpenmpcCloseGpInnovationEvidence:Frame", ...
        "The stored prediction frame must be finite and three dimensional.");
end
observedF = frameIFromF.' * observedInnovationI;
evidence.observed_innovation_f_mps2 = observedF.';
evidence.innovation_error_f_mps2 = evidence.observed_innovation_f_mps2 ...
    - double(evidence.predicted_mean_f_mps2);
halfWidth = double(evidence.calibrated_half_width_f_mps2);
if numel(halfWidth) ~= 3 || any(~isfinite(halfWidth)) || any(halfWidth <= 0.0)
    error("gpenmpcCloseGpInnovationEvidence:HalfWidth", ...
        "The calibrated innovation half-width must contain three positive finite values.");
end
evidence.normalized_innovation_error_f = ...
    abs(evidence.innovation_error_f_mps2) ./ max(halfWidth, 1.0e-12);
evidence.innovation_consistency_score_f = ...
    1.0 ./ (1.0 + evidence.normalized_innovation_error_f .^ 2);
evidence.observed_innovation_consistent_f = ...
    evidence.normalized_innovation_error_f <= 1.0 + 1.0e-12;
% This field is descriptive and excluded from supervisor decisions. The
% supervisor uses the causal EWMA score from gpenmpcUpdateGpAgreementWeight.
evidence.observed_innovation_consistent = ...
    mean(evidence.innovation_consistency_score_f) ...
    >= consistencyEnterThreshold(config);
evidence.eligible_for_b1_dwell = ...
    ~logicalField(evidence, "hard_invalid", true) ...
    && double(evidence.trust) >= minimumSoftTrust(config) ...
    && evidence.observed_innovation_consistent;
end


function value = consistencyEnterThreshold(config)
if isfield(config, "innovation_consistency_enter_threshold")
    value = double(config.innovation_consistency_enter_threshold);
else
    value = 0.50;
end
end


function value = minimumSoftTrust(config)
if isfield(config, "minimum_soft_trust")
    value = double(config.minimum_soft_trust);
elseif isfield(config, "raw") && isfield(config.raw, "uncertainty_tightening")
    value = double(config.raw.uncertainty_tightening.minimum_soft_trust);
else
    error("gpenmpcCloseGpInnovationEvidence:Config", ...
        "Configuration must define minimum_soft_trust.");
end
end


function value = logicalField(source, name, fallback)
if isfield(source, name)
    value = logical(source.(name));
else
    value = logical(fallback);
end
end
