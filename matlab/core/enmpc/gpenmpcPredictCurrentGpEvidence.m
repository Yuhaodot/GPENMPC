function pending = gpenmpcPredictCurrentGpEvidence(velocityI, reference, ...
        windEstimateXY, payloadKg, desiredForceI, previousRotorCommandN, ...
        residualHistoryF, context, gpModel, causalValid, config)
%GPENMPCPREDICTCURRENTGPEVIDENCE Record a same-sample causal GP prediction.
%
% The prediction is formed after the current control command is available
% and before the next plant observation arrives. The matching innovation is
% attached only by gpenmpcCloseGpInnovationEvidence at k+1.

arguments
    velocityI (3,1) double
    reference (1,1) struct
    windEstimateXY (2,1) double
    payloadKg (1,1) double
    desiredForceI (3,1) double
    previousRotorCommandN double
    residualHistoryF (3,1) double
    context (1,1) struct
    gpModel struct
    causalValid (1,1) logical
    config (1,1) struct
end

[frameIFromF, curvature] = gpenmpcFrenetFrame(reference);
pending = emptyPending(frameIFromF, minimumSoftTrust(config));
pending.causal_valid = causalValid;
pending.gp_model_available = ~isempty(fieldnames(gpModel));
if ~pending.causal_valid || ~pending.gp_model_available
    return
end

history = [gpenmpcNormalizedTotalRotorCommand( ...
    previousRotorCommandN, payloadKg); residualHistoryF];
wind3 = [windEstimateXY; 0.0];
features = gpenmpcBuildAeroF17Features(velocityI, reference, ...
    frameIFromF, wind3, payloadKg, desiredForceI, curvature, history, context);
prediction = gpenmpcSparseGpPredict(gpModel, features.');

pending.available = true;
pending.features_f17 = features.';
pending.predicted_mean_f_mps2 = double(prediction.mean_mps2(1, :));
pending.calibrated_half_width_f_mps2 = ...
    double(prediction.calibrated_half_width_mps2(1, :));
pending.trust = double(prediction.trust(1));
pending.support_distance = double(prediction.support_distance(1));
pending.latent_variance_max = ...
    max(double(prediction.latent_variance_standardized(1, :)));
pending.hard_invalid = logical(prediction.hard_invalid(1));
pending.runtime_weighted_mean_f_mps2 = config.gp_mean_scale ...
    .* pending.trust .* pending.predicted_mean_f_mps2;
end


function pending = emptyPending(frameIFromF, minimumTrust)
pending = struct;
pending.schema = "GPENMPC_CURRENT_SAMPLE_GP_PREDICTION_V1";
pending.read_only = true;
pending.available = false;
pending.causal_valid = false;
pending.gp_model_available = false;
pending.hard_invalid = false;
pending.trust = 0.0;
pending.minimum_soft_trust = minimumTrust;
pending.support_distance = NaN;
pending.latent_variance_max = NaN;
pending.features_f17 = nan(1, 17);
pending.gp_frame_i_from_f = frameIFromF;
pending.predicted_mean_f_mps2 = nan(1, 3);
pending.runtime_weighted_mean_f_mps2 = nan(1, 3);
pending.calibrated_half_width_f_mps2 = nan(1, 3);
pending.prediction_sample_closed = false;
pending.observed_innovation_available = false;
pending.observed_innovation_f_mps2 = nan(1, 3);
pending.innovation_error_f_mps2 = nan(1, 3);
pending.observed_innovation_consistent = false;
pending.eligible_for_b1_dwell = false;
end


function value = minimumSoftTrust(config)
if isfield(config, "minimum_soft_trust")
    value = double(config.minimum_soft_trust);
elseif isfield(config, "raw") && isfield(config.raw, "uncertainty_tightening")
    value = double(config.raw.uncertainty_tightening.minimum_soft_trust);
else
    error("gpenmpcPredictCurrentGpEvidence:Config", ...
        "Configuration must define minimum_soft_trust.");
end
end
