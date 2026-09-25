function prepared = prepareCanonicalCurrentGpPrediction(velocityI, reference, ...
        windEstimateXY, payloadKg, desiredForceI, previousRotorCommandN, ...
        residualHistoryF, context, gpModelAvailable, causalValid, config)
% PREPARECANONICALCURRENTGPPREDICTION Prepare canonical GP features.
% The caller supplies gpModelAvailable from its verified model binding.
% Required query: prediction = gpenmpcSparseGpPredict(model, prepared.features_f17).
% The prediction stays open until the next-sample innovation closure.
arguments
    velocityI (3,1) double
    reference (1,1) struct
    windEstimateXY (2,1) double
    payloadKg (1,1) double
    desiredForceI (3,1) double
    previousRotorCommandN double
    residualHistoryF (3,1) double
    context (1,1) struct
    gpModelAvailable (1,1) logical
    causalValid (1,1) logical
    config (1,1) struct
end

[frameIFromF, curvature] = gpenmpcFrenetFrame(reference);
pending = emptyPending(frameIFromF, minimumSoftTrust(config));
pending.causal_valid = causalValid;
pending.gp_model_available = gpModelAvailable;
prepared = struct('schema', "CANONICAL_CURRENT_GP_NUMERICAL_SPLIT_V1", ...
    'prediction_required', false, 'pending', pending, ...
    'features_f17', nan(1, 17), 'gp_mean_scale', NaN);
if ~pending.causal_valid || ~pending.gp_model_available
    return
end

history = [gpenmpcNormalizedTotalRotorCommand( ...
    previousRotorCommandN, payloadKg); residualHistoryF];
wind3 = [windEstimateXY; 0.0];
features = gpenmpcBuildAeroF17Features(velocityI, reference, ...
    frameIFromF, wind3, payloadKg, desiredForceI, curvature, history, context);
prepared.prediction_required = true;
prepared.features_f17 = features.';
prepared.gp_mean_scale = config.gp_mean_scale;
end

function pending = emptyPending(frameIFromF, minimumTrust)
% Field order, types, initial NaNs, and flags match the canonical function.
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
        "The existing minimum soft trust is missing.");
end
end
