function pending = completeCanonicalCurrentGpPrediction(prepared, prediction)
% COMPLETECANONICALCURRENTGPPREDICTION Complete the numerical GP-query result.
% The caller validates source, ticket, age and lifecycle.
% Preserve canonical hard-invalid values, including NaNs.
arguments
    prepared (1,1) struct
    prediction (1,1) struct
end
require(all(isfield(prepared, {'schema','prediction_required','pending', ...
    'features_f17','gp_mean_scale'})) && ...
    isequal(prepared.schema, "CANONICAL_CURRENT_GP_NUMERICAL_SPLIT_V1"), ...
    'Preparation', 'Expected a numerical GP preparation.');
require(islogical(prepared.prediction_required) && isscalar(prepared.prediction_required) ...
    && isstruct(prepared.pending) && isscalar(prepared.pending), ...
    'Preparation', 'Preparation flags and pending shape are invalid.');
pending = prepared.pending;
require(all(isfield(pending, {'causal_valid','gp_model_available', ...
    'prediction_sample_closed','observed_innovation_available','available'})) ...
    && ~pending.prediction_sample_closed && ~pending.observed_innovation_available ...
    && ~pending.available, 'Preparation', 'Only an open, unfilled prediction is accepted.');
require(isequal(prepared.prediction_required, ...
    logical(pending.causal_valid && pending.gp_model_available)), ...
    'Preparation', 'Query requirement differs from the original early-return condition.');
if ~prepared.prediction_required
    require(isempty(fieldnames(prediction)), 'UnexpectedPrediction', ...
        'A skipped GP query must not supply a prediction.');
    return
end
require(isa(prepared.features_f17,'double') && ...
    isequal(size(prepared.features_f17),[1 17]) && all(isfinite(prepared.features_f17)), ...
    'Features', 'The numerical query must contain the original finite 1-by-17 row.');
rows = {'mean_mps2','calibrated_half_width_mps2','latent_variance_standardized'};
scalars = {'trust','support_distance','hard_invalid'};
require(all(isfield(prediction,[rows scalars])), 'PredictionFields', ...
    'The canonical GP result is missing a consumed field.');
for k = 1:numel(rows)
    value = prediction.(rows{k});
    require(isa(value,'double') && isreal(value) && isequal(size(value),[1 3]), ...
        'PredictionShape', 'Consumed GP vector fields must be real double 1-by-3 rows.');
end
for k = 1:numel(scalars)
    value = prediction.(scalars{k});
    require((isa(value,'double') || islogical(value)) && isreal(value) && isscalar(value), ...
        'PredictionShape', 'Consumed GP scalar fields must be real scalars.');
end

% Preserve canonical assignment and multiplication order.
% Use mean_mps2, not applied_mean; innovation closes on the next sample.
pending.available = true;
pending.features_f17 = prepared.features_f17;
pending.predicted_mean_f_mps2 = double(prediction.mean_mps2(1, :));
pending.calibrated_half_width_f_mps2 = ...
    double(prediction.calibrated_half_width_mps2(1, :));
pending.trust = double(prediction.trust(1));
pending.support_distance = double(prediction.support_distance(1));
pending.latent_variance_max = ...
    max(double(prediction.latent_variance_standardized(1, :)));
pending.hard_invalid = logical(prediction.hard_invalid(1));
pending.runtime_weighted_mean_f_mps2 = prepared.gp_mean_scale ...
    .* pending.trust .* pending.predicted_mean_f_mps2;
end

function require(ok, suffix, message)
assert(ok, "gpenmpcNative:GpSplit" + suffix, message);
end
