function prediction = gpenmpcSparseGpPredict(model, inputs)
%GPENMPCSPARSEGPPREDICT Sparse variational GP online inference.
%
% Inputs are rows. The three Frenet residual axes share inducing inputs and kernel
% geometry, while their posterior means, covariances and noise values remain
% independent.

arguments
    model (1,1) struct
    inputs (:,:) double
end

standardized = (inputs - model.input_mean) ./ model.input_scale;
if isfield(model, "prepared_scaled_inducing")
    scaledInducing = model.prepared_scaled_inducing;
else
    scaledInducing = model.inducing_standardized ./ model.lengthscale;
end
scaledInputs = standardized ./ model.lengthscale;
if isfield(model, "prepared_scaled_inducing_squared_norm")
    scaledInducingSquaredNorm = ...
        model.prepared_scaled_inducing_squared_norm;
else
    scaledInducingSquaredNorm = sum(scaledInducing.^2, 2);
end
squaredDistance = max(scaledInducingSquaredNorm + sum(scaledInputs.^2, 2).' ...
    - 2.0 .* (scaledInducing * scaledInputs.'), 0.0);

if ~strcmp(string(model.kernel_name), "ARD_RBF")
    error("gpenmpcSparseGpPredict:Kernel", "The supplied GP model must use ARD_RBF.");
end
kernelZx = model.signal_std.^2 .* exp(-0.5 .* squaredDistance);
phi = model.kmm_cholesky \ kernelZx;
standardizedMean = phi.' * model.posterior_mean_white.';

sampleCount = size(inputs, 1);
latentVariance = zeros(sampleCount, 3);
priorDiagonal = model.signal_std.^2;
projection = sum(phi .* phi, 1).';
if isfield(model, "prepared_posterior_covariance_white_stack")
    inducingCount = size(phi, 1);
    stackedProjection = ...
        model.prepared_posterior_covariance_white_stack * phi;
    for axisIndex = 1:3
        rows = (axisIndex - 1) .* inducingCount + (1:inducingCount);
        covarianceProjection = ...
            sum(phi .* stackedProjection(rows, :), 1).';
        latentVariance(:, axisIndex) = max( ...
            priorDiagonal - projection + covarianceProjection, 1.0e-12);
    end
else
    posteriorCovariances = {
        model.posterior_cov_white_axis1;
        model.posterior_cov_white_axis2;
        model.posterior_cov_white_axis3
    };
    for axisIndex = 1:3
        covarianceProjection = ...
            sum(phi .* (posteriorCovariances{axisIndex} * phi), 1).';
        latentVariance(:, axisIndex) = max( ...
            priorDiagonal - projection + covarianceProjection, 1.0e-12);
    end
end
predictiveVariance = latentVariance + model.noise_std_standardized.^2;

supportSquaredDistance = zeros(size(standardized, 1), ...
    size(model.inducing_standardized, 1));
for featureIndex = 1:size(standardized, 2)
    featureDifference = standardized(:, featureIndex) ...
        - model.inducing_standardized(:, featureIndex).';
    supportSquaredDistance = supportSquaredDistance + featureDifference.^2;
end
supportDistance = sqrt(min(supportSquaredDistance, [], 2) ...
    ./ max(size(standardized, 2), 1));

meanMps2 = model.output_mean + standardizedMean .* model.output_scale;
rawStdMps2 = sqrt(predictiveVariance) .* model.output_scale;
halfWidthMps2 = rawStdMps2 .* model.calibration_score_quantile;
latentMaximum = max(latentVariance, [], 2);
trust = gpenmpcSmoothGate(supportDistance, model.distance_soft_q95, model.distance_hard_q995) ...
    .* gpenmpcSmoothGate(latentMaximum, model.latent_soft_q95, model.latent_hard_q995);
hardInvalid = any(~isfinite(meanMps2), 2) ...
    | any(~isfinite(halfWidthMps2), 2) ...
    | supportDistance >= model.distance_hard_q995 ...
    | latentMaximum >= model.latent_hard_q995;
trust(hardInvalid) = 0.0;
appliedMeanMps2 = trust .* meanMps2;
appliedMeanMps2(repmat(hardInvalid, 1, size(meanMps2, 2))) = 0.0;

prediction = struct;
prediction.mean_mps2 = meanMps2;
prediction.raw_std_mps2 = rawStdMps2;
prediction.calibrated_half_width_mps2 = halfWidthMps2;
prediction.latent_variance_standardized = latentVariance;
prediction.support_distance = supportDistance;
prediction.trust = trust;
prediction.hard_invalid = hardInvalid;
prediction.applied_mean_f_mps2 = appliedMeanMps2;
end
