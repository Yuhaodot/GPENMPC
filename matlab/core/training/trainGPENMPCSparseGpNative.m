function result = trainGPENMPCSparseGpNative(dataPath, inducingPath, outputRoot, options)
%TRAINGPENMPCSPARSEGPNATIVE Train the GPENMPC sparse GP from native MATLAB inputs.
%
% The data MAT is produced by buildGPENMPCNativeTrainingData and the inducing
% MAT by buildGPENMPCNativeInducingData. The configuration is a 17-input,
% three-output, M=256, ARD-RBF, Titsias-style collapsed VFE fit with two
% initializations and 48 Adam steps per initialization by default.

arguments
    dataPath (1,1) string
    inducingPath (1,1) string
    outputRoot (1,1) string
    options.OptimizationSteps (1,1) double {mustBeInteger,mustBePositive} = 48
    options.Restarts (1,1) double {mustBeMember(options.Restarts,[1,2])} = 2
end

if ~isfile(dataPath)
    error("trainGPENMPCSparseGpNative:Input", ...
        "MATLAB-native training data are missing: %s", dataPath);
end
if ~isfile(inducingPath)
    error("trainGPENMPCSparseGpNative:Input", ...
        "MATLAB-native inducing inputs are missing: %s", inducingPath);
end
if isfolder(outputRoot) && ~isempty(dir(fullfile(outputRoot, "*")))
    entries = dir(fullfile(outputRoot, "*"));
    entries = entries(~ismember({entries.name}, {'.', '..'}));
    if ~isempty(entries)
        error("trainGPENMPCSparseGpNative:AppendOnly", ...
            "Output directory is not empty: %s", outputRoot);
    end
end
if ~isfolder(outputRoot)
    mkdir(outputRoot);
end

data = load(dataPath);
requiredDataFields = [
    "X_train", "Y_train", "X_calibration", "Y_calibration", ...
    "train_mission_index", "calibration_mission_index", ...
    "input_mean_matlab", "input_scale_matlab", ...
    "output_mean_matlab", "output_scale_matlab", ...
    "featureNames", "native_data_audit"
];
requireFields(data, requiredDataFields, "MATLAB-native training data");
inducingData = load(inducingPath);
requireFields(inducingData, ...
    ["inducing_standardized_matlab", "inducing_audit", "result"], ...
    "MATLAB-native inducing data");
expectedFeatureNames = [
    "payload_fraction", ...
    "v_air_est_t_mps", ...
    "v_air_est_n_mps", ...
    "v_air_est_z_mps", ...
    "v_air_est_norm_mps", ...
    "payload_tilt_drag_basis_t_mps2", ...
    "payload_tilt_drag_basis_n_mps2", ...
    "turn_crossflow_basis_mps2", ...
    "a_ref_t_mps2", ...
    "a_ref_n_mps2", ...
    "a_ref_z_mps2", ...
    "tilt_rad", ...
    "previous_normalized_total_rotor_command", ...
    "thrust_payload_vertical_demand_basis", ...
    "previous_residual_ewma_t_mps2", ...
    "previous_residual_ewma_n_mps2", ...
    "previous_residual_ewma_z_mps2"
];
if ~isequal(string(data.featureNames(:)), expectedFeatureNames(:))
    error("trainGPENMPCSparseGpNative:FeatureIdentity", ...
        "The training data must use the required causal F17 order.");
end
requireFields(data.native_data_audit, ["status", "parity"], "Training audit");
parity = data.native_data_audit.parity;
requireFields(parity, "reference_used", "Training reference comparison");
referenceUsed = isequal(parity.reference_used, true);
if string(data.native_data_audit.status) ...
        ~= "PASS_MATLAB_NATIVE_CAUSAL_DATA_RECONSTRUCTION" ...
        || ~(referenceUsed || isequal(parity.reference_used, false)) ...
        || (referenceUsed && (~isfield(parity, "pass") || ~isequal(parity.pass, true)))
    error("trainGPENMPCSparseGpNative:DataAudit", ...
        "Training reconstruction or the requested reference comparison failed.");
end
if isfield(inducingData.result, "validation_pass")
    inducingValid = isequal(inducingData.result.validation_pass, true) ...
        && isfield(inducingData.result, "data_sha256") ...
        && strcmpi(inducingData.result.data_sha256, gpenmpcSha256File(dataPath));
else
    inducingValid = isfield(inducingData.result, "exact_within_tolerance") ...
        && isequal(inducingData.result.exact_within_tolerance, true);
end
if string(inducingData.result.status) ~= "PASS" || ~inducingValid
    error("trainGPENMPCSparseGpNative:InducingAudit", ...
        "The inducing-point selection check failed.");
end

xMeanComputed = mean(data.X_train, 1);
xScaleRaw = std(data.X_train, 0, 1);
xScaleComputed = xScaleRaw;
xScaleComputed(xScaleComputed <= 1.0e-10) = 1.0;
yMeanComputed = mean(data.Y_train, 1);
yScaleComputed = std(data.Y_train, 0, 1);
yScaleComputed(yScaleComputed <= 1.0e-12) = 1.0;
xMean = data.input_mean_matlab;
xScale = data.input_scale_matlab;
yMean = data.output_mean_matlab;
yScale = data.output_scale_matlab;
normalizationDifferences = struct;
normalizationDifferences.input_mean = max(abs(xMean - xMeanComputed), [], "all");
normalizationDifferences.input_scale = max(abs(xScale - xScaleComputed), [], "all");
normalizationDifferences.output_mean = max(abs(yMean - yMeanComputed), [], "all");
normalizationDifferences.output_scale = max(abs(yScale - yScaleComputed), [], "all");
normalizationTolerance = 1.0e-12;
if any(structfun(@(value) value > normalizationTolerance, normalizationDifferences))
    error("trainGPENMPCSparseGpNative:Normalization", ...
        "Stored MATLAB-native normalization does not match the training rows.");
end
xStandardized = (data.X_train - xMean) ./ xScale;
yStandardized = (data.Y_train - yMean) ./ yScale;
inducing = inducingData.inducing_standardized_matlab;

if ~isequal(size(xStandardized), [4680, 17]) || ~isequal(size(yStandardized), [4680, 3])
    error("trainGPENMPCSparseGpNative:Identity", "Expected 4680 by 17 and 4680 by 3 matrices.");
end
if ~isequal(size(data.X_calibration), [2340, 17]) ...
        || ~isequal(size(data.Y_calibration), [2340, 3])
    error("trainGPENMPCSparseGpNative:Identity", ...
        "Expected 2340 by 17 and 2340 by 3 calibration matrices.");
end
if ~isequal(size(inducing), [256, 17])
    error("trainGPENMPCSparseGpNative:Identity", "Expected 256 by 17 inducing inputs.");
end
if numel(unique(data.train_mission_index(:))) ~= 12 ...
        || numel(unique(data.calibration_mission_index(:))) ~= 6
    error("trainGPENMPCSparseGpNative:Identity", ...
        "Expected 12 training missions and 6 calibration missions.");
end
if any(~isfinite(xStandardized), "all") ...
        || any(~isfinite(yStandardized), "all") ...
        || any(~isfinite(data.X_calibration), "all") ...
        || any(~isfinite(data.Y_calibration), "all") ...
        || any(~isfinite(inducing), "all")
    error("trainGPENMPCSparseGpNative:Finite", ...
        "Native standardized training or inducing arrays contain nonfinite values.");
end

learningRate = 0.045;
betaOne = 0.9;
betaTwo = 0.999;
adamEpsilon = 1.0e-8;
gradientNormLimit = 20.0;
optimizationSteps = options.OptimizationSteps;
restartCount = options.Restarts;
restartInitialLength = [0.75, 1.50];
restartInitialNoise = [0.18, 0.35];
restartRecords = repmat(struct, restartCount, 1);
bestLoss = inf;
bestParameters = struct;
trainingStarted = tic;

for restartIndex = 1:restartCount
    parameters = struct;
    parameters.logLengthScale = dlarray(log(restartInitialLength(restartIndex)) .* ones(17, 1));
    parameters.logSignalStd = dlarray(0.0);
    parameters.logNoiseStd = dlarray(log(restartInitialNoise(restartIndex)) .* ones(3, 1));
    firstMoment = zeros(21, 1);
    secondMoment = zeros(21, 1);
    lossTrace = nan(optimizationSteps, 1);
    restartStarted = tic;
    finite = true;
    failureMessage = "";

    for stepIndex = 1:optimizationSteps
        try
            [loss, gradients] = dlfeval(@modelGradients, parameters, ...
                xStandardized, yStandardized, inducing);
        catch exception
            finite = false;
            failureMessage = string(exception.identifier) + ": " + string(exception.message);
            break
        end
        lossValue = double(extractdata(loss));
        gradientVector = [
            double(extractdata(gradients.logLengthScale(:)));
            double(extractdata(gradients.logSignalStd(:)));
            double(extractdata(gradients.logNoiseStd(:)))
        ];
        if ~isfinite(lossValue) || any(~isfinite(gradientVector))
            finite = false;
            failureMessage = "Nonfinite objective or gradient";
            break
        end
        gradientNorm = norm(gradientVector, 2);
        if gradientNorm > gradientNormLimit
            gradientVector = gradientVector .* (gradientNormLimit ./ gradientNorm);
        end
        firstMoment = betaOne .* firstMoment + (1.0 - betaOne) .* gradientVector;
        secondMoment = betaTwo .* secondMoment + (1.0 - betaTwo) .* gradientVector.^2;
        correctedFirst = firstMoment ./ (1.0 - betaOne.^stepIndex);
        correctedSecond = secondMoment ./ (1.0 - betaTwo.^stepIndex);
        parameterVector = [
            double(extractdata(parameters.logLengthScale(:)));
            double(extractdata(parameters.logSignalStd(:)));
            double(extractdata(parameters.logNoiseStd(:)))
        ];
        parameterVector = parameterVector - learningRate .* correctedFirst ...
            ./ (sqrt(correctedSecond) + adamEpsilon);
        parameterVector(1:17) = min(max(parameterVector(1:17), log(0.08)), log(12.0));
        parameterVector(18) = min(max(parameterVector(18), log(0.08)), log(6.0));
        parameterVector(19:21) = min(max(parameterVector(19:21), log(0.01)), log(1.50));
        parameters.logLengthScale = dlarray(parameterVector(1:17));
        parameters.logSignalStd = dlarray(parameterVector(18));
        parameters.logNoiseStd = dlarray(parameterVector(19:21));
        lossTrace(stepIndex) = lossValue;
    end

    completedSteps = find(isfinite(lossTrace), 1, "last");
    if isempty(completedSteps)
        completedSteps = 0;
    end
    finalLoss = inf;
    if finite && completedSteps == optimizationSteps
        try
            finalLoss = double(extractdata(gpenmpcCollapsedSparseVfeLoss(...
                parameters, xStandardized, yStandardized, inducing)));
            finite = isfinite(finalLoss);
            if ~finite
                failureMessage = "Nonfinite final objective";
            end
        catch exception
            finite = false;
            failureMessage = string(exception.identifier) + ": " + string(exception.message);
        end
    end
    restartRecords(restartIndex).restart = restartIndex - 1;
    restartRecords(restartIndex).finite = finite && completedSteps == optimizationSteps;
    restartRecords(restartIndex).steps_completed = completedSteps;
    restartRecords(restartIndex).initial_loss_per_sample_axis = lossTrace(1);
    restartRecords(restartIndex).final_loss_per_sample_axis = finalLoss;
    restartRecords(restartIndex).minimum_loss_per_sample_axis = min([lossTrace; finalLoss], [], "omitnan");
    restartRecords(restartIndex).elapsed_seconds = toc(restartStarted);
    restartRecords(restartIndex).failure_message = failureMessage;
    restartRecords(restartIndex).loss_trace = lossTrace(1:completedSteps);
    restartRecords(restartIndex).loss_trace_parameter_update_counts = (0:completedSteps-1).';
    restartRecords(restartIndex).final_parameter_update_count = completedSteps;
    restartRecords(restartIndex).final_lengthscale = exp(double(extractdata(parameters.logLengthScale))).';
    restartRecords(restartIndex).final_signal_std = exp(double(extractdata(parameters.logSignalStd)));
    restartRecords(restartIndex).final_noise_std_standardized = exp(double(extractdata(parameters.logNoiseStd))).';
    if restartRecords(restartIndex).finite && finalLoss < bestLoss
        bestLoss = finalLoss;
        bestParameters = struct;
        bestParameters.lengthScale = exp(double(extractdata(parameters.logLengthScale))).';
        bestParameters.signalStd = exp(double(extractdata(parameters.logSignalStd)));
        bestParameters.noiseStd = exp(double(extractdata(parameters.logNoiseStd))).';
        bestParameters.restart = restartIndex - 1;
    end
end

if ~isfinite(bestLoss)
    error("trainGPENMPCSparseGpNative:Optimization", "All requested MATLAB Adam restarts failed.");
end

[choleskyMm, posteriorMeanWhite, posteriorCovarianceWhite] = posteriorFromHyperparameters(...
    xStandardized, yStandardized, inducing, bestParameters.lengthScale, ...
    bestParameters.signalStd, bestParameters.noiseStd);
nativeModel = struct;
nativeModel.input_mean = xMean;
nativeModel.input_scale = xScale;
nativeModel.output_mean = yMean;
nativeModel.output_scale = yScale;
nativeModel.inducing_standardized = inducing;
nativeModel.kernel_name = "ARD_RBF";
nativeModel.feature_names = cellstr(expectedFeatureNames);
nativeModel.feature_set = "AERO_PHYSICS_F17";
nativeModel.feature_timing = "STRICTLY_CAUSAL_AT_K_WITH_LABEL_AVAILABLE_AT_K_PLUS_1";
nativeModel.input_active = uint8(xScaleRaw > 1.0e-10);
nativeModel.lengthscale = bestParameters.lengthScale;
nativeModel.signal_std = bestParameters.signalStd;
nativeModel.noise_std_standardized = bestParameters.noiseStd;
nativeModel.kmm_cholesky = choleskyMm;
nativeModel.posterior_mean_white = posteriorMeanWhite;
nativeModel.posterior_cov_white_axis1 = squeeze(posteriorCovarianceWhite(1, :, :));
nativeModel.posterior_cov_white_axis2 = squeeze(posteriorCovarianceWhite(2, :, :));
nativeModel.posterior_cov_white_axis3 = squeeze(posteriorCovarianceWhite(3, :, :));

[nativeMean, nativeRawStd] = predictUncalibrated(nativeModel, data.X_calibration);
calibrationScore = abs(data.Y_calibration - nativeMean) ./ max(nativeRawStd, 1.0e-12);
calibrationWeights = taskEqualWeights(data.calibration_mission_index);
calibrationQuantile = zeros(1, 3);
for axisIndex = 1:3
    calibrationQuantile(axisIndex) = weightedQuantile(...
        calibrationScore(:, axisIndex), calibrationWeights, 0.90);
end
nativeHalfWidth = nativeRawStd .* calibrationQuantile;
nativeModel.calibration_score_quantile = calibrationQuantile;
[~, ~, trainLatentVariance, trainSupportDistance] = ...
    predictUncalibrated(nativeModel, data.X_train);
[~, ~, calibrationLatentVariance, calibrationSupportDistance] = ...
    predictUncalibrated(nativeModel, data.X_calibration);
combinedMissionIndex = [
    double(data.train_mission_index(:));
    12.0 + double(data.calibration_mission_index(:))
];
oodWeights = taskEqualWeights(combinedMissionIndex);
combinedSupportDistance = [trainSupportDistance; calibrationSupportDistance];
combinedLatentMaximum = [
    max(trainLatentVariance, [], 2);
    max(calibrationLatentVariance, [], 2)
];
nativeModel.distance_soft_q95 = weightedQuantile(...
    combinedSupportDistance, oodWeights, 0.95);
nativeModel.distance_hard_q995 = weightedQuantile(...
    combinedSupportDistance, oodWeights, 0.995);
nativeModel.latent_soft_q95 = weightedQuantile(...
    combinedLatentMaximum, oodWeights, 0.95);
nativeModel.latent_hard_q995 = weightedQuantile(...
    combinedLatentMaximum, oodWeights, 0.995);
covered = abs(data.Y_calibration - nativeMean) <= nativeHalfWidth;
missionIds = unique(data.calibration_mission_index(:), "stable");
missionCoverage = zeros(length(missionIds), 1);
for missionIndex = 1:length(missionIds)
    mask = data.calibration_mission_index(:) == missionIds(missionIndex);
    missionCoverage(missionIndex) = mean(covered(mask, :), "all");
end

result = struct;
result.schema = "GPENMPC_MATLAB_NATIVE_TITSIAS_SPARSE_GP_TRAINING_RESULT_V2";
result.status = "COMPLETE_MATLAB_NATIVE_DATA_AND_INDUCING_TRAINING";
result.training_missions = 12;
result.calibration_missions = 6;
result.training_rows = size(data.X_train, 1);
result.calibration_rows = size(data.X_calibration, 1);
result.input_dimension = 17;
result.output_dimension = 3;
result.inducing_count = 256;
result.objective = "COLLAPSED_VARIATIONAL_FREE_ENERGY";
result.optimizer = "MATLAB_AUTOMATIC_DIFFERENTIATION_WITH_ADAM";
result.automatic_differentiation_route = ...
    "EXACT_DOUBLE_CHOLESKY_MATRIX_ADJOINTS_WITH_DLGRADIENT_ARD_KERNEL_VJP";
result.optimization_steps = optimizationSteps;
result.restarts = restartCount;
result.selected_restart = bestParameters.restart;
result.selected_loss_per_sample_axis = bestLoss;
result.lengthscale = bestParameters.lengthScale;
result.signal_std = bestParameters.signalStd;
result.noise_std_standardized = bestParameters.noiseStd;
result.calibration_score_quantile = calibrationQuantile;
result.ood_thresholds = struct;
result.ood_thresholds.distance_soft_q95 = nativeModel.distance_soft_q95;
result.ood_thresholds.distance_hard_q995 = nativeModel.distance_hard_q995;
result.ood_thresholds.latent_soft_q95 = nativeModel.latent_soft_q95;
result.ood_thresholds.latent_hard_q995 = nativeModel.latent_hard_q995;
result.ood_thresholds.source = ...
    "TASK_EQUAL_WEIGHTED_TRAIN_AND_CALIBRATION_SELECTED_ROWS";
result.task_equal_calibration_coverage = mean(missionCoverage);
result.worst_mission_calibration_coverage = min(missionCoverage);
result.calibration_mean_error_mps2 = mean(nativeMean - data.Y_calibration, 1);
result.calibration_rmse_mps2 = sqrt(mean(...
    (nativeMean - data.Y_calibration).^2, 1));
result.normalization_validation = normalizationDifferences;
result.normalization_validation.maximum_allowed_absolute_difference = ...
    normalizationTolerance;
result.data_source_identity = "MATLAB_NATIVE_CAUSAL_F17_DATA";
result.inducing_source_identity = "MATLAB_NATIVE_DETERMINISTIC_KMEANS_PLUS_PLUS";
result.inducing_variable = "inducing_standardized_matlab";
result.restart_records = restartRecords;
result.elapsed_seconds = toc(trainingStarted);

numericModelFields = [
    "input_mean", "input_scale", "output_mean", "output_scale", ...
    "inducing_standardized", "input_active", "lengthscale", ...
    "signal_std", "noise_std_standardized", "kmm_cholesky", ...
    "posterior_mean_white", "posterior_cov_white_axis1", ...
    "posterior_cov_white_axis2", "posterior_cov_white_axis3", ...
    "calibration_score_quantile", "distance_soft_q95", ...
    "distance_hard_q995", "latent_soft_q95", "latent_hard_q995"
];
for modelFieldIndex = 1:numel(numericModelFields)
    value = nativeModel.(numericModelFields(modelFieldIndex));
    if any(~isfinite(double(value)), "all")
        error("trainGPENMPCSparseGpNative:SavedModelFinite", ...
            "Saved model field %s contains nonfinite values.", ...
            numericModelFields(modelFieldIndex));
    end
end
result.all_saved_model_arrays_finite = true;

modelPath = fullfile(outputRoot, "MATLAB_NATIVE_SPARSE_GP_MODEL.mat");
save(modelPath, "nativeModel", "result", "-v7.3");
jsonPath = fullfile(outputRoot, "MATLAB_NATIVE_SPARSE_GP_TRAINING_RESULT.json");
if isfile(jsonPath)
    error("trainGPENMPCSparseGpNative:AppendOnly", "Refusing to overwrite result JSON.");
end
fileId = fopen(jsonPath, "w", "n", "UTF-8");
if fileId < 0
    error("trainGPENMPCSparseGpNative:Output", "Cannot create result JSON.");
end
cleanup = onCleanup(@() fclose(fileId));
fwrite(fileId, jsonencode(result, PrettyPrint=true), "char");
clear cleanup
end


function requireFields(value, names, description)
for fieldIndex = 1:numel(names)
    if ~isfield(value, names(fieldIndex))
        error("trainGPENMPCSparseGpNative:Schema", ...
            "%s are missing required field %s.", description, names(fieldIndex));
    end
end
end


function [loss, gradients] = modelGradients(parameters, x, y, inducing)
[loss, gradients] = gpenmpcCollapsedSparseVfeLossAndGradients(...
    parameters, x, y, inducing);
end


function [choleskyMm, posteriorMean, posteriorCovariance] = ...
        posteriorFromHyperparameters(x, y, inducing, lengthScale, signalStd, noiseStd)
inducingCount = size(inducing, 1);
identity = eye(inducingCount);
kernelMm = gpenmpcArdRbfKernel(inducing, inducing, lengthScale, signalStd) ...
    + 1.0e-6 .* identity;
choleskyMm = chol(kernelMm, "lower");
kernelMn = gpenmpcArdRbfKernel(inducing, x, lengthScale, signalStd);
whitened = choleskyMm \ kernelMn;
posteriorMean = zeros(3, inducingCount);
posteriorCovariance = zeros(3, inducingCount, inducingCount);
for axisIndex = 1:3
    scaled = whitened ./ noiseStd(axisIndex);
    system = identity + scaled * scaled.';
    choleskySystem = chol(system + 1.0e-9 .* identity, "lower");
    covariance = choleskySystem.' \ (choleskySystem \ identity);
    rightHandSide = scaled * y(:, axisIndex) ./ noiseStd(axisIndex);
    posteriorMean(axisIndex, :) = (covariance * rightHandSide).';
    posteriorCovariance(axisIndex, :, :) = covariance;
end
end


function [meanMps2, rawStdMps2, latentVariance, supportDistance] = ...
        predictUncalibrated(model, inputs)
standardized = (inputs - model.input_mean) ./ model.input_scale;
kernelZx = gpenmpcArdRbfKernel(model.inducing_standardized, standardized, ...
    model.lengthscale, model.signal_std);
phi = model.kmm_cholesky \ kernelZx;
standardizedMean = phi.' * model.posterior_mean_white.';
sampleCount = size(inputs, 1);
latentVariance = zeros(sampleCount, 3);
projection = sum(phi .* phi, 1).';
covariances = {
    model.posterior_cov_white_axis1;
    model.posterior_cov_white_axis2;
    model.posterior_cov_white_axis3
};
for axisIndex = 1:3
    covarianceProjection = sum(phi .* (covariances{axisIndex} * phi), 1).';
    latentVariance(:, axisIndex) = max(model.signal_std.^2 - projection ...
        + covarianceProjection, 1.0e-12);
end
predictiveVariance = latentVariance + model.noise_std_standardized.^2;
meanMps2 = model.output_mean + standardizedMean .* model.output_scale;
rawStdMps2 = sqrt(predictiveVariance) .* model.output_scale;
inputSquaredNorm = sum(standardized.^2, 2);
inducingSquaredNorm = sum(model.inducing_standardized.^2, 2).';
squaredDistance = max(inputSquaredNorm + inducingSquaredNorm ...
    - 2.0 .* (standardized * model.inducing_standardized.'), 0.0);
supportDistance = sqrt(min(squaredDistance, [], 2) ...
    ./ max(size(standardized, 2), 1));
end


function weights = taskEqualWeights(missionIndex)
missionIndex = missionIndex(:);
missions = unique(missionIndex, "stable");
weights = zeros(size(missionIndex));
for index = 1:length(missions)
    mask = missionIndex == missions(index);
    weights(mask) = 1.0 ./ sum(mask);
end
end


function value = weightedQuantile(values, weights, probability)
[orderedValues, order] = sort(values(:), "ascend");
orderedWeights = weights(order);
cumulative = cumsum(orderedWeights);
threshold = probability .* sum(orderedWeights);
position = find(cumulative >= threshold, 1, "first");
value = orderedValues(min(position, length(orderedValues)));
end
