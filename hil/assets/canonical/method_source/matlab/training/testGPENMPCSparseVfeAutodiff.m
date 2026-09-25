function result = testGPENMPCSparseVfeAutodiff(outputPath)
%TESTGPENMPCSPARSEVFEAUTODIFF Lightweight MATLAB autodiff preflight.

arguments
    outputPath (1,1) string = ""
end

rng(20260831, "twister");
x = randn(32, 17);
y = randn(32, 3);
inducing = x(1:16, :);
parameters = struct;
parameters.logLengthScale = dlarray(log(1.25) .* ones(17, 1));
parameters.logSignalStd = dlarray(0.0);
parameters.logNoiseStd = dlarray(log(0.25) .* ones(3, 1));
[loss, gradients] = dlfeval(@localGradients, parameters, x, y, inducing);
values = [
    double(extractdata(loss));
    double(extractdata(gradients.logLengthScale(:)));
    double(extractdata(gradients.logSignalStd(:)));
    double(extractdata(gradients.logNoiseStd(:)))
];
result = struct;
result.status = "PASS_MATLAB_SPARSE_VFE_AUTODIFF_PREFLIGHT";
result.finite = all(isfinite(values));
result.loss_per_sample_axis = values(1);
result.gradient_norm = norm(values(2:end));
parameterVector = [
    double(extractdata(parameters.logLengthScale(:)));
    double(extractdata(parameters.logSignalStd(:)));
    double(extractdata(parameters.logNoiseStd(:)))
];
analyticGradient = values(2:end);
finiteDifferenceGradient = localFiniteDifference(parameterVector, x, y, inducing);
gradientDifference = abs(analyticGradient - finiteDifferenceGradient);
gradientScale = max(abs(finiteDifferenceGradient), 1.0e-8);
result.finite_difference_max_absolute_error = max(gradientDifference);
result.finite_difference_max_relative_error = max(gradientDifference ./ gradientScale);
result.finite_difference_tolerance_absolute = 2.0e-5;
result.finite_difference_tolerance_relative = 2.0e-3;
result.gradient_check_pass = ...
    result.finite_difference_max_absolute_error ...
        <= result.finite_difference_tolerance_absolute ...
    || result.finite_difference_max_relative_error ...
        <= result.finite_difference_tolerance_relative;
result.rows = size(x, 1);
result.features = size(x, 2);
result.outputs = size(y, 2);
result.inducing = size(inducing, 1);
if ~result.finite || ~result.gradient_check_pass
    error("testGPENMPCSparseVfeAutodiff:Nonfinite", ...
        "The MATLAB sparse VFE objective or gradient check failed.");
end
fprintf("%s|loss=%.12g|gradient_norm=%.12g|fd_abs=%.3g|fd_rel=%.3g\n", ...
    result.status, result.loss_per_sample_axis, result.gradient_norm, ...
    result.finite_difference_max_absolute_error, ...
    result.finite_difference_max_relative_error);
if strlength(outputPath) > 0
    if isfile(outputPath)
        error("testGPENMPCSparseVfeAutodiff:AppendOnly", ...
            "Refusing to overwrite %s", outputPath);
    end
    file = fopen(outputPath, "w", "n", "UTF-8");
    if file < 0
        error("testGPENMPCSparseVfeAutodiff:Output", ...
            "Cannot create %s", outputPath);
    end
    cleanup = onCleanup(@() fclose(file));
    fwrite(file, jsonencode(result, PrettyPrint=true), "char");
end
end


function [loss, gradients] = localGradients(parameters, x, y, inducing)
[loss, gradients] = gpenmpcCollapsedSparseVfeLossAndGradients(...
    parameters, x, y, inducing);
end


function gradient = localFiniteDifference(parameterVector, x, y, inducing)
step = 1.0e-5;
gradient = zeros(size(parameterVector));
for index = 1:numel(parameterVector)
    upper = parameterVector;
    lower = parameterVector;
    upper(index) = upper(index) + step;
    lower(index) = lower(index) - step;
    gradient(index) = (localNumericLoss(upper, x, y, inducing) ...
        - localNumericLoss(lower, x, y, inducing)) ./ (2.0 .* step);
end
end


function loss = localNumericLoss(parameterVector, x, y, inducing)
lengthScale = exp(parameterVector(1:17));
signalStd = exp(parameterVector(18));
noiseStd = exp(parameterVector(19:21));
[loss, ~, ~, ~, ~] = gpenmpcCollapsedSparseVfeObjectiveAdjoints(...
    x, y, inducing, lengthScale, signalStd, noiseStd);
end
