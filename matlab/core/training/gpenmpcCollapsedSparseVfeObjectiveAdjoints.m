function [loss, gradientKmm, gradientKmn, ...
        gradientSignalDiagonal, gradientLogNoise] = ...
        gpenmpcCollapsedSparseVfeObjectiveAdjoints(xStandardized, ...
        yStandardized, inducingStandardized, lengthScale, signalStd, noiseStd)
%GPENMPCCOLLAPSEDSPARSEVFEOBJECTIVEADJOINTS Exact VFE and matrix adjoints.
%
% The implementation evaluates the collapsed Titsias variational free
% energy with ordinary double-precision Cholesky factors. It also returns
% exact adjoints with respect to Kmm, Kmn, the RBF diagonal variance and
% log likelihood noise. A separate function passes these adjoints through
% the differentiable ARD kernel with MATLAB automatic differentiation.

sampleCount = size(xStandardized, 1);
inducingCount = size(inducingStandardized, 1);
identity = eye(inducingCount);
kernelMm = gpenmpcArdRbfKernel(inducingStandardized, ...
    inducingStandardized, lengthScale, signalStd) + 1.0e-6 .* identity;
kernelMm = 0.5 .* (kernelMm + kernelMm.');
kernelMn = gpenmpcArdRbfKernel(inducingStandardized, xStandardized, ...
    lengthScale, signalStd);
choleskyMm = chol(kernelMm, "lower");
whitened = choleskyMm \ kernelMn;
inverseKmm = choleskyMm.' \ (choleskyMm \ identity);
rawTraceGap = sampleCount .* signalStd.^2 ...
    - sum(whitened.^2, "all");
traceActive = double(rawTraceGap > 0.0);
traceGap = max(rawTraceGap, 0.0);
kernelCrossProduct = kernelMn * kernelMn.';

loss = 0.0;
gradientKmm = zeros(inducingCount);
gradientKmn = zeros(inducingCount, sampleCount);
gradientSignalDiagonal = 0.0;
gradientLogNoise = zeros(3, 1);
for axisIndex = 1:3
    axisNoise = noiseStd(axisIndex);
    noiseVariance = axisNoise.^2;
    scaled = whitened ./ axisNoise;
    system = identity + scaled * scaled.';
    system = 0.5 .* (system + system.') + 1.0e-9 .* identity;
    choleskySystem = chol(system, "lower");
    response = yStandardized(:, axisIndex);
    projected = choleskySystem \ (scaled * response);
    quadratic = (sum(response.^2) - sum(projected.^2)) ...
        ./ noiseVariance;
    logDeterminant = sampleCount .* log(noiseVariance) ...
        + 2.0 .* sum(log(diag(choleskySystem)));
    loss = loss + 0.5 .* (sampleCount .* log(2.0 .* pi) ...
        + logDeterminant + quadratic + traceGap ./ noiseVariance);

    woodburySystem = kernelMm ...
        + kernelCrossProduct ./ noiseVariance;
    woodburySystem = 0.5 .* (woodburySystem + woodburySystem.');
    choleskyWoodbury = chol(woodburySystem, "lower");
    inverseWoodburyKernel = choleskyWoodbury.' ...
        \ (choleskyWoodbury \ kernelMn);
    projectedResponse = inverseWoodburyKernel * response;
    alpha = response ./ noiseVariance ...
        - kernelMn.' * projectedResponse ./ noiseVariance.^2;
    kernelAlpha = kernelMn * alpha;

    identityCoefficient = (1.0 - traceActive) ./ noiseVariance;
    kernelTimesA = 0.5 .* (identityCoefficient .* kernelMn ...
        - kernelCrossProduct * inverseWoodburyKernel ...
        ./ noiseVariance.^2 - kernelAlpha * alpha.');
    gradientKmn = gradientKmn + 2.0 .* inverseKmm * kernelTimesA;
    gradientKmm = gradientKmm ...
        - inverseKmm * (kernelTimesA * kernelMn.') * inverseKmm;
    gradientSignalDiagonal = gradientSignalDiagonal ...
        + traceActive .* 0.5 .* sampleCount ./ noiseVariance;

    traceInverseCovariance = sampleCount ./ noiseVariance ...
        - sum(kernelMn .* inverseWoodburyKernel, "all") ...
        ./ noiseVariance.^2;
    derivativeNoiseVariance = 0.5 .* (traceInverseCovariance ...
        - sum(alpha.^2) - traceGap ./ noiseVariance.^2);
    gradientLogNoise(axisIndex) = 2.0 .* noiseVariance ...
        .* derivativeNoiseVariance;
end

normalizer = 3.0 .* sampleCount;
loss = loss ./ normalizer;
gradientKmm = 0.5 .* (gradientKmm + gradientKmm.') ./ normalizer;
gradientKmn = gradientKmn ./ normalizer;
gradientSignalDiagonal = gradientSignalDiagonal ./ normalizer;
gradientLogNoise = gradientLogNoise ./ normalizer;
end
