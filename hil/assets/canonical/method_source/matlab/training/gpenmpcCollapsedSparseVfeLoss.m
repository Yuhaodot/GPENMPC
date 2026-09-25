function loss = gpenmpcCollapsedSparseVfeLoss(parameters, xStandardized, ...
    yStandardized, inducingStandardized)
%GPENMPCCOLLAPSEDSPARSEVFELOSS Collapsed Titsias variational objective.
%
% The objective is evaluated with double-precision Cholesky algebra. Its companion
% gpenmpcCollapsedSparseVfeLossAndGradients supplies exact matrix adjoints
% and uses dlgradient for the ARD kernel vector-Jacobian product.

lengthScale = exp(double(extractdata(parameters.logLengthScale)));
signalStd = exp(double(extractdata(parameters.logSignalStd)));
noiseStd = exp(double(extractdata(parameters.logNoiseStd)));
[lossValue, ~, ~, ~, ~] = gpenmpcCollapsedSparseVfeObjectiveAdjoints(...
    xStandardized, yStandardized, inducingStandardized, ...
    lengthScale, signalStd, noiseStd);
loss = dlarray(lossValue);
end
