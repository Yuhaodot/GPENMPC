function kernel = gpenmpcArdRbfKernel(left, right, lengthScale, signalStd)
%GPENMPCARDRBFKERNEL ARD squared exponential covariance used by GPENMPC.

scaledLeft = left ./ reshape(lengthScale, 1, []);
scaledRight = right ./ reshape(lengthScale, 1, []);
squaredDistance = max(sum(scaledLeft .* scaledLeft, 2) ...
    + sum(scaledRight .* scaledRight, 2).' ...
    - 2.0 .* (scaledLeft * scaledRight.'), 0.0);
kernel = signalStd.^2 .* exp(-0.5 .* squaredDistance);
end
