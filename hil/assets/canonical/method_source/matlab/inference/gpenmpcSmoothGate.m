function value = gpenmpcSmoothGate(metric, softLimit, hardLimit)
%GPENMPCSMOOTHGATE Smooth confidence weight between the soft and hard limits.

value = zeros(size(metric), 'like', metric);
below = isfinite(metric) & metric <= softLimit;
middle = isfinite(metric) & metric > softLimit & metric < hardLimit;
value(below) = 1.0;
fraction = (metric(middle) - softLimit) ./ (hardLimit - softLimit);
value(middle) = 1.0 - fraction.^2 .* (3.0 - 2.0 .* fraction);
end
