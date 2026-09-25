function rotation = gpenmpcSo3Exp(vector)
%GPENMPCSO3EXP Exponential map from a three-vector to SO(3).

value = double(vector(:));
if numel(value) ~= 3 || any(~isfinite(value))
    error("gpenmpcSo3Exp:Input", "Input must be a finite three-vector.");
end
angle = norm(value);
skew = [0, -value(3), value(2); ...
    value(3), 0, -value(1); -value(2), value(1), 0];
if angle < 1.0e-7
    a = 1.0 - angle.^2 ./ 6.0 + angle.^4 ./ 120.0;
    b = 0.5 - angle.^2 ./ 24.0 + angle.^4 ./ 720.0;
else
    a = sin(angle) ./ angle;
    b = (1.0 - cos(angle)) ./ angle.^2;
end
rotation = eye(3) + a .* skew + b .* (skew * skew);
end
