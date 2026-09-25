function [frameIFromF, curvature, signedYawRate] = gpenmpcFrenetFrame(reference)
%GPENMPCFRENETFRAME Build the inherited horizontal Frenet frame.

velocity = double(reference.velocity_mps(:));
acceleration = double(reference.acceleration_mps2(:));
if numel(velocity) ~= 3 || numel(acceleration) ~= 3 ...
        || any(~isfinite([velocity; acceleration]))
    error("gpenmpcFrenetFrame:Reference", ...
        "Reference velocity and acceleration must be finite three-axis vectors.");
end
horizontalSpeed = norm(velocity(1:2), 2);
if horizontalSpeed < 1.0e-9
    tangent = [1.0; 0.0];
else
    tangent = velocity(1:2) ./ horizontalSpeed;
end
frameIFromF = [tangent(1), -tangent(2), 0.0; ...
    tangent(2), tangent(1), 0.0; 0.0, 0.0, 1.0];
curvature = 0.0;
signedYawRate = 0.0;
if horizontalSpeed >= 0.75
    cross = velocity(1) .* acceleration(2) - velocity(2) .* acceleration(1);
    curvature = abs(cross) ./ max(horizontalSpeed.^3, 1.0e-12);
    signedYawRate = cross ./ max(horizontalSpeed.^2, 1.0e-12);
end
end
