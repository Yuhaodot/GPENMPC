function features = gpenmpcBuildAeroF17Features(velocityI, reference, ...
        frameIFromF, wind3, payloadKg, desiredForce, curvature, history, context)
%GPENMPCBUILDAEROF17FEATURES Build the shared causal AERO_PHYSICS_F17 row.
%
% Shared by the ordinary-B2 rollout and the B1 shadow query, this function
% centralizes the feature order and scaling used by both paths.

airF = frameIFromF.' * (velocityI - wind3);
referenceAccelerationF = frameIFromF.' * reference.acceleration_mps2;
verticalForce = max(desiredForce(3), 1.0e-9);
tilt = atan2(norm(desiredForce(1:2), 2), verticalForce);
payloadFraction = payloadKg ./ context.maximum_payload_kg;
speed = norm(airF, 2);
tiltFactor = 1.0 + 0.35 .* sin(tilt).^2;
dragT = payloadFraction .* tiltFactor .* airF(1) .* abs(airF(1));
dragN = payloadFraction .* tiltFactor .* airF(2) .* abs(airF(2));
turn = curvature .* airF(1) .* abs(airF(1)) ...
    + airF(2) .* abs(airF(1));
previousCommand = history(1);
verticalDemand = max(0.0, ...
    1.0 + referenceAccelerationF(3) ./ context.gravity_mps2);
thrustLoad = previousCommand .* (1.0 + payloadFraction) .* verticalDemand;
features = [payloadFraction; airF; speed; dragT; dragN; turn; ...
    referenceAccelerationF; tilt; previousCommand; thrustLoad; history(2:4)];
if numel(features) ~= 17 || any(~isfinite(features))
    error("gpenmpcBuildAeroF17Features:Features", ...
        "AERO_PHYSICS_F17 must contain 17 finite causal features.");
end
end
