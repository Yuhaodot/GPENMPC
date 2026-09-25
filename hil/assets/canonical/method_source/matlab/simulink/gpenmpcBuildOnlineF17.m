function [features, frameIFromF] = gpenmpcBuildOnlineF17( ...
        state, reference, environment, residualHistoryF, lastTangentXY)
%GPENMPCBUILDONLINEF17 Build the frozen 17-dimensional causal GP input.

gravity = 9.80665;
maximumPayloadKg = 4.54;
velocity = double(state(4:6));
quaternion = double(state(7:10));
quaternion = quaternion ./ max(norm(quaternion), 1.0e-15);
rotorThrust = double(state(14:19));
referenceVelocity = double(reference(4:6));
referenceAcceleration = double(reference(7:9));
windEstimate = [double(environment(1:2)); 0.0];
payloadKg = double(environment(5));

horizontal = referenceVelocity(1:2);
if norm(horizontal) >= 0.75
    tangent = horizontal ./ norm(horizontal);
else
    tangent = double(lastTangentXY(:));
    tangent = tangent ./ max(norm(tangent), 1.0e-15);
end
normal = [-tangent(2); tangent(1)];
frameIFromF = [tangent(1), normal(1), 0; ...
    tangent(2), normal(2), 0; 0, 0, 1];
estimatedAirF = frameIFromF.' * (velocity - windEstimate);
referenceAccelerationF = frameIFromF.' * referenceAcceleration;
payloadFraction = payloadKg ./ maximumPayloadKg;
rotation = quaternionRotation(quaternion);
tilt = acos(min(max(rotation(3, 3), -1.0), 1.0));
tiltFactor = 1.0 + 0.35 .* sin(tilt).^2;
airSpeed = norm(estimatedAirF);
dragT = payloadFraction .* tiltFactor .* estimatedAirF(1) .* abs(estimatedAirF(1));
dragN = payloadFraction .* tiltFactor .* estimatedAirF(2) .* abs(estimatedAirF(2));
speed = norm(horizontal);
curvature = 0.0;
if speed >= 0.75
    curvature = abs(referenceVelocity(1) .* referenceAcceleration(2) ...
        - referenceVelocity(2) .* referenceAcceleration(1)) ./ max(speed.^3, 1.0e-12);
end
turnCrossflow = curvature .* estimatedAirF(1) .* abs(estimatedAirF(1)) ...
    + estimatedAirF(2) .* abs(estimatedAirF(1));
nominalMass = 9.5 + payloadKg;
previousCommand = min(max(sum(rotorThrust) ./ max(nominalMass .* gravity, 1.0e-12), ...
    0.0), 2.0);
verticalDemand = max(0.0, 1.0 + referenceAccelerationF(3) ./ gravity);
thrustPayloadDemand = previousCommand .* (1.0 + payloadFraction) .* verticalDemand;

features = [payloadFraction, estimatedAirF.', airSpeed, dragT, dragN, ...
    turnCrossflow, referenceAccelerationF.', tilt, previousCommand, ...
    thrustPayloadDemand, double(residualHistoryF(:).')];
end

function R = quaternionRotation(q)
w=q(1); x=q(2); y=q(3); z=q(4);
R=[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w); ...
   2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w); ...
   2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)];
end

