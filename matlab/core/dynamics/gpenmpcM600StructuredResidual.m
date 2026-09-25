function structured = gpenmpcM600StructuredResidual(plantState, reference, ...
        payloadKg, actualAirMps, trueMassKg, mission, profile)
%GPENMPCM600STRUCTUREDRESIDUAL Software-only structured plant mismatch.
%
% This common plant mismatch supports the B1/B2 comparison. Controllers use
% the causal observable state while the structured residual remains a
% plant-side disturbance.

spec = mission.structured_residual;
zero = zeros(3, 1);
if ~logical(spec.enabled)
    structured = struct("drag", zero, "turn", zero, "thrust", zero);
    return
end

x = double(plantState(:));
referenceVelocity = double(reference.velocity_mps(:));
referenceAcceleration = double(reference.acceleration_mps2(:));
velocityXy = referenceVelocity(1:2);
speed = norm(velocityXy);
if speed >= 0.75
    tangentXy = velocityXy ./ speed;
else
    airXy = actualAirMps(1:2);
    tangentXy = airXy ./ max(norm(airXy), 1e-12);
end
normalXy = [-tangentXy(2); tangentXy(1)];
tangentI = [tangentXy; 0];
normalI = [normalXy; 0];
rotation = gpenmpcQuaternionRotation(x(7:10));
tilt = acos(min(max(rotation(3, 3), -1), 1));
payloadFraction = min(max(double(payloadKg) ./ 4.54, 0), 1);
payloadFactor = 0.35 + 0.65 .* payloadFraction;

dragCoefficient = double(spec.drag_payload_coupling_n_per_mps2_xyz(:));
tiltFactor = 1.0 + double(spec.drag_tilt_gain) .* sin(tilt).^2;
drag = -dragCoefficient .* payloadFactor .* tiltFactor ...
    .* actualAirMps .* abs(actualAirMps) ./ max(trueMassKg, 1e-12);

airT = dot(actualAirMps, tangentI);
airN = dot(actualAirMps, normalI);
sideForce = -double(spec.turn_sideforce_n_per_mps2) ...
    .* payloadFactor .* airN .* abs(airT);
signedCentripetal = 0.0;
if speed >= 0.75
    signedCentripetal = (velocityXy(1) .* referenceAcceleration(2) ...
        - velocityXy(2) .* referenceAcceleration(1)) ./ max(speed, 1e-12);
end
turnAcceleration = sideForce ./ max(trueMassKg, 1e-12) ...
    - double(spec.turn_centripetal_gain) .* payloadFactor ...
    .* signedCentripetal .* (0.25 + 0.75 .* abs(sin(tilt)));
turn = turnAcceleration .* normalI;

nominalMassKg = double(profile.mass_properties.base_mass_kg) + double(payloadKg);
totalThrustN = sum(x(14:19));
normalizedCommand = totalThrustN ./ max(nominalMassKg .* 9.80665, 1e-12);
commandDelta = normalizedCommand - 0.90;
smoothPositive = 0.5 .* (commandDelta ...
    + sqrt(commandDelta.^2 + 0.01.^2));
verticalDemand = abs(referenceAcceleration(3)) ./ 9.80665;
efficiencyLoss = double(spec.thrust_efficiency_base_loss) ...
    + double(spec.thrust_efficiency_payload_gain) .* payloadFraction ...
    + double(spec.thrust_efficiency_command_gain) .* smoothPositive ...
    + double(spec.thrust_efficiency_vertical_demand_gain) .* verticalDemand;
thrust = -efficiencyLoss .* totalThrustN ./ max(trueMassKg, 1e-12) ...
    .* rotation(:, 3);

structured = struct("drag", drag, "turn", turn, "thrust", thrust);
end
