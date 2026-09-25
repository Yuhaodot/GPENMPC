function [powerW, diagnostic] = gpenmpcM600ControllerSensitivePower( ...
        profile, calibration, payloadKg, plantState, reference, ...
        actualWindXyMps, windEstimateXyMps)
%GPENMPCM600CONTROLLERSENSITIVEPOWER Rotor-sensitive native 6DoF power model.
%
% Combines the phase-power table at the actual airspeed with the induced-
% power difference between the six rotor thrusts and a uniform allocation
% of the nominal reference force. The reference force uses the estimated
% wind, nominal drag and reference acceleration.

x = double(plantState(:));
airspeed = norm(x(4:6) - [double(actualWindXyMps(:)); 0]);
[publicPowerW, statusCode] = gpenmpcM600PhasePower( ...
    profile, payloadKg, airspeed, x(6));

allocation = gpenmpcM600Allocation(calibration);
referenceAir = double(reference.velocity_mps(:)) ...
    - [double(windEstimateXyMps(:)); 0];
referenceDrag = allocation.nominal_drag_n_per_mps2 ...
    .* referenceAir .* abs(referenceAir);
nominalMassKg = double(profile.mass_properties.base_mass_kg) + double(payloadKg);
nominalReferenceForce = nominalMassKg ...
    .* (double(reference.acceleration_mps2(:)) + [0; 0; 9.80665]) ...
    + referenceDrag;

diameterM = double(profile.rotor_system.diameter_m);
areaM2 = pi .* (0.5 .* diameterM).^2;
inducedDenominator = sqrt(2.0 .* 1.225 .* areaM2);
actualInducedW = sum(max(x(14:19), 0).^1.5) ./ inducedDenominator;
uniformThrustN = repmat(max(norm(nominalReferenceForce), 0) ./ 6.0, 6, 1);
nominalInducedW = sum(uniformThrustN.^1.5) ./ inducedDenominator;
powerW = max(publicPowerW + actualInducedW - nominalInducedW, 0.0);

diagnostic = struct;
diagnostic.public_power_w = publicPowerW;
diagnostic.actual_induced_power_w = actualInducedW;
diagnostic.nominal_induced_power_w = nominalInducedW;
diagnostic.airspeed_mps = airspeed;
diagnostic.power_domain_status_code = statusCode;
end
