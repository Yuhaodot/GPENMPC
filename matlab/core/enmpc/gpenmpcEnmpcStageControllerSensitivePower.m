function powerW = gpenmpcEnmpcStageControllerSensitivePower( ...
        publicPowerW, rotorThrustN, nominalReferenceTotalThrustN, ...
        rotorDiameterM, airDensityKgM3)
%GPENMPCENMPCSTAGECONTROLLERSENSITIVEPOWER Rotor-sensitive eNMPC stage power.
%
% Adds the induced-power difference between the six candidate rotor thrusts
% and a uniform allocation of nominal reference thrust to the phase-table
% power, using the specified rotor diameter and air density.

arguments
    publicPowerW (1,1) double
    rotorThrustN (1,6) double
    nominalReferenceTotalThrustN (1,1) double
    rotorDiameterM (1,1) double
    airDensityKgM3 (1,1) double = 1.225
end
diskArea = pi .* (0.5 .* rotorDiameterM).^2;
denominator = sqrt(2.0 .* airDensityKgM3 .* diskArea);
actualPower = sum(max(rotorThrustN, 0.0).^1.5, 2) ./ denominator;
uniformNominal = max(nominalReferenceTotalThrustN, 0.0) ./ 6.0;
nominalPower = 6.0 .* uniformNominal.^1.5 ./ denominator;
powerW = max(publicPowerW + actualPower - nominalPower, 0.0);
end
