function [derivative, diagnostic] = gpenmpcM600ClosedLoopDerivative( ...
        plantState, reference, payloadKg, actualWindXyMps, ...
        windEstimateXyMps, globalTimeS, augmentationIMps2, ...
        mission, calibration, profile, attitudeCommand)
%GPENMPCM600CLOSEDLOOPDERIVATIVE Compose common controller and software plant.

if nargin < 11
    attitudeCommand = [];
end
[rotorCommandN, control] = gpenmpcRobustSe3Control(plantState, reference, ...
    payloadKg, windEstimateXyMps, augmentationIMps2, calibration, profile, ...
    attitudeCommand);
[derivative, plant] = gpenmpcM600SixDofPlantDerivative(plantState, ...
    rotorCommandN, reference, payloadKg, actualWindXyMps, globalTimeS, ...
    mission, calibration, profile);

% The causal F17 learner is trained against the one-step residual of this
% known nominal model.  The shared helper is also used by the online Simulink
% path, which prevents the two runtime paths from drifting in label semantics.
nominalAcceleration = gpenmpcKnownNominalAccelerationFromObservation( ...
    plantState, windEstimateXyMps, payloadKg, calibration, profile);

diagnostic = control;
diagnostic.actual_acceleration_mps2 = plant.actual_acceleration_mps2;
diagnostic.nominal_acceleration_mps2 = nominalAcceleration;
diagnostic.residual_acceleration_mps2 = ...
    plant.actual_acceleration_mps2 - nominalAcceleration;
diagnostic.actual_airspeed_mps = plant.actual_airspeed_mps;
diagnostic.actual_air_velocity_mps = plant.actual_air_velocity_mps;
diagnostic.true_drag_n = plant.true_drag_n;
diagnostic.true_wrench = plant.true_wrench;
diagnostic.structured_acceleration_mps2 = ...
    plant.structured_acceleration_mps2;
diagnostic.fast_acceleration_mps2 = plant.fast_acceleration_mps2;
diagnostic.true_mass_kg = plant.true_mass_kg;
end
