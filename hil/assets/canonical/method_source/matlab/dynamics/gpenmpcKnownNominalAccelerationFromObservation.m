function nominalAcceleration = gpenmpcKnownNominalAccelerationFromObservation( ...
        plantState, windEstimateXyMps, payloadKg, calibration, profile)
%GPENMPCKNOWNNOMINALACCELERATIONFROMOBSERVATION Causal nominal-model output.
%
% Uses the observed state, retained rotor thrusts, wind estimate and payload
% to evaluate nominal force and acceleration.

x = double(plantState(:));
if numel(x) ~= 19
    error("gpenmpcKnownNominalAccelerationFromObservation:State", ...
        "The M600 state must contain 19 values.");
end
windEstimate = double(windEstimateXyMps(:));
if numel(windEstimate) ~= 2
    error("gpenmpcKnownNominalAccelerationFromObservation:Wind", ...
        "The wind estimate must contain two horizontal components.");
end
allocation = gpenmpcM600Allocation(calibration);
rotation = gpenmpcQuaternionRotation(x(7:10));
nominalMassKg = double(profile.mass_properties.base_mass_kg) ...
    + double(payloadKg);
estimatedAir = x(4:6) - [windEstimate; 0];
nominalDrag = allocation.nominal_drag_n_per_mps2 ...
    .* estimatedAir .* abs(estimatedAir);
nominalWrench = allocation.matrix * x(14:19);
nominalAcceleration = (rotation * [0; 0; nominalWrench(1)] ...
    - nominalDrag) ./ nominalMassKg - [0; 0; 9.80665];
end
