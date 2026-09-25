function gpenmpcNativeInnerPlantSfun(block)
%GPENMPCNATIVEINNERPLANTSFUN Robust SE(3), allocator and 6DoF software plant.
setup(block);
end

function setup(block)
block.NumDialogPrms = 0;
block.NumInputPorts = 2;
block.NumOutputPorts = 7;
inputWidths = [12, 5];
outputWidths = [19, 9, 3, 4, 6, 1, 3];
for index = 1:2
    block.InputPort(index).Dimensions = inputWidths(index);
    block.InputPort(index).DatatypeID = 0;
    block.InputPort(index).Complexity = "Real";
    block.InputPort(index).DirectFeedthrough = true;
    block.InputPort(index).SamplingMode = "Sample";
end
for index = 1:7
    block.OutputPort(index).Dimensions = outputWidths(index);
    block.OutputPort(index).DatatypeID = 0;
    block.OutputPort(index).Complexity = "Real";
    block.OutputPort(index).SamplingMode = "Sample";
end
block.SampleTimes = [0.01, 0];
block.SimStateCompliance = "DefaultSimState";
block.RegBlockMethod("PostPropagationSetup", @postPropagationSetup);
block.RegBlockMethod("InitializeConditions", @initialize);
block.RegBlockMethod("Outputs", @outputs);
block.RegBlockMethod("Update", @update);
end

function postPropagationSetup(block)
block.NumDworks = 14;
names = ["state", "robust_filtered", "robust_authority", "robust_tangent", ...
    "vertical_observer_estimate", "vertical_observer_previous_velocity_z", ...
    "vertical_observer_previous_nominal_z", "vertical_observer_previous_time", ...
    "vertical_observer_previous_leg", "vertical_observer_previous_payload", ...
    "vertical_observer_valid", "vertical_observer_update_enabled", ...
    "vertical_observer_reset_count", "vertical_observer_hold_count"];
widths = [19, 3, 1, 2, ones(1, 10)];
for index = 1:4
    block.Dwork(index).Name = names(index);
    block.Dwork(index).Dimensions = widths(index);
    block.Dwork(index).DatatypeID = 0;
    block.Dwork(index).Complexity = "Real";
    block.Dwork(index).UsedAsDiscState = true;
end
end

function initialize(block)
assets = gpenmpcNativeHarnessAssets();
state = zeros(19, 1);
state(3) = 1.5;
state(7) = 1.0;
payload = 1.20;
mass = double(assets.profile.mass_properties.base_mass_kg) + payload;
state(14:19) = mass .* 9.80665 ./ 6.0;
block.Dwork(1).Data = state;
robustState = gpenmpcInitializeRobustSe3State();
robustState = gpenmpcResetCausalVerticalDisturbanceObserver( ...
    robustState, state(4:6), 0.0, 1.0, payload);
setRobustState(block, robustState);
end

function outputs(block)
assets = gpenmpcNativeHarnessAssets();
state = block.Dwork(1).Data;
reference = vectorReference(block.InputPort(1).Data);
environment = double(block.InputPort(2).Data(:));
robustState = getRobustState(block);
[augmentation, ~] = gpenmpcRobustSe3Augmentation( ...
    state, reference, assets.robust, robustState, assets.sample_period_s);
[rotorCommand, control] = gpenmpcRobustSe3Control(state, reference, ...
    environment(5), environment(1:2), augmentation, ...
    assets.calibration, assets.profile);
[~, plant] = gpenmpcM600SixDofPlantDerivative(state, rotorCommand, reference, ...
    environment(5), environment(3:4), block.CurrentTime, assets.mission, ...
    assets.calibration, assets.profile);
power = gpenmpcM600ControllerSensitivePower(assets.profile, assets.calibration, ...
    environment(5), state, reference, environment(3:4), environment(1:2));

block.OutputPort(1).Data = state;
block.OutputPort(2).Data = robustDiagnostic(augmentation, control, robustState, ...
    state, reference);
block.OutputPort(3).Data = control.desired_force_projected_n(:);
block.OutputPort(4).Data = [control.desired_thrust_n; control.desired_moment_nm(:)];
block.OutputPort(5).Data = rotorCommand(:);
block.OutputPort(6).Data = power;
block.OutputPort(7).Data = plant.actual_acceleration_mps2(:);
end

function update(block)
assets = gpenmpcNativeHarnessAssets();
dt = assets.sample_period_s;
state = block.Dwork(1).Data;
reference = vectorReference(block.InputPort(1).Data);
environment = double(block.InputPort(2).Data(:));
robustState = getRobustState(block);
[robustState, ~] = gpenmpcObserveCausalVerticalDisturbance( ...
    robustState, state(4:6), block.CurrentTime, 1.0, environment(5), assets.robust);
[augmentation, robustState] = gpenmpcRobustSe3Augmentation( ...
    state, reference, assets.robust, robustState, dt);
[rotorCommand, control] = gpenmpcRobustSe3Control(state, reference, ...
    environment(5), environment(1:2), augmentation, ...
    assets.calibration, assets.profile);
robustState = gpenmpcRobustSe3ObserveControl(robustState, control, dt);
nominalAcceleration = gpenmpcKnownNominalAccelerationFromObservation( ...
    state, environment(1:2), environment(5), assets.calibration, assets.profile);
robustState = gpenmpcCommitCausalVerticalDisturbanceObserver( ...
    robustState, state(4:6), nominalAcceleration, block.CurrentTime, ...
    1.0, environment(5));

actualWind = environment(3:4);
time = block.CurrentTime;
k1 = gpenmpcM600SixDofPlantDerivative(state, rotorCommand, reference, ...
    environment(5), actualWind, time, assets.mission, assets.calibration, assets.profile);
k2 = gpenmpcM600SixDofPlantDerivative(state + 0.5 .* dt .* k1, rotorCommand, ...
    extrapolateReference(reference, 0.5 .* dt), environment(5), actualWind, ...
    time + 0.5 .* dt, assets.mission, assets.calibration, assets.profile);
k3 = gpenmpcM600SixDofPlantDerivative(state + 0.5 .* dt .* k2, rotorCommand, ...
    extrapolateReference(reference, 0.5 .* dt), environment(5), actualWind, ...
    time + 0.5 .* dt, assets.mission, assets.calibration, assets.profile);
k4 = gpenmpcM600SixDofPlantDerivative(state + dt .* k3, rotorCommand, ...
    extrapolateReference(reference, dt), environment(5), actualWind, ...
    time + dt, assets.mission, assets.calibration, assets.profile);
state = state + dt .* (k1 + 2 .* k2 + 2 .* k3 + k4) ./ 6.0;
state(7:10) = state(7:10) ./ max(norm(state(7:10)), 1.0e-15);
block.Dwork(1).Data = state;
setRobustState(block, robustState);
end

function reference = vectorReference(value)
value = double(value(:));
reference = struct("position_m", value(1:3), "velocity_mps", value(4:6), ...
    "acceleration_mps2", value(7:9), "jerk_mps3", value(10:12));
end

function reference = extrapolateReference(reference, horizon)
reference = struct( ...
    "position_m", reference.position_m + horizon .* reference.velocity_mps ...
        + 0.5 .* horizon.^2 .* reference.acceleration_mps2 ...
        + horizon.^3 ./ 6.0 .* reference.jerk_mps3, ...
    "velocity_mps", reference.velocity_mps + horizon .* reference.acceleration_mps2 ...
        + 0.5 .* horizon.^2 .* reference.jerk_mps3, ...
    "acceleration_mps2", reference.acceleration_mps2 + horizon .* reference.jerk_mps3, ...
    "jerk_mps3", reference.jerk_mps3);
end

function state = getRobustState(block)
state = struct("filtered_compensation_i_mps2", block.Dwork(2).Data, ...
    "authority_scale", block.Dwork(3).Data, ...
    "last_tangent_xy", block.Dwork(4).Data, ...
    "vertical_disturbance_ewma_mps2", block.Dwork(5).Data, ...
    "vertical_observer_previous_velocity_z_mps", block.Dwork(6).Data, ...
    "vertical_observer_previous_nominal_acceleration_z_mps2", block.Dwork(7).Data, ...
    "vertical_observer_previous_time_s", block.Dwork(8).Data, ...
    "vertical_observer_previous_leg_index", block.Dwork(9).Data, ...
    "vertical_observer_previous_payload_kg", block.Dwork(10).Data, ...
    "vertical_observer_observation_valid", block.Dwork(11).Data > 0.5, ...
    "vertical_observer_update_enabled", block.Dwork(12).Data > 0.5, ...
    "vertical_observer_reset_count", block.Dwork(13).Data, ...
    "vertical_observer_antiwindup_hold_count", block.Dwork(14).Data);
end

function setRobustState(block, state)
block.Dwork(2).Data = state.filtered_compensation_i_mps2(:);
block.Dwork(3).Data = state.authority_scale;
block.Dwork(4).Data = state.last_tangent_xy(:);
block.Dwork(5).Data = state.vertical_disturbance_ewma_mps2;
block.Dwork(6).Data = state.vertical_observer_previous_velocity_z_mps;
block.Dwork(7).Data = state.vertical_observer_previous_nominal_acceleration_z_mps2;
block.Dwork(8).Data = state.vertical_observer_previous_time_s;
block.Dwork(9).Data = state.vertical_observer_previous_leg_index;
block.Dwork(10).Data = state.vertical_observer_previous_payload_kg;
block.Dwork(11).Data = double(state.vertical_observer_observation_valid);
block.Dwork(12).Data = double(state.vertical_observer_update_enabled);
block.Dwork(13).Data = state.vertical_observer_reset_count;
block.Dwork(14).Data = state.vertical_observer_antiwindup_hold_count;
end

function vector = robustDiagnostic(augmentation, control, state, plantState, reference)
positionError = double(reference.position_m(:)) - double(plantState(1:3));
velocityError = double(reference.velocity_mps(:)) - double(plantState(4:6));
vector = [augmentation(:); norm(positionError); ...
    norm(velocityError); norm(control.attitude_error); ...
    state.authority_scale; double(control.force_projection_norm_mismatch_n); ...
    double(control.rotor_saturated)];
end
