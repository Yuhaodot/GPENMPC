function result = runNativeSe3PlantPowerTests(sampleCountRequested)
%RUNNATIVESE3PLANTPOWERTESTS Focused parity tests for shared closed-loop dynamics.

arguments
    sampleCountRequested (1,1) double {mustBeInteger,mustBePositive} = 500
end

thisFile = string(mfilename("fullpath"));
stageRoot = fileparts(fileparts(thisFile));
dynamicsRoot = fullfile(stageRoot, "matlab", "dynamics");
addpath(dynamicsRoot);

inputPath = gpenmpcExternalPath("matched_closed_loop_input");
oraclePath = gpenmpcExternalPath("matched_closed_loop_reference");
assert(isfile(inputPath) && isfile(oraclePath), ...
    "The read-only matched replay evidence is unavailable.");

data = load(inputPath);
oracle = load(oraclePath);
mission = jsondecode(localText(data.mission_json));
robustEnvelope = jsondecode(localText(data.robust_configuration_json));
calibration = jsondecode(localText(data.platform_calibration_json));
profile = jsondecode(localText(data.platform_profile_json));
robust = robustEnvelope.robust_control;

allocation = gpenmpcM600Allocation(calibration);
assert(isequal(size(allocation.matrix), [4, 6]));
assert(rank(allocation.matrix) == 4);
assert(norm(allocation.matrix * allocation.pseudoinverse - eye(4), "fro") < 1e-12);

referenceZero = struct("position_m", zeros(3, 1), ...
    "velocity_mps", zeros(3, 1), ...
    "acceleration_mps2", zeros(3, 1), ...
    "jerk_mps3", zeros(3, 1));
nominalMassKg = double(profile.mass_properties.base_mass_kg);
hoverPerRotorN = nominalMassKg .* 9.80665 ./ 6.0;
hoverState = [zeros(6, 1); 1; zeros(6, 1); ...
    repmat(hoverPerRotorN, 6, 1)];
[hoverCommand, hoverControl] = gpenmpcRobustSe3Control(hoverState, ...
    referenceZero, 0.0, zeros(2, 1), zeros(3, 1), calibration, profile);
assert(max(abs(hoverCommand - hoverPerRotorN)) < 1e-11);
assert(norm(hoverControl.attitude_error) < 1e-14);

nominalMission = localNominalMission(mission);
[hoverDerivative, hoverPlant] = gpenmpcM600SixDofPlantDerivative( ...
    hoverState, hoverCommand, referenceZero, 0.0, zeros(2, 1), 0.0, ...
    nominalMission, calibration, profile);
assert(norm(hoverDerivative) < 1e-10);
assert(norm(hoverPlant.actual_acceleration_mps2) < 1e-10);

[lightPower, lightPowerDiagnostic] = gpenmpcControllerSensitivePower( ...
    profile, calibration, 0.0, hoverState, referenceZero, zeros(2, 1), zeros(2, 1));
[heavyPower, ~] = gpenmpcControllerSensitivePower( ...
    profile, calibration, 4.54, hoverState, referenceZero, zeros(2, 1), zeros(2, 1));
assert(heavyPower > lightPower);
assert(lightPowerDiagnostic.power_domain_status_code <= uint8(1));

testPower = [10; 20; 30; 100; 120];
testTime = [0; 1; 2; 5; 6];
testSegment = [1; 1; 1; 2; 2];
[testEnergy, cumulative] = gpenmpcIntegrateControllerSensitiveEnergy( ...
    testPower, testTime, testSegment);
assert(abs(testEnergy - 150.0) < 1e-12);
assert(abs(cumulative(end) - 150.0) < 1e-12);

sampleCount = min(sampleCountRequested, numel(data.leg_index));
[parity, reconstructed] = localReplayPrefix(data, mission, robust, ...
    calibration, profile, sampleCount);
expected = oracle.matlabTrace;

parity.position_m = max(abs(reconstructed.position_m ...
    - double(expected.position_m(1:sampleCount, :))), [], "all");
parity.velocity_mps = max(abs(reconstructed.velocity_mps ...
    - double(expected.velocity_mps(1:sampleCount, :))), [], "all");
parity.quaternion_wxyz = max(abs(reconstructed.quaternion_wxyz ...
    - double(expected.quaternion_wxyz(1:sampleCount, :))), [], "all");
parity.body_rate_rad_s = max(abs(reconstructed.body_rate_rad_s ...
    - double(expected.body_rate_rad_s(1:sampleCount, :))), [], "all");
parity.per_rotor_thrust_n = max(abs(reconstructed.per_rotor_thrust_n ...
    - double(expected.per_rotor_thrust_n(1:sampleCount, :))), [], "all");
parity.rotor_command_n = max(abs(reconstructed.rotor_command_n ...
    - double(expected.rotor_command_n(1:sampleCount, :))), [], "all");
parity.desired_force_projected_n = max(abs(reconstructed.desired_force_projected_n ...
    - double(expected.desired_force_projected_n(1:sampleCount, :))), [], "all");
parity.attitude_error_rad = max(abs(reconstructed.attitude_error_rad ...
    - double(expected.attitude_error_rad(1:sampleCount, :))), [], "all");
parity.actual_acceleration_mps2 = max(abs(reconstructed.actual_acceleration_mps2 ...
    - double(expected.actual_acceleration_mps2(1:sampleCount, :))), [], "all");
parity.robust_acceleration_i_mps2 = max(abs(reconstructed.robust_acceleration_i_mps2 ...
    - double(expected.robust_acceleration_i_mps2(1:sampleCount, :))), [], "all");
parity.controller_sensitive_power_w = max(abs(reconstructed.controller_sensitive_power_w ...
    - double(expected.controller_sensitive_power_w(1:sampleCount))), [], "all");

names = fieldnames(parity);
for index = 1:numel(names)
    assert(parity.(names{index}) <= 5e-10, ...
        "Parent matched-replay parity failed for %s: %.17g", ...
        names{index}, parity.(names{index}));
end

[prefixEnergy, ~] = gpenmpcIntegrateControllerSensitiveEnergy( ...
    reconstructed.controller_sensitive_power_w, ...
    double(data.global_time_s(1:sampleCount)), ...
    double(data.leg_index(1:sampleCount)));
[oracleEnergy, ~] = gpenmpcIntegrateControllerSensitiveEnergy( ...
    double(expected.controller_sensitive_power_w(1:sampleCount)), ...
    double(data.global_time_s(1:sampleCount)), ...
    double(data.leg_index(1:sampleCount)));
assert(abs(prefixEnergy - oracleEnergy) < 1e-8);

sourceFiles = dir(fullfile(dynamicsRoot, "*.m"));
issues = checkcode(fullfile({sourceFiles.folder}, {sourceFiles.name}), "-id");
assert(sum(cellfun(@numel, issues)) == 0, ...
    "MATLAB Code Analyzer reported issues in modular dynamics sources.");

result = struct;
result.status = "MATLAB_NATIVE_SHARED_SE3_PLANT_POWER_TESTS_PASS";
result.check_count = 12 + numel(names);
result.parent_parity_sample_count = sampleCount;
result.parent_parity_max_abs = max(cell2mat(struct2cell(parity)));
result.hover_derivative_norm = norm(hoverDerivative);
result.hover_rotor_command_n = hoverPerRotorN;
result.light_payload_power_w = lightPower;
result.heavy_payload_power_w = heavyPower;
result.prefix_energy_difference_j = prefixEnergy - oracleEnergy;
result.shared_between_b1_and_b2 = true;

fprintf(['%s|checks=%d|samples=%d|max_abs=%.3e|hover_dx=%.3e|' ...
    'energy_diff=%.3e\n'], result.status, result.check_count, ...
    sampleCount, result.parent_parity_max_abs, result.hover_derivative_norm, ...
    result.prefix_energy_difference_j);
end


function [parity, output] = localReplayPrefix(data, mission, robust, ...
        calibration, profile, sampleCount)
parity = struct;
time = double(data.global_time_s(:));
leg = double(data.leg_index(:));
dt = double(mission.simulation.sample_period_s);
windDuration = double(data.actual_wind_estimated_total_duration_s);

output = struct;
fields3 = ["position_m", "velocity_mps", "body_rate_rad_s", ...
    "desired_force_projected_n", "attitude_error_rad", ...
    "actual_acceleration_mps2", "robust_acceleration_i_mps2"];
for field = fields3
    output.(field) = zeros(sampleCount, 3);
end
output.quaternion_wxyz = zeros(sampleCount, 4);
output.per_rotor_thrust_n = zeros(sampleCount, 6);
output.rotor_command_n = zeros(sampleCount, 6);
output.controller_sensitive_power_w = zeros(sampleCount, 1);

state = [double(data.position_m(1, :)).'; ...
    double(data.velocity_mps(1, :)).'; ...
    double(data.quaternion_wxyz(1, :)).'; ...
    double(data.body_rate_rad_s(1, :)).'; ...
    double(data.per_rotor_thrust_n(1, :)).'];
robustState = gpenmpcInitializeRobustSe3State();

for sample = 1:sampleCount
    if sample > 1 && leg(sample) ~= leg(sample - 1)
        state = [double(data.position_m(sample, :)).'; ...
            double(data.velocity_mps(sample, :)).'; ...
            double(data.quaternion_wxyz(sample, :)).'; ...
            double(data.body_rate_rad_s(sample, :)).'; ...
            double(data.per_rotor_thrust_n(sample, :)).'];
    end
    reference = localReference(data, sample);
    payload = double(data.payload_kg(sample));
    windEstimate = double(data.wind_estimate_xy_mps(sample, :)).';
    actualWind = double(data.actual_wind_xy_mps(sample, :)).';
    [augmentation, robustState] = gpenmpcRobustSe3Augmentation( ...
        state, reference, robust, robustState, dt);
    [derivative, diagnostic] = gpenmpcM600ClosedLoopDerivative( ...
        state, reference, payload, actualWind, windEstimate, time(sample), ...
        augmentation, mission, calibration, profile);
    power = gpenmpcControllerSensitivePower(profile, calibration, payload, ...
        state, reference, actualWind, windEstimate);

    output.position_m(sample, :) = state(1:3).';
    output.velocity_mps(sample, :) = state(4:6).';
    output.quaternion_wxyz(sample, :) = ...
        (state(7:10) ./ norm(state(7:10))).';
    output.body_rate_rad_s(sample, :) = state(11:13).';
    output.per_rotor_thrust_n(sample, :) = state(14:19).';
    output.rotor_command_n(sample, :) = diagnostic.rotor_command_n.';
    output.desired_force_projected_n(sample, :) = ...
        diagnostic.desired_force_projected_n.';
    output.attitude_error_rad(sample, :) = diagnostic.attitude_error.';
    output.actual_acceleration_mps2(sample, :) = ...
        diagnostic.actual_acceleration_mps2.';
    output.robust_acceleration_i_mps2(sample, :) = augmentation.';
    output.controller_sensitive_power_w(sample) = power;
    robustState = gpenmpcRobustSe3ObserveControl(robustState, diagnostic, dt);

    if sample < sampleCount && leg(sample + 1) == leg(sample)
        step = time(sample + 1) - time(sample);
        midReference = gpenmpcReferenceExtrapolation(reference, 0.5 .* step);
        nextReference = gpenmpcReferenceExtrapolation(reference, step);
        midTime = time(sample) + 0.5 .* step;
        nextTime = time(sample) + step;
        midActualWind = gpenmpcActualWindAtTime(mission, windDuration, midTime);
        nextActualWind = gpenmpcActualWindAtTime(mission, windDuration, nextTime);
        k1 = derivative;
        k2 = gpenmpcM600ClosedLoopDerivative(state + 0.5 .* step .* k1, ...
            midReference, payload, midActualWind, windEstimate, midTime, ...
            augmentation, mission, calibration, profile);
        k3 = gpenmpcM600ClosedLoopDerivative(state + 0.5 .* step .* k2, ...
            midReference, payload, midActualWind, windEstimate, midTime, ...
            augmentation, mission, calibration, profile);
        k4 = gpenmpcM600ClosedLoopDerivative(state + step .* k3, ...
            nextReference, payload, nextActualWind, windEstimate, nextTime, ...
            augmentation, mission, calibration, profile);
        state = state + step .* (k1 + 2 .* k2 + 2 .* k3 + k4) ./ 6.0;
        state(7:10) = state(7:10) ./ max(norm(state(7:10)), 1e-15);
    end
end
end


function reference = localReference(data, sample)
reference = struct("position_m", double(data.reference_position_m(sample, :)).', ...
    "velocity_mps", double(data.reference_velocity_mps(sample, :)).', ...
    "acceleration_mps2", double(data.reference_acceleration_mps2(sample, :)).', ...
    "jerk_mps3", double(data.reference_jerk_mps3(sample, :)).');
end


function mission = localNominalMission(mission)
mismatch = mission.plant_mismatch;
mismatch.mass_bias_kg = 0.0;
mismatch.drag_scale_xyz = ones(3, 1);
mismatch.cross_drag_matrix_n_per_mps2 = zeros(3, 3);
mismatch.thrust_effectiveness_by_rotor = ones(6, 1);
mismatch.external_acceleration_amplitude_mps2 = 0.0;
mismatch.external_acceleration_frequencies_hz = zeros(3, 1);
mismatch.external_acceleration_phases_rad = zeros(3, 1);
mismatch.acceleration_bias_inertial_mps2 = zeros(3, 1);
mission.plant_mismatch = mismatch;
mission.structured_residual.enabled = false;
end


function text = localText(value)
if iscell(value)
    value = value{1};
end
text = char(value(:).');
end
