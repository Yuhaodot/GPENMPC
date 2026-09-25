function result = runNativeOrdinaryB2RolloutParity(projectRoot)
%RUNNATIVEORDINARYB2ROLLOUTPARITY Compare fixed parent-Python and MATLAB rollout.

arguments
    projectRoot (1,1) string
end
enmpcRoot = fullfile(projectRoot, "matlab", "enmpc");
inferenceRoot = fullfile(projectRoot, "matlab", "inference");
dataRoot = fullfile(projectRoot, "matlab", "data");
addpath(enmpcRoot, inferenceRoot, dataRoot);
cleanup = onCleanup(@() rmpath(enmpcRoot, inferenceRoot, dataRoot));
fixturePath = fullfile(projectRoot, "tests", "fixtures", ...
    "ordinary_b2_rollout_python_reference.json");
fixture = jsondecode(fileread(fixturePath));
modelPath = fullfile(projectRoot, "matlab", "native_training", ...
    "raw", "MATLAB_NATIVE_SPARSE_GP_MODEL.mat");
stored = load(modelPath, "nativeModel");
config = gpenmpcLoadOrdinaryB2Config(string(fixture.runtime_config_path));
trajectory = struct( ...
    "total_duration_s", double(fixture.trajectory.total_duration_s), ...
    "coefficients_ascending", double(fixture.trajectory.coefficients_ascending));
phaseState = fixture.phase_state;
observation = fixture.observation;
context = fixture.context;
context.phase_power_fcn = @fixedPower;
causal = fixture.causal_context_f17;
decision = double(fixture.decision(:).');
b1 = gpenmpcEvaluateOrdinaryB2Rollout(decision, config.b1_fallback_method, ...
    trajectory, phaseState, observation, context, struct, ...
    struct("valid", false, "values", zeros(4, 1)), config);
b2 = gpenmpcEvaluateOrdinaryB2Rollout(decision, config.method, trajectory, ...
    phaseState, observation, context, stored.nativeModel, causal, config);

tolerance = 3.0e-9;
b1Errors = compareRollout(b1, fixture.python_b1);
b2Errors = compareRollout(b2, fixture.python_b2);
assert(max(struct2array(b1Errors)) <= tolerance);
assert(max(struct2array(b2Errors)) <= tolerance);
assert(~b2.hard_invalid && ~b2.current_hard_invalid);
assert(b2.mean_trust > 0.0);
assert(config.gp_mean_scale == 1.0);
assert(config.additional_prediction_acceleration_reserve_mps2 == 0.0);
assert(~config.n1_residual_tube_enabled && ~config.b2r1_enabled);

% Invalid causal context must reduce to a same-start B1 outer solve. The
% deadline is enlarged only in this deterministic unit test to avoid host load
% affecting the decision identity.
testConfig = config;
testConfig.solver_deadline_s = 60.0;
[fallback, ~, fallbackAudit] = gpenmpcOrdinaryB2OuterStep(zeros(1, 7), ...
    trajectory, phaseState, observation, context, stored.nativeModel, ...
    struct("valid", false, "values", zeros(4, 1)), testConfig);
assert(fallback.success);
assert(fallback.gp_b1_fallback_active);
assert(fallback.hard_invalid);
assert(fallbackAudit.b1_fallback_active);
assert(fallbackAudit.b1_fallback_trigger == "GP_HARD_INVALID");
assert(fallback.method == config.method);
assert(max(abs(fallbackAudit.b1.primary_candidates(1, :))) == 0.0);
assert(isequal(fallback.decision_blocks_s_inv, ...
    fallbackAudit.b1.selected_candidate(1:config.control_blocks)));
assert(isequal(fallback.outer_acceleration_correction_f_mps2, ...
    fallbackAudit.b1.selected_candidate(config.control_blocks + (1:3))));

result = struct;
result.schema = "GPENMPC_MATLAB_NATIVE_ORDINARY_B2_ROLLOUT_PARITY_RESULT_V1";
result.status = "PASS";
result.python_reference_path = fixturePath;
result.native_model_path = modelPath;
result.tolerance = tolerance;
result.b1_errors = b1Errors;
result.b2_errors = b2Errors;
result.b1_max_error = max(struct2array(b1Errors));
result.b2_max_error = max(struct2array(b2Errors));
result.b2_mean_trust = b2.mean_trust;
result.b2_hard_invalid = b2.hard_invalid;
result.hard_invalid_exact_b1_fallback = true;
result.gp_mean_scale = config.gp_mean_scale;
result.robust_tube_identity = config.robust_tube_identity;
result.n1_or_b2r1_active = false;
reportRoot = fullfile(projectRoot, "reports");
if ~isfolder(reportRoot)
    mkdir(reportRoot);
end
resultPath = fullfile(reportRoot, ...
    "NATIVE_ORDINARY_B2_ROLLOUT_PARITY_RESULT.json");
result.result_path = resultPath;
writelines(jsonencode(result, PrettyPrint=true), resultPath, Encoding="UTF-8");
fprintf("MATLAB_NATIVE_ORDINARY_B2_ROLLOUT_PARITY_PASS b1=%.3e b2=%.3e trust=%.6f\n", ...
    result.b1_max_error, result.b2_max_error, result.b2_mean_trust);

    function powerW = fixedPower(phase, airspeed, payload)
        phaseDelta = 15.0 .* double(string(phase) == "ASCEND") ...
            + 8.0 .* double(string(phase) == "DESCEND");
        powerW = 1050.0 + 9.0 .* airspeed.^2 + 20.0 .* payload + phaseDelta;
    end
end


function errors = compareRollout(actual, expected)
errors = struct;
errors.objective = abs(actual.objective - expected.objective);
errors.constraints = max(abs(actual.constraints(:) - double(expected.constraints(:))));
errors.risk_constraints = max(abs(actual.risk_constraints_normalized(:) ...
    - double(expected.risk_constraints_normalized(:))));
errors.risk_violation = abs(actual.risk_violation - expected.risk_violation);
errors.tracking_zone = abs(actual.tracking_zone_violation ...
    - expected.tracking_zone_violation);
errors.predicted_energy = abs(actual.predicted_energy_j - expected.predicted_energy_j);
errors.terminal_energy = abs(actual.terminal_energy_to_go_j ...
    - expected.terminal_energy_to_go_j);
errors.terminal_progress = abs(actual.terminal_progress_s - expected.terminal_progress_s);
errors.mean_trust = abs(actual.mean_trust - expected.mean_trust);
errors.reference_position = rowMaximum(actual.rows, expected.rows, "reference_position_m");
errors.reference_velocity = rowMaximum(actual.rows, expected.rows, "reference_velocity_mps");
errors.reference_acceleration = rowMaximum(actual.rows, expected.rows, "reference_acceleration_mps2");
errors.reference_jerk = rowMaximum(actual.rows, expected.rows, "reference_jerk_mps3");
errors.predicted_position = rowMaximum(actual.rows, expected.rows, "predicted_position_m");
errors.predicted_velocity = rowMaximum(actual.rows, expected.rows, "predicted_velocity_mps");
errors.predicted_acceleration = rowMaximum(actual.rows, expected.rows, "predicted_acceleration_mps2");
errors.common_robust = rowMaximum(actual.rows, expected.rows, ...
    "predicted_common_robust_acceleration_i_mps2");
errors.desired_force = rowMaximum(actual.rows, expected.rows, "desired_force_n");
errors.gp_mean = rowMaximum(actual.rows, expected.rows, "gp_mean_i_mps2");
errors.half_width = rowMaximum(actual.rows, expected.rows, ...
    "calibrated_half_width_f_mps2");
errors.trust = rowMaximum(actual.rows, expected.rows, "trust");
errors.support_distance = rowMaximum(actual.rows, expected.rows, "support_distance");
errors.latent_variance_max = rowMaximum(actual.rows, expected.rows, "latent_variance_max");
errors.stage_energy = rowMaximum(actual.rows, expected.rows, "stage_energy_j");
errors.transition_fraction = rowMaximum(actual.rows, expected.rows, ...
    "outer_transition_fraction");
end


function maximum = rowMaximum(actualRows, expectedRows, fieldName)
maximum = 0.0;
for index = 1:numel(actualRows)
    actual = double(actualRows(index).(fieldName));
    expected = double(expectedRows(index).(fieldName));
    maximum = max(maximum, max(abs(actual(:) - expected(:))));
end
end
