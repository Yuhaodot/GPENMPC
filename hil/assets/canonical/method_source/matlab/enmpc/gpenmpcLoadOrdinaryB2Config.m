function config = gpenmpcLoadOrdinaryB2Config(runtimeConfigurationPath, outerPeriodS)
%GPENMPCLOADORDINARYB2CONFIG Load the selected ordinary-B2 configuration.
%
% This loader exposes the supported 0.20 s and 0.30 s online cadences. The
% internal prediction grid remains 0.20 s x 8 = 1.60 s for both, and the
% selected B1/B2 identities are validated on load.

arguments
    runtimeConfigurationPath (1,1) string
    outerPeriodS (1,1) double = NaN
end
if strlength(string(fileparts(runtimeConfigurationPath)))==0
    runtimeConfigurationPath=fullfile(fileparts(fileparts(mfilename('fullpath'))), ...
        'simulink','assets',runtimeConfigurationPath);
end
if ~isfile(runtimeConfigurationPath)
    error("gpenmpcLoadOrdinaryB2Config:MissingRuntime", ...
        "Selected runtime configuration is missing: %s", runtimeConfigurationPath);
end
raw = jsondecode(fileread(runtimeConfigurationPath));
requireText(raw, "schema", "GPENMPC_AERO_F17_GP_PREDICTIVE_ENMPC_RUNTIME_V1");
requireText(raw.fixed_method_identity, "B1", "B1_ENMPC_TOTAL_ROBUST_TUBE_NO_GP");
requireText(raw.fixed_method_identity, "B2", "B2_ENMPC_GP_MEAN_TOTAL_TUBE");
requireText(raw.fixed_method_identity, "N1", "N1_ENMPC_GP_MEAN_RESIDUAL_TUBE");
requireText(raw.solver, "algorithm", ...
    "DETERMINISTIC_FINITE_PROFILE_SHOOTING_WITH_MOVE_BLOCKING");

solver = raw.solver;
bounds = raw.phase_bounds;
screens = raw.prediction_screens;
scales = raw.objective_scales;
inner = raw.common_inner_loop_predictor;
if isfield(solver, "prediction_step_s")
    predictionStepS = double(solver.prediction_step_s);
else
    predictionStepS = double(solver.outer_period_s);
end
selectedOuterPeriodS = double(solver.outer_period_s);
selectedDeadlineS = double(solver.deadline_s);
if isnan(outerPeriodS)
    outerPeriodS = selectedOuterPeriodS;
end
if abs(outerPeriodS - 0.20) <= 1.0e-12
    deadlineS = 0.18;
elseif abs(outerPeriodS - 0.30) <= 1.0e-12
    deadlineS = 0.28;
else
    error("gpenmpcLoadOrdinaryB2Config:Cadence", ...
        "Ordinary B2 permits only the frozen 0.20 s and 0.30 s outer cadences.");
end

bounded = struct;
if isfield(raw, "bounded_b2_optimization")
    bounded = raw.bounded_b2_optimization;
end
gpMeanScale = optionalScalar(bounded, "gp_mean_scale", 1.0);
additionalReserve = optionalScalar( ...
    bounded, "additional_prediction_acceleration_reserve_mps2", 0.0);
deadlineGuardReserveS = optionalScalar(bounded, "deadline_guard_reserve_s", 0.0);
parentCandidateProfile = optionalText( ...
    bounded, "gp_candidate_profile", "PARENT_FIRST_SAMPLE_CANCELLATION");
candidateProfile = "GP_MEAN_PREDICTION_ONLY__SHARED_PHYSICAL_CANDIDATE_LIBRARY";

config = struct;
config.schema = "GPENMPC_MATLAB_NATIVE_ORDINARY_B2_CORE_CONFIG_V1";
config.source_schema = string(raw.schema);
config.source_path = runtimeConfigurationPath;
config.method = "B2_ENMPC_GP_MEAN_TOTAL_TUBE";
config.b1_fallback_method = "B1_ENMPC_TOTAL_ROBUST_TUBE_NO_GP";
config.gp_candidate_profile = candidateProfile;
config.parent_gp_candidate_profile = parentCandidateProfile;
config.gp_candidate_profile_runtime_override = true;
config.gp_mean_scale = gpMeanScale;
config.additional_prediction_acceleration_reserve_mps2 = additionalReserve;
config.robust_tube_identity = "COMMON_TOTAL_ROBUST_TUBE";
config.common_robust_total_radius_f_mps2 = rowVector(inner.rho_m2_total_f_mps2);
config.post_gp_residual_radius_f_mps2 = ...
    rowVector(raw.authority_separation.rho_post_gp_f_mps2);
config.n1_residual_tube_enabled = false;
config.b2r1_enabled = false;
config.outer_period_s = outerPeriodS;
config.selected_runtime_outer_period_s = selectedOuterPeriodS;
config.prediction_step_s = predictionStepS;
config.prediction_horizon_s = double(solver.prediction_horizon_s);
config.horizon_steps = double(solver.horizon_steps);
config.move_block_size = double(solver.move_block_size);
config.control_blocks = config.horizon_steps / config.move_block_size;
config.decision_dimension = config.control_blocks + 3;
config.solver_deadline_s = deadlineS;
config.selected_runtime_deadline_s = selectedDeadlineS;
config.deadline_guard_reserve_s = deadlineGuardReserveS;
config.phase_rate_min = double(bounds.phase_rate_min);
config.phase_rate_max = double(bounds.phase_rate_max);
config.phase_acceleration_max_s_inv = double(bounds.phase_acceleration_max_s_inv);
config.outer_acceleration_correction_max_mps2 = ...
    double(bounds.outer_acceleration_correction_max_mps2);
config.shooting_perturbation_s_inv = double(solver.shooting_perturbation_s_inv);
config.shooting_constant_levels_s_inv = rowVector(solver.shooting_constant_levels_s_inv);
config.shooting_acceleration_perturbation_mps2 = ...
    double(solver.shooting_acceleration_perturbation_mps2);
config.solver_max_iterations = double(solver.maximum_iterations);
config.solver_ftol = double(solver.ftol);
config.position_screen_m = double(screens.position_m);
config.velocity_screen_mps = double(screens.velocity_mps);
config.acceleration_screen_mps2 = double(screens.acceleration_mps2);
config.common_inner_loop_acceleration_reserve_mps2 = ...
    double(screens.common_inner_loop_acceleration_reserve_mps2);
config.jerk_screen_mps3 = double(screens.jerk_mps3);
config.corridor_half_width_m = double(screens.corridor_half_width_m);
config.energy_scale_j = double(scales.energy_j);
config.position_scale_m = double(scales.position_m);
config.velocity_scale_mps = double(scales.velocity_mps);
config.force_variation_scale_n = double(scales.force_variation_n);
config.jerk_scale_mps3 = double(scales.jerk_mps3);
config.phase_acceleration_scale_s_inv = double(scales.phase_acceleration_s_inv);
config.progress_scale_s = double(scales.progress_s);
config.weights = raw.objective_weights;
config.tracking_position_zone_m = double(raw.feasibility_first.tracking_position_zone_m);
config.tracking_velocity_zone_mps = double(raw.feasibility_first.tracking_velocity_zone_mps);
config.terminal_energy_to_go_weight = double(raw.feasibility_first.terminal_energy_to_go_weight);
config.minimum_soft_trust = ...
    double(raw.uncertainty_tightening.minimum_soft_trust);
config.reference_transition_jerk_limit_mps3 = ...
    double(raw.bumpless_recovery.reference_transition_jerk_limit_mps3);
config.recovery_minimum_dwell_outer_ticks = ...
    double(raw.bumpless_recovery.minimum_dwell_outer_ticks);
config.reentry_success_persistence_outer_ticks = ...
    double(raw.bumpless_recovery.reentry_success_persistence_outer_ticks);
% GP consistency uses the calibrated interval and a 0.50 s causal residual
% memory with a continuous normalized-innovation state. The two thresholds
% form a fixed hysteresis pair: a model re-enters at the calibrated interval
% boundary (score 0.5) and exits after two outer updates below score 0.25.
% The hysteresis thresholds are engineering design values.
config.innovation_consistency_time_constant_s = 0.50;
config.innovation_consistency_enter_threshold = 0.50;
config.innovation_consistency_exit_threshold = 0.25;
config.innovation_consistency_exit_dwell_outer_ticks = 2;
% Warm-candidate retention selects the evaluated warm candidate when a newly
% ranked candidate provides no risk/tracking improvement and less than
% 0.5 percent economic improvement. This common B1/B2 rule operates within
% the configured candidate set, solver budget and controller authority.
config.candidate_commit_hysteresis_fraction = 0.005;
config.candidate_primary_equivalence_tolerance = 1.0e-12;
% Optional execution-alignment settings default off.
config.continuous_axis_weight_supervision_enabled = false;
config.execution_consistent_cost_enabled = false;
config.execution_rotor_variation_scale_n = config.force_variation_scale_n;
% Prediction-effective commit is disabled by default. When enabled, soft GP
% trust remains a continuous mean weight, while candidate changes must clear
% the configured 0.5 percent economic hysteresis plus bounded uncertainty and
% normalized decision-change charges. Predicted reference jerk and
% desired-force variation must each satisfy the configured tolerance.
config.prediction_effective_commit_enabled = false;
config.prediction_effective_direct_b2_enabled = false;
config.prediction_effective_commit_base_fraction = ...
    config.candidate_commit_hysteresis_fraction;
config.prediction_effective_commit_uncertainty_fraction = ...
    config.candidate_commit_hysteresis_fraction;
config.prediction_effective_commit_switch_fraction = ...
    config.candidate_commit_hysteresis_fraction;
config.prediction_effective_execution_nonworse_tolerance = 1.0e-12;
% Coordinated GP execution is disabled by default. When enabled, one causal
% GP cancellation term operates within the shared 0.75 m/s^2 compensation
% authority and uses the same mean, frame, trust and cap in prediction and
% execution through the shared compensation channel.
config.coordinated_gp_execution_feedforward_enabled = false;
config.coordinated_gp_objective_scaling_enabled = false;
config.coordinated_gp_progress_scale_s = 2.5;
config.coordinated_gp_execution_rotor_variation_scale_n = 16.3385037412;
config.coordinated_gp_observer_mode = ...
    "B1_ON_FALLBACK__OFF_WHEN_GP_ACTIVE__COMMON_FILTERED_TRANSITION";
config.coordinated_gp_alpha_zero_tolerance = 1.0e-12;
% Bounded-responsibility parameters are applied by the dedicated bounded-responsibility
% configuration function. The residual floor is the axiswise maximum of the
% nominal B1 closed-innovation P95 and the calibrated GP physical
% observation-noise standard deviation.
config.gp_responsibility_gate_enabled = false;
config.responsibility_residual_floor_f_mps2 = ...
    [0.03219254730192427, 0.029209190368417185, 0.037378411972285294];
config.responsibility_innovation_ewma_time_constant_s = 0.30;
config.gp_responsibility_enter_ratio = 1.25;
config.gp_responsibility_exit_ratio = 0.75;
config.gp_responsibility_enter_dwell_s = 0.30;
config.gp_responsibility_exit_dwell_s = 0.60;
config.responsibility_cross_fade_duration_s = 0.20;
config.responsibility_gp_mean_filter_time_constant_s = 0.12;
config.responsibility_common_energy_scale_j = 500.0;
config.hard_constraints_per_step = 15;
config.risk_constraints_per_step = 6;
config.hard_invalid_sentinel_count = 1;
config.expected_hard_constraint_count = ...
    config.horizon_steps * config.hard_constraints_per_step + 1;
config.expected_risk_constraint_count = ...
    config.horizon_steps * config.risk_constraints_per_step;
config.maximum_primary_candidates = 9;
config.maximum_refinement_candidates = 6;
config.feasibility_tolerance = 1.0e-7;
config.raw = raw;

validateFrozenIdentity(config);
end


function validateFrozenIdentity(config)
assertClose(config.selected_runtime_outer_period_s, 0.20, "selected outer period");
assertClose(config.selected_runtime_deadline_s, 0.18, "selected deadline");
assertClose(config.prediction_step_s, 0.20, "prediction step");
assertClose(config.prediction_horizon_s, 1.60, "prediction horizon");
assertClose(config.horizon_steps, 8, "horizon steps");
assertClose(config.move_block_size, 2, "move block size");
assertClose(config.control_blocks, 4, "control block count");
assertClose(config.decision_dimension, 7, "decision dimension");
assertClose(config.phase_rate_min, 0.70, "minimum phase rate");
assertClose(config.phase_rate_max, 1.08, "maximum phase rate");
assertClose(config.phase_acceleration_max_s_inv, 0.08, "phase acceleration bound");
assertClose(config.outer_acceleration_correction_max_mps2, 0.10, ...
    "outer correction bound");
assertVector(config.shooting_constant_levels_s_inv, [0.04, 0.08], ...
    "constant shooting levels");
assertClose(config.shooting_perturbation_s_inv, 0.08, "shooting perturbation");
assertClose(config.shooting_acceleration_perturbation_mps2, 0.05, ...
    "correction refinement perturbation");
assertClose(config.gp_mean_scale, 1.0, "ordinary-B2 GP mean scale");
assertClose(config.additional_prediction_acceleration_reserve_mps2, 0.0, ...
    "ordinary-B2 additional reserve");
assertClose(config.deadline_guard_reserve_s, 0.0, "deadline guard reserve");
if config.parent_gp_candidate_profile ~= "PARENT_FIRST_SAMPLE_CANCELLATION"
    error("gpenmpcLoadOrdinaryB2Config:Profile", ...
        "The parent ordinary-B2 profile identity is not recognized.");
end
if config.gp_candidate_profile ~= ...
        "GP_MEAN_PREDICTION_ONLY__SHARED_PHYSICAL_CANDIDATE_LIBRARY"
    error("gpenmpcLoadOrdinaryB2Config:RuntimeProfile", ...
        "The runtime profile must apply the GP mean only in prediction.");
end
assertVector(config.common_robust_total_radius_f_mps2, ...
    [0.125021561465691, 0.10321513341156671, 0.39568699178777844], ...
    "common total robust tube");
assertClose(config.common_inner_loop_acceleration_reserve_mps2, 0.75, ...
    "common inner-loop reserve");
assertClose(config.acceleration_screen_mps2, 2.2, "acceleration screen");
assertClose(config.minimum_soft_trust, 0.25, "minimum soft trust");
assertClose(config.innovation_consistency_time_constant_s, 0.50, ...
    "innovation consistency time constant");
assertClose(config.innovation_consistency_enter_threshold, 0.50, ...
    "innovation consistency enter threshold");
assertClose(config.innovation_consistency_exit_threshold, 0.25, ...
    "innovation consistency exit threshold");
assertClose(config.innovation_consistency_exit_dwell_outer_ticks, 2, ...
    "innovation consistency exit dwell");
assertClose(config.candidate_commit_hysteresis_fraction, 0.005, ...
    "candidate commit hysteresis fraction");
assertClose(config.candidate_primary_equivalence_tolerance, 1.0e-12, ...
    "candidate primary equivalence tolerance");
assertClose(config.prediction_effective_commit_base_fraction, 0.005, ...
    "prediction-effective base fraction");
assertClose(config.prediction_effective_commit_uncertainty_fraction, 0.005, ...
    "prediction-effective uncertainty fraction");
assertClose(config.prediction_effective_commit_switch_fraction, 0.005, ...
    "prediction-effective switch fraction");
assertClose(config.prediction_effective_execution_nonworse_tolerance, 1.0e-12, ...
    "prediction-effective execution tolerance");
assertClose(config.coordinated_gp_progress_scale_s, 2.5, ...
    "coordinated GP progress scale");
assertClose(config.coordinated_gp_execution_rotor_variation_scale_n, ...
    16.3385037412, "coordinated GP rotor-variation scale");
assertClose(config.coordinated_gp_alpha_zero_tolerance, 1.0e-12, ...
    "coordinated GP alpha-zero tolerance");
assertVector(config.responsibility_residual_floor_f_mps2, ...
    [0.03219254730192427, 0.029209190368417185, 0.037378411972285294], ...
    "residual floor");
assertClose(config.responsibility_innovation_ewma_time_constant_s, 0.30, ...
    "innovation EWMA time constant");
assertClose(config.gp_responsibility_enter_ratio, 1.25, ...
    "responsibility enter ratio");
assertClose(config.gp_responsibility_exit_ratio, 0.75, ...
    "responsibility exit ratio");
assertClose(config.gp_responsibility_enter_dwell_s, 0.30, ...
    "responsibility enter dwell");
assertClose(config.gp_responsibility_exit_dwell_s, 0.60, ...
    "responsibility exit dwell");
assertClose(config.responsibility_cross_fade_duration_s, 0.20, ...
    "cross fade duration");
assertClose(config.responsibility_gp_mean_filter_time_constant_s, 0.12, ...
    "GP mean filter time constant");
assertClose(config.responsibility_common_energy_scale_j, 500.0, ...
    "common energy scale");
if config.coordinated_gp_execution_feedforward_enabled ...
        || config.coordinated_gp_objective_scaling_enabled
    error("gpenmpcLoadOrdinaryB2Config:CoordinatedDefault", ...
        "Coordinated GP options must be disabled in the base configuration.");
end
if config.n1_residual_tube_enabled || config.b2r1_enabled
    error("gpenmpcLoadOrdinaryB2Config:MethodDrift", ...
        "N1 residual-tube and B2R1 behavior are outside ordinary B2.");
end
requiredWeights = ["energy", "position", "velocity", "force_variation", ...
    "jerk", "phase_acceleration", "phase_acceleration_variation", "progress"];
for name = requiredWeights
    if ~isfield(config.weights, name)
        error("gpenmpcLoadOrdinaryB2Config:Weights", ...
            "Missing frozen objective weight %s.", name);
    end
end
end


function requireText(value, fieldName, expected)
if ~isfield(value, fieldName) || string(value.(fieldName)) ~= string(expected)
    error("gpenmpcLoadOrdinaryB2Config:Identity", ...
        "Runtime identity field %s does not match %s.", fieldName, expected);
end
end


function value = optionalScalar(source, fieldName, defaultValue)
if isfield(source, fieldName)
    value = double(source.(fieldName));
else
    value = double(defaultValue);
end
end


function value = optionalText(source, fieldName, defaultValue)
if isfield(source, fieldName)
    value = string(source.(fieldName));
else
    value = string(defaultValue);
end
end


function value = rowVector(input)
value = double(input(:).');
end


function assertClose(actual, expected, label)
if ~isfinite(actual) || abs(double(actual) - double(expected)) > 1.0e-12
    error("gpenmpcLoadOrdinaryB2Config:FrozenValue", ...
        "Frozen %s changed (actual %.17g, expected %.17g).", ...
        label, actual, expected);
end
end


function assertVector(actual, expected, label)
actual = double(actual(:).');
expected = double(expected(:).');
if ~isequal(size(actual), size(expected)) || any(abs(actual - expected) > 1.0e-12)
    error("gpenmpcLoadOrdinaryB2Config:FrozenVector", ...
        "Frozen %s changed.", label);
end
end
