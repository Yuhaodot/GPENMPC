function result = runNativeEnmpcCoreTests(projectRoot)
%RUNNATIVEENMPCCORETESTS Deterministic ordinary-B2 core unit tests.

arguments
    projectRoot (1,1) string
end
enmpcRoot = fullfile(projectRoot, "matlab", "enmpc");
addpath(enmpcRoot);
cleanup = onCleanup(@() rmpath(enmpcRoot));
runtimePath = fullfile(projectRoot, "matlab", "simulink", "assets", ...
    "enmpc_runtime_configuration.json");
config = gpenmpcLoadOrdinaryB2Config(runtimePath);
identity = gpenmpcOrdinaryB2Identity(config);

assert(config.control_blocks == 4);
assert(config.decision_dimension == 7);
assert(config.horizon_steps == 8);
assert(config.prediction_step_s == 0.2);
assert(config.prediction_horizon_s == 1.6);
assert(config.outer_period_s == 0.2);
assert(config.solver_deadline_s == 0.18);
assert(identity.method == "B2_ENMPC_GP_MEAN_TOTAL_TUBE");
assert(identity.fallback_method == "B1_ENMPC_TOTAL_ROBUST_TUBE_NO_GP");
assert(identity.robust_tube == "COMMON_TOTAL_ROBUST_TUBE");
assert(identity.gp_mean_scale == 1.0);
assert(identity.additional_prediction_acceleration_reserve_mps2 == 0.0);
assert(~identity.n1_residual_tube_enabled);
assert(~identity.b2r1_attenuation_enabled);
assert(identity.candidate_profile == ...
    "GP_MEAN_PREDICTION_ONLY__SHARED_PHYSICAL_CANDIDATE_LIBRARY");
assert(identity.parent_candidate_profile == "PARENT_FIRST_SAMPLE_CANCELLATION");
assert(identity.candidate_profile_runtime_override);
assert(~identity.gp_direct_physical_reference_candidate_enabled);
assert(isequal(identity.decision_shape, [4, 3]));

cadence030 = gpenmpcLoadOrdinaryB2Config(runtimePath, 0.30);
assert(cadence030.outer_period_s == 0.30);
assert(cadence030.solver_deadline_s == 0.28);
assert(cadence030.prediction_step_s == 0.20);
assert(cadence030.horizon_steps == 8);

warm = [0.01, -0.02, 0.03, -0.04, 0.02, -0.03, 0.04];
gpMeans = zeros(8, 3);
gpMeans(1, :) = [0.20, 0.00, 0.00];
b2 = gpenmpcBuildEnmpcCandidateLibrary(config.method, warm, gpMeans, config);
assert(b2.count == 8);
assert(~b2.gp_candidate_inserted);
assert(b2.gp_prediction_only);
assert(isequal(b2.labels, [ ...
    "WARM_START"; ...
    "ZERO"; ...
    "PHASE_CONSTANT_POS_0P04"; ...
    "PHASE_CONSTANT_NEG_0P04"; ...
    "PHASE_CONSTANT_POS_0P08"; ...
    "PHASE_CONSTANT_NEG_0P08"; ...
    "PHASE_MONOTONE_INCREASING"; ...
    "PHASE_MONOTONE_DECREASING"]));
assert(max(abs(b2.values(1, :) - warm)) == 0.0);
assert(max(abs(b2.values(2, :))) == 0.0);
assert(max(abs(b2.values(3, 1:4) - 0.04)) <= 1.0e-15);
assert(max(abs(b2.values(4, 1:4) + 0.04)) <= 1.0e-15);
assert(max(abs(b2.values(5, 1:4) - 0.08)) <= 1.0e-15);
assert(max(abs(b2.values(6, 1:4) + 0.08)) <= 1.0e-15);
assert(max(abs(b2.values(7, 1:4) - linspace(-0.08, 0.08, 4))) <= 1.0e-15);
assert(max(abs(b2.values(8, 1:4) - linspace(0.08, -0.08, 4))) <= 1.0e-15);
b2Repeat = gpenmpcBuildEnmpcCandidateLibrary(config.method, warm, gpMeans, config);
assert(isequal(b2.values, b2Repeat.values));
assert(isequal(b2.labels, b2Repeat.labels));

b1 = gpenmpcBuildEnmpcCandidateLibrary( ...
    config.b1_fallback_method, warm, zeros(0, 3), config);
assert(b1.count == 8);
assert(~b1.gp_candidate_inserted);
assert(~any(b1.labels == "GP_FIRST_SAMPLE_CANCELLATION"));
assert(isequal(b2.values, b1.values));
assert(isequal(b2.labels, b1.labels));
zeroB2 = gpenmpcBuildEnmpcCandidateLibrary( ...
    config.method, zeros(1, 7), zeros(8, 3), config);
assert(zeroB2.count == 7);
assert(~zeroB2.gp_candidate_inserted);

anchor = [0.01, 0.02, 0.03, 0.04, 0.09, -0.09, 0.0];
refinement = gpenmpcBuildEnmpcRefinementCandidates(anchor, config);
assert(refinement.count == 6);
assert(max(abs(refinement.values(:, 1:4) - repmat(anchor(1:4), 6, 1)), [], "all") == 0.0);
expectedCorrections = [ ...
    0.04, -0.09, 0.00; ...
    0.10, -0.09, 0.00; ...
    0.09, -0.10, 0.00; ...
    0.09, -0.04, 0.00; ...
    0.09, -0.09, -0.05; ...
    0.09, -0.09, 0.05];
assert(max(abs(refinement.values(:, 5:7) - expectedCorrections), [], "all") <= 1.0e-15);

rows = repmat(stageTemplate(config), config.horizon_steps, 1);
for index = 1:config.horizon_steps
    rows(index).stage_energy_j = 10.0 * index;
    rows(index).position_error_norm_m = 0.01 * index;
    rows(index).velocity_error_norm_mps = 0.02 * index;
    rows(index).force_variation_norm_n = 0.5 * index;
    rows(index).reference_jerk_norm_mps3 = 0.10 * index;
    rows(index).phase_acceleration_s_inv = 0.005 * index;
    rows(index).phase_jerk_s_inv2 = 0.01 * index;
    rows(index).progress_deficit_s = 0.02 * index;
    rows(index).trust = 0.1 * index;
end
rows(1).predicted_next_position_error_m = 0.60;
rows(1).predicted_next_velocity_error_mps = 1.50;
rows(2).risk_constraints_normalized(3) = -0.5;
book = gpenmpcBookkeepEnmpcRollout(rows, 500.0, false, false, config);
expectedObjective = manualObjective(rows, 500.0, config);
assert(abs(book.objective - expectedObjective) <= 1.0e-12);
assert(book.hard_constraint_count == 121);
assert(book.risk_constraint_count == 48);
assert(book.constraints(end) == 1.0);
assert(abs(book.risk_violation - 0.25) <= 1.0e-15);
assert(abs(book.tracking_zone_violation - 0.29) <= 1.0e-12);
assert(abs(book.predicted_energy_j - 360.0) <= 1.0e-12);
assert(abs(book.mean_trust - 0.45) <= 1.0e-12);
invalidBook = gpenmpcBookkeepEnmpcRollout(rows, 500.0, true, true, config);
assert(invalidBook.constraints(end) == -1.0);
assert(invalidBook.current_hard_invalid);

candidates = [ ...
    0, 0, 0, 0, 0, 0, 0; ...
    0, 0, 0, 0, 0, 0, 0.02; ...
    0, 0, 0, 0, 0, 0, -0.02; ...
    0, 0, 0, 0, 0, 0, -0.03];
evaluations = repmat(mockEvaluation(1.0, 0.0, 0.0, false), 4, 1);
evaluations(1) = mockEvaluation(0.5, 0.1, 0.0, false);
evaluations(2) = mockEvaluation(1.0, 0.0, 0.0, false);
evaluations(3) = mockEvaluation(1.0, 0.0, 0.0, false);
evaluations(4) = mockEvaluation(0.1, 0.0, 0.0, true);
selection = gpenmpcSelectEnmpcCandidate(candidates, evaluations, config);
assert(selection.has_feasible);
assert(selection.index == 3);
assert(isequal(selection.candidate, candidates(3, :)));

successB1 = decisionTemplate(config.b1_fallback_method, true, config);
lifted = gpenmpcLiftB1FallbackDecision( ...
    successB1, 0.37, true, "GP_HARD_INVALID", config);
assert(lifted.method == config.method);
assert(lifted.success);
assert(~lifted.fallback_active);
assert(lifted.gp_b1_fallback_active);
assert(lifted.mean_trust == 0.37);
assert(lifted.hard_invalid);
assert(isequal(lifted.decision_blocks_s_inv, successB1.decision_blocks_s_inv));
assert(isequal(lifted.outer_acceleration_correction_f_mps2, ...
    successB1.outer_acceleration_correction_f_mps2));
failedB1 = decisionTemplate(config.b1_fallback_method, false, config);
failedB1.fallback_reason = "NO_FEASIBLE_SHOOTING_PROFILE";
liftedFailure = gpenmpcLiftB1FallbackDecision( ...
    failedB1, 0.2, false, "GP_NO_FEASIBLE_PROFILE", config);
assert(liftedFailure.fallback_reason == ...
    "GP_NO_FEASIBLE_PROFILE__NO_FEASIBLE_SHOOTING_PROFILE");

shifted = gpenmpcShiftEnmpcWarmStart(anchor, true, config);
assert(isequal(shifted, [anchor(2:4), anchor(4), anchor(5:7)]));
assert(isequal(gpenmpcShiftEnmpcWarmStart(anchor, false, config), zeros(1, 7)));

% Exercise the bounded core and exact B1 fallback with deterministic mocks.
coreConfig = config;
coreConfig.solver_deadline_s = 10.0;
[coreDecision, coreWarm, coreAudit] = gpenmpcSolveOrdinaryB2FiniteShooting( ...
    warm, gpMeans, @b2Mock, @b1Mock, coreConfig);
assert(coreDecision.success);
assert(coreDecision.method == config.method);
assert(~coreDecision.gp_b1_fallback_active);
assert(coreAudit.primary_count == 8);
assert(coreAudit.refinement_count == 6);
assert(numel(coreWarm) == 7);
[fallbackDecision, fallbackWarm, fallbackAudit] = ...
    gpenmpcSolveOrdinaryB2FiniteShooting( ...
    warm, gpMeans, @hardInvalidB2Mock, @b1Mock, coreConfig);
assert(fallbackDecision.success);
assert(fallbackDecision.method == config.method);
assert(fallbackDecision.gp_b1_fallback_active);
assert(fallbackDecision.hard_invalid);
assert(fallbackAudit.b1_fallback_active);
assert(fallbackAudit.b1_fallback_trigger == "GP_HARD_INVALID");
assert(fallbackAudit.b1_fallback_full_budget);
assert(fallbackAudit.b1_fallback_budget_identity == ...
    "FRESH_FULL_B1_OUTER_DEADLINE");
assert(fallbackAudit.b1_fallback_method == config.b1_fallback_method);
assert(numel(fallbackWarm) == 7);
[failedDecision, resetWarm, failedAudit] = ...
    gpenmpcSolveOrdinaryB2FiniteShooting( ...
    warm, gpMeans, @noFeasibleMock, @noFeasibleMock, coreConfig);
assert(~failedDecision.success);
assert(isequal(resetWarm, zeros(size(warm))));
assert(~failedAudit.failure_warm_start_preserved);
assert(~failedAudit.b1.failure_warm_start_preserved);
assert(failedAudit.argmin_constraint_name == "PREDICTED_ACCELERATION");
assert(failedAudit.argmin_horizon_step == 2);

result = struct;
result.schema = "GPENMPC_MATLAB_NATIVE_ENMPC_CORE_TEST_RESULT_V1";
result.status = "PASS";
result.runtime_config_path = runtimePath;
result.decision_dimension = config.decision_dimension;
result.maximum_primary_candidates = config.maximum_primary_candidates;
result.maximum_refinement_candidates = config.maximum_refinement_candidates;
result.b2_primary_count_tested = b2.count;
result.b1_primary_count_tested = b1.count;
result.hard_constraint_count = book.hard_constraint_count;
result.risk_constraint_count = book.risk_constraint_count;
result.b1_fallback_identity_tested = true;
result.n1_or_b2r1_implemented = false;
fprintf("MATLAB_NATIVE_ENMPC_CORE_PASS decision=%d primary=%d refinement=%d hard=%d risk=%d\n", ...
    result.decision_dimension, result.b2_primary_count_tested, ...
    result.maximum_refinement_candidates, result.hard_constraint_count, ...
    result.risk_constraint_count);

    function item = b2Mock(candidate)
        item = mockEvaluation(sum(candidate.^2), 0.0, 0.0, false);
        item.mean_trust = 0.8;
    end

    function item = hardInvalidB2Mock(candidate) %#ok<INUSD>
        item = mockEvaluation(1.0, 0.0, 0.0, true);
        item.current_hard_invalid = true;
        item.mean_trust = 0.0;
    end

    function item = b1Mock(candidate)
        item = mockEvaluation(sum(candidate.^2), 0.0, 0.0, false);
        item.mean_trust = 0.0;
    end

    function item = noFeasibleMock(candidate)
        item = mockEvaluation(sum(candidate.^2), 0.0, 0.0, false);
        item.constraints((2 - 1) * coreConfig.hard_constraints_per_step + 8) = -0.2;
        item.hard_constraints = item.constraints;
    end
end


function row = stageTemplate(config)
row = struct;
row.stage_energy_j = 0.0;
row.position_error_norm_m = 0.0;
row.velocity_error_norm_mps = 0.0;
row.force_variation_norm_n = 0.0;
row.reference_jerk_norm_mps3 = 0.0;
row.phase_acceleration_s_inv = 0.0;
row.phase_jerk_s_inv2 = 0.0;
row.progress_deficit_s = 0.0;
row.predicted_next_position_error_m = 0.0;
row.predicted_next_velocity_error_mps = 0.0;
row.trust = 0.0;
row.hard_constraints = ones(1, config.hard_constraints_per_step);
row.risk_constraints_normalized = ones(1, config.risk_constraints_per_step);
end


function objective = manualObjective(rows, terminalEnergyToGoJ, config)
objective = 0.0;
for row = rows.'
    objective = objective + config.weights.energy * row.stage_energy_j / config.energy_scale_j;
    objective = objective + config.weights.position ...
        * (row.position_error_norm_m / config.position_scale_m)^2;
    objective = objective + config.weights.velocity ...
        * (row.velocity_error_norm_mps / config.velocity_scale_mps)^2;
    objective = objective + config.weights.force_variation ...
        * (row.force_variation_norm_n / config.force_variation_scale_n)^2;
    objective = objective + config.weights.jerk ...
        * (row.reference_jerk_norm_mps3 / config.jerk_scale_mps3)^2;
    objective = objective + config.weights.phase_acceleration ...
        * (row.phase_acceleration_s_inv / config.phase_acceleration_scale_s_inv)^2;
    objective = objective + config.weights.phase_acceleration_variation ...
        * (row.phase_jerk_s_inv2 * config.prediction_step_s ...
        / config.phase_acceleration_scale_s_inv)^2;
    objective = objective + config.weights.progress ...
        * (row.progress_deficit_s / config.progress_scale_s)^2;
end
objective = objective + config.terminal_energy_to_go_weight ...
    * terminalEnergyToGoJ / config.energy_scale_j;
end


function item = mockEvaluation(objective, risk, tracking, hardInvalid)
item = struct;
item.objective = objective;
item.constraints = ones(121, 1);
item.hard_constraints = item.constraints;
item.risk_violation = risk;
item.tracking_zone_violation = tracking;
item.predicted_energy_j = 100.0;
item.terminal_energy_to_go_j = 20.0;
item.terminal_progress_s = 1.0;
item.mean_trust = 0.5;
item.hard_invalid = hardInvalid;
item.current_hard_invalid = false;
if hardInvalid
    item.constraints(end) = -1.0;
    item.hard_constraints(end) = -1.0;
end
end


function decision = decisionTemplate(method, success, config)
decision = struct;
decision.schema = "TEST_DECISION";
decision.method = method;
decision.phase_acceleration_s_inv = 0.01;
decision.objective = 1.25;
decision.success = success;
decision.fallback_active = ~success;
decision.fallback_reason = "";
decision.elapsed_seconds = 0.01;
decision.solver_iterations = 14;
decision.minimum_constraint_margin = 0.2;
decision.predicted_energy_j = 100.0;
decision.predicted_progress_s = 1.1;
decision.mean_trust = 0.0;
decision.hard_invalid = false;
decision.decision_blocks_s_inv = [0.01, 0.02, 0.03, 0.04];
decision.outer_acceleration_correction_f_mps2 = [0.02, -0.01, 0.0];
decision.deadline_guard_truncated = false;
decision.risk_violation = 0.0;
decision.tracking_zone_violation = 0.0;
decision.terminal_energy_to_go_j = 10.0;
decision.gp_b1_fallback_active = false;
assert(numel(decision.decision_blocks_s_inv) == config.control_blocks);
end
