function result = runOuterSupervisorTests(projectRoot, outputPath)
%RUNOUTERSUPERVISORTESTS Test outer-solver failure and recovery transitions.

arguments
    projectRoot (1,1) string
    outputPath (1,1) string = ""
end
addpath(genpath(fullfile(projectRoot, "matlab")));
config = minimalConfig();
state = gpenmpcInitializeOuterSupervisor(config);

% A stale causal context bypasses B2 and requests B1 directly.
request = gpenmpcOuterSupervisorRequest(state, false, config);
assert(request.requested_method == config.b1_fallback_method);
[state, applied, event] = gpenmpcOuterSupervisorStep(state, request, ...
    goodDecision(config.b1_fallback_method, 0.02, [0.03;0;0]), ...
    emptyAudit(), config);
assert(state.mode == "B1_ONLY");
assert(state.b1_success_dwell_ticks == 0);
assert(applied.source == "B1_FULL_BUDGET_DECISION");
assert(event.transition_reason == "STALE_CONTEXT_DIRECT_B1");

% Two causal B1 successes are required before a B2 probe is requested.
request = gpenmpcOuterSupervisorRequest(state, true, config);
[state, ~] = gpenmpcOuterSupervisorStep(state, request, ...
    goodDecision(config.b1_fallback_method, 0.03, [0.04;0;0]), ...
    eligibleShadowAudit(config), config);
assert(state.mode == "B1_ONLY");
assert(state.b1_success_dwell_ticks == 1);
request = gpenmpcOuterSupervisorRequest(state, true, config);
[state, ~] = gpenmpcOuterSupervisorStep(state, request, ...
    goodDecision(config.b1_fallback_method, 0.04, [0.05;0;0]), ...
    eligibleShadowAudit(config), config);
assert(state.mode == "B2_REENTRY_PROBE");
assert(state.b1_success_dwell_ticks == 2);

% The first successful B2 probe is withheld. The second consecutive success
% is applied and returns the FSM to B2_ACTIVE.
request = gpenmpcOuterSupervisorRequest( ...
    state, true, config, eligibleShadow(config));
assert(request.request_reason == "B2_REENTRY_PROBE_WITH_CLOSED_GP_EVIDENCE");
[state, applied] = gpenmpcOuterSupervisorStep(state, request, ...
    goodDecision(config.method, 0.08, [-0.098;0.019;0]), ...
    emptyAudit(), config);
assert(state.mode == "B2_REENTRY_PROBE");
assert(state.b2_probe_success_ticks == 1);
assert(applied.source == "B1_REENTRY_PERSISTENCE_HOLD");
assert(applied.phase_acceleration_s_inv == 0.04);
request = gpenmpcOuterSupervisorRequest( ...
    state, true, config, eligibleShadow(config));
[state, applied] = gpenmpcOuterSupervisorStep(state, request, ...
    goodDecision(config.method, 0.08, [-0.098;0.019;0]), ...
    emptyAudit(), config);
assert(state.mode == "B2_ACTIVE");
assert(state.reentry_count == 1);
assert(applied.source == "B2_REENTRY_BLEND_DECISION");
assert(abs(applied.phase_delta_from_previous_s_inv) <= 0.04 + 1.0e-12);
assert(applied.correction_delta_norm_from_previous_mps2 <= 0.05 + 1.0e-12);

% A B2 deadline receives exactly one last-feasible hold. The next failed B1
% solve cannot extend that hold and therefore applies the fixed C3 target.
request = gpenmpcOuterSupervisorRequest( ...
    state, true, config, eligibleShadow(config));
[state, firstFailureApplied] = gpenmpcOuterSupervisorStep(state, request, ...
    failedDecision(config.method, "SOLVER_DEADLINE_MISSED"), ...
    emptyAudit(), config);
assert(state.mode == "B1_ONLY");
assert(firstFailureApplied.source == "LAST_FEASIBLE_ONE_TICK_HOLD");
assert(state.failure_hold_count == 1);
request = gpenmpcOuterSupervisorRequest(state, true, config);
assert(request.requested_method == config.b1_fallback_method);
[state, secondFailureApplied] = gpenmpcOuterSupervisorStep(state, request, ...
    failedDecision(config.b1_fallback_method, "SOLVER_DEADLINE_MISSED"), ...
    emptyAudit(), config, 0.70);
assert(state.mode == "FIXED_REFERENCE_HOLD");
assert(secondFailureApplied.source == "FIXED_C3_REFERENCE");
assert(secondFailureApplied.phase_acceleration_s_inv > 0.0);
assert(secondFailureApplied.phase_acceleration_s_inv ...
    <= config.phase_acceleration_max_s_inv);
assert(all(secondFailureApplied.outer_acceleration_correction_f_mps2 == 0.0));

% A later full-budget B1 success recovers from fixed reference without a
% command-authority increase.
request = gpenmpcOuterSupervisorRequest(state, true, config);
[state, recovered] = gpenmpcOuterSupervisorStep(state, request, ...
    goodDecision(config.b1_fallback_method, -0.06, [0.02;-0.03;0.01]), ...
    eligibleShadowAudit(config), config);
assert(state.mode == "B1_ONLY");
assert(recovered.source == "B1_FULL_BUDGET_DECISION");

% A B2 infeasibility with a successful exact B1 solve inside the original
% deadline applies that B1 result. Only an unavailable B1 result uses hold.
infeasibleState = gpenmpcInitializeOuterSupervisor(config);
request = gpenmpcOuterSupervisorRequest( ...
    infeasibleState, true, config, eligibleShadow(config));
[infeasibleState, ~] = gpenmpcOuterSupervisorStep( ...
    infeasibleState, request, ...
    goodDecision(config.method, 0.01, [0.01;0;0]), emptyAudit(), config);
request = gpenmpcOuterSupervisorRequest( ...
    infeasibleState, true, config, eligibleShadow(config));
inlineB1 = goodDecision(config.method, -0.03, [0.02;0;0]);
inlineB1.gp_b1_fallback_active = true;
inlineAudit = emptyAudit();
inlineAudit.b1_fallback_active = true;
[infeasibleState, infeasibleApplied] = gpenmpcOuterSupervisorStep( ...
    infeasibleState, request, inlineB1, inlineAudit, config);
assert(infeasibleState.mode == "B1_ONLY");
assert(infeasibleApplied.source == "B2_INFEASIBLE_EXACT_B1_DECISION");
request = gpenmpcOuterSupervisorRequest(infeasibleState, true, config);
assert(request.requested_method == config.b1_fallback_method);
assert(request.request_reason == "B1_RECOVERY_DWELL");

% A detected current hard-invalid B2 result may apply only the exact B1
% fallback result and then remains on B1_ONLY.
hardState = gpenmpcInitializeOuterSupervisor(config);
hardState.last_feasible_valid = true;
hardState.last_feasible_phase_acceleration_s_inv = 0.01;
hardState.last_feasible_outer_correction_f_mps2 = zeros(3,1);
hardState.failure_hold_available = true;
request = gpenmpcOuterSupervisorRequest( ...
    hardState, true, config, eligibleShadow(config));
hardDecision = goodDecision(config.method, -0.02, [0.01;0.01;0]);
hardDecision.hard_invalid = true;
hardDecision.gp_b1_fallback_active = true;
hardAudit = emptyAudit();
hardAudit.current_hard_invalid = true;
hardAudit.b1_fallback_active = true;
[hardState, hardApplied] = gpenmpcOuterSupervisorStep( ...
    hardState, request, hardDecision, hardAudit, config);
assert(hardState.mode == "B1_ONLY");
assert(hardApplied.source == "HARD_INVALID_EXACT_B1_DECISION");
assert(hardState.b1_success_dwell_ticks == 0);
assert(hardState.b2_probe_success_ticks == 0);
assert(~hardState.reentry_blend_active);

% The existing causal transition enforces the unchanged reference-jerk cap
% when a supervisor target changes.
time = [0; 1];
position = [0,0,0; 1,0,0];
velocity = [1,0,0; 1,0,0];
zero = zeros(2,3);
trajectory = gpenmpcSampledC3Trajectory(time, position, velocity, zero, zero);
transition = gpenmpcJerkBoundedReferenceTransition(trajectory, 0.25, 1.0, ...
    0.0, 0.08, zeros(3,1), [0.10;0.10;0.10], 0.01, ...
    config.reference_transition_jerk_limit_mps3);
assert(norm(transition.reference.jerk_mps3) ...
    <= config.reference_transition_jerk_limit_mps3 + 1.0e-12);
assert(abs(transition.phase_acceleration_s_inv) ...
    <= config.phase_acceleration_max_s_inv + 1.0e-12);
assert(all(abs(transition.outer_correction_i_mps2) ...
    <= config.outer_acceleration_correction_max_mps2 + 1.0e-12));

result = struct;
result.schema = "GPENMPC_SOFTWARE_OUTER_SUPERVISOR_TEST_RESULT_V1";
result.status = "PASS";
result.stale_context_direct_b1 = true;
result.minimum_b1_dwell_ticks = config.recovery_minimum_dwell_outer_ticks;
result.required_b2_probe_success_ticks = ...
    config.reentry_success_persistence_outer_ticks;
result.single_failure_hold_count = state.failure_hold_count;
result.fixed_reference_after_repeated_failure = true;
result.hard_invalid_exact_b1 = true;
result.b2_infeasible_exact_b1_within_deadline = true;
result.reference_jerk_norm_mps3 = norm(transition.reference.jerk_mps3);
result.reference_jerk_limit_mps3 = ...
    config.reference_transition_jerk_limit_mps3;
if strlength(outputPath) > 0
    parent = fileparts(outputPath);
    if ~isfolder(parent), mkdir(parent); end
    handle = fopen(outputPath, "w");
    cleaner = onCleanup(@() fclose(handle));
    fprintf(handle, "%s\n", jsonencode(result, PrettyPrint=true));
end
fprintf("%s\n", jsonencode(result));
end


function config = minimalConfig()
config = struct;
config.method = "B2_ENMPC_GP_MEAN_TOTAL_TUBE";
config.b1_fallback_method = "B1_ENMPC_TOTAL_ROBUST_TUBE_NO_GP";
config.recovery_minimum_dwell_outer_ticks = 2;
config.reentry_success_persistence_outer_ticks = 2;
config.innovation_consistency_time_constant_s = 0.50;
config.innovation_consistency_enter_threshold = 0.50;
config.innovation_consistency_exit_threshold = 0.25;
config.innovation_consistency_exit_dwell_outer_ticks = 2;
config.phase_acceleration_max_s_inv = 0.08;
config.outer_acceleration_correction_max_mps2 = 0.10;
config.minimum_soft_trust = 0.25;
config.reference_transition_jerk_limit_mps3 = 2.0;
config.outer_period_s = 0.30;
config.raw = struct( ...
    "bumpless_recovery", struct("target_recovery_rate", 1.0), ...
    "uncertainty_tightening", struct("minimum_soft_trust", 0.25));
end


function decision = goodDecision(method, phaseAcceleration, correction)
decision = decisionTemplate(method);
decision.success = true;
decision.fallback_active = false;
decision.phase_acceleration_s_inv = phaseAcceleration;
decision.outer_acceleration_correction_f_mps2 = correction(:).';
end


function decision = failedDecision(method, reason)
decision = decisionTemplate(method);
decision.success = false;
decision.fallback_active = true;
decision.fallback_reason = string(reason);
end


function decision = decisionTemplate(method)
decision = struct;
decision.method = string(method);
decision.success = false;
decision.phase_acceleration_s_inv = 0.0;
decision.outer_acceleration_correction_f_mps2 = zeros(1,3);
decision.hard_invalid = false;
decision.gp_b1_fallback_active = false;
decision.fallback_active = false;
decision.fallback_reason = "";
end


function audit = emptyAudit()
audit = struct("current_hard_invalid", false, "b1_fallback_active", false);
end


function audit = eligibleShadowAudit(config)
audit = emptyAudit();
audit.shadow_gp = struct( ...
    "available", true, ...
    "causal_valid", true, ...
    "observed_innovation_available", true, ...
    "gp_model_available", true, ...
    "hard_invalid", false, ...
    "trust", config.minimum_soft_trust, ...
    "consistency_ewma_f", [0.8,0.8,0.8], ...
    "consistency_ewma_aggregate", 0.8, ...
    "observed_innovation_consistent", true);
end


function shadow = eligibleShadow(config)
audit = eligibleShadowAudit(config);
shadow = audit.shadow_gp;
end
