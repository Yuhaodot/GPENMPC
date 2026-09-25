function result = runShadowReentryFocusedTests( ...
        stagingRoot, authorityRoot, outputPath)
%RUNSHADOWREENTRYFOCUSEDTESTS Focused causal shadow/re-entry regressions.

arguments
    stagingRoot (1,1) string
    authorityRoot (1,1) string
    outputPath (1,1) string = ""
end
originalPath = path;
cleanup = onCleanup(@() path(originalPath));
addpath(genpath(authorityRoot), "-end");
addpath(fullfile(stagingRoot, "matlab", "enmpc"), "-begin");
addpath(fullfile(stagingRoot, "matlab", "comparison"), "-begin");
addpath(fullfile(stagingRoot, "tests", "helpers"), "-begin");

config = minimalConfig();

%% Sustained invalid/unavailable shadow evidence never reaches probe mode.
state = b1State(config);
for index = 1:6
    request = gpenmpcOuterSupervisorRequest(state, true, config);
    [state, applied, event] = gpenmpcOuterSupervisorStep( ...
        state, request, goodDecision(config.b1_fallback_method, ...
        -0.02, [0.02; 0.0; 0.0]), invalidShadowAudit(config), config);
    assert(state.mode == "B1_ONLY");
    assert(state.b1_success_dwell_ticks == 0);
    assert(state.b2_probe_success_ticks == 0);
    assert(~state.reentry_blend_active);
    assert(applied.source == "B1_FULL_BUDGET_DECISION");
    assert(~event.shadow_eligible_for_b1_dwell);
end
sustainedInvalidNoProbe = state.b2_probe_request_count == 0;
assert(sustainedInvalidNoProbe);

%% Alternating valid/unavailable evidence resets the consecutive B1 dwell.
state = b1State(config);
state = oneB1Tick(state, eligibleShadowAudit(config), config);
assert(state.b1_success_dwell_ticks == 1);
state = oneB1Tick(state, invalidShadowAudit(config), config);
assert(state.b1_success_dwell_ticks == 0);
state = oneB1Tick(state, eligibleShadowAudit(config), config);
assert(state.b1_success_dwell_ticks == 1);
assert(state.mode == "B1_ONLY");
alternatingReset = true;

%% Innovation-inconsistent evidence cannot accumulate dwell or request re-entry.
state = b1State(config);
state = oneB1Tick(state, eligibleShadowAudit(config), config);
assert(state.b1_success_dwell_ticks == 1);
request = gpenmpcOuterSupervisorRequest(state, true, config);
[state, ~, event] = gpenmpcOuterSupervisorStep(state, request, ...
    goodDecision(config.b1_fallback_method, 0.01, [0.01; 0.0; 0.0]), ...
    inconsistentShadowAudit(config), config);
assert(state.mode == "B1_ONLY");
assert(state.b1_success_dwell_ticks == 0);
assert(~event.shadow_eligible_for_b1_dwell);
assert(event.transition_reason == ...
    "B1_SHADOW_CONSISTENCY_EWMA_BELOW_ENTER_RESET");
innovationInconsistentResetsDwell = true;

state = gpenmpcInitializeOuterSupervisor(config);
state.mode = "B2_REENTRY_PROBE";
request = gpenmpcOuterSupervisorRequest( ...
    state, true, config, inconsistentShadow(config));
assert(request.requested_method == config.b1_fallback_method);
assert(request.request_reason == ...
    "B2_REENTRY_GP_CONSISTENCY_EWMA_BELOW_ENTER_DIRECT_B1");
innovationInconsistentBlocksReentryRequest = true;

%% The real k to k+1 evidence closure carries consistency into dwell eligibility.
pending = struct( ...
    "available", true, ...
    "hard_invalid", false, ...
    "trust", 0.8, ...
    "gp_frame_i_from_f", eye(3), ...
    "predicted_mean_f_mps2", zeros(1,3), ...
    "calibrated_half_width_f_mps2", 0.1 .* ones(1,3));
closedInconsistent = gpenmpcCloseGpInnovationEvidence( ...
    pending, [0.2; 0.2; 0.2], true, config);
assert(~closedInconsistent.observed_innovation_consistent);
assert(~closedInconsistent.eligible_for_b1_dwell);
closedConsistent = gpenmpcCloseGpInnovationEvidence( ...
    pending, [0.05; 0.0; 0.0], true, config);
assert(closedConsistent.observed_innovation_consistent);
assert(closedConsistent.eligible_for_b1_dwell);
closedEvidenceConsistencyEnforced = true;

%% Probe requests are counted using the emitted closed-evidence reason code.
counterEvent = struct( ...
    "transitioned", false, ...
    "applied_source", "B1_REENTRY_PERSISTENCE_HOLD", ...
    "fixed_reference_applied", false, ...
    "request_reason", "B2_REENTRY_PROBE_WITH_CLOSED_GP_EVIDENCE", ...
    "transition_reason", "B2_PROBE_SUCCESS_PERSISTENCE_PENDING");
counters = gpenmpcObserveSupervisorEvent(struct, counterEvent);
assert(counters.supervisor_b2_probe_request_count == 1);
closedProbeReasonCounted = true;

%% Preserve two B1 dwell ticks and two successful B2 probe ticks.
state = b1State(config);
b1Target = goodDecision(config.b1_fallback_method, ...
    -0.08, [0.10; 0.0; 0.0]);
for index = 1:2
    request = gpenmpcOuterSupervisorRequest(state, true, config);
    [state, ~] = gpenmpcOuterSupervisorStep(state, request, ...
        b1Target, eligibleShadowAudit(config), config);
end
assert(state.mode == "B2_REENTRY_PROBE");
assert(state.b1_success_dwell_ticks == 2);
b2Target = goodDecision(config.method, 0.08, [-0.10; 0.0; 0.0]);
request = gpenmpcOuterSupervisorRequest( ...
    state, true, config, eligibleShadow(config));
[state, firstProbeApplied] = gpenmpcOuterSupervisorStep( ...
    state, request, b2Target, emptyAudit(), config);
assert(state.mode == "B2_REENTRY_PROBE");
assert(state.b2_probe_success_ticks == 1);
assert(firstProbeApplied.source == "B1_REENTRY_PERSISTENCE_HOLD");
previousPhase = firstProbeApplied.phase_acceleration_s_inv;
previousCorrection = firstProbeApplied.outer_acceleration_correction_f_mps2;
request = gpenmpcOuterSupervisorRequest( ...
    state, true, config, eligibleShadow(config));
[state, secondProbeApplied] = gpenmpcOuterSupervisorStep( ...
    state, request, b2Target, emptyAudit(), config);
assert(state.mode == "B2_ACTIVE");
assert(state.reentry_count == 1);
assert(secondProbeApplied.source == "B2_REENTRY_BLEND_DECISION");
assert(abs(secondProbeApplied.phase_acceleration_s_inv - previousPhase) ...
    <= 0.5 * config.phase_acceleration_max_s_inv + 1.0e-12);
assert(norm(secondProbeApplied.outer_acceleration_correction_f_mps2 ...
    - previousCorrection, 2) ...
    <= 0.5 * config.outer_acceleration_correction_max_mps2 + 1.0e-12);
twoPlusTwoTakeover = true;

%% Every blend update is bounded and a constant target converges finitely.
maximumPhaseDelta = 0.0;
maximumCorrectionDelta = 0.0;
blendUpdateCount = 1;
previousPhase = secondProbeApplied.phase_acceleration_s_inv;
previousCorrection = secondProbeApplied.outer_acceleration_correction_f_mps2;
maximumPhaseDelta = max(maximumPhaseDelta, ...
    abs(previousPhase - firstProbeApplied.phase_acceleration_s_inv));
maximumCorrectionDelta = max(maximumCorrectionDelta, ...
    norm(previousCorrection ...
    - firstProbeApplied.outer_acceleration_correction_f_mps2, 2));
for index = 1:16
    if ~state.reentry_blend_active
        break
    end
    request = gpenmpcOuterSupervisorRequest( ...
        state, true, config, eligibleShadow(config));
    [state, applied] = gpenmpcOuterSupervisorStep( ...
        state, request, b2Target, emptyAudit(), config);
    phaseDelta = abs(applied.phase_acceleration_s_inv - previousPhase);
    correctionDelta = norm(applied.outer_acceleration_correction_f_mps2 ...
        - previousCorrection, 2);
    maximumPhaseDelta = max(maximumPhaseDelta, phaseDelta);
    maximumCorrectionDelta = max(maximumCorrectionDelta, correctionDelta);
    assert(phaseDelta <= 0.5 * config.phase_acceleration_max_s_inv + 1.0e-12);
    assert(correctionDelta ...
        <= 0.5 * config.outer_acceleration_correction_max_mps2 + 1.0e-12);
    previousPhase = applied.phase_acceleration_s_inv;
    previousCorrection = applied.outer_acceleration_correction_f_mps2;
    blendUpdateCount = blendUpdateCount + 1;
end
assert(~state.reentry_blend_active);
assert(abs(previousPhase - b2Target.phase_acceleration_s_inv) <= 1.0e-12);
assert(norm(previousCorrection ...
    - b2Target.outer_acceleration_correction_f_mps2(:), 2) <= 1.0e-12);
assert(norm(previousCorrection, 2) ...
    <= config.outer_acceleration_correction_max_mps2 + 1.0e-12);
boundedFiniteBlend = true;

%% Hard-invalid bypasses an active blend and applies exact B1 immediately.
state = stateWithActiveBlend(config);
request = gpenmpcOuterSupervisorRequest( ...
    state, true, config, eligibleShadow(config));
exactB1 = goodDecision(config.method, -0.031, [0.017; -0.023; 0.009]);
exactB1.hard_invalid = true;
exactB1.gp_b1_fallback_active = true;
audit = emptyAudit();
audit.current_hard_invalid = true;
audit.b1_fallback_active = true;
[state, hardApplied] = gpenmpcOuterSupervisorStep( ...
    state, request, exactB1, audit, config);
hardInvalidExactError = max([ ...
    abs(hardApplied.phase_acceleration_s_inv ...
        - exactB1.phase_acceleration_s_inv), ...
    abs(hardApplied.outer_acceleration_correction_f_mps2(:) ...
        - exactB1.outer_acceleration_correction_f_mps2(:)).']);
assert(hardInvalidExactError <= 1.0e-12);
assert(state.mode == "B1_ONLY");
assert(state.b1_success_dwell_ticks == 0);
assert(state.b2_probe_success_ticks == 0);
assert(~state.reentry_blend_active);
assert(state.reentry_blend_progress_ticks == 0);

%% A future suffix cannot retroactively change an already produced prefix.
[prefixStateA, prefixAppliedA, prefixEventsA] = fixedPrefix(config);
[prefixStateB, prefixAppliedB, prefixEventsB] = fixedPrefix(config);
suffixStateA = applySuffix(prefixStateA, true, config);
suffixStateB = applySuffix(prefixStateB, false, config); %#ok<NASGU>
assert(isequaln(prefixStateA, prefixStateB));
assert(isequaln(prefixAppliedA, prefixAppliedB));
assert(isequaln(prefixEventsA, prefixEventsB));
assert(~isequaln(suffixStateA, prefixStateA));
futureSuffixDoesNotChangePast = true;

%% Real frozen model: legacy/formal first step and shadow query agree.
fixturePath = fullfile(authorityRoot, "tests", "fixtures", ...
    "ordinary_b2_rollout_python_reference.json");
fixture = jsondecode(fileread(fixturePath));
modelPath = fullfile(authorityRoot, "matlab", ...
    "native_training", "raw", ...
    "MATLAB_NATIVE_SPARSE_GP_MODEL.mat");
stored = load(modelPath, "nativeModel");
realConfig = gpenmpcLoadOrdinaryB2Config(string(fixture.runtime_config_path));
trajectory = struct( ...
    "total_duration_s", double(fixture.trajectory.total_duration_s), ...
    "coefficients_ascending", double(fixture.trajectory.coefficients_ascending));
phaseState = fixture.phase_state;
observation = fixture.observation;
context = fixture.context;
context.phase_power_fcn = @fixedPower;
causal = fixture.causal_context_f17;
candidate = double(fixture.decision(:).');

legacy = gpenmpcEvaluateOrdinaryB2RolloutLegacyBaseline(candidate, ...
    realConfig.method, trajectory, phaseState, observation, context, ...
    stored.nativeModel, causal, realConfig);
formal = gpenmpcEvaluateOrdinaryB2Rollout(candidate, realConfig.method, ...
    trajectory, phaseState, observation, context, stored.nativeModel, ...
    causal, realConfig);
legacyFormalFirstStepError = commonNumericFieldError( ...
    legacy.rows(1), formal.rows(1));
legacyFormalRolloutError = max([ ...
    abs(legacy.objective - formal.objective), ...
    max(abs(legacy.constraints(:) - formal.constraints(:))), ...
    max(abs(legacy.risk_constraints_normalized(:) ...
        - formal.risk_constraints_normalized(:)))]);
assert(legacyFormalFirstStepError <= 1.0e-12);
assert(legacyFormalRolloutError <= 1.0e-12);

observedInnovationI = formal.rows(1).gp_frame_i_from_f ...
    * formal.rows(1).gp_mean_f_mps2(:);
snapshots = {candidate, trajectory, phaseState, observation, context, ...
    stored.nativeModel, causal, observedInnovationI, realConfig};
shadow = gpenmpcEvaluateShadowGpSupport(candidate, trajectory, ...
    phaseState, observation, context, stored.nativeModel, causal, ...
    observedInnovationI, true, realConfig);
shadowFormalFirstStepError = max([ ...
    max(abs(shadow.features_f17(:) ...
        - formal.rows(1).gp_features_f17(:))), ...
    max(abs(shadow.gp_frame_i_from_f(:) ...
        - formal.rows(1).gp_frame_i_from_f(:))), ...
    max(abs(shadow.predicted_mean_f_mps2(:) ...
        - formal.rows(1).gp_mean_f_mps2(:))), ...
    max(abs(shadow.calibrated_half_width_f_mps2(:) ...
        - formal.rows(1).calibrated_half_width_f_mps2(:))), ...
    abs(shadow.trust - formal.rows(1).trust), ...
    abs(shadow.support_distance - formal.rows(1).support_distance), ...
    abs(shadow.latent_variance_max ...
        - formal.rows(1).latent_variance_max)]);
assert(shadowFormalFirstStepError <= 1.0e-12);
assert(shadow.available && ~shadow.hard_invalid);
assert(shadow.trust >= realConfig.minimum_soft_trust);
assert(shadow.observed_innovation_consistent);
assert(~shadow.eligible_for_b1_dwell);
assert(shadow.read_only);
afterSnapshots = {candidate, trajectory, phaseState, observation, context, ...
    stored.nativeModel, causal, observedInnovationI, realConfig};
shadowReadOnly = isequaln(snapshots, afterSnapshots);
assert(shadowReadOnly);

result = struct;
result.schema = "GPENMPC_SHADOW_REENTRY_FOCUSED_TEST_RESULT_V1";
result.status = "PASS";
result.sustained_invalid_never_probes = sustainedInvalidNoProbe;
result.alternating_evidence_resets_dwell = alternatingReset;
result.innovation_inconsistent_resets_dwell = ...
    innovationInconsistentResetsDwell;
result.innovation_inconsistent_blocks_reentry_request = ...
    innovationInconsistentBlocksReentryRequest;
result.closed_evidence_consistency_enforced = ...
    closedEvidenceConsistencyEnforced;
result.closed_probe_reason_counted = closedProbeReasonCounted;
result.two_b1_plus_two_b2_takeover = twoPlusTwoTakeover;
result.blend_bounded_and_finitely_convergent = boundedFiniteBlend;
result.blend_update_count_to_constant_target = blendUpdateCount;
result.maximum_phase_delta_s_inv = maximumPhaseDelta;
result.phase_delta_limit_s_inv = ...
    0.5 * config.phase_acceleration_max_s_inv;
result.maximum_correction_delta_norm_mps2 = maximumCorrectionDelta;
result.correction_delta_norm_limit_mps2 = ...
    0.5 * config.outer_acceleration_correction_max_mps2;
result.final_correction_norm_mps2 = norm(previousCorrection, 2);
result.outer_correction_authority_norm_mps2 = ...
    config.outer_acceleration_correction_max_mps2;
result.hard_invalid_exact_b1_error = hardInvalidExactError;
result.future_suffix_does_not_change_past = ...
    futureSuffixDoesNotChangePast;
result.legacy_formal_first_step_max_error = ...
    legacyFormalFirstStepError;
result.legacy_formal_full_rollout_max_error = ...
    legacyFormalRolloutError;
result.shadow_formal_first_step_max_error = ...
    shadowFormalFirstStepError;
result.shadow_read_only = shadowReadOnly;
result.minimum_soft_trust = realConfig.minimum_soft_trust;
result.minimum_soft_trust_provenance = ...
    "EXISTING_RUNTIME_CALIBRATED_SOFT_TRUST";
result.fixture_path = fixturePath;
result.model_path = modelPath;
if strlength(outputPath) > 0
    parent = fileparts(outputPath);
    if ~isfolder(parent), mkdir(parent); end
    writelines(jsonencode(result, PrettyPrint=true), ...
        outputPath, Encoding="UTF-8");
end
fprintf("SHADOW_REENTRY_FOCUSED_PASS legacy=%.3e shadow=%.3e hard=%.3e\n", ...
    legacyFormalFirstStepError, shadowFormalFirstStepError, ...
    hardInvalidExactError);
end


function state = oneB1Tick(state, audit, config)
request = gpenmpcOuterSupervisorRequest( ...
    state, true, config, eligibleShadow(config));
[state, ~] = gpenmpcOuterSupervisorStep(state, request, ...
    goodDecision(config.b1_fallback_method, 0.01, [0.01; 0.0; 0.0]), ...
    audit, config);
end


function state = b1State(config)
state = gpenmpcInitializeOuterSupervisor(config);
state.mode = "B1_ONLY";
end


function state = stateWithActiveBlend(config)
state = gpenmpcInitializeOuterSupervisor(config);
state.mode = "B2_ACTIVE";
state.b1_success_dwell_ticks = 2;
state.b2_probe_success_ticks = 1;
state.reentry_blend_active = true;
state.reentry_blend_progress_ticks = 1;
state.last_applied_phase_acceleration_s_inv = -0.04;
state.last_applied_outer_correction_f_mps2 = [0.05; -0.05; 0.0];
state.last_feasible_valid = true;
state.last_feasible_phase_acceleration_s_inv = -0.04;
state.last_feasible_outer_correction_f_mps2 = [0.05; -0.05; 0.0];
end


function [state, appliedRows, eventRows] = fixedPrefix(config)
state = b1State(config);
appliedRows = cell(3, 1);
eventRows = cell(3, 1);
for index = 1:2
    request = gpenmpcOuterSupervisorRequest(state, true, config);
    [state, appliedRows{index}, eventRows{index}] = ...
        gpenmpcOuterSupervisorStep(state, request, ...
        goodDecision(config.b1_fallback_method, 0.02, [0.01;0;0]), ...
        eligibleShadowAudit(config), config);
end
request = gpenmpcOuterSupervisorRequest( ...
    state, true, config, eligibleShadow(config));
[state, appliedRows{3}, eventRows{3}] = gpenmpcOuterSupervisorStep( ...
    state, request, goodDecision(config.method, 0.08, [-0.10;0;0]), ...
    emptyAudit(), config);
end


function state = applySuffix(state, eligible, config)
request = gpenmpcOuterSupervisorRequest( ...
    state, true, config, eligibleShadow(config));
if eligible
    decision = goodDecision(config.method, 0.08, [-0.10;0;0]);
    audit = emptyAudit();
else
    decision = failedDecision(config.method, "FUTURE_SUFFIX_INVALID");
    audit = emptyAudit();
end
[state, ~] = gpenmpcOuterSupervisorStep( ...
    state, request, decision, audit, config);
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
decision.fallback_active = true;
decision.fallback_reason = "";
end


function audit = emptyAudit()
audit = struct("current_hard_invalid", false, ...
    "b1_fallback_active", false);
end


function audit = eligibleShadowAudit(config)
audit = shadowAudit(config, true, false, ...
    config.minimum_soft_trust, true);
end


function shadow = eligibleShadow(config)
audit = eligibleShadowAudit(config);
shadow = audit.shadow_gp;
end


function shadow = inconsistentShadow(config)
audit = inconsistentShadowAudit(config);
shadow = audit.shadow_gp;
end


function audit = invalidShadowAudit(config)
audit = shadowAudit(config, true, true, 0.0, false);
end


function audit = inconsistentShadowAudit(config)
audit = shadowAudit(config, true, false, 0.8, false);
end


function audit = shadowAudit(config, available, hardInvalid, trust, consistent)
audit = emptyAudit();
consistencyScore = 0.10;
if consistent
    consistencyScore = 0.80;
end
audit.shadow_gp = struct( ...
    "available", logical(available), ...
    "causal_valid", true, ...
    "observed_innovation_available", logical(available), ...
    "gp_model_available", logical(available), ...
    "hard_invalid", logical(hardInvalid), ...
    "trust", double(trust), ...
    "minimum_soft_trust", config.minimum_soft_trust, ...
    "observed_innovation_consistent", logical(consistent), ...
    "consistency_ewma_f", consistencyScore .* ones(1,3), ...
    "consistency_ewma_aggregate", consistencyScore);
end


function maximum = commonNumericFieldError(left, right)
maximum = 0.0;
names = intersect(fieldnames(left), fieldnames(right));
for index = 1:numel(names)
    name = names{index};
    leftValue = left.(name);
    rightValue = right.(name);
    if (isnumeric(leftValue) || islogical(leftValue)) ...
            && (isnumeric(rightValue) || islogical(rightValue))
        maximum = max(maximum, ...
            max(abs(double(leftValue(:)) - double(rightValue(:)))));
    end
end
end


function powerW = fixedPower(phase, airspeed, payload)
phaseDelta = 15.0 .* double(string(phase) == "ASCEND") ...
    + 8.0 .* double(string(phase) == "DESCEND");
powerW = 1050.0 + 9.0 .* airspeed.^2 + 20.0 .* payload + phaseDelta;
end
