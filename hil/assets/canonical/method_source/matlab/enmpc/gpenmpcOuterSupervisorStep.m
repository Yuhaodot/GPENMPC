function [state, applied, event] = gpenmpcOuterSupervisorStep( ...
        state, request, decision, audit, config, currentPhaseRate)
%GPENMPCOUTERSUPERVISORSTEP Apply deterministic fail-closed mode logic.
%
% A failed B2 update may reuse the last feasible target for one outer tick.
% The following update is a fresh B1 request. A failed B1 update receives at
% most the same one-tick hold and then the fixed C3 reference (zero outer
% correction and zero phase acceleration). Re-entry requires the configured
% B1 dwell and consecutive successful B2 probes.

arguments
    state (1,1) struct
    request (1,1) struct
    decision (1,1) struct
    audit (1,1) struct
    config (1,1) struct
    currentPhaseRate (1,1) double = 1.0
end
modeBefore = string(state.mode);
validateInvocation(state, request, decision, config, currentPhaseRate);
state.tick_index = state.tick_index + 1;
rawSuccess = logicalField(decision, "success", false);
rawHardInvalid = logicalField(decision, "hard_invalid", false) ...
    || logicalField(audit, "current_hard_invalid", false);
gpB1Fallback = logicalField(decision, "gp_b1_fallback_active", false) ...
    || logicalField(audit, "b1_fallback_active", false);
rawReason = textField(decision, "fallback_reason", "");
requestedMethod = string(request.requested_method);
isB1Request = requestedMethod == config.b1_fallback_method;
isB2Request = requestedMethod == config.method;
if ~(isB1Request || isB2Request)
    error("gpenmpcOuterSupervisorStep:Request", ...
        "Supervisor requested an unknown method %s.", requestedMethod);
end
shadow = shadowDwellStatus(audit, request, config);
[state, consistencyExitTriggered] = updateConsistencyExitState( ...
    state, shadow, modeBefore, isB2Request, config);

transitionReason = "NO_MODE_CHANGE";
if isB1Request
    state.b1_direct_request_count = state.b1_direct_request_count + 1;
    if rawSuccess
        applied = rawApplied(decision, "B1_FULL_BUDGET_DECISION", config);
        state = rememberFeasible(state, applied);
        state = resetReentryBlend(state);
        state.mode = "B1_ONLY";
        state.b2_probe_success_ticks = 0;
        if shadow.eligible
            state.b1_success_dwell_ticks = state.b1_success_dwell_ticks + 1;
            state.shadow_dwell_accept_count = ...
                state.shadow_dwell_accept_count + 1;
        else
            state.b1_success_dwell_ticks = 0;
            state.shadow_dwell_reset_count = ...
                state.shadow_dwell_reset_count + 1;
        end
        if state.b1_success_dwell_ticks >= state.minimum_b1_dwell_ticks
            state.mode = "B2_REENTRY_PROBE";
            state.b2_probe_success_ticks = 0;
            transitionReason = "B1_DWELL_COMPLETE";
        elseif ~shadow.eligible
            transitionReason = shadow.rejection_reason;
        else
            transitionReason = "B1_DWELL_CONTINUES";
        end
    else
        [state, applied] = failClosedAfterFailure( ...
            state, config, currentPhaseRate);
        state.mode = "FIXED_REFERENCE_HOLD";
        state.b1_success_dwell_ticks = 0;
        state.b2_probe_success_ticks = 0;
        state = resetReentryBlend(state);
        transitionReason = "B1_FAILED__" + normalizedReason(rawReason);
    end
elseif rawSuccess && ~gpB1Fallback && ~rawHardInvalid
    if modeBefore == "B2_REENTRY_PROBE"
        state.b2_probe_request_count = state.b2_probe_request_count + 1;
        state.b2_probe_success_ticks = state.b2_probe_success_ticks + 1;
        if state.b2_probe_success_ticks < state.required_b2_probe_success_ticks
            applied = persistentProbeHold(state, config);
            state.mode = "B2_REENTRY_PROBE";
            transitionReason = "B2_PROBE_SUCCESS_PERSISTENCE_PENDING";
        else
            [state, applied] = boundedReentryApplied( ...
                state, decision, config);
            state = rememberFeasible(state, applied);
            state.mode = "B2_ACTIVE";
            state.b1_success_dwell_ticks = 0;
            state.b2_probe_success_ticks = 0;
            state.reentry_count = state.reentry_count + 1;
            transitionReason = "B2_REENTRY_PERSISTENCE_SATISFIED";
        end
    else
        if state.reentry_blend_active
            [state, applied] = boundedReentryApplied( ...
                state, decision, config);
        else
            applied = rawApplied(decision, "B2_ACTIVE_DECISION", config);
        end
        state = rememberFeasible(state, applied);
        state.mode = "B2_ACTIVE";
        state.b1_success_dwell_ticks = 0;
        state.b2_probe_success_ticks = 0;
        if consistencyExitTriggered
            state.mode = "B1_ONLY";
            state.innovation_exit_count = state.innovation_exit_count + 1;
            state = resetReentryBlend(state);
            transitionReason = "B2_CONSISTENCY_EXIT_DWELL_COMPLETE";
        end
    end
elseif gpB1Fallback && rawSuccess
    % The ordinary-B2 core already returned the exact B1 decision. Apply it
    % and keep subsequent updates on the fresh, full-budget B1 path.
    if rawHardInvalid
        source = "HARD_INVALID_EXACT_B1_DECISION";
    else
        source = "B2_INFEASIBLE_EXACT_B1_DECISION";
    end
    applied = rawApplied(decision, source, config);
    state = rememberFeasible(state, applied);
    state.mode = "B1_ONLY";
    state.b1_success_dwell_ticks = 0;
    state.b2_probe_success_ticks = 0;
    state = resetReentryBlend(state);
    if rawHardInvalid
        transitionReason = "GP_HARD_INVALID_TO_B1";
    else
        transitionReason = "B2_INFEASIBLE_EXACT_B1_WITHIN_DEADLINE";
    end
else
    % Includes B2 deadline, B2 infeasibility, and a failed inline B1
    % fallback. Ignore any unavailable target, hold once if possible, then
    % force the next tick to use a fresh B1 solve.
    [state, applied] = failClosedAfterFailure( ...
        state, config, currentPhaseRate);
    state.mode = "B1_ONLY";
    state.b1_success_dwell_ticks = 0;
    state.b2_probe_success_ticks = 0;
    state = resetReentryBlend(state);
    if rawHardInvalid
        transitionReason = "GP_HARD_INVALID_B1_UNAVAILABLE";
    elseif gpB1Fallback
        transitionReason = "B2_INFEASIBLE_INLINE_B1_UNAVAILABLE";
    else
        transitionReason = "B2_FAILED__" + normalizedReason(rawReason);
    end
end

state.last_applied_phase_acceleration_s_inv = ...
    applied.phase_acceleration_s_inv;
state.last_applied_outer_correction_f_mps2 = ...
    applied.outer_acceleration_correction_f_mps2;
if state.mode ~= modeBefore
    state.transition_count = state.transition_count + 1;
end
state.last_transition_reason = transitionReason;
event = buildEvent(state, request, decision, audit, modeBefore, ...
    transitionReason, rawSuccess, rawHardInvalid, gpB1Fallback, applied, shadow);
end


function [state, triggered] = updateConsistencyExitState( ...
        state, shadow, modeBefore, isB2Request, config)
triggered = false;
if continuousAxisWeightSupervision(config)
    state.innovation_exit_low_ticks = 0;
    return
end
if modeBefore ~= "B2_ACTIVE" || ~isB2Request
    state.innovation_exit_low_ticks = 0;
    return
end

if ~shadow.available || shadow.hard_invalid
    state.innovation_exit_low_ticks = 0;
    return
end
if shadow.consistency_aggregate < consistencyExitThreshold(config)
    state.innovation_exit_low_ticks = state.innovation_exit_low_ticks + 1;
else
    state.innovation_exit_low_ticks = 0;
end
triggered = state.innovation_exit_low_ticks ...
    >= consistencyExitDwellTicks(config);
end


function enabled = continuousAxisWeightSupervision(config)
enabled = isfield(config, "continuous_axis_weight_supervision_enabled") ...
    && logical(config.continuous_axis_weight_supervision_enabled);
end


function [state, applied] = failClosedAfterFailure( ...
        state, config, currentPhaseRate)
if state.last_feasible_valid && state.failure_hold_available
    applied = targetApplied( ...
        state.last_feasible_phase_acceleration_s_inv, ...
        state.last_feasible_outer_correction_f_mps2, ...
        "LAST_FEASIBLE_ONE_TICK_HOLD", true, false, config);
    state.failure_hold_available = false;
    state.failure_hold_count = state.failure_hold_count + 1;
else
    phaseAcceleration = fixedReferencePhaseAcceleration( ...
        currentPhaseRate, config);
    applied = targetApplied(phaseAcceleration, zeros(3, 1), ...
        "FIXED_C3_REFERENCE", false, true, config);
    state.fixed_reference_count = state.fixed_reference_count + 1;
end


function phaseAcceleration = fixedReferencePhaseAcceleration(currentRate, config)
targetRate = 1.0;
if isfield(config, "raw") && isfield(config.raw, "bumpless_recovery") ...
        && isfield(config.raw.bumpless_recovery, "target_recovery_rate")
    targetRate = double(config.raw.bumpless_recovery.target_recovery_rate);
end
if ~isfinite(targetRate) || targetRate <= 0.0
    error("gpenmpcOuterSupervisorStep:RecoveryRate", ...
        "The fixed-reference recovery rate must be finite and positive.");
end
period = 1.0;
if isfield(config, "outer_period_s")
    period = double(config.outer_period_s);
end
if ~isfinite(period) || period <= 0.0
    error("gpenmpcOuterSupervisorStep:OuterPeriod", ...
        "The outer period must be finite and positive.");
end
unbounded = (targetRate - double(currentRate)) ./ period;
limit = double(config.phase_acceleration_max_s_inv);
phaseAcceleration = min(max(unbounded, -limit), limit);
end
end


function applied = persistentProbeHold(state, config)
if state.last_feasible_valid
    applied = targetApplied( ...
        state.last_feasible_phase_acceleration_s_inv, ...
        state.last_feasible_outer_correction_f_mps2, ...
        "B1_REENTRY_PERSISTENCE_HOLD", true, false, config);
else
    applied = targetApplied(0.0, zeros(3, 1), ...
        "FIXED_C3_REFERENCE", false, true, config);
end
end


function [state, applied] = boundedReentryApplied(state, decision, config)
raw = rawApplied(decision, "B2_REENTRY_RAW_TARGET", config);
phaseStep = 0.5 .* double(config.phase_acceleration_max_s_inv);
correctionStep = 0.5 .* ...
    double(config.outer_acceleration_correction_max_mps2);
previousPhase = double(state.last_applied_phase_acceleration_s_inv);
previousCorrection = ...
    double(state.last_applied_outer_correction_f_mps2(:));
rawPhaseDelta = raw.phase_acceleration_s_inv - previousPhase;
phaseDelta = min(max(rawPhaseDelta, -phaseStep), phaseStep);
rawCorrectionDelta = raw.outer_acceleration_correction_f_mps2 ...
    - previousCorrection;
correctionDelta = clipNorm(rawCorrectionDelta, correctionStep);
applied = targetApplied(previousPhase + phaseDelta, ...
    previousCorrection + correctionDelta, ...
    "B2_REENTRY_BLEND_DECISION", false, false, config);
applied.reentry_blend_applied = true;
applied.phase_delta_from_previous_s_inv = phaseDelta;
applied.correction_delta_norm_from_previous_mps2 = ...
    norm(correctionDelta, 2);
reached = abs(applied.phase_acceleration_s_inv ...
    - raw.phase_acceleration_s_inv) <= 1.0e-12 ...
    && norm(applied.outer_acceleration_correction_f_mps2 ...
    - raw.outer_acceleration_correction_f_mps2, 2) <= 1.0e-12;
state.reentry_blend_active = ~reached;
if reached
    state.reentry_blend_progress_ticks = 0;
else
    state.reentry_blend_progress_ticks = ...
        state.reentry_blend_progress_ticks + 1;
end
end


function state = resetReentryBlend(state)
state.reentry_blend_active = false;
state.reentry_blend_progress_ticks = 0;
end


function shadow = shadowDwellStatus(audit, request, config)
shadow = struct( ...
    "eligible", false, ...
    "available", false, ...
    "hard_invalid", false, ...
    "trust", 0.0, ...
    "minimum_soft_trust", existingMinimumSoftTrust(config), ...
    "observed_innovation_consistent", false, ...
    "consistency_aggregate", 0.0, ...
    "enter_threshold", consistencyEnterThreshold(config), ...
    "exit_threshold", consistencyExitThreshold(config), ...
    "rejection_reason", "B1_SHADOW_UNAVAILABLE_RESET");
if ~logical(request.causal_valid)
    shadow.rejection_reason = "STALE_CONTEXT_DIRECT_B1";
    return
end
if ~isfield(audit, "shadow_gp") || ~isstruct(audit.shadow_gp)
    return
end
source = audit.shadow_gp;
shadow.available = logicalField(source, "available", false) ...
    && logicalField(source, "causal_valid", false) ...
    && logicalField(source, "observed_innovation_available", false) ...
    && logicalField(source, "gp_model_available", false);
shadow.hard_invalid = logicalField(source, "hard_invalid", true);
shadow.trust = numericField(source, "trust", 0.0);
shadow.observed_innovation_consistent = logicalField( ...
    source, "observed_innovation_consistent", false);
shadow.consistency_aggregate = consistencyAggregate(source);
if ~shadow.available
    shadow.rejection_reason = "B1_SHADOW_UNAVAILABLE_RESET";
elseif shadow.hard_invalid
    shadow.rejection_reason = "B1_SHADOW_HARD_INVALID_RESET";
elseif ~isfinite(shadow.trust) ...
        || shadow.trust < shadow.minimum_soft_trust
    shadow.rejection_reason = "B1_SHADOW_TRUST_BELOW_MINIMUM_RESET";
elseif shadow.consistency_aggregate < shadow.enter_threshold
    shadow.rejection_reason = ...
        "B1_SHADOW_CONSISTENCY_EWMA_BELOW_ENTER_RESET";
else
    shadow.eligible = true;
    shadow.rejection_reason = "";
end
end


function value = consistencyAggregate(source)
if isfield(source, "consistency_ewma_aggregate")
    value = double(source.consistency_ewma_aggregate);
elseif isfield(source, "consistency_ewma_f")
    value = mean(double(source.consistency_ewma_f(:)));
else
    value = 0.0;
end
if ~isscalar(value) || ~isfinite(value)
    value = 0.0;
end
end


function value = consistencyEnterThreshold(config)
if isfield(config, "innovation_consistency_enter_threshold")
    value = double(config.innovation_consistency_enter_threshold);
else
    value = 0.50;
end
end


function value = consistencyExitThreshold(config)
if isfield(config, "innovation_consistency_exit_threshold")
    value = double(config.innovation_consistency_exit_threshold);
else
    value = 0.25;
end
end


function value = consistencyExitDwellTicks(config)
if isfield(config, "innovation_consistency_exit_dwell_outer_ticks")
    value = double(config.innovation_consistency_exit_dwell_outer_ticks);
else
    value = 2;
end
if value < 1 || value ~= round(value)
    error("gpenmpcOuterSupervisorStep:ConsistencyExitDwell", ...
        "Consistency exit dwell must be a positive integer.");
end
end


function value = existingMinimumSoftTrust(config)
if isfield(config, "minimum_soft_trust")
    value = double(config.minimum_soft_trust);
elseif isfield(config, "raw") ...
        && isfield(config.raw, "uncertainty_tightening") ...
        && isfield(config.raw.uncertainty_tightening, "minimum_soft_trust")
    value = double(config.raw.uncertainty_tightening.minimum_soft_trust);
else
    error("gpenmpcOuterSupervisorStep:MinimumTrust", ...
        "The existing calibrated minimum_soft_trust is required.");
end
if ~isscalar(value) || ~isfinite(value) || value < 0.0 || value > 1.0
    error("gpenmpcOuterSupervisorStep:MinimumTrust", ...
        "minimum_soft_trust must be one finite probability-scale value.");
end
end


function state = rememberFeasible(state, applied)
state.last_feasible_valid = true;
state.last_feasible_phase_acceleration_s_inv = ...
    applied.phase_acceleration_s_inv;
state.last_feasible_outer_correction_f_mps2 = ...
    applied.outer_acceleration_correction_f_mps2;
state.failure_hold_available = true;
end


function applied = rawApplied(decision, source, config)
applied = targetApplied(double(decision.phase_acceleration_s_inv), ...
    double(decision.outer_acceleration_correction_f_mps2(:)), ...
    source, false, false, config);
end


function applied = targetApplied(phaseAcceleration, correctionF, source, ...
        holdApplied, fixedApplied, config)
phaseAcceleration = double(phaseAcceleration);
correctionF = double(correctionF(:));
if ~isfinite(phaseAcceleration) || numel(correctionF) ~= 3 ...
        || any(~isfinite(correctionF))
    error("gpenmpcOuterSupervisorStep:Finite", ...
        "Supervisor target must be finite with one phase and three correction values.");
end
tolerance = 1.0e-12;
if abs(phaseAcceleration) > config.phase_acceleration_max_s_inv + tolerance
    error("gpenmpcOuterSupervisorStep:PhaseAuthority", ...
        "Supervisor may not increase the frozen phase-acceleration authority.");
end
if norm(correctionF, 2) ...
        > config.outer_acceleration_correction_max_mps2 + tolerance
    error("gpenmpcOuterSupervisorStep:CorrectionAuthority", ...
        "Supervisor may not increase the frozen outer-correction authority.");
end
applied = struct;
applied.schema = "GPENMPC_OUTER_SUPERVISOR_APPLIED_TARGET_V1";
applied.phase_acceleration_s_inv = phaseAcceleration;
applied.outer_acceleration_correction_f_mps2 = correctionF;
applied.source = string(source);
applied.hold_applied = logical(holdApplied);
applied.fixed_reference_applied = logical(fixedApplied);
applied.reentry_blend_applied = false;
applied.phase_delta_from_previous_s_inv = NaN;
applied.correction_delta_norm_from_previous_mps2 = NaN;
end


function clipped = clipNorm(vector, maximumNorm)
vector = double(vector(:));
vectorNorm = norm(vector, 2);
if vectorNorm > maximumNorm
    clipped = vector .* (maximumNorm ./ max(vectorNorm, 1.0e-15));
else
    clipped = vector;
end
end


function event = buildEvent(state, request, decision, audit, modeBefore, ...
        reason, rawSuccess, rawHardInvalid, gpB1Fallback, applied, shadow)
event = struct;
event.schema = "GPENMPC_OUTER_SUPERVISOR_EVENT_V1";
event.tick_index = state.tick_index;
event.mode_before = modeBefore;
event.mode_after = string(state.mode);
event.transitioned = modeBefore ~= state.mode;
event.transition_reason = reason;
event.requested_method = string(request.requested_method);
event.request_reason = string(request.request_reason);
event.causal_valid = logical(request.causal_valid);
event.raw_method = textField(decision, "method", "");
event.raw_success = rawSuccess;
event.raw_hard_invalid = rawHardInvalid;
event.raw_gp_b1_fallback = gpB1Fallback;
event.raw_fallback_reason = textField(decision, "fallback_reason", "");
event.raw_b1_fallback_trigger = ...
    textField(audit, "b1_fallback_trigger", "");
event.raw_elapsed_seconds = numericField(decision, "elapsed_seconds", NaN);
event.raw_mean_trust = numericField(decision, "mean_trust", 0.0);
event.raw_minimum_constraint_margin = ...
    numericField(decision, "minimum_constraint_margin", NaN);
event.raw_best_minimum_constraint_margin = ...
    numericField(audit, "best_minimum_constraint_margin", NaN);
event.raw_argmin_constraint_name = ...
    textField(audit, "argmin_constraint_name", "UNAVAILABLE");
event.raw_argmin_horizon_step = ...
    numericField(audit, "argmin_horizon_step", 0);
event.failure_warm_start_preserved = ...
    logicalField(audit, "failure_warm_start_preserved", false);
if isfield(audit, "b1")
    event.inline_b1_best_minimum_constraint_margin = ...
        numericField(audit.b1, "best_minimum_constraint_margin", NaN);
    event.inline_b1_argmin_constraint_name = ...
        textField(audit.b1, "argmin_constraint_name", "UNAVAILABLE");
    event.inline_b1_argmin_horizon_step = ...
        numericField(audit.b1, "argmin_horizon_step", 0);
else
    event.inline_b1_best_minimum_constraint_margin = NaN;
    event.inline_b1_argmin_constraint_name = "UNAVAILABLE";
    event.inline_b1_argmin_horizon_step = 0;
end
event.applied_source = applied.source;
event.hold_applied = applied.hold_applied;
event.fixed_reference_applied = applied.fixed_reference_applied;
event.reentry_blend_applied = applied.reentry_blend_applied;
event.reentry_blend_active_after = state.reentry_blend_active;
event.phase_delta_from_previous_s_inv = ...
    applied.phase_delta_from_previous_s_inv;
event.correction_delta_norm_from_previous_mps2 = ...
    applied.correction_delta_norm_from_previous_mps2;
event.applied_phase_acceleration_s_inv = ...
    applied.phase_acceleration_s_inv;
event.applied_outer_correction_f_mps2 = ...
    applied.outer_acceleration_correction_f_mps2(:).';
event.b1_success_dwell_ticks = state.b1_success_dwell_ticks;
event.b2_probe_success_ticks = state.b2_probe_success_ticks;
event.shadow_available = shadow.available;
event.shadow_hard_invalid = shadow.hard_invalid;
event.shadow_trust = shadow.trust;
event.shadow_minimum_soft_trust = shadow.minimum_soft_trust;
event.shadow_observed_innovation_consistent = ...
    shadow.observed_innovation_consistent;
event.shadow_consistency_aggregate = shadow.consistency_aggregate;
event.shadow_consistency_enter_threshold = shadow.enter_threshold;
event.shadow_consistency_exit_threshold = shadow.exit_threshold;
event.shadow_eligible_for_b1_dwell = shadow.eligible;
event.shadow_rejection_reason = shadow.rejection_reason;
event.innovation_exit_low_ticks = state.innovation_exit_low_ticks;
event.innovation_exit_count = state.innovation_exit_count;
end


function value = logicalField(source, name, fallback)
if isfield(source, name)
    value = logical(source.(name));
else
    value = logical(fallback);
end
end


function value = numericField(source, name, fallback)
if isfield(source, name)
    value = double(source.(name));
else
    value = double(fallback);
end
end


function value = textField(source, name, fallback)
if isfield(source, name)
    value = string(source.(name));
else
    value = string(fallback);
end
end


function value = normalizedReason(reason)
value = string(reason);
if strlength(value) == 0
    value = "UNSPECIFIED_FAILURE";
end
end


function validateInvocation(state, request, decision, config, currentPhaseRate)
if string(request.mode_before) ~= string(state.mode)
    error("gpenmpcOuterSupervisorStep:StaleRequest", ...
        "The request mode does not match the current supervisor state.");
end
if ~isfinite(currentPhaseRate) || currentPhaseRate < 0.0
    error("gpenmpcOuterSupervisorStep:PhaseRate", ...
        "Current phase rate must be finite and nonnegative.");
end
phaseLimit = double(config.phase_acceleration_max_s_inv);
correctionLimit = double(config.outer_acceleration_correction_max_mps2);
if ~isscalar(phaseLimit) || ~isfinite(phaseLimit) || phaseLimit < 0.0 ...
        || ~isscalar(correctionLimit) || ~isfinite(correctionLimit) ...
        || correctionLimit < 0.0
    error("gpenmpcOuterSupervisorStep:AuthorityConfig", ...
        "Frozen authority bounds must be finite nonnegative scalars.");
end
requestedMethod = string(request.requested_method);
decisionMethod = textField(decision, "method", "");
if requestedMethod == config.method
    if decisionMethod ~= config.method
        error("gpenmpcOuterSupervisorStep:DecisionIdentity", ...
            "A B2 request must return a B2-identity decision.");
    end
elseif requestedMethod == config.b1_fallback_method
    if decisionMethod ~= config.b1_fallback_method
        error("gpenmpcOuterSupervisorStep:DecisionIdentity", ...
            "A direct B1 request must return a B1-identity decision.");
    end
else
    error("gpenmpcOuterSupervisorStep:RequestIdentity", ...
        "The requested method is outside the frozen B1/B2 pair.");
end
if ~logical(request.causal_valid) ...
        && requestedMethod ~= config.b1_fallback_method
    error("gpenmpcOuterSupervisorStep:StaleCausalRequest", ...
        "A stale causal context must request B1 directly.");
end
end
