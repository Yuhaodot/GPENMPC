function [state, applied, event] = gpenmpcApplyCommandContinuity( ...
        state, decision, audit, requestedMethod, config)
%GPENMPCAPPLYCOMMANDCONTINUITY Apply one-period fail-closed continuity.

arguments
    state (1,1) struct
    decision (1,1) struct
    audit (1,1) struct
    requestedMethod (1,1) string
    config (1,1) struct
end

state.update_index = state.update_index + 1;
previousSource = string(state.previous_command_source);
rawSuccess = logicalField(decision, "success", false);
rawHardInvalid = logicalField(decision, "hard_invalid", false) ...
    || logicalField(audit, "current_hard_invalid", false);
isRequestedB1 = requestedMethod == string(config.b1_fallback_method);
isRequestedB2 = requestedMethod == string(config.method);
if ~(isRequestedB1 || isRequestedB2)
    error("gpenmpcApplyCommandContinuity:RequestedIdentity", ...
        "Unknown requested method %s.", requestedMethod);
end

rawMethod = string(fieldOr(decision, "method", ""));
inlineB1 = isRequestedB2 && ( ...
    logicalField(decision, "gp_b1_fallback_active", false) ...
    || logicalField(audit, "b1_fallback_active", false) ...
    || (isfield(audit, "coordinated_selected_method") ...
        && string(audit.coordinated_selected_method) ...
            == string(config.b1_fallback_method)));
validIdentity = (isRequestedB1 ...
        && rawMethod == string(config.b1_fallback_method)) ...
    || (isRequestedB2 && (rawMethod == string(config.method) ...
        || (inlineB1 && rawMethod == string(config.b1_fallback_method))));
if ~validIdentity
    error("gpenmpcApplyCommandContinuity:DecisionIdentity", ...
        "Decision method %s is inconsistent with requested method %s.", ...
        rawMethod, requestedMethod);
end

rawPhase = numericField(decision, "phase_acceleration_s_inv", 0.0);
rawCorrection = vector3Field(decision, ...
    "outer_acceleration_correction_f_mps2", zeros(3,1));
validateAuthority(rawPhase, rawCorrection, config);

if rawSuccess
    applied.phase_acceleration_s_inv = rawPhase;
    applied.outer_acceleration_correction_f_mps2 = rawCorrection;
    applied.hold_applied = false;
    applied.safe_neutral_applied = false;
    applied.hard_invalid_termination = false;
    if inlineB1
        source = "EXACT_B1_FALLBACK";
        state.exact_b1_fallback_count = state.exact_b1_fallback_count + 1;
    else
        source = "NEW_FEASIBLE_CANDIDATE";
        state.new_feasible_count = state.new_feasible_count + 1;
    end
    state.last_feasible_valid = true;
    state.last_feasible_phase_acceleration_s_inv = rawPhase;
    state.last_feasible_outer_correction_f_mps2 = rawCorrection;
    state.one_period_hold_available = true;
elseif rawHardInvalid
    applied = neutralApplied(true);
    source = "HARD_INVALID_TERMINATION";
    state.one_period_hold_available = false;
    state.hard_invalid_termination_count = ...
        state.hard_invalid_termination_count + 1;
elseif state.last_feasible_valid && state.one_period_hold_available
    applied.phase_acceleration_s_inv = ...
        state.last_feasible_phase_acceleration_s_inv;
    applied.outer_acceleration_correction_f_mps2 = ...
        state.last_feasible_outer_correction_f_mps2;
    applied.hold_applied = true;
    applied.safe_neutral_applied = false;
    applied.hard_invalid_termination = false;
    source = "ONE_PERIOD_LAST_FEASIBLE_HOLD";
    state.one_period_hold_available = false;
    state.one_period_hold_count = state.one_period_hold_count + 1;
else
    applied = neutralApplied(false);
    source = "SAFE_NEUTRAL_COMMAND";
    state.one_period_hold_available = false;
    state.safe_neutral_count = state.safe_neutral_count + 1;
end
applied.schema = "GPENMPC_APPLIED_OUTER_COMMAND_V1";
applied.source = source;
applied.source_code = sourceCode(source, config);
validateAuthority(applied.phase_acceleration_s_inv, ...
    applied.outer_acceleration_correction_f_mps2, config);

event = buildEvent(state.update_index, requestedMethod, decision, audit, ...
    previousSource, rawPhase, rawCorrection, applied, rawSuccess, ...
    rawHardInvalid, inlineB1);
state.previous_command_source = source;
end


function applied = neutralApplied(hardInvalid)
applied = struct( ...
    "phase_acceleration_s_inv", 0.0, ...
    "outer_acceleration_correction_f_mps2", zeros(3,1), ...
    "hold_applied", false, ...
    "safe_neutral_applied", ~hardInvalid, ...
    "hard_invalid_termination", hardInvalid);
end


function event = buildEvent(index, requestedMethod, decision, audit, ...
        previousSource, rawPhase, rawCorrection, applied, rawSuccess, ...
        rawHardInvalid, inlineB1)
event = struct;
event.schema = "GPENMPC_OUTER_UPDATE_EVENT_V1";
event.update_index = index;
event.requested_method = requestedMethod;
event.raw_method = string(fieldOr(decision, "method", ""));
event.raw_success = rawSuccess;
event.raw_hard_invalid = rawHardInvalid;
event.raw_inline_exact_b1_fallback = inlineB1;
event.raw_fallback_reason = string(fieldOr(decision, "fallback_reason", ""));
event.raw_elapsed_seconds = numericField(decision, "elapsed_seconds", NaN);
event.raw_selected_candidate_index = numericField(audit, "selected_index", 0);
event.raw_phase_acceleration_s_inv = rawPhase;
event.raw_outer_correction_f_mps2 = rawCorrection(:).';
event.previous_command_source = previousSource;
event.applied_command_source = applied.source;
event.applied_command_source_code = applied.source_code;
event.applied_phase_acceleration_s_inv = ...
    applied.phase_acceleration_s_inv;
event.applied_outer_correction_f_mps2 = ...
    applied.outer_acceleration_correction_f_mps2(:).';
event.hold_applied = applied.hold_applied;
event.safe_neutral_applied = applied.safe_neutral_applied;
event.hard_invalid_termination = applied.hard_invalid_termination;
event.closest_candidate_index = numericField(audit, ...
    "best_candidate_index_by_margin", 0);
event.closest_candidate_minimum_margin = numericField(audit, ...
    "best_minimum_constraint_margin", NaN);
event.dominant_constraint = string(fieldOr(audit, ...
    "argmin_constraint_name", "UNAVAILABLE"));
event.dominant_constraint_horizon_step = numericField(audit, ...
    "argmin_horizon_step", 0);
event.b1_closest_candidate_index = 0;
event.b1_closest_candidate_minimum_margin = NaN;
event.b1_dominant_constraint = "NOT_APPLICABLE";
event.b1_dominant_constraint_horizon_step = 0;
if isfield(audit, "b1") && isstruct(audit.b1)
    event.b1_closest_candidate_index = numericField(audit.b1, ...
        "best_candidate_index_by_margin", 0);
    event.b1_closest_candidate_minimum_margin = numericField(audit.b1, ...
        "best_minimum_constraint_margin", NaN);
    event.b1_dominant_constraint = string(fieldOr(audit.b1, ...
        "argmin_constraint_name", "UNAVAILABLE"));
    event.b1_dominant_constraint_horizon_step = numericField(audit.b1, ...
        "argmin_horizon_step", 0);
end
event.next_update_recovery = "NOT_OBSERVED_END_OF_CASE";
event.next_update_raw_success = false;
end


function code = sourceCode(source, config)
book = config.command_source_codebook;
if ~isfield(book, source)
    error("gpenmpcApplyCommandContinuity:SourceCode", ...
        "Unknown command source %s.", source);
end
code = double(book.(source));
end


function validateAuthority(phaseAcceleration, correctionF, config)
tolerance = 1.0e-12;
if ~isscalar(phaseAcceleration) || ~isfinite(phaseAcceleration) ...
        || abs(phaseAcceleration) ...
            > double(config.phase_acceleration_max_s_inv) + tolerance
    error("gpenmpcApplyCommandContinuity:PhaseAuthority", ...
        "Outer phase acceleration is nonfinite or outside frozen authority.");
end
correctionF = double(correctionF(:));
if numel(correctionF) ~= 3 || any(~isfinite(correctionF)) ...
        || norm(correctionF, 2) ...
            > double(config.outer_acceleration_correction_max_mps2) + tolerance
    error("gpenmpcApplyCommandContinuity:CorrectionAuthority", ...
        "Outer correction is nonfinite or outside frozen authority.");
end
end


function value = vector3Field(source, name, fallback)
value = double(fallback(:));
if isstruct(source) && isfield(source, name)
    candidate = double(source.(name));
    if numel(candidate) == 3
        value = candidate(:);
    end
end
end


function value = numericField(source, name, fallback)
value = double(fallback);
if isstruct(source) && isfield(source, name)
    candidate = double(source.(name));
    if isscalar(candidate)
        value = candidate;
    end
end
end


function value = logicalField(source, name, fallback)
value = logical(fallback);
if isstruct(source) && isfield(source, name)
    candidate = logical(source.(name));
    if isscalar(candidate)
        value = candidate;
    end
end
end


function value = fieldOr(source, name, fallback)
value = fallback;
if isstruct(source) && isfield(source, name)
    value = source.(name);
end
end
