function request = gpenmpcOuterSupervisorRequest( ...
        state, causalValid, config, activeShadow)
%GPENMPCOUTERSUPERVISORREQUEST Select B2 or full-budget B1 before solving.

arguments
    state (1,1) struct
    causalValid (1,1) logical
    config (1,1) struct
    activeShadow (1,1) struct = struct
end
validateState(state);
request = struct;
request.schema = "GPENMPC_OUTER_SUPERVISOR_REQUEST_V1";
request.mode_before = string(state.mode);
request.causal_valid = causalValid;
request.requested_method = config.method;
request.request_reason = "B2_ACTIVE";

% In execution-alignment mode, the causal per-axis GP agreement weights govern
% soft adoption and withdrawal. The state machine handles missing causal
% context, hard-invalid B2 fallback and solver failure.
if continuousAxisWeightSupervision(config)
    if ~causalValid
        request.requested_method = config.b1_fallback_method;
        request.request_reason = "GP_CAUSAL_CONTEXT_STALE_DIRECT_B1";
    else
        request.requested_method = config.method;
        request.request_reason = "CONTINUOUS_AXIS_WEIGHTED_B2";
    end
    return
end

if ~causalValid
    request.requested_method = config.b1_fallback_method;
    request.request_reason = "GP_CAUSAL_CONTEXT_STALE_DIRECT_B1";
elseif state.mode == "B1_ONLY"
    request.requested_method = config.b1_fallback_method;
    request.request_reason = "B1_RECOVERY_DWELL";
elseif state.mode == "FIXED_REFERENCE_HOLD"
    request.requested_method = config.b1_fallback_method;
    request.request_reason = "B1_RECOVERY_FROM_FIXED_REFERENCE";
elseif state.mode == "B2_REENTRY_PROBE"
        if shadowEligibleForReentry(activeShadow, config)
            request.requested_method = config.method;
            request.request_reason = "B2_REENTRY_PROBE_WITH_CLOSED_GP_EVIDENCE";
        else
            request.requested_method = config.b1_fallback_method;
            request.request_reason = "B2_REENTRY_" ...
                + reentryShadowReason(activeShadow, config);
        end
elseif state.mode == "B2_ACTIVE" ...
        && ~shadowUsableForActiveUse(activeShadow, config)
        request.requested_method = config.b1_fallback_method;
        request.request_reason = activeShadowReason(activeShadow, config);
end

end


function enabled = continuousAxisWeightSupervision(config)
enabled = isfield(config, "continuous_axis_weight_supervision_enabled") ...
    && logical(config.continuous_axis_weight_supervision_enabled);
end

function eligible = shadowUsableForActiveUse(shadow, config)
minimumTrust = double(config.minimum_soft_trust);
eligible = logicalField(shadow, "available", false) ...
    && logicalField(shadow, "causal_valid", false) ...
    && logicalField(shadow, "observed_innovation_available", false) ...
    && logicalField(shadow, "gp_model_available", false) ...
    && ~logicalField(shadow, "hard_invalid", true) ...
    && numericField(shadow, "trust", 0.0) >= minimumTrust;
end

function eligible = shadowEligibleForReentry(shadow, config)
eligible = shadowUsableForActiveUse(shadow, config) ...
    && consistencyAggregate(shadow) ...
        >= consistencyEnterThreshold(config);
end

function reason = reentryShadowReason(shadow, config)
if ~shadowUsableForActiveUse(shadow, config)
    reason = activeShadowReason(shadow, config);
elseif consistencyAggregate(shadow) < consistencyEnterThreshold(config)
    reason = "GP_CONSISTENCY_EWMA_BELOW_ENTER_DIRECT_B1";
else
    reason = "GP_NOT_ELIGIBLE_DIRECT_B1";
end
end

function value = consistencyAggregate(shadow)
if isfield(shadow, "consistency_ewma_aggregate")
    value = double(shadow.consistency_ewma_aggregate);
elseif isfield(shadow, "consistency_ewma_f")
    value = mean(double(shadow.consistency_ewma_f(:)));
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

function reason = activeShadowReason(shadow, config)
if ~logicalField(shadow, "available", false) ...
        || ~logicalField(shadow, "causal_valid", false) ...
        || ~logicalField(shadow, "observed_innovation_available", false) ...
        || ~logicalField(shadow, "gp_model_available", false)
    reason = "B2_ACTIVE_GP_EVIDENCE_UNAVAILABLE_DIRECT_B1";
elseif logicalField(shadow, "hard_invalid", true)
    reason = "B2_ACTIVE_GP_HARD_INVALID_DIRECT_B1";
elseif numericField(shadow, "trust", 0.0) < double(config.minimum_soft_trust)
    reason = "B2_ACTIVE_GP_TRUST_BELOW_MINIMUM_DIRECT_B1";
else
    reason = "B2_ACTIVE_GP_NOT_ELIGIBLE_DIRECT_B1";
end
end

function value = logicalField(source, name, fallback)
if isfield(source, name), value = logical(source.(name)); else, value = logical(fallback); end
end

function value = numericField(source, name, fallback)
if isfield(source, name), value = double(source.(name)); else, value = double(fallback); end
end

function validateState(state)
required = ["schema", "mode"];
for name = required
    if ~isfield(state, name)
        error("gpenmpcOuterSupervisorRequest:State", ...
            "Missing supervisor state field %s.", name);
    end
end
allowed = ["B2_ACTIVE", "B1_ONLY", "B2_REENTRY_PROBE", ...
    "FIXED_REFERENCE_HOLD"];
if ~any(string(state.mode) == allowed)
    error("gpenmpcOuterSupervisorRequest:Mode", ...
        "Unsupported supervisor mode %s.", string(state.mode));
end
end
