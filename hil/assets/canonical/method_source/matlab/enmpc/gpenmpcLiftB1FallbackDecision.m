function decision = gpenmpcLiftB1FallbackDecision(b1Decision, gpMeanTrust, gpHardInvalid, trigger, config)
%GPENMPCLIFTB1FALLBACKDECISION Return the B1 fallback with B2 diagnostics.
%
% A successful B1 solve remains a continuous eNMPC decision, with method
% identity, GP diagnostics and fallback activation recorded in the result. A
% failed B1 solve preserves its state and receives the compatible status prefix.

arguments
    b1Decision (1,1) struct
    gpMeanTrust (1,1) double
    gpHardInvalid (1,1) logical
    trigger (1,1) string
    config (1,1) struct
end
if ~isfield(b1Decision, "method") ...
        || string(b1Decision.method) ~= config.b1_fallback_method
    error("gpenmpcLiftB1FallbackDecision:Identity", ...
        "Fallback input must be an exact B1 decision.");
end
if trigger ~= "GP_HARD_INVALID" && trigger ~= "GP_NO_FEASIBLE_PROFILE"
    error("gpenmpcLiftB1FallbackDecision:Trigger", ...
        "Unsupported ordinary-B2 to B1 fallback trigger.");
end
decision = b1Decision;
decision.method = config.method;
decision.mean_trust = gpMeanTrust;
decision.hard_invalid = gpHardInvalid;
decision.gp_b1_fallback_active = true;
if ~logical(b1Decision.success)
    reason = "B1_FALLBACK_FAILED";
    if isfield(b1Decision, "fallback_reason") ...
            && strlength(string(b1Decision.fallback_reason)) > 0
        reason = string(b1Decision.fallback_reason);
    end
    decision.fallback_reason = trigger + "__" + reason;
end
end
