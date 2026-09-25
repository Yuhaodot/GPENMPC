function value = gpenmpcNormalizedTotalRotorCommand(rotorCommandN, payloadKg)
%GPENMPCNORMALIZEDTOTALROTORCOMMAND Shared causal F17 command feature.
%
% The feature is the previous post-allocation six-rotor command sum divided
% by nominal weight.  Training extraction, online execution and predictive
% rollout call this helper to keep the feature definition identical when
% actuator lag or saturation is present.

command = double(rotorCommandN(:));
if numel(command) ~= 6 || any(~isfinite(command))
    error("gpenmpcNormalizedTotalRotorCommand:Command", ...
        "The previous rotor command must contain six finite values.");
end
payload = double(payloadKg);
if ~isscalar(payload) || ~isfinite(payload)
    error("gpenmpcNormalizedTotalRotorCommand:Payload", ...
        "Payload must be one finite scalar.");
end
nominalMassKg = 9.5 + payload;
value = sum(command) ./ max(nominalMassKg .* 9.80665, 1.0e-12);
value = min(max(value, 0.0), 2.0);
end
