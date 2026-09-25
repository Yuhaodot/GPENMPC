function pass = gpenmpcAllFiniteActiveTrace(trace)
%GPENMPCALLFINITEACTIVETRACE Check active control-chain fields only.
%
% GP diagnostic arrays use NaN when a prediction or closed innovation is
% inactive. Explicit masks select active GP rows for finite-value checks;
% every other numeric trace field is checked across the complete trace.

arguments
    trace (1,1) struct
end
pass = true;
names = fieldnames(trace);
predictionDiagnostics = [ ...
    "gp_predicted_mean_f_mps2", ...
    "gp_calibrated_half_width_f_mps2", ...
    "gp_support_distance", ...
    "gp_latent_variance_max"];
closedPredictionDiagnostics = [ ...
    "gp_observed_innovation_f_mps2", ...
    "gp_innovation_error_f_mps2"];
consistencyDiagnostics = [ ...
    "gp_normalized_innovation_error_f", ...
    "gp_consistency_score_instant_f"];
predictionMask = logicalMask(trace, "gp_evidence_available");
closedMask = predictionMask ...
    & logicalMask(trace, "gp_observed_innovation_available");
consistencyMask = logicalMask(trace, "gp_consistency_evidence_active");
for index = 1:numel(names)
    name = string(names{index});
    value = trace.(names{index});
    if any(name == predictionDiagnostics)
        value = maskedRows(value, predictionMask, name);
    elseif any(name == closedPredictionDiagnostics)
        value = maskedRows(value, closedMask, name);
    elseif any(name == consistencyDiagnostics)
        value = maskedRows(value, consistencyMask, name);
    end
    if isnumeric(value) && any(~isfinite(double(value)), "all")
        pass = false;
        return
    end
end
end


function mask = logicalMask(trace, name)
if ~isfield(trace, name)
    error("gpenmpcAllFiniteActiveTrace:MissingMask", ...
        "Trace is missing diagnostic activation mask %s.", name);
end
mask = logical(trace.(name)(:));
end


function value = maskedRows(value, mask, name)
if size(value, 1) ~= numel(mask)
    error("gpenmpcAllFiniteActiveTrace:MaskSize", ...
        "Diagnostic %s does not match its activation mask.", name);
end
value = value(mask, :);
end
