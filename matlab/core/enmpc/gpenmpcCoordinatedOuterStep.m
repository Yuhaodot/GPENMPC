function [decision, nextWarmStart, audit] = gpenmpcCoordinatedOuterStep( ...
        warmStart, trajectory, phaseState, observation, context, gpModel, ...
        causalContextF17, gpEvidence, config)
%GPENMPCCOORDINATEDOUTERSTEP Causal GP/B1 outer supervisor.
%
% Effective axis authority determines the learned effect. Zero or invalid
% authority, stale evidence, hard invalidity and low trust select exact B1.

arguments
    warmStart (1,:) double
    trajectory (1,1) struct
    phaseState (1,1) struct
    observation (1,1) struct
    context (1,1) struct
    gpModel (1,1) struct
    causalContextF17 (1,1) struct
    gpEvidence (1,1) struct
    config (1,1) struct
end

valid = logicalField(gpEvidence, "available", false) ...
    && logicalField(gpEvidence, "prediction_sample_closed", false) ...
    && logicalField(gpEvidence, "observed_innovation_available", false) ...
    && ~logicalField(gpEvidence, "hard_invalid", true);
trust = numericField(gpEvidence, "trust", 0.0);
blend = optionalScalar(causalContextF17, ...
    "runtime_gp_responsibility_blend", 0.0);
agreementF = optionalVector3(causalContextF17, ...
    "runtime_gp_axis_weight_f", zeros(3,1));
meanF = optionalVector3(causalContextF17, ...
    "runtime_gp_filtered_mean_f_mps2", zeros(3,1));
responsibility = gpenmpcComputeGpResponsibility( ...
    blend, double(config.gp_mean_scale), trust, agreementF, meanF, ...
    valid, double(config.minimum_soft_trust), ...
    double(config.coordinated_exact_b1_tolerance));

if responsibility.exact_b1_required
    [decision, nextWarmStart, audit] = gpenmpcNativeB1OuterStep( ...
        warmStart, trajectory, phaseState, observation, context, config);
    selectedMethod = config.b1_fallback_method;
else
    [decision, nextWarmStart, audit] = gpenmpcOrdinaryB2OuterStep( ...
        warmStart, trajectory, phaseState, observation, context, gpModel, ...
        causalContextF17, config);
    selectedMethod = config.method;
end
audit.coordinated_single_supervisor = true;
audit.coordinated_selected_method = selectedMethod;
audit.coordinated_responsibility = responsibility;
audit.coordinated_architecture_mode = string(config.coordinated_architecture_mode);
audit.shadow_gp = gpEvidence;
end


function value = logicalField(source, name, fallback)
if isfield(source, name), value = logical(source.(name)); else, value = logical(fallback); end
end


function value = numericField(source, name, fallback)
if isfield(source, name), value = double(source.(name)); else, value = double(fallback); end
end


function value = optionalScalar(source, name, fallback)
if isfield(source, name), value = double(source.(name)); else, value = double(fallback); end
end


function value = optionalVector3(source, name, fallback)
if isfield(source, name)
    candidate = source.(name);
    value = double(candidate(:));
else
    value = double(fallback(:));
end
if numel(value) ~= 3 || any(~isfinite(value)), value = double(fallback(:)); end
end
