function shadow = gpenmpcEvaluateShadowGpSupport(candidate, trajectory, ...
        phaseState, observation, context, gpModel, causalContextF17, ...
        observedInnovationI, observedInnovationAvailable, config)
%GPENMPCEVALUATESHADOWGPSUPPORT Current-tick GP support diagnostic.
%
% This diagnostic path reconstructs the rollout's first-step
% AERO_PHYSICS_F17 query for the selected B1 candidate, evaluates the supplied
% GP and compares the newest causal innovation with its calibrated half-width.
% It reports the instantaneous score consumed by the actual-dt EWMA in the
% online supervisor.

arguments
    candidate (1,:) double
    trajectory (1,1) struct
    phaseState (1,1) struct
    observation (1,1) struct
    context (1,1) struct
    gpModel struct
    causalContextF17 (1,1) struct
    observedInnovationI (3,1) double
    observedInnovationAvailable (1,1) logical
    config (1,1) struct
end

minimumTrust = minimumSoftTrust(config);
shadow = emptyShadow(minimumTrust);
shadow.causal_valid = isfield(causalContextF17, "valid") ...
    && logical(causalContextF17.valid);
shadow.observed_innovation_available = logical(observedInnovationAvailable) ...
    && all(isfinite(observedInnovationI));
shadow.gp_model_available = ~isempty(fieldnames(gpModel));
if ~shadow.causal_valid || ~shadow.observed_innovation_available ...
        || ~shadow.gp_model_available
    return
end

query = buildFirstStepQuery(candidate, trajectory, phaseState, observation, ...
    context, causalContextF17, config);
prediction = gpenmpcSparseGpPredict(gpModel, query.features_f17.');
shadow.available = true;
shadow.features_f17 = query.features_f17.';
shadow.gp_frame_i_from_f = query.gp_frame_i_from_f;
shadow.predicted_mean_f_mps2 = prediction.mean_mps2(1, :);
shadow.calibrated_half_width_f_mps2 = ...
    prediction.calibrated_half_width_mps2(1, :);
shadow.trust = double(prediction.trust(1));
shadow.support_distance = double(prediction.support_distance(1));
shadow.latent_variance_max = ...
    max(double(prediction.latent_variance_standardized(1, :)));
shadow.hard_invalid = logical(prediction.hard_invalid(1));
shadow.observed_innovation_f_mps2 = ...
    (query.gp_frame_i_from_f.' * observedInnovationI).';
shadow.innovation_error_f_mps2 = shadow.observed_innovation_f_mps2 ...
    - shadow.predicted_mean_f_mps2;
% The 1e-12 term is a floating-point boundary tolerance; the calibrated
% interval remains the model half-width.
halfWidth = double(shadow.calibrated_half_width_f_mps2);
if any(~isfinite(halfWidth)) || any(halfWidth <= 0.0)
    error("gpenmpcEvaluateShadowGpSupport:HalfWidth", ...
        "The GP interval must contain three positive finite half-widths.");
end
shadow.normalized_innovation_error_f = ...
    abs(shadow.innovation_error_f_mps2) ./ max(halfWidth, 1.0e-12);
shadow.innovation_consistency_score_f = ...
    1.0 ./ (1.0 + shadow.normalized_innovation_error_f .^ 2);
shadow.observed_innovation_consistent_f = ...
    shadow.normalized_innovation_error_f <= 1.0 + 1.0e-12;
shadow.instantaneous_consistency_aggregate = ...
    mean(shadow.innovation_consistency_score_f);
shadow.observed_innovation_consistent = ...
    shadow.instantaneous_consistency_aggregate >= 0.50;
shadow.eligible_for_b1_dwell = false;
end


function query = buildFirstStepQuery(candidate, trajectory, phaseState, ...
        observation, context, causalContextF17, config)
if numel(candidate) ~= config.decision_dimension || any(~isfinite(candidate))
    error("gpenmpcEvaluateShadowGpSupport:Candidate", ...
        "The shadow query requires one finite seven-coordinate candidate.");
end
dt = config.prediction_step_s;
phaseBlocks = candidate(1:config.control_blocks);
outerCorrectionF = candidate(config.control_blocks + (1:3)).';
p = column(observation.position_m, 3, "position_m");
v = column(observation.velocity_mps, 3, "velocity_mps");
phase = double(phaseState.progress_s);
rate = double(phaseState.progress_rate);
previousPhaseAcceleration = ...
    double(phaseState.previous_phase_acceleration_s_inv);
previousForce = column(observation.previous_desired_force_n, 3, ...
    "previous_desired_force_n");
previousOuterCorrectionI = column( ...
    observation.previous_outer_acceleration_correction_i_mps2, 3, ...
    "previous_outer_acceleration_correction_i_mps2");
wind3 = [column(observation.wind_estimate_xy_mps, 2, ...
    "wind_estimate_xy_mps"); 0.0];
payloadKg = double(observation.payload_kg);
mass = context.base_mass_kg + payloadKg;
predictedContext = double(causalContextF17.values(:));
if numel(predictedContext) ~= 4 || any(~isfinite(predictedContext))
    error("gpenmpcEvaluateShadowGpSupport:CausalContext", ...
        "The causal F17 context must contain four finite values.");
end

initialReference = gpenmpcPhaseReference(trajectory, phase, rate, ...
    previousPhaseAcceleration, 0.0);
initialReference.acceleration_mps2 = ...
    initialReference.acceleration_mps2 + previousOuterCorrectionI;
initialAir = initialReference.velocity_mps - wind3;
initialDrag = context.nominal_drag_n_per_mps2(:) ...
    .* initialAir .* abs(initialAir);
initialTrackingAcceleration = initialReference.acceleration_mps2 ...
    + context.position_gain_s2(:) .* (initialReference.position_m - p) ...
    + context.velocity_gain_s(:) .* (initialReference.velocity_mps - v);
initialForceWithoutRobust = mass .* (initialTrackingAcceleration ...
    + [0.0; 0.0; context.gravity_mps2]) + initialDrag;
predictedCommonRobustI = clipNorm( ...
    (previousForce - initialForceWithoutRobust) ./ mass, ...
    config.raw.common_inner_loop_predictor.combined_cap_mps2);

transition = gpenmpcJerkBoundedReferenceTransition(trajectory, phase, rate, ...
    previousPhaseAcceleration, phaseBlocks(1), previousOuterCorrectionI, ...
    outerCorrectionF, dt, config.reference_transition_jerk_limit_mps3);
reference = transition.reference;
referenceAir = reference.velocity_mps - wind3;
dragFeedforward = context.nominal_drag_n_per_mps2(:) ...
    .* referenceAir .* abs(referenceAir);
trackingAcceleration = reference.acceleration_mps2 ...
    + context.position_gain_s2(:) .* (reference.position_m - p) ...
    + context.velocity_gain_s(:) .* (reference.velocity_mps - v);
desiredForceForFeature = mass .* (trackingAcceleration ...
    + predictedCommonRobustI + [0.0; 0.0; context.gravity_mps2]) ...
    + dragFeedforward;
query = struct;
query.gp_frame_i_from_f = transition.reference_frame_i_from_f;
query.features_f17 = gpenmpcBuildAeroF17Features(v, reference, ...
    query.gp_frame_i_from_f, wind3, payloadKg, desiredForceForFeature, ...
    transition.reference_curvature, predictedContext, context);
end


function value = minimumSoftTrust(config)
if isfield(config, "minimum_soft_trust")
    value = double(config.minimum_soft_trust);
elseif isfield(config, "raw") ...
        && isfield(config.raw, "uncertainty_tightening") ...
        && isfield(config.raw.uncertainty_tightening, "minimum_soft_trust")
    value = double(config.raw.uncertainty_tightening.minimum_soft_trust);
else
    error("gpenmpcEvaluateShadowGpSupport:MinimumTrust", ...
        "Configuration must define minimum_soft_trust.");
end
if ~isscalar(value) || ~isfinite(value) || value < 0.0 || value > 1.0
    error("gpenmpcEvaluateShadowGpSupport:MinimumTrust", ...
        "minimum_soft_trust must be one finite probability-scale value.");
end
end


function shadow = emptyShadow(minimumTrust)
shadow = struct;
shadow.schema = "GPENMPC_SHADOW_GP_SUPPORT_V1";
shadow.read_only = true;
shadow.available = false;
shadow.causal_valid = false;
shadow.observed_innovation_available = false;
shadow.gp_model_available = false;
shadow.hard_invalid = false;
shadow.trust = 0.0;
shadow.minimum_soft_trust = minimumTrust;
shadow.support_distance = NaN;
shadow.latent_variance_max = NaN;
shadow.features_f17 = nan(1, 17);
shadow.gp_frame_i_from_f = eye(3);
shadow.predicted_mean_f_mps2 = nan(1, 3);
shadow.calibrated_half_width_f_mps2 = nan(1, 3);
shadow.observed_innovation_f_mps2 = nan(1, 3);
shadow.innovation_error_f_mps2 = nan(1, 3);
shadow.normalized_innovation_error_f = nan(1, 3);
shadow.innovation_consistency_score_f = nan(1, 3);
shadow.observed_innovation_consistent_f = false(1, 3);
shadow.instantaneous_consistency_aggregate = NaN;
shadow.observed_innovation_consistent = false;
shadow.eligible_for_b1_dwell = false;
end


function value = column(input, count, name)
value = double(input(:));
if numel(value) ~= count || any(~isfinite(value))
    error("gpenmpcEvaluateShadowGpSupport:Input", ...
        "%s must contain %d finite values.", name, count);
end
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
