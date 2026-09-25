function [augmentation, state, diagnostic] = ...
        gpenmpcCoordinatedGpRobustSe3Augmentation(plantState, reference, ...
        robustConfig, state, dt, gpEvidence, gpAgreementWeightF, config)
%GPENMPCCOORDINATEDGPROBUSTSE3AUGMENTATION One coordinated GP/robust action.
%
% The learned acceleration residual enters the prediction model with a
% positive sign. This execution function applies its negative, in the exact
% prediction frame, as a bounded cancellation term within the shared robust-
% compensation channel. Unavailable, hard-invalid, low-trust or zero-
% responsibility evidence selects gpenmpcRobustSe3Augmentation, yielding exact
% B1 behavior.

arguments
    plantState (:,1) double
    reference (1,1) struct
    robustConfig (1,1) struct
    state (1,1) struct
    dt (1,1) double {mustBePositive}
    gpEvidence (1,1) struct
    gpAgreementWeightF (3,1) double
    config (1,1) struct
end

agreementF = min(max(double(gpAgreementWeightF(:)), 0.0), 1.0);
valid = logicalField(gpEvidence, "available", false) ...
    && logicalField(gpEvidence, "prediction_sample_closed", false) ...
    && logicalField(gpEvidence, "observed_innovation_available", false) ...
    && ~logicalField(gpEvidence, "hard_invalid", true);
trust = numericField(gpEvidence, "trust", 0.0);
valid = valid && isfinite(trust) && trust >= double(config.minimum_soft_trust);

meanF = vectorField(gpEvidence, "predicted_mean_f_mps2", nan(3,1));
frameEvidence = matrixField(gpEvidence, "gp_frame_i_from_f", nan(3,3));
if ~valid || any(~isfinite(meanF)) || any(~isfinite(frameEvidence), "all")
    state = resetResponsibilityState(state);
    [augmentation, state, diagnostic] = exactB1Fallback( ...
        plantState, reference, robustConfig, state, dt, "INVALID_OR_STALE_GP");
    return
end

meanF = gpenmpcClipNorm(meanF, double(robustConfig.gp_compensation_cap_mps2));
responsibilityEnabled = isfield(config, "gp_responsibility_gate_enabled") ...
    && logical(config.gp_responsibility_gate_enabled);
rawGpControlI = -(frameEvidence * meanF);
if responsibilityEnabled
    observedF = vectorField(gpEvidence, ...
        "observed_innovation_f_mps2", nan(3,1));
    halfWidthF = vectorField(gpEvidence, ...
        "calibrated_half_width_f_mps2", nan(3,1));
    if any(~isfinite(observedF)) || any(~isfinite(halfWidthF)) ...
            || any(halfWidthF <= 0.0)
        state = resetResponsibilityState(state);
        [augmentation, state, diagnostic] = exactB1Fallback( ...
            plantState, reference, robustConfig, state, dt, ...
            "INVALID_CLOSED_INNOVATION");
        return
    end
    [state, gate] = updateResponsibilityGate( ...
        state, observedF, dt, config);
    if state.gp_responsibility_mode_active ...
            || state.gp_responsibility_blend > 0.0
        filterGain = 1.0 - exp(-dt ./ ...
            double(config.responsibility_gp_mean_filter_time_constant_s));
        state.responsibility_filtered_gp_mean_f_mps2 = ...
            state.responsibility_filtered_gp_mean_f_mps2 ...
            + filterGain .* (meanF - state.responsibility_filtered_gp_mean_f_mps2);
    else
        state.responsibility_filtered_gp_mean_f_mps2 = zeros(3,1);
    end
    appliedMeanF = state.responsibility_filtered_gp_mean_f_mps2;
    responsibilityBlend = state.gp_responsibility_blend;
else
    gate = defaultGateDiagnostic();
    appliedMeanF = meanF;
    responsibilityBlend = 1.0;
end

responsibility = gpenmpcComputeGpResponsibility( ...
    responsibilityBlend, double(config.gp_mean_scale), trust, agreementF, ...
    appliedMeanF, true, double(config.minimum_soft_trust), ...
    double(config.coordinated_gp_alpha_zero_tolerance));
alphaF = responsibility.alpha_effective_f;
alpha = responsibility.alpha_effective_mean;
if responsibility.exact_b1_required
    [augmentation, state, diagnostic] = exactB1Fallback( ...
        plantState, reference, robustConfig, state, dt, ...
        responsibility.reason);
    diagnostic = attachGateDiagnostic(diagnostic, gate, state, ...
        rawGpControlI, zeros(3,1));
    diagnostic.gp_prediction_axis_authority_f = alphaF;
    diagnostic.gp_physical_axis_authority_f = zeros(3,1);
    return
end

x = double(plantState(:));
horizontal = double(reference.velocity_mps(1:2));
speed = norm(horizontal);
if speed >= 0.75
    state.last_tangent_xy = horizontal ./ speed;
end
tangent = state.last_tangent_xy;
frameRobust = [tangent(1), -tangent(2), 0; ...
    tangent(2), tangent(1), 0; 0, 0, 1];

lambda = double(robustConfig.lambda_position_s_inv(:));
slidingI = x(4:6) - double(reference.velocity_mps(:)) ...
    + lambda .* (x(1:3) - double(reference.position_m(:)));
slidingF = frameRobust.' * slidingI;
rhoB1 = double(robustConfig.robust_radius_f_mps2(:)) ...
    + double(robustConfig.residual_tail_margin_f_mps2(:)) ...
    + double(robustConfig.projection_margin_f_mps2(:));
rhoPostGp = double(config.post_gp_residual_radius_f_mps2(:));
boundary = double(robustConfig.robust_boundary_layer_mps(:));
observerShadowI = [0; 0; -double(state.vertical_disturbance_ewma_mps2)];
physicalGpEnabled = ~isfield(config, ...
    "coordinated_physical_gp_application_enabled") ...
    || logical(config.coordinated_physical_gp_application_enabled);
composition = gpenmpcComposeResidualTarget( ...
    slidingF, boundary, rhoB1, rhoPostGp, responsibility, ...
    frameEvidence, frameRobust, observerShadowI, physicalGpEnabled);
transition = gpenmpcAdvanceCompensationState( ...
    state.filtered_compensation_i_mps2, composition.raw_target_i_mps2, ...
    dt, double(robustConfig.trust_filter_time_constant_s), ...
    double(robustConfig.compensation_slew_limit_mps3), ...
    double(robustConfig.combined_compensation_cap_mps2), ...
    double(state.authority_scale));
state.filtered_compensation_i_mps2 = transition.applied_i_mps2;
augmentation = state.filtered_compensation_i_mps2;

diagnostic = baseDiagnostic();
if ~physicalGpEnabled
    diagnostic.mode = "PREDICTION_ONLY_EXACT_B1_EXECUTION";
elseif responsibilityBlend >= 1.0
    diagnostic.mode = "COORDINATED_GP_ACTIVE";
else
    diagnostic.mode = "BUMPLESS_RESPONSIBILITY_CROSS_FADE";
end
diagnostic.exact_b1_fallback = false;
diagnostic.gp_trust = trust;
diagnostic.gp_axis_authority_f = alphaF;
diagnostic.gp_prediction_axis_authority_f = alphaF;
diagnostic.gp_physical_axis_authority_f = composition.alpha_physical_f;
diagnostic.gp_authority = mean(composition.alpha_physical_f);
diagnostic.gp_mean_f_mps2 = meanF;
diagnostic.gp_filtered_mean_f_mps2 = appliedMeanF;
diagnostic.gp_frame_i_from_f = frameEvidence;
diagnostic.robust_frame_i_from_f = frameRobust;
diagnostic.sliding_i_mps = slidingI;
diagnostic.sliding_f_mps = slidingF;
diagnostic.radius_f_mps2 = composition.rho_applied_f_mps2;
diagnostic.b1_radius_f_mps2 = rhoB1;
diagnostic.post_gp_radius_f_mps2 = rhoPostGp;
diagnostic.gp_execution_feedforward_i_mps2 = composition.gp_control_i_mps2;
diagnostic.gp_raw_control_target_i_mps2 = rawGpControlI;
diagnostic.gp_filtered_control_target_i_mps2 = ...
    -(frameEvidence * appliedMeanF);
diagnostic.residual_robust_compensation_i_mps2 = composition.robust_i_mps2;
diagnostic.vertical_observer_compensation_i_mps2 = composition.observer_i_mps2;
diagnostic.vertical_observer_shadow_i_mps2 = observerShadowI;
diagnostic.vertical_disturbance_estimate_mps2 = ...
    double(state.vertical_disturbance_ewma_mps2);
diagnostic.raw_target_i_mps2 = composition.raw_target_i_mps2;
diagnostic.target_i_mps2 = transition.target_i_mps2;
diagnostic.total_applied_compensation_i_mps2 = augmentation;
diagnostic.filter_gain = transition.filter_gain;
diagnostic = attachGateDiagnostic(diagnostic, gate, state, ...
    rawGpControlI, -(frameEvidence * appliedMeanF));
end


function [state, diagnostic] = updateResponsibilityGate( ...
        state, observedF, dt, config)
floorF = double(config.responsibility_residual_floor_f_mps2(:));
ratioF = abs(double(observedF(:))) ./ max(floorF, 1.0e-12);
gain = 1.0 - exp(-dt ./ ...
    double(config.responsibility_innovation_ewma_time_constant_s));
state.responsibility_innovation_ratio_ewma_f = ...
    state.responsibility_innovation_ratio_ewma_f ...
    + gain .* (ratioF - state.responsibility_innovation_ratio_ewma_f);
aggregate = max(state.responsibility_innovation_ratio_ewma_f);
enterThreshold = double(config.gp_responsibility_enter_ratio);
exitThreshold = double(config.gp_responsibility_exit_ratio);

if state.gp_responsibility_mode_active
    state.gp_responsibility_enter_elapsed_s = 0.0;
    if aggregate < exitThreshold
        state.gp_responsibility_exit_elapsed_s = ...
            state.gp_responsibility_exit_elapsed_s + dt;
    else
        state.gp_responsibility_exit_elapsed_s = 0.0;
    end
    if state.gp_responsibility_exit_elapsed_s ...
            >= double(config.gp_responsibility_exit_dwell_s)
        state.gp_responsibility_mode_active = false;
        state.gp_responsibility_exit_elapsed_s = 0.0;
        state.gp_responsibility_mode_transition_count = state.gp_responsibility_mode_transition_count + 1;
    end
else
    state.gp_responsibility_exit_elapsed_s = 0.0;
    if aggregate > enterThreshold
        state.gp_responsibility_enter_elapsed_s = ...
            state.gp_responsibility_enter_elapsed_s + dt;
    else
        state.gp_responsibility_enter_elapsed_s = 0.0;
    end
    if state.gp_responsibility_enter_elapsed_s ...
            >= double(config.gp_responsibility_enter_dwell_s)
        state.gp_responsibility_mode_active = true;
        state.gp_responsibility_enter_elapsed_s = 0.0;
        state.gp_responsibility_mode_transition_count = state.gp_responsibility_mode_transition_count + 1;
    end
end

step = dt ./ double(config.responsibility_cross_fade_duration_s);
if state.gp_responsibility_mode_active
    state.gp_responsibility_blend = min(1.0, ...
        state.gp_responsibility_blend + step);
else
    state.gp_responsibility_blend = max(0.0, ...
        state.gp_responsibility_blend - step);
end
diagnostic = struct( ...
    "residual_floor_f_mps2", floorF, ...
    "observed_innovation_f_mps2", double(observedF(:)), ...
    "instantaneous_ratio_f", ratioF, ...
    "ewma_ratio_f", state.responsibility_innovation_ratio_ewma_f, ...
    "aggregate_ratio", aggregate, ...
    "mode_active", state.gp_responsibility_mode_active, ...
    "blend", state.gp_responsibility_blend, ...
    "mode_transition_count", state.gp_responsibility_mode_transition_count);
end


function state = resetResponsibilityState(state)
if isfield(state, "responsibility_innovation_ratio_ewma_f")
    state.responsibility_innovation_ratio_ewma_f = zeros(3,1);
    state.gp_responsibility_mode_active = false;
    state.gp_responsibility_enter_elapsed_s = 0.0;
    state.gp_responsibility_exit_elapsed_s = 0.0;
    state.gp_responsibility_blend = 0.0;
    state.responsibility_filtered_gp_mean_f_mps2 = zeros(3,1);
end
end


function diagnostic = defaultGateDiagnostic()
diagnostic = struct( ...
    "residual_floor_f_mps2", zeros(3,1), ...
    "observed_innovation_f_mps2", zeros(3,1), ...
    "instantaneous_ratio_f", zeros(3,1), ...
    "ewma_ratio_f", zeros(3,1), ...
    "aggregate_ratio", 0.0, ...
    "mode_active", true, ...
    "blend", 1.0, ...
    "mode_transition_count", 0);
end


function diagnostic = attachGateDiagnostic( ...
        diagnostic, gate, state, rawControlI, filteredControlI)
diagnostic.gp_residual_floor_f_mps2 = gate.residual_floor_f_mps2;
diagnostic.gp_innovation_ratio_instant_f = gate.instantaneous_ratio_f;
diagnostic.gp_innovation_ratio_ewma_f = gate.ewma_ratio_f;
diagnostic.gp_innovation_ratio_aggregate = gate.aggregate_ratio;
diagnostic.gp_responsibility_mode_active = gate.mode_active;
diagnostic.gp_responsibility_blend = gate.blend;
diagnostic.gp_mode_transition_count = gate.mode_transition_count;
diagnostic.gp_raw_control_target_i_mps2 = rawControlI;
diagnostic.gp_filtered_control_target_i_mps2 = filteredControlI;
if ~isfield(diagnostic, "vertical_observer_shadow_i_mps2")
    diagnostic.vertical_observer_shadow_i_mps2 = ...
        [0; 0; -double(state.vertical_disturbance_ewma_mps2)];
end
end


function [augmentation, state, diagnostic] = exactB1Fallback( ...
        plantState, reference, robustConfig, state, dt, reason)
[augmentation, state, inherited] = gpenmpcRobustSe3Augmentation( ...
    plantState, reference, robustConfig, state, dt);
diagnostic = baseDiagnostic();
diagnostic.mode = "EXACT_B1_FALLBACK";
diagnostic.fallback_reason = string(reason);
diagnostic.exact_b1_fallback = true;
diagnostic.sliding_i_mps = inherited.sliding_i_mps;
diagnostic.sliding_f_mps = inherited.sliding_f_mps;
diagnostic.radius_f_mps2 = inherited.radius_f_mps2;
diagnostic.b1_radius_f_mps2 = inherited.radius_f_mps2;
diagnostic.post_gp_radius_f_mps2 = inherited.radius_f_mps2;
diagnostic.residual_robust_compensation_i_mps2 = ...
    inherited.raw_target_i_mps2 ...
    - inherited.vertical_observer_compensation_i_mps2;
diagnostic.vertical_observer_compensation_i_mps2 = ...
    inherited.vertical_observer_compensation_i_mps2;
diagnostic.vertical_disturbance_estimate_mps2 = ...
    inherited.vertical_disturbance_estimate_mps2;
diagnostic.raw_target_i_mps2 = inherited.raw_target_i_mps2;
diagnostic.target_i_mps2 = inherited.target_i_mps2;
diagnostic.total_applied_compensation_i_mps2 = augmentation;
diagnostic.filter_gain = inherited.filter_gain;
end


function diagnostic = baseDiagnostic()
diagnostic = struct;
diagnostic.mode = "UNSET";
diagnostic.fallback_reason = "NONE";
diagnostic.exact_b1_fallback = false;
diagnostic.gp_trust = 0.0;
diagnostic.gp_axis_authority_f = zeros(3,1);
diagnostic.gp_prediction_axis_authority_f = zeros(3,1);
diagnostic.gp_physical_axis_authority_f = zeros(3,1);
diagnostic.gp_authority = 0.0;
diagnostic.gp_mean_f_mps2 = zeros(3,1);
diagnostic.gp_filtered_mean_f_mps2 = zeros(3,1);
diagnostic.gp_frame_i_from_f = eye(3);
diagnostic.robust_frame_i_from_f = eye(3);
diagnostic.sliding_i_mps = zeros(3,1);
diagnostic.sliding_f_mps = zeros(3,1);
diagnostic.radius_f_mps2 = zeros(3,1);
diagnostic.b1_radius_f_mps2 = zeros(3,1);
diagnostic.post_gp_radius_f_mps2 = zeros(3,1);
diagnostic.gp_execution_feedforward_i_mps2 = zeros(3,1);
diagnostic.gp_raw_control_target_i_mps2 = zeros(3,1);
diagnostic.gp_filtered_control_target_i_mps2 = zeros(3,1);
diagnostic.residual_robust_compensation_i_mps2 = zeros(3,1);
diagnostic.vertical_observer_compensation_i_mps2 = zeros(3,1);
diagnostic.vertical_observer_shadow_i_mps2 = zeros(3,1);
diagnostic.vertical_disturbance_estimate_mps2 = 0.0;
diagnostic.raw_target_i_mps2 = zeros(3,1);
diagnostic.target_i_mps2 = zeros(3,1);
diagnostic.total_applied_compensation_i_mps2 = zeros(3,1);
diagnostic.filter_gain = 0.0;
diagnostic.gp_residual_floor_f_mps2 = zeros(3,1);
diagnostic.gp_innovation_ratio_instant_f = zeros(3,1);
diagnostic.gp_innovation_ratio_ewma_f = zeros(3,1);
diagnostic.gp_innovation_ratio_aggregate = 0.0;
diagnostic.gp_responsibility_mode_active = false;
diagnostic.gp_responsibility_blend = 0.0;
diagnostic.gp_mode_transition_count = 0;
end


function value = logicalField(source, name, fallback)
if isfield(source, name), value = logical(source.(name)); else, value = logical(fallback); end
end


function value = numericField(source, name, fallback)
if isfield(source, name), value = double(source.(name)); else, value = double(fallback); end
end


function value = vectorField(source, name, fallback)
if isfield(source, name)
    candidate = source.(name);
    value = double(candidate(:));
else
    value = double(fallback(:));
end
if numel(value) ~= 3
    value = nan(3,1);
end
end


function value = matrixField(source, name, fallback)
if isfield(source, name)
    value = double(source.(name));
else
    value = double(fallback);
end
if ~isequal(size(value), [3,3])
    value = nan(3,3);
end
end
