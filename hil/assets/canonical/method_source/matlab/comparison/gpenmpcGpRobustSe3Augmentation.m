function [augmentation, state, diagnostic] = ...
        gpenmpcGpRobustSe3Augmentation(plantState, reference, ...
        windEstimateXyMps, payloadKg, robustConfig, gpModel, state, dt, ...
        forceHardInvalid)
%GPENMPCGPROBUSTSE3AUGMENTATION Causal GP mean plus shared robust SE(3).
%
% Hard-invalid input sets the GP mean and calibrated radius to zero in the
% same sample and delegates the full control expression to the shared R0
% robust function.  This is the exact fail-closed authority boundary.

arguments
    plantState (:,1) double
    reference (1,1) struct
    windEstimateXyMps (:,1) double
    payloadKg (1,1) double
    robustConfig (1,1) struct
    gpModel (1,1) struct
    state (1,1) struct
    dt (1,1) double {mustBePositive}
    forceHardInvalid (1,1) logical = false
end

x = double(plantState(:));
featureState = x;
featureState(14:19) = state.previous_rotor_command_n;
environment = [double(windEstimateXyMps(:)); 0; 0; double(payloadKg)];
referenceVector = [double(reference.position_m(:)); ...
    double(reference.velocity_mps(:)); ...
    double(reference.acceleration_mps2(:)); ...
    double(reference.jerk_mps3(:))];
[features, frame] = gpenmpcBuildOnlineF17(featureState, referenceVector, ...
    environment, state.previous_residual_ewma_f_mps2, ...
    state.robust.last_tangent_xy);
horizontalSpeed = norm(double(reference.velocity_mps(1:2)));
if horizontalSpeed >= 0.75
    state.robust.last_tangent_xy = frame(1:2, 1);
end

started = tic;
prediction = gpenmpcSparseGpPredict(gpModel, features);
inferenceSeconds = toc(started);
state.inference_seconds_sum = state.inference_seconds_sum + inferenceSeconds;
state.inference_seconds_max = max(state.inference_seconds_max, inferenceSeconds);
hardInvalid = forceHardInvalid || horizontalSpeed < 0.75 ...
    || logical(prediction.hard_invalid) ...
    || any(~isfinite(prediction.mean_mps2)) ...
    || any(~isfinite(prediction.calibrated_half_width_mps2));
trustTarget = double(prediction.trust);
softBudget = double(robustConfig.inference_soft_budget_s);
hardBudget = double(robustConfig.inference_hard_timeout_s);
if inferenceSeconds > hardBudget
    hardInvalid = true;
    trustTarget = 0.0;
elseif inferenceSeconds > softBudget
    trustTarget = trustTarget .* max(0.0, ...
        (hardBudget - inferenceSeconds) ./ (hardBudget - softBudget));
end

if hardInvalid
    state.filtered_trust = 0.0;
    state.hard_invalid_observations = state.hard_invalid_observations + 1;
    [augmentation, state.robust, robustDiagnostic] = ...
        gpenmpcRobustSe3Augmentation(x, reference, robustConfig, ...
        state.robust, dt);
    state.fallback_observations = state.fallback_observations + 1;
    diagnostic = baseDiagnostic(frame, prediction, inferenceSeconds, true, gpModel);
    diagnostic.trust_weight = 0.0;
    diagnostic.gp_mean_f_mps2 = zeros(3, 1);
    diagnostic.gp_std_f_mps2 = zeros(3, 1);
    diagnostic.robust_acceleration_f_mps2 = ...
        robustDiagnostic.radius_f_mps2 .* 0.0 ...
        - robustDiagnostic.radius_f_mps2 .* ...
        tanh(robustDiagnostic.sliding_f_mps ./ ...
        double(robustConfig.robust_boundary_layer_mps(:)));
    diagnostic.applied_robust_radius_f_mps2 = ...
        robustDiagnostic.radius_f_mps2;
    diagnostic.rho_m2_total_f_mps2 = robustDiagnostic.radius_f_mps2;
    diagnostic.rho_m3_residual_f_mps2 = ...
        double(robustConfig.residual_tail_margin_f_mps2(:)) ...
        + double(robustConfig.projection_margin_f_mps2(:));
    diagnostic.target_i_mps2 = robustDiagnostic.target_i_mps2;
    diagnostic.frame_i_from_f = frame;
    return
end

gain = 1.0 - exp(-dt ./ double(robustConfig.trust_filter_time_constant_s));
requested = gain .* (trustTarget - state.filtered_trust);
maximumChange = double(robustConfig.trust_rate_limit_per_s) .* dt;
state.filtered_trust = min(max(state.filtered_trust ...
    + min(max(requested, -maximumChange), maximumChange), 0.0), 1.0);
alpha = state.filtered_trust;
state.valid_gp_observations = state.valid_gp_observations + 1;

positionError = x(1:3) - double(reference.position_m(:));
velocityError = x(4:6) - double(reference.velocity_mps(:));
slidingI = velocityError ...
    + double(robustConfig.lambda_position_s_inv(:)) .* positionError;
slidingF = frame.' * slidingI;
meanF = gpenmpcClipNorm(double(prediction.mean_mps2(:)), ...
    double(robustConfig.gp_compensation_cap_mps2));
gpFeedforwardF = -alpha .* meanF;
fixedMarginF = double(robustConfig.residual_tail_margin_f_mps2(:)) ...
    + double(robustConfig.projection_margin_f_mps2(:));
rhoM2 = fixedMarginF + double(robustConfig.robust_radius_f_mps2(:));
rhoM3 = fixedMarginF ...
    + double(prediction.calibrated_half_width_mps2(:));
rhoApplied = alpha .* rhoM3 + (1.0 - alpha) .* rhoM2;
boundary = double(robustConfig.robust_boundary_layer_mps(:));
robustF = -rhoApplied .* tanh(slidingF ./ boundary);
desiredI = gpenmpcClipNorm(frame * (gpFeedforwardF + robustF), ...
    double(robustConfig.combined_compensation_cap_mps2)) ...
    .* double(state.robust.authority_scale);

lowPass = state.robust.filtered_compensation_i_mps2 ...
    + gain .* (desiredI - state.robust.filtered_compensation_i_mps2);
delta = gpenmpcClipNorm(lowPass ...
    - state.robust.filtered_compensation_i_mps2, ...
    double(robustConfig.compensation_slew_limit_mps3) .* dt);
state.robust.filtered_compensation_i_mps2 = gpenmpcClipNorm( ...
    state.robust.filtered_compensation_i_mps2 + delta, ...
    double(robustConfig.combined_compensation_cap_mps2));
augmentation = state.robust.filtered_compensation_i_mps2;

diagnostic = baseDiagnostic(frame, prediction, inferenceSeconds, false, gpModel);
diagnostic.trust_weight = alpha;
diagnostic.gp_mean_f_mps2 = meanF;
diagnostic.gp_std_f_mps2 = double(prediction.raw_std_mps2(:));
diagnostic.robust_acceleration_f_mps2 = robustF;
diagnostic.applied_robust_radius_f_mps2 = rhoApplied;
diagnostic.rho_m2_total_f_mps2 = rhoM2;
diagnostic.rho_m3_residual_f_mps2 = rhoM3;
diagnostic.target_i_mps2 = desiredI;
diagnostic.frame_i_from_f = frame;
end


function diagnostic = baseDiagnostic(frame, prediction, inferenceSeconds, ...
        hardInvalid, gpModel)
diagnostic = struct;
diagnostic.frame_i_from_f = frame;
diagnostic.support_distance = double(prediction.support_distance);
diagnostic.latent_variance_max = ...
    max(double(prediction.latent_variance_standardized));
diagnostic.ood_score = max( ...
    diagnostic.support_distance ./ max(double(gpModel.distance_hard_q995), 1.0e-12), ...
    diagnostic.latent_variance_max ./ max(double(gpModel.latent_hard_q995), 1.0e-12));
diagnostic.hard_invalid = logical(hardInvalid);
diagnostic.fallback_active = logical(hardInvalid);
diagnostic.inference_seconds = double(inferenceSeconds);
end
