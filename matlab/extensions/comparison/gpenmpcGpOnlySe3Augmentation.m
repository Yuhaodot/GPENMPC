function [augmentation, state, diagnostic] = ...
        gpenmpcGpOnlySe3Augmentation(plantState, reference, ...
        windEstimateXyMps, payloadKg, robustConfig, gpModel, state, dt, ...
        forceHardInvalid)
%GPENMPCGPONLYSE3AUGMENTATION Causal GP mean without robust augmentation.
%
% This comparator uses the same F17 model, trust filter, GP cap, total cap
% and slew limit as the GP-plus-robust controller. Its robust component and
% uncertainty radius are zero, while hard-invalid input selects nominal
% geometric control.

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
    state.robust.last_tangent_xy = frame(1:2,1);
end

started = tic;
prediction = gpenmpcSparseGpPredict(gpModel, features);
inferenceSeconds = toc(started);
state.inference_seconds_sum = state.inference_seconds_sum + inferenceSeconds;
state.inference_seconds_max = max(state.inference_seconds_max, inferenceSeconds);
softBudget = double(robustConfig.inference_soft_budget_s);
hardBudget = double(robustConfig.inference_hard_timeout_s);
hardInvalid = forceHardInvalid || horizontalSpeed < 0.75 ...
    || logical(prediction.hard_invalid) ...
    || any(~isfinite(prediction.mean_mps2)) ...
    || any(~isfinite(prediction.calibrated_half_width_mps2)) ...
    || inferenceSeconds > hardBudget;
trustTarget = double(prediction.trust);
if inferenceSeconds > softBudget && ~hardInvalid
    trustTarget = trustTarget .* max(0.0, ...
        (hardBudget - inferenceSeconds) ./ (hardBudget - softBudget));
end

if hardInvalid
    state.filtered_trust = 0.0;
    state.robust.filtered_compensation_i_mps2 = zeros(3,1);
    state.hard_invalid_observations = state.hard_invalid_observations + 1;
    state.fallback_observations = state.fallback_observations + 1;
    augmentation = zeros(3,1);
    diagnostic = baseDiagnostic(frame,prediction,inferenceSeconds,true,gpModel);
    diagnostic.trust_weight = 0.0;
    diagnostic.gp_mean_f_mps2 = zeros(3,1);
    diagnostic.gp_std_f_mps2 = zeros(3,1);
    diagnostic.robust_acceleration_f_mps2 = zeros(3,1);
    diagnostic.applied_robust_radius_f_mps2 = zeros(3,1);
    diagnostic.rho_m2_total_f_mps2 = zeros(3,1);
    diagnostic.rho_m3_residual_f_mps2 = zeros(3,1);
    diagnostic.target_i_mps2 = zeros(3,1);
    return
end

gain = 1.0 - exp(-dt ./ double(robustConfig.trust_filter_time_constant_s));
requested = gain .* (trustTarget - state.filtered_trust);
maximumChange = double(robustConfig.trust_rate_limit_per_s) .* dt;
state.filtered_trust = min(max(state.filtered_trust ...
    + min(max(requested,-maximumChange),maximumChange),0.0),1.0);
alpha = state.filtered_trust;
state.valid_gp_observations = state.valid_gp_observations + 1;
meanF = gpenmpcClipNorm(double(prediction.mean_mps2(:)), ...
    double(robustConfig.gp_compensation_cap_mps2));
gpFeedforwardF = -alpha .* meanF;
desiredI = gpenmpcClipNorm(frame * gpFeedforwardF, ...
    double(robustConfig.combined_compensation_cap_mps2)) ...
    .* double(state.robust.authority_scale);
if alpha <= 1.0e-15
    state.robust.filtered_compensation_i_mps2 = zeros(3,1);
else
    lowPass = state.robust.filtered_compensation_i_mps2 ...
        + gain .* (desiredI - state.robust.filtered_compensation_i_mps2);
    delta = gpenmpcClipNorm(lowPass ...
        - state.robust.filtered_compensation_i_mps2, ...
        double(robustConfig.compensation_slew_limit_mps3) .* dt);
    state.robust.filtered_compensation_i_mps2 = gpenmpcClipNorm( ...
        state.robust.filtered_compensation_i_mps2 + delta, ...
        double(robustConfig.combined_compensation_cap_mps2));
end
augmentation = state.robust.filtered_compensation_i_mps2;

diagnostic = baseDiagnostic(frame,prediction,inferenceSeconds,false,gpModel);
diagnostic.trust_weight = alpha;
diagnostic.gp_mean_f_mps2 = meanF;
diagnostic.gp_std_f_mps2 = double(prediction.raw_std_mps2(:));
diagnostic.robust_acceleration_f_mps2 = zeros(3,1);
diagnostic.applied_robust_radius_f_mps2 = zeros(3,1);
diagnostic.rho_m2_total_f_mps2 = zeros(3,1);
diagnostic.rho_m3_residual_f_mps2 = zeros(3,1);
diagnostic.target_i_mps2 = desiredI;
end


function diagnostic = baseDiagnostic(frame,prediction,inferenceSeconds, ...
        hardInvalid,gpModel)
diagnostic = struct;
diagnostic.frame_i_from_f = frame;
diagnostic.support_distance = double(prediction.support_distance);
diagnostic.latent_variance_max = ...
    max(double(prediction.latent_variance_standardized));
diagnostic.ood_score = max( ...
    diagnostic.support_distance ...
        ./ max(double(gpModel.distance_hard_q995),1.0e-12), ...
    diagnostic.latent_variance_max ...
        ./ max(double(gpModel.latent_hard_q995),1.0e-12));
diagnostic.hard_invalid = logical(hardInvalid);
diagnostic.fallback_active = logical(hardInvalid);
diagnostic.inference_seconds = double(inferenceSeconds);
end
