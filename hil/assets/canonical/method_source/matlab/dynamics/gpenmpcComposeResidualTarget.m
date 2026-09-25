function composition = gpenmpcComposeResidualTarget( ...
        slidingF, boundaryF, rhoB1F, rhoPostGpF, responsibility, ...
        gpFrameIFromF, robustFrameIFromF, observerShadowI, ...
        physicalGpApplicationEnabled)
%GPENMPCCOMPOSERESIDUALTARGET Shared prediction/runtime composition.
%
% A1 applies the GP cancellation term physically and assigns the remaining
% observer and robust responsibility per axis.  A2 keeps the same GP plant
% residual in prediction but uses the exact B1 physical target.

arguments
    slidingF (3,1) double
    boundaryF (3,1) double
    rhoB1F (3,1) double
    rhoPostGpF (3,1) double
    responsibility (1,1) struct
    gpFrameIFromF (3,3) double
    robustFrameIFromF (3,3) double
    observerShadowI (3,1) double
    physicalGpApplicationEnabled (1,1) logical
end

alphaPredictionF = double(responsibility.alpha_effective_f(:));
weightedMeanF = double(responsibility.weighted_mean_f_mps2(:));
predictionResidualI = gpFrameIFromF * weightedMeanF;

if physicalGpApplicationEnabled && ~responsibility.exact_b1_required
    alphaPhysicalF = alphaPredictionF;
    rhoAppliedF = (1.0 - alphaPhysicalF) .* rhoB1F ...
        + alphaPhysicalF .* rhoPostGpF;
    gpControlI = -predictionResidualI;
    observerShadowF = robustFrameIFromF.' * observerShadowI;
    observerFractionF = 1.0 - alphaPhysicalF;
    observerI = robustFrameIFromF * (observerFractionF .* observerShadowF);
else
    alphaPhysicalF = zeros(3,1);
    rhoAppliedF = rhoB1F;
    gpControlI = zeros(3,1);
    observerFractionF = ones(3,1);
    observerI = observerShadowI;
end

robustF = -rhoAppliedF .* tanh(slidingF ./ boundaryF);
robustI = robustFrameIFromF * robustF;
rawTargetI = robustI + gpControlI + observerI;

composition = struct( ...
    "alpha_prediction_f", alphaPredictionF, ...
    "alpha_physical_f", alphaPhysicalF, ...
    "prediction_residual_i_mps2", predictionResidualI, ...
    "gp_control_i_mps2", gpControlI, ...
    "observer_shadow_i_mps2", observerShadowI, ...
    "observer_fraction_f", observerFractionF, ...
    "observer_i_mps2", observerI, ...
    "rho_b1_f_mps2", rhoB1F, ...
    "rho_post_gp_f_mps2", rhoPostGpF, ...
    "rho_applied_f_mps2", rhoAppliedF, ...
    "robust_i_mps2", robustI, ...
    "raw_target_i_mps2", rawTargetI, ...
    "physical_gp_application_enabled", physicalGpApplicationEnabled);
end
