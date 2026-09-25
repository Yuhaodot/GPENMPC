function config = gpenmpcApplyArchitectureConfiguration( ...
        config, robustConfig, architectureMode)
%GPENMPCAPPLYARCHITECTURECONFIGURATION Configure A1 or A2 architecture.

arguments
    config (1,1) struct
    robustConfig (1,1) struct
    architectureMode (1,1) string
end

if ~any(architectureMode == ["A1_COORDINATED_PHYSICAL", ...
        "A2_PREDICTION_ONLY"])
    error("gpenmpcApplyArchitectureConfiguration:Mode", ...
        "Architecture mode must be A1_COORDINATED_PHYSICAL or A2_PREDICTION_ONLY.");
end

config = gpenmpcApplyResponsibilityConfiguration(config);
config.coordinated_architecture_enabled = true;
config.coordinated_architecture_mode = architectureMode;
config.coordinated_single_supervisor_enabled = true;
config.coordinated_physical_gp_application_enabled = ...
    architectureMode == "A1_COORDINATED_PHYSICAL";
config.coordinated_gp_execution_feedforward_enabled = true;
config.coordinated_b1_radius_f_mps2 = ...
    double(robustConfig.robust_radius_f_mps2(:)) ...
    + double(robustConfig.residual_tail_margin_f_mps2(:)) ...
    + double(robustConfig.projection_margin_f_mps2(:));
config.coordinated_post_gp_radius_f_mps2 = ...
    double(config.post_gp_residual_radius_f_mps2(:));
config.coordinated_exact_b1_tolerance = 1.0e-12;

% The hard-feasibility tolerance governs constraint acceptance. These values
% define when normalized soft-ranking terms are numerically indistinguishable.
config.coordinated_risk_equivalence_tolerance = 1.0e-6;
config.coordinated_tracking_equivalence_tolerance = 1.0e-4;
config.coordinated_objective_relative_switch_improvement = ...
    double(config.candidate_commit_hysteresis_fraction);

if any(config.coordinated_b1_radius_f_mps2 ...
        < double(config.common_robust_total_radius_f_mps2(:)) - 1.0e-12)
    error("gpenmpcApplyArchitectureConfiguration:Radius", ...
        "Runtime B1 radius must be at least the configured common total-tube radius.");
end
config.gp_candidate_profile = "COORDINATED_" + architectureMode ...
    + "__AXISWISE_RESPONSIBILITY__SINGLE_SUPERVISOR";
end
