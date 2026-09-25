function config = gpenmpcApplyCoordinatedGpConfiguration(config)
%GPENMPCAPPLYCOORDINATEDGPCONFIGURATION Apply the coordinated-GP configuration.
%
% Enables coordinated GP execution feedforward and execution-consistent
% objective evaluation, with progress and rotor-variation scales set from
% the coordinated-GP configuration.

arguments
    config (1,1) struct
end

required = ["coordinated_gp_execution_feedforward_enabled", ...
    "coordinated_gp_objective_scaling_enabled", ...
    "coordinated_gp_progress_scale_s", ...
    "coordinated_gp_execution_rotor_variation_scale_n"];
for name = required
    if ~isfield(config, name)
        error("gpenmpcApplyCoordinatedGpConfiguration:MissingField", ...
            "The configuration loader is missing %s.", name);
    end
end

config.coordinated_gp_execution_feedforward_enabled = true;
config.coordinated_gp_objective_scaling_enabled = true;
config.execution_consistent_cost_enabled = true;
config.progress_scale_s = double(config.coordinated_gp_progress_scale_s);
config.execution_rotor_variation_scale_n = ...
    double(config.coordinated_gp_execution_rotor_variation_scale_n);
config.prediction_effective_commit_enabled = false;
config.prediction_effective_direct_b2_enabled = false;
config.continuous_axis_weight_supervision_enabled = false;
config.gp_candidate_profile = ...
    "COORDINATED_GP_EXECUTION__TRACE_NORMALIZED_OBJECTIVE";

if abs(config.progress_scale_s - 2.5) > 1.0e-12
    error("gpenmpcApplyCoordinatedGpConfiguration:ProgressScale", ...
        "The configured progress scale must be 2.5 s.");
end
if abs(config.execution_rotor_variation_scale_n - 16.3385037412) > 1.0e-10
    error("gpenmpcApplyCoordinatedGpConfiguration:RotorScale", ...
        "The configured rotor-variation scale must remain 16.3385037412 N.");
end
end
