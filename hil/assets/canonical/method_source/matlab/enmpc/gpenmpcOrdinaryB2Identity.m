function identity = gpenmpcOrdinaryB2Identity(config)
%GPENMPCORDINARYB2IDENTITY Machine-readable ordinary-B2 method boundary.

arguments
    config (1,1) struct
end
identity = struct;
identity.schema = "GPENMPC_MATLAB_NATIVE_ORDINARY_B2_IDENTITY_V1";
identity.method = config.method;
identity.fallback_method = config.b1_fallback_method;
identity.learning = "FROZEN_GP_MEAN_IN_EVERY_HORIZON_ROLLOUT";
identity.gp_mean_scale = config.gp_mean_scale;
identity.robust_tube = config.robust_tube_identity;
identity.robust_tube_radius_f_mps2 = config.common_robust_total_radius_f_mps2;
identity.additional_prediction_acceleration_reserve_mps2 = ...
    config.additional_prediction_acceleration_reserve_mps2;
identity.n1_residual_tube_enabled = config.n1_residual_tube_enabled;
identity.b2r1_attenuation_enabled = config.b2r1_enabled;
identity.decision_shape = [config.control_blocks, 3];
identity.decision_dimension = config.decision_dimension;
identity.maximum_primary_candidates = config.maximum_primary_candidates;
identity.maximum_refinement_candidates = config.maximum_refinement_candidates;
identity.candidate_profile = config.gp_candidate_profile;
identity.parent_candidate_profile = config.parent_gp_candidate_profile;
identity.candidate_profile_runtime_override = ...
    config.gp_candidate_profile_runtime_override;
identity.gp_direct_physical_reference_candidate_enabled = false;
identity.b1_fallback_state_rule = ...
    "PRESERVE_EXACT_B1_DECISION_AND_WARM_STATE__RELABEL_METHOD_AND_GP_DIAGNOSTICS_ONLY";
end
