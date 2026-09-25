function contract = gpenmpcFixedC3MethodContract(methodId)
%GPENMPCFIXEDC3METHODCONTRACT Declare fixed-reference native method identity.

methodId = string(methodId);
contract = struct;
contract.method_id = methodId;
contract.fixed_c3_reference = true;
contract.common_robust_se3_inner_loop = true;
contract.common_six_rotor_allocation = true;
contract.common_m600_six_dof_plant = true;
contract.external_reference_provider_required = false;
contract.use_pid = false;
switch methodId
    case "C0_CASCADED_PID"
        contract.use_gp = false;
        contract.use_robust = false;
        contract.use_pid = true;
        contract.common_robust_se3_inner_loop = false;
        contract.identity = ...
            "FIXED_C3_CASCADED_POSITION_ATTITUDE_PID_WITH_COMMON_ALLOCATION";
    case "N0_NOMINAL_SE3"
        contract.use_gp = false;
        contract.use_robust = false;
        contract.common_robust_se3_inner_loop = false;
        contract.identity = "FIXED_C3_NOMINAL_GEOMETRIC_SE3";
    case "R0_ROBUST_SE3"
        contract.use_gp = false;
        contract.use_robust = true;
        contract.identity = "FIXED_C3_ROBUST_SE3_NO_GP";
    case "G0_AERO_GP_ONLY_SE3"
        contract.use_gp = true;
        contract.use_robust = false;
        contract.identity = ...
            "FIXED_C3_CAUSAL_F17_GP_MEAN_WITHOUT_ROBUST_AUGMENTATION";
    case "GR0_GP_ROBUST_SE3"
        contract.use_gp = true;
        contract.use_robust = true;
        contract.identity = ...
            "FIXED_C3_CAUSAL_F17_GP_MEAN_PLUS_TRUST_CONDITIONED_ROBUST_SE3";
    case {"B1_ENMPC_TOTAL_ROBUST_TUBE_NO_GP", ...
            "B2_ENMPC_GP_MEAN_TOTAL_TUBE"}
        contract.use_gp = methodId == "B2_ENMPC_GP_MEAN_TOTAL_TUBE";
        contract.use_robust = true;
        contract.identity = "ENMPC_REFERENCE_PROVIDER_INTERFACE";
        contract.external_reference_provider_required = true;
    otherwise
        error("gpenmpcFixedC3MethodContract:Method", ...
            "Unsupported fixed-C3 method %s.", methodId);
end
end
