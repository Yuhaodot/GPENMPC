function result = runNoDisturbanceNominalContractTests(projectRoot)
%RUNNODISTURBANCENOMINALCONTRACTTESTS Check zero-disturbance task construction.

arguments
    projectRoot (1,1) string
end
addpath(genpath(fullfile(projectRoot, "matlab")));
multiCityRoot = ...
    gpenmpcExternalPath("comparison_fixed_reference_cases");
ordinaryB2Root = ...
    gpenmpcExternalPath("comparison_enmpc_cases");
units = gpenmpcDiscoverNativeComparisonUnits(multiCityRoot, ordinaryB2Root);
task = gpenmpcLoadNativeComparisonUnit(units(1), ...
    fullfile(projectRoot, "matlab", "data"));
% The loader restores its temporary data path; re-add the shared
% feature helpers for the focused closed-loop contract below.
addpath(fullfile(projectRoot, "matlab", "data"));
diagnosticTask = gpenmpcMakeNoDisturbanceDiagnosticTask(task);

assert(diagnosticTask.mission_payload_sha256 == task.mission_payload_sha256);
assert(diagnosticTask.plan_payload_sha256 == task.plan_payload_sha256);
assert(diagnosticTask.reference_trace_sha256 == task.reference_trace_sha256);
assert(isequal(diagnosticTask.route_candidate_ids, task.route_candidate_ids));
assert(isequal(diagnosticTask.visit_order, task.visit_order));
assert(all(diagnosticTask.reference.actual_wind_xy_mps == 0.0, "all"));
assert(all(diagnosticTask.reference.wind_estimate_xy_mps == 0.0, "all"));
assert(~diagnosticTask.mission_config.structured_residual.enabled);

contract = gpenmpcFixedC3MethodContract("N0_NOMINAL_SE3");
assert(~contract.use_gp && ~contract.use_robust);
assert(~contract.common_robust_se3_inner_loop);
assets = gpenmpcNativeHarnessAssets();
[trace, row] = gpenmpcRunFixedC3WholeTask( ...
    diagnosticTask, "N0_NOMINAL_SE3", assets, maximum_samples=80);
assert(all(isfinite(trace.position_m), "all"));
assert(all(trace.augmentation_acceleration_i_mps2 == 0.0, "all"));
assert(all(trace.gp_mean_f_mps2 == 0.0, "all"));
assert(all(trace.robust_acceleration_f_mps2 == 0.0, "all"));

result = struct;
result.schema = "GPENMPC_NO_DISTURBANCE_NOMINAL_CONTRACT_TEST_V1";
result.status = "PASS";
result.source_mission_id = string(task.mission_id);
result.diagnostic_mission_id = string(diagnosticTask.mission_id);
result.sample_count = 80;
result.reference_identity_preserved = true;
result.wind_and_plant_disturbance_zeroed = true;
result.nominal_augmentation_exact_zero = true;
fprintf("%s\n", jsonencode(result));
end
