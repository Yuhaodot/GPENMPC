function result = runNativeComparisonInputIdentity(projectRoot, multiCityRoot, ordinaryB2Root)
%RUNNATIVECOMPARISONINPUTIDENTITY Verify all twelve shared task-route units.

arguments
    projectRoot (1,1) string
    multiCityRoot (1,1) string
    ordinaryB2Root (1,1) string
end
comparisonRoot = fullfile(projectRoot, "matlab", "comparison");
dataRoot = fullfile(projectRoot, "matlab", "data");
addpath(comparisonRoot);
cleanup = onCleanup(@() rmpath(comparisonRoot));
units = gpenmpcDiscoverNativeComparisonUnits(multiCityRoot, ordinaryB2Root);
assert(numel(units) == 12);
assert(all([units.route_identity_match]));
assert(all([units.visit_order_match]));
assert(all([units.mission_identity_match]));
assert(all([units.plan_identity_match]));

rowCounts = zeros(numel(units), 1);
legCounts = zeros(numel(units), 1);
for index = 1:numel(units)
    task = gpenmpcLoadNativeComparisonUnit(units(index), dataRoot);
    assert(task.parent_control_injection_prohibited);
    assert(task.parent_reference.task_complete);
    assert(task.mission_id == units(index).mission_id);
    assert(task.planner_id == units(index).planner_id);
    rowCounts(index) = numel(task.reference.global_time_s);
    legCounts(index) = numel(unique(task.reference.leg_index));
    assert(rowCounts(index) > 1000);
    assert(legCounts(index) >= 5);
    assert(all(isfinite(task.reference.position_m), "all"));
    assert(all(isfinite(task.reference.velocity_mps), "all"));
    assert(all(isfinite(task.reference.acceleration_mps2), "all"));
    assert(all(isfinite(task.reference.jerk_mps3), "all"));
end

result = struct;
result.schema = "GPENMPC_MATLAB_NATIVE_COMPARISON_INPUT_IDENTITY_RESULT_V1";
result.status = "PASS";
result.unit_count = numel(units);
result.mission_count = numel(unique([units.mission_id]));
result.planner_count = numel(unique([units.planner_id]));
result.minimum_trace_rows = min(rowCounts);
result.maximum_trace_rows = max(rowCounts);
result.minimum_leg_count = min(legCounts);
result.maximum_leg_count = max(legCounts);
fprintf("MATLAB_NATIVE_COMPARISON_INPUT_PASS units=%d rows=%d..%d legs=%d..%d\n", ...
    result.unit_count, result.minimum_trace_rows, result.maximum_trace_rows, ...
    result.minimum_leg_count, result.maximum_leg_count);
end
