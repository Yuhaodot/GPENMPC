function receipt = runIndependentDevelopmentTaskLoaderTests(projectRoot)
%RUNINDEPENDENTDEVELOPMENTTASKLOADERTESTS Static eight-task adapter tests.

arguments
    projectRoot (1,1) string
end
addpath(genpath(fullfile(projectRoot, "matlab")));
caseBase = gpenmpcExternalPath("independent_development_cases");
dataRoot = fullfile(projectRoot, "matlab", "data");
expectedLegs = [5, 5, 5, 5, 4, 5, 5, 5];
expectedEvents = ["NONE", "NONE", "NONE", "NONE", ...
    "CANCEL", "REDIRECT", "NONE", "NONE"];
rows = repmat(struct("mission_id", "", "sample_count", 0, ...
    "leg_count", 0, "event_kind", "", "route_count", 0, ...
    "plan_versions", zeros(0,1), "source_trace_sha256", "", ...
    "pass", false), 8, 1);

for index = 1:8
    mission = "ENMPC_DV_" + compose("%03d", index);
    caseRoot = fullfile(caseBase, mission, "B0_FIXED_TIMING_ROBUST_SE3");
    task = gpenmpcLoadIndependentDevelopmentTask(caseRoot, dataRoot);
    assert(task.mission_id == mission);
    assert(task.parent_control_injection_prohibited);
    assert(abs(double(task.mission_config.simulation.sample_period_s) ...
        - 0.01) <= 1.0e-12);
    assert(numel(unique(task.reference.leg_index, "stable")) ...
        == expectedLegs(index));
    assert(numel(task.plans(1).legs) == expectedLegs(index));
    assert(numel(task.route_candidate_ids) == expectedLegs(index));
    assert(string(task.mission_config.task_event.kind) ...
        == expectedEvents(index));
    for leg = 1:expectedLegs(index)
        assert(string(task.plans(1).legs(leg).route_candidate_id) ...
            == string(task.route_candidate_ids(leg)));
    end
    assert(all(isfinite(task.reference.position_m), "all"));
    assert(all(isfinite(task.reference.actual_wind_xy_mps), "all"));
    assert(all(diff(task.reference.global_time_s) >= -1.0e-12));
    rows(index) = struct("mission_id", mission, ...
        "sample_count", numel(task.reference.global_time_s), ...
        "leg_count", expectedLegs(index), ...
        "event_kind", expectedEvents(index), ...
        "route_count", numel(task.route_candidate_ids), ...
        "plan_versions", task.actual_plan_versions, ...
        "source_trace_sha256", task.reference_trace_sha256, ...
        "pass", true);
end

receipt = struct;
receipt.schema = "GPENMPC_INDEPENDENT_DEVELOPMENT_LOADER_TEST_V1";
receipt.status = "PASS_8_OF_8_STATIC_TASK_ADAPTER_TESTS";
receipt.rows = rows;
end
