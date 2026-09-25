function report = test_release(options)
%TEST_RELEASE Check packaged tasks and run short numerical cases.
%   test_release(RunShortSimulation=false) checks files and configuration only.

arguments
    options.RunShortSimulation (1,1) logical = true
    options.ShortSamples (1,1) double {mustBeInteger,mustBePositive} = 40
    options.OutputRoot (1,1) string = ""
end
assert(options.ShortSamples >= 2, "test_release:SampleLimit", ...
    "ShortSamples must be at least 2.");
root = string(fileparts(fileparts(mfilename("fullpath"))));
originalPath = path;
cleanupPath = onCleanup(@() path(originalPath)); %#ok<NASGU>
addpath(root);
paths = setup_project();
tasks = ["cambridge_01", "cambridge_02", "seattle_01", "seattle_02", ...
    "manhattan_01", "manhattan_02"];
planners = ["energy", "distance"];
taskCount = 0;
for taskId = tasks
    for planner = planners
        file = fullfile(paths.tasks, taskId + "_" + planner + ".mat");
        assert(isfile(file), "test_release:MissingTask", "Missing task: %s", file);
        stored = load(file, "task");
        task = stored.task;
        assert(isstruct(task) && isscalar(task));
        assert(~isfield(task, "parent_reference"));
        assert(logical(task.parent_control_injection_prohibited));
        assert(all(isfinite(task.reference.position_m), "all"));
        assert(all(abs(task.reference.position_m(:,3) - 10.0) <= 1e-9));
        assert(abs(task.mission_config.simulation.sample_period_s - 0.01) <= 1e-12);
        services = 0;
        for leg = reshape(task.plans(1).legs, 1, [])
            service = leg.service;
            if isfield(service, "status"), continue; end
            expected = [10.0, 1.004, 8.996, 9.996, 8.0, 9.996];
            actual = [service.service_start_altitude_m, ...
                service.service_target_altitude_m, service.vertical_distance_each_m, ...
                service.descent_duration_s, service.ground_service_duration_s, ...
                service.ascent_duration_s];
            assert(all(abs(double(actual) - expected) <= 1e-9));
            services = services + 1;
        end
        assert(services == 4);
        taskCount = taskCount + 1;
    end
end

assets = gpenmpcLoadNativeEnmpcComparisonAssets(root, 0.30);
assert(string(assets.gp_model_sha256) == ...
    "732874E66D27E33CC0C7D48FF6ED781C81C026F67EC1328595E8EA089C7DB8E8");
config = gpenmpcApplyArchitectureConfiguration( ...
    assets.enmpc, assets.robust, "A1_COORDINATED_PHYSICAL");
config = gpenmpcApplyCommandContinuityConfiguration(config);
assert(config.coordinated_architecture_enabled && config.command_continuity_enabled);
assert(string(config.coordinated_architecture_mode) == "A1_COORDINATED_PHYSICAL");
assert(abs(config.outer_period_s - 0.30) <= 1e-12);
assert(abs(config.prediction_step_s - 0.20) <= 1e-12);
assert(config.horizon_steps == 8);
assert(abs(config.solver_deadline_s - 0.28) <= 1e-12);

report = struct("tasks_checked", taskCount, ...
    "configuration_checked", true, "short_runs", strings(0,1));
if options.RunShortSimulation
    methods = ["gp_enmpc", "enmpc", "robust_se3", "nominal_se3", "gp_robust_se3"];
    if isfile(fullfile(root, "config", "cascaded_pid.json"))
        methods(end+1) = "pid";
    end
    for method = methods
        [trace, result, runDirectory] = run_case("cambridge_01", "energy", ...
            Method=method, MaximumInnerSamples=options.ShortSamples, OutputRoot=options.OutputRoot);
        assert(result.run_mode == "sample_limited");
        assert(result.sample_count <= options.ShortSamples);
        assert(result.sample_count >= 2);
        assert(~result.task_complete);
        assert(all(isfinite(trace.position_m), "all"));
        assert(all(isfinite(trace.rotor_command_n), "all"));
        assert(isfile(fullfile(runDirectory, "simulation.mat")));
        saved = load(fullfile(runDirectory, "simulation.mat"), "configuration", "runInfo");
        assert(saved.runInfo.task_complete == result.task_complete);
        if method == "gp_enmpc"
            assert(saved.configuration.enmpc.coordinated_architecture_enabled);
            assert(saved.configuration.enmpc.command_continuity_enabled);
            assert(string(saved.configuration.enmpc.coordinated_architecture_mode) == ...
                "A1_COORDINATED_PHYSICAL");
        end
        report.short_runs(end+1,1) = runDirectory;
    end
    assert(numel(unique(report.short_runs)) == numel(methods));
end
report.passed = true;
fprintf("Checked %d tasks and %d short numerical runs.\n", ...
    report.tasks_checked, numel(report.short_runs));
end
