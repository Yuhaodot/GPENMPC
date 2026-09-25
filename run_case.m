function [trace, result, runDirectory] = run_case(taskId, planner, options)
%RUN_CASE Simulate one city task with the selected flight controller.
%   run_case("cambridge_01", "energy") runs Gaussian process (GP) enhanced eNMPC.
%   run_case("seattle_01", "distance", Method="robust_se3") selects a
%   fixed-reference comparison. MaximumInnerSamples limits a short run.

arguments
    taskId (1,1) string {mustBeMember(taskId, [ ...
        "cambridge_01", "cambridge_02", "seattle_01", "seattle_02", ...
        "manhattan_01", "manhattan_02"])} = "cambridge_01"
    planner (1,1) string {mustBeMember(planner, ["energy", "distance"])} = "energy"
    options.Method (1,1) string {mustBeMember(options.Method, [ ...
        "gp_enmpc", "enmpc", "robust_se3", "nominal_se3", ...
        "gp_robust_se3", "pid"])} = "gp_enmpc"
    options.MaximumInnerSamples (1,1) double = Inf
    options.OutputRoot (1,1) string = ""
end

maximumSamples = options.MaximumInnerSamples;
assert((isinf(maximumSamples) && maximumSamples > 0) || ...
    (isfinite(maximumSamples) && maximumSamples >= 2 && ...
    fix(maximumSamples) == maximumSamples && maximumSamples <= double(intmax("int32"))), ...
    "run_case:SampleLimit", ...
    "MaximumInnerSamples must be an integer of at least 2, or Inf.");

originalPath = path;
cleanupPath = onCleanup(@() path(originalPath)); %#ok<NASGU>
paths = setup_project();
taskPath = fullfile(paths.tasks, taskId + "_" + planner + ".mat");
assert(isfile(taskPath), "run_case:MissingTask", ...
    "Task file not found: %s", taskPath);
stored = load(taskPath, "task");
assert(isfield(stored, "task") && isstruct(stored.task) && isscalar(stored.task), ...
    "run_case:TaskVariable", "The task file must contain a scalar struct named task.");
task = stored.task;
validateTask(task, taskId, planner);
if isfield(task, "parent_reference")
    task = rmfield(task, "parent_reference");
end

method = methodDefinition(options.Method);
assets = methodAssets(paths.root, method);
if method.use_pid
    pidPath = fullfile(paths.root, "config", "cascaded_pid.json");
    assert(isfile(pidPath), "run_case:MissingPidConfiguration", ...
        "PID configuration not found: %s", pidPath);
    pid = jsondecode(fileread(pidPath));
    assert(string(pid.selected_profile_id) == "HIGH", ...
        "run_case:PidProfile", "The comparison uses the HIGH PID profile.");
    assets.pid_configuration = pid.selected_configuration;
end

outputRoot = options.OutputRoot;
if strlength(outputRoot) == 0
    outputRoot = paths.outputs;
end
outputRoot = string(char(java.io.File(char(outputRoot)).getCanonicalPath()));
if ~isfolder(outputRoot)
    [created, message] = mkdir(outputRoot);
    assert(created, "run_case:OutputFolder", "%s", message);
end
stamp = string(datetime("now", "Format", "yyyyMMdd_HHmmss_SSS"));
suffix = extractBefore(string(char(java.util.UUID.randomUUID())), 9);
runDirectory = fullfile(outputRoot, ...
    stamp + "_" + taskId + "_" + planner + "_" + options.Method + "_" + suffix);
assert(~isfolder(runDirectory), "run_case:ExistingRun", ...
    "Output folder already exists: %s", runDirectory);
[created, message] = mkdir(runDirectory);
assert(created, "run_case:OutputFolder", "%s", message);

runInfo = struct( ...
    "task", taskId, "planner", planner, "method", options.Method, ...
    "mission_id", string(task.mission_id), ...
    "planner_id", string(task.planner_id), ...
    "runtime_method_id", method.runtime_id, ...
    "maximum_inner_samples", maximumSamples, ...
    "task_file", taskPath, ...
    "started_at", string(datetime("now", "TimeZone", "UTC", ...
        "Format", "yyyy-MM-dd'T'HH:mm:ssXXX")));
if isfinite(maximumSamples)
    runInfo.run_mode = "sample_limited";
else
    runInfo.run_mode = "full_task";
end
configuration = struct("enmpc", assets.enmpc, "robust", assets.robust, ...
    "calibration", assets.calibration, "profile", assets.profile, ...
    "gp_model_sha256", string(assets.gp_model_sha256));
if method.use_pid
    configuration.pid = assets.pid_configuration;
end
fprintf("Running %s | %s | %s\n", taskId, planner, method.public_name);
started = tic;
try
    if method.fixed_reference
        [trace, result] = gpenmpcRunFixedC3WholeTask( ...
            task, method.runtime_id, assets, maximum_samples=maximumSamples);
        result.outer_period_s = method.outer_period_s;
        result.solver_deadline_s = 0.0;
        result.prediction_step_s = 0.0;
        result.horizon_steps = 0;
        result.prediction_horizon_s = 0.0;
        result.task_complete = logical(result.task_complete_under_fixed_reference);
    else
        nativeLimit = min(maximumSamples, double(intmax("int32")));
        [trace, result] = gpenmpcRunNativeEnmpcWholeTask( ...
            task, method.runtime_id, assets, ...
            MaximumInnerSamples=nativeLimit, ...
            AttitudeContinuityEnabled=true, ...
            AttitudeContinuityTimeConstantS=0.08, ...
            AttitudeContinuityMaximumAngularVelocityRadS=1.5, ...
            AttitudeContinuityMaximumAngularAccelerationRadS2=8.0, ...
            AttitudeContinuityMaximumAngularJerkRadS3=50.0, ...
            AttitudeContinuityMaximumContinuousDtS=0.05);
    end
catch exception
    runInfo.elapsed_wall_time_s = toc(started);
    failure = struct("identifier", string(exception.identifier), ...
        "message", string(exception.message));
    save(fullfile(runDirectory, "error.mat"), "runInfo", "failure", "configuration");
    rethrow(exception);
end

runInfo.elapsed_wall_time_s = toc(started);
runInfo.task_complete = logical(result.task_complete);
runInfo.sample_count = numel(trace.global_time_s);
result.public_method_name = method.public_name;
result.method_internal_id = method.internal_id;
result.runtime_method_id = method.runtime_id;
result.method_id = method.runtime_id;
result.run_mode = runInfo.run_mode;
result.maximum_inner_samples = maximumSamples;
result.input_identity = struct( ...
    "mission_payload_sha256", string(task.mission_payload_sha256), ...
    "plan_payload_sha256", string(task.plan_payload_sha256), ...
    "reference_trace_sha256", string(task.reference_trace_sha256), ...
    "gp_model_sha256", string(assets.gp_model_sha256));
save(fullfile(runDirectory, "simulation.mat"), ...
    "trace", "result", "runInfo", "configuration", "-v7.3");
if runInfo.run_mode == "sample_limited"
    fprintf("Finished sample-limited run: %d samples, task complete: %d.\n", ...
        runInfo.sample_count, runInfo.task_complete);
else
    fprintf("Finished: %d samples, task complete: %d.\n", ...
        runInfo.sample_count, runInfo.task_complete);
end
fprintf("Results: %s\n", runDirectory);
end

function validateTask(task, taskId, planner)
required = ["mission_id", "planner_id", "city", "mission_config", ...
    "plans", "reference", "initial_state", ...
    "mission_payload_sha256", "plan_payload_sha256", ...
    "reference_trace_sha256", "route_candidate_ids", "visit_order", ...
    "parent_control_injection_prohibited"];
assert(all(isfield(task, required)), "run_case:TaskFields", ...
    "The task file is missing required fields.");
assert(logical(task.parent_control_injection_prohibited), ...
    "run_case:TaskInput", "The task must contain an independent reference.");
if planner == "energy"
    expectedPlanner = "P_ENERGY_WIND_PAYLOAD";
else
    expectedPlanner = "P_DIST_FIXED";
end
assert(string(task.planner_id) == expectedPlanner, ...
    "run_case:PlannerIdentity", "Task planner does not match %s.", planner);
parts = split(taskId, "_");
cityToken = upper(parts(1));
assert(contains(upper(string(task.mission_id)), cityToken) && ...
    endsWith(string(task.mission_id), "_" + parts(2)), ...
    "run_case:MissionIdentity", "Task identity does not match %s.", taskId);
assert(abs(double(task.mission_config.simulation.sample_period_s) - 0.01) <= 1e-12, ...
    "run_case:SamplePeriod", "The task sample period must be 0.01 s.");
end

function method = methodDefinition(name)
method = struct("fixed_reference", true, "use_pid", false, ...
    "outer_period_s", 0.01, "solver_deadline_s", 0.0);
switch name
    case "gp_enmpc"
        method.internal_id = "M6_GP_ENHANCED_ENMPC_ROBUST_TRACKING_030";
        method.runtime_id = "B2_ENMPC_GP_MEAN_TOTAL_TUBE";
        method.public_name = "Gaussian Process Enhanced eNMPC with Robust Tracking Control";
        method.fixed_reference = false;
    case "enmpc"
        method.internal_id = "M5_ENMPC_ROBUST_TRACKING_030";
        method.runtime_id = "B1_ENMPC_TOTAL_ROBUST_TUBE_NO_GP";
        method.public_name = "eNMPC with Robust Tracking Control";
        method.fixed_reference = false;
    case "robust_se3"
        method.internal_id = "M1_ROBUST_GEOMETRIC_TRACKING";
        method.runtime_id = "R0_ROBUST_SE3";
        method.public_name = "Robust Geometric Tracking Control";
    case "nominal_se3"
        method.internal_id = "M0_NOMINAL_GEOMETRIC_TRACKING";
        method.runtime_id = "N0_NOMINAL_SE3";
        method.public_name = "Nominal Geometric Tracking Control";
    case "gp_robust_se3"
        method.internal_id = "M3_GP_ENHANCED_ROBUST_GEOMETRIC_TRACKING";
        method.runtime_id = "GR0_GP_ROBUST_SE3";
        method.public_name = "Gaussian Process Enhanced Robust Geometric Tracking Control";
    case "pid"
        method.internal_id = "C0_CASCADED_PID";
        method.runtime_id = "C0_CASCADED_PID";
        method.public_name = "Cascaded PID Control";
        method.use_pid = true;
end
if ~method.fixed_reference
    method.outer_period_s = 0.30;
    method.solver_deadline_s = 0.28;
end
end

function assets = methodAssets(root, method)
assets = gpenmpcLoadNativeEnmpcComparisonAssets(root, ...
    max(method.outer_period_s, 0.20));
if ~method.fixed_reference
    assert(string(assets.enmpc.method) == "B2_ENMPC_GP_MEAN_TOTAL_TUBE");
    assert(abs(double(assets.enmpc.prediction_step_s) - 0.20) <= 1e-12);
    assert(double(assets.enmpc.horizon_steps) == 8);
    assert(abs(double(assets.enmpc.prediction_horizon_s) - 1.60) <= 1e-12);
    assert(abs(double(assets.enmpc.outer_period_s) - 0.30) <= 1e-12);
    assert(abs(double(assets.enmpc.solver_deadline_s) - 0.28) <= 1e-12);
end
if method.runtime_id == "B2_ENMPC_GP_MEAN_TOTAL_TUBE"
    assets.enmpc = gpenmpcApplyArchitectureConfiguration( ...
        assets.enmpc, assets.robust, "A1_COORDINATED_PHYSICAL");
    assets.enmpc = gpenmpcApplyCommandContinuityConfiguration(assets.enmpc);
    assert(logical(assets.enmpc.coordinated_architecture_enabled));
    assert(string(assets.enmpc.coordinated_architecture_mode) == "A1_COORDINATED_PHYSICAL");
    assert(logical(assets.enmpc.command_continuity_enabled));
    assert(abs(double(assets.enmpc.outer_period_s) - 0.30) <= 1e-12);
end
end
