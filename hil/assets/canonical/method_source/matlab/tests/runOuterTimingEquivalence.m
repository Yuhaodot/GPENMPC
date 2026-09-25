function result = runOuterTimingEquivalence(projectRoot, outputPath)
%RUNOUTERTIMINGEQUIVALENCE Compare outer-solver decisions and elapsed times.

arguments
    projectRoot (1,1) string
    outputPath (1,1) string = ""
end
addpath(genpath(fullfile(projectRoot, "matlab")));
fixture = jsondecode(fileread(fullfile(projectRoot, "tests", "fixtures", ...
    "ordinary_b2_rollout_python_reference.json")));
stored = load(fullfile(projectRoot, "matlab", ...
    "native_training", "raw", ...
    "MATLAB_NATIVE_SPARSE_GP_MODEL.mat"), "nativeModel");
preparedModel = gpenmpcPrepareSparseGpModel(stored.nativeModel);
priorPreparedModel = rmfield(preparedModel, ...
    "prepared_posterior_covariance_white_stack");
config = gpenmpcLoadOrdinaryB2Config(string(fixture.runtime_config_path), 0.30);
config.solver_deadline_s = 60.0;
trajectory = struct( ...
    "total_duration_s", double(fixture.trajectory.total_duration_s), ...
    "coefficients_ascending", ...
        double(fixture.trajectory.coefficients_ascending));
phaseState = fixture.phase_state;
observation = fixture.observation;
context = fixture.context;
context.phase_power_fcn = @fixedPower;
causal = fixture.causal_context_f17;
warmStart = zeros(1, config.decision_dimension);

gpenmpcOrdinaryB2OuterStep(warmStart, trajectory, phaseState, observation, ...
    context, priorPreparedModel, causal, config);
gpenmpcOrdinaryB2OuterStep(warmStart, trajectory, phaseState, observation, ...
    context, preparedModel, causal, config);

repeatCount = 20;
priorStarted = tic;
for index = 1:repeatCount
    [priorDecision, priorWarm, priorAudit] = gpenmpcOrdinaryB2OuterStep( ...
        warmStart, trajectory, phaseState, observation, context, ...
        priorPreparedModel, causal, config);
end
priorElapsedS = toc(priorStarted);
newStarted = tic;
for index = 1:repeatCount
    [newDecision, newWarm, newAudit] = gpenmpcOrdinaryB2OuterStep( ...
        warmStart, trajectory, phaseState, observation, context, ...
        preparedModel, causal, config);
end
newElapsedS = toc(newStarted);

priorComparable = rmfield(priorDecision, "elapsed_seconds");
newComparable = rmfield(newDecision, "elapsed_seconds");
assert(isequaln(priorComparable, newComparable));
assert(isequaln(priorWarm, newWarm));
assert(isequaln(priorAudit.primary_candidates, newAudit.primary_candidates));
assert(isequaln(priorAudit.selected_candidate, newAudit.selected_candidate));
assert(isequaln(priorAudit.feasible_mask, newAudit.feasible_mask));

result = struct;
result.schema = "GPENMPC_OUTER_TIMING_EQUIVALENCE_V1";
result.status = "PASS";
result.fixture_path = fullfile(projectRoot, "tests", "fixtures", ...
    "ordinary_b2_rollout_python_reference.json");
result.repeat_count = repeatCount;
result.prior_prepared_elapsed_s = priorElapsedS;
result.equivalent_repair_elapsed_s = newElapsedS;
result.elapsed_ratio_new_over_prior = newElapsedS ./ priorElapsedS;
result.decision_equivalent_excluding_wall_clock = true;
result.next_warm_start_byte_equivalent = true;
result.candidate_library_byte_equivalent = true;
result.selection_byte_equivalent = true;
if strlength(outputPath) > 0
    parent = fileparts(outputPath);
    if ~isfolder(parent)
        mkdir(parent);
    end
    writelines(jsonencode(result, PrettyPrint=true), outputPath, ...
        Encoding="UTF-8");
end
fprintf("%s\n", jsonencode(result));

    function powerW = fixedPower(~, airspeed, payload)
        powerW = 1050.0 + 9.0 .* airspeed.^2 + 20.0 .* payload;
    end
end
