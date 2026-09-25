function metrics = gpenmpcComputeUnifiedComparisonMetrics(trace, result, publicName)
%GPENMPCCOMPUTEUNIFIEDCOMPARISONMETRICS Recompute shared trace metrics.
%
% This evaluator applies one nearest-rank P95 definition and the same
% leg-segmented jerk and rotor-command variation rules to fixed-reference
% robust control and eNMPC traces.  Energy and mission time are taken from
% each whole-task result because both include their declared service phases.

arguments
    trace (1,1) struct
    result (1,1) struct
    publicName (1,1) string
end

positionError = double(trace.reference_position_m) - double(trace.position_m);
positionNorm = vecnorm(positionError, 2, 2);
verticalError = positionError(:,3);
verticalAbsoluteError = abs(verticalError);
attitudeNorm = vectorNorm(trace, "attitude_error_rad", ...
    "attitude_error_norm_rad");
accelerationNorm = vectorNorm(trace, "actual_acceleration_mps2", ...
    "acceleration_norm_mps2");
[jerkNorm, rotorVariation] = segmentedDynamics(trace);
robustNorm = robustCompensationNorm(trace);

rotorUpperN = 32.145727009134916;
minimumRotorMargin = min(rotorUpperN - double(trace.rotor_command_n), [], "all");
tilt = double(trace.tilt_rad(:));

metrics = struct;
metrics.schema = "GPENMPC_UNIFIED_TRACE_COMPARISON_METRICS_V1";
metrics.public_method_name = publicName;
metrics.mission_id = string(result.mission_id);
metrics.planner_id = string(result.planner_id);
metrics.method_id = methodIdentity(result);
metrics.task_complete = logicalField(result, ...
    ["task_complete", "task_complete_under_fixed_reference"]);
metrics.sample_count = numel(trace.global_time_s);
metrics.mission_time_s = numericField(result, "mission_time_s");
metrics.controller_sensitive_modeled_energy_j = numericField(result, ...
    "mission_controller_sensitive_energy_j");
metrics.position_rms_m = sqrt(mean(positionNorm.^2));
metrics.position_p95_m = nearestRank(positionNorm, 0.95);
metrics.position_max_m = max(positionNorm);
metrics.vertical_mean_error_m = mean(verticalError);
metrics.vertical_rms_error_m = sqrt(mean(verticalError.^2));
metrics.vertical_p95_error_m = nearestRank(verticalAbsoluteError,0.95);
metrics.vertical_steady_state_error_m = ...
    mean(verticalError(max(1,floor(0.9*numel(verticalError))):end));
metrics.vertical_maximum_error_m = max(verticalAbsoluteError);
metrics.attitude_rms_rad = sqrt(mean(attitudeNorm.^2));
metrics.attitude_p95_rad = nearestRank(attitudeNorm, 0.95);
metrics.attitude_max_rad = max(attitudeNorm);
metrics.acceleration_p95_mps2 = nearestRank(accelerationNorm, 0.95);
metrics.acceleration_max_mps2 = max(accelerationNorm);
metrics.jerk_p95_mps3 = nearestRank(jerkNorm, 0.95);
metrics.jerk_max_mps3 = maximumOrZero(jerkNorm);
metrics.robust_compensation_p95_mps2 = nearestRank(robustNorm, 0.95);
metrics.rotor_command_total_variation_n = rotorVariation;
metrics.minimum_per_rotor_margin_n = minimumRotorMargin;
metrics.tilt_max_rad = maximumOrZero(tilt);
metrics.solver_deadline_miss_count = solverDeadlineMissCount(result);
metrics.screens = struct( ...
    "task_complete", metrics.task_complete, ...
    "finite", allFiniteNumericTrace(trace), ...
    "position_max_le_1p2", metrics.position_max_m <= 1.2 + 1.0e-12, ...
    "acceleration_max_le_2p2", ...
        metrics.acceleration_max_mps2 <= 2.2 + 1.0e-12, ...
    "jerk_max_le_4p0", metrics.jerk_max_mps3 <= 4.0 + 1.0e-12, ...
    "tilt_le_25deg", metrics.tilt_max_rad <= deg2rad(25.0) + 1.0e-12, ...
    "rotor_margin_nonnegative", minimumRotorMargin >= -1.0e-12, ...
    "solver_deadline_no_miss",metrics.solver_deadline_miss_count==0);
metrics.all_shared_dynamic_screens_pass = ...
    all(structfun(@logical, metrics.screens));
metrics.p95_definition = ...
    "EMPIRICAL_NEAREST_RANK__CEIL_0P95_TIMES_SAMPLE_COUNT";
metrics.jerk_definition = ...
    "NORM_OF_ACTUAL_ACCELERATION_FIRST_DIFFERENCE_WITHIN_EACH_LEG";
metrics.rotor_variation_definition = ...
    "SUM_ABSOLUTE_SIX_ROTOR_COMMAND_FIRST_DIFFERENCES_WITHIN_EACH_LEG";
end


function values = vectorNorm(trace, vectorField, scalarField)
if isfield(trace, vectorField)
    values = vecnorm(double(trace.(vectorField)), 2, 2);
elseif isfield(trace, scalarField)
    scalarValues = trace.(scalarField);
    values = double(scalarValues(:));
else
    error("gpenmpcComputeUnifiedComparisonMetrics:TraceField", ...
        "Trace lacks %s and %s.", vectorField, scalarField);
end
end


function [jerkNorm, rotorVariation] = segmentedDynamics(trace)
acceleration = double(trace.actual_acceleration_mps2);
rotor = double(trace.rotor_command_n);
time = double(trace.global_time_s(:));
leg = double(trace.leg_index(:));
jerkNorm = zeros(0,1);
rotorVariation = 0.0;
for identity = unique(leg).'
    rows = find(leg == identity);
    if numel(rows) < 2
        continue
    end
    dt = diff(time(rows));
    valid = isfinite(dt) & dt > 1.0e-12;
    deltaAcceleration = diff(acceleration(rows,:), 1, 1);
    jerk = deltaAcceleration(valid,:) ./ dt(valid);
    jerkNorm = [jerkNorm; vecnorm(jerk, 2, 2)]; %#ok<AGROW>
    rotorVariation = rotorVariation ...
        + sum(abs(diff(rotor(rows,:), 1, 1)), "all");
end
end


function values = robustCompensationNorm(trace)
if isfield(trace, "robust_compensation_i_mps2")
    values = vecnorm(double(trace.robust_compensation_i_mps2), 2, 2);
elseif isfield(trace, "augmentation_acceleration_i_mps2")
    values = vecnorm(double(trace.augmentation_acceleration_i_mps2), 2, 2);
else
    error("gpenmpcComputeUnifiedComparisonMetrics:RobustField", ...
        "Trace lacks a total applied robust-compensation field.");
end
end


function value = nearestRank(input, probability)
ordered = sort(double(input(:)));
if isempty(ordered)
    value = 0.0;
    return
end
index = max(1, min(numel(ordered), ceil(probability .* numel(ordered))));
value = ordered(index);
end


function value = maximumOrZero(input)
if isempty(input)
    value = 0.0;
else
    value = max(double(input(:)));
end
end


function value = numericField(source, name)
if ~isfield(source, name)
    error("gpenmpcComputeUnifiedComparisonMetrics:ResultField", ...
        "Result lacks %s.", name);
end
value = double(source.(name));
end


function value = logicalField(source, candidates)
for name = candidates
    if isfield(source, name)
        value = logical(source.(name));
        return
    end
end
error("gpenmpcComputeUnifiedComparisonMetrics:ResultField", ...
    "Result lacks every task-completion field.");
end


function identity = methodIdentity(result)
if isfield(result, "method_id")
    identity = string(result.method_id);
else
    identity = string(result.method);
end
end


function count = solverDeadlineMissCount(result)
count = 0;
if isfield(result, "solver") && isfield(result.solver, "deadline_miss_count")
    count = double(result.solver.deadline_miss_count);
end
end


function pass = allFiniteNumericTrace(trace)
% Only fields required by every method are tested. Method-disabled arrays
% are allowed to carry sentinel values and are evaluated by their own mask.
required=["global_time_s","leg_index","reference_position_m", ...
    "position_m","velocity_mps","quaternion_wxyz","body_rate_rad_s", ...
    "rotor_command_n","actual_acceleration_mps2","tilt_rad"];
pass=true;
for name=required
    if ~isfield(trace,name)
        pass = false;
        return
    end
    value=trace.(name);
    if any(~isfinite(double(value)),"all")
        pass=false;
        return
    end
end
end
