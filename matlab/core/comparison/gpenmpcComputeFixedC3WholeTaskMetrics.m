function result = gpenmpcComputeFixedC3WholeTaskMetrics( ...
        trace, task, contract, methodState, completeReference)
%GPENMPCCOMPUTEFIXEDC3WHOLETASKMETRICS Unified task-level native metrics.

positionError = trace.reference_position_m - trace.position_m;
positionNorm = vecnorm(positionError, 2, 2);
attitudeNorm = vecnorm(trace.attitude_error_rad, 2, 2);
accelerationNorm = vecnorm(trace.actual_acceleration_mps2, 2, 2);
robustNorm = vecnorm(trace.robust_acceleration_f_mps2, 2, 2);
augmentationNorm = vecnorm(trace.augmentation_acceleration_i_mps2, 2, 2);
gpAppliedNorm = trace.trust_weight ...
    .* vecnorm(trace.gp_mean_f_mps2, 2, 2);

jerkNorm = zeros(0, 1);
rotorTotalVariationN = 0.0;
for leg = unique(trace.leg_index(:)).'
    rows = find(trace.leg_index == leg);
    if numel(rows) > 1
        step = diff(trace.global_time_s(rows));
        jerk = diff(trace.actual_acceleration_mps2(rows,:), 1, 1) ./ step;
        jerkNorm = [jerkNorm; vecnorm(jerk, 2, 2)]; %#ok<AGROW>
        rotorTotalVariationN = rotorTotalVariationN ...
            + sum(abs(diff(trace.rotor_command_n(rows,:), 1, 1)), "all");
    end
end
if isempty(jerkNorm)
    maximumJerk = 0.0;
else
    maximumJerk = max(jerkNorm);
end
minimumRotorMargin = 32.145727009134916 ...
    - max(trace.per_rotor_thrust_n, [], "all");
thresholds = struct( ...
    "position", max(positionNorm) <= 1.2 + 1.0e-9, ...
    "acceleration", max(accelerationNorm) <= 2.2 + 1.0e-9, ...
    "jerk", maximumJerk <= 4.0 + 1.0e-9, ...
    "tilt", max(trace.tilt_rad) <= deg2rad(25.0) + 1.0e-9, ...
    "rotor_margin", minimumRotorMargin > 0.0, ...
    "finite", allFiniteTrace(trace));
serviceRecords = trace.service_records;
serviceTimeS = sum([serviceRecords.duration_s]);
servicePublicEnergyJ = sum([serviceRecords.public_energy_j]);
result = struct;
result.schema = "GPENMPC_MATLAB_NATIVE_FIXED_C3_WHOLE_TASK_RESULT_V1";
result.mission_id = string(task.mission_id);
result.planner_id = string(task.planner_id);
result.city = string(task.city);
result.method = contract.method_id;
result.method_identity = contract.identity;
result.status = "FIXED_REFERENCE_SIMULATION_RESULT";
result.reference_complete = logical(completeReference);
result.task_complete_under_fixed_reference = logical(completeReference);
result.sample_count = numel(trace.global_time_s);
result.leg_count = numel(unique(trace.leg_index));
result.service_phase_count = numel(serviceRecords);
result.service_phase_time_s = serviceTimeS;
result.service_phase_public_energy_j = servicePublicEnergyJ;
result.mission_time_s = trace.global_time_s(end) - trace.global_time_s(1);
result.mission_public_phase_energy_j = ...
    trace.cumulative_public_energy_j(end);
result.mission_controller_sensitive_energy_j = ...
    trace.cumulative_controller_sensitive_energy_j(end);
result.mission_execution_energy_correction_j = ...
    result.mission_controller_sensitive_energy_j ...
    - result.mission_public_phase_energy_j;
result.tracking_position_mean_m = mean(positionNorm);
result.tracking_position_rms_m = sqrt(mean(positionNorm.^2));
result.tracking_position_p95_m = quantile(positionNorm, 0.95);
result.tracking_position_max_m = max(positionNorm);
result.tracking_attitude_mean_rad = mean(attitudeNorm);
result.tracking_attitude_p95_rad = quantile(attitudeNorm, 0.95);
result.tracking_attitude_max_rad = max(attitudeNorm);
result.maximum_speed_mps = max(vecnorm(trace.velocity_mps, 2, 2));
result.maximum_acceleration_mps2 = max(accelerationNorm);
result.maximum_jerk_mps3 = maximumJerk;
result.maximum_tilt_deg = rad2deg(max(trace.tilt_rad));
result.minimum_rotor_thrust_margin_n = minimumRotorMargin;
result.rotor_saturation_samples = sum(logical(trace.rotor_saturated));
result.rotor_command_total_variation_n = rotorTotalVariationN;
result.robust_compensation_mean_mps2 = mean(robustNorm);
result.robust_compensation_p95_mps2 = quantile(robustNorm, 0.95);
result.robust_compensation_max_mps2 = max(robustNorm);
result.gp_feedforward_mean_mps2 = mean(gpAppliedNorm);
result.gp_feedforward_p95_mps2 = quantile(gpAppliedNorm, 0.95);
result.total_augmentation_mean_mps2 = mean(augmentationNorm);
result.total_augmentation_p95_mps2 = quantile(augmentationNorm, 0.95);
result.mean_trust_weight = mean(trace.trust_weight);
result.p95_trust_weight = quantile(trace.trust_weight, 0.95);
result.hard_invalid_gp_samples = sum(logical(trace.hard_invalid));
result.gp_fallback_samples = sum(logical(trace.fallback_active));
result.inference_mean_s = mean(trace.inference_seconds);
result.inference_maximum_s = max(trace.inference_seconds);
result.threshold_checks = thresholds;
result.all_threshold_checks_pass = all(structfun(@logical, thresholds));
result.method_runtime_diagnostics = struct( ...
    "hard_invalid_observations", methodState.hard_invalid_observations, ...
    "fallback_observations", methodState.fallback_observations, ...
    "valid_gp_observations", methodState.valid_gp_observations, ...
    "terminal_trust", methodState.filtered_trust, ...
    "terminal_authority_scale", methodState.robust.authority_scale);
result.parent_control_injection_prohibited = ...
    logical(task.parent_control_injection_prohibited);
end


function finite = allFiniteTrace(trace)
finite = true;
names = fieldnames(trace);
for index = 1:numel(names)
    value = trace.(names{index});
    if isnumeric(value)
        finite = finite && all(isfinite(double(value(:))));
    end
end
end
