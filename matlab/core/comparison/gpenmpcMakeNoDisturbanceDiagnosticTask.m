function diagnosticTask = gpenmpcMakeNoDisturbanceDiagnosticTask(task)
%GPENMPCMAKENODISTURBANCEDIAGNOSTICTASK Remove execution disturbances in memory.
%
% Route geometry, visit order, payload service schedule and C3 reference
% timing are retained while the software plant and wind disturbances are set
% to a controlled zero-disturbance diagnostic condition.

arguments
    task (1,1) struct
end
diagnosticTask = task;
diagnosticTask.diagnostic_source_mission_id = string(task.mission_id);
diagnosticTask.mission_id = string(task.mission_id) ...
    + "__NO_DISTURBANCE_DIAGNOSTIC";
diagnosticTask.diagnostic_only = true;

reference = diagnosticTask.reference;
reference.actual_wind_xy_mps(:) = 0.0;
reference.wind_estimate_xy_mps(:) = 0.0;
diagnosticTask.reference = reference;

mission = diagnosticTask.mission_config;
mission.actual_wind.base_mean_xy_mps = [0.0, 0.0];
mission.actual_wind.gust_amplitude_mps = 0.0;
mission.actual_wind.gust_duration_s = 0.0;
mission.actual_wind.persistent_shift_xy_mps = [0.0, 0.0];
mission.actual_wind.persistent_shift_transition_duration_s = 0.0;
mission.actual_wind.structured_estimation_bias.amplitude_mps = 0.0;
mission.actual_wind.wind_measurement_noise_std_mps = 0.0;
mission.plan_forecast.mean_xy_mps = [0.0, 0.0];
mission.plan_forecast.error_vertices_xy_mps = zeros(4, 2);

mismatch = mission.plant_mismatch;
mismatch.acceleration_bias_inertial_mps2 = zeros(1, 3);
mismatch.cross_drag_matrix_n_per_mps2 = zeros(3, 3);
mismatch.drag_scale_xyz = ones(1, 3);
mismatch.external_acceleration_amplitude_mps2 = 0.0;
mismatch.mass_bias_kg = 0.0;
mismatch.thrust_effectiveness_by_rotor = ones(1, 6);
mission.plant_mismatch = mismatch;

if isfield(mission, "structured_residual")
    residual = mission.structured_residual;
    residual.enabled = false;
    numericFields = ["drag_payload_coupling_n_per_mps2_xyz", ...
        "drag_tilt_gain", "scale", "slow_wind_bias_amplitude_mps", ...
        "thrust_efficiency_base_loss", "thrust_efficiency_command_gain", ...
        "thrust_efficiency_payload_gain", ...
        "thrust_efficiency_vertical_demand_gain", ...
        "turn_centripetal_gain", "turn_sideforce_n_per_mps2"];
    for fieldName = numericFields
        if isfield(residual, fieldName)
            residual.(fieldName) = zeros(size(residual.(fieldName)));
        end
    end
    mission.structured_residual = residual;
end
mission.family = "NO_DISTURBANCE_DIAGNOSTIC";
mission.scenario_stratum = "MECHANISM_DIAGNOSTIC_ONLY";
diagnosticTask.mission_config = mission;
end
