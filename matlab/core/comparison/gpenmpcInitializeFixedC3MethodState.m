function state = gpenmpcInitializeFixedC3MethodState(initialPlantState)
%GPENMPCINITIALIZEFIXEDC3METHODSTATE Initialise shared robust and GP memory.

x = double(initialPlantState(:));
if numel(x) ~= 19
    error("gpenmpcInitializeFixedC3MethodState:State", ...
        "The M600 plant state must contain 19 values.");
end
state = struct;
state.robust = gpenmpcInitializeRobustSe3State();
state.filtered_trust = 0.0;
state.previous_residual_ewma_f_mps2 = zeros(3, 1);
state.previous_normalized_total_rotor_command = 1.0;
state.previous_rotor_command_n = x(14:19);
state.previous_velocity_mps = zeros(3, 1);
state.previous_nominal_acceleration_mps2 = zeros(3, 1);
state.previous_frame_i_from_f = eye(3);
state.previous_time_s = NaN;
state.previous_leg_index = NaN;
state.previous_observation_valid = false;
state.hard_invalid_observations = 0;
state.fallback_observations = 0;
state.valid_gp_observations = 0;
state.inference_seconds_sum = 0.0;
state.inference_seconds_max = 0.0;
end
