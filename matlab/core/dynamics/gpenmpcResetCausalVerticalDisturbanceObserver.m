function state = gpenmpcResetCausalVerticalDisturbanceObserver( ...
        state, velocityMps, timeS, legIndex, payloadKg)
%GPENMPCRESETCAUSALVERTICALDISTURBANCEOBSERVER Reset at a context boundary.

state.vertical_disturbance_ewma_mps2 = 0.0;
state.vertical_observer_previous_velocity_z_mps = double(velocityMps(3));
state.vertical_observer_previous_nominal_acceleration_z_mps2 = 0.0;
state.vertical_observer_previous_time_s = double(timeS);
state.vertical_observer_previous_leg_index = double(legIndex);
state.vertical_observer_previous_payload_kg = double(payloadKg);
state.vertical_observer_observation_valid = false;
state.vertical_observer_update_enabled = true;
state.vertical_observer_reset_count = state.vertical_observer_reset_count + 1;
if isfield(state, "responsibility_innovation_ratio_ewma_f")
    state.responsibility_innovation_ratio_ewma_f = zeros(3, 1);
    state.gp_responsibility_mode_active = false;
    state.gp_responsibility_enter_elapsed_s = 0.0;
    state.gp_responsibility_exit_elapsed_s = 0.0;
    state.gp_responsibility_blend = 0.0;
    state.responsibility_filtered_gp_mean_f_mps2 = zeros(3, 1);
end
end
