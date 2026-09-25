function state = gpenmpcInitializeRobustSe3State()
%GPENMPCINITIALIZEROBUSTSE3STATE Initialise the shared robust inner-loop memory.

state = struct;
state.filtered_compensation_i_mps2 = zeros(3, 1);
state.authority_scale = 1.0;
state.last_tangent_xy = [1; 0];
state.vertical_disturbance_ewma_mps2 = 0.0;
state.vertical_observer_previous_velocity_z_mps = 0.0;
state.vertical_observer_previous_nominal_acceleration_z_mps2 = 0.0;
state.vertical_observer_previous_time_s = NaN;
state.vertical_observer_previous_leg_index = NaN;
state.vertical_observer_previous_payload_kg = NaN;
state.vertical_observer_observation_valid = false;
state.vertical_observer_update_enabled = true;
state.vertical_observer_reset_count = 0;
state.vertical_observer_antiwindup_hold_count = 0;
state.responsibility_innovation_ratio_ewma_f = zeros(3, 1);
state.gp_responsibility_mode_active = false;
state.gp_responsibility_enter_elapsed_s = 0.0;
state.gp_responsibility_exit_elapsed_s = 0.0;
state.gp_responsibility_blend = 0.0;
state.responsibility_filtered_gp_mean_f_mps2 = zeros(3, 1);
state.gp_responsibility_mode_transition_count = 0;
end
