function state = gpenmpcInitializeCascadedPidState()
%GPENMPCINITIALIZECASCADEDPIDSTATE Initialise causal four-loop PID memory.

state = struct;
state.velocity_integral_error_m = zeros(3,1);
state.body_rate_integral_error_rad = zeros(3,1);
state.previous_velocity_error_mps = zeros(3,1);
state.previous_body_rate_error_rad_s = zeros(3,1);
state.previous_error_valid = false;
state.last_leg_index = NaN;
state.integrator_freeze_samples = 0;
state.reset_count = 0;
end
