function state = gpenmpcResetCascadedPidState(state, legIndex)
%GPENMPCRESETCASCADEDPIDSTATE Fail-closed reset at a leg/service boundary.

state.velocity_integral_error_m(:) = 0.0;
state.body_rate_integral_error_rad(:) = 0.0;
state.previous_velocity_error_mps(:) = 0.0;
state.previous_body_rate_error_rad_s(:) = 0.0;
state.previous_error_valid = false;
state.last_leg_index = double(legIndex);
state.reset_count = state.reset_count + 1;
end
