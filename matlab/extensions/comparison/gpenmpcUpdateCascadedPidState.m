function state = gpenmpcUpdateCascadedPidState( ...
        state, diagnostic, configuration, dt, legIndex)
%GPENMPCUPDATECASCADEDPIDSTATE Commit one causal PID state transition.

if ~isfinite(state.last_leg_index) || state.last_leg_index ~= legIndex
    state = gpenmpcResetCascadedPidState(state, legIndex);
end
freeze = logical(configuration.anti_windup. ...
    freeze_integrators_while_rotor_saturated) ...
    && logical(diagnostic.rotor_saturated);
if freeze
    state.integrator_freeze_samples = state.integrator_freeze_samples + 1;
else
    velocityLimit = double(configuration.velocity_loop. ...
        integral_error_limit_m(:));
    rateLimit = double(configuration.body_rate_loop. ...
        integral_error_limit_rad(:));
    state.velocity_integral_error_m = clampVector( ...
        state.velocity_integral_error_m ...
        + double(dt) .* diagnostic.velocity_error_mps, velocityLimit);
    state.body_rate_integral_error_rad = clampVector( ...
        state.body_rate_integral_error_rad ...
        + double(dt) .* diagnostic.body_rate_error_rad_s, rateLimit);
end
state.previous_velocity_error_mps = diagnostic.velocity_error_mps;
state.previous_body_rate_error_rad_s = diagnostic.body_rate_error_rad_s;
state.previous_error_valid = true;
state.last_leg_index = double(legIndex);
end

function value = clampVector(value, limit)
value = min(max(double(value(:)), -double(limit(:))), double(limit(:)));
end
