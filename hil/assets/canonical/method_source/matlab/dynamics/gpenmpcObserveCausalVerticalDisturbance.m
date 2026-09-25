function [state, diagnostic] = gpenmpcObserveCausalVerticalDisturbance( ...
        state, velocityMps, timeS, legIndex, payloadKg, robustConfig)
%GPENMPCOBSERVECAUSALVERTICALDISTURBANCE Finish interval k-1 at sample k.
%
% The observer is common to B1 and B2.  It estimates only the vertical
% residual that remains after the known nominal model.  Its estimate is
% bounded by the existing total compensation cap and is frozen whenever the
% previous control sample saturated or required force projection.

velocity = double(velocityMps(:));
now = double(timeS);
leg = double(legIndex);
payload = double(payloadKg);
sameContext = state.vertical_observer_observation_valid ...
    && leg == state.vertical_observer_previous_leg_index ...
    && abs(payload - state.vertical_observer_previous_payload_kg) <= 1.0e-12;
dt = now - state.vertical_observer_previous_time_s;
validInterval = sameContext && isfinite(dt) && dt > 1.0e-9 ...
    && dt <= 0.0100001 && isfinite(velocity(3));

residualZ = 0.0;
updated = false;
reset = false;
if validInterval
    observedAccelerationZ = (velocity(3) ...
        - state.vertical_observer_previous_velocity_z_mps) ./ dt;
    residualZ = observedAccelerationZ ...
        - state.vertical_observer_previous_nominal_acceleration_z_mps2;
    if state.vertical_observer_update_enabled && isfinite(residualZ)
        % Use the F17 causal-history time constant for the observer bandwidth.
        gain = 1.0 - exp(-dt ./ 0.50);
        candidate = state.vertical_disturbance_ewma_mps2 ...
            + gain .* (residualZ - state.vertical_disturbance_ewma_mps2);
        bound = double(robustConfig.combined_compensation_cap_mps2);
        state.vertical_disturbance_ewma_mps2 = ...
            min(max(candidate, -bound), bound);
        updated = true;
    else
        state.vertical_observer_antiwindup_hold_count = ...
            state.vertical_observer_antiwindup_hold_count + 1;
    end
elseif state.vertical_observer_observation_valid
    state = gpenmpcResetCausalVerticalDisturbanceObserver( ...
        state, velocity, now, leg, payload);
    reset = true;
end
% Consume the retained interval.  A new one becomes valid only after commit.
state.vertical_observer_observation_valid = false;
diagnostic = struct( ...
    "interval_valid", validInterval, ...
    "updated", updated, ...
    "reset", reset, ...
    "residual_z_mps2", residualZ, ...
    "estimate_z_mps2", state.vertical_disturbance_ewma_mps2);
end
