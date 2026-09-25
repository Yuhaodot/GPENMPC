function state = gpenmpcObserveFixedC3CausalHistory( ...
        state, plantState, globalTimeS, legIndex)
%GPENMPCOBSERVEFIXEDC3CAUSALHISTORY Finish label k-1 at observation k.
%
% The update uses only the newly observed velocity and the nominal
% acceleration retained from the previous sample.  Leg boundaries and time
% gaps are excluded, matching the frozen task-level causal-data contract.

x = double(plantState(:));
now = double(globalTimeS);
leg = double(legIndex);
if state.previous_observation_valid
    dt = now - state.previous_time_s;
    sameLeg = leg == state.previous_leg_index;
    if sameLeg && isfinite(dt) && dt > 1.0e-9 && dt <= 0.0100001
        observedAcceleration = (x(4:6) - state.previous_velocity_mps) ./ dt;
        residualI = observedAcceleration ...
            - state.previous_nominal_acceleration_mps2;
        residualF = state.previous_frame_i_from_f.' * residualI;
        gain = 1.0 - exp(-dt ./ 0.50);
        state.previous_residual_ewma_f_mps2 = ...
            state.previous_residual_ewma_f_mps2 ...
            + gain .* (residualF ...
            - state.previous_residual_ewma_f_mps2);
    end
end
end
