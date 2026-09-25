function state = gpenmpcCommitCausalVerticalDisturbanceObserver( ...
        state, velocityMps, nominalAccelerationMps2, timeS, legIndex, payloadKg)
%GPENMPCCOMMITCAUSALVERTICALDISTURBANCEOBSERVER Retain sample k for k+1.

velocity = double(velocityMps(:));
nominal = double(nominalAccelerationMps2(:));
state.vertical_observer_previous_velocity_z_mps = velocity(3);
state.vertical_observer_previous_nominal_acceleration_z_mps2 = nominal(3);
state.vertical_observer_previous_time_s = double(timeS);
state.vertical_observer_previous_leg_index = double(legIndex);
state.vertical_observer_previous_payload_kg = double(payloadKg);
state.vertical_observer_observation_valid = true;
end
