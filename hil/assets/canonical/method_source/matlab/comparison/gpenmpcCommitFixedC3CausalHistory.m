function state = gpenmpcCommitFixedC3CausalHistory(state, plantState, ...
        nominalAccelerationMps2, rotorCommandN, frameIFromF, ...
        globalTimeS, legIndex, payloadKg)
%GPENMPCCOMMITFIXEDC3CAUSALHISTORY Retain sample k for the next observation.

x = double(plantState(:));
state.previous_velocity_mps = x(4:6);
state.previous_nominal_acceleration_mps2 = ...
    double(nominalAccelerationMps2(:));
state.previous_rotor_command_n = double(rotorCommandN(:));
state.previous_normalized_total_rotor_command = ...
    gpenmpcNormalizedTotalRotorCommand(state.previous_rotor_command_n, payloadKg);
state.previous_frame_i_from_f = double(frameIFromF);
state.previous_time_s = double(globalTimeS);
state.previous_leg_index = double(legIndex);
state.previous_observation_valid = true;
end
