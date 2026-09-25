function context = gpenmpcNativeEnmpcPredictionContext(assets)
%GPENMPCNATIVEENMPCPREDICTIONCONTEXT Build the common finite-horizon context.

allocation = gpenmpcM600Allocation(assets.calibration);
profile = assets.profile;
controller = assets.calibration.controller;
speedKnots = double(profile.energy_model.phase_average_power_table_w.speed_knots_mps(:));
preparedPowerTable = gpenmpcPrepareM600PhasePowerTable(profile);
context = struct;
context.base_mass_kg = double(profile.mass_properties.base_mass_kg);
context.gravity_mps2 = 9.80665;
context.position_gain_s2 = double(controller.position_gain_s2(:));
context.velocity_gain_s = double(controller.velocity_gain_s(:));
context.nominal_drag_n_per_mps2 = allocation.nominal_drag_n_per_mps2;
context.maximum_payload_kg = 4.54;
context.maximum_total_thrust_n = allocation.total_thrust_upper_n;
context.maximum_per_rotor_thrust_n = allocation.per_rotor_upper_n;
context.maximum_tilt_rad = allocation.maximum_tilt_rad;
context.rotor_allocation_matrix = allocation.matrix;
context.rotor_allocation_pseudoinverse = allocation.pseudoinverse;
context.inertia_kg_m2 = allocation.inertia_kg_m2;
context.attitude_moment_gain_nm_per_rad = ...
    double(controller.attitude_moment_gain_nm_per_rad(:));
context.body_rate_moment_gain_nm_s_per_rad = ...
    double(controller.body_rate_moment_gain_nm_s_per_rad(:));
context.rotor_diameter_m = double(profile.rotor_system.diameter_m);
context.airspeed_min_mps = min(speedKnots);
context.airspeed_max_mps = max(speedKnots);
context.prepared_phase_power_table = preparedPowerTable;
context.phase_power_fcn = @phasePower;

    function powerW = phasePower(phase, airspeedMps, payloadKg)
        switch string(phase)
            case "ASCEND"
                vertical = 1.0;
            case "DESCEND"
                vertical = -1.0;
            case "HOVER"
                vertical = 0.0;
                airspeedMps = 0.0;
            otherwise
                vertical = 0.0;
        end
        powerW = gpenmpcM600PreparedPhasePower( ...
            preparedPowerTable, payloadKg, airspeedMps, vertical);
    end
end
