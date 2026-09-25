function state = gpenmpcInitializeOuterSupervisor(config)
%GPENMPCINITIALIZEOUTERSUPERVISOR Initialize the fail-closed outer FSM.
%
% The supervisor selects when a computed B2 or B1 decision may be applied.
% It provides a bounded one-tick hold before deterministic fixed-reference
% behavior when a solve is unavailable, with recovery dwell and persistent
% B2 probe success governing re-entry.

arguments
    config (1,1) struct
end
required = ["recovery_minimum_dwell_outer_ticks", ...
    "reentry_success_persistence_outer_ticks", ...
    "phase_acceleration_max_s_inv", ...
    "outer_acceleration_correction_max_mps2"];
for name = required
    if ~isfield(config, name)
        error("gpenmpcInitializeOuterSupervisor:Config", ...
            "Missing supervisor configuration field %s.", name);
    end
end
minimumDwell = double(config.recovery_minimum_dwell_outer_ticks);
requiredProbeSuccesses = double(config.reentry_success_persistence_outer_ticks);
if minimumDwell < 1 || minimumDwell ~= round(minimumDwell)
    error("gpenmpcInitializeOuterSupervisor:Dwell", ...
        "Recovery dwell must be a positive integer number of outer ticks.");
end
if requiredProbeSuccesses < 1 || requiredProbeSuccesses ~= round(requiredProbeSuccesses)
    error("gpenmpcInitializeOuterSupervisor:Persistence", ...
        "Re-entry persistence must be a positive integer number of outer ticks.");
end

state = struct;
state.schema = "GPENMPC_OUTER_SUPERVISOR_STATE_V1";
state.mode = "B2_ACTIVE";
state.tick_index = 0;
state.b1_success_dwell_ticks = 0;
state.b2_probe_success_ticks = 0;
state.reentry_blend_active = false;
state.reentry_blend_progress_ticks = 0;
state.minimum_b1_dwell_ticks = minimumDwell;
state.required_b2_probe_success_ticks = requiredProbeSuccesses;
state.last_feasible_valid = false;
state.last_feasible_phase_acceleration_s_inv = 0.0;
state.last_feasible_outer_correction_f_mps2 = zeros(3, 1);
state.last_applied_phase_acceleration_s_inv = 0.0;
state.last_applied_outer_correction_f_mps2 = zeros(3, 1);
state.failure_hold_available = false;
state.failure_hold_count = 0;
state.fixed_reference_count = 0;
state.b1_direct_request_count = 0;
state.b2_probe_request_count = 0;
state.reentry_count = 0;
state.shadow_dwell_accept_count = 0;
state.shadow_dwell_reset_count = 0;
state.innovation_exit_low_ticks = 0;
state.innovation_exit_count = 0;
state.transition_count = 0;
state.last_transition_reason = "INITIALIZED";
end
