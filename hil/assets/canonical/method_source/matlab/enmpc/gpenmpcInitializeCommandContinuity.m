function state = gpenmpcInitializeCommandContinuity(config)
%GPENMPCINITIALIZECOMMANDCONTINUITY Initialize one-leg command memory.

arguments
    config (1,1) struct
end
if ~isfield(config, "command_continuity_enabled") ...
        || ~logical(config.command_continuity_enabled)
    error("gpenmpcInitializeCommandContinuity:Disabled", ...
        "Command continuity must be enabled explicitly.");
end
if double(config.last_feasible_hold_outer_ticks) ~= 1
    error("gpenmpcInitializeCommandContinuity:HoldCount", ...
        "Exactly one outer-period hold is permitted.");
end
state = struct;
state.schema = "GPENMPC_COMMAND_CONTINUITY_STATE_V1";
state.update_index = 0;
state.last_feasible_valid = false;
state.last_feasible_phase_acceleration_s_inv = 0.0;
state.last_feasible_outer_correction_f_mps2 = zeros(3,1);
state.one_period_hold_available = false;
state.previous_command_source = "SAFE_NEUTRAL_COMMAND";
state.new_feasible_count = 0;
state.one_period_hold_count = 0;
state.exact_b1_fallback_count = 0;
state.safe_neutral_count = 0;
state.hard_invalid_termination_count = 0;
end
