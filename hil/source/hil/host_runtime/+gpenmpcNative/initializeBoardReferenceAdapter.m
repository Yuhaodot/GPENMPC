function state = initializeBoardReferenceAdapter(config, trajectory, requestedMethod)
% Initialize the causal outer-reference adapter for the board path.

arguments
    config (1,1) struct
    trajectory (1,1) struct
    requestedMethod (1,1) string = "B2_ENMPC_GP_MEAN_TOTAL_TUBE"
end
assert(requestedMethod == string(config.method) ...
    || requestedMethod == string(config.b1_fallback_method), ...
    'gpenmpcNative:BoardReferenceMethod','Unsupported method identity.');
assert(isfield(trajectory,'total_duration_s') ...
    && isfinite(trajectory.total_duration_s) && trajectory.total_duration_s > 0, ...
    'gpenmpcNative:BoardReferenceTrajectory','Invalid trajectory duration.');
assert(config.outer_period_s == 0.30 && config.prediction_step_s == 0.20 ...
    && config.horizon_steps == 8 && config.prediction_horizon_s == 1.60 ...
    && config.solver_deadline_s == 0.28, ...
    'gpenmpcNative:BoardReferenceConfiguration','Unexpected selected configuration.');

state = struct;
state.schema = "GPENMPC_BOARD_REFERENCE_ADAPTER_STATE_V1";
state.requested_method = requestedMethod;
state.phase_s = 0.0;
state.phase_rate = 1.0;
state.previous_phase_acceleration_s_inv = 0.0;
state.previous_outer_correction_i_mps2 = zeros(3,1);
state.target_phase_acceleration_s_inv = 0.0;
state.target_outer_correction_f_mps2 = zeros(3,1);
state.continuity = gpenmpcInitializeCommandContinuity(config);
state.last_committed_generation = uint64(0);
state.reference_sample_count = uint64(0);
state.hard_invalid_latched = false;
state.publication_allowed = true;
state.trajectory_duration_s = double(trajectory.total_duration_s);
state.hardware_actions = 0;
end
