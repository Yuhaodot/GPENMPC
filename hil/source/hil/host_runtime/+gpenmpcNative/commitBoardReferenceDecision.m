function [state, event] = commitBoardReferenceDecision( ...
        state, decision, audit, generation, config)
% Commit one generation-bound outer decision to the reference target.
% A hard-invalid decision permanently suppresses reference publication.

arguments
    state (1,1) struct
    decision (1,1) struct
    audit (1,1) struct
    generation (1,1) uint64
    config (1,1) struct
end
assert(string(state.schema) == "GPENMPC_BOARD_REFERENCE_ADAPTER_STATE_V1", ...
    'gpenmpcNative:BoardReferenceState','Unexpected adapter state.');
assert(~state.hard_invalid_latched, ...
    'gpenmpcNative:BoardReferenceLatched','Hard-invalid adapter is terminal.');
assert(generation == state.last_committed_generation + uint64(1), ...
    'gpenmpcNative:BoardReferenceGeneration', ...
    'Decision generations must be exact, monotone, and gap-free.');

[continuity, applied, event] = gpenmpcApplyCommandContinuity( ...
    state.continuity, decision, audit, state.requested_method, config);
state.continuity = continuity;
state.last_committed_generation = generation;
state.target_phase_acceleration_s_inv = ...
    double(applied.phase_acceleration_s_inv);
state.target_outer_correction_f_mps2 = ...
    double(applied.outer_acceleration_correction_f_mps2(:));
state.hard_invalid_latched = logical(applied.hard_invalid_termination);
state.publication_allowed = ~state.hard_invalid_latched;
event.generation = double(generation);
event.board_reference_publication_allowed = state.publication_allowed;
event.hardware_actions = 0;
end
