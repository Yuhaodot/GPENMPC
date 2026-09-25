function state = gpenmpcRobustSe3ObserveControl(state, controlDiagnostic, dt)
%GPENMPCROBUSTSE3OBSERVECONTROL Update shared fail-soft authority memory.

if logical(controlDiagnostic.rotor_saturated)
    target = 0.35;
elseif double(controlDiagnostic.force_projection_norm_mismatch_n) > 1e-6
    target = 0.65;
else
    target = 1.0;
end
gain = 1.0 - exp(-max(double(dt), 1e-6) ./ 0.20);
state.authority_scale = min(max(state.authority_scale ...
    + gain .* (target - state.authority_scale), 0.25), 1.0);
state.vertical_observer_update_enabled = ...
    ~logical(controlDiagnostic.rotor_saturated) ...
    && double(controlDiagnostic.force_projection_norm_mismatch_n) <= 1.0e-6;
if logical(controlDiagnostic.rotor_saturated)
    state.filtered_compensation_i_mps2 = ...
        0.80 .* state.filtered_compensation_i_mps2;
end
end
