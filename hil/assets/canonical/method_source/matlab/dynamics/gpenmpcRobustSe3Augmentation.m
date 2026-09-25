function [augmentation, state, diagnostic] = gpenmpcRobustSe3Augmentation( ...
        plantState, reference, robustConfig, state, dt)
%GPENMPCROBUSTSE3AUGMENTATION Shared causal robust acceleration augmentation.
%
% Inputs use only current state, current reference and retained controller
% memory.  The same function is used by B1 and ordinary B2.

x = double(plantState(:));
dt = double(dt);
horizontal = double(reference.velocity_mps(1:2));
speed = norm(horizontal);
if speed >= 0.75
    state.last_tangent_xy = horizontal ./ speed;
end
tangent = state.last_tangent_xy;
frame = [tangent(1), -tangent(2), 0; ...
    tangent(2), tangent(1), 0; ...
    0, 0, 1];

lambda = double(robustConfig.lambda_position_s_inv(:));
slidingI = x(4:6) - double(reference.velocity_mps(:)) ...
    + lambda .* (x(1:3) - double(reference.position_m(:)));
slidingF = frame.' * slidingI;
radiusF = double(robustConfig.robust_radius_f_mps2(:)) ...
    + double(robustConfig.residual_tail_margin_f_mps2(:)) ...
    + double(robustConfig.projection_margin_f_mps2(:));
boundary = double(robustConfig.robust_boundary_layer_mps(:));
robustF = -radiusF .* tanh(slidingF ./ boundary);
% A causal vertical disturbance estimate is shared by B1 and B2.  It is
% composed with the existing robust target before the existing total cap,
% filter and slew limit, so it does not introduce additional authority.
observerI = [0; 0; -double(state.vertical_disturbance_ewma_mps2)];
rawTargetI = frame * robustF + observerI;
targetI = gpenmpcClipNorm(rawTargetI, ...
    double(robustConfig.combined_compensation_cap_mps2)) ...
    .* double(state.authority_scale);

gain = 1.0 - exp(-dt ./ double(robustConfig.trust_filter_time_constant_s));
lowPass = state.filtered_compensation_i_mps2 ...
    + gain .* (targetI - state.filtered_compensation_i_mps2);
delta = gpenmpcClipNorm(lowPass - state.filtered_compensation_i_mps2, ...
    double(robustConfig.compensation_slew_limit_mps3) .* dt);
state.filtered_compensation_i_mps2 = gpenmpcClipNorm( ...
    state.filtered_compensation_i_mps2 + delta, ...
    double(robustConfig.combined_compensation_cap_mps2));
augmentation = state.filtered_compensation_i_mps2;

diagnostic = struct;
diagnostic.sliding_i_mps = slidingI;
diagnostic.sliding_f_mps = slidingF;
diagnostic.radius_f_mps2 = radiusF;
diagnostic.target_i_mps2 = targetI;
diagnostic.raw_target_i_mps2 = rawTargetI;
diagnostic.vertical_disturbance_estimate_mps2 = ...
    double(state.vertical_disturbance_ewma_mps2);
diagnostic.vertical_observer_compensation_i_mps2 = observerI;
diagnostic.filter_gain = gain;
end
