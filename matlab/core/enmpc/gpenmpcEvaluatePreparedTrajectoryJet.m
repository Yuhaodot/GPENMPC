function jet = gpenmpcEvaluatePreparedTrajectoryJet(trajectory, progressS)
%GPENMPCEVALUATEPREPAREDTRAJECTORYJET Evaluate orders zero through three.
%
% Each polynomial multiplication retains the same ascending-power order as
% gpenmpcEvaluateTrajectoryDerivative. The helper only amortizes validation,
% dispatch and derivative-coefficient construction.

arguments
    trajectory (1,1) struct
    progressS (1,1) double
end
if ~isfield(trajectory, "prepared_derivative_coefficients_ascending") ...
        || double(trajectory.prepared_maximum_derivative_order) < 3
    error("gpenmpcEvaluatePreparedTrajectoryJet:Preparation", ...
        "Trajectory must be prepared through derivative order three.");
end
progress = min(max(double(progressS), 0.0), ...
    double(trajectory.total_duration_s));
derivatives = trajectory.prepared_derivative_coefficients_ascending;
jet = cell(4, 1);
for order = 0:3
    coefficients = double(derivatives{order + 1});
    powers = progress .^ (0:(size(coefficients, 2) - 1));
    value = coefficients * powers.';
    jet{order + 1} = double(value(:));
end
end
