function value = gpenmpcEvaluateTrajectoryDerivative(trajectory, progressS, derivativeOrder)
%GPENMPCEVALUATETRAJECTORYDERIVATIVE Evaluate a host-side C3 trajectory.
%
% Supported identities are deliberately small and deterministic:
%   1. trajectory.evaluate_fcn(progress, order), for an existing MATLAB
%      trajectory object wrapped by the caller;
%   2. trajectory.coefficients_ascending, a 3-by-N polynomial coefficient
%      array with powers increasing from zero.

% Progress is clipped to the declared trajectory duration, matching the
% inherited Python QuinticBezierC3Trajectory evaluation boundary.

arguments
    trajectory (1,1) struct
    progressS (1,1) double
    derivativeOrder (1,1) double {mustBeInteger,mustBeNonnegative}
end
if ~isfield(trajectory, "total_duration_s") ...
        || ~isfinite(double(trajectory.total_duration_s)) ...
        || trajectory.total_duration_s <= 0.0
    error("gpenmpcEvaluateTrajectoryDerivative:Duration", ...
        "Trajectory must declare one positive finite total_duration_s.");
end
progress = min(max(double(progressS), 0.0), double(trajectory.total_duration_s));
if isfield(trajectory, "evaluate_fcn")
    value = double(trajectory.evaluate_fcn(progress, derivativeOrder));
elseif isfield(trajectory, "prepared_derivative_coefficients_ascending") ...
        && derivativeOrder <= double( ...
            trajectory.prepared_maximum_derivative_order)
    derivatives = trajectory.prepared_derivative_coefficients_ascending;
    coefficients = double(derivatives{derivativeOrder + 1});
    powers = progress .^ (0:(size(coefficients, 2) - 1));
    value = coefficients * powers.';
elseif isfield(trajectory, "coefficients_ascending")
    coefficients = double(trajectory.coefficients_ascending);
    if size(coefficients, 1) ~= 3 || any(~isfinite(coefficients), "all")
        error("gpenmpcEvaluateTrajectoryDerivative:Coefficients", ...
            "Ascending trajectory coefficients must be one finite 3-by-N array.");
    end
    for order = 1:derivativeOrder
        powers = 1:(size(coefficients, 2) - 1);
        if isempty(powers)
            coefficients = zeros(3, 1);
            break;
        end
        coefficients = coefficients(:, 2:end) .* powers;
    end
    powers = progress .^ (0:(size(coefficients, 2) - 1));
    value = coefficients * powers.';
else
    error("gpenmpcEvaluateTrajectoryDerivative:Identity", ...
        "Trajectory needs evaluate_fcn or coefficients_ascending.");
end
value = double(value(:));
if numel(value) ~= 3 || any(~isfinite(value))
    error("gpenmpcEvaluateTrajectoryDerivative:Output", ...
        "Trajectory derivative must be one finite three-axis vector.");
end
end
