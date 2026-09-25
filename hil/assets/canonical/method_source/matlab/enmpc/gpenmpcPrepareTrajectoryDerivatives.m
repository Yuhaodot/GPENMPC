function prepared = gpenmpcPrepareTrajectoryDerivatives(trajectory, maximumOrder)
%GPENMPCPREPARETRAJECTORYDERIVATIVES Cache immutable polynomial derivatives.
%
% The coefficient recurrence follows gpenmpcEvaluateTrajectoryDerivative while
% derivative coefficients are constructed once for repeated evaluation.
% Function-handle trajectories retain their caller-defined evaluation
% semantics.

arguments
    trajectory (1,1) struct
    maximumOrder (1,1) double {mustBeInteger,mustBeNonnegative} = 3
end
prepared = trajectory;
if isfield(trajectory, "evaluate_fcn")
    return
end
if ~isfield(trajectory, "coefficients_ascending")
    error("gpenmpcPrepareTrajectoryDerivatives:Identity", ...
        "Trajectory needs evaluate_fcn or coefficients_ascending.");
end
coefficients = double(trajectory.coefficients_ascending);
if size(coefficients, 1) ~= 3 || any(~isfinite(coefficients), "all")
    error("gpenmpcPrepareTrajectoryDerivatives:Coefficients", ...
        "Ascending trajectory coefficients must be one finite 3-by-N array.");
end
derivatives = cell(maximumOrder + 1, 1);
for order = 0:maximumOrder
    derivatives{order + 1} = coefficients;
    powers = 1:(size(coefficients, 2) - 1);
    if isempty(powers)
        coefficients = zeros(3, 1);
    else
        coefficients = coefficients(:, 2:end) .* powers;
    end
end
prepared.prepared_derivative_coefficients_ascending = derivatives;
prepared.prepared_maximum_derivative_order = maximumOrder;
end
