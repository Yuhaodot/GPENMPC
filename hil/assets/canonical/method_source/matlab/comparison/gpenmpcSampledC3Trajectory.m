function trajectory = gpenmpcSampledC3Trajectory(localTimeS, positionM, velocityMps, accelerationMps2, jerkMps3)
%GPENMPCSAMPLEDC3TRAJECTORY Wrap a sampled C3 reference for prediction.
%
% The four derivative arrays are interpolated independently on the original
% 100 Hz grid and remain independent task-reference inputs.

time = double(localTimeS(:));
arrays = {positionM, velocityMps, accelerationMps2, jerkMps3};
if numel(time) < 2 || any(~isfinite(time)) || any(diff(time) <= 0)
    error("gpenmpcSampledC3Trajectory:Time", ...
        "A leg needs at least two finite, strictly increasing samples.");
end
for index = 1:numel(arrays)
    value = double(arrays{index});
    if ~isequal(size(value), [numel(time), 3]) || any(~isfinite(value), "all")
        error("gpenmpcSampledC3Trajectory:Shape", ...
            "Each sampled derivative must be finite N-by-3.");
    end
end
trajectory = struct;
trajectory.schema = "GPENMPC_SAMPLED_C3_REFERENCE_V1";
trajectory.total_duration_s = time(end);
trajectory.evaluate_fcn = @evaluate;
trajectory.sample_count = numel(time);
trajectory.source_is_parent_control_output = false;

    function value = evaluate(progressS, derivativeOrder)
        switch derivativeOrder
            case 0
                source = positionM;
            case 1
                source = velocityMps;
            case 2
                source = accelerationMps2;
            case 3
                source = jerkMps3;
            otherwise
                error("gpenmpcSampledC3Trajectory:Derivative", ...
                    "Only derivative orders zero through three are available.");
        end
        query = min(max(double(progressS), time(1)), time(end));
        value = interp1(time, double(source), query, "linear").';
    end
end
