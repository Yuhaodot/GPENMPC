function nextWarmStart = gpenmpcShiftEnmpcWarmStart(candidate, success, config)
%GPENMPCSHIFTENMPCWARMSTART Shift phase blocks and retain the 3D correction.

arguments
    candidate (1,:) double
    success (1,1) logical
    config (1,1) struct
end
candidate = double(candidate(:).');
if numel(candidate) ~= config.decision_dimension || any(~isfinite(candidate))
    error("gpenmpcShiftEnmpcWarmStart:Candidate", ...
        "Candidate must be one finite frozen-dimension decision.");
end
if ~success
    nextWarmStart = zeros(1, config.decision_dimension);
    return;
end
phase = candidate(1:config.control_blocks);
shiftedPhase = [phase(2:end), phase(end)];
nextWarmStart = [shiftedPhase, candidate(config.control_blocks + 1:end)];
end
