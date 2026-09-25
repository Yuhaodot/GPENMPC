function [selectedOneBased, audit] = gpenmpcSelectMissionCoverageRows(features, validMask, maximumPoints)
%GPENMPCSELECTMISSIONCOVERAGEROWS Deterministic label-blind mission subset.

arguments
    features (:,:) double
    validMask (:,1) logical
    maximumPoints (1,1) double {mustBeInteger,mustBePositive} = 390
end
valid = find(validMask);
pool = valid(1:25:end);
if isempty(pool)
    error("gpenmpcSelectMissionCoverageRows:Empty", "No causal-valid mission rows.");
end
if numel(pool) <= maximumPoints
    selectedOneBased = pool;
else
    candidate = features(pool, :);
    scale = std(candidate, 0, 1);
    active = scale > 1.0e-10;
    if ~any(active)
        error("gpenmpcSelectMissionCoverageRows:Inactive", "All candidate features are inactive.");
    end
    standardized = (candidate(:, active) - mean(candidate(:, active), 1)) ...
        ./ scale(active);
    centroid = mean(standardized, 1);
    [~, first] = min(sum((standardized - centroid).^2, 2));
    selectedPosition = zeros(maximumPoints, 1);
    selectedPosition(1) = first;
    chosen = false(numel(pool), 1);
    chosen(first) = true;
    minimumDistance = sum((standardized - standardized(first, :)).^2, 2);
    minimumDistance(first) = -inf;
    for index = 2:maximumPoints
        [~, next] = max(minimumDistance);
        selectedPosition(index) = next;
        chosen(next) = true;
        newDistance = sum((standardized - standardized(next, :)).^2, 2);
        minimumDistance = min(minimumDistance, newDistance);
        minimumDistance(chosen) = -inf;
    end
    selectedOneBased = sort(pool(selectedPosition));
end
audit = struct;
audit.schema = "GPENMPC_MATLAB_NATIVE_LABEL_BLIND_FEATURE_COVERAGE_SELECTION_V1";
audit.valid_count = numel(valid);
audit.stride = 25;
audit.pool_count = numel(pool);
audit.maximum_points = maximumPoints;
audit.selected_count = numel(selectedOneBased);
audit.selected_trace_index_one_based = selectedOneBased(:).';
audit.selected_trace_index_zero_based = selectedOneBased(:).' - 1;
end
