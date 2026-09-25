function [centres, audit] = gpenmpcSelectInducingPoints(standardizedInputs, count)
%GPENMPCSELECTINDUCINGPOINTS Deterministic PCG64 k-means++ for 4680 F17 rows.

arguments
    standardizedInputs (:,:) double
    count (1,1) double {mustBeInteger,mustBePositive} = 256
end
points = standardizedInputs;
if isempty(points) || any(~isfinite(points), "all")
    error("gpenmpcSelectInducingPoints:Input", "Inputs must be finite and nonempty.");
end
if ~isequal(size(points), [4680, 17])
    error("gpenmpcSelectInducingPoints:Domain", ...
        "The fixed PCG64 stream requires 4680 rows and 17 features.");
end
if count > size(points, 1)
    error("gpenmpcSelectInducingPoints:Count", ...
        "Inducing count cannot exceed the 4680 training rows.");
end
[firstZeroBased, uniforms, rngAudit] = gpenmpcPcg64AeroKmeansStream(max(count - 1, 0));
centres = zeros(count, size(points, 2));
first = firstZeroBased + 1;
centres(1, :) = points(first, :);
nearestSquared = sum((points - centres(1, :)).^2, 2);
selectedRows = zeros(count, 1);
selectedRows(1) = first;
for index = 2:count
    total = sum(nearestSquared);
    if total <= 1.0e-15
        centred = points - mean(points, 1);
        [~, candidate] = max(vecnorm(centred, 2, 2));
    else
        threshold = uniforms(index - 1) * total;
        candidate = find(cumsum(nearestSquared) > threshold, 1, "first");
        if isempty(candidate)
            candidate = size(points, 1);
        end
    end
    selectedRows(index) = candidate;
    centres(index, :) = points(candidate, :);
    nearestSquared = min(nearestSquared, sum((points - centres(index, :)).^2, 2));
end

iterationsCompleted = 0;
maximumUpdate = inf;
for iteration = 1:12
    squaredDistance = pairwiseSquaredDistance(points, centres);
    [~, labels] = min(squaredDistance, [], 2);
    updated = centres;
    for centreIndex = 1:count
        members = points(labels == centreIndex, :);
        if ~isempty(members)
            updated(centreIndex, :) = mean(members, 1);
        end
    end
    maximumUpdate = max(abs(updated - centres), [], "all");
    centres = updated;
    iterationsCompleted = iteration;
    if maximumUpdate <= 1.0e-10
        break
    end
end

audit = struct;
audit.schema = "GPENMPC_MATLAB_NATIVE_DETERMINISTIC_KMEANS_PLUS_PLUS_V1";
audit.input_rows = size(points, 1);
audit.input_dimension = size(points, 2);
audit.inducing_count = count;
audit.seed = uint32(1323034700);
audit.initial_selected_rows_one_based = selectedRows(:).';
audit.initial_selected_rows_zero_based = selectedRows(:).' - 1;
audit.lloyd_iteration_limit = 12;
audit.lloyd_iterations_completed = iterationsCompleted;
audit.final_maximum_update = maximumUpdate;
audit.rng = rngAudit;
end


function squared = pairwiseSquaredDistance(points, centres)
% Algebraically identical squared Euclidean distance with bounded negatives.
pointNorm = sum(points.^2, 2);
centreNorm = sum(centres.^2, 2).';
squared = pointNorm + centreNorm - 2.0 .* (points * centres.');
squared = max(squared, 0.0);
end
