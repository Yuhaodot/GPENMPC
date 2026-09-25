function library = gpenmpcBuildEnmpcCandidateLibrary(method, warmStart, gpMeansIMps2, config)
%GPENMPCBUILDENMPCCANDIDATELIBRARY Build the deterministic finite shooting order.
%
% B1 and ordinary B2 evaluate the same physical control candidate library.
% Ordinary B2 incorporates the GP mean once in its predictive dynamics while
% retaining the shared physical candidate set.

arguments
    method (1,1) string
    warmStart (1,:) double
    gpMeansIMps2 double
    config (1,1) struct
end
validateConfigShape(config);
if method ~= config.method && method ~= config.b1_fallback_method
    error("gpenmpcBuildEnmpcCandidateLibrary:Method", ...
        "Only ordinary B2 and its exact B1 fallback are supported.");
end
warm = double(warmStart(:).');
if numel(warm) ~= config.decision_dimension || any(~isfinite(warm))
    error("gpenmpcBuildEnmpcCandidateLibrary:WarmStart", ...
        "Warm start must be one finite 7D decision vector.");
end
warm(1:config.control_blocks) = clipComponents( ...
    warm(1:config.control_blocks), config.phase_acceleration_max_s_inv);
warm(config.control_blocks + 1:end) = clipComponents( ...
    warm(config.control_blocks + 1:end), ...
    config.outer_acceleration_correction_max_mps2);

rawCandidates = zeros(0, config.decision_dimension);
rawLabels = strings(0, 1);
[rawCandidates, rawLabels] = appendUniqueByBytes( ...
    rawCandidates, rawLabels, warm, "WARM_START");
[rawCandidates, rawLabels] = appendUniqueByBytes( ...
    rawCandidates, rawLabels, zeros(1, config.decision_dimension), "ZERO");

for level = config.shooting_constant_levels_s_inv
    positive = warm;
    negative = warm;
    positive(1:config.control_blocks) = level;
    negative(1:config.control_blocks) = -level;
    token = replace(compose("%.2f", level), ".", "P");
    [rawCandidates, rawLabels] = appendUniqueByBytes( ...
        rawCandidates, rawLabels, positive, "PHASE_CONSTANT_POS_" + token);
    [rawCandidates, rawLabels] = appendUniqueByBytes( ...
        rawCandidates, rawLabels, negative, "PHASE_CONSTANT_NEG_" + token);
end
if config.control_blocks >= 2
    level = max(config.shooting_constant_levels_s_inv);
    increasing = warm;
    decreasing = warm;
    increasing(1:config.control_blocks) = linspace(-level, level, config.control_blocks);
    decreasing(1:config.control_blocks) = linspace(level, -level, config.control_blocks);
    [rawCandidates, rawLabels] = appendUniqueByBytes( ...
        rawCandidates, rawLabels, increasing, "PHASE_MONOTONE_INCREASING");
    [rawCandidates, rawLabels] = appendUniqueByBytes( ...
        rawCandidates, rawLabels, decreasing, "PHASE_MONOTONE_DECREASING");
end

baseCandidates = rawCandidates;
if method == config.method
    gpMeans = double(gpMeansIMps2);
    if ~isequal(size(gpMeans), [config.horizon_steps, 3]) || any(~isfinite(gpMeans), "all")
        error("gpenmpcBuildEnmpcCandidateLibrary:GpMeans", ...
            "Ordinary B2 requires one finite 8-by-3 GP-mean horizon.");
    end
    % Validate the supplied horizon; the rollout predicts GP means at its
    % candidate states.
end

if size(rawCandidates, 1) > config.maximum_primary_candidates
    error("gpenmpcBuildEnmpcCandidateLibrary:Budget", ...
        "Primary candidate library exceeded the frozen maximum of nine.");
end
library = struct;
library.schema = "GPENMPC_MATLAB_NATIVE_ENMPC_CANDIDATE_LIBRARY_V1";
library.method = method;
library.values = rawCandidates;
library.labels = rawLabels;
library.count = size(rawCandidates, 1);
library.base_count = size(baseCandidates, 1);
library.maximum_count = config.maximum_primary_candidates;
library.decision_dimension = config.decision_dimension;
library.phase_block_columns = 1:config.control_blocks;
library.correction_columns = config.control_blocks + (1:3);
library.gp_candidate_inserted = false;
library.gp_mean_remains_in_rollout = method == config.method;
library.gp_prediction_only = method == config.method;
library.robust_tube_identity = config.robust_tube_identity;
end


function validateConfigShape(config)
required = ["method", "b1_fallback_method", "decision_dimension", ...
    "control_blocks", "horizon_steps", "phase_acceleration_max_s_inv", ...
    "outer_acceleration_correction_max_mps2", ...
    "shooting_constant_levels_s_inv", "maximum_primary_candidates", ...
    "gp_candidate_profile", "robust_tube_identity"];
for name = required
    if ~isfield(config, name)
        error("gpenmpcBuildEnmpcCandidateLibrary:Config", ...
            "Configuration is missing %s.", name);
    end
end
if config.decision_dimension ~= 7 || config.control_blocks ~= 4 ...
        || config.horizon_steps ~= 8
    error("gpenmpcBuildEnmpcCandidateLibrary:DecisionIdentity", ...
        "The fixed ordinary-B2 decision identity is 4 phase blocks plus 3 corrections.");
end
end


function clipped = clipComponents(values, bound)
clipped = min(max(double(values), -double(bound)), double(bound));
end


function [values, labels] = appendUniqueByBytes(values, labels, candidate, label)
candidate = double(candidate(:).');
key = typecast(candidate(:), "uint64");
for index = 1:size(values, 1)
    existingKey = typecast(values(index, :).', "uint64");
    if isequal(key, existingKey)
        return;
    end
end
values(end + 1, :) = candidate;
labels(end + 1, 1) = string(label);
end
