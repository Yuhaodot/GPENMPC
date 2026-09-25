function refinement = gpenmpcBuildEnmpcRefinementCandidates(anchor, config)
%GPENMPCBUILDENMPCREFINEMENTCANDIDATES One deterministic axis/sign refinement pass.

arguments
    anchor (1,:) double
    config (1,1) struct
end
anchor = double(anchor(:).');
if numel(anchor) ~= config.decision_dimension || any(~isfinite(anchor))
    error("gpenmpcBuildEnmpcRefinementCandidates:Anchor", ...
        "Refinement anchor must be a finite 7D decision.");
end
values = repmat(anchor, config.maximum_refinement_candidates, 1);
labels = strings(config.maximum_refinement_candidates, 1);
cursor = 1;
for axis = 1:3
    for signValue = [-1.0, 1.0]
        column = config.control_blocks + axis;
        values(cursor, column) = min(max( ...
            anchor(column) + signValue * config.shooting_acceleration_perturbation_mps2, ...
            -config.outer_acceleration_correction_max_mps2), ...
            config.outer_acceleration_correction_max_mps2);
        direction = "NEG";
        if signValue > 0.0
            direction = "POS";
        end
        labels(cursor) = compose("CORRECTION_AXIS_%d_%s", axis, direction);
        cursor = cursor + 1;
    end
end
refinement = struct;
refinement.schema = "GPENMPC_MATLAB_NATIVE_ENMPC_REFINEMENT_LIBRARY_V1";
refinement.values = values;
refinement.labels = labels;
refinement.count = size(values, 1);
refinement.maximum_count = config.maximum_refinement_candidates;
refinement.anchor = anchor;
refinement.order = "AXIS_1_NEG_POS__AXIS_2_NEG_POS__AXIS_3_NEG_POS";
end
