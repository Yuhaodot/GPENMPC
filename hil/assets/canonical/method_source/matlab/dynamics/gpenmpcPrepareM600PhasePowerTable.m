function prepared = gpenmpcPrepareM600PhasePowerTable(profile)
%GPENMPCPREPAREM600PHASEPOWERTABLE Parse immutable phase-power knots once.
%
% Prepare numeric arrays for gpenmpcM600PhasePower to avoid repeated
% JSON-structure traversal and field-name parsing.

arguments
    profile (1,1) struct
end
table = profile.energy_model.phase_average_power_table_w;
payloadKnots = double(table.payload_knots_kg(:));
speedKnots = double(table.speed_knots_mps(:));
speedNames = fieldnames(table.values_by_speed_and_payload);
speedValues = cellfun(@(name) str2double(strrep( ...
    regexprep(name, "^x", ""), "_", ".")), speedNames);
phaseMatrices = zeros(numel(speedKnots), numel(payloadKnots), 4);
for speedIndex = 1:numel(speedKnots)
    [~, nearestSpeed] = min(abs(speedValues - speedKnots(speedIndex)));
    speedData = table.values_by_speed_and_payload.(speedNames{nearestSpeed});
    payloadNames = fieldnames(speedData);
    payloadValues = cellfun(@(name) str2double(strrep( ...
        regexprep(name, "^x", ""), "_", ".")), payloadNames);
    for payloadIndex = 1:numel(payloadKnots)
        [~, nearestPayload] = min(abs(payloadValues - payloadKnots(payloadIndex)));
        values = double(speedData.(payloadNames{nearestPayload}));
        phaseMatrices(speedIndex, payloadIndex, :) = reshape(values(1:4), 1, 1, 4);
    end
end
prepared = struct;
prepared.schema = "GPENMPC_M600_PREPARED_PHASE_POWER_TABLE_V1";
prepared.payload_knots_kg = payloadKnots;
prepared.speed_knots_mps = speedKnots;
prepared.payload_span_inverse_kg = 1.0 ./ diff(payloadKnots);
prepared.speed_span_inverse_mps = 1.0 ./ diff(speedKnots);
prepared.phase_matrices_w = phaseMatrices;
prepared.phase_order = ["ASCEND", "DESCEND", "FORWARD", "HOVER"];
end
