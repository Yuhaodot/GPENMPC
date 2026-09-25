function [powerW, statusCode] = gpenmpcM600PhasePower( ...
        profile, payloadKg, airspeedMps, verticalSpeedMps)
%GPENMPCM600PHASEPOWER Interpolate the public M600 phase-power table.
%
% statusCode is 0 in the tabulated domain, 1 in the documented low-speed
% hover bridge and 2 when the requested speed is clipped above the table.

table = profile.energy_model.phase_average_power_table_w;
payloadKnots = double(table.payload_knots_kg(:));
speedKnots = double(table.speed_knots_mps(:));
airspeed = double(airspeedMps);
verticalSpeed = double(verticalSpeedMps);

if verticalSpeed > 0.10
    phase = 1;
elseif verticalSpeed < -0.10
    phase = 2;
elseif airspeed < 0.20
    phase = 4;
else
    phase = 3;
end

matrix = zeros(numel(speedKnots), numel(payloadKnots));
speedNames = fieldnames(table.values_by_speed_and_payload);
speedValues = cellfun(@(name) str2double(strrep( ...
    regexprep(name, "^x", ""), "_", ".")), speedNames);
for speedIndex = 1:numel(speedKnots)
    [~, nearestSpeed] = min(abs(speedValues - speedKnots(speedIndex)));
    speedData = table.values_by_speed_and_payload.(speedNames{nearestSpeed});
    payloadNames = fieldnames(speedData);
    payloadValues = cellfun(@(name) str2double(strrep( ...
        regexprep(name, "^x", ""), "_", ".")), payloadNames);
    for payloadIndex = 1:numel(payloadKnots)
        [~, nearestPayload] = min(abs(payloadValues - payloadKnots(payloadIndex)));
        values = double(speedData.(payloadNames{nearestPayload}));
        matrix(speedIndex, payloadIndex) = values(phase);
    end
end

payload = min(max(double(payloadKg), payloadKnots(1)), payloadKnots(end));
statusCode = uint8(0);
if phase == 3
    if airspeed > speedKnots(end)
        statusCode = uint8(2);
    elseif airspeed < speedKnots(1)
        statusCode = uint8(1);
    end
    speed = min(max(airspeed, speedKnots(1)), speedKnots(end));
else
    speed = speedKnots(1);
end

alongPayload = zeros(numel(speedKnots), 1);
for speedIndex = 1:numel(speedKnots)
    alongPayload(speedIndex) = interp1(payloadKnots, ...
        matrix(speedIndex, :), payload, "linear");
end
powerW = interp1(speedKnots, alongPayload, speed, "linear");

if phase == 3 && airspeed < speedKnots(1)
    hover = zeros(1, numel(payloadKnots));
    [~, nearestSpeed] = min(abs(speedValues - speedKnots(1)));
    speedData = table.values_by_speed_and_payload.(speedNames{nearestSpeed});
    payloadNames = fieldnames(speedData);
    payloadValues = cellfun(@(name) str2double(strrep( ...
        regexprep(name, "^x", ""), "_", ".")), payloadNames);
    for payloadIndex = 1:numel(payloadKnots)
        [~, nearestPayload] = min(abs(payloadValues - payloadKnots(payloadIndex)));
        values = double(speedData.(payloadNames{nearestPayload}));
        hover(payloadIndex) = values(4);
    end
    hoverPower = interp1(payloadKnots, hover, payload, "linear");
    blend = min(max(airspeed ./ speedKnots(1), 0), 1);
    powerW = (1.0 - blend) .* hoverPower + blend .* powerW;
end
end
