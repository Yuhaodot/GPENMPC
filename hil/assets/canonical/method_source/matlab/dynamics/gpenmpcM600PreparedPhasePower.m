function [powerW, statusCode] = gpenmpcM600PreparedPhasePower( ...
        prepared, payloadKg, airspeedMps, verticalSpeedMps)
%GPENMPCM600PREPAREDPHASEPOWER Interpolate a pre-parsed M600 power table.

arguments
    prepared (1,1) struct
    payloadKg (1,1) double
    airspeedMps (1,1) double
    verticalSpeedMps (1,1) double
end
payloadKnots = double(prepared.payload_knots_kg(:));
speedKnots = double(prepared.speed_knots_mps(:));
payloadSpanInverse = double(prepared.payload_span_inverse_kg(:));
speedSpanInverse = double(prepared.speed_span_inverse_mps(:));
phaseMatrices = double(prepared.phase_matrices_w);
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

[payloadLower, payloadFraction] = interpolationBracket( ...
    payloadKnots, payloadSpanInverse, payload);
matrix = phaseMatrices(:, :, phase);
alongPayload = matrix(:, payloadLower) ...
    + payloadFraction .* (matrix(:, payloadLower + 1) ...
        - matrix(:, payloadLower));
[speedLower, speedFraction] = interpolationBracket( ...
    speedKnots, speedSpanInverse, speed);
powerW = alongPayload(speedLower) ...
    + speedFraction .* (alongPayload(speedLower + 1) ...
        - alongPayload(speedLower));
if phase == 3 && airspeed < speedKnots(1)
    hover = phaseMatrices(1, :, 4);
    hoverPower = hover(payloadLower) ...
        + payloadFraction .* (hover(payloadLower + 1) ...
            - hover(payloadLower));
    blend = min(max(airspeed ./ speedKnots(1), 0), 1);
    powerW = (1.0 - blend) .* hoverPower + blend .* powerW;
end
end


function [lower, fraction] = interpolationBracket(knots, spanInverse, value)
if value <= knots(1)
    lower = 1;
    fraction = 0.0;
elseif value >= knots(end)
    lower = numel(knots) - 1;
    fraction = 1.0;
else
    lower = find(knots <= value, 1, "last");
    lower = min(lower, numel(knots) - 1);
    fraction = (value - knots(lower)) .* spanInverse(lower);
end
end
