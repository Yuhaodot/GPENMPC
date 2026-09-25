function biasXyMps = gpenmpcWindMeasurementBiasAtTime(mission, timeS)
%GPENMPCWINDMEASUREMENTBIASATTIME Evaluate shared causal wind-estimate bias.

spec = mission.actual_wind;
biasXyMps = zeros(2, 1);
if ~isfield(spec, "structured_estimation_bias")
    return
end
structured = spec.structured_estimation_bias;
amplitude = double(structured.amplitude_mps);
if amplitude <= 0
    return
end
period = max(double(structured.period_s), 1e-6);
ramp = max(double(structured.ramp_s), 1e-6);
base = double(spec.base_mean_xy_mps(:));
if norm(base) < 1e-9
    direction = [0; 1];
else
    base = base ./ norm(base);
    direction = [-base(2); base(1)];
end
time = max(double(timeS), 0);
rampGain = 0.5 .* (1.0 - cos(pi .* min(time ./ ramp, 1.0)));
waveform = 0.70 + 0.30 .* sin(2.0 .* pi .* time ./ period);
biasXyMps = amplitude .* rampGain .* waveform .* direction;
end
