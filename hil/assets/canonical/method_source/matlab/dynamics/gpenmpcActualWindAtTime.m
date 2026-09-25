function windXyMps = gpenmpcActualWindAtTime(mission, estimatedDurationS, timeS)
%GPENMPCACTUALWINDATTIME Evaluate the common software wind realization.

spec = mission.actual_wind;
windXyMps = double(spec.base_mean_xy_mps(:));
time = double(timeS);
durationEstimate = max(double(estimatedDurationS), 1e-9);

if isfield(spec, "persistent_shift_start_fraction") ...
        && ~isempty(spec.persistent_shift_start_fraction)
    start = double(spec.persistent_shift_start_fraction) .* durationEstimate;
    transition = double(spec.persistent_shift_transition_duration_s);
    if time >= start %#ok<BDSCI>
        if transition > 0
            progress = min(max((time - start) ./ transition, 0), 1);
            gain = 0.5 .* (1.0 - cos(pi .* progress));
        else
            gain = 1.0;
        end
        windXyMps = windXyMps ...
            + gain .* double(spec.persistent_shift_xy_mps(:));
    end
end

if isfield(spec, "gust_start_fraction") ...
        && ~isempty(spec.gust_start_fraction) ...
        && double(spec.gust_duration_s) > 0
    start = double(spec.gust_start_fraction) .* durationEstimate;
    duration = double(spec.gust_duration_s);
    if time >= start && time <= start + duration %#ok<BDSCI>
        phase = (time - start) ./ duration;
        gain = 0.5 .* (1.0 - cos(2.0 .* pi .* phase));
        direction = double(spec.gust_direction_xy(:));
        direction = direction ./ max(norm(direction), 1e-15);
        windXyMps = windXyMps ...
            + double(spec.gust_amplitude_mps) .* gain .* direction;
    end
end
end
