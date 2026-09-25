function [matched, diagnostic] = applyRelaunchStateMatch(state, nominal, serviceOrdinal, servicePhase, elapsedS, durationS)
%APPLYRELAUNCHSTATEMATCH Apply C3 horizontal removal and vertical frame transform.
arguments
    state (1,1) struct
    nominal (1,1) struct
    serviceOrdinal (1,1) double {mustBeInteger,mustBeNonnegative}
    servicePhase (1,1) string
    elapsedS (1,1) double {mustBeFinite,mustBeNonnegative}
    durationS (1,1) double {mustBeFinite,mustBeNonnegative}
end
required = ["position_ned_m","velocity_ned_mps","acceleration_ned_mps2","jerk_ned_mps3"];
for name = required
    assert(isfield(nominal,name),"gpenmpcHil:BadReference", ...
        "Missing reference field %s",name);
end
matched = nominal;
matched.position_ned_m = reshape(double(nominal.position_ned_m),1,3);
matched.velocity_ned_mps = reshape(double(nominal.velocity_ned_mps),1,3);
matched.acceleration_ned_mps2 = reshape(double(nominal.acceleration_ned_mps2),1,3);
matched.jerk_ned_mps3 = reshape(double(nominal.jerk_ned_mps3),1,3);
vertical = [0.0,0.0,state.current_vertical_frame_offset_m];
matched.position_ned_m = matched.position_ned_m + vertical;
diagnostic = struct("active",false,"anchor_missing",false, ...
    "vertical_frame_transform_active",state.capture_count>0, ...
    "vertical_frame_offset_m",state.current_vertical_frame_offset_m, ...
    "remaining_fraction",0.0,"horizontal_offset_m",0.0, ...
    "plant_truth_used_for_command",false);
if servicePhase ~= "SERVICE_ASCENT"
    return
end
if serviceOrdinal < 1 || serviceOrdinal > numel(state.anchor_valid) || ~state.anchor_valid(serviceOrdinal)
    diagnostic.anchor_missing = true;
    return
end
assert(durationS>0,"gpenmpcHil:InvalidRelaunchDuration", ...
    "Service-ascent duration must be positive");
[s,ds,d2s,d3s] = smoothstep7(elapsedS,durationS);
remaining = 1.0-s;
offset = state.horizontal_offset_ned_m(serviceOrdinal,:);
matched.position_ned_m = matched.position_ned_m + remaining*offset;
matched.velocity_ned_mps = matched.velocity_ned_mps - ds*offset;
matched.acceleration_ned_mps2 = matched.acceleration_ned_mps2 - d2s*offset;
matched.jerk_ned_mps3 = matched.jerk_ned_mps3 - d3s*offset;
diagnostic.active = elapsedS < durationS;
diagnostic.remaining_fraction = remaining;
diagnostic.horizontal_offset_m = norm(offset(1:2));
diagnostic.elapsed_s = min(max(elapsedS,0.0),durationS);
diagnostic.duration_s = durationS;
diagnostic.service_ordinal = serviceOrdinal;
end

function [s,ds,d2s,d3s] = smoothstep7(elapsedS,durationS)
tau = min(max(elapsedS/durationS,0.0),1.0);
s = 35*tau^4-84*tau^5+70*tau^6-20*tau^7;
ds = (140*tau^3-420*tau^4+420*tau^5-140*tau^6)/durationS;
d2s = (420*tau^2-1680*tau^3+2100*tau^4-840*tau^5)/durationS^2;
d3s = (840*tau-5040*tau^2+8400*tau^3-4200*tau^4)/durationS^3;
end
