function [dx,diagnostic,contact,rotorCommandN] = derivativePx4( ...
    x,controls,referenceJet,payloadKg,windXyMps,timeS,p)
%#codegen
%DERIVATIVEPX4 Exact accepted normalized-thrust feedback adaptation.
% controls: 16-by-1 HIL actuator values; channels 1:6 must be in [0,1].
% Unused 7:16 may be NaN, matching adaptV7NominalControlFeedback. The first
% six represent linear normalized THRUST, not PWM-to-speed, RPM or voltage.
assert(isequal(size(controls),[16,1]));
active = controls(1:6);
assert(all(isfinite(active)) && all(active>=0) && all(active<=1));
rotorCommandN = active([5;1;4;6;2;3]) ...
    *p.calibration.rotor_allocation.per_rotor_thrust_upper_n;
[dx,diagnostic,contact] = m600check.derivativeSoftware( ...
    x,rotorCommandN,referenceJet,payloadKg,windXyMps,timeS,p);
end
