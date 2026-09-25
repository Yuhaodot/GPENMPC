function [stateUp,memory,contact] = initialStateFromNed( ...
    positionNed,eulerNedRad,payloadKg,p)
%#codegen
%INITIALSTATEFROMNED Ground-reset state with exact current frame conventions.
% Euler input is aerospace ZYX [roll;pitch;yaw] in radians, body FRD to NED.
% Velocity, rate and six thrust states are zero; no RPM state is invented.
assert(isequal(size(positionNed),[3,1]) &&all(isfinite(positionNed)));
assert(isequal(size(eulerNedRad),[3,1]) &&all(isfinite(eulerNedRad)));
assert(isfinite(payloadKg) &&payloadKg>=0);
mass=p.profile.mass_properties.base_mass_kg+payloadKg+p.mission.plant_mismatch.mass_bias_kg;
assert(isfinite(mass) &&mass>0);
half=eulerNedRad/2;
cr=cos(half(1));sr=sin(half(1));cp=cos(half(2));sp=sin(half(2));
cy=cos(half(3));sy=sin(half(3));
qNed=[cr*cp*cy+sr*sp*sy;sr*cp*cy-cr*sp*sy; ...
    cr*sp*cy+sr*cp*sy;cr*cp*sy-sr*sp*cy];
qNed=qNed/max(norm(qNed),1e-15);
stateUp=zeros(19,1);
stateUp(1:3)=positionNed.*[1;1;-1];
stateUp(7:10)=qNed.*[1;-1;-1;1];
memory=m600check.initialStepMemory();
contact=m600check.contactKernel(stateUp,mass,p.contact);
% LiveHilPlantService initializes at z=0 with 0.50 s ground support.
% Airborne test initial poses reset the prior ground-support state.
if positionNed(3)~=0
    memory.ground_support_dwell_s=0;
    memory.ground_confirmed=false;
end
assert(contact.contact_overtravel_m<=1e-12);
end
