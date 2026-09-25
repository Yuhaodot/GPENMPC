function [forceWorld,torqueBody,e] = passiveGroundDissipation(velocityWorld,omegaBody,massKg,inertiaBody,normalForceN,mu,contactParameters)
%PASSIVEGROUNDDISSIPATION HOST candidate, not measured M600 landing gear.
% World XY velocity [m/s], body omega [rad/s], body inertia [kg m^2].
% Positive normal force [N] is supplied by the contact law.
% Returns world XY force [N] (Z exactly zero), body torque [N m].
% No state reset, normal-force modification, static support geometry or IO.
assert(isnumeric(velocityWorld)&&isreal(velocityWorld)&&isequal(size(velocityWorld),[3 1])&&all(isfinite(velocityWorld)), ...
    'm600check:PassiveVelocity','Expected finite real 3x1 world velocity.');
assert(isnumeric(omegaBody)&&isreal(omegaBody)&&isequal(size(omegaBody),[3 1])&&all(isfinite(omegaBody)), ...
    'm600check:PassiveOmega','Expected finite real 3x1 body angular velocity.');
assert(isnumeric(massKg)&&isreal(massKg)&&isscalar(massKg)&&isfinite(massKg)&&massKg>0, ...
    'm600check:PassiveMass','Expected positive finite mass in kg.');
assert(isnumeric(inertiaBody)&&isreal(inertiaBody)&&isequal(size(inertiaBody),[3 3])&&all(isfinite(inertiaBody(:))) ...
    &&norm(inertiaBody-inertiaBody.','fro')<=1e-12*max(1,norm(inertiaBody,'fro')), ...
    'm600check:PassiveInertia','Expected finite symmetric body inertia.');
[~,cholFailure]=chol(inertiaBody);
assert(cholFailure==0,'m600check:PassiveInertia','Body inertia must be positive definite.');
assert(isnumeric(normalForceN)&&isreal(normalForceN)&&isscalar(normalForceN)&&isfinite(normalForceN)&&normalForceN>=0, ...
    'm600check:PassiveNormalForce','Normal support force must be finite and nonnegative.');
assert(isnumeric(mu)&&isreal(mu)&&isscalar(mu)&&isfinite(mu)&&mu>=0, ...
    'm600check:PassiveMu','Engineering friction coefficient must be finite and nonnegative.');
assert(isstruct(contactParameters)&&isscalar(contactParameters)&&isfield(contactParameters,'damping_ratio') ...
    &&isfield(contactParameters,'static_deflection_m'),'m600check:PassiveParameters','Contact damping and deflection are required.');
zeta=contactParameters.damping_ratio;deflection=contactParameters.static_deflection_m;
assert(isnumeric(zeta)&&isreal(zeta)&&isscalar(zeta)&&isfinite(zeta)&&zeta>0 ...
    &&isnumeric(deflection)&&isreal(deflection)&&isscalar(deflection)&&isfinite(deflection)&&deflection>0, ...
    'm600check:PassiveParameters','Contact damping and static deflection must be positive finite values.');
c=2*double(zeta)*sqrt(9.80665/double(deflection));
rEff=min(sqrt(diag(double(inertiaBody))/double(massKg)));
desiredForce=-double(massKg)*c*[double(velocityWorld(1:2));0];
desiredTorque=-c*double(inertiaBody)*double(omegaBody);
combinedDesired=norm([desiredForce;desiredTorque/rEff]);
cap=double(mu)*double(normalForceN);
assert(all(isfinite([c;rEff;desiredForce;desiredTorque;combinedDesired;cap]))&&rEff>0, ...
    'm600check:PassiveOverflow','Derived candidate wrench must be finite.');
if normalForceN==0||cap==0
    scale=0;
elseif combinedDesired==0
    scale=1;
else
    ratio=combinedDesired/cap;
    if ratio<sqrt(eps)
        scale=1-ratio^2/3; % stable tanh(ratio)/ratio at zero, same limiting law
    else
        scale=tanh(ratio)/ratio;
    end
end
forceWorld=desiredForce*scale;torqueBody=desiredTorque*scale;
power=dot(forceWorld,double(velocityWorld))+dot(torqueBody,double(omegaBody));
e=struct('schema','HOST_PASSIVE_GROUND_DISSIPATION_CANDIDATE_V1', ...
    'engineering_candidate_not_measured_gear',true,'mu',double(mu),'normal_force_unchanged_N',double(normalForceN), ...
    'damping_rate_per_s',c,'effective_gyration_radius_m',rEff, ...
    'effective_radius_not_measured_footprint',true,'desired_force_world_N',desiredForce, ...
    'desired_torque_body_Nm',desiredTorque,'combined_desired_N',combinedDesired, ...
    'friction_cap_N',cap,'scale',scale,'combined_actual_N',norm([forceWorld;torqueBody/rEff]), ...
    'power_W',power,'normal_force_delta_N',0,'state_reset',false,'hardware_actions',0);
end
