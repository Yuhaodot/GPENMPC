function contact = contactKernel(stateUp,trueMassKg,p)
%#codegen
%CONTACTKERNEL Numeric-only exact port of compliantContactState arithmetic.
% Oracle SHA D90339C9D83373C121AC3A4C5826E72420B6F29C1127344685626097AEC4C095.
% This removes only arguments/default/dynamic-field/schema-string handling.
% It preserves every numeric diagnostic and the original proxy intervals.
assert(numel(stateUp)==19 && all(isfinite(stateUp(:))));
assert(isscalar(trueMassKg) && isfinite(trueMassKg) && trueMassKg>0);
staticDeflection = p.static_deflection_m;
dampingRatio = p.damping_ratio;
maximumDeflection = p.maximum_deflection_m;
smoothForceFraction = p.smooth_force_fraction_of_weight;
bumpMultiplier = p.bump_stop_stiffness_multiplier;
engagementFraction = p.damping_engagement_depth_fraction_of_static_deflection;
assert(staticDeflection>=0.02 && staticDeflection<=0.05);
assert(dampingRatio>=0.8 && dampingRatio<=1.2);
assert(maximumDeflection>staticDeflection);
assert(smoothForceFraction>=0.005 && smoothForceFraction<=0.05);
assert(bumpMultiplier>=1.0 && abs(engagementFraction-1.0)<=1e-15);
gravity = 9.80665;
stiffness = trueMassKg*gravity/staticDeflection;
damping = 2.0*dampingRatio*sqrt(stiffness*trueMassKg);
surfaceHeight=0.0;
if isfield(p,'surface_height_up_m')
    surfaceHeight=p.surface_height_up_m;
    assert(isscalar(surfaceHeight)&&isfinite(surfaceHeight));
end
% World datum stays fixed; only penetration uses local terrain clearance.
deflection = max(0.0,staticDeflection-(stateUp(3)-surfaceHeight));
compressionRate = -stateUp(6);
engagementDepth = staticDeflection*engagementFraction;
normalized = min(max(deflection/engagementDepth,0.0),1.0);
dampingEngagement = normalized^2*(3.0-2.0*normalized);
elasticForce = stiffness*deflection;
dampingForce = damping*dampingEngagement*compressionRate;
if deflection>0.0
    rawForce = elasticForce+dampingForce;
else
    rawForce = 0.0;
end
transition = smoothForceFraction*trueMassKg*gravity;
if rawForce<=0.0
    supportForce = 0.0;
elseif rawForce>=transition
    supportForce = rawForce;
else
    normalized = rawForce/transition;
    supportForce = transition*normalized^2*(2.0-normalized);
end
overtravel = max(0.0,deflection-maximumDeflection);
bumpForce = bumpMultiplier*stiffness*overtravel;
totalForce = supportForce+bumpForce;
contact = struct('contact_active',totalForce>1e-9, ...
    'contact_force_n',totalForce,'support_force_n',supportForce, ...
    'elastic_force_n',elasticForce,'damping_force_n',dampingForce, ...
    'damping_engagement_fraction',dampingEngagement, ...
    'damping_engagement_depth_m',engagementDepth, ...
    'bump_stop_force_n',bumpForce,'contact_deflection_m',deflection, ...
    'contact_compression_rate_mps',compressionRate,'contact_overtravel_m',overtravel, ...
    'stiffness_n_per_m',stiffness,'damping_n_s_per_m',damping, ...
    'static_deflection_m',staticDeflection,'maximum_deflection_m',maximumDeflection, ...
    'vertical_acceleration_up_mps2',totalForce/trueMassKg, ...
    'force_continuous_at_first_contact',true,'tensile_force_n',0.0, ...
    'plant_truth_used_for_command',false);
end
