function [y,diagnostic]=copterSimFlatTerrainCore( ...
    controls,reset,initialWorldPositionNed,initialEulerNedRad,environment,terrain,p)
%#codegen
%COPTERSIMFLATTERRAINCORE Fixed world datum with sampled terrain contact height.
% terrain(1) is the independently observed WORLD NED ground z, never inferred
% from initial aircraft altitude. terrain is the official 15x1 TerrainIn15d.
% A new run requires explicit reset=true. Only that event locks the world
% datum. Later TerrainIn15d samples describe the surface at current XY, not
% a new coordinate origin. Feed that height to the existing vertical contact
% law without moving aircraft state/reference or changing contact parameters.
% This is height-only ground contact, not slope-normal/landing-gear geometry.
% diagnostic.state_up stays LOCAL z-up; world position has a separate field.
%
% Nonfinite terrain or a missing initial reset latches failure_code=4 and
% freezes the last returned accepted state/time without invoking the core.
% A later good terrain sample cannot clear this fault. Only an explicit new
% reset can clear it. Before any accepted reset, finite placeholder fields
% carry state_snapshot_available=false and observation_valid=false; they are
% not simulated or measured aircraft state and MUST NOT enter a HIL stream.
assert(isequal(size(terrain),[15,1])&&isscalar(reset));
assert(isequal(size(initialWorldPositionNed),[3,1]));
assert(isequal(size(environment.reference_jet_ned),[12,1]));
persistent initialized lockedTerrainZ terrainFault terrainReason lastY lastD snapshotAvailable
if isempty(initialized)
    initialized=false;lockedTerrainZ=0.0;terrainFault=false;terrainReason=uint8(0);
    x=zeros(19,1);x(7)=1;
    lastY=m600check.copterSimOutputs(x,zeros(3,1),zeros(2,1),zeros(3,1), ...
        p.profile.mass_properties.base_mass_kg);
    lastD=blankDiagnostic(x);snapshotAvailable=false;
end
finiteTerrain=all(isfinite(terrain));
if reset&&finiteTerrain
    lockedTerrainZ=terrain(1);terrainFault=false;terrainReason=uint8(0);
    initialized=true;
elseif ~terrainFault
    if ~finiteTerrain
        terrainFault=true;terrainReason=uint8(2);
    elseif ~initialized
        terrainFault=true;terrainReason=uint8(1);
    end
end
if ~terrainFault
    localPosition=initialWorldPositionNed;
    localPosition(3)=localPosition(3)-lockedTerrainZ;
    localEnvironment=environment;
    localEnvironment.reference_jet_ned(3)=environment.reference_jet_ned(3)-lockedTerrainZ;
    terrainParameters=p;
    terrainParameters.contact.surface_height_up_m=lockedTerrainZ-terrain(1);
    [localY,coreD]=m600check.copterSimIoCore(controls,reset,localPosition, ...
        initialEulerNedRad,localEnvironment,terrainParameters);
    localY.position_ned_m(3)=localY.position_ned_m(3)+lockedTerrainZ;
    lastY=localY;lastD=coreD;snapshotAvailable=true;
end
y=lastY;diagnostic=lastD;
if terrainFault
    diagnostic.failed=true;diagnostic.observation_valid=false;
    diagnostic.failure_code=uint8(4);diagnostic.step_accepted=false;
    diagnostic.reset_applied=false;diagnostic.liftoff_this_step=false;
    diagnostic.contact_entry_this_step=false;
end
diagnostic.core_failure_code=lastD.failure_code;
diagnostic.terrain_fault_latched=terrainFault;
diagnostic.terrain_failure_reason=terrainReason; % 1=no explicit reset, 2=nonfinite; old 3 retained in raw history
diagnostic.terrain_locked=initialized;
diagnostic.terrain_world_ned_z_m=lockedTerrainZ;
diagnostic.state_snapshot_available=snapshotAvailable;
diagnostic.world_position_ned_m=y.position_ned_m;
diagnostic.local_ground_relative_position_ned_m=y.position_ned_m-[0;0;terrain(1)];
diagnostic.flat_terrain_translation_only=finiteTerrain&&terrain(1)==lockedTerrainZ;
end

function d=blankDiagnostic(x)
d=struct('state_up',x,'sim_time_s',0.0,'fixed_step_s',0.01, ...
    'reset_applied',false,'step_accepted',false,'observation_valid',false, ...
    'failed',false,'failure_code',uint8(0),'plant_step_count',uint64(0), ...
    'contact_force_n',0.0,'contact_active',false,'contact_deflection_m',0.0, ...
    'candidate_contact_force_n',0.0,'candidate_overtravel_m',0.0, ...
    'contact_entry_count',uint64(0),'airborne_observed',false,'liftoff_this_step',false, ...
    'contact_entry_this_step',false,'ground_confirmed',false,'ground_support_dwell_s',0.0, ...
    'source_native_sensor_only',true,'software_plant_instances',uint8(1));
end
