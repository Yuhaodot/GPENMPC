function [stateNext,diagnostic,contact,memoryNext,status] = stepPx4Rk4( ...
    stateUp,controls,referenceJet,payloadKg,windXyMps,timeS,p,memory)
%#codegen
%STEPPX4RK4 Fixed 0.010-s version of LiveHilPlantService.integrate.
% Same RK4/reference extrapolation/q normalization/thrust clamp/contact test.
% State is z-up19, reference is z-up12, controls is PX4 normalized thrust16.
% Parameter p is from packParameters; memory is from initialStepMemory.
% Failure never commits candidate state/observer/time. status.candidate_state
% preserves the failed numerical endpoint; the caller MUST stop on !accepted.
% failed is an explicit adapter latch replacing an escaping host exception.
assert(isequal(size(stateUp),[19,1]) && isequal(size(controls),[16,1]));
assert(isequal(size(referenceJet),[12,1]) && isequal(size(windXyMps),[2,1]));
dt=0.01;
stateNext=stateUp;
memoryNext=memory;
[diagnostic,contact]=blankOutputs();
status=struct('accepted',false,'failure_code',uint8(0), ...
    'input_valid',false,'candidate_finite',false,'contact_overtravel',false, ...
    'candidate_state',stateUp,'next_time_s',timeS,'step_dt_s',dt, ...
    'rotor_state_clamped',false,'quaternion_normalized',false, ...
    'liftoff_this_step',false,'contact_entry_this_step',false);
if memory.failed
    status.failure_code=uint8(5); % latched earlier failure: no continuation
    return
end
mass=p.profile.mass_properties.base_mass_kg+payloadKg ...
    +p.mission.plant_mismatch.mass_bias_kg;
active=controls(1:6);
valid=all(isfinite(stateUp)) &&isfinite(norm(stateUp(7:10))) &&norm(stateUp(7:10))>1e-15 ...
    &&all(isfinite(active)) &&all(active>=0) &&all(active<=1) ...
    &&all(isfinite(referenceJet)) &&all(isfinite(windXyMps)) ...
    &&isfinite(payloadKg) &&payloadKg>=0 &&isfinite(mass) &&mass>0 ...
    &&isfinite(timeS) &&isfinite(memory.ground_support_dwell_s) ...
    &&memory.ground_support_dwell_s>=0;
status.input_valid=valid;
if ~valid
    status.failure_code=uint8(1);
    memoryNext.failed=true;
    return
end
rotor=active([5;1;4;6;2;3]) ...
    *p.calibration.rotor_allocation.per_rotor_thrust_upper_n;
reference=jetReference(referenceJet);
mid=gpenmpcReferenceExtrapolation(reference,0.5*dt);
next=gpenmpcReferenceExtrapolation(reference,dt);
midJet=referenceVector(mid);nextJet=referenceVector(next);
if ~all(isfinite(midJet)) ||~all(isfinite(nextJet))
    status.failure_code=uint8(2);memoryNext.failed=true;return
end
[k1,~,~]=m600check.derivativeSoftware(stateUp,rotor,referenceJet, ...
    payloadKg,windXyMps,timeS,p);
trial=stateUp+0.5*dt*k1;
if ~validTrial(trial)
    status.failure_code=uint8(2);status.candidate_state=trial;
    memoryNext.failed=true;return
end
[k2,~,~]=m600check.derivativeSoftware(trial,rotor,midJet, ...
    payloadKg,windXyMps,timeS+0.5*dt,p);
trial=stateUp+0.5*dt*k2;
if ~validTrial(trial)
    status.failure_code=uint8(2);status.candidate_state=trial;
    memoryNext.failed=true;return
end
[k3,~,~]=m600check.derivativeSoftware(trial,rotor,midJet, ...
    payloadKg,windXyMps,timeS+0.5*dt,p);
trial=stateUp+dt*k3;
if ~validTrial(trial)
    status.failure_code=uint8(2);status.candidate_state=trial;
    memoryNext.failed=true;return
end
[k4,~,~]=m600check.derivativeSoftware(trial,rotor,nextJet, ...
    payloadKg,windXyMps,timeS+dt,p);
state=stateUp+dt*(k1+2*k2+2*k3+k4)/6;
status.candidate_state=state;
if ~validTrial(state)
    status.failure_code=uint8(2);memoryNext.failed=true;return
end
state(7:10)=state(7:10)/max(norm(state(7:10)),1e-15);
upper=p.calibration.rotor_allocation.per_rotor_thrust_upper_n;
status.rotor_state_clamped=any(state(14:19)<0)||any(state(14:19)>upper);
state(14:19)=min(max(state(14:19),0),upper);
status.quaternion_normalized=true;
status.candidate_state=state;
[~,diagnostic,contact]=m600check.derivativeSoftware(state,rotor,nextJet, ...
    payloadKg,windXyMps,timeS+dt,p);
status.candidate_finite=all(isfinite(state)) ...
    &&all(isfinite(diagnostic.actual_acceleration_mps2)) ...
    &&isfinite(contact.contact_force_n);
if ~status.candidate_finite
    status.failure_code=uint8(2);memoryNext.failed=true;return
end
% The authority checks ONLY the integrated endpoint, not RK trial states.
status.contact_overtravel=contact.contact_overtravel_m>1e-12;
if status.contact_overtravel
    status.failure_code=uint8(3);memoryNext.failed=true;return
end
status.contact_entry_this_step=contact.contact_active &&~memory.previous_contact_active;
if status.contact_entry_this_step
    memoryNext.contact_entry_count=memory.contact_entry_count+uint64(1);
end
memoryNext.previous_contact_active=contact.contact_active;
surfaceHeight=0.0;
if isfield(p.contact,'surface_height_up_m'),surfaceHeight=p.contact.surface_height_up_m;end
groundClearance=state(3)-surfaceHeight;
status.liftoff_this_step=~memory.airborne_observed &&groundClearance>0.02;
memoryNext.airborne_observed=memory.airborne_observed||groundClearance>0.02;
supportCandidate=contact.contact_active &&groundClearance<=0.10 &&abs(state(6))<=0.15;
if supportCandidate
    memoryNext.ground_support_dwell_s=memory.ground_support_dwell_s+dt;
else
    memoryNext.ground_support_dwell_s=0.0;
end
memoryNext.ground_confirmed=memoryNext.ground_support_dwell_s+1e-12>=0.50;
memoryNext.plant_step_count=memory.plant_step_count+uint64(1);
stateNext=state;
status.accepted=true;
status.next_time_s=timeS+dt;
end

function valid=validTrial(x)
valid=all(isfinite(x)) &&isfinite(norm(x(7:10))) &&norm(x(7:10))>1e-15;
end

function reference=jetReference(jet)
reference=struct('position_m',jet(1:3),'velocity_mps',jet(4:6), ...
    'acceleration_mps2',jet(7:9),'jerk_mps3',jet(10:12));
end

function jet=referenceVector(reference)
jet=[reference.position_m;reference.velocity_mps; ...
    reference.acceleration_mps2;reference.jerk_mps3];
end

function [d,c]=blankOutputs()
d=struct('actual_acceleration_mps2',zeros(3,1),'actual_airspeed_mps',0, ...
    'actual_air_velocity_mps',zeros(3,1),'true_drag_n',zeros(3,1), ...
    'true_wrench',zeros(4,1),'structured_acceleration_mps2',zeros(3,1), ...
    'fast_acceleration_mps2',zeros(3,1),'true_mass_kg',0);
c=struct('contact_active',false,'contact_force_n',0,'support_force_n',0, ...
    'elastic_force_n',0,'damping_force_n',0,'damping_engagement_fraction',0, ...
    'damping_engagement_depth_m',0,'bump_stop_force_n',0, ...
    'contact_deflection_m',0,'contact_compression_rate_mps',0, ...
    'contact_overtravel_m',0,'stiffness_n_per_m',0,'damping_n_s_per_m',0, ...
    'static_deflection_m',0,'maximum_deflection_m',0, ...
    'vertical_acceleration_up_mps2',0,'force_continuous_at_first_contact',true, ...
    'tensile_force_n',0,'plant_truth_used_for_command',false);
end
