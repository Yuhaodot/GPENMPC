function [y,diagnostic] = copterSimIoCore( ...
    controls,reset,initialPositionNed,initialEulerNedRad,environment,p)
%#codegen
%COPTERSIMIOCORE Single fixed-10ms current M600 plant for an I/O shell.
% p is a fixed numeric parameter input (compile-time constant recommended),
% prepared once by prepareConstantParameters; this core performs no file I/O.
% environment has reference_jet_ned(12x1), payload_kg, wind_xy_mps(2x1).
% reset=true initializes and emits t=0 WITHOUT advancing. Every subsequent
% reset=false call attempts exactly one 10ms step. Do not call at another rate.
% A fault freezes the accepted state/time and stays failed until explicit new
% run reset. The shell MUST cease its scientific run on diagnostic.failed.
persistent x memory simTime acceleration angularAcceleration lastMass lastWind lastContact initialized faultCode
if isempty(initialized)
    [x,memory,lastContact]=m600check.initialStateFromNed( ...
        initialPositionNed,initialEulerNedRad,environment.payload_kg,p);
    simTime=0.0;acceleration=zeros(3,1);angularAcceleration=zeros(3,1);
    lastMass=p.profile.mass_properties.base_mass_kg+environment.payload_kg ...
        +p.mission.plant_mismatch.mass_bias_kg;
    lastWind=zeros(2,1);
    initialized=true;faultCode=uint8(0);
    reset=true;
end
assert(isscalar(reset));
assert(isequal(size(environment.reference_jet_ned),[12,1]));
assert(isequal(size(environment.wind_xy_mps),[2,1]));
jet=environment.reference_jet_ned;
for k=0:3
    jet(3*k+(1:3))=jet(3*k+(1:3)).*[1;1;-1];
end
didReset=false;accepted=false;liftoff=false;contactEntry=false;
candidateContactForce=lastContact.contact_force_n;
candidateOvertravel=0.0;
if reset
    [x,memory,lastContact]=m600check.initialStateFromNed( ...
        initialPositionNed,initialEulerNedRad,environment.payload_kg,p);
    assert(all(isfinite(environment.wind_xy_mps)) &&all(isfinite(jet)));
    assert(isequal(size(controls),[16,1]) &&all(isfinite(controls(1:6))) ...
        &&all(controls(1:6)>=0) &&all(controls(1:6)<=1));
    rotor=controls([5;1;4;6;2;3])*p.calibration.rotor_allocation.per_rotor_thrust_upper_n;
    [initialDerivative,initialDiagnostic,lastContact]=m600check.derivativeSoftware( ...
        x,rotor,jet,environment.payload_kg,environment.wind_xy_mps,0,p);
    simTime=0.0;acceleration=initialDiagnostic.actual_acceleration_mps2;
    angularAcceleration=initialDerivative(11:13);
    lastMass=initialDiagnostic.true_mass_kg;lastWind=environment.wind_xy_mps;
    faultCode=uint8(0);didReset=true;
    candidateContactForce=lastContact.contact_force_n;
else
    [next,d,c,mn,s]=m600check.stepPx4Rk4(x,controls,jet, ...
        environment.payload_kg,environment.wind_xy_mps,simTime,p,memory);
    memory=mn;
    accepted=s.accepted;
    liftoff=s.liftoff_this_step;contactEntry=s.contact_entry_this_step;
    candidateContactForce=c.contact_force_n;
    candidateOvertravel=c.contact_overtravel_m;
    if s.accepted
        x=next;acceleration=d.actual_acceleration_mps2;
        inertia=diag(p.calibration.mass_inertia.inertia_nominal_kg_m2);
        angularAcceleration=inertia\(d.true_wrench(2:4)-cross(x(11:13),inertia*x(11:13)));
        % Publish the COMPLETE angular acceleration for the optional new
        % passive-contact plant. d.true_wrench remains the rotor wrench;
        % contact torque must not masquerade as an actuator command.
        if isfield(p,'ground_dissipation')
            if p.ground_dissipation.enabled &&c.contact_force_n>0
                [~,groundTorque]=m600check.passiveGroundDissipation( ...
                    x(4:6),x(11:13),d.true_mass_kg,inertia,c.contact_force_n, ...
                    p.ground_dissipation.coefficient_of_friction,p.contact);
                angularAcceleration=angularAcceleration+inertia\groundTorque;
            end
        end
        lastMass=d.true_mass_kg;
        lastContact=c;lastWind=environment.wind_xy_mps;simTime=s.next_time_s;
    elseif faultCode==0
        faultCode=s.failure_code;
    end
end
y=m600check.copterSimOutputs(x,acceleration,lastWind,angularAcceleration,lastMass);
diagnostic=struct('state_up',x,'sim_time_s',simTime,'fixed_step_s',0.01, ...
    'reset_applied',didReset,'step_accepted',accepted, ...
    'observation_valid',~memory.failed,'failed',memory.failed,'failure_code',faultCode, ...
    'plant_step_count',memory.plant_step_count, ...
    'contact_force_n',lastContact.contact_force_n, ...
    'contact_active',lastContact.contact_active, ...
    'contact_deflection_m',lastContact.contact_deflection_m, ...
    'candidate_contact_force_n',candidateContactForce, ...
    'candidate_overtravel_m',candidateOvertravel, ...
    'contact_entry_count',memory.contact_entry_count, ...
    'airborne_observed',memory.airborne_observed,'liftoff_this_step',liftoff, ...
    'contact_entry_this_step',contactEntry, ...
    'ground_confirmed',memory.ground_confirmed, ...
    'ground_support_dwell_s',memory.ground_support_dwell_s, ...
    'source_native_sensor_only',true,'software_plant_instances',uint8(1));
end
