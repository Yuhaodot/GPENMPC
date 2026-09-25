function [dx,diagnostic,contact] = derivativeSoftware( ...
    x,rotorCommandN,referenceJet,payloadKg,windXyMps,timeS,p)
%#codegen
%DERIVATIVESOFTWARE Evaluate the 19-state MATLAB plant with ground contact.
% x: 19-by-1 z-up state [world p;world v;q_wxyz;body rates;6 thrust states N].
% rotorCommandN: 6-by-1 in software rotor-angle order, not PX4 order/RPM.
% referenceJet: 12-by-1 z-up [p;v;a;jerk]. Only v/a drive residual model.
% p is the fixed numeric nested struct from m600check.packParameters.
assert(isequal(size(x),[19,1]) && all(isfinite(x)));
assert(isequal(size(rotorCommandN),[6,1]) && all(isfinite(rotorCommandN)));
assert(isequal(size(referenceJet),[12,1]) && all(isfinite(referenceJet)));
assert(isequal(size(windXyMps),[2,1]) && all(isfinite(windXyMps)));
assert(isfinite(payloadKg) && payloadKg>=0 && isfinite(timeS));
assert(norm(x(7:10))>1e-15);
upper = p.calibration.rotor_allocation.per_rotor_thrust_upper_n;
assert(all(rotorCommandN>=0) && all(rotorCommandN<=upper));
mass = p.profile.mass_properties.base_mass_kg+payloadKg ...
    +p.mission.plant_mismatch.mass_bias_kg;
assert(isfinite(mass) && mass>0);
reference = struct('position_m',referenceJet(1:3), ...
    'velocity_mps',referenceJet(4:6), ...
    'acceleration_mps2',referenceJet(7:9),'jerk_mps3',referenceJet(10:12));
% The MATLAB path uses the plant function and its allocation, residual and
% quaternion dependencies.
if coder.target('MATLAB')
    [dx,diagnostic] = gpenmpcM600SixDofPlantDerivative(x,rotorCommandN, ...
        reference,payloadKg,windXyMps,timeS,p.mission,p.calibration,p.profile);
else
    % The generated counterpart uses fixed-field structs for MATLAB Coder.
    [dx,diagnostic] = m600check.generatedPlantDerivative(x,rotorCommandN, ...
        reference,payloadKg,windXyMps,timeS,p.mission,p.calibration,p.profile);
end
contact = m600check.contactKernel(x,mass,p.contact);
dx(6) = dx(6)+contact.vertical_acceleration_up_mps2;
diagnostic.actual_acceleration_mps2(3) = ...
    diagnostic.actual_acceleration_mps2(3)+contact.vertical_acceleration_up_mps2;
% Optional passive horizontal and rotational ground dissipation is a modeled
% contact effect. The branch is bypassed when disabled or the normal force is
% zero; free-flight dynamics then use the derivative computed above.
if isfield(p,'ground_dissipation')
    assert(islogical(p.ground_dissipation.enabled) ...
        &&isscalar(p.ground_dissipation.enabled));
    if p.ground_dissipation.enabled
        mu=p.ground_dissipation.coefficient_of_friction;
        assert(isscalar(mu)&&isfinite(mu)&&mu>=0);
        if contact.contact_force_n>0
            inertia=diag(p.calibration.mass_inertia.inertia_nominal_kg_m2);
            [groundForce,groundTorque]=m600check.passiveGroundDissipation( ...
                x(4:6),x(11:13),mass,inertia,contact.contact_force_n,mu,p.contact);
            dx(4:6)=dx(4:6)+groundForce/mass;
            dx(11:13)=dx(11:13)+inertia\groundTorque;
            diagnostic.actual_acceleration_mps2= ...
                diagnostic.actual_acceleration_mps2+groundForce/mass;
        end
    end
end
end
