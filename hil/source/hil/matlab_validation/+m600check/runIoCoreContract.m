function report = runIoCoreContract()
%RUNIOCORECONTRACT HOST-only output/frame/persistent-core checks.
% Neutral coefficients below are EXPLICIT numerical fixtures, not a mission
% or Cambridge run configuration. Original fixture/assets remain untouched.
fixture=m600check.loadFixture();p=fixture.parameters;
p.mission.plant_mismatch.mass_bias_kg=0;
p.mission.plant_mismatch.acceleration_bias_inertial_mps2=zeros(3,1);
p.mission.plant_mismatch.external_acceleration_amplitude_mps2=zeros(3,1);
p.mission.structured_residual.enabled=false;
environment=struct('reference_jet_ned',zeros(12,1), ...
    'payload_kg',0,'wind_xy_mps',zeros(2,1));
u=nan(16,1);u(1:6)=0;
rows=struct('name',{},'passed',{},'maximum_absolute_difference',{});
angles=[0,0,0;.2,0,0;0,-.3,0;0,0,.7;-.2,.3,-1.1];
for k=1:size(angles,1)
    euler=angles(k,:).';
    for altitude=[0,1]
        position=[3;4;-altitude];
        [y,d]=m600check.copterSimIoCore(u,true,position,euler,environment,p);
        assert(d.reset_applied &&~d.step_accepted &&~d.failed &&d.plant_step_count==0);
        assert(d.sim_time_s==0 &&y.sample_period_s==.01);
        err=near(y.position_ned_m,position);
        err=max(err,near(y.euler_rpy_rad,euler));
        err=max(err,near(y.rotation_body_from_ned,y.rotation_ned_from_body.'));
        err=max(err,near(y.rotation_body_from_ned*y.rotation_ned_from_body,eye(3)));
        if altitude==0
            expectedSpecific=y.rotation_body_from_ned*[0;0;-9.80665];
            assert(d.ground_confirmed &&d.contact_active);
        else
            expectedSpecific=zeros(3,1);
            assert(~d.ground_confirmed &&~d.contact_active);
        end
        err=max(err,near(y.specific_force_body_frd_mps2,expectedSpecific));
        err=max(err,near(y.angular_acceleration_body_frd_rad_s2,zeros(3,1)));
        rows(end+1)=row(sprintf('reset_orientation_%d_altitude_%g',k,altitude),err); %#ok<AGROW>
    end
end
% Body wind and body air are distinct; neutral test observation only.
x=zeros(19,1);x(7)=1;x(4:6)=[2;-.5;.3];
y=m600check.copterSimOutputs(x,zeros(3,1),[1;-.2],zeros(3,1),9.5);
err=near(y.air_velocity_body_frd_mps,y.velocity_body_frd_mps-y.wind_body_frd_mps);
rows(end+1)=row('body_air_equals_body_velocity_minus_body_wind',err);
% One accepted step has the original integrated-state angular acceleration.
[~,~]=m600check.copterSimIoCore(u,true,[0;0;-1],[0;0;0],environment,p);
u(1)=.2;
[y,d]=m600check.copterSimIoCore(u,false,[0;0;-1],[0;0;0],environment,p);
assert(d.step_accepted &&~d.failed &&d.plant_step_count==1 &&d.sim_time_s==.01);
ref=struct('position_m',zeros(3,1),'velocity_mps',zeros(3,1), ...
    'acceleration_mps2',zeros(3,1),'jerk_mps3',zeros(3,1));
rotor=u([5;1;4;6;2;3])*p.calibration.rotor_allocation.per_rotor_thrust_upper_n;
[oracle,~]=gpenmpcM600SixDofPlantDerivative(d.state_up,rotor,ref,0,zeros(2,1), ...
    d.sim_time_s,p.mission,p.calibration,p.profile);
err=near(y.angular_acceleration_body_frd_rad_s2,oracle(11:13).*[-1;-1;1]);
assert(norm(y.angular_acceleration_body_frd_rad_s2)>0);
rows(end+1)=row('accepted_dwb_is_actual_endpoint_derivative',err);
% Fault state cannot masquerade as fresh telemetry or advance a hidden step.
previous=d;bad=u;bad(1)=NaN;
[~,failed]=m600check.copterSimIoCore(bad,false,[0;0;-1],[0;0;0],environment,p);
assert(failed.failed &&~failed.observation_valid &&failed.failure_code==1);
assert(failed.sim_time_s==previous.sim_time_s &&failed.plant_step_count==previous.plant_step_count);
near(failed.state_up,previous.state_up);
[~,latched]=m600check.copterSimIoCore(u,false,[0;0;-1],[0;0;0],environment,p);
assert(latched.failed &&latched.failure_code==1 &&~latched.step_accepted);
assert(latched.sim_time_s==previous.sim_time_s);
rows(end+1)=row('first_fault_preserved_no_state_time_advance',0);
% Explicit new numerical fixture reset, not autonomous failure recovery.
[~,reset]=m600check.copterSimIoCore(zeros(16,1),true,[0;0;0],zeros(3,1),environment,p);
assert(~reset.failed &&reset.plant_step_count==0 &&reset.sim_time_s==0);
rows(end+1)=row('explicit_new_fixture_reset',0);
report=struct('status','PASS_HOST_IO_CORE_CONTRACT_ONLY', ...
    'parameter_environment_role','EXPLICIT_NEUTRAL_NUMERIC_FIXTURE_NOT_MISSION', ...
    'neutral_delta',{{'mass bias=0','external acceleration bias/amplitude=0','structured residual disabled'}}, ...
    'source_manifest_sha256',fixture.source_manifest_sha256, ...
    'test_count',numel(rows),'passed_count',nnz([rows.passed]), ...
    'maximum_absolute_difference',max([rows.maximum_absolute_difference]), ...
    'tests',rows,'hardware_actions',0,'model_simulation_runs',0, ...
    'codegen_executed',false);
disp(jsonencode(report,PrettyPrint=true));
end

function r=row(name,err)
r=struct('name',name,'passed',true,'maximum_absolute_difference',err);
end

function err=near(a,b)
assert(isequal(size(a),size(b)) &&all(isfinite(a(:))) &&all(isfinite(b(:))));
err=max(abs(a(:)-b(:)));
assert(err<=5e-11*max(1,max(abs(b(:)))),'m600check:IoMismatch','I/O arithmetic differs %.17g.',err);
end
