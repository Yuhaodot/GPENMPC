function report=run_passive_ground_dissipation_replay(outputDir)
% Replay retained disarmed p/v/q/omega with zero thrust and optional passive
% XY force and body torque through the plant derivative.
arguments,outputDir (1,1) string,end
assert(~isfolder(outputDir)&&~isfile(outputDir),'replay:OutputExists','No overwrite.');
build=string(fileparts(fileparts(mfilename('fullpath'))));previousPath=path;
addpath(fullfile(build,'matlab_validation'),fullfile(build,'m600_coptersim','matlab_validation'));
cleanup=onCleanup(@()path(previousPath)); %#ok<NASGU>
m600check.loadFixture(); % Load the numerical fixture.
model=fullfile(gpenmpc_external_path('flat_terrain_model'),'M600_CORE_PARAMETERS.mat');
assert(strcmpi(m600check.fileSha256(model),'969D347CE041715CAED49BE0B7897486E34FA4E0B22EB82E50E9752B8F4F19E4'), ...
    'replay:ModelIdentity','Original neutral model parameters changed.');
loaded=load(model,'parameters');p=loaded.parameters;
assert(p.contact.damping_ratio==.9&&p.contact.static_deflection_m==.035, ...
    'replay:ContactIdentity','Contact damping/deflection must remain unchanged.');
assert(~p.mission.structured_residual.enabled&&all(p.mission.plant_mismatch.thrust_effectiveness_by_rotor==1), ...
    'replay:NeutralFixture','Expected existing neutral hover model.');
payload=2.21;mass=p.profile.mass_properties.base_mass_kg+payload+p.mission.plant_mismatch.mass_bias_kg;
J=diag(p.calibration.mass_inertia.inertia_nominal_kg_m2);
assert(abs(mass-11.71)<1e-12&&isequal(J,diag([1.6 1.6 3])),'replay:MassIdentity','No mass/inertia retuning.');
input=string(gpenmpc_external_path('passive_ground_raw_fixture'));
assert(strcmpi(m600check.fileSha256(input),'904AD5D0F46771D0BF088787BC94CAC085850EFE1DEC703C294EDDB9EB95B3B6'), ...
    'replay:RawIdentity','Retained raw input changed.');
inventory=whos('-file',input);disp(struct2table(inventory));loaded=load(input,'result');r=loaded.result;clear loaded
events=r.events;didx=find(strcmp({events.kind},'STANDARD_DISARM_REQUEST'),1);
assert(~isempty(didx),'replay:Disarm','Missing real disarm event.');
request=events(didx).time_s;disarmed=NaN;
for rawIndex=1:numel(r.transport_evidence.raw_mavlink)
    a=r.transport_evidence.raw_mavlink{rawIndex};
    if strcmp(a.topic,'HEARTBEAT')&&a.rx_s>=request&&bitand(uint8(a.message.Payload.base_mode),128)==0
        disarmed=a.rx_s;break
    end
end
assert(isfinite(disarmed),'replay:Disarm','Missing post-request disarmed heartbeat.');
offset=r.transport_evidence.coptersim_start_utc_s-r.transport_evidence.utc_zero_s;
selected=[];retained=[];initialIndex=0;initialRx=NaN;
for rawIndex=1:numel(r.transport_evidence.raw_truth_datagrams)
    a=r.transport_evidence.raw_truth_datagrams{rawIndex};
    if ~ismember(numel(a.bytes),[168 200]),continue;end
    q=m600check.decodeTruthPacket(uint8(a.bytes),struct('expected_copter_id',1,'expected_vehicle_type',5));
    if ~q.valid||~q.quaternion_present||~q.angular_rate_present,continue;end
    host=q.time_s+offset;
    if host<disarmed,continue;end
    if isempty(selected),selected=q;initialIndex=rawIndex;initialRx=a.rx_s;end
    if host>selected.time_s+offset+8.05,break;end
    retained(end+1,:)=[host,q.position_ned_m,q.velocity_ned_mps,q.quaternion_wxyz,q.angular_rate_body_radps,q.euler_rad]; %#ok<AGROW>
end
assert(~isempty(selected)&&size(retained,1)>350,'replay:RawWindow','Expected complete retained post-disarm truth.');
initial=[selected.position_ned_m(:).*[1;1;-1];selected.velocity_ned_mps(:).*[1;1;-1]; ...
    selected.quaternion_wxyz(:).*[1;-1;-1;1];selected.angular_rate_body_radps(:).*[-1;-1;1];zeros(6,1)];
initialQuaternionNorm=norm(initial(7:10));initial(7:10)=initial(7:10)/initialQuaternionNorm;
reference=[initial(1:3);zeros(9,1)];wind=zeros(2,1);startSource=selected.time_s;
clear r
specs=struct('name',{'ORIGINAL_NO_ADDED_DISSIPATION','PASSIVE_MU_040','SENSITIVITY_MU_020','SENSITIVITY_MU_080'}, ...
    'enabled',{false,true,true,true},'mu',{0,.4,.2,.8});
dtValues=[.01 .005 .0025];
sources=struct([]);paths=[string(which('m600check.derivativeSoftware'));string(which('m600check.contactKernel')); ...
    string(which('m600check.passiveGroundDissipation'));string(which('gpenmpcM600SixDofPlantDerivative')); ...
    string(which('gpenmpcM600Allocation'));string(which('gpenmpcM600StructuredResidual')); ...
    string(which('gpenmpcQuaternionRotation'));string(which('gpenmpcQuaternionDerivativeMatrix')); ...
    string(which('m600check.decodeTruthPacket'));model;string(mfilename('fullpath'))+".m"];
for pathIndex=1:numel(paths),a=identity(paths(pathIndex));if isempty(sources),sources=a;else,sources(end+1)=a;end;end %#ok<AGROW>
prospective=struct('schema','HOST_PASSIVE_GROUND_REPLAY_PROSPECTIVE_V1','input',identity(input), ...
    'selected_truth_raw_index',initialIndex,'selected_truth_RX_s',initialRx,'selected_truth',selected, ...
    'first_disarmed_HB_RX_s',disarmed,'model_source_to_host_offset_s',offset, ...
    'initial_19state_up',initial,'initial_wire_quaternion_norm',initialQuaternionNorm, ...
    'internal_thrust_states','ZERO COUNTERFACTUAL: six actual internal thrust states not published in retained truth', ...
    'initial_quaternion_processing','Normalize the quaternion to unit norm using the endpoint rule.', ...
    'candidate_mu',.4,'predeclared_sensitivity_mu',[.2 .8],'sensitivity_not_selection',true, ...
    'contact_candidate_provenance','Engineering model: mu is assumed; r_eff is the gyration radius.', ...
    'duration_s',8,'dt_convergence_only_s',dtValues,'mass_kg',mass,'payload_kg',payload,'inertia_kg_m2',J, ...
    'reference_jet_up',reference,'wind_xy_mps',wind,'cases',specs,'sources',sources, ...
    'no_static_support_geometry',true, ...
    'hardware_actions',0,'COM_UDP_actions',0,'MEX_DLL_builds',0,'controller_runs',0,'HIL_result',false);
mkdir(outputDir);writeJson(fullfile(outputDir,'PROSPECTIVE.json'),prospective);

% Actual-free-flight null test of the wrapper, without changing the model.
freeChecks=struct('name',{},'passed',{});
for freeIndex=1:8
    free=initial;free(3)=1+freeIndex/10;free(4:6)=[.1;-.2;.3]*freeIndex;
    free(11:13)=[.03;-.02;.01]*freeIndex;
    [base,~,ct]=m600check.derivativeSoftware(free,zeros(6,1),reference,payload,wind,startSource,p);
    [augmented,~,~]=rhs(free,startSource,true,.4,p,mass,J,payload,reference,wind);
    freeChecks(end+1)=struct('name',sprintf('free_flight_exact_%d',freeIndex), ...
        'passed',ct.contact_force_n==0&&isequal(base,augmented)); %#ok<AGROW>
end
allData=cell(numel(specs),numel(dtValues));results=struct([]);
for caseIndex=1:numel(specs)
    for dtIndex=1:numel(dtValues)
        dt=dtValues(dtIndex);[data,outcome]=integrate(initial,dt,specs(caseIndex),p,mass,J,payload,reference,wind,startSource);
        data.retained_truth_comparison=compareRetained(data,retained,selected.time_s+offset);
        outcome.retained_truth_comparison=data.retained_truth_comparison;
        allData{caseIndex,dtIndex}=data;
        if isempty(results),results=outcome;else,results(end+1)=outcome;end %#ok<AGROW>
        writetable(data.trace,fullfile(outputDir,sprintf('%s_DT_%g.csv',specs(caseIndex).name,dt)));
    end
end
convergence=struct([]);
for caseIndex=1:numel(specs)
    coarse=allData{caseIndex,1};medium=allData{caseIndex,2};fine=allData{caseIndex,3};
    a=struct('name',specs(caseIndex).name, ...
        'final_state_absdiff_010_vs_0025',abs(coarse.state(end,:)-fine.state(end,:)), ...
        'final_state_absdiff_005_vs_0025',abs(medium.state(end,:)-fine.state(end,:)), ...
        'passive_work_absdiff_010_vs_0025_J',abs(coarse.passiveWork(end)-fine.passiveWork(end)), ...
        'passive_work_absdiff_005_vs_0025_J',abs(medium.passiveWork(end)-fine.passiveWork(end)));
    if isempty(convergence),convergence=a;else,convergence(end+1)=a;end %#ok<AGROW>
end
report=struct('schema','HOST_PASSIVE_GROUND_DISSIPATION_REPLAY_V1','prospective',identity(fullfile(outputDir,'PROSPECTIVE.json')), ...
    'results',results,'convergence',convergence,'free_flight_checks',freeChecks, ...
    'all_passivity_checks',all([results.power_nonpositive])&&all([results.combined_cap_respected]), ...
    'all_free_flight_exact',all([freeChecks.passed]),'full_duration_cases',nnz([results.completed_8s]), ...
    'case_denominator',numel(results),'hardware_actions',0,'COM_UDP_actions',0,'HIL_result',false, ...
    'limitation','Passive damping can stop motion at a nonzero attitude. No static landing-gear restoring geometry, PX4 detector, 8s ground delivery or restart is demonstrated.');
save(fullfile(outputDir,'RAW_HOST_REPLAY.mat'),'allData','retained','prospective','report','-v7.3');
writeJson(fullfile(outputDir,'RESULT.json'),report);
disp(jsonencode(struct('full_duration_cases',report.full_duration_cases,'case_denominator',report.case_denominator, ...
    'all_passivity_checks',report.all_passivity_checks,'all_free_flight_exact',report.all_free_flight_exact)));
end
function [dx,contact,e]=rhs(x,t,enabled,mu,p,mass,J,payload,reference,wind)
[dx,~,contact]=m600check.derivativeSoftware(x,zeros(6,1),reference,payload,wind,t,p);
[force,torque,e]=m600check.passiveGroundDissipation(x(4:6),x(11:13),mass,J,contact.contact_force_n,mu,p.contact);
if enabled
    dx(4:6)=dx(4:6)+force/mass;dx(11:13)=dx(11:13)+J\torque;
else
    e.power_W=0;e.scale=0;e.combined_actual_N=0;force(:)=0;torque(:)=0;
end
e.force_world_N=force;e.torque_body_Nm=torque;
end
function [data,outcome]=integrate(initial,dt,spec,p,mass,J,payload,reference,wind,startSource)
count=round(8/dt);state=zeros(count+1,19);derivative=zeros(count+1,19);wrench=zeros(count+1,6);
normal=zeros(count+1,1);power=normal;scale=normal;cone=normal;cap=normal;kineticXY=normal;kineticRot=normal;
time=(0:count)'*dt;state(1,:)=initial.';failure='';last=count+1;
for step=1:count+1
    x=state(step,:).';t=startSource+time(step);
    [a,contact,e]=rhs(x,t,spec.enabled,spec.mu,p,mass,J,payload,reference,wind);
    derivative(step,:)=a.';wrench(step,:)=[e.force_world_N;e.torque_body_Nm].';
    normal(step)=contact.contact_force_n;power(step)=e.power_W;scale(step)=e.scale;
    cone(step)=e.combined_actual_N;cap(step)=e.friction_cap_N;
    kineticXY(step)=.5*mass*sum(x(4:5).^2);kineticRot(step)=.5*x(11:13).'*J*x(11:13);
    if step==count+1,break;end
    b=rhs(x+dt*a/2,t+dt/2,spec.enabled,spec.mu,p,mass,J,payload,reference,wind);
    c=rhs(x+dt*b/2,t+dt/2,spec.enabled,spec.mu,p,mass,J,payload,reference,wind);
    d=rhs(x+dt*c,t+dt,spec.enabled,spec.mu,p,mass,J,payload,reference,wind);
    next=x+dt*(a+2*b+2*c+d)/6;
    if any(~isfinite(next))||norm(next(7:10))<1e-15
        failure='NONFINITE_OR_INVALID_QUATERNION';last=step;break
    end
    next(7:10)=next(7:10)/norm(next(7:10)); % original RK4 numerical unit normalization only
    state(step+1,:)=next.';
end
time=time(1:last);state=state(1:last,:);derivative=derivative(1:last,:);wrench=wrench(1:last,:);
normal=normal(1:last);power=power(1:last);scale=scale(1:last);cone=cone(1:last);cap=cap(1:last);
kineticXY=kineticXY(1:last);kineticRot=kineticRot(1:last);passiveWork=cumtrapz(time,power);
eulerNed=zeros(last,3);
for step=1:last
    q=state(step,7:10).'.*[1;-1;-1;1];R=gpenmpcQuaternionRotation(q);
    eulerNed(step,:)=[atan2(R(3,2),R(3,3)),asin(min(max(R(3,1)*-1,-1),1)),atan2(R(2,1),R(1,1))]*180/pi;
end
columns=["time_s",compose("state_%02d",1:19),compose("derivative_%02d",1:19), ...
    "force_world_x_N","force_world_y_N","force_world_z_N","torque_body_x_Nm","torque_body_y_Nm","torque_body_z_Nm", ...
    "normal_contact_N","dissipation_power_W","cumulative_passive_work_J","common_scale", ...
    "combined_friction_N","friction_cap_N","kinetic_world_XY_J","kinetic_rotation_J", ...
    "truth_roll_NED_deg","truth_pitch_NED_deg","truth_yaw_NED_deg"];
trace=array2table([time,state,derivative,wrench,normal,power,passiveWork,scale,cone,cap,kineticXY,kineticRot,eulerNed], ...
    'VariableNames',cellstr(columns));
data=struct('time',time,'state',state,'derivative',derivative,'wrench',wrench,'normal',normal, ...
    'power',power,'passiveWork',passiveWork,'kineticXY',kineticXY,'kineticRot',kineticRot,'eulerNed',eulerNed,'trace',trace);
outcome=struct('name',spec.name,'mu',spec.mu,'dt_s',dt,'rows',last,'completed_8s',last==count+1&&isempty(failure), ...
    'failure',failure,'initial_state',state(1,:),'final_state',state(end,:), ...
    'initial_RPY_deg',eulerNed(1,:),'final_RPY_deg',eulerNed(end,:), ...
    'max_abs_RPY_deg',max(abs(eulerNed),[],1),'final_horizontal_speed_mps',norm(state(end,4:5)), ...
    'final_body_rate_deg_s',state(end,11:13)*180/pi,'horizontal_displacement_m',norm(state(end,1:2)-state(1,1:2)), ...
    'kinetic_XY_initial_final_J',kineticXY([1 end]).','kinetic_rotation_initial_final_J',kineticRot([1 end]).', ...
    'passive_work_J',passiveWork(end),'maximum_dissipation_power_W',max(power), ...
    'normal_force_range_N',[min(normal),max(normal)],'normal_force_delta_by_candidate_N',0, ...
    'power_nonpositive',all(power<=1e-12),'combined_cap_respected',all(cone<=cap+1e-11), ...
    'added_vertical_force_exact_zero',all(wrench(:,3)==0),'quaternion_norm_error',max(abs(vecnorm(state(:,7:10),2,2)-1)), ...
    'state_clamps',0,'attitude_reset',false,'quaternion_unit_normalization_each_step',true, ...
    'static_support_geometry_proved',false,'stationary_delivery_8s_proved',false);
end
function out=compareRetained(data,retained,startHost)
time=retained(:,1)-startHost;mask=time>=0&time<=data.time(end);time=time(mask);actual=retained(mask,:);
predicted=interp1(data.time,data.state,time,'linear');nedP=predicted(:,1:3).*[1 1 -1];nedV=predicted(:,4:6).*[1 1 -1];
rates=predicted(:,11:13).*[-1 -1 1];
out=struct('rows',numel(time),'comparison','Diagnostic; zero internal rotor-thrust counterfactual, not bit-exact original replay', ...
    'position_max_gap_m',max(vecnorm(nedP-actual(:,2:4),2,2)), ...
    'velocity_max_gap_mps',max(vecnorm(nedV-actual(:,5:7),2,2)), ...
    'rate_max_gap_rad_s',max(vecnorm(rates-actual(:,12:14),2,2)));
end
function id=identity(p)
item=dir(p);assert(numel(item)==1,'replay:Identity','Source missing or ambiguous.');
id=struct('path',char(p),'bytes',item.bytes,'sha256',m600check.fileSha256(p));
end
function writeJson(p,r)
fid=fopen(p,'w','n','UTF-8');assert(fid>=0,'replay:File','Cannot write new result.');c=onCleanup(@()fclose(fid));
fprintf(fid,'%s\n',jsonencode(r,PrettyPrint=true));clear c
end
