function report=test_canonical_passive_ground_integration(outputDir)
% Test passive-ground integration through the production functions.
arguments,outputDir (1,1) string,end
assert(~isfolder(outputDir)&&~isfile(outputDir),'test:OutputExists','No overwrite.');
build=string(fileparts(fileparts(mfilename('fullpath'))));previousPath=path;
addpath(fullfile(build,'matlab_validation'),fullfile(build,'m600_coptersim','matlab_validation'));
cleanup=onCleanup(@()finish(previousPath)); %#ok<NASGU>
m600check.loadFixture();
model=fullfile(gpenmpc_external_path('flat_terrain_model'),'M600_CORE_PARAMETERS.mat');
assert(strcmpi(m600check.fileSha256(model),'969D347CE041715CAED49BE0B7897486E34FA4E0B22EB82E50E9752B8F4F19E4'), ...
    'test:ModelIdentity','Existing neutral model changed.');
loaded=load(model,'parameters');old=loaded.parameters;
disabled=old;disabled.ground_dissipation=struct('enabled',false,'coefficient_of_friction',.4);
enabled=old;enabled.ground_dissipation=struct('enabled',true,'coefficient_of_friction',.4);
priorPath=fullfile(gpenmpc_external_path('host_passive_ground_replay'),'PROSPECTIVE.json');
assert(strcmpi(m600check.fileSha256(priorPath),'46B57F6BE6AA0AEF84D2E50E8C28F6A21F3943388FAE3B5B88495AA4CE6DCEDD'), ...
    'test:InitialIdentity','Retained post-disarm initial state binding changed.');
prior=jsondecode(fileread(priorPath));initial=prior.initial_19state_up(:);
payload=prior.payload_kg;mass=prior.mass_kg;J=prior.inertia_kg_m2;
jet=[initial(1:3);zeros(9,1)];wind=zeros(2,1);zero=zeros(6,1);
rows=struct('name',{},'passed',{},'detail',{});
sourceBindings=struct([]);paths=[string(which('m600check.derivativeSoftware'));string(which('m600check.copterSimIoCore')); ...
    string(which('m600check.passiveGroundDissipation'));string(which('m600check.stepPx4Rk4')); ...
    string(which('m600check.copterSimOutputs'));string(which('gpenmpcM600SixDofPlantDerivative')); ...
    string(which('m600check.contactKernel'));model;priorPath;string(mfilename('fullpath'))+".m"];
for sourceIndex=1:numel(paths),a=identity(paths(sourceIndex));if isempty(sourceBindings),sourceBindings=a;else,sourceBindings(end+1)=a;end;end %#ok<AGROW>

for scenario=1:12
    x=initial;x(4:6)=initial(4:6)*(scenario-6);x(11:13)=initial(11:13)*(scenario-6);
    if mod(scenario,2)==0,x(3)=1;end
    [base,bd,bc]=m600check.derivativeSoftware(x,zero,jet,payload,wind,scenario*.01,old);
    [off,od,oc]=m600check.derivativeSoftware(x,zero,jet,payload,wind,scenario*.01,disabled);
    record(sprintf('legacy_disabled_bitwise_%02d',scenario),bitsEqual(base,off)&&bitsEqual(bd,od)&&bitsEqual(bc,oc), ...
        'Derivative, diagnostic and contact values for the default branch.');
    [on,nd,nc]=m600check.derivativeSoftware(x,zero,jet,payload,wind,scenario*.01,enabled);
    if bc.contact_force_n==0
        record(sprintf('no_contact_enabled_bitwise_%02d',scenario),bitsEqual(base,on)&&bitsEqual(bd,nd)&&bitsEqual(bc,nc), ...
            'No +0 arithmetic or free-flight change.');
    else
        [F,tau,e]=m600check.passiveGroundDissipation(x(4:6),x(11:13),mass,J,bc.contact_force_n,.4,old.contact);
        expected=base;expected(4:6)=expected(4:6)+F/mass;expected(11:13)=expected(11:13)+J\tau;
        expectedAccel=bd.actual_acceleration_mps2+F/mass;
        record(sprintf('contact_full_derivative_%02d',scenario),bitsEqual(on,expected)&&bitsEqual(nd.actual_acceleration_mps2,expectedAccel), ...
            'F/m enters world acceleration; J\tau enters body angular acceleration.');
        record(sprintf('rotor_wrench_not_forged_%02d',scenario),bitsEqual(bd.true_wrench,nd.true_wrench)&&bitsEqual(bc,nc), ...
            'Rotor true_wrench and normal-contact diagnostics.');
        record(sprintf('contact_passive_%02d',scenario),e.power_W<=0&&F(3)==0&&e.combined_actual_N<=e.friction_cap_N+1e-12, ...
            'Unilateral passive force/torque with the original normal force.');
    end
end

env=struct('reference_jet_ned',[initial(1:3).*[1;1;-1];zeros(9,1)],'payload_kg',payload,'wind_xy_mps',wind);
initialPosition=initial(1:3).*[1;1;-1];initialEuler=prior.selected_truth.euler_rad(:);
% Reset the core through p/Euler; velocity, angular rate and thrust start at zero.
% Use the five-input prefix as a unit fixture.
prefix=zeros(16,5);prefix(1:6,:)=.03;prefix(1,:)=.04;
[legacyData,legacyChecks]=coreCase(old,initialPosition,initialEuler,env,prefix,800);
[disabledData,disabledChecks]=coreCase(disabled,initialPosition,initialEuler,env,prefix,800);
record('legacy_disabled_entire_core_bitwise',bitsEqual(legacyData,disabledData), ...
    'Identical reset, five-step input prefix, all 800 zero-command tail outputs and diagnostics.');
record('legacy_full_duration',all(struct2array(legacyChecks))&&all(struct2array(disabledChecks)), ...
    'The continuous core runs cover 8 s of ground dynamics.');
[enabledData,enabledChecks]=coreCase(enabled,initialPosition,initialEuler,env,prefix,800);
fields=fieldnames(enabledChecks);
for fieldIndex=1:numel(fields),record(['enabled_core_' fields{fieldIndex}],enabledChecks.(fields{fieldIndex}),'Actual persistent production core.');end
[zeroData,zeroChecks]=coreCase(enabled,initialPosition,initialEuler,env,zeros(16,0),800);
fields=fieldnames(zeroChecks);
for fieldIndex=1:numel(fields),record(['zero_prefix_core_' fields{fieldIndex}],zeroChecks.(fields{fieldIndex}),'Reset then 800 entirely zero-command steps.');end
record('prefix_actually_excites_body_rate',norm(enabledData.state(1,11:13))>0, ...
    'A five-step HOST input prefix generates residual rotation.');
record('zero_prefix_no_artificial_rotation',all(zeroData.state(:,11:13)==0,'all'), ...
    'No torque added at zero angular velocity.');

freePosition=[0;0;-2];freeEnv=env;freeEnv.reference_jet_ned=[freePosition;zeros(9,1)];
[freeOld,~]=coreCase(old,freePosition,[.03;-.02;.1],freeEnv,zeros(16,0),10);
[freeNew,~]=coreCase(enabled,freePosition,[.03;-.02;.1],freeEnv,zeros(16,0),10);
record('airborne_core_entire_bitwise',bitsEqual(freeOld,freeNew)&&all(freeNew.contact==0), ...
    'Airborne 100ms actual core has exact original output bits.');

report=struct('schema','HOST_CANONICAL_PASSIVE_GROUND_INTEGRATION_V1','passed',all([rows.passed]), ...
    'checks',numel(rows),'passed_checks',nnz([rows.passed]),'rows',rows,'sources',sourceBindings, ...
    'configuration_change',enabled.ground_dissipation,'core_prefix_controls',prefix,'zero_tail_steps',800,'core_dt_s',.01, ...
    'initial_state_scope','Core uses its p/Euler reset then v/omega/thrust0. Derivative checks separately use exact 005 p/v/q/omega.', ...
    'true_wrench_semantics','Rotor wrench only; contact torque is separate and contributes to published complete angular acceleration.', ...
    'final_legacy_state',legacyData.state(end,:),'final_candidate_state',enabledData.state(end,:), ...
    'hardware_actions',0,'COM_UDP_actions',0,'DLL_builds',0,'controller_runs',0,'live_admission',false, ...
    'stationary_delivery_proved',false);
mkdir(outputDir);save(fullfile(outputDir,'RAW_CORE_TEST.mat'),'legacyData','disabledData','enabledData','zeroData','freeOld','freeNew','report','-v7.3');
writetable(enabledData.trace,fullfile(outputDir,'CANDIDATE_ZERO_INPUT_8S.csv'));
writetable(legacyData.trace,fullfile(outputDir,'LEGACY_ZERO_INPUT_8S.csv'));
fid=fopen(fullfile(outputDir,'RESULT.json'),'w','n','UTF-8');assert(fid>=0,'test:File','Cannot write new result.');closer=onCleanup(@()fclose(fid));
fprintf(fid,'%s\n',jsonencode(report,PrettyPrint=true));clear closer
disp(jsonencode(struct('passed',report.passed,'checks',report.checks,'passed_checks',report.passed_checks)));
assert(report.passed,'test:IntegrationFailed','Production optional contact integration test failed.');
    function record(name,pass,detail)
        rows(end+1)=struct('name',name,'passed',logical(pass),'detail',detail);
    end
end
function [data,checks]=coreCase(p,position,euler,env,prefix,steps)
clear m600check.copterSimIoCore
zero=zeros(16,1);[first,firstD]=m600check.copterSimIoCore(zero,true,position,euler,env,p);
prefixY=cell(1,size(prefix,2));prefixD=cell(1,size(prefix,2));
for k=1:size(prefix,2)
    [prefixY{k},prefixD{k}]=m600check.copterSimIoCore(prefix(:,k),false,position,euler,env,p);
end
if isempty(prefixY),lastY=first;lastD=firstD;else,lastY=prefixY{end};lastD=prefixD{end};end
baseTime=lastD.sim_time_s;Y=cell(steps+1,1);D=cell(steps+1,1);Y{1}=lastY;D{1}=lastD;
state=zeros(steps+1,19);state(1,:)=lastD.state_up.';time=zeros(steps+1,1);time(1)=baseTime;
contact=zeros(steps+1,1);contact(1)=lastD.contact_force_n;mass=zeros(steps+1,1);mass(1)=lastY.mass_kg;
angular=zeros(steps+1,3);angular(1,:)=lastY.angular_acceleration_body_frd_rad_s2.';
expectedAngular=nan(steps+1,3);maxErrorAngular=0;maxErrorAccel=0;wrenchSame=true;stepIdentity=true;
resetCount=double(firstD.reset_applied);prefixAccepted=true;
for k=1:numel(prefixD),prefixAccepted=prefixAccepted&&prefixD{k}.step_accepted&&~prefixD{k}.failed;resetCount=resetCount+double(prefixD{k}.reset_applied);end
jet=env.reference_jet_ned;for axis=0:3,jet(3*axis+(1:3))=jet(3*axis+(1:3)).*[1;1;-1];end
memory=m600check.initialStepMemory();
% Recreate the real observer memory through the same prefix for per-step
% reference integration. No private persistent-state mutation is used.
[expectedState,memory]=m600check.initialStateFromNed(position,euler,env.payload_kg,p);
expectedTime=0;
for k=1:size(prefix,2)
    [expectedState,~,~,memory,status]=m600check.stepPx4Rk4(expectedState,prefix(:,k),jet,env.payload_kg,env.wind_xy_mps,expectedTime,p,memory);
    expectedTime=status.next_time_s;
end
for k=1:steps
    [y,d]=m600check.copterSimIoCore(zero,false,position,euler,env,p);Y{k+1}=y;D{k+1}=d;
    resetCount=resetCount+double(d.reset_applied);state(k+1,:)=d.state_up.';time(k+1)=d.sim_time_s;
    contact(k+1)=d.contact_force_n;mass(k+1)=y.mass_kg;angular(k+1,:)=y.angular_acceleration_body_frd_rad_s2.';
    [expectedState,~,~,memory,status]=m600check.stepPx4Rk4(expectedState,zero,jet,env.payload_kg,env.wind_xy_mps,expectedTime,p,memory);
    expectedTime=status.next_time_s;
    stepIdentity=stepIdentity&&status.accepted&&bitsEqual(expectedState,d.state_up);
    [dx,endpoint]=m600check.derivativeSoftware(d.state_up,zeros(6,1),jet,env.payload_kg,env.wind_xy_mps,d.sim_time_s,p);
    expectedAngular(k+1,:)=(dx(11:13).*[-1;-1;1]).';
    maxErrorAngular=max(maxErrorAngular,max(abs(angular(k+1,:)-expectedAngular(k+1,:))));
    maxErrorAccel=max(maxErrorAccel,max(abs(y.acceleration_ned_mps2-endpoint.actual_acceleration_mps2.*[1;1;-1])));
    disabled=p;if isfield(disabled,'ground_dissipation'),disabled.ground_dissipation.enabled=false;end
    [~,rotorOnly]=m600check.derivativeSoftware(d.state_up,zeros(6,1),jet,env.payload_kg,env.wind_xy_mps,d.sim_time_s,disabled);
    wrenchSame=wrenchSame&&bitsEqual(endpoint.true_wrench,rotorOnly.true_wrench);
end
checks=struct('full_duration',abs(time(end)-baseTime-steps*.01)<1e-9, ...
    'monotonic_fixed_time',all(abs(diff(time)-.01)<1e-12), ...
    'single_initial_reset_only',resetCount==1, ...
    'all_steps_accepted',prefixAccepted&&all(cellfun(@(q)q.step_accepted&&~q.failed,D(2:end))), ...
    'same_actual_step_state_bits',stepIdentity, ...
    'state_finite',all(isfinite(state),'all'), ...
    'mass_exact_constant',all(mass==first.mass_kg), ...
    'full_endpoint_angular_acceleration',maxErrorAngular<1e-11, ...
    'full_endpoint_linear_acceleration',maxErrorAccel<1e-11, ...
    'rotor_true_wrench_unchanged',wrenchSame, ...
    'thrust_states_nonnegative',all(state(:,14:19)>=0,'all'));
columns=["time_s",compose("state_%02d",1:19),"contact_force_N","mass_kg", ...
    "published_alpha_FRD_x","published_alpha_FRD_y","published_alpha_FRD_z", ...
    "expected_alpha_FRD_x","expected_alpha_FRD_y","expected_alpha_FRD_z",compose("input_%02d",1:16)];
trace=array2table([time,state,contact,mass,angular,expectedAngular,zeros(steps+1,16)],'VariableNames',cellstr(columns));
data=struct('initial_y',first,'initial_diagnostic',firstD,'prefix_y',{prefixY},'prefix_diagnostic',{prefixD}, ...
    'y',{Y},'diagnostic',{D},'state',state,'time',time,'contact',contact,'mass',mass, ...
    'angular',angular,'expected_angular',expectedAngular,'max_angular_error',maxErrorAngular, ...
    'max_acceleration_error',maxErrorAccel,'zero_tail_controls',zeros(16,steps),'trace',trace);
clear m600check.copterSimIoCore
end
function ok=bitsEqual(a,b)
if ~strcmp(class(a),class(b))||~isequal(size(a),size(b)),ok=false;return;end
if isstruct(a)
    keys=fieldnames(a);if ~isequal(keys,fieldnames(b)),ok=false;return;end
    ok=true;for i=1:numel(a),for k=1:numel(keys),if ~bitsEqual(a(i).(keys{k}),b(i).(keys{k})),ok=false;return;end;end;end
elseif iscell(a)
    ok=true;for i=1:numel(a),if ~bitsEqual(a{i},b{i}),ok=false;return;end;end
elseif isnumeric(a)
    ok=isequal(typecast(a(:),'uint8'),typecast(b(:),'uint8'));
elseif istable(a)
    ok=isequal(a.Properties,b.Properties)&&bitsEqual(table2array(a),table2array(b));
else
    ok=isequaln(a,b);
end
end
function id=identity(p)
item=dir(p);assert(numel(item)==1,'test:Identity','Missing/ambiguous source.');
id=struct('path',char(p),'bytes',item.bytes,'sha256',m600check.fileSha256(p));
end
function finish(previousPath)
clear m600check.copterSimIoCore
path(previousPath);
end
