function report = runPointwiseEquivalence(workRoot,taskPath)
%RUNPOINTWISEEQUIVALENCE Pointwise arithmetic regression.
% The caller saves the returned report.
if nargin==0
    f=m600check.loadFixture();
elseif nargin==1
    f=m600check.loadFixture(workRoot);
else
    f=m600check.loadFixture(workRoot,taskPath);
end
p=f.parameters;
rows=struct('name',{},'passed',{},'maximum_absolute_difference',{});
worldSign=[1;1;-1];qSign=[1;-1;-1;1];rateSign=[-1;-1;1];
zCases=[-.03,0,.01,.034,.035,.036,.1,4];
% Deterministic inputs exercise both residual enable branches, changing
% payload, finite arbitrary attitudes, wind and world-axis drag cross terms.
for residualMode=0:1
    mission=f.mission;
    mission.structured_residual.enabled=logical(residualMode);
    pp=m600check.packParameters(f.calibration,f.profile,mission);
    for k=1:48
        x=zeros(19,1);
        x(1:3)=[sin(.3*k);cos(.2*k);zCases(1+mod(k-1,numel(zCases)))];
        x(4:6)=[2*sin(.17*k);-1.3*cos(.19*k);.8*sin(.31*k)];
        q=[1;.2*sin(k);.15*cos(.7*k);.4*sin(.4*k)];
        x(7:10)=q/norm(q);
        x(11:13)=[.2*sin(k);.3*cos(k);.1*sin(.6*k)];
        x(14:19)=pp.calibration.rotor_allocation.per_rotor_thrust_upper_n ...
            *(.35+.15*sin(k+(1:6).'));
        command=pp.calibration.rotor_allocation.per_rotor_thrust_upper_n ...
            *(.4+.2*cos(.3*k+(1:6).'));
        jet=[zeros(3,1);2*sin(.1*k);1.5*cos(.2*k);.3; ...
            .2*cos(k);-.1*sin(k);.15*cos(.3*k);zeros(3,1)];
        if mod(k,4)==0
            jet(4:5)=0; % low-speed tangent-from-air branch
        end
        ref=struct('position_m',jet(1:3),'velocity_mps',jet(4:6), ...
            'acceleration_mps2',jet(7:9),'jerk_mps3',jet(10:12));
        payload=f.initial_payload_kg*mod(k,5)/4;
        wind=[.4*sin(.2*k);-.7*cos(.1*k)];
        time=.037*k;
        [oracle,od]=gpenmpcM600SixDofPlantDerivative(x,command,ref,payload, ...
            wind,time,mission,f.calibration,f.profile);
        oc=gpenmpcNative.compliantContactState(x,od.true_mass_kg,pp.contact);
        oracle(6)=oracle(6)+oc.vertical_acceleration_up_mps2;
        od.actual_acceleration_mps2(3)=od.actual_acceleration_mps2(3) ...
            +oc.vertical_acceleration_up_mps2;
        [actual,ad,ac]=m600check.derivativeSoftware(x,command,jet,payload,wind,time,pp);
        err=closeNumeric(actual,oracle);
        err=max(err,compareFields(ad,od));
        err=max(err,compareFields(ac,oc));
        rows(end+1)=row(sprintf('oracle_residual_%d_sample_%02d',residualMode,k),err); %#ok<AGROW>
        controls=nan(16,1);
        controls([5;1;4;6;2;3])=command/pp.calibration.rotor_allocation.per_rotor_thrust_upper_n;
        xn=x;
        xn(1:3)=x(1:3).*worldSign;xn(4:6)=x(4:6).*worldSign;
        xn(7:10)=x(7:10).*qSign;xn(11:13)=x(11:13).*rateSign;
        jn=jet;
        for j=0:3
            jn(3*j+(1:3))=jet(3*j+(1:3)).*worldSign;
        end
        [dn,~,~,mapped]=m600check.derivativeNed(xn,controls,jn,payload,wind,time,pp);
        dn(1:3)=dn(1:3).*worldSign;dn(4:6)=dn(4:6).*worldSign;
        dn(7:10)=dn(7:10).*qSign;dn(11:13)=dn(11:13).*rateSign;
        err=max(closeNumeric(dn,actual),closeNumeric(mapped,command));
        rows(end+1)=row(sprintf('frame_and_actuator_%d_sample_%02d',residualMode,k),err); %#ok<AGROW>
    end
end
x=zeros(19,1);x(7)=1;jet=zeros(12,1);u=nan(16,1);u(1:6)=0;
upper=p.calibration.rotor_allocation.per_rotor_thrust_upper_n;
perm=[5;1;4;6;2;3];
for k=1:6
    uk=u;uk(k)=1;
    [~,~,~,command]=m600check.derivativePx4(x,uk,jet,0,zeros(2,1),0,p);
    expected=zeros(6,1);expected(perm==k)=upper;
    rows(end+1)=row(sprintf('single_PX4_rotor_%d_linear_thrust',k),closeNumeric(command,expected)); %#ok<AGROW>
end
for value=[0,.05,.5,.95,1]
    uk=u;uk(1:6)=value;
    [dx,~,~,command]=m600check.derivativePx4(x,uk,jet,0,zeros(2,1),0,p);
    err=max(closeNumeric(command,ones(6,1)*upper*value), ...
        closeNumeric(dx(14:19),command/.12));
    rows(end+1)=row(sprintf('linear_thrust_no_deadband_%0.2f',value),err); %#ok<AGROW>
end
% Contact, static support, release and overtravel arithmetic are compared
% with the retained software oracle.
for z=[-.03,-.025,0,.034999999,.035,.035000001,.1]
    for vz=[-1,0,1]
        x(3)=z;x(6)=vz;
        a=m600check.contactKernel(x,12.4,p.contact);
        b=gpenmpcNative.compliantContactState(x,12.4,p.contact);
        rows(end+1)=row(sprintf('contact_z_%0.9f_v_%0.1f',z,vz),compareFields(a,b)); %#ok<AGROW>
    end
end
x=zeros(19,1);x(7)=1;
bad=u;bad(1)=NaN;
reject(@()m600check.derivativePx4(x,bad,jet,0,zeros(2,1),0,p));
rows(end+1)=row('reject_active_nan',0);
bad=u;bad(1)=1.0001;
reject(@()m600check.derivativePx4(x,bad,jet,0,zeros(2,1),0,p));
rows(end+1)=row('reject_active_out_of_range',0);
badX=x;badX(7:10)=0;
reject(@()m600check.derivativePx4(badX,u,jet,0,zeros(2,1),0,p));
rows(end+1)=row('reject_zero_quaternion',0);
badJet=jet;badJet(4)=NaN;
reject(@()m600check.derivativePx4(x,u,badJet,0,zeros(2,1),0,p));
rows(end+1)=row('reject_nonfinite_reference',0);
reject(@()m600check.contactKernel(x,-1,p.contact));
rows(end+1)=row('reject_nonpositive_mass',0);
badX=x;badX(2)=NaN;
reject(@()m600check.contactKernel(badX,12.4,p.contact));
rows(end+1)=row('reject_nonfinite_contact_state',0);
report=struct('status','PASS_POINTWISE_ARITHMETIC_EQUIVALENCE_ONLY', ...
    'source_manifest_sha256',f.source_manifest_sha256, ...
    'task_fixture_path',f.task_path,'task_fixture_sha256',f.task_sha256, ...
    'fixture_description',f.fixture_description, ...
    'test_count',numel(rows),'passed_count',nnz([rows.passed]), ...
    'maximum_absolute_difference',max([rows.maximum_absolute_difference]), ...
    'tests',rows,'hardware_actions',0,'model_simulation_runs',0, ...
    'code_generation_executed',false, ...
    'claim','Pointwise MATLAB wrapper, contact and coordinate-frame consistency.');
disp(jsonencode(rmfield(report,'tests'),PrettyPrint=true));
end

function r=row(name,err)
r=struct('name',name,'passed',true,'maximum_absolute_difference',err);
end

function err=compareFields(actual,expected)
names=fieldnames(actual);err=0;
for k=1:numel(names)
    name=names{k};
    err=max(err,closeNumeric(double(actual.(name)),double(expected.(name))));
end
end

function err=closeNumeric(actual,expected)
assert(isequal(size(actual),size(expected)) && all(isfinite(actual(:))) ...
    && all(isfinite(expected(:))),'m600check:NumericShape','Nonfinite or shape mismatch.');
err=max(abs(actual(:)-expected(:)));
% Relative tolerance for floating-point arithmetic equivalence.
assert(err<=5e-12*max(1,max(abs(expected(:)))), ...
    'm600check:NumericMismatch','Oracle arithmetic mismatch: %.17g.',err);
end

function reject(callback)
caught=false;
try
    callback();
catch
    caught=true;
end
assert(caught,'m600check:MissingRejection','Negative input was not rejected.');
end
