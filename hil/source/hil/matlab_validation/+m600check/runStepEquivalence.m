function report = runStepEquivalence(workRoot,taskPath)
%RUNSTEPEQUIVALENCE Independent MATLAB oracle for the exact 0.010-s update.
% No MATLAB model, simulator, process, COM, controller or hardware is started.
% Prescribed actuator fixtures test numerical update equivalence.
if nargin==0
    f=m600check.loadFixture();
elseif nargin==1
    f=m600check.loadFixture(workRoot);
else
    f=m600check.loadFixture(workRoot,taskPath);
end
extrapolator=which('gpenmpcReferenceExtrapolation');
assert(strcmpi(gpenmpcNative.fileSha256(extrapolator), ...
    '30BAB334E997921387BE45669842F8580C3CEA197C406E05E9FDFE943F098BAF'));
p=f.parameters;dt=0.01;
upper=p.calibration.rotor_allocation.per_rotor_thrust_upper_n;
mass=p.profile.mass_properties.base_mass_kg+f.initial_payload_kg ...
    +p.mission.plant_mismatch.mass_bias_kg;
hoverFraction=mass*9.80665/(6*upper);
assert(hoverFraction>0 &&1.35*hoverFraction<1);
caseNames={'ground_rest','prescribed_takeoff','descending_contact', ...
    'airborne_reference_extrapolation','rotor_clamp','overtravel_stop'};
caseSteps=[60,120,100,50,1,1];
rows=struct('case_name',{},'step',{},'accepted',{},'maximum_absolute_difference',{});
cases=struct('name',{},'planned_steps',{},'compared_steps',{}, ...
    'accepted_steps',{},'failure_code',{},'liftoff_observed',{}, ...
    'contact_entries',{},'ground_confirmed',{},'final_state_up',{});
for c=1:numel(caseNames)
    x=zeros(19,1);x(7)=1;
    if c==3
        x(3)=0.045;x(6)=-0.08;
    elseif c==4
        x(1:6)=[1;2;4;.4;-.3;.2];x(7:10)=[1.4;.1;-.05;.2];
        x(11:13)=[.05;-.02;.01];x(14:19)=mass*9.80665/6;
    elseif c==5
        x(3)=2;x(14:19)=[-1;upper+1;-2;upper+2;0;upper];
    elseif c==6
        x(3)=-.08;x(6)=-.2;
    end
    actual=x;expected=x;
    memory=m600check.initialStepMemory();oracleMemory=independentMemory();
    % Above-ground fixtures start with zero prior ground-support dwell.
    if c==3 ||c==4 ||c==5
        memory.ground_support_dwell_s=0;memory.ground_confirmed=false;
        oracleMemory.ground_support_dwell_s=0;oracleMemory.ground_confirmed=false;
    end
    acceptedCount=0;compared=0;lastCode=uint8(0);
    for k=1:caseSteps(c)
        t=(k-1)*dt;
        controls=nan(16,1);controls(1:6)=0;
        jet=zeros(12,1);wind=zeros(2,1);
        if c==2
            controls(1:6)=1.35*hoverFraction;
        elseif c==4
            controls(1:6)=hoverFraction*(.98+.015*sin(k+(1:6).'));
            jet=[1;2;4;.4;-.3;.2;.12;-.03;.06;.02;-.015;.01];
            wind=[.4;-.2];
        elseif c==5
            controls(1:6)=0.2;
        end
        beforeMemory=memory;beforeState=actual;
        [next,d,contact,mn,s]=m600check.stepPx4Rk4(actual,controls,jet, ...
            f.initial_payload_kg,wind,t,p,memory);
        [oracle,od,oc,om,os]=oracleStep(expected,controls,jet, ...
            f.initial_payload_kg,wind,t,f,oracleMemory);
        err=closeNumeric(next,oracle);
        err=max(err,compareFields(d,od));err=max(err,compareFields(contact,oc));
        err=max(err,compareFields(mn,om));
        err=max(err,closeNumeric(s.candidate_state,os.candidate_state));
        assert(s.accepted==os.accepted &&s.failure_code==os.failure_code);
        assert(s.step_dt_s==0.01);
        assert(s.contact_overtravel==os.contact_overtravel);
        assert(s.next_time_s==os.next_time_s);
        rows(end+1)=struct('case_name',caseNames{c},'step',k, ...
            'accepted',s.accepted,'maximum_absolute_difference',err); %#ok<AGROW>
        compared=compared+1;lastCode=s.failure_code;
        % Independent NED round trip on each first and last/failed sample.
        if k==1 ||k==caseSteps(c) ||~s.accepted
            [xn,jn]=toNed(actual,jet);
            [nn,nd,nc,nm,ns]=m600check.stepPx4Rk4Ned(xn,controls,jn, ...
                f.initial_payload_kg,wind,t,p,beforeMemory);
            [back,~]=toNed(nn,zeros(12,1));
            closeNumeric(back,next);compareFields(nd,d);compareFields(nc,contact);
            compareFields(nm,mn);compareFields(ns,s);
        end
        actual=next;expected=oracle;memory=mn;oracleMemory=om;
        if ~s.accepted
            % Unlike a caught exception, a generated caller receives a latch;
            % neither next call nor failure may consume a physical step.
            assert(isequal(next,beforeState));
            [again,~,~,againMemory,againStatus]=m600check.stepPx4Rk4( ...
                next,controls,jet,f.initial_payload_kg,wind,t,p,memory);
            assert(~againStatus.accepted &&againStatus.failure_code==5);
            assert(isequal(again,next) &&againMemory.plant_step_count==memory.plant_step_count);
            break
        end
        acceptedCount=acceptedCount+1;
    end
    cases(end+1)=struct('name',caseNames{c},'planned_steps',caseSteps(c), ...
        'compared_steps',compared,'accepted_steps',acceptedCount, ...
        'failure_code',double(lastCode),'liftoff_observed',memory.airborne_observed, ...
        'contact_entries',double(memory.contact_entry_count), ...
        'ground_confirmed',memory.ground_confirmed,'final_state_up',actual.'); %#ok<AGROW>
end
% Dwell boundary follows the exact accepted contact predicate, no new margin.
x=zeros(19,1);x(7)=1;u=nan(16,1);u(1:6)=0;jet=zeros(12,1);
m=independentMemory();m.ground_support_dwell_s=0;m.ground_confirmed=false;
for k=1:50
    [x,~,~,m,s]=m600check.stepPx4Rk4(x,u,jet, ...
        f.initial_payload_kg,zeros(2,1),(k-1)*dt,p,m);
    assert(s.accepted);
    if k==49
        assert(~m.ground_confirmed);
    end
end
assert(m.ground_confirmed &&m.plant_step_count==50);
% Generated-friendly failure result must not advance a failed input.
x=zeros(19,1);x(7)=1;bad=u;bad(2)=NaN;m=independentMemory();
[same,~,~,failed,flag]=m600check.stepPx4Rk4(x,bad,jet, ...
    f.initial_payload_kg,zeros(2,1),0,p,m);
assert(~flag.accepted &&flag.failure_code==1 &&isequal(same,x));
assert(failed.failed &&failed.plant_step_count==0 &&flag.next_time_s==0);
assert(cases(2).liftoff_observed,'m600check:FixtureCoverage','Takeoff fixture did not exercise liftoff.');
assert(cases(3).contact_entries>0,'m600check:FixtureCoverage','Descent fixture did not exercise contact.');
assert(cases(6).failure_code==3,'m600check:FixtureCoverage','Overtravel fixture did not exercise rejection.');
report=struct('status','PASS_FIXED_10MS_RK4_ORACLE_EQUIVALENCE_ONLY', ...
    'source_manifest_sha256',f.source_manifest_sha256, ...
    'task_fixture_sha256',f.task_sha256,'fixed_dt_s',dt, ...
    'compared_oracle_step_rows',numel(rows), ...
    'extra_ground_dwell_boundary_steps',50,'invalid_input_negative_count',1, ...
    'maximum_absolute_difference',max([rows.maximum_absolute_difference]), ...
    'cases',cases,'steps',rows,'hardware_actions',0,'simulator_runs',0, ...
    'codegen_executed',false, ...
    'claim','Numerical equivalence of the 0.010-s discrete update.');
disp(jsonencode(rmfield(report,'steps'),PrettyPrint=true));
end

function [state,d,contact,mn,s]=oracleStep(start,controls,jet,payload,wind,time,f,m)
dt=.01;
command=controls([5;1;4;6;2;3])*f.calibration.rotor_allocation.per_rotor_thrust_upper_n;
reference=struct('position_m',jet(1:3),'velocity_mps',jet(4:6), ...
    'acceleration_mps2',jet(7:9),'jerk_mps3',jet(10:12));
[k1,~]=oracleDerivative(start,command,reference,payload,wind,time,f);
mid=gpenmpcReferenceExtrapolation(reference,.5*dt);
[k2,~]=oracleDerivative(start+.5*dt*k1,command,mid,payload,wind,time+.5*dt,f);
[k3,~]=oracleDerivative(start+.5*dt*k2,command,mid,payload,wind,time+.5*dt,f);
next=gpenmpcReferenceExtrapolation(reference,dt);
[k4,~]=oracleDerivative(start+dt*k3,command,next,payload,wind,time+dt,f);
candidate=start+dt*(k1+2*k2+2*k3+k4)/6;
candidate(7:10)=candidate(7:10)/max(norm(candidate(7:10)),1e-15);
candidate(14:19)=min(max(candidate(14:19),0),f.calibration.rotor_allocation.per_rotor_thrust_upper_n);
[~,d]=oracleDerivative(candidate,command,next,payload,wind,time+dt,f);
contact=gpenmpcNative.compliantContactState(candidate,d.true_mass_kg,f.parameters.contact);
% Drop only nonnumeric schema to compare with generated numeric contact.
contact=rmfield(contact,'schema');
overtravel=contact.contact_overtravel_m>1e-12;
s=struct('accepted',~overtravel,'failure_code',uint8(0), ...
    'contact_overtravel',overtravel,'candidate_state',candidate,'next_time_s',time);
state=start;mn=m;
if overtravel
    s.failure_code=uint8(3);mn.failed=true;return
end
if contact.contact_active &&~m.previous_contact_active
    mn.contact_entry_count=mn.contact_entry_count+uint64(1);
end
mn.previous_contact_active=contact.contact_active;
mn.airborne_observed=mn.airborne_observed||candidate(3)>.02;
if contact.contact_active &&candidate(3)<=.10 &&abs(candidate(6))<=.15
    mn.ground_support_dwell_s=mn.ground_support_dwell_s+dt;
else
    mn.ground_support_dwell_s=0;
end
mn.ground_confirmed=mn.ground_support_dwell_s+1e-12>=.50;
mn.plant_step_count=mn.plant_step_count+uint64(1);
state=candidate;s.next_time_s=time+dt;
end

function [dx,d]=oracleDerivative(x,command,reference,payload,wind,time,f)
[dx,d]=gpenmpcM600SixDofPlantDerivative(x,command,reference,payload,wind,time, ...
    f.mission,f.calibration,f.profile);
c=gpenmpcNative.compliantContactState(x,d.true_mass_kg,f.parameters.contact);
dx(6)=dx(6)+c.vertical_acceleration_up_mps2;
d.actual_acceleration_mps2(3)=d.actual_acceleration_mps2(3)+c.vertical_acceleration_up_mps2;
end

function memory=independentMemory()
memory=struct('previous_contact_active',false,'contact_entry_count',uint64(0), ...
    'airborne_observed',false,'ground_support_dwell_s',.50, ...
    'ground_confirmed',true,'plant_step_count',uint64(0),'failed',false);
end

function [x,jet]=toNed(x,jet)
x(1:3)=x(1:3).*[1;1;-1];x(4:6)=x(4:6).*[1;1;-1];
x(7:10)=x(7:10).*[1;-1;-1;1];x(11:13)=x(11:13).*[-1;-1;1];
for k=0:3
    jet(3*k+(1:3))=jet(3*k+(1:3)).*[1;1;-1];
end
end

function err=compareFields(actual,expected)
names=fieldnames(actual);err=0;
for k=1:numel(names)
    err=max(err,closeNumeric(double(actual.(names{k})),double(expected.(names{k}))));
end
end

function err=closeNumeric(actual,expected)
assert(isequal(size(actual),size(expected)) &&all(isfinite(actual(:))) &&all(isfinite(expected(:))));
err=max(abs(actual(:)-expected(:)));
assert(err<=5e-11*max(1,max(abs(expected(:)))),'m600check:StepMismatch', ...
    'Discrete arithmetic mismatch %.17g.',err);
end
