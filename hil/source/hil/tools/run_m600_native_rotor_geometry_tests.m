function result=run_m600_native_rotor_geometry_tests(outputRoot)
% Test geometry with files and arrays.
build=fileparts(fileparts(mfilename('fullpath')));
if nargin<1,outputRoot=fullfile(build,'evidence','native_rotor_geometry');end
assert(~isfolder(outputRoot),'m600check:ExistingEvidence','Choose a new output directory.');
audit=audit_m600_native_rotor_geometry();assert(audit.passed,'m600check:ActualAudit','Actual audit failed.');
r=jsondecode(fileread(audit.receipt_path));c=audit.canonical;
rows=struct('name',{},'passed',{},'detail',{});
record('actual_typed_79',audit.passed&&audit.receipt_is_actual_file,'Use the recorded typed parameter receipt.');
roundtrip=audit_m600_native_rotor_geometry(jsondecode(jsonencode(r)));
record('json_roundtrip',roundtrip.passed,'Real receipt schema roundtrip.');
record('observed_minus_30_degree',abs(audit.roll_pitch_rotation_deg+30)<1e-5,'Numerical geometry comparison only.');
record('observed_cross_axis',abs(audit.roll_pitch_cross_axis_map(1,2)-0.5)<1e-6&& ...
    abs(audit.roll_pitch_cross_axis_map(2,1)+0.5)<1e-6,'Cross-axis terms are not zero.');
record('candidate_alignment',audit.candidate_geometry_aligned,'REAL32 target geometry matches the plant.');
record('yaw_and_collective',audit.actual_yaw_sign_matches&&audit.actual_collective_thrust_matches,'Yaw signs and collective direction already agree.');
B=audit.canonical_wrench_per_newton;E=audit.actual_effectiveness_per_ct;N=audit.proposed_effectiveness_per_ct;
record('zero_input',all(B*zeros(6,1)==0)&&all(N*zeros(6,1)==0),'No affine force/torque added.');
record('symmetric_collective',norm(B(1:3,:)*ones(6,1))<1e-12&& ...
    norm(N(1:3,:)*ones(6,1))<1e-7&&B(6,:)*ones(6,1)==-6,'Equal motors have no moment.');
for ch=1:6
    pulse=zeros(6,1);pulse(ch)=1;
    sw=find(c.rotor_order==ch);theta=c.angles_deg(sw);
    oracle=[-c.arm_radius_m*sind(theta);c.arm_radius_m*cosd(theta);c.yaw_arm_m*c.spin_sign(sw);0;0;-1];
    record(sprintf('motor_%d_positive_unit',ch),norm(B*pulse-oracle)<1e-12&& ...
        norm(N*pulse-oracle)<1e-7,'Each physical column checked with an independent cross-product oracle.');
    record(sprintf('motor_%d_negative_differential',ch),norm(B*(-pulse)+oracle)<1e-12, ...
        'Signed algebraic perturbation for the allocator comparison.');
end
% Apply bounded differential axis pulses around 0.5.
for axis=1:3
    direction=pinv(B([1:3,6],:))*[double((1:3).'==axis);0];
    direction=direction*(0.1/max(abs(direction)));
    for signum=[-1,1]
        u=0.5*ones(6,1)+signum*direction;v=B*(u-0.5);
        record(sprintf('axis_%d_sign_%+d',axis,signum),all(u>=0&u<=1)&& ...
            signum*v(axis)>0&&norm(v(setdiff(1:3,axis)))<1e-12&& ...
            norm(N*(u-0.5)-v)<1e-7,'Single-axis torque, candidate geometry, same admissible actuator domain.');
    end
end
u=[0.07;0.23;0.41;0.59;0.71;0.93];
record('linear_superposition',norm(B*u-sum(B.*u.',2))<1e-12&& ...
    norm(N*u-B*u)<1e-7,'Mixed finite control inputs.');
record('newton_scaling',norm(audit.canonical_wrench_per_normalized_control-B*c.thrust_upper_n,'fro')<1e-12, ...
    'Normalized control to Newton mapping is linear once; not PWM microseconds or squared RPM.');
record('ct_not_physical_ceiling',norm(audit.actual_effectiveness_6x6-6.5*E,'fro')<1e-12&& ...
    c.thrust_upper_n~=6.5,'Allocator CT and model force normalization kept distinct.');
delta=audit.proposed_parameter_delta;
record('exact_12_geometry_deltas',numel(delta)==12&&all([delta.changed])&& ...
    all(~cellfun(@isempty,regexp({delta.name},'^CA_ROTOR[0-5]_P[XY]$','once'))),'Only model-position alignment proposed.');
record('exact_rollback',all(strcmp({delta.original_raw_bits_hex},{delta.rollback_raw_bits_hex}))&& ...
    isequal([delta.original_value],[delta.rollback_value]),'Rollback restores original observed REAL32 bits.');
candidate=r;
for k=1:numel(delta),candidate=setv(candidate,delta(k).name,delta(k).candidate_value,9);end
candidateAudit=audit_m600_native_rotor_geometry(candidate);
record('candidate_receipt_recompute',candidateAudit.passed&& ...
    candidateAudit.actual_alignment_max_abs<=1e-7,'Reparse candidate typed values through the same production audit.');
neg={'missing_row','duplicate_row','wrong_type','wrong_bits','nonfinite','wrong_uid','unsafe', ...
    'read_error','write_present','zero_axis','non_up_axis','negative_ct','nonuniform_ct', ...
    'thr_model_nonzero','wrong_allocator','wrong_reverse','bad_order','bad_angle'};
for k=1:numel(neg)
    x=r;cc=c;
    switch neg{k}
        case 'missing_row',x.diagnostic_parameter_observations(1)=[];
        case 'duplicate_row',x.diagnostic_parameter_observations(2)=x.diagnostic_parameter_observations(1);
        case 'wrong_type',j=ix(x,'CA_ROTOR0_PX');x.diagnostic_parameter_observations(j).typed_value.mav_type=6;
        case 'wrong_bits',j=ix(x,'CA_ROTOR0_PX');x.diagnostic_parameter_observations(j).typed_value.raw_bits_hex='3F800000';
        case 'nonfinite',j=ix(x,'CA_ROTOR0_PX');x.diagnostic_parameter_observations(j).typed_value.decoded=NaN;
        case 'wrong_uid',x.uid='0';
        case 'unsafe',x.armed=true;
        case 'read_error',x.diagnostic_parameter_observations(1).read_error='timeout';
        case 'write_present',x.parameter_writes=1;
        case 'zero_axis',x=setv(x,'CA_ROTOR0_AZ',0,9);
        case 'non_up_axis',x=setv(x,'CA_ROTOR0_AX',0.1,9);
        case 'negative_ct',x=setv(x,'CA_ROTOR0_CT',-1,9);
        case 'nonuniform_ct',x=setv(x,'CA_ROTOR0_CT',7,9);
        case 'thr_model_nonzero',x=setv(x,'THR_MDL_FAC',0.3,9);
        case 'wrong_allocator',x=setv(x,'CA_AIRFRAME',1,6);
        case 'wrong_reverse',x=setv(x,'CA_R_REV',1,6);
        case 'bad_order',cc.rotor_order(2)=cc.rotor_order(1);
        case 'bad_angle',cc.angles_deg(1)=NaN;
    end
    a=audit_m600_native_rotor_geometry(x,cc);
    record(['negative_' neg{k}],~a.passed&&~isempty(a.failure)&&a.hardware_actions==0, ...
        'Reject invalid evidence/unsupported contract; do not mask as geometry PASS.');
end
x=setv(r,'CA_ROTOR0_KM',0.025,9);a=audit_m600_native_rotor_geometry(x);
record('wrong_yaw_not_silently_fixed',a.passed&&~a.actual_yaw_sign_matches&&~a.candidate_geometry_aligned, ...
    'Yaw sign consistency requires a separate correction from rotor positions.');
record('no_flight_claim',~audit.flight_admission&&~audit.unique_instability_cause_proven&&audit.hardware_actions==0, ...
    'Geometry validation retains the separate flight and dynamic-stability gates.');
result=struct('schema','HOST_NATIVE_ROTOR_GEOMETRY_TEST_V1','passed',all([rows.passed]), ...
    'case_count',numel(rows),'cases_passed',sum([rows.passed]),'cases',rows,'audit',audit, ...
    'candidate_receipt_audit',candidateAudit,'COM_UDP_board_model_actions',0,'flight_admission',false);
mkdir(outputRoot);save(fullfile(outputRoot,'RESULT.mat'),'result');
f=fopen(fullfile(outputRoot,'RESULT.json'),'w');assert(f>=0);g=onCleanup(@()fclose(f));
fprintf(f,'%s\n',jsonencode(result,PrettyPrint=true));clear g
assert(result.passed,'m600check:GeometryTestsFailed','%d/%d checks passed.',result.cases_passed,result.case_count);
    function record(name,passed,detail)
        rows(end+1)=struct('name',name,'passed',logical(passed),'detail',detail); %#ok<AGROW>
    end
end
function j=ix(r,name),j=find(strcmp({r.diagnostic_parameter_observations.name},name));assert(isscalar(j));end
function r=setv(r,name,value,mavType)
j=ix(r,name);t=r.diagnostic_parameter_observations(j).typed_value;t.mav_type=mavType;
if mavType==9,u=typecast(single(value),'uint32');t.decoded=double(single(value));
else,u=typecast(int32(value),'uint32');t.decoded=double(int32(value));end
t.raw_bits_hex=upper(dec2hex(u,8));t.raw_float=double(typecast(u,'single'));
r.diagnostic_parameter_observations(j).typed_value=t;
end
