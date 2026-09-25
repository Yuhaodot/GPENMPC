function out=analyze_native_hover_from_raw(outputDir)
% Analyze retained host records without constructing a transport or plant.
arguments,outputDir (1,1) string,end
assert(~isfolder(outputDir),'analysis:ExistingOutput','Use a new derived output directory.');
b=fileparts(fileparts(mfilename('fullpath')));addpath(fullfile(b,'m600_coptersim','matlab_validation'));
root=gpenmpc_external_path('native_hover_motion_recording');
input=fullfile(root,'INNER_MATLAB','RAW_MATLAB_HIL.mat');x=load(input,'result');r=x.result;e=r.transport_evidence;
auditFile=fullfile(gpenmpc_external_path('host_native_rotor_geometry'),'RESULT.json');a=jsondecode(fileread(auditFile));
assert(a.passed&&a.audit.passed,'analysis:GeometryAudit','Expected completed HOST geometry audit.');
W=a.audit.canonical_wrench_per_normalized_control;I=diag([1.6,1.6,3]);
offset=mean([e.fixed_best_timesync.offset_lower_s,e.fixed_best_timesync.offset_upper_s]);
truthOffset=e.coptersim_start_utc_s-e.utc_zero_s;
armTime=eventTime('LOGICAL_ARM_REQUEST');formalTime=eventTime('FORMAL_BEGIN');
% Retain model-fault, observer and exception timestamps separately.
assert(isstruct(r.first_model_failure)&&isfield(r.first_model_failure,'diagnostic'), ...
    'analysis:ModelFaultReceipt','Expected the retained 004 model-failure receipt.');
cut=r.first_model_failure.diagnostic.first_fault.received_at_s;
captureTime=NaN;
if isstruct(r.first_failed_state_evidence)&&isfield(r.first_failed_state_evidence,'captured_at_s')
    captureTime=r.first_failed_state_evidence.captured_at_s;
end
exceptionTime=eventTime('EXCEPTION');
att=zeros(0,9);target=zeros(0,9);act=zeros(0,21);hb=zeros(0,7);truth=zeros(0,29);diagRows=zeros(0,9);
texts=struct('raw_index',{},'rx_s',{},'severity',{},'text',{});
badTruth=struct('raw_index',{},'rx_s',{},'bytes',{},'reason',{});
for k=1:numel(e.raw_mavlink)
    v=e.raw_mavlink{k};p=v.message.Payload;
    switch v.topic
        case 'ATTITUDE'
            att(end+1,:)=[k,v.rx_s,double(p.time_boot_ms)/1000+offset, ...
                double([p.roll,p.pitch,p.yaw,p.rollspeed,p.pitchspeed,p.yawspeed])]; %#ok<AGROW>
        case 'ATTITUDE_TARGET'
            target(end+1,:)=[k,v.rx_s,double(p.time_boot_ms)/1000+offset, ...
                quaternionEuler(double(p.q)),double([p.body_roll_rate,p.body_pitch_rate,p.body_yaw_rate])]; %#ok<AGROW>
        case 'HIL_ACTUATOR_CONTROLS'
            rawTime=double(p.time_usec)/1e6;
            act(end+1,:)=[k,v.rx_s,rawTime+offset,rawTime,double(p.mode),reshape(double(p.controls),1,[])]; %#ok<AGROW>
        case 'HEARTBEAT'
            mode=uint32(p.custom_mode);
            hb(end+1,:)=[k,v.rx_s,double(bitand(uint8(p.base_mode),128)~=0), ...
                double(bitand(bitshift(mode,-16),255)),double(bitand(bitshift(mode,-24),255)), ...
                double(p.base_mode),double(p.system_status)]; %#ok<AGROW>
        case 'STATUSTEXT'
            texts(end+1)=struct('raw_index',k,'rx_s',v.rx_s,'severity',double(p.severity), ...
                'text',strtrim(strrep(char(p.text),char(0),''))); %#ok<AGROW>
    end
end
for k=1:numel(e.raw_truth_datagrams)
    v=e.raw_truth_datagrams{k};bytes=uint8(v.bytes(:).');
    if numel(bytes)==264
        header=typecast(bytes(1:8),'int32');d=typecast(bytes(9:end),'double');
        assert(header(1)==1234567890&&header(2)==1,'analysis:DiagnosticHeader','Unexpected diagnostic header.');
        diagRows(end+1,:)=[k,v.rx_s,d(3)+truthOffset,d(3),d(1),d(2),d(4),d(5),d(6)]; %#ok<AGROW>
    elseif ismember(numel(bytes),[112,168,200])
        d=m600check.decodeTruthPacket(bytes,struct('expected_copter_id',1,'expected_vehicle_type',5));
        if ~d.valid
            badTruth(end+1)=struct('raw_index',k,'rx_s',v.rx_s,'bytes',numel(bytes),'reason',d.reason); %#ok<AGROW>
            continue
        end
        truth(end+1,:)=[k,v.rx_s,d.time_s+truthOffset,d.time_s,d.position_ned_m,d.velocity_ned_mps, ...
            d.euler_rad,d.angular_rate_body_radps,d.acceleration_body_mps2,d.motor_rpm, ...
            d.quaternion_norm,double(d.valid)]; %#ok<AGROW>
    end
end
assert(~isempty(truth)&&~isempty(att)&&~isempty(act),'analysis:MissingStreams','Required retained streams missing.');
% Differentiate unique sources and retain the complete packet tables.
[~,uniqueTruth]=unique(truth(:,4),'stable');uTruth=truth(uniqueTruth,:);
assert(all(diff(uTruth(:,4))>0),'analysis:TruthSourceReverse','Source reversal must be separately analyzed.');
n=size(uTruth,1);matched=nan(n,42);lagged=nan(6,1);lastTime=NaN;lastU=nan(6,1);
for k=1:n
    row=uTruth(k,:);t=row(3);mappedAct=find(act(:,3)<=t,1,'last');mappedTarget=find(target(:,3)<=t,1,'last');
    [pairDelta,at]=min(abs(att(:,3)-t));
    controls=nan(6,1);actLag=NaN;actIndex=NaN;
    if ~isempty(mappedAct)
        controls=act(mappedAct,6:11).';actLag=t-act(mappedAct,3);actIndex=act(mappedAct,1);
    end
    % The generated motor_rpm animation channel encodes 1000*model input.
    animationInput=row(20:25).'/1000;
    inputValid=all(isfinite(animationInput))&&all(animationInput>=0)&&all(animationInput<=1);
    if inputValid
        commandWrench=W*animationInput;
        if isnan(lastTime),lagged=animationInput;
        else
            alpha=exp(-(row(4)-lastTime)/0.12);
            lagged=alpha*lagged+(1-alpha)*lastU;
        end
        lastU=animationInput;lastTime=row(4);
        lagWrench=W*lagged;
    else,commandWrench=nan(6,1);lagWrench=nan(6,1);
    end
    angularAcceleration=nan(3,1);momentFromMotion=nan(3,1);eulerRateBody=nan(3,1);
    if k>1
        dt=row(4)-uTruth(k-1,4);omega=(row(14:16)+uTruth(k-1,14:16)).'/2;
        angularAcceleration=(row(14:16)-uTruth(k-1,14:16)).'/dt;
        momentFromMotion=I*angularAcceleration+cross(omega,I*omega);
        drpy=wrap(row(11:13)-uTruth(k-1,11:13)).'/dt;
        midpoint=uTruth(k-1,11:13)+wrap(row(11:13)-uTruth(k-1,11:13))/2;
        roll=midpoint(1);pitch=midpoint(2);
        eulerRateBody=[drpy(1)-drpy(3)*sin(pitch); ...
            drpy(2)*cos(roll)+drpy(3)*sin(roll)*cos(pitch); ...
            -drpy(2)*sin(roll)+drpy(3)*cos(roll)*cos(pitch)];
    end
    desired=nan(1,3);targetAge=NaN;
    if ~isempty(mappedTarget),desired=target(mappedTarget,4:6);targetAge=t-target(mappedTarget,3);end
    gap=wrap(att(at,4:6)-row(11:13));
    matched(k,:)=[row(1),row(3),row(4),actIndex,actLag,pairDelta, ...
        gap,row(11:13),att(at,4:6),desired,targetAge, ...
        commandWrench(1:3).',lagWrench(1:3).',angularAcceleration.',momentFromMotion.', ...
        (eulerRateBody-row(14:16).').',max(abs(controls-animationInput)), ...
        dot(commandWrench(1:3),row(14:16).'),dot(commandWrench(1:3),wrap(row(11:13)-desired).'), ...
        row(7),double(inputValid),row(14:16)];
end
% Preserve every packet and matched unique-source row.
mkdir(outputDir);
write(att,{'raw_index','rx_s','mapped_source_s','roll','pitch','yaw','p','q','r'},'ATTITUDE.csv');
write(target,{'raw_index','rx_s','mapped_source_s','roll_sp','pitch_sp','yaw_sp','p_sp','q_sp','r_sp'},'ATTITUDE_TARGET.csv');
write(act,[{'raw_index','rx_s','mapped_source_s','raw_source_s','mode'},reshape(cellstr(compose('control%d',1:16)),1,[])],'ACTUATOR.csv');
write(truth,[{'raw_index','rx_s','mapped_source_s','sim_s','x','y','z','vx','vy','vz','roll','pitch','yaw', ...
    'p','q','r','ax','ay','az'},reshape(cellstr(compose('animation%d',1:8)),1,[]),{'quat_norm','valid'}],'TRUTH.csv');
write(diagRows,{'raw_index','rx_s','mapped_source_s','sim_s','failed','failure_code','contact_force_N','ground','airborne'},'MODEL_DIAGNOSTIC.csv');
matchNames={'truth_raw_index','mapped_source_s','sim_s','actuator_raw_index','actuator_source_age_s','att_truth_pair_dt_s', ...
    'roll_gap','pitch_gap','yaw_gap','plant_roll','plant_pitch','plant_yaw','px4_roll','px4_pitch','px4_yaw', ...
    'desired_roll','desired_pitch','desired_yaw','target_source_age_s','command_tau_x','command_tau_y','command_tau_z', ...
    'approx_lag_tau_x','approx_lag_tau_y','approx_lag_tau_z','measured_alpha_x','measured_alpha_y','measured_alpha_z', ...
    'motion_inferred_tau_x','motion_inferred_tau_y','motion_inferred_tau_z','euler_vs_rate_x','euler_vs_rate_y','euler_vs_rate_z', ...
    'mavlink_vs_animation_max_gap','command_tau_dot_rate','command_tau_dot_attitude_error','z','animation_input_valid','plant_p','plant_q','plant_r'};
% The declared table width is derived from the actual column names below.
assert(size(matched,2)==numel(matchNames),'analysis:ColumnSchema','Moment row schema mismatch.');
write(matched,matchNames,'MOTOR_MOMENT_MOTION.csv');
if ~isempty(texts),writetable(struct2table(texts),fullfile(outputDir,'STATUSTEXT.csv'));end
apply=r.allocator_geometry_apply;entries=r.allocator_geometry_contract.entries;
exactApply=apply.passed&&apply.final_identity_verified&&numel(apply.final_readback)==12;
for k=1:numel(entries)
    q=find(strcmp({apply.final_readback.name},entries(k).name));
    exactApply=exactApply&&isscalar(q);
    if isscalar(q),v=apply.final_readback(q).row;exactApply=exactApply&&v.mav_type==9&&strcmpi(v.raw_bits_hex,entries(k).target_raw_bits_hex);end
end
inFlight=matched(:,2)>=armTime&matched(:,2)<=cut;
aligned=inFlight&matched(:,5)>=0&matched(:,5)<=r.config.state_max_age_s& ...
    matched(:,6)<=r.config.state_max_age_s&matched(:,39)==1;
crossings=struct();
for deg=[5,10,20,30,45,60]
    crossings.(sprintf('abs_roll_%d_deg',deg))=first(matched,inFlight&abs(matched(:,10))>=deg*pi/180);
end
ff=first(diagRows,diagRows(:,5)~=0);fp=[];
assert(~isempty(ff)&&ff(2)==cut,'analysis:FirstModelFaultAtom','Raw first failed diagnostic and retained first-fault receipt must agree.');
if ~isempty(ff),[~,q]=min(abs(truth(:,3)-ff(3)));fp=truth(q,:);end
motion=matched(aligned,:);targetsKnown=aligned&isfinite(matched(:,19))&matched(:,19)<=r.config.state_max_age_s;
inputInfo=dir(input);
out=struct('schema','HOST_NATIVE_HOVER_MOTOR_MOTION_ANALYSIS_V1','input',input,'input_sha256',sha(input), ...
    'geometry_audit_sha256',sha(auditFile),'input_bytes',inputInfo.bytes, ...
    'geometry_apply_12_of_12_exact',exactApply,'geometry_apply_receipt',apply, ...
    'source_time_offsets',struct('PX4_to_host_s',offset,'model_to_host_s',truthOffset), ...
    'events',r.events,'arm_request_s',armTime,'formal_begin_s',formalTime, ...
    'first_model_fault_received_s',cut,'first_model_fault_mapped_source_s',ff(3), ...
    'model_failure_handler_s',r.first_model_failure.time_s,'failure_capture_s',captureTime, ...
    'exception_s',exceptionTime,'comparison_window_end_basis','FIRST_RAW_MODEL_FAULT_RECEIVE__NOT_LATER_EXCEPTION_OR_FINALLY', ...
    'raw_counts',struct('MAVLink',numel(e.raw_mavlink),'truth_datagrams',numel(e.raw_truth_datagrams), ...
        'ATTITUDE',size(att,1),'ATTITUDE_TARGET',size(target,1),'actuator',size(act,1), ...
        'decoded_truth',size(truth,1),'unique_truth_source',size(uTruth,1),'diagnostic',size(diagRows,1), ...
        'truth_finite_body_rate_rows',nnz(all(isfinite(truth(:,14:16)),2))), ...
    'invalid_truth_packets',badTruth,'first_model_fault_row',ff,'nearest_truth_to_first_model_fault',fp, ...
    'model_fault_source_semantics','Code 4 identifies the flat-terrain initialization, nonfinite-input or changed-first-height latch. The 264-byte payload omits the reason and Terrain15 values.', ...
    'source_references',[sourceRef(fullfile(b,'matlab_validation','+m600check','copterSimFlatTerrainCore.m'), ...
        'Lines 30-40: all15 finite, explicit initialization and terrain(1) exact identity; line56 assigns code4; reason exists only internally.'), ...
        sourceRef(fullfile(b,'matlab_validation','+m600check','copterSimOutputs.m'), ...
        'NED/FRD output quaternion and angular-rate frame conversion.'), ...
        sourceRef(fullfile(gpenmpc_external_path('flat_terrain_model_dll'),'GPENMPC_M600_Canonical_ert_rtw','GPENMPC_M600_Canonical.cpp'), ...
        'Generated source rotor input permutation, allocation, canonical parameters and animation-only 1000*input evidence; linked by completed geometry audit.')], ...
    'angle_crossings_descriptive_only',crossings,'status_text',texts,'heartbeat_rows',hb, ...
    'comparison',struct('full_arm_to_failure_rows',nnz(inFlight),'aligned_diagnostic_rows',nnz(aligned), ...
        'max_abs_plant_roll_deg',finiteMax(abs(matched(inFlight,10)))*180/pi, ...
        'max_abs_estimate_plant_roll_gap_deg',finiteMax(abs(matched(inFlight,7)))*180/pi, ...
        'command_vs_motion_sign',signSummary(motion(:,20:22),motion(:,29:31)), ...
        'lagged_command_vs_motion_sign',signSummary(motion(:,23:25),motion(:,29:31)), ...
        'command_torque_dissipative_rate_fraction',negativeFraction(motion(:,36)), ...
        'command_torque_toward_reported_attitude_target_fraction',negativeFraction(matched(targetsKnown,37)), ...
        'max_bodyrate_vs_euler_derivative_gap',maxFiniteColumns(abs(motion(:,32:34))), ...
        'max_mavlink_vs_model_animation_control_gap',finiteMax(motion(:,35))), ...
    'hypotheses',{{'Geometry applied exactly but cannot alone establish or exclude dynamic stability.', ...
        'Compare actual corrective command moments with motion-inferred moments; large mismatch prioritizes held-command latency, actuator state/lag, or body-rate frame/units.', ...
        'Matching corrective moments but growing oscillation prioritizes native rate-loop gains/actuator lag, without yet modifying gains.', ...
        'Terrain latch is a separate terminal adapter mechanism; raw Terrain15 omission prevents unique changed/nonfinite attribution.'}}, ...
    'limitations',{{'Animation channels are 1000 times input control, NOT measured physical RPM or rotor-thrust states.', ...
        'First-order 0.12 s lag replay with held observed inputs.', ...
        'Motion-inferred torque uses finite differences and fixed model inertia; sampling, clock uncertainty and contact transients remain explicit.', ...
        'Sign fractions describe the observed directional response.', ...
        'The outer recovery receipt records final safety.'}}, ...
    'hardware_actions',0,'flight_admission',false,'unique_cause_proven',false);
f=fopen(fullfile(outputDir,'SUMMARY.json'),'w');assert(f>=0);g=onCleanup(@()fclose(f));
fprintf(f,'%s\n',jsonencode(out,PrettyPrint=true));clear g
disp(jsonencode(struct('geometry_applied',exactApply,'first_fault',ff,'comparison',out.comparison),PrettyPrint=true));
    function t=eventTime(kind),ix=find(strcmp({r.events.kind},kind),1);assert(~isempty(ix));t=r.events(ix).time_s;end
    function write(data,names,file),writetable(array2table(data,'VariableNames',names),fullfile(outputDir,file));end
end
function v=quaternionEuler(q)
q=double(q(:));assert(numel(q)==4&&all(isfinite(q))&&norm(q)>0);q=q/norm(q);w=q(1);x=q(2);y=q(3);z=q(4);
v=[atan2(2*(w*x+y*z),1-2*(x*x+y*y)),asin(max(-1,min(1,2*(w*y-z*x)))),atan2(2*(w*z+x*y),1-2*(y*y+z*z))];
end
function v=wrap(v),v=atan2(sin(v),cos(v));end
function v=first(rows,mask),i=find(mask,1);v=[];if ~isempty(i),v=rows(i,:);end;end
function v=finiteMax(x),x=x(isfinite(x));v=NaN;if ~isempty(x),v=max(x);end;end
function v=maxFiniteColumns(x),v=nan(1,size(x,2));for k=1:size(x,2),v(k)=finiteMax(x(:,k));end;end
function r=negativeFraction(x)
valid=isfinite(x)&x~=0;r=struct('nonzero_finite_denominator',nnz(valid),'negative_count',nnz(x(valid)<0),'fraction',NaN);
if any(valid),r.fraction=nnz(x(valid)<0)/nnz(valid);end
end
function r=signSummary(a,b)
r=repmat(struct('paired_nonzero_finite_rows',0,'same_sign_count',0,'fraction',NaN),1,3);
for k=1:3,v=isfinite(a(:,k))&isfinite(b(:,k))&a(:,k)~=0&b(:,k)~=0;r(k).paired_nonzero_finite_rows=nnz(v);
    r(k).same_sign_count=nnz(sign(a(v,k))==sign(b(v,k)));if any(v),r(k).fraction=r(k).same_sign_count/nnz(v);end
end
end
function h=sha(path)
f=fopen(path,'rb');assert(f>=0);g=onCleanup(@()fclose(f));m=java.security.MessageDigest.getInstance('SHA-256');m.update(fread(f,Inf,'*uint8'));
h=upper(reshape(dec2hex(typecast(m.digest(),'uint8'),2).',1,[]));clear g
end
function r=sourceRef(path,note)
f=dir(path);assert(isscalar(f),'analysis:SourceMissing','Missing source reference.');
r=struct('path',path,'bytes',f.bytes,'sha256',sha(path),'proof',note);
end
