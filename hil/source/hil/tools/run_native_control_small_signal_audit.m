function report=run_native_control_small_signal_audit(outputDir)
% Compare baseline and candidate XY geometry with offline small-signal algebra.
arguments,outputDir (1,1) string,end
assert(~isfolder(outputDir),'m600check:AuditOutputExists','Preserve earlier outputs.');
mkdir(outputDir);
build=fileparts(fileparts(mfilename('fullpath')));
px4=gpenmpc_external_path('px4_module_source_root');
nativePath=gpenmpc_external_path('native_control_readback_receipt');
auditPath=fullfile(gpenmpc_external_path('host_native_rotor_geometry'),'RESULT.json');
sourcePath=fullfile(gpenmpc_external_path('flat_terrain_model_dll'),'GPENMPC_M600_Canonical_ert_rtw','GPENMPC_M600_Canonical.cpp');
sourceSpecs={nativePath,'F0C96E672086D8AB4F385F8FD3CCF085DF4E91B25D336D263AF23A3522404D95'; ...
    auditPath,'4A399A8569FC3E0178AFF641A417586B86FC5589667439A82EE30D4EB2DEF511'; ...
    sourcePath,'8C9EEB3CACF50E042C1588E93EC50A4E9CE2703DB4DA6A837B2325AA43F9035C'; ...
    fullfile(px4,'lib','rate_control','rate_control.cpp'),'446012432C15E5CA49038E73796BA847EB611ABBE02F003BD8B198275F8DC960'; ...
    fullfile(px4,'modules','mc_rate_control','MulticopterRateControl.cpp'),'CADFD68FAF87E2A7828074FA5130BA2D1C38C1C93B7E9760D94570B4318AE2C4'; ...
    fullfile(px4,'modules','mc_att_control','AttitudeControl','AttitudeControl.cpp'),'25733BEF293C2670E3D327D8830C91F9665F226606CF33F13DD42FB91095E03C'; ...
    fullfile(px4,'lib','control_allocation','control_allocation','ControlAllocationPseudoInverse.cpp'),'3200E76A8A150C0DD514C8E4008D7392729BCA53BF42E71151A74D364C803499'; ...
    fullfile(px4,'modules','control_allocator','VehicleActuatorEffectiveness','ActuatorEffectivenessRotors.cpp'),'C2D49F8EE5F3D2FEE7B373C73AC79AC5EB7669AD01AC94072CB7ED75B7FF5627'; ...
    fullfile(px4,'modules','control_allocator','VehicleActuatorEffectiveness','ActuatorEffectivenessMultirotor.hpp'),'82C1BEB4513547C43BB82E3557388AED94714BB3C610C4F2EE51E25F607A6D82'; ...
    fullfile(px4,'lib','control_allocation','control_allocation','ControlAllocationSequentialDesaturation.cpp'),'07E60BC11A9948D41C99950FD99FFF06368BB6203F05B79D64386A0A04351D7C'; ...
    fullfile(px4,'modules','control_allocator','ControlAllocator.cpp'),'56482F1E2D384CF3151E438065D0EB00555A01F060C8439F6115525B6EED938F'; ...
    fullfile(px4,'lib','mixer_module','functions','FunctionMotors.hpp'),'BE4E6AAA363BC49A5B63CC5683E6CDB805494955431D250F3023EE70C9F8628A'};
report=struct('schema','HOST_NATIVE_ATTITUDE_RATE_NORMALIZATION_LAG_AUDIT_V1', ...
    'passed',false,'failure','','classification','HOST_SMALL_SIGNAL_ANALYSIS', ...
    'hardware_actions',0,'COM_UDP_actions',0,'model_or_gain_changes',0, ...
    'unique_instability_cause_proven',false,'flight_admission',false,'bindings',[], ...
    'script_identity',identity([mfilename('fullpath') '.m']));
try
    for k=1:size(sourceSpecs,1)
        r=identity(sourceSpecs{k,1});assert(strcmp(r.sha256,sourceSpecs{k,2}), ...
            'm600check:SmallSignalInputHash','Consumed source/parameter identity differs: %s',r.path);
        if isempty(report.bindings),report.bindings=r;else,report.bindings(end+1)=r;end
    end
    native=jsondecode(fileread(nativePath));proof=jsondecode(fileread(auditPath));a=proof.audit;
    assert(native.passed&&native.diagnostic_parameters_complete&&numel(native.diagnostic_parameter_observations)==79);
    assert(proof.passed&&a.passed&&a.candidate_geometry_aligned&&a.hardware_actions==0);
    rows=native.diagnostic_parameter_observations;assert(numel(unique({rows.name}))==79);
    for k=1:numel(rows),assert(isempty(rows(k).read_error)&&rows(k).write_count==0);decode(rows(k).typed_value);end
    assert(value('CA_AIRFRAME',6)==0&&value('CA_METHOD',6)==2&&value('CA_R_REV',6)==0&& ...
        value('MC_AIRMODE',6)==0&&value('THR_MDL_FAC',9)==0,'m600check:SmallSignalNativeMode','Actual native allocation mode differs.');
    source=fileread(sourcePath);J=extractArray(source,'f_calibration_mass_inertia_iner',3);
    angles=extractArray(source,'f_calibration_rotor_allocation_',6);
    spin=extractArray(source,'f_calibration_rotor_allocatio_0',6);
    tokens=regexp(source,['f_calibration_rotor_allocation_,\s*f_calibration_rotor_allocatio_0,\s*' ...
        '([\d.eE+\-]+),\s*([\d.eE+\-]+),\s*([\d.eE+\-]+),\s*([\d.eE+\-]+),\s*f_calibration_mass_inertia_iner'], 'tokens');
    assert(numel(tokens)==1,'m600check:SmallSignalModelParse','Unique generated model call required.');
    modelValues=str2double(tokens{1});arm=modelValues(1);yawArm=modelValues(2);fmax=modelValues(3);tau=modelValues(4);
    assert(all(isfinite(modelValues))&&isequal(J,[1.6;1.6;3])&&tau==.12&& ...
        arm==a.canonical.arm_radius_m&&yawArm==a.canonical.yaw_arm_m&&fmax==a.canonical.thrust_upper_n&& ...
        isequal(angles,a.canonical.angles_deg(:))&&isequal(spin,a.canonical.spin_sign(:)), ...
        'm600check:SmallSignalModelBinding','Generated model constants no longer match this audit identity.');
    assert(contains(source,'dx[k + 13] = (rotorCommandN[k] - x[k + 13]) /'), ...
        'm600check:SmallSignalLagLaw','Thrust-state lag source changed.');
    nativePos=zeros(3,6);axis=zeros(3,6);ct=zeros(1,6);km=zeros(1,6);candidatePos=zeros(3,6);
    for motor=0:5
        for d=1:3
            letters='XYZ';nativePos(d,motor+1)=value(sprintf('CA_ROTOR%d_P%c',motor,letters(d)),9);
            axis(d,motor+1)=value(sprintf('CA_ROTOR%d_A%c',motor,letters(d)),9);
        end
        candidatePos(:,motor+1)=nativePos(:,motor+1);
        ct(motor+1)=value(sprintf('CA_ROTOR%d_CT',motor),9);km(motor+1)=value(sprintf('CA_ROTOR%d_KM',motor),9);
    end
    for k=1:numel(a.proposed_parameter_delta)
        e=a.proposed_parameter_delta(k);observed=typed(e.name,9);
        assert(strcmpi(observed.raw_bits_hex,e.original_raw_bits_hex));
        match=regexp(e.name,'^CA_ROTOR([0-5])_P([XY])$','tokens','once');assert(~isempty(match));
        index=str2double(match{1})+1;axisIndex=find('XY'==match{2});
        candidatePos(axisIndex,index)=double(typecast(uint32(hex2dec(e.candidate_raw_bits_hex)),'single'));
    end
    Aoriginal=effectiveness(nativePos,axis,ct,km);Acorrected=effectiveness(candidatePos,axis,ct,km);
    assert(max(abs(Aoriginal-a.actual_effectiveness_6x6),[],'all')<1e-12&& ...
        max(abs(Acorrected-a.proposed_effectiveness_6x6),[],'all')<1e-12);
    B=zeros(6,6);order=a.canonical.rotor_order(:);
    assert(isequal(sort(order),(1:6).'));
    for ch=1:6
        rotor=find(order==ch);x=arm*cosd(angles(rotor));y=arm*sind(angles(rotor));
        B(:,ch)=fmax*[-y;x;yawArm*spin(rotor);0;0;-1];
    end
    assert(max(abs(B-a.canonical_wrench_per_normalized_control),[],'all')<1e-12);
    [Moriginal,scaleOriginal]=normalizedMix(Aoriginal);
    [Mcorrected,scaleCorrected]=normalizedMix(Acorrected);
    [MdoubleCT,~]=normalizedMix(2*Acorrected);
    assert(max(abs(MdoubleCT-Mcorrected),[],'all')<1e-12,'m600check:SmallSignalCTCancellation','Uniform CT must cancel in this normalized no-saturation mapping.');
    Koriginal=B(1:2,:)*Moriginal(:,1:2);Kcorrected=B(1:2,:)*Mcorrected(:,1:2);
    P=zeros(2);I=P;D=P;Ka=P;selected=[];
    for d=1:2
        names={'ROLL','PITCH'};axisName=names{d};gainK=value(['MC_' axisName 'RATE_K'],9);
        P(d,d)=gainK*value(['MC_' axisName 'RATE_P'],9);I(d,d)=gainK*value(['MC_' axisName 'RATE_I'],9);
        D(d,d)=gainK*value(['MC_' axisName 'RATE_D'],9);Ka(d,d)=value(['MC_' axisName '_P'],9);
        assert(value(['MC_' axisName 'RATE_FF'],9)==0);
        for suffix={'P','I','D','K','FF'}
            p=typed(['MC_' axisName 'RATE_' suffix{1}],9);if isempty(selected),selected=p;else,selected(end+1)=p;end
        end
        selected(end+1)=typed(['MC_' axisName '_P'],9);
    end
    assert(isequal(diag(P),repmat(double(single(.15)),2,1))&& ...
        isequal(diag(I),repmat(double(single(.20)),2,1))&& ...
        isequal(diag(D),repmat(double(single(.003)),2,1))&&isequal(diag(Ka),[6.5;6.5]), ...
        'm600check:SmallSignalGainBinding','Use the bound controller gains.');
    report.selected_actual_typed_parameters=selected;
    report.model_constants_from_generated_source=struct('J_kg_m2',J,'thrust_state_tau_s',tau, ...
        'arm_m',arm,'yaw_moment_arm_m',yawArm,'per_rotor_command_full_scale_N',fmax);
    report.original=struct('effectiveness',Aoriginal,'normalized_mix',Moriginal,'normalization_scale',scaleOriginal, ...
        'model_moment_gain_Nm_per_normalized_torque',Koriginal,'rate_integral_enabled', ...
        stateModel(Koriginal,J(1:2),tau,P,I,D,Ka,true),'rate_integral_frozen',stateModel(Koriginal,J(1:2),tau,P,I,D,Ka,false));
    report.corrected=struct('effectiveness',Acorrected,'normalized_mix',Mcorrected,'normalization_scale',scaleCorrected, ...
        'model_moment_gain_Nm_per_normalized_torque',Kcorrected,'rate_integral_enabled', ...
        stateModel(Kcorrected,J(1:2),tau,P,I,D,Ka,true),'rate_integral_frozen',stateModel(Kcorrected,J(1:2),tau,P,I,D,Ka,false));
    g=sqrt(6)*arm*fmax/J(1);p=P(1,1);i=I(1,1);d=D(1,1);ka=Ka(1,1);
    coeff=[tau,1+g*d,g*p,g*(p*ka+i),g*i*ka];b1=(coeff(2)*coeff(3)-coeff(1)*coeff(4))/coeff(2);
    c1=(b1*coeff(4)-coeff(2)*coeff(5))/b1;
    report.symmetric_ideal_crosscheck=struct('note','Exact regular hex formula; corrected REAL32 matrix differs only by rounding.', ...
        'moment_gain_Nm_per_normalized_torque',sqrt(6)*arm*fmax,'angular_acceleration_gain',g, ...
        'I_enabled_characteristic_coefficients',coeff,'I_enabled_roots',poleReport(roots(coeff)), ...
        'I_frozen_characteristic_coefficients',[tau,1+g*d,g*p,g*p*ka], ...
        'I_frozen_roots',poleReport(roots([tau,1+g*d,g*p,g*p*ka])), ...
        'rate_only_characteristic_coefficients',[tau,1+g*d,g*p,g*i], ...
        'rate_only_roots',poleReport(roots([tau,1+g*d,g*p,g*i])), ...
        'routh_first_column',[coeff(1);coeff(2);b1;c1;coeff(5)]);
    report.equations=struct('attitude','omega_sp = Ka*(theta_sp-theta), small-angle roll/pitch only', ...
        'rate','u = P*(omega_sp-omega) + integral_state - D*omega_dot; FF=0', ...
        'thrust_moment','tau*moment_dot + moment = K_moment*u', ...
        'rigid_body','theta_dot=omega; J*omega_dot=moment, linearized at zero body rates', ...
        'integral_enabled','integral_state_dot=I*(omega_sp-omega)', ...
        'integral_frozen','integral derivative=0; zero perturbation of held integral; remove invariant zero modes', ...
        'state_order','theta_roll_pitch; omega_roll_pitch; physical_moment_roll_pitch; integral_state_roll_pitch when enabled');
    rateSource=fileread(sourceSpecs{4,1});moduleSource=fileread(sourceSpecs{5,1});
    assert(contains(rateSource,'if (!landed)')&&contains(rateSource,'updateIntegral(rate_error, dt);')&& ...
        contains(moduleSource,'_maybe_landed || _landed')&&contains(moduleSource,'_rate_control.resetIntegral();'));
    report.integral_source_evidence=struct('rate_source_lines',[78 81 82 91 115], ...
        'module_source_lines',[140 145 192 193 215 220], ...
        'update_condition','RateControl::update updates integral only if !landed; caller passes maybe_landed || landed.', ...
        'disarmed_condition','MulticopterRateControl resets integral when not armed or not rotary-wing.', ...
        'additional_nonlinearity','Actual integral includes saturation sign clamping, 400 deg/s i_factor reduction and integral limit.', ...
        'do_not_infer_from_extended_state','maybe_landed and internal integral state require separate observations.');
    report.analysis=struct('uniform_CT_cancels',true,'scope','IDEAL_LINEAR_SMALL_SIGNAL_MODEL', ...
        'growth_limit','The ideal corrected model has slow unstable growth (~0.2465/s); fast large-angle motion requires additional dynamic analysis.', ...
        'required_raw_alignment','Earliest roll/rate departure, reference, actual control vector and source times, thrust state/moment, saturation and landed/maybe_landed integral enable.');
    report.exclusions={'Continuous controller with zero transport delay.';'No IMU/EKF filtering or estimator dynamics'; ...
        'Unsaturated allocation with full available thrust authority.'; ...
        'No ground/contact constraints, large-angle coupling or position-loop dynamics'; ...
        'No battery scale, actuator slew limiting or failure-detector dynamics'; ...
        'Matrix pseudoinverse uses MATLAB double precision; not bit-exact PX4 float geninv or generated-code replay'; ...
        'Local source identity is recorded by SHA-256.'};
    report.passed=true;
catch problem
    report.failure=[problem.identifier ': ' problem.message];
end
fid=fopen(fullfile(outputDir,'SMALL_SIGNAL_RESULT.json'),'w','n','UTF-8');assert(fid>=0);cleanup=onCleanup(@()fclose(fid)); %#ok<NASGU>
fprintf(fid,'%s',jsonencode(report,PrettyPrint=true));
fprintf('HOST native small-signal audit passed=%d; COM/board/model changes=0\n',report.passed);
assert(report.passed,'m600check:SmallSignalAuditFailed','See preserved SMALL_SIGNAL_RESULT.json.');
    function p=typed(name,type)
        idx=find(strcmp({rows.name},name));assert(isscalar(idx));p=rows(idx).typed_value;
        assert(p.mav_type==type&&strcmp(p.name,name));decode(p);
    end
    function v=value(name,type),p=typed(name,type);v=decode(p);end
end
function v=decode(p)
assert(ischar(p.raw_bits_hex)&&~isempty(regexp(p.raw_bits_hex,'^[0-9A-Fa-f]{8}$','once')));
u=uint32(hex2dec(p.raw_bits_hex));if p.mav_type==9,v=double(typecast(u,'single'));elseif p.mav_type==6,v=double(typecast(u,'int32'));else,error('m600check:SmallSignalType','Unexpected type.');end
assert(isscalar(p.decoded)&&isfinite(v)&&isequal(double(p.decoded),v));
end
function values=extractArray(source,name,n)
tokens=regexp(source,['static const real_T ' name '\[' num2str(n) '\] = \{([^}]+)\}'],'tokens');
assert(numel(tokens)==1);values=str2double(strsplit(strtrim(tokens{1}{1}),',')).';
assert(numel(values)==n&&all(isfinite(values)));
end
function A=effectiveness(pos,axis,ct,km)
A=zeros(6,6);for k=1:6,u=axis(:,k)/norm(axis(:,k));A(:,k)=ct(k)*[cross(pos(:,k),u)-km(k)*u;u];end
end
function [mix,scale]=normalizedMix(A)
% Source formulas ControlAllocationPseudoInverse.cpp 83-173; n=6 motors.
assert(rank(A)==4);mix=pinv(A);scale=ones(6,1);
nRoll=nnz(abs(mix(:,1))>1e-3);nPitch=nnz(abs(mix(:,2))>1e-3);
roll=sqrt(sum(mix(:,1).^2)/(nRoll/2));pitch=sqrt(sum(mix(:,2).^2)/(nPitch/2));
scale(1:2)=max(roll,pitch);scale(3)=max(mix(:,3));
for axisIndex=6:-1:4
    count=nnz(abs(mix(:,axisIndex))>double(eps('single')));
    if count>0,scale(axisIndex)=sum(abs(mix(:,axisIndex)))/count;else,scale(axisIndex)=scale(6);end
end
mix=mix./scale.';mix(abs(mix)<1e-3)=0;
end
function report=stateModel(K,J,tau,P,I,D,Ka,integralEnabled)
if integralEnabled,n=8;else,n=6;end
A=zeros(n);Ji=diag(1./J(:));A(1:2,3:4)=eye(2);A(3:4,5:6)=Ji;
A(5:6,1:2)=-K*P*Ka/tau;A(5:6,3:4)=-K*P/tau;A(5:6,5:6)=-(eye(2)+K*D*Ji)/tau;
if integralEnabled,A(5:6,7:8)=K/tau;A(7:8,1:2)=-I*Ka;A(7:8,3:4)=-I;end
report=struct('integral_enabled',logical(integralEnabled),'A',A,'poles',poleReport(eig(A)));
end
function r=poleReport(p)
p=p(:);[~,ix]=sortrows([real(p),imag(p)],[1 2]);p=p(ix);maxReal=max(real(p));
growthDoubling=NaN;if maxReal>0,growthDoubling=log(2)/maxReal;end
r=struct('real_s_inverse',real(p),'imag_s_inverse',imag(p),'frequency_Hz',abs(imag(p))/(2*pi), ...
    'maximum_real_part_s_inverse',maxReal,'asymptotically_stable_ideal_model',all(real(p)<0), ...
    'growth_doubling_s_if_unstable',growthDoubling,'nan_meaning','Not applicable where no positive growth exists.');
end
function r=identity(path)
fid=fopen(path,'rb');assert(fid>=0);cleanup=onCleanup(@()fclose(fid)); %#ok<NASGU>
raw=fread(fid,Inf,'*uint8');md=java.security.MessageDigest.getInstance('SHA-256');md.update(raw);
h=upper(reshape(dec2hex(typecast(md.digest(),'uint8'),2).',1,[]));r=struct('path',char(path),'bytes',numel(raw),'sha256',h);
end
