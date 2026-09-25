function report=audit_m600_native_rotor_geometry(receipt,canonical)
% Audit typed parameters and plant geometry without accessing a device.
% The result is a candidate configuration.
build=fileparts(fileparts(mfilename('fullpath')));
source=fullfile(gpenmpc_external_path('flat_terrain_model_dll'), ...
    'GPENMPC_M600_Canonical_ert_rtw','GPENMPC_M600_Canonical.cpp');
sourceHash='8C9EEB3CACF50E042C1588E93EC50A4E9CE2703DB4DA6A837B2325AA43F9035C';
receiptPath=gpenmpc_external_path('native_control_readback_receipt');
receiptHash='F0C96E672086D8AB4F385F8FD3CCF085DF4E91B25D336D263AF23A3522404D95';
report=struct('schema','HOST_M600_NATIVE_ROTOR_GEOMETRY_AUDIT_V1', ...
    'passed',false,'failure','','hardware_actions',0,'flight_admission',false, ...
    'unique_instability_cause_proven',false,'source_sha256',sourceHash, ...
    'receipt_sha256',receiptHash,'receipt_path',receiptPath,'source_path',source);
try
    assert(strcmpi(sha(source),sourceHash),'m600check:SourceHash','Generated DLL source changed.');
    if nargin<1||isempty(receipt)
        f=dir(receiptPath);
        assert(isscalar(f)&&f.bytes==37832&&strcmpi(sha(receiptPath),receiptHash), ...
            'm600check:ReceiptHash','Actual typed receipt bytes/hash changed.');
        receipt=jsondecode(fileread(receiptPath));
        report.receipt_is_actual_file=true;
    else
        report.receipt_is_actual_file=false; % explicitly a host fixture
    end
    if nargin<2||isempty(canonical)
        canonical=struct('angles_deg',[0;60;120;180;240;300], ...
            'spin_sign',[1;-1;1;-1;1;-1],'arm_radius_m',0.5665, ...
            'yaw_arm_m',0.025,'rotor_order',[5;1;4;6;2;3], ...
            'thrust_upper_n',32.145727009134916,'mass_kg',11.71);
    end
    validateCanonical(canonical);
    assert(strcmp(receipt.schema,'M600_CANONICAL_SERIAL_READONLY_SAFETY_V1')&& ...
        receipt.passed&&receipt.COM_closed&&~receipt.armed&&receipt.landed_state==1&& ...
        receipt.diagnostic_parameters_complete&&receipt.virtual_output_path_disabled&& ...
        receipt.physical_output_path_disabled,'m600check:ReceiptSafety','Receipt is not safe and complete.');
    assert(strcmp(receipt.uid,gpenmpc_device_identity('uid'))&&receipt.product_id==56&& ...
        strcmp(receipt.commit,'6ea3539157ca358c70a515878b77077af7d4611d')&& ...
        strcmpi(receipt.flight_custom_version_hex,'000000579153A36E'), ...
        'm600check:ReceiptIdentity','Unexpected board/application identity.');
    zero={'parameter_writes','mapping_writes','arm_disarm_mode_requests','flash_reboot_count','physical_output_actions'};
    for k=1:numel(zero),assert(receipt.(zero{k})==0,'m600check:NotReadonly','Receipt reports mutations.');end
    d=receipt.diagnostic_parameter_observations(:);
    assert(numel(d)==79&&numel(unique(string({d.name})))==79, ...
        'm600check:DiagnosticDenominator','Expected 79 distinct diagnostic parameter rows.');
    for k=1:numel(d)
        assert(isempty(d(k).read_error)&&d(k).write_count==0&& ...
            strcmp(d(k).name,d(k).typed_value.name),'m600check:TypedRow','Bad diagnostic row.');
        validateTyped(d(k).typed_value);
    end
    base=receipt.parameters(:);
    assert(numel(unique(string({base.name})))==numel(base),'m600check:DuplicateBase','Duplicate base row.');
    requireBase('SYS_HITL',1); requireBase('SYS_AUTOSTART',6001);
    requireBase('CA_ROTOR_COUNT',6); requireBase('RA_CTRL_MODE',0);
    assert(val('CA_AIRFRAME',6)==0&&val('CA_METHOD',6)==2&&val('CA_R_REV',6)==0, ...
        'm600check:AllocatorMode','The observed native allocator must use non-reversible multirotor motors.');
    assert(val('THR_MDL_FAC',9)==0,'m600check:ThrustModel','Nonzero thrust model needs a separate interface audit.');
    pos=zeros(3,6);axis=zeros(3,6);ct=zeros(1,6);km=zeros(1,6);
    for motor=0:5
        for q=1:3
            letters='XYZ';pos(q,motor+1)=val(sprintf('CA_ROTOR%d_P%c',motor,letters(q)),9);
            axis(q,motor+1)=val(sprintf('CA_ROTOR%d_A%c',motor,letters(q)),9);
        end
        ct(motor+1)=val(sprintf('CA_ROTOR%d_CT',motor),9);
        km(motor+1)=val(sprintf('CA_ROTOR%d_KM',motor),9);
    end
    assert(all(ct>0)&&all(ct==ct(1)),'m600check:ThrustCoefficient','Expected positive common CT, not guessed scaling.');
    assert(all(all(axis==repmat([0;0;-1],1,6)))&&all(pos(3,:)==0), ...
        'm600check:AxisContract','This audit only covers actual horizontal, non-tilted rotors.');
    actual=effectiveness(pos,axis,ct,km);
    sw=zeros(1,6);canonicalPerN=zeros(6,6);targetPos=zeros(3,6);
    for ch=1:6
        sw(ch)=find(canonical.rotor_order==ch);
        angle=canonical.angles_deg(sw(ch));
        % Use sind/cosd for exact orthogonal zeros.
        targetPos(:,ch)=[canonical.arm_radius_m*cosd(angle);canonical.arm_radius_m*sind(angle);0];
        canonicalPerN(:,ch)=[-targetPos(2,ch);targetPos(1,ch); ...
            canonical.yaw_arm_m*canonical.spin_sign(sw(ch));0;0;-1];
    end
    proposalPos=double(single(targetPos)); % exact REAL32 parameter target
    candidate=effectiveness(proposalPos,axis,ct,km);
    actualPerN=actual./ct; candidatePerN=candidate./ct;
    crossMap=canonicalPerN(1:2,:)*pinv(actualPerN(1:2,:));
    delta=repmat(struct('name','','mav_type',9,'original_value',0,'original_raw_bits_hex','', ...
        'candidate_value',0,'candidate_raw_bits_hex','','rollback_value',0, ...
        'rollback_raw_bits_hex','','provenance','','changed',false),12,1);
    for ch=1:6
        for q=1:2
            n=sprintf('CA_ROTOR%d_P%c',ch-1,'X'+q-1);v=typed(n,9);j=2*(ch-1)+q;
            nv=proposalPos(q,ch);
            delta(j)=struct('name',n,'mav_type',9,'original_value',double(v.decoded), ...
                'original_raw_bits_hex',upper(v.raw_bits_hex),'candidate_value',nv, ...
                'candidate_raw_bits_hex',bits(nv),'rollback_value',double(v.decoded), ...
                'rollback_raw_bits_hex',upper(v.raw_bits_hex), ...
                'provenance','MODEL_GEOMETRY_INTERFACE_ALIGNMENT', ...
                'changed',~strcmpi(v.raw_bits_hex,bits(nv)));
        end
    end
    report.canonical=canonical;report.motor_to_software_rotor=sw;
    report.actual_position_frd_m=pos;report.proposed_position_frd_real32_m=proposalPos;
    report.actual_effectiveness_6x6=actual;
    report.actual_effectiveness_per_ct=actualPerN;
    report.canonical_wrench_per_newton=canonicalPerN;
    report.canonical_wrench_per_normalized_control=canonicalPerN*canonical.thrust_upper_n;
    report.proposed_effectiveness_6x6=candidate;
    report.proposed_effectiveness_per_ct=candidatePerN;
    report.roll_pitch_cross_axis_map=crossMap;
    report.roll_pitch_rotation_deg=atan2d(crossMap(2,1)-crossMap(1,2),crossMap(1,1)+crossMap(2,2));
    report.actual_alignment_max_abs=max(abs(actualPerN-canonicalPerN),[],'all');
    report.candidate_alignment_max_abs=max(abs(candidatePerN-canonicalPerN),[],'all');
    report.numerical_real32_alignment_tolerance=1e-7;
    report.tolerance_provenance='REAL32_ROUNDING';
    report.candidate_geometry_aligned=report.candidate_alignment_max_abs<=1e-7;
    report.actual_yaw_sign_matches=isequal(sign(actualPerN(3,:)),sign(canonicalPerN(3,:)));
    report.actual_collective_thrust_matches=max(abs(actualPerN(4:6,:)-canonicalPerN(4:6,:)),[],'all')==0;
    report.proposed_parameter_delta=delta;
    report.unchanged_diagnostic_parameters=d(~ismember(string({d.name}),string({delta.name})));
    report.expected_equal_control_hover=canonical.mass_kg*9.80665/(6*canonical.thrust_upper_n);
    report.units={'tau_x_Nm','tau_y_Nm','tau_z_Nm','force_x_N','force_y_N','force_z_N'};
    report.normalization_boundary=['Raw CT=6.5 defines allocator effectiveness. ' ...
        'PX4 normalized allocation is retained; only geometry per CT is compared with model moment per N.'];
    report.candidate_scope=['PROPOSED_NOT_EXECUTED: only 12 CA_ROTOR*_PX/PY REAL32 values; ' ...
        'Original raw parameter bits define rollback targets.'];
    report.adapter_alternative=['The Hex-X angle sets differ by 30 degrees; resolving this requires a geometry transform. ' ...
        'A dense virtual-motor remixer also changes saturation, nullspace and allocator behavior.'];
    report.claim=['The static allocator/model interface is inconsistent. ' ...
        'Closed-loop hover behavior requires a separate dynamic assessment.'];
    report.passed=true;
catch e
    report.failure=struct('identifier',e.identifier,'message',e.message);
end
    function v=typed(name,mavType)
        ix=find(strcmp({d.name},name));
        assert(isscalar(ix),'m600check:MissingParameter','Missing or duplicate %s.',name);
        v=d(ix).typed_value;
        assert(v.mav_type==mavType,'m600check:ParameterType','Wrong MAVLink type for %s.',name);
    end
    function v=val(name,mavType),t=typed(name,mavType);v=double(t.decoded);end
    function requireBase(name,value)
        ix=find(strcmp({base.name},name));assert(isscalar(ix),'m600check:MissingBase','Missing base %s.',name);
        t=base(ix);validateTyped(t);assert(t.mav_type==6&&double(t.decoded)==value,'m600check:BaseIdentity','Base identity differs.');
    end
end

function e=effectiveness(pos,axis,ct,km)
e=zeros(6,6);
for i=1:6,a=axis(:,i)/norm(axis(:,i));e(:,i)=ct(i)*[cross(pos(:,i),a)-km(i)*a;a];end
end
function validateTyped(t)
assert(ischar(t.raw_bits_hex)&&numel(t.raw_bits_hex)==8&& ...
    all(ismember(upper(t.raw_bits_hex),'0123456789ABCDEF'))&&isscalar(t.decoded)&&isfinite(t.decoded), ...
    'm600check:TypedBits','Invalid typed value.');
u=uint32(hex2dec(t.raw_bits_hex));
if t.mav_type==9,v=double(typecast(u,'single'));elseif t.mav_type==6,v=double(typecast(u,'int32'));
else,error('m600check:TypedType','Unsupported parameter MAVLink type.');end
assert(isfinite(v)&&isequal(v,double(t.decoded)),'m600check:TypedBits','Decoded value does not match raw bits.');
end
function validateCanonical(c)
assert(numel(c.angles_deg)==6&&numel(c.spin_sign)==6&&numel(c.rotor_order)==6&& ...
    all(isfinite(c.angles_deg))&&all(ismember(c.spin_sign,[-1,1]))&& ...
    isequal(sort(c.rotor_order(:)),(1:6).')&&isscalar(c.arm_radius_m)&&c.arm_radius_m>0&& ...
    isfinite(c.arm_radius_m)&&isscalar(c.yaw_arm_m)&&isfinite(c.yaw_arm_m)&&c.yaw_arm_m>0&& ...
    isscalar(c.thrust_upper_n)&&isfinite(c.thrust_upper_n)&&c.thrust_upper_n>0&& ...
    isscalar(c.mass_kg)&&isfinite(c.mass_kg)&&c.mass_kg>0, ...
    'm600check:CanonicalInput','Invalid canonical geometry or motor permutation.');
end
function s=bits(x),s=upper(dec2hex(typecast(single(x),'uint32'),8));end
function s=sha(path)
f=fopen(path,'rb');assert(f>=0,'m600check:File','Cannot read source file.');c=onCleanup(@()fclose(f));
b=fread(f,Inf,'*uint8');h=java.security.MessageDigest.getInstance('SHA-256');h.update(b);
s=upper(reshape(dec2hex(typecast(h.digest(),'uint8'),2).',1,[]));clear c
end
