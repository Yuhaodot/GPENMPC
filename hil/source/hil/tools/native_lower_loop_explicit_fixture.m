function [cfg,metadata]=native_lower_loop_explicit_fixture()
% Combine typed parameters with explicit test inputs.
b=fileparts(fileparts(mfilename('fullpath')));
current=gpenmpc_external_path('native_control_readback_receipt');
geometry=fullfile(gpenmpc_external_path('host_native_rotor_geometry'),'RESULT.json');
currentIdentity=identity(current);
geometryIdentity=identity(geometry);
assert(strcmp(geometryIdentity.sha256,'4A399A8569FC3E0178AFF641A417586B86FC5589667439A82EE30D4EB2DEF511'), ...
    'gpenmpc:GeometryIdentity','Consumed corrected-geometry audit changed.');
assert(strcmp(currentIdentity.sha256,'F0C96E672086D8AB4F385F8FD3CCF085DF4E91B25D336D263AF23A3522404D95'), ...
    'gpenmpc:CurrentIdentity','Typed parameter receipt changed.');
r=jsondecode(fileread(current));g=jsondecode(fileread(geometry));
assert(r.passed&&g.passed&&g.audit.passed&&g.audit.candidate_geometry_aligned, ...
    'gpenmpc:CurrentReceipt','Current read-only/geometry evidence must pass.');
assert(strcmp(g.audit.receipt_sha256,currentIdentity.sha256),'gpenmpc:GeometryParent','Geometry must bind to the typed parameter receipt.');
rows=r.diagnostic_parameter_observations;used=struct('name',{},'mav_type',{},'raw_bits_hex',{},'value',{});
cfg=struct('mc_airmode',value('MC_AIRMODE',6), ...
    'parameter_provenance','CURRENT_TYPED_AND_EXPLICIT_FIXTURE', ...
    'attitude_p',[value('MC_ROLL_P',9);value('MC_PITCH_P',9);value('MC_YAW_P',9)], ...
    'rate_k',gains('K'),'rate_p',gains('P'),'rate_i',gains('I'),'rate_d',gains('D'),'rate_ff',gains('FF'), ...
    'effectiveness',double(g.audit.proposed_effectiveness_6x6), ...
    'yaw_weight',1.0,'rate_limits_rad_s',[10;10;10], ...
    'integral_limits',[1;1;1],'yaw_torque_cutoff_hz',0.0, ...
    'battery_scale_enabled',false,'slew_limits_s',zeros(6,1),'initial_motor_commands',zeros(6,1));
assert(cfg.mc_airmode==0&&isequal(size(cfg.effectiveness),[6,6]));
missing=struct('field',{},'value',{},'status',{},'reason',{});
add('yaw_weight',cfg.yaw_weight,'Full-weight yaw test setting.');
add('rate_limits_rad_s',cfg.rate_limits_rad_s,'10 rad/s test limit; record limiter activity.');
add('integral_limits',cfg.integral_limits,'Normalized unit integral-limit fixture.');
add('yaw_torque_cutoff_hz',cfg.yaw_torque_cutoff_hz,'AlphaFilter cutoff 0 explicitly selects passthrough.');
add('battery_scale_enabled',cfg.battery_scale_enabled,'Battery-scale compensation disabled for this test.');
add('slew_limits_s',cfg.slew_limits_s,'Actuator slew limiting disabled for this test.');
add('initial_motor_commands',cfg.initial_motor_commands,'Explicit diagnostic allocator state; SIL caller must record any equilibrium initialization override.');
metadata=struct('schema','PX4_NATIVE_LOWER_LOOP_EXPLICIT_HOST_FIXTURE_V1', ...
    'current_typed_receipt',currentIdentity,'geometry_receipt',geometryIdentity, ...
    'current_typed_fields',used,'missing_fields_explicit_fixtures',missing, ...
    'geometry_basis','NORMALIZED_CT_KM_EFFECTIVENESS_WITH_12_REAL32_GEOMETRY_PARAMETERS', ...
    'current_all_parameters_known',false, ...
    'hardware_actions',0,'flight_admission',false, ...
    'claim','Host kernel test inputs.');
    function v=value(name,type)
        ix=find(strcmp({rows.name},name));assert(isscalar(ix),'gpenmpc:TypedMissing','Missing/duplicate current parameter %s',name);
        row=rows(ix);p=row.typed_value;
        assert(isempty(row.read_error)&&p.mav_type==type&&strcmp(p.name,name)&&isfinite(p.decoded),'gpenmpc:TypedInvalid');
        bits=uint32(hex2dec(p.raw_bits_hex));
        if type==9,v=double(typecast(bits,'single'));else,v=double(typecast(bits,'int32'));end
        assert(isequal(v,double(p.decoded)),'gpenmpc:TypedBits','Name/type/raw bits mismatch.');
        used(end+1)=struct('name',name,'mav_type',type,'raw_bits_hex',p.raw_bits_hex,'value',v);
    end
    function v=gains(suffix)
        v=[value(['MC_ROLLRATE_' suffix],9);value(['MC_PITCHRATE_' suffix],9);value(['MC_YAWRATE_' suffix],9)];
    end
    function add(name,val,reason)
        missing(end+1)=struct('field',name,'value',val,'status','HOST_FIXTURE_NOT_LIVE','reason',reason);
    end
end
function r=identity(path)
f=fopen(path,'rb');assert(f>=0);c=onCleanup(@()fclose(f));bytes=fread(f,Inf,'*uint8');
m=java.security.MessageDigest.getInstance('SHA-256');m.update(bytes);
r=struct('path',path,'bytes',numel(bytes),'sha256',upper(reshape(dec2hex(typecast(m.digest(),'uint8'),2).',1,[])));clear c
end
