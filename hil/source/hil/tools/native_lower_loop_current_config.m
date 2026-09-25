function [cfg,meta]=native_lower_loop_current_config()
% Read typed lower-loop parameters and initial state.
[cfg,oldMeta]=native_lower_loop_explicit_fixture();
parentPath=gpenmpc_external_path('native_control_readback_receipt');
detailPath=gpenmpc_external_path('native_lower_loop_readback_receipt');
parentId=identity(parentPath);detailId=identity(detailPath);
require(strcmp(parentId.sha256,'F0C96E672086D8AB4F385F8FD3CCF085DF4E91B25D336D263AF23A3522404D95'),'Parent79 SHA changed.');
require(strcmp(detailId.sha256,'EC504AEB12444D41B88DD0E5411DDAA3B59B513AB9410433736A5B59C47E1D9C'),'Current105 SHA changed.');
parent=jsondecode(fileread(parentPath));detail=jsondecode(fileread(detailPath));
require(strcmp(detail.schema,'M600_NATIVE_LOWER_LOOP_READONLY_DETAILS_V1'),'Unknown detailed receipt schema.');
for name={'passed','safety_preflight_passed','all_105_diagnostic_reads_complete','lower_loop_details_ready','original_79_semantics_match','COM_closed'}
    truth(detail,name{1});
end
require(detail.original_name_count==79&&detail.additional_name_count==26&&isempty(detail.failure),'Unexpected complete-read denominator/failure.');
zeroActions(detail);require(islogical(detail.live_hil_or_flight_run)&&~detail.live_hil_or_flight_run,'Not a read-only collection.');
require(isequal(detail.original_79_baseline,parentId),'Detailed receipt parent path/bytes/SHA mismatch.');
rawId=identity(detail.raw_receipt.path);require(isequal(rawId,detail.raw_receipt),'Raw receipt path/bytes/SHA mismatch.');
raw=jsondecode(fileread(rawId.path));
require(strcmp(raw.schema,'M600_CANONICAL_SERIAL_READONLY_SAFETY_V1'),'Unknown raw safety schema.');
for name={'passed','COM_closed','diagnostic_parameters_complete','virtual_output_path_disabled','physical_output_path_disabled'}
    truth(raw,name{1});
end
zeroActions(raw);require(isempty(raw.failure)&&islogical(raw.armed)&&~raw.armed&&raw.landed_state==1,'Raw final state not disarmed/on-ground.');
require(strcmp(raw.uid,gpenmpc_device_identity('uid'))&&raw.board_version==56&&raw.product_id==56 ...
    &&strcmp(raw.hw_arch,'PX4_FMU_V6C')&&strcmp(raw.commit,'6ea3539157ca358c70a515878b77077af7d4611d') ...
    &&strcmp(raw.flight_custom_version_hex,'000000579153A36E'),'Raw board identity mismatch.');
require(contains(raw.pwm_out_status,'not running')&&contains(raw.custom_controller_status,'not running'),'Output/control owner not stopped.');
truth(raw.sd_diagnostics,'passed');
require(isempty(raw.sd_diagnostics.active_fault_log_names)&&~raw.virtual_output_observation.any_nonzero_or_nonfinite,'Raw diagnostic/output safety evidence failed.');
rows=raw.diagnostic_parameter_observations;prior=parent.diagnostic_parameter_observations;
require(numel(rows)==105&&numel(prior)==79&&numel(unique({rows.name}))==105&&numel(unique({prior.name}))==79,'Missing/duplicate raw parameter observations.');
require(isequal(sort(string(detail.requested_names(:))),sort(string({rows.name}).')),'Requested/raw parameter set mismatch.');
for k=1:numel(rows),typed(rows(k),rows(k).typed_value.mav_type);end
for k=1:numel(prior)
    p=typed(prior(k),prior(k).typed_value.mav_type);j=find(strcmp({rows.name},prior(k).name));
    require(isscalar(j),'Parent parameter missing.');q=typed(rows(j),p.mav_type);
    require(strcmp(q.name,p.name)&&strcmp(q.raw_bits_hex,p.raw_bits_hex)&&q.decoded==p.decoded,'Original79 semantics changed.');
end
require(detail.original_79_validation.passed&&detail.original_79_validation.matching_rows==79 ...
    &&detail.original_79_validation.expected_rows==79&&all([detail.original_79_validation.rows.passed]),'Parent comparison receipt failed.');
validation=detail.lower_loop_validation;
require(validation.passed&&numel(validation.rows)==26&&all([validation.rows.passed]),'Lower-loop typed validation failed.');
for k=1:numel(validation.rows)
    v=validation.rows(k);p=typed(v.observation,v.required_mav_type);j=find(strcmp({rows.name},v.name));
    require(isscalar(j)&&strcmp(v.name,p.name),'Lower-loop validation name is ambiguous.');
    q=typed(rows(j),v.required_mav_type);
    require(strcmp(q.raw_bits_hex,p.raw_bits_hex)&&q.decoded==p.decoded,'Summary/raw typed disagreement.');
end
selected=struct('name',{},'mav_type',{},'raw_bits_hex',{},'value',{});
cfg.yaw_weight=take('MC_YAW_WEIGHT',9);
degrees=[take('MC_ROLLRATE_MAX',9);take('MC_PITCHRATE_MAX',9);take('MC_YAWRATE_MAX',9)];
% Matches math::radians<float>: degrees * (float(MATH_PI)/float(180)).
cfg.rate_limits_rad_s=double(single(degrees).*(single(pi)/single(180)));
cfg.integral_limits=[take('MC_RR_INT_LIM',9);take('MC_PR_INT_LIM',9);take('MC_YR_INT_LIM',9)];
cfg.yaw_torque_cutoff_hz=take('MC_YAW_TQ_CUTOFF',9);
battery=take('MC_BAT_SCALE_EN',6);require(ismember(battery,[0,1]),'Unsupported battery flag.');
cfg.battery_scale_enabled=logical(battery);
for k=0:5,cfg.slew_limits_s(k+1,1)=take(sprintf('CA_R%d_SLEW',k),9);end
cfg.initial_motor_commands=zeros(6,1); % Initial motor-command state.
cfg.parameter_provenance='CURRENT_TYPED_ALL_REQUIRED';
require(cfg.yaw_weight>=0&&cfg.yaw_weight<=1&&all(cfg.rate_limits_rad_s>0) ...
    &&all(cfg.integral_limits>0)&&cfg.yaw_torque_cutoff_hz>=0&&all(cfg.slew_limits_s>=0),'Kernel parameter domain invalid.');
newNames=setdiff({rows.name},{prior.name},'stable');
require(numel(newNames)==26&&numel(selected)==15,'Missing exact new parameter set.');
sensorNames=setdiff(newNames,{selected.name},'stable');sensorRows=rows(ismember({rows.name},sensorNames));
require(numel(sensorRows)==11,'Unexpected sensor-parameter remainder.');
meta=struct('schema','PX4_NATIVE_LOWER_LOOP_CURRENT_TYPED_CONFIGURATION_V1', ...
    'original79',parentId,'current105',detailId,'current105_raw',rawId, ...
    'original79_semantics_recomputed_match',true,'original79_matching_count',79, ...
    'current105_typed_rows_verified',105,'lower_kernel_current_parameter_rows',[oldMeta.current_typed_fields(:);selected(:)], ...
    'new_lower_kernel_parameter_count',15,'current_lower_kernel_parameter_count',34, ...
    'rate_limit_conversion',struct('input_degrees_per_s',degrees,'output_radians_per_s',cfg.rate_limits_rad_s, ...
        'arithmetic','PX4 math::radians<float>, float PI/180 then float product; converted to double only at MEX input boundary'), ...
    'geometry_receipt',oldMeta.geometry_receipt,'geometry_basis',oldMeta.geometry_basis, ...
    'geometry_is_corrected_HOST_candidate_not_current_board_state',true, ...
    'observed_sensor_parameters_not_reconstructed_in_this_kernel',sensorRows, ...
    'initial_state_fixture',struct('field','initial_motor_commands','value',zeros(6,1), ...
        'status','EXPLICIT_HOST_INITIAL_STATE_NOT_LIVE_PARAMETER','caller_override_must_be_recorded',true), ...
    'kernel_parameter_completeness','ALL_PARAMETERS_REQUIRED_BY_CURRENT_NATIVE_ATTITUDE_RATE_ALLOCATION_MEX_ARE_FROM_CURRENT_TYPED_RECEIPTS', ...
    'sensor_sample_rate_filter_state_and_timing_reconstruction_known',false, ...
    'hardware_actions',0,'flight_admission',false, ...
    'limitations',{{'Sensor/filter parameter values are observed, but filter states, sample rate, driver/work-queue timing and EKF behavior are not reconstructed here.', ...
        'The corrected six-channel geometry is an explicit HOST candidate already used in HOVER004; the current read-only board receipt contains restored original geometry.', ...
        'Initial allocator commands and SIL scenario states remain explicit initialization fixtures.', ...
        'Typed lower-loop configuration for HOST kernel tests.'}});
    function v=take(name,type)
        j=find(strcmp({rows.name},name));require(isscalar(j),['Missing exact required parameter ' name]);
        p=typed(rows(j),type);v=p.decoded;
        selected(end+1)=struct('name',name,'mav_type',type,'raw_bits_hex',p.raw_bits_hex,'value',v);
    end
end
function p=typed(row,type)
require(isscalar(row)&&isfield(row,'typed_value')&&isempty(row.read_error)&&row.write_count==0,'Invalid read-only typed row.');
p=row.typed_value;require(ischar(p.name)&&strcmp(row.name,p.name)&&ismember(type,[6,9])&&p.mav_type==type,'Wrong name or MAVLink type.');
require(ischar(p.raw_bits_hex)&&~isempty(regexp(p.raw_bits_hex,'^[0-9A-F]{8}$','once')),'Malformed raw bits.');
bits=uint32(hex2dec(p.raw_bits_hex));
if type==9,value=double(typecast(bits,'single'));else,value=double(typecast(bits,'int32'));end
require(isnumeric(p.decoded)&&isscalar(p.decoded)&&isfinite(value)&&isequal(value,double(p.decoded)),'Raw bits/value disagreement.');
end
function truth(s,name)
require(isfield(s,name)&&islogical(s.(name))&&isscalar(s.(name))&&s.(name),['Missing/false logical evidence: ' name]);
end
function zeroActions(s)
for name={'parameter_writes','mapping_writes','arm_disarm_mode_requests','flash_reboot_count','physical_output_actions'}
    require(isfield(s,name{1})&&isnumeric(s.(name{1}))&&isscalar(s.(name{1}))&&s.(name{1})==0,['Nonzero/unknown read-only action: ' name{1}]);
end
end
function require(value,message),assert(isscalar(value)&&value,'gpenmpc:CurrentNativeConfig','%s',message);end
function r=identity(path)
path=char(path);f=fopen(path,'rb');assert(f>=0,'gpenmpc:MissingCurrentInput','Missing %s',path);c=onCleanup(@()fclose(f));
bytes=fread(f,Inf,'*uint8');m=java.security.MessageDigest.getInstance('SHA-256');m.update(bytes);
r=struct('path',path,'bytes',numel(bytes),'sha256',upper(reshape(dec2hex(typecast(m.digest(),'uint8'),2).',1,[])));clear c
end
