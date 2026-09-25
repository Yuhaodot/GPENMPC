function [cfg,moduleCfg,meta]=native_position_control_current_config()
% Read the collected 138-value parameter receipt.
receiptPath=gpenmpc_external_path('native_position_readback_receipt');
expectedReceiptSHA='BF31EEF276ADC9756371E5FEF282AC10465056B5710E3A87D64CD422F5F1E0B4';
receiptId=identity(receiptPath);require(strcmpi(receiptId.sha256,expectedReceiptSHA),'Position receipt SHA differs.');
r=jsondecode(fileread(receiptPath));
require(strcmp(r.schema,'M600_NATIVE_POSITION_READONLY_DETAILS_V1'),'Unknown position receipt schema.');
for n={'passed','safety_preflight_passed','all_138_diagnostic_reads_complete','position_details_ready','original_105_semantics_match','COM_closed'}
    requireTrue(r,n{1});
end
require(r.original_name_count==105&&r.additional_name_count==33&&isempty(r.failure),'Incomplete position receipt.');
zeroActions(r);require(islogical(r.live_hil_or_flight_run)&&~r.live_hil_or_flight_run,'Not a read-only receipt.');
priorPath=gpenmpc_external_path('native_lower_loop_readback_receipt');
priorId=identity(priorPath);
require(strcmp(priorId.sha256,'EC504AEB12444D41B88DD0E5411DDAA3B59B513AB9410433736A5B59C47E1D9C'),'Current105 identity changed.');
require(isequal(r.original_105_baseline,priorId),'Recorded original105 binding differs.');
p=jsondecode(fileread(priorPath));requireTrue(p,'passed');requireTrue(p,'original_79_semantics_match');
priorRawId=identity(p.raw_receipt.path);require(isequal(priorRawId,p.raw_receipt),'Original105 raw binding differs.');
require(isequal(priorRawId,r.original_105_raw_baseline),'New receipt original105 raw reference differs.');
priorRaw=jsondecode(fileread(priorRawId.path));
rawId=identity(r.raw_receipt.path);require(isequal(rawId,r.raw_receipt),'Position raw bytes/SHA differ.');
raw=jsondecode(fileread(rawId.path));
require(strcmp(raw.schema,'M600_CANONICAL_SERIAL_READONLY_SAFETY_V1'),'Unknown raw schema.');
for n={'passed','COM_closed','diagnostic_parameters_complete','virtual_output_path_disabled','physical_output_path_disabled'}
    requireTrue(raw,n{1});
end
zeroActions(raw);
require(isempty(raw.failure)&&islogical(raw.armed)&&~raw.armed&&raw.landed_state==1,'Raw board not safely disarmed/on ground.');
require(strcmp(raw.uid,gpenmpc_device_identity('uid'))&&raw.board_version==56&&raw.product_id==56 ...
    &&strcmp(raw.hw_arch,'PX4_FMU_V6C')&&strcmp(raw.commit,'6ea3539157ca358c70a515878b77077af7d4611d') ...
    &&strcmp(raw.flight_custom_version_hex,'000000579153A36E'),'Raw identity differs.');
requireTrue(raw.sd_diagnostics,'passed');
require(isempty(raw.sd_diagnostics.active_fault_log_names)&&~raw.virtual_output_observation.any_nonzero_or_nonfinite,'Crash/output evidence failed.');
require(contains(raw.pwm_out_status,'[pwm_out] not running')&&contains(raw.custom_controller_status,'[gpenmpc_se3_control] not running'),'Output/controller owner not stopped.');
rows=raw.diagnostic_parameter_observations;oldRows=priorRaw.diagnostic_parameter_observations;
oldNames=reshape(string({oldRows.name}),1,[]);allNames=reshape(string({rows.name}),1,[]);
[newNames,newTypes]=additionalNames();
require(numel(oldRows)==105&&numel(unique(oldNames))==105&&numel(rows)==138&&numel(unique(allNames))==138,'Missing or duplicate parameter rows.');
require(isequal(allNames,[oldNames,newNames])&&isequal(allNames,reshape(string(r.requested_names),1,[])),'Selector/order differs.');
for k=1:105
    a=typed(oldRows(k),oldRows(k).typed_value.mav_type);b=typed(rows(k),a.mav_type);
    require(strcmp(a.name,b.name)&&strcmpi(a.raw_bits_hex,b.raw_bits_hex)&&a.decoded==b.decoded,['Original105 changed: ' a.name]);
end
for k=1:33,typed(rows(105+k),newTypes(k));end
require(r.original_105_validation.passed&&r.original_105_validation.matching_rows==105 ...
    &&r.additional_validation.passed&&r.additional_validation.passed_rows==33,'Derived validation disagrees.');
for k=1:33
    a=r.additional_validation.rows(k);b=typed(a.observation,newTypes(k));
    require(a.passed&&strcmp(a.name,newNames(k))&&strcmpi(b.raw_bits_hex,rows(105+k).typed_value.raw_bits_hex),'Additional validation/raw differs.');
end
selected=struct('name',{},'mav_type',{},'raw_bits_hex',{},'value',{});
cfg=struct('parameter_provenance','CURRENT_TYPED_WITH_EXPLICIT_MODULE_PHASE');
cfg.position_p=[take('MPC_XY_P');take('MPC_XY_P');take('MPC_Z_P')];
cfg.velocity_p=[take('MPC_XY_VEL_P_ACC');take('MPC_XY_VEL_P_ACC');take('MPC_Z_VEL_P_ACC')];
cfg.velocity_i=[take('MPC_XY_VEL_I_ACC');take('MPC_XY_VEL_I_ACC');take('MPC_Z_VEL_I_ACC')];
cfg.velocity_d=[take('MPC_XY_VEL_D_ACC');take('MPC_XY_VEL_D_ACC');take('MPC_Z_VEL_D_ACC')];
cfg.velocity_limits_mps=[take('MPC_XY_VEL_MAX');take('MPC_Z_VEL_MAX_UP');take('MPC_Z_VEL_MAX_DN')];
cfg.thrust_limits=[take('MPC_THR_MIN');take('MPC_THR_MAX')];
cfg.horizontal_thrust_margin=take('MPC_THR_XY_MARG');
cfg.tilt_limit_rad=double(single(take('MPC_TILTMAX_AIR'))*(single(pi)/single(180)));
cfg.hover_thrust=take('MPC_THR_HOVER');
decouple=take('MPC_ACC_DECOUPLE',6);require(ismember(decouple,[0,1]),'Invalid acceleration decouple flag.');
cfg.decouple_horizontal_vertical=logical(decouple);
moduleCfg=struct();
for k=1:33,moduleCfg.(char(newNames(k)))=take(char(newNames(k)),newTypes(k));end
moduleCfg.MPC_THR_HOVER=take('MPC_THR_HOVER');
moduleCfg.MPC_USE_HTE=take('MPC_USE_HTE',6);
moduleCfg.COM_SPOOLUP_TIME=take('COM_SPOOLUP_TIME');
require(ismember(moduleCfg.MPC_USE_HTE,[0,1])&&ismember(moduleCfg.COM_THROW_EN,[0,1]),'Unsupported Boolean parameter value.');
require(all(cfg.position_p>=0)&&all(cfg.velocity_p>0)&&all(cfg.velocity_i>=0)&&all(cfg.velocity_d>=0) ...
    &&all(cfg.velocity_limits_mps>=0)&&all(cfg.thrust_limits>=0)&&cfg.thrust_limits(1)<=cfg.thrust_limits(2) ...
    &&cfg.thrust_limits(2)<=1&&cfg.thrust_limits(2)>=.001&&cfg.horizontal_thrust_margin>=0 ...
    &&cfg.horizontal_thrust_margin<=cfg.thrust_limits(2)&&cfg.tilt_limit_rad>=0&&cfg.tilt_limit_rad<pi/2 ...
    &&cfg.hover_thrust>=.05&&cfg.hover_thrust<=.9,'Observed values lie outside the existing kernel API domain.');
sources=r.declaration_only_not_live_values.mechanism_sources;
for k=1:numel(sources),require(isequal(identity(sources(k).path),sources(k)),'Bound mechanism source changed.');end
root=gpenmpc_external_path('px4_source_root');
headerId=identity(fullfile(root,'src','modules','mc_pos_control','MulticopterPositionControl.hpp'));
require(strcmp(headerId.sha256,'0EDC6BF43258D4E4B98531F45FD1CB8E92A5CA2159605450A1F851AC2E6F4828'),'Module phase source header changed.');
meta=struct('schema','NATIVE_POSITION_CURRENT_TYPED_CONFIGURATION_V1', ...
    'current138',receiptId,'current138_raw',rawId,'current105',priorId,'current105_raw',priorRawId, ...
    'current_typed_row_count',138,'original105_recomputed_matches',105,'new_typed_count',33, ...
    'parameter_rows',selected,'mechanism_sources',sources,'module_header',headerId, ...
    'defaults_used',false,'hardware_actions',0,'COM_UDP_actions',0,'MEX_calls',0, ...
    'nominal_cfg_is_not_a_module_runtime_snapshot',true, ...
    'module_phase_contract',struct('explicit_new_instance_effective_vertical_I',0, ...
        'effective_vertical_I_slew_per_s',.30,'no_route_motion_hover_target_I',0, ...
        'I_values_basis','Bound module source header and implementation.', ...
        'requires_effective_vertical_I_each_step',true,'native_LAND_target_I','nominal MPC_Z_VEL_I_ACC', ...
        'terminal_negative_integral_update_inhibit',false,'accumulated_I_observed',false), ...
    'runtime_limit_contract','Typed flight limits initialize cfg; takeoff updates minimum thrust, ramp speed and slew-limited tilt through setters on each step.', ...
    'runtime_unobserved',{{'Module/filter init time, sample interval mean and filter histories', ...
        'Takeoff phase/ramp progress and ground_contact uORB state', ...
        'HTE message validity/time/value and accumulated position-controller Iz', ...
        'Route/payload topic freshness/absence and position-enable/source reset counters'}}, ...
    'claim','Current parameter configuration.');
    function v=take(name,type)
        if nargin<2,type=9;end
        j=find(strcmp({rows.name},name));require(isscalar(j),['Missing exact parameter ' name]);
        q=typed(rows(j),type);v=q.decoded;
        if ~any(strcmp({selected.name},name))
            selected(end+1)=struct('name',name,'mav_type',type,'raw_bits_hex',q.raw_bits_hex,'value',v);
        end
    end
end
function [n,t]=additionalNames()
n=["MPC_XY_P","MPC_Z_P","MPC_XY_VEL_P_ACC","MPC_XY_VEL_I_ACC","MPC_XY_VEL_D_ACC", ...
    "MPC_Z_VEL_P_ACC","MPC_Z_VEL_I_ACC","MPC_Z_VEL_D_ACC","MPC_XY_VEL_MAX","MPC_Z_VEL_MAX_UP","MPC_Z_VEL_MAX_DN", ...
    "MPC_TILTMAX_AIR","MPC_TILTMAX_LND","MPC_THR_MIN","MPC_THR_MAX","MPC_THR_XY_MARG","MPC_ACC_DECOUPLE", ...
    "MPC_VEL_LP","MPC_VEL_NF_FRQ","MPC_VEL_NF_BW","MPC_VELD_LP","MPC_TKO_RAMP_T","COM_THROW_EN","MPC_LAND_SPEED", ...
    "SYS_VEHICLE_RESP","MPC_XY_VEL_ALL","MPC_Z_VEL_ALL","HTE_HT_NOISE","HTE_ACC_GATE","HTE_HT_ERR_INIT","HTE_THR_RANGE","HTE_VXY_THR","HTE_VZ_THR"];
t=9*ones(1,33);t([17,23])=6;
end
function p=typed(row,type)
require(isscalar(row)&&isfield(row,'typed_value')&&isempty(row.read_error)&&row.write_count==0,'Invalid read-only typed row.');
p=row.typed_value;require(ischar(p.name)&&strcmp(row.name,p.name)&&ismember(type,[6,9])&&p.mav_type==type,'Wrong parameter name/type.');
require(ischar(p.raw_bits_hex)&&~isempty(regexp(p.raw_bits_hex,'^[0-9A-Fa-f]{8}$','once')),'Malformed parameter raw bits.');
bits=uint32(hex2dec(p.raw_bits_hex));
if type==9,value=double(typecast(bits,'single'));else,value=double(typecast(bits,'int32'));end
require(isnumeric(p.decoded)&&isscalar(p.decoded)&&isfinite(value)&&isequal(value,double(p.decoded)),'Parameter raw bits/value disagreement.');
end
function requireTrue(s,name)
require(isfield(s,name)&&islogical(s.(name))&&isscalar(s.(name))&&s.(name),['Missing/false evidence ' name]);
end
function zeroActions(s)
for n={'parameter_writes','mapping_writes','arm_disarm_mode_requests','flash_reboot_count','physical_output_actions'}
    require(isfield(s,n{1})&&isnumeric(s.(n{1}))&&isscalar(s.(n{1}))&&s.(n{1})==0,['Unknown/nonzero action ' n{1}]);
end
end
function require(value,message),assert(isscalar(value)&&value,'gpenmpc:CurrentPositionConfig','%s',message);end
function r=identity(path)
path=char(path);f=fopen(path,'rb');require(f>=0,['Missing current input ' path]);c=onCleanup(@()fclose(f));
b=fread(f,Inf,'*uint8');m=java.security.MessageDigest.getInstance('SHA-256');m.update(b);
r=struct('path',path,'bytes',numel(b),'sha256',upper(reshape(dec2hex(typecast(m.digest(),'uint8'),2).',1,[])));clear c
end
