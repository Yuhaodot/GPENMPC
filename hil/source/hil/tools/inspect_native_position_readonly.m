function receipt=inspect_native_position_readonly(receiptFile,live)
% Read 33 additional selectors through the existing COM owner.
% Default false runs host-only tests.
arguments
    receiptFile (1,1) string
    live (1,1) logical = false
end
assert(~ismissing(receiptFile)&&strlength(receiptFile)>0,'m600check:PositionOutputPath');
rawFile=receiptFile+".preflight.json";
assert(~isfile(receiptFile)&&~isfile(rawFile)&&~isfile(rawFile+".mat"), ...
    'm600check:PreflightOutputExists','No existing receipt is overwritten.');
here=fileparts(mfilename('fullpath'));oldPath=path;pathGuard=onCleanup(@()path(oldPath)); %#ok<NASGU>
addpath(here);
base=inspect_native_lower_loop_readonly(rawFile,false);
baseline=load105(reshape(string(base.requested_names),1,[]));
[names,types,groups]=positionNames();allNames=[baseline.names,names];
assert(numel(allNames)==138&&numel(unique(allNames))==138);
declaration=sourceDeclaration(names,types,groups);
expected=struct('uid',gpenmpc_device_identity('uid'),'board_version',56, ...
    'commit','6ea3539157ca358c70a515878b77077af7d4611d','diagnostic_parameter_names',allNames);
if ~live
    receipt=hostTests(base,baseline,names,types,allNames,declaration,expected,rawFile);
    assert(~isfile(receiptFile)&&~isfile(rawFile)&&~isfile(rawFile+".mat"), ...
        'm600check:PositionHostUnexpectedWrite');
    return
end
% The live owner closes COM before returning, including on read failure.
% Retain its JSON and MAT separately from this derived result.
raw=collect(true,@m600_canonical_serial_preflight,rawFile,expected);
result=adjudicate(raw,baseline,names,types,allNames);
receipt=struct('schema','M600_NATIVE_POSITION_READONLY_DETAILS_V1','passed',result.passed, ...
    'safety_preflight_passed',result.safety.passed,'all_138_diagnostic_reads_complete',result.complete, ...
    'position_details_ready',result.additional.passed,'original_105_semantics_match',result.original.passed, ...
    'original_105_validation',result.original,'original_105_baseline',baseline.source, ...
    'original_105_raw_baseline',baseline.raw_source,'additional_validation',result.additional, ...
    'safety_validation',result.safety,'requested_names',allNames,'original_name_count',105, ...
    'additional_name_count',33,'declaration_only_not_live_values',declaration, ...
    'raw_receipt',identity(rawFile),'raw_mat_path',rawFile+".mat", ...
    'COM_open_attempts',get(raw,'COM_open_attempts',[]),'COM_closed',get(raw,'COM_closed',false), ...
    'parameter_writes',get(raw,'parameter_writes',[]),'mapping_writes',get(raw,'mapping_writes',[]), ...
    'arm_disarm_mode_requests',get(raw,'arm_disarm_mode_requests',[]), ...
    'flash_reboot_count',get(raw,'flash_reboot_count',[]),'physical_output_actions',get(raw,'physical_output_actions',[]), ...
    'profile_current_distinction',profileClaim(), ...
    'live_hil_or_flight_run',false,'failure',result.failure);
f=fopen(receiptFile,'w','n','UTF-8');assert(f>=0);g=onCleanup(@()fclose(f)); %#ok<NASGU>
fprintf(f,'%s\n',jsonencode(receipt,PrettyPrint=true));
end

function [names,types,groups]=positionNames()
names=["MPC_XY_P","MPC_Z_P","MPC_XY_VEL_P_ACC","MPC_XY_VEL_I_ACC","MPC_XY_VEL_D_ACC", ...
    "MPC_Z_VEL_P_ACC","MPC_Z_VEL_I_ACC","MPC_Z_VEL_D_ACC", ...
    "MPC_XY_VEL_MAX","MPC_Z_VEL_MAX_UP","MPC_Z_VEL_MAX_DN", ...
    "MPC_TILTMAX_AIR","MPC_TILTMAX_LND","MPC_THR_MIN","MPC_THR_MAX","MPC_THR_XY_MARG", ...
    "MPC_ACC_DECOUPLE","MPC_VEL_LP","MPC_VEL_NF_FRQ","MPC_VEL_NF_BW","MPC_VELD_LP", ...
    "MPC_TKO_RAMP_T","COM_THROW_EN","MPC_LAND_SPEED", ...
    "SYS_VEHICLE_RESP","MPC_XY_VEL_ALL","MPC_Z_VEL_ALL", ...
    "HTE_HT_NOISE","HTE_ACC_GATE","HTE_HT_ERR_INIT","HTE_THR_RANGE","HTE_VXY_THR","HTE_VZ_THR"];
types=9*ones(1,33);types([17,23])=6;
groups=[repmat("POSITION_TAKEOFF_FAILSAFE",1,24),repmat("PARAMETER_AUTO_CONFIG",1,3),repmat("HTE_ESTIMATOR",1,6)];
assert(numel(names)==33&&numel(unique(names))==33&&all(strlength(names)<=16)&& ...
    all(~cellfun(@isempty,regexp(cellstr(names),'^[A-Z][A-Z0-9_]*$','once'))));
end

function d=sourceDeclaration(names,types,groups)
root=gpenmpc_external_path('px4_source_root');
path=fullfile(root,'build','px4_fmu-v6c_default','parameters.json');
p=jsondecode(fileread(path));actual=zeros(1,numel(names));
parameters=p.parameters;
if isstruct(parameters),parameters=num2cell(parameters);end
assert(iscell(parameters)&&all(cellfun(@(x)isstruct(x)&&isscalar(x)&&all(isfield(x,{'name','type'})),parameters)), ...
    'm600check:PositionDeclaration','Malformed declared parameter catalogue.');
catalogueNames=string(cellfun(@(x)x.name,parameters,'UniformOutput',false));
for k=1:numel(names)
    at=find(catalogueNames==names(k));assert(isscalar(at),'m600check:PositionDeclaration');
    if strcmp(parameters{at}.type,'Float'),actual(k)=9;
    elseif strcmp(parameters{at}.type,'Int32'),actual(k)=6;
    else,error('m600check:PositionDeclaration','Unknown declared type.');end
end
assert(isequal(actual,types),'m600check:PositionDeclaration','Declared type mismatch.');
paths={fullfile(root,'src','modules','mc_pos_control','MulticopterPositionControl.cpp'), ...
    fullfile(root,'src','modules','mc_pos_control','PositionControl','PositionControl.cpp'), ...
    fullfile(root,'src','modules','mc_hover_thrust_estimator','MulticopterHoverThrustEstimator.cpp')};
hashes={'D568CC47CE984FEE096A5F5E20B625C45A745AF5B25F2D011C8B8097A927C243', ...
    '7AAC7280D989CE584BB2E7FE179D91C319638CB312C2B2E7F7934492A5EF8A82', ...
    '85AFE70E8BFE614E47D902B182BBB11E21C0BF0FFF526DB0721CA354E988693C'};
sources=struct([]);
for k=1:numel(paths)
    v=identity(paths{k});assert(strcmp(v.sha256,hashes{k}),'m600check:PositionSourceChanged');
    if isempty(sources),sources=v;else,sources(end+1)=v;end %#ok<AGROW>
end
d=struct('source',identity(path),'names',names,'mav_types',types,'groups',groups, ...
    'declaration_passed',true,'default_values_used',false,'mechanism_sources',sources);
end

function b=load105(names)
path=gpenmpc_external_path('native_lower_loop_readback_receipt');
source=identity(path);
assert(strcmp(source.sha256,'EC504AEB12444D41B88DD0E5411DDAA3B59B513AB9410433736A5B59C47E1D9C'), ...
    'm600check:Original105Identity','Current105 reference changed.');
r=jsondecode(fileread(path));
assert(r.passed&&r.original_79_semantics_match&&r.all_105_diagnostic_reads_complete&&r.lower_loop_details_ready&&r.COM_closed);
rawId=identity(r.raw_receipt.path);assert(isequal(rawId,r.raw_receipt),'m600check:Original105Identity');
raw=jsondecode(fileread(rawId.path));safe=safety(raw);assert(safe.passed,'m600check:Original105Identity','Unsafe original105 receipt.');
rows=raw.diagnostic_parameter_observations;
assert(numel(names)==105&&numel(rows)==105&&isequal(reshape(string({rows.name}),1,[]),names), ...
    'm600check:Original105Identity','Original105 selector/order differs.');
types=reshape(arrayfun(@(x)double(x.typed_value.mav_type),rows),1,[]);
assert(all(ismember(types,[6,9])));checked=validateRows(rows,names,types);assert(checked.passed);
b=struct('source',source,'raw_source',rawId,'names',names,'types',types,'rows',rows);
end

function out=collect(live,owner,file,expected)
out=[];if live,out=owner(file,expected);end
end

function r=adjudicate(raw,b,names,types,allNames)
rows=get(raw,'diagnostic_parameter_observations',struct([]));
r.safety=safety(raw);r.additional=validateRows(rows,names,types);r.original=compare105(rows,b);
exact=isstruct(rows)&&isfield(rows,'name')&&numel(rows)==138&& ...
    numel(unique(string({rows.name})))==138&& ...
    isequal(reshape(string({rows.name}),1,[]),allNames);
r.complete=truth(raw,'diagnostic_parameters_complete')&&exact;
r.passed=r.safety.passed&&r.additional.passed&&r.original.passed&&r.complete;
parts=strings(0,1);
if ~r.safety.passed,parts(end+1)="SAFETY: "+r.safety.failure;end
if ~r.additional.passed,parts(end+1)="ADDITIONAL33: "+r.additional.failure;end
if ~r.original.passed,parts(end+1)="ORIGINAL105: "+r.original.failure;end
if ~r.complete,parts(end+1)="138_EXACT_COMPLETE_DIAGNOSTIC_SET_NOT_PROVED";end
r.failure=strjoin(parts,' | ');
end

function r=safety(raw)
checks=struct('name',{},'passed',{});
add('base_preflight_pass',truth(raw,'passed'));
add('exact_schema',strcmp(string(get(raw,'schema','')),'M600_CANONICAL_SERIAL_READONLY_SAFETY_V1'));
add('empty_primary_failure',strlength(string(get(raw,'failure','UNKNOWN')))==0);
add('COM_closed',truth(raw,'COM_closed'));
add('one_COM_open_attempt',isequal(get(raw,'COM_open_attempts',[]),1));
add('UID',strcmp(string(get(raw,'uid','')),gpenmpc_device_identity('uid')));
add('board_FMUv6C',isequal(get(raw,'board_version',[]),56)&&isequal(get(raw,'product_id',[]),56)&& ...
    strcmp(string(get(raw,'hw_arch','')),'PX4_FMU_V6C'));
add('source_commit',strcmp(string(get(raw,'commit','')),'6ea3539157ca358c70a515878b77077af7d4611d')&& ...
    strcmp(string(get(raw,'flight_custom_version_hex','')),'000000579153A36E'));
add('fresh_disarmed_landed',isfield(raw,'armed')&&islogical(raw.armed)&&isscalar(raw.armed)&&~raw.armed&&isequal(get(raw,'landed_state',[]),1));
add('output_paths_disabled',truth(raw,'virtual_output_path_disabled')&&truth(raw,'physical_output_path_disabled'));
output=get(raw,'virtual_output_observation',struct());
add('no_observed_nonzero_or_nonfinite_output',isfield(output,'any_nonzero_or_nonfinite')&& ...
    islogical(output.any_nonzero_or_nonfinite)&&isscalar(output.any_nonzero_or_nonfinite)&&~output.any_nonzero_or_nonfinite);
add('pwm_out_stopped',~isempty(regexp(char(string(get(raw,'pwm_out_status',''))),'\[pwm_out\]\s+not running','once')));
add('custom_controller_stopped',~isempty(regexp(char(string(get(raw,'custom_controller_status',''))),'\[gpenmpc_se3_control\]\s+not running','once')));
sd=get(raw,'sd_diagnostics',struct());
add('SD_no_active_faults',truth(sd,'passed')&&isfield(sd,'active_fault_log_names')&&isempty(sd.active_fault_log_names));
for n={'parameter_writes','mapping_writes','arm_disarm_mode_requests','flash_reboot_count','physical_output_actions'}
    add(['zero_' n{1}],isequal(get(raw,n{1},[]),0));
end
% Verify each of the 29 baseline profile values.
guardNames=["SYS_HITL","SYS_AUTOSTART","MAV_TYPE","CA_ROTOR_COUNT","RA_CTRL_MODE", ...
    compose("HIL_ACT_FUNC%d",1:16),compose("PWM_MAIN_FUNC%d",1:8)];
guardValues=[1,6001,13,6,0,zeros(1,24)];p=get(raw,'parameters',struct([]));profile=true;
for k=1:29
    if ~isstruct(p)||~isfield(p,'name'),profile=false;break;end
    at=find(string({p.name})==guardNames(k));
    if ~isscalar(at),profile=false;continue;end
    row=struct('name',char(guardNames(k)),'typed_value',p(at),'read_error','','write_count',0);
    t=validateRows(row,guardNames(k),6);profile=profile&&t.passed;
    if t.passed,profile=profile&&p(at).decoded==guardValues(k);end
end
add('29_exact_profile_typed_zero_mappings',profile);
bad=string({checks(~[checks.passed]).name});
r=struct('passed',all([checks.passed]),'checks',checks,'failure',strjoin(bad,' | '));
    function add(name,v),checks(end+1)=struct('name',name,'passed',isscalar(v)&&logical(v));end %#ok<AGROW>
end

function r=validateRows(rows,names,types)
entries=repmat(struct('name','','required_mav_type',0,'passed',false,'failure','','observation',struct()),1,numel(names));
for k=1:numel(names)
    entries(k).name=char(names(k));entries(k).required_mav_type=types(k);
    try
        assert(isstruct(rows)&&isfield(rows,'name'),'m600check:PositionTypedMissing');
        at=find(string({rows.name})==names(k));assert(isscalar(at),'m600check:PositionTypedMissing');
        row=rows(at);entries(k).observation=row;
        assert(isfield(row,'read_error')&&strlength(string(row.read_error))==0&&isequal(get(row,'write_count',[]),0),'m600check:PositionReadFailed');
        v=row.typed_value;assert(isscalar(v)&&isstruct(v)&&all(isfield(v,{'name','mav_type','raw_bits_hex','decoded'})),'m600check:PositionTypedSchema');
        assert(strcmp(string(v.name),names(k))&&isequal(double(v.mav_type),types(k)),'m600check:PositionTypedType');
        bits=char(string(v.raw_bits_hex));assert(~isempty(regexp(bits,'^[0-9A-Fa-f]{8}$','once')),'m600check:PositionTypedBits');
        raw=uint32(hex2dec(bits));
        if types(k)==9,x=double(typecast(raw,'single'));else,x=double(typecast(raw,'int32'));end
        assert(isfinite(x)&&isnumeric(v.decoded)&&isscalar(v.decoded)&&isfinite(v.decoded)&&isequal(double(v.decoded),x),'m600check:PositionTypedDecoded');
        entries(k).passed=true;
    catch e,entries(k).failure=[e.identifier ': ' e.message];end
end
bad=string({entries(~[entries.passed]).name});
r=struct('passed',all([entries.passed]),'expected_rows',numel(names),'passed_rows',sum([entries.passed]),'rows',entries,'failure',strjoin(bad,' | '));
end

function r=compare105(rows,b)
v=validateRows(rows,b.names,b.types);mismatches=struct('name',{},'expected_type',{},'expected_bits',{},'observed',{},'failure',{});matched=0;
for k=1:105
    expected=b.rows(k).typed_value;ok=v.rows(k).passed;observed=struct();failure=v.rows(k).failure;
    if ok
        observed=v.rows(k).observation.typed_value;
        ok=strcmpi(string(observed.raw_bits_hex),string(expected.raw_bits_hex))&&observed.mav_type==expected.mav_type;
        if ~ok,failure='CURRENT_TYPED_VALUE_DIFFERS_FROM_ORIGINAL105__NO_RESTORE_OR_WRITE';end
    end
    if ok,matched=matched+1;else
        mismatches(end+1)=struct('name',char(b.names(k)),'expected_type',expected.mav_type, ...
            'expected_bits',expected.raw_bits_hex,'observed',observed,'failure',failure); %#ok<AGROW>
    end
end
r=struct('passed',matched==105,'expected_rows',105,'matching_rows',matched,'mismatches',mismatches, ...
    'failure',strjoin(string({mismatches.name}),' | '),'restore_attempts',0,'parameter_writes',0);
end

function r=hostTests(base,b,names,types,allNames,declaration,expected,file)
tests=struct('name',{},'passed',{});ownerCalls=0;
add('inherited_lower_loop_host_tests',base.passed&&base.case_count==22&&base.COM_open==0);
add('138_exact_unique_names',numel(allNames)==138&&numel(unique(allNames))==138&&isequal(allNames(1:105),b.names));
add('33_declarations_no_defaults',declaration.declaration_passed&&~declaration.default_values_used&&numel(names)==33);
prior=compare105(b.rows,b);add('original105_self_match',prior.passed&&prior.matching_rows==105);
raw=jsondecode(fileread(b.raw_source.path));mock=repmat(b.rows(1),1,33);
for k=1:33
    if types(k)==9,bits='3E800000';value=.25;else,bits='00000001';value=1;end
    mock(k).name=char(names(k));mock(k).typed_value=struct('name',char(names(k)),'mav_type',types(k), ...
        'raw_float',typecast(uint32(hex2dec(bits)),'single'),'raw_bits_hex',bits,'decoded',value);
    mock(k).read_utc='HOST_SYNTHETIC_NOT_LIVE';mock(k).read_error='';mock(k).write_count=0;
end
raw.diagnostic_parameter_observations=[reshape(b.rows,1,[]),mock];
check=adjudicate(raw,b,names,types,allNames);add('synthetic138_complete_positive',check.passed&&check.original.matching_rows==105&&check.additional.passed_rows==33);
bad=raw;bad.diagnostic_parameter_observations(106)=[];negative('missing_new_parameter',bad);
bad=raw;bad.diagnostic_parameter_observations(end+1)=bad.diagnostic_parameter_observations(106);negative('duplicate_parameter',bad);
bad=raw;bad.diagnostic_parameter_observations(106).typed_value.mav_type=6;negative('wrong_REAL32_type',bad);
bad=raw;bad.diagnostic_parameter_observations(122).typed_value.mav_type=9;negative('wrong_INT32_type',bad);
bad=raw;bad.diagnostic_parameter_observations(106).typed_value.raw_bits_hex='NOT_BITS';negative('malformed_bits',bad);
bad=raw;bad.diagnostic_parameter_observations(106).typed_value.decoded=.5;negative('bits_value_disagree',bad);
bad=raw;bad.diagnostic_parameter_observations(106).typed_value.raw_bits_hex='7FC00000';bad.diagnostic_parameter_observations(106).typed_value.decoded=NaN;negative('NaN_no_default_fill',bad);
bad=raw;bad.diagnostic_parameter_observations(106).read_error='READ_TIMEOUT';negative('new_parameter_timeout',bad);
bad=raw;bad.diagnostic_parameter_observations(106).write_count=1;negative('unexpected_parameter_write',bad);
bad=raw;bad.diagnostic_parameter_observations(1).typed_value.raw_bits_hex='00000063';bad.diagnostic_parameter_observations(1).typed_value.decoded=99;negative('original105_drift_no_restore',bad);
bad=raw;bad.diagnostic_parameter_observations(1).typed_value.mav_type=9;negative('original105_type_drift',bad);
bad=raw;bad.diagnostic_parameter_observations(1)=[];negative('original105_missing',bad);
bad=raw;bad.diagnostic_parameter_observations([106,107])=bad.diagnostic_parameter_observations([107,106]);negative('selector_order_changed',bad);
bad=raw;bad.diagnostic_parameters_complete=false;negative('aggregate_incomplete',bad);
bad=raw;bad.COM_closed=false;negative('COM_not_closed',bad);
bad=raw;bad.uid='OTHER_BOARD';negative('UID_drift',bad);
bad=raw;bad.commit='UNKNOWN';negative('firmware_source_identity_unknown',bad);
bad=raw;bad.armed=true;negative('armed_guard',bad);
bad=raw;bad.landed_state=2;negative('landed_guard',bad);
bad=raw;bad.parameters(6).decoded=1;bad.parameters(6).raw_bits_hex='00000001';negative('mapping_guard_even_aggregate_true',bad);
bad=raw;bad.virtual_output_observation.any_nonzero_or_nonfinite=true;negative('reported_output_guard',bad);
bad=raw;bad.pwm_out_status='INFO [pwm_out] running';negative('physical_PWM_guard',bad);
bad=raw;bad.sd_diagnostics.passed=false;bad.sd_diagnostics.active_fault_log_names={'fault_NEW.log'};negative('new_crash_guard',bad);
bad=raw;bad.custom_controller_status='INFO [gpenmpc_se3_control] running';negative('custom_publisher_guard',bad);
bad=raw;bad.parameter_writes=1;negative('aggregate_write_attempt_guard',bad);
bad=rmfield(raw,'virtual_output_observation');negative('unknown_output_evidence',bad);
out=collect(false,@fakeOwner,file,expected);add('live_false_never_invokes_owner',isempty(out)&&ownerCalls==0);
out=collect(true,@fakeOwner,file,expected);add('injected_single_owner_called_once',ownerCalls==1&&out.fixture&&out.selector_count==138);
add('false_entry_did_not_create_raw',~isfile(file)&&~isfile(file+".mat"));
assert(all([tests.passed]),'m600check:PositionHostTests','HOST-only tests failed.');
r=struct('schema','HOST_NATIVE_POSITION_READONLY_ENTRY_TESTS_V1','passed',true,'case_count',numel(tests), ...
    'cases_passed',sum([tests.passed]),'tests',tests,'requested_names',allNames,'additional_names',names, ...
    'additional_types',types,'source_declaration',declaration,'original_105_baseline',b.source, ...
    'inherited_lower_loop_case_count',base.case_count,'inherited_selector_negative_count',base.inherited_selector_negative_cases, ...
    'injected_fake_owner_calls',ownerCalls,'real_owner_calls',0,'COM_open',0,'board_access',0,'parameter_writes',0, ...
    'mapping_writes',0,'arm_disarm_mode_requests',0,'reboot_flash',0,'output_actions',0,'files_written',0, ...
    'profile_current_distinction',profileClaim());
    function add(name,value),tests(end+1)=struct('name',name,'passed',logical(value));end %#ok<AGROW>
    function negative(name,bad),a=adjudicate(bad,b,names,types,allNames);add(name,~a.passed&&strlength(a.failure)>0);end
    function out=fakeOwner(~,e),ownerCalls=ownerCalls+1;out=struct('fixture',true,'selector_count',numel(e.diagnostic_parameter_names));end
end

function claim=profileClaim()
claim=struct('declared_parameter_types_are_not_current_values',true,'no_defaults_substituted',true, ...
    'raw138_compared_with_original105',true,'no_parameter_restore_or_tuning',true, ...
    'position_control_identity','GPENMPC PositionControl uses a phase-controlled vertical I target of zero when route motion is absent.', ...
    'HTE_boundary','HTE validity requires dist_bottom > 1 m eligibility, source timing, the current estimate and cumulative Iz.', ...
    'application_identity_limit','Application file identity is provided by the separately recorded package/flash SHA-256.', ...
    'unknown_runtime_states',{{'actual position/filter sample rate and filter states','effective vertical I and accumulated Iz', ...
        'HTE validity/state and publication times','takeoff ramp state','route/payload topic availability'}}, ...
    'claim','Read-only parameter collection.');
end
function value=get(s,name,fallbackValue)
value=fallbackValue;if isstruct(s)&&isscalar(s)&&isfield(s,name),value=s.(name);end
end
function result=truth(s,name)
v=get(s,name,[]);result=islogical(v)&&isscalar(v)&&v;
end
function result=identity(path)
path=char(path);assert(isfile(path));f=fopen(path,'rb');assert(f>=0);g=onCleanup(@()fclose(f)); %#ok<NASGU>
b=fread(f,Inf,'*uint8');d=java.security.MessageDigest.getInstance('SHA-256');d.update(b);
result=struct('path',path,'bytes',numel(b),'sha256',upper(reshape(dec2hex(typecast(d.digest(),'uint8'),2).',1,[])));
end
