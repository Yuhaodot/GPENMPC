function receipt = inspect_native_lower_loop_readonly(outputPath, live)
% Read 26 additional selectors through the existing COM owner.
% live=false tests selector/schema handling; live=true retains raw and derived results.
arguments
    outputPath (1,1) string
    live (1,1) logical = false
end
assert(strlength(outputPath)>0 && ~ismissing(outputPath),'m600check:LowerLoopOutputPath');
rawPath=outputPath+".preflight.json";
assert(~isfile(outputPath)&&~isfile(rawPath)&&~isfile(rawPath+".mat"),...
    'm600check:PreflightOutputExists','No existing evidence is overwritten.');
here=fileparts(mfilename('fullpath'));addpath(here);
% Existing entry's false branch exercises invalid-name rejection before the
% serial class is constructed and returns its exact 79-selector vector.
base=inspect_m600_native_control_profile(rawPath,false);
[names,types]=lowerLoopNames();allNames=[string(base.requested_names),names];
assert(numel(allNames)==105&&numel(unique(allNames))==105);
declaration=checkDeclaration(names,types);
baseline=loadOriginal79(string(base.requested_names));
if ~live
    receipt=hostTests(base,names,types,allNames,declaration,baseline);
    return
end
expected=struct('uid',gpenmpc_device_identity('uid'),'board_version',56,...
    'commit','6ea3539157ca358c70a515878b77077af7d4611d',...
    'diagnostic_parameter_names',allNames);
raw=m600_canonical_serial_preflight(rawPath,expected);
rows=struct([]);
if isfield(raw,'diagnostic_parameter_observations'),rows=raw.diagnostic_parameter_observations;end
details=validateRows(rows,names,types);
original=compareOriginal79(rows,baseline);
complete=isfield(raw,'diagnostic_parameters_complete')&&raw.diagnostic_parameters_complete;
receipt=struct('schema','M600_NATIVE_LOWER_LOOP_READONLY_DETAILS_V1',...
    'passed',logical(raw.passed&&complete&&details.passed&&original.passed),...
    'safety_preflight_passed',logical(raw.passed),...
    'all_105_diagnostic_reads_complete',logical(complete),...
    'lower_loop_details_ready',logical(details.passed),...
    'original_79_semantics_match',logical(original.passed),...
    'original_79_validation',original,'original_79_baseline',baseline.source,...
    'requested_names',allNames,'original_name_count',79,'additional_name_count',26,...
    'lower_loop_validation',details,'declaration_only_not_live_values',declaration,...
    'raw_receipt',identity(rawPath),'raw_mat_path',rawPath+".mat",...
    'COM_closed',raw.COM_closed,'parameter_writes',raw.parameter_writes,...
    'mapping_writes',raw.mapping_writes,'arm_disarm_mode_requests',raw.arm_disarm_mode_requests,...
    'flash_reboot_count',raw.flash_reboot_count,'physical_output_actions',raw.physical_output_actions,...
    'live_hil_or_flight_run',false,'failure',string(raw.failure));
if ~details.passed,receipt.failure=receipt.failure+" | "+details.failure;end
if ~original.passed,receipt.failure=receipt.failure+" | "+original.failure;end
% This occurs only after the reused preflight has closed its COM owner.
fid=fopen(outputPath,'w','n','UTF-8');assert(fid>=0);guard=onCleanup(@()fclose(fid)); %#ok<NASGU>
fprintf(fid,'%s\n',jsonencode(receipt,PrettyPrint=true));
end

function baseline=loadOriginal79(names)
path=gpenmpc_external_path('native_control_readback_receipt');
source=identity(path);
assert(strcmp(source.sha256,'F0C96E672086D8AB4F385F8FD3CCF085DF4E91B25D336D263AF23A3522404D95'),...
    'm600check:Original79Identity','Original typed baseline source changed.');
raw=jsondecode(fileread(path));rows=raw.diagnostic_parameter_observations;
assert(raw.passed&&raw.diagnostic_parameters_complete&&numel(rows)==79,...
    'm600check:Original79Identity','Original baseline must contain 79 completed reads.');
assert(isequal(reshape(string({rows.name}),1,[]),names),...
    'm600check:Original79Identity','Original 79 selector order changed.');
types=arrayfun(@(x)double(x.typed_value.mav_type),rows).';
types=reshape(types,1,[]);valid=validateRows(rows,names,types);
assert(valid.passed,'m600check:Original79Identity','Original baseline typed values are invalid.');
baseline=struct('source',source,'names',names,'types',types,'rows',rows);
end

function result=compareOriginal79(rows,baseline)
typed=validateRows(rows,baseline.names,baseline.types);
evidence=repmat(struct('name','','passed',false,'expected_type',0,...
    'expected_raw_bits_hex','','observed_type',[],'observed_raw_bits_hex','','failure',''),1,79);
for k=1:79
    expected=baseline.rows(k).typed_value;evidence(k).name=char(baseline.names(k));
    evidence(k).expected_type=double(expected.mav_type);
    evidence(k).expected_raw_bits_hex=char(expected.raw_bits_hex);
    if ~typed.rows(k).passed
        evidence(k).failure=typed.rows(k).failure;continue
    end
    observed=typed.rows(k).observation.typed_value;
    evidence(k).observed_type=double(observed.mav_type);
    evidence(k).observed_raw_bits_hex=char(observed.raw_bits_hex);
    evidence(k).passed=isequal(double(observed.mav_type),double(expected.mav_type))&&...
        strcmpi(string(observed.raw_bits_hex),string(expected.raw_bits_hex));
    if ~evidence(k).passed,evidence(k).failure='CURRENT_NAME_TYPE_RAW_BITS_DIFFER_FROM_ORIGINAL79__NO_RESTORE_OR_WRITE';end
end
failureNames=string({evidence(~[evidence.passed]).name});
result=struct('passed',all([evidence.passed]),'expected_rows',79,...
    'matching_rows',sum([evidence.passed]),'rows',evidence,...
    'uncertain_or_changed_names',failureNames,...
    'failure',strjoin(failureNames,' | '),'restore_attempts',0,'parameter_writes',0);
end

function [names,types]=lowerLoopNames()
names=["MC_YAW_WEIGHT","MC_ROLLRATE_MAX","MC_PITCHRATE_MAX","MC_YAWRATE_MAX",...
    "MC_RR_INT_LIM","MC_PR_INT_LIM","MC_YR_INT_LIM","MC_YAW_TQ_CUTOFF",...
    "CA_R0_SLEW","CA_R1_SLEW","CA_R2_SLEW","CA_R3_SLEW","CA_R4_SLEW","CA_R5_SLEW",...
    "IMU_GYRO_CUTOFF","IMU_DGYRO_CUTOFF","IMU_GYRO_NF0_FRQ","IMU_GYRO_NF0_BW",...
    "IMU_GYRO_NF1_FRQ","IMU_GYRO_NF1_BW","IMU_GYRO_DNF_BW","IMU_GYRO_DNF_MIN",...
    "MC_BAT_SCALE_EN","IMU_GYRO_DNF_EN","IMU_GYRO_DNF_HMC","IMU_GYRO_RATEMAX"];
types=[9*ones(1,22),6*ones(1,4)];
assert(numel(names)==26&&numel(unique(names))==26&&all(strlength(names)<=16));
end

function result=checkDeclaration(names,types)
path=gpenmpc_external_path('px4_sitl_parameter_header');
source=fileread(path);observed=zeros(1,numel(names));
for k=1:numel(names)
    pattern=['\.name\s*=\s*"' char(names(k)) '"\s*,\s*\.val\s*=\s*\{\s*\.(f|i)\s*='];
    token=regexp(source,pattern,'tokens');assert(numel(token)==1,'m600check:ParameterDeclaration','Ambiguous declaration: %s',names(k));
    if strcmp(token{1}{1},'f'),observed(k)=9;else,observed(k)=6;end
end
assert(isequal(observed,types),'m600check:ParameterDeclaration','Source-declared type differs.');
result=struct('source',identity(path),'names',names,'mav_types',types,...
    'declaration_passed',true,'default_values_used',false);
end

function result=validateRows(rows,names,types)
evidence=repmat(struct('name','','required_mav_type',0,'passed',false,...
    'failure','','observation',struct()),1,numel(names));
failures=strings(0,1);
for k=1:numel(names)
    evidence(k).name=char(names(k));evidence(k).required_mav_type=types(k);
    try
        assert(isstruct(rows)&&isfield(rows,'name'),'m600check:LowerLoopMissingRow','No typed rows.');
        at=find(string({rows.name})==names(k));assert(numel(at)==1,'m600check:LowerLoopMissingRow','Missing/duplicate row.');
        row=rows(at);evidence(k).observation=row;
        assert(isfield(row,'read_error')&&strlength(string(row.read_error))==0,'m600check:LowerLoopReadError','Read error.');
        assert(isfield(row,'write_count')&&isequal(row.write_count,0),'m600check:LowerLoopUnexpectedWrite','Nonzero/unknown write count.');
        v=row.typed_value;
        assert(isstruct(v)&&isscalar(v)&&all(isfield(v,{'name','mav_type','raw_bits_hex','decoded'})),...
            'm600check:LowerLoopTypedSchema','Missing typed payload.');
        assert(strcmp(string(v.name),names(k))&&isequal(double(v.mav_type),types(k)),...
            'm600check:LowerLoopType','Typed name/type mismatch.');
        bits=char(string(v.raw_bits_hex));assert(~isempty(regexp(bits,'^[0-9A-Fa-f]{8}$','once')),...
            'm600check:LowerLoopBits','Expected eight raw hex digits.');
        raw=uint32(hex2dec(bits));
        if types(k)==9,decoded=double(typecast(raw,'single'));else,decoded=double(typecast(raw,'int32'));end
        assert(isfinite(decoded)&&isnumeric(v.decoded)&&isscalar(v.decoded)&&isfinite(v.decoded)&&double(v.decoded)==decoded,...
            'm600check:LowerLoopDecoded','Nonfinite or raw-bits/decoded disagreement.');
        evidence(k).passed=true;
    catch err
        evidence(k).failure=[err.identifier ': ' err.message];
        failures(end+1)=names(k)+": "+string(evidence(k).failure); %#ok<AGROW>
    end
end
result=struct('passed',all([evidence.passed]),'expected_rows',numel(names),...
    'passed_rows',sum([evidence.passed]),'rows',evidence,'failure',strjoin(failures,' | '));
end

function result=hostTests(base,names,types,allNames,declaration,baseline)
tests=struct('name',{},'passed',{});
add('original_selector_negative_controls',base.passed&&base.tests==5);
add('105_unique_original_order_preserved',numel(allNames)==105&&isequal(allNames(1:79),string(base.requested_names)));
add('26_types_source_declared_no_default_values',declaration.declaration_passed&&~declaration.default_values_used);
add('selector_exact_length_and_characters',all(strlength(names)<=16)&&all(~cellfun(@isempty,regexp(cellstr(names),'^[A-Z][A-Z0-9_]*$','once'))));
original=compareOriginal79(baseline.rows,baseline);add('original79_exact_bits_positive',original.passed&&original.matching_rows==79);
changed=baseline.rows;v=changed(1).typed_value;
if v.mav_type==6,v.raw_bits_hex='00000063';v.decoded=99;else,v.raw_bits_hex='42C60000';v.decoded=99;end
changed(1).typed_value=v;original=compareOriginal79(changed,baseline);
add('original79_changed_value_rejected_without_restore',~original.passed&&original.matching_rows==78&&original.restore_attempts==0);
changed=baseline.rows;changed(1).typed_value.mav_type=5;original=compareOriginal79(changed,baseline);
add('original79_changed_type_rejected',~original.passed&&original.matching_rows==78);
original=compareOriginal79(baseline.rows(2:end),baseline);add('original79_missing_rejected',~original.passed&&original.matching_rows==78);
mock=repmat(struct('name','','typed_value',struct(),'read_error','','read_utc','HOST_SYNTHETIC_NOT_LIVE','write_count',0),1,26);
for k=1:26
    if types(k)==9,bits='3E800000';value=.25;else,bits='00000001';value=1;end
    mock(k).name=char(names(k));mock(k).typed_value=struct('name',char(names(k)),'mav_type',types(k),'raw_bits_hex',bits,'decoded',value);
end
r=validateRows(mock,names,types);add('synthetic_exact_typed_positive',r.passed&&r.passed_rows==26);
bad=mock(2:end);negative('missing_typed_row',bad);
bad=[mock,mock(1)];negative('duplicate_typed_row',bad);
bad=mock;bad(1).typed_value.mav_type=6;negative('wrong_float_type',bad);
bad=mock;bad(26).typed_value.mav_type=9;negative('wrong_integer_type',bad);
bad=mock;bad(1).typed_value.name='MC_YAW_P';negative('wrong_typed_name',bad);
bad=mock;bad(1).read_error='timeout';negative('read_timeout_retained',bad);
bad=mock;bad(1).typed_value=struct();negative('empty_payload',bad);
bad=mock;bad(1).typed_value.raw_bits_hex='7FC00000';bad(1).typed_value.decoded=NaN;negative('nan_float_bits',bad);
bad=mock;bad(1).typed_value.raw_bits_hex='7F800000';bad(1).typed_value.decoded=Inf;negative('infinite_float_bits',bad);
bad=mock;bad(1).typed_value.decoded=.5;negative('decoded_bits_mismatch',bad);
bad=mock;bad(26).typed_value.decoded=1.5;negative('noninteger_decoded',bad);
bad=mock;bad(1).typed_value.raw_bits_hex='XYZ';negative('invalid_bits_string',bad);
bad=mock;bad(1).write_count=1;negative('write_counter_not_zero',bad);
assert(all([tests.passed]),'m600check:LowerLoopHostTests','HOST-only tests failed.');
result=struct('schema','HOST_NATIVE_LOWER_LOOP_READONLY_ENTRY_TESTS_V1',...
    'passed',true,'tests',tests,'case_count',numel(tests),'cases_passed',sum([tests.passed]),...
    'inherited_selector_negative_cases',base.tests,'requested_names',allNames,...
    'additional_names',names,'additional_types',types,'source_declaration',declaration,...
    'original_79_baseline',baseline.source,...
    'COM_open',0,'board_access',0,'parameter_writes',0,'mapping_writes',0,...
    'arm_disarm_mode_requests',0,'reboot_flash',0,'output_actions',0);
    function add(name,passed),tests(end+1)=struct('name',name,'passed',logical(passed));end %#ok<AGROW>
    function negative(name,rows),r=validateRows(rows,names,types);add(name,~r.passed&&strlength(r.failure)>0);end
end

function result=identity(path)
path=char(path);assert(isfile(path));f=fopen(path,'rb');assert(f>=0);c=onCleanup(@()fclose(f)); %#ok<NASGU>
b=fread(f,Inf,'*uint8');md=java.security.MessageDigest.getInstance('SHA-256');md.update(b);
hash=upper(reshape(dec2hex(typecast(md.digest(),'uint8'),2).',1,[]));
result=struct('path',path,'bytes',numel(b),'sha256',hash);
end
