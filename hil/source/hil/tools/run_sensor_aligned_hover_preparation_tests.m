function result=run_sensor_aligned_hover_preparation_tests(outputRoot)
% Test preparation bindings using in-memory mutations.
build=fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(build,'m600_coptersim','matlab_validation'));
if nargin<1, outputRoot=fullfile(build,'evidence','sensor_aligned_hover_preparation'); end
assert(~isfolder(outputRoot),'m600check:ExistingEvidence','Output must be new.');
mkdir(outputRoot);
context=make_sensor_aligned_hover_preparation(); contract=context.calibration_contract;
names={'reference_observation','json_roundtrip','missing_context','old_v2_alone','missing_reference', ...
    'missing_file','bad_hash','bad_bytes','changed_dll','changed_profile','incomplete_window', ...
    'incomplete_duration','reset','source_reversal','fatal','unsafe_finally','wrong_uid', ...
    'wrong_firmware','stale_context','wrong_scope','flight_permission','changed_contract', ...
    'changed_offset_policy','unknown_context_field','duplicate_reference','changed_encoder'};
entries=repmat(struct('name','','passed',false,'expected_pass',false,'report',[]),numel(names),1);
for k=1:numel(names)
    c=context; cc=contract; n=names{k}; expected=k<=2;
    switch n
        case 'json_roundtrip', c=jsondecode(jsonencode(c)); cc=jsondecode(jsonencode(cc));
        case 'missing_context', c=[];
        case 'old_v2_alone', c=contract;
        case 'missing_reference', c.references(1)=[];
        case 'missing_file', c.references(end).path=[c.references(end).path '.NONEXISTENT'];
        case 'bad_hash', c.references(end).sha256=repmat('0',1,64);
        case 'bad_bytes', c.references(end).bytes=c.references(end).bytes+1;
        case 'changed_dll', c.runtime_binding.model_dll_sha256=repmat('0',1,64);
        case 'changed_profile', c.runtime_binding.hil_profile_sha256=repmat('0',1,64);
        case 'incomplete_window', c.observation.elapsed_s=44.9;
        case 'incomplete_duration', c.observation.duration_contract_s=44;
        case 'reset', c.observation.source_reset_count=1;
        case 'source_reversal', c.observation.source_reversal_count=1;
        case 'fatal', c.observation.fatal_present=true;
        case 'unsafe_finally', c.observation.safe_finally=false;
        case 'wrong_uid', c.expected_identity.uid='3473490377090611258';
        case 'wrong_firmware', c.expected_identity.flight_custom_version_hex='0000000000000000';
        case 'stale_context', c.observation_execution_id='UNRELATED_OBSERVATION';
        case 'wrong_scope', c.scope='FLIGHT';
        case 'flight_permission', c.flight_admission=true;
        case 'changed_contract', cc.mount(2).raw_bits_hex='00000000';
        case 'changed_offset_policy', cc.offset_policy='ASSUME_ZERO'; c.calibration_contract=cc;
        case 'unknown_context_field', c.ignore_errors=true;
        case 'duplicate_reference', c.references(end+1)=c.references(end);
        case 'changed_encoder', c.runtime_binding.sensor_frame_encoder_sha256=repmat('0',1,64);
    end
    r=m600check.verifySensorAlignedHoverPreparation(c,cc);
    entries(k)=struct('name',n,'passed',r.passed==expected&&~r.flight_admission&& ...
        r.required_fresh_preflight&&r.hardware_actions==0,'expected_pass',expected,'report',r);
end
result=struct('schema','HOST_SENSOR_ALIGNED_HOVER_PREPARATION_TEST_V1', ...
    'passed',all([entries.passed]),'case_count',numel(entries),'cases_passed',sum([entries.passed]), ...
    'context',context,'cases',entries,'COM_UDP_board_model_process_actions',0,'flight_admission',false);
result.sources=struct('helper',source(fullfile(build,'m600_coptersim','matlab_validation','+m600check','verifySensorAlignedHoverPreparation.m')), ...
    'maker',source(fullfile(build,'tools','make_sensor_aligned_hover_preparation.m')), ...
    'tests',source([mfilename('fullpath') '.m']));
fid=fopen(fullfile(outputRoot,'RESULT.json'),'w','n','UTF-8'); assert(fid>=0);
clean=onCleanup(@()fclose(fid)); fprintf(fid,'%s',jsonencode(result,PrettyPrint=true)); clear clean
fprintf('HOST SENSOR-ALIGNED HOVER PREPARATION: %d/%d\n',result.cases_passed,result.case_count);
assert(result.passed,'m600check:HostTestsFailed','HOST cases failed.');
end
function r=source(path)
fid=fopen(path,'rb'); cl=onCleanup(@()fclose(fid)); data=fread(fid,Inf,'*uint8'); %#ok<NASGU>
md=java.security.MessageDigest.getInstance('SHA-256'); md.update(data);
h=upper(reshape(dec2hex(typecast(md.digest(),'uint8'),2).',1,[]));
r=struct('path',path,'bytes',numel(data),'sha256',h);
end
