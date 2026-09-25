function result=run_terrain_diagnostic_runtime_contract_tests(newOutputDir)
% Test terrain runtime contracts using files and structs.
arguments,newOutputDir (1,1) string,end
assert(~isfolder(newOutputDir)&&~isfile(newOutputDir),'m600check:TerrainContractTestsExist');
b=string(fileparts(fileparts(mfilename('fullpath'))));oldPath=path;
guard=onCleanup(@()path(oldPath)); %#ok<NASGU>
addpath(fullfile(b,'m600_coptersim','matlab_validation'),fullfile(b,'matlab_validation'),fullfile(b,'tools'),'-begin');
mkdir(newOutputDir);checks=struct('name',{},'passed',{});
contract=struct();validation=struct('passed',false,'failure','NOT_RUN');
firstError=struct('identifier','','message','','extended_report','');failure='';
try
[contract,validation]=m600check.buildTerrainDiagnosticRuntimeContract();
assert(validation.passed,'m600check:TerrainContractPositiveFixture','%s',validation.failure);
check('actual_bound_HOST_evidence_passes',validation.passed);
check('actual_contract_has_no_live_flight_or_installation_credit', ...
    ~validation.new_live_credit&&~validation.flight_admission&&~validation.consumer_verified&& ...
    contract.require_terrain_diagnostic_extension&&~contract.claims.runtime_installed);
roundtrip=jsondecode(jsonencode(contract));
r=m600check.validateTerrainDiagnosticRuntimeContract(roundtrip);
check('JSON_roundtrip_preserves_semantics',r.passed);
consumer=struct('model_dll',entry('new_dll'),'source_model',entry('new_model'), ...
    'core_parameters',entry('new_parameters'),'require_terrain_diagnostic_extension',true);
r=m600check.validateTerrainDiagnosticRuntimeContract(contract,consumer);
check('actual_consumer_copy_identity_and_flag_bind_without_live_credit', ...
    r.passed&&r.consumer_verified&&~r.new_live_credit&&~r.flight_admission);
bad=contract;bad.require_terrain_diagnostic_extension=false;reject('missing_required_flag_false',bad);
bad=rmfield(contract,'require_terrain_diagnostic_extension');reject('missing_required_flag_field',bad);
bad=contract;bad.require_terrain_diagnostic_extension=1;reject('numeric_one_is_not_boolean_flag',bad);
bad=contract;bad.requires_fresh_runtime_preflight_and_observation=1;reject('numeric_fresh_flag_rejected',bad);
bad=contract;bad.require_terrain_diagnostic_extension=[true,true];reject('vector_extension_flag_rejected',bad);
bad=contract;bad.requires_fresh_runtime_preflight_and_observation=false;reject('old_45s_is_not_new_runtime_observation',bad);
bad=contract;bad.old_observation.dll_sha256=consumer.model_dll.sha256;reject('reference_model_identity_mismatch',bad);
bad=contract;bad.old_observation.duration_s=60;reject('old_window_not_relabelled',bad);
bad=contract;bad.changes.diagnostic_prefix7='CHANGED';reject('changed_prefix_contract_rejected',bad);
bad=contract;bad.changes.sensor_pack_and_level_inverse='DOUBLE_COMPENSATION';reject('sensor_change_rejected',bad);
bad=contract;bad.changes.terrain_guard='ALLOW_NONFINITE';reject('guard_relaxation_rejected',bad);
bad=contract;bad.layout.first_terrain15=10:24;reject('shifted_wire_fields_rejected',bad);
bad=contract;bad.layout.packet_bytes=272;reject('new_protocol_shape_rejected',bad);
bad=contract;bad.claims.new_live_verified=true;reject('fake_live_PASS_rejected',bad);
bad=contract;bad.claims.control_performance_PASS=true;reject('off_scope_performance_claim_rejected',bad);
bad=contract;bad.claims.terrain_source_cause_identified=true;reject('historical_root_cause_not_invented',bad);
bad=contract;bad.claims.runtime_installed=true;reject('installation_claim_rejected',bad);
bad=contract;bad.claims.extra_flight_PASS=true;reject('unknown_claim_field_rejected',bad);
bad=contract;bad.claims.host_equivalence=1;reject('nested_numeric_one_is_not_logical_true',bad);
bad=contract;bad.claims.new_live_verified=0;reject('nested_numeric_zero_is_not_logical_false',bad);
bad=contract;bad.authority.COM_open=false;reject('nested_logical_false_is_not_numeric_action_count',bad);
for field={'COM_open','UDP_open','board_actions','parameter_writes','firmware_writes','arm_requests','mode_requests','outputs'}
    bad=contract;bad.authority.(field{1})=1;reject(['authority_' field{1} '_rejected'],bad);
end
bad=contract;bad.bindings(end+1)=bad.bindings(1);reject('duplicate_binding_rejected',bad);
bad=contract;bad.bindings(1)=[];reject('missing_binding_rejected',bad);
for label={'new_dll','new_model','new_parameters','core_source','sensor_encoder','terrain_decoder'}
    bad=contract;index=find(strcmp({bad.bindings.label},label{1}));
    bad.bindings(index).sha256=repmat('0',1,64);reject(['sha_' label{1} '_rejected'],bad);
end
bad=contract;index=find(strcmp({bad.bindings.label},'new_dll'));bad.bindings(index).bytes=0;reject('wrong_bytes_rejected',bad);
bad=contract;bad.bindings(index).path=fullfile(newOutputDir,'MISSING.dll');reject('missing_file_rejected',bad);
% Require the completed probe record; a synthetic PASS file is insufficient.
fakePath=fullfile(newOutputDir,'SYNTHETIC_FALSE_PASS.json');
writeText(fakePath,jsonencode(struct('passed',true,'status','PASS','rows',5940,'cases',50)));
bad=contract;index=find(strcmp({bad.bindings.label},'extension_probe'));info=dir(fakePath);
bad.bindings(index).path=char(fakePath);bad.bindings(index).bytes=info.bytes;
bad.bindings(index).sha256=m600check.fileSha256(fakePath);reject('synthetic_PASS_even_with_own_hash_rejected',bad);
wrong=consumer;wrong.require_terrain_diagnostic_extension=false;rejectConsumer('consumer_flag_false_rejected',wrong);
wrong=consumer;wrong.require_terrain_diagnostic_extension=1;rejectConsumer('consumer_numeric_flag_rejected',wrong);
wrong=consumer;wrong.require_terrain_diagnostic_extension=[true,true];rejectConsumer('consumer_vector_flag_rejected',wrong);
wrong=rmfield(consumer,'require_terrain_diagnostic_extension');rejectConsumer('consumer_missing_flag_rejected',wrong);
wrong=consumer;wrong.model_dll=entry('old_dll');rejectConsumer('consumer_old_DLL_not_new_DLL',wrong);
wrong=consumer;wrong.source_model=entry('old_model');rejectConsumer('consumer_old_model_not_new_model',wrong);
wrong=consumer;wrong.flight_PASS=true;rejectConsumer('consumer_extra_claim_rejected',wrong);
catch problem
    failure=[problem.identifier ': ' problem.message];
    firstError=struct('identifier',problem.identifier,'message',problem.message, ...
        'extended_report',getReport(problem,'extended','hyperlinks','off'));
end
passed=isempty(failure)&&~isempty(checks)&&all([checks.passed])&&validation.passed;
status='PASS_HOST_TERRAIN_DIAGNOSTIC_RUNTIME_CONTRACT_TESTS';
if ~passed,status='FAIL_HOST_TERRAIN_DIAGNOSTIC_RUNTIME_CONTRACT_TESTS__PARTIAL_CHECKS_RETAINED';end
result=struct('schema','HOST_TERRAIN_DIAGNOSTIC_RUNTIME_CONTRACT_TESTS_V1', ...
    'status',status,'passed',passed,'failure',failure,'first_error',firstError, ...
    'checks_total',numel(checks),'checks_passed',sum([checks.passed]), ...
    'checks',checks,'contract',contract,'positive_validation',validation, ...
    'COM_open',0,'UDP_open',0,'board_actions',0,'model_loaded',false,'DLL_loaded',false, ...
    'runtime_installed',false,'live_or_flight_claim',false);
result.tested_sources=struct( ...
    'builder_sha256',m600check.fileSha256(which('m600check.buildTerrainDiagnosticRuntimeContract')), ...
    'checker_sha256',m600check.fileSha256(which('m600check.validateTerrainDiagnosticRuntimeContract')), ...
    'tests_sha256',m600check.fileSha256([mfilename('fullpath') '.m']));
writeText(fullfile(newOutputDir,'RESULT.json'),jsonencode(result,PrettyPrint=true));
disp(struct('passed',result.passed,'checks',result.checks_total,'COM_open',0));
assert(result.passed,'m600check:TerrainRuntimeContractTestsFailed','%s',result.failure);
    function check(name,passed)
        passed=isscalar(passed)&&logical(passed);checks(end+1)=struct('name',name,'passed',passed);
        assert(passed,'m600check:TerrainRuntimeContractTest','%s',name);
    end
    function reject(name,bad)
        r=m600check.validateTerrainDiagnosticRuntimeContract(bad);
        check(name,~r.passed&&~r.new_live_credit&&~r.flight_admission&&~isempty(r.failure));
    end
    function rejectConsumer(name,bad)
        r=m600check.validateTerrainDiagnosticRuntimeContract(contract,bad);
        check(name,~r.passed&&~r.consumer_verified&&~r.new_live_credit&&~r.flight_admission);
    end
    function r=entry(label)
        index=find(strcmp({contract.bindings.label},label));assert(isscalar(index));r=rmfield(contract.bindings(index),'label');
    end
end
function writeText(file,text)
f=fopen(file,'w','n','UTF-8');assert(f>=0);guard=onCleanup(@()fclose(f)); %#ok<NASGU>
fprintf(f,'%s\n',text);
end
