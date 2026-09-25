function report=run_temporary_allocator_geometry_contract_tests(outputDir)
% Test allocator-geometry data and file bindings.
arguments,outputDir (1,1) string,end
assert(~isfolder(outputDir),'m600check:TestOutputExists','Never overwrite a test result.');
mkdir(outputDir);
build=fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(build,'m600_coptersim','matlab_validation'));
auditPath=fullfile(gpenmpc_external_path('host_native_rotor_geometry'),'RESULT.json');
math=jsondecode(fileread(auditPath));a=math.audit;n=jsondecode(fileread(a.receipt_path));
[c,v]=m600check.buildTemporaryAllocatorGeometryContract(auditPath);
checks=struct('name',{},'passed',{},'observed_pass',{},'failure',{});
note('actual_readonly_assembler_and_file_bindings',v,true);
assert(v.passed,'m600check:ContractFixture','Actual contract failed: %s',v.failure);
e=c.entries;g=c.unchanged_guard_entries;
note('exact_12_changes_67_guards_79_native_rows',struct('passed', ...
    numel(v.entries)==12&&numel(v.unchanged_guard_entries)==67&&v.data_validation.native_parameter_count==79,'failure',''),true);
note('JSON_round_trip_preserves_exact_bits',m600check.validateTemporaryAllocatorGeometry(jsondecode(jsonencode(c))),true);
note('pure_actual_data',pure(e,a,n,g),true);
note('entry_and_guard_order_independent',pure(e(end:-1:1),a,n,g(end:-1:1)),true);
q=e;for k=1:numel(q),q(k).original_raw_bits_hex=lower(q(k).original_raw_bits_hex);q(k).target_raw_bits_hex=lower(q(k).target_raw_bits_hex);end
note('hex_case_normalized_without_numeric_reencoding',pure(q,a,n,g),true);

q=e(1:11);note('missing_geometry_entry',pure(q,a,n,g),false);
q=[e;e(1)];note('extra_geometry_entry',pure(q,a,n,g),false);
q=e;q(2)=q(1);note('duplicate_geometry_entry',pure(q,a,n,g),false);
q=e;q(1).name='CA_ROTOR0_PZ';note('PZ_not_a_writable_geometry_coordinate',pure(q,a,n,g),false);
q=e;q(1).name='MC_ROLL_P';note('gain_not_a_writable_geometry_coordinate',pure(q,a,n,g),false);
q=e;q(1).mav_type=6;note('geometry_must_be_REAL32',pure(q,a,n,g),false);
for h={'7FC00000','7F800000','FF800000','123','GGGGGGGG'}
    q=e;q(1).target_raw_bits_hex=h{1};note(['target_reject_' h{1}],pure(q,a,n,g),false);
end
q=e;q(1).original_raw_bits_hex='80000000';note('original_signed_zero_bit_mismatch',pure(q,a,n,g),false);
q=e;q(1).target_raw_bits_hex='3F000000';note('target_finite_but_not_proved_candidate',pure(q,a,n,g),false);
q=rmfield(e,'target_raw_bits_hex');note('missing_target_field_returns_error',pure(q,a,n,g),false);
q=a;q.proposed_parameter_delta(1).rollback_raw_bits_hex='80000000';note('audit_rollback_not_exact_original',pure(e,q,n,g),false);
q=a;q.proposed_parameter_delta(1).candidate_value=q.proposed_parameter_delta(1).candidate_value+.01;note('audit_numeric_target_not_raw_bits',pure(e,q,n,g),false);
q=a;q.proposed_parameter_delta(2)=q.proposed_parameter_delta(1);note('audit_duplicate_delta',pure(e,q,n,g),false);
for field={'passed','candidate_geometry_aligned','actual_yaw_sign_matches','actual_collective_thrust_matches'}
    q=a;q.(field{1})=false;note(['math_reject_' field{1}],pure(e,q,n,g),false);
end
q=a;q.flight_admission=true;note('no_live_claim_from_math_proof',pure(e,q,n,g),false);
q=a;q.flight_admission=NaN;note('ambiguous_flight_claim_rejected',pure(e,q,n,g),false);
q=a;q.hardware_actions=1;note('math_proof_has_no_hardware',pure(e,q,n,g),false);
q=a;q.candidate_alignment_max_abs=2*q.numerical_real32_alignment_tolerance;note('numeric_alignment_must_pass',pure(e,q,n,g),false);
q=a;q.numerical_real32_alignment_tolerance=0;note('invalid_rounding_tolerance',pure(e,q,n,g),false);

q=n;q.schema='OTHER';note('native_receipt_schema_exact',pure(e,a,q,g),false);
q=n;q.diagnostic_parameters_complete=false;note('native_incomplete',pure(e,a,q,g),false);
q=n;q.diagnostic_parameter_observations=q.diagnostic_parameter_observations(1:78);note('native79_missing_row',pure(e,a,q,g),false);
q=n;q.diagnostic_parameter_observations(2)=q.diagnostic_parameter_observations(1);note('native79_duplicate_row',pure(e,a,q,g),false);
q=n;q.diagnostic_parameter_observations(1).read_error='TIMEOUT';note('native_read_error_retained',pure(e,a,q,g),false);
q=n;q.diagnostic_parameter_observations(1).write_count=1;note('native_row_write_rejected',pure(e,a,q,g),false);
q=n;q.diagnostic_parameter_observations(1).typed_value.name='OTHER';note('native_nested_name_mismatch',pure(e,a,q,g),false);
q=n;q.diagnostic_parameter_observations(1).typed_value.decoded=99;note('native_decoded_raw_bits_mismatch',pure(e,a,q,g),false);
for field={'parameter_writes','mapping_writes','arm_disarm_mode_requests','flash_reboot_count','physical_output_actions'}
    q=n;q.(field{1})=1;note(['native_reject_' field{1}],pure(e,a,q,g),false);
end
note('missing_guard',pure(e,a,n,g(1:66)),false);
q=g;q(2)=q(1);note('duplicate_guard',pure(e,a,n,q),false);
for name={'CA_ROTOR0_CT','CA_ROTOR0_KM','CA_ROTOR0_PZ','CA_ROTOR0_AX','CA_ROTOR0_AY', ...
        'CA_ROTOR0_AZ','MC_ROLL_P','MC_ROLLRATE_P','FD_FAIL_R','THR_MDL_FAC'}
    q=g;k=find(strcmp({q.name},name{1}));assert(isscalar(k));q(k).raw_bits_hex='3F400000';
    note(['unchanged_guard_reject_' name{1}],pure(e,a,n,q),false);
end
q=g;q(1).mav_type=9;note('unchanged_guard_type_exact',pure(e,a,n,q),false);
q=a;q.unchanged_diagnostic_parameters(1).read_error='FAILED';note('audit_unchanged_guard_read_error',pure(e,q,n,g),false);

q=c;q.schema='OTHER';bound('contract_schema',q);
q=c;q.maximum_apply_passes=2;bound('at_most_one_apply_pass',q);
q=c;q.maximum_restore_passes=1;bound('no_cross_owner_restore_count_cap',q);
q=c;q.restore_policy='ONE_RESTORE_ONLY';bound('restore_policy_exact',q);
for field={'gains_changes','failure_detector_changes','physical_mapping_changes','model_changes'}
    q=c;q.(field{1})=1;bound(['contract_scope_' field{1}],q);
end
for key={'geometry_audit','native79','canonical_source'}
    q=c;q.bindings.(key{1}).sha256=repmat('0',1,64);bound(['binding_SHA_' key{1}],q);
    q=c;q.bindings.(key{1}).bytes=q.bindings.(key{1}).bytes+1;bound(['binding_bytes_' key{1}],q);
    q=c;q.bindings.(key{1}).path=[q.bindings.(key{1}).path '.NOT_PRESENT'];bound(['binding_missing_' key{1}],q);
end
q=c;q.bindings.canonical_source=q.bindings.native79;bound('valid_hash_wrong_source_rejected',q);
q=c;q.entries(1).target_raw_bits_hex='3F000000';bound('file_proof_and_contract_candidate_must_match',q);
q=c;q.unchanged_guard_entries(1).raw_bits_hex='00000001';bound('file_native_and_guard_bits_must_match',q);
[~,bad]=m600check.buildTemporaryAllocatorGeometryContract([auditPath '.NOT_PRESENT']);note('assembler_missing_file_returns_error',bad,false);

report=struct('schema','HOST_TEMPORARY_ALLOCATOR_GEOMETRY_CONTRACT_TEST_V1', ...
    'passed',all([checks.passed]),'case_count',numel(checks),'cases_passed',nnz([checks.passed]), ...
    'cases',checks,'actual_contract',c,'actual_validation',v,'COM_UDP_board_model_actions',0, ...
    'flight_admission',false,'read_only_input_bindings',c.bindings,'source_identities',[]);
paths={mfilename('fullpath'),which('m600check.validateTemporaryAllocatorGeometry'), ...
    which('m600check.validateTemporaryAllocatorGeometryEntries'),which('m600check.buildTemporaryAllocatorGeometryContract')};
paths{1}=[paths{1} '.m'];
for k=1:numel(paths),r=identity(paths{k});if isempty(report.source_identities),report.source_identities=r;else,report.source_identities(end+1)=r;end;end
fid=fopen(fullfile(outputDir,'RESULT.json'),'w','n','UTF-8');assert(fid>=0);cleanup=onCleanup(@()fclose(fid)); %#ok<NASGU>
fprintf(fid,'%s',jsonencode(report,PrettyPrint=true));
fprintf('HOST temporary allocator contract %d/%d passed; COM/board/model actions=0\n',report.cases_passed,report.case_count);
assert(report.passed,'m600check:ContractTestsFailed','See preserved per-case RESULT.json.');
    function r=pure(entries,audit,native,guards)
        r=m600check.validateTemporaryAllocatorGeometryEntries(entries,audit,native,guards);
    end
    function bound(name,contract)
        note(name,m600check.validateTemporaryAllocatorGeometry(contract),false);
    end
    function note(name,result,expected)
        observed=isstruct(result)&&isscalar(result)&&isfield(result,'passed')&&isequal(result.passed,true);
        safeError=observed||(isfield(result,'failure')&&~isempty(result.failure)&&isfield(result,'errors')&&~isempty(result.errors));
        checks(end+1)=struct('name',name,'passed',observed==expected&&safeError, ...
            'observed_pass',observed,'failure',result.failure);
    end
end
function r=identity(path)
fid=fopen(path,'rb');assert(fid>=0);c=onCleanup(@()fclose(fid)); %#ok<NASGU>
raw=fread(fid,Inf,'*uint8');md=java.security.MessageDigest.getInstance('SHA-256');md.update(raw);
h=upper(reshape(dec2hex(typecast(md.digest(),'uint8'),2).',1,[]));r=struct('path',char(path),'bytes',numel(raw),'sha256',h);
end
