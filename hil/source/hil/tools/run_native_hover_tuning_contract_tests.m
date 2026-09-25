function result=run_native_hover_tuning_contract_tests(outputDir)
% Test tuning-contract bindings and invalid schemas.
arguments
    outputDir (1,1) string
end
assert(~isfolder(outputDir)&&~isfile(outputDir),'m600check:TuningTestOutputExists');
[c,v]=m600check.buildNativeHoverTuningContract();
tests=struct('name',{},'passed',{});failures=struct('test',{},'failure',{});
add('actual_three_parameter_builder_pass',v.passed);
assert(v.passed,'m600check:TuningBuildFailed','%s',v.failure);
add('three_exact_names',isequal(reshape({c.entries.name},1,[]),{'MC_ROLL_P','MC_PITCH_P','MPC_THR_HOVER'}));
add('all_three_REAL32',all([c.entries.mav_type]==9));
add('135_original_guards_preserved',numel(c.unchanged_guard_entries)==135&&numel(unique({c.unchanged_guard_entries.name}))==135);
add('29_output_profile_guards_preserved',numel(c.output_guard_entries)==29);
add('no_authority_no_device',~c.authority_granted&&v.hardware_actions==0&&v.COM_UDP_actions==0);
add('margin_not_safety_gate',~c.engineering_margin_is_safety_gate);
add('consumed_source_byte_identity_preserved', ...
    c.source_provenance_resolution.declared.bytes==c.source_provenance_resolution.consumed.bytes&& ...
    strcmp(c.source_provenance_resolution.declared.sha256,c.source_provenance_resolution.consumed.sha256)&& ...
    ~strcmp(c.source_provenance_resolution.declared.path,c.source_provenance_resolution.consumed.path));
bad=c;bad.schema='UNKNOWN';negative('wrong_schema',bad);
bad=c;bad.disable_FD=true;negative('unknown_top_level_field',bad);
bad=c;bad.entries(1).name='FD_FAIL_R';negative('unknown_parameter_name',bad);
bad=c;bad.entries(1).name='CA_ROTOR0_PX';negative('geometry_cannot_be_smuggled',bad);
bad=c;bad.entries(1).name='PWM_MAIN_FUNC1';negative('output_mapping_cannot_be_smuggled',bad);
bad=c;bad.entries(2)=bad.entries(1);negative('duplicate_parameter',bad);
bad=c;bad.entries(3)=[];negative('missing_parameter',bad);
bad=c;bad.entries(4)=bad.entries(3);negative('fourth_parameter_rejected',bad);
bad=c;bad.entries(1).mav_type=6;negative('wrong_type',bad);
bad=c;bad.entries(1).target_raw_bits_hex='BAD_BITS';negative('malformed_bits',bad);
bad=c;bad.entries(1).target_raw_bits_hex='7FC00000';negative('NaN_bits',bad);
bad=c;bad.entries(3).target_raw_bits_hex='7F800000';negative('Inf_bits',bad);
bad=c;bad.entries(1).original_raw_bits_hex='40000000';negative('original_value_changed',bad);
bad=c;bad.entries(3).target_raw_bits_hex='3F19999A';negative('finite_target_not_selected',bad);
bad=c;bad.entries(1).unsafe_override=true;negative('unknown_entry_field',bad);
bad=c;bad.bindings.current138.sha256=repmat('0',1,64);negative('current138_hash_changed',bad);
bad=c;bad.bindings.current138_raw.bytes=bad.bindings.current138_raw.bytes+1;negative('raw_bytes_changed',bad);
bad=c;bad.bindings.attitude_candidate.sha256=repmat('F',1,64);negative('attitude_proof_hash_changed',bad);
bad=c;bad.bindings.ground_selection.sha256=repmat('A',1,64);negative('selection_hash_changed',bad);
bad=c;bad.source_provenance(1).sha256=repmat('0',1,64);negative('source_provenance_changed',bad);
bad=c;bad.source_provenance(1)=[];negative('source_provenance_missing',bad);
bad=c;bad.source_provenance_resolution.consumed.sha256=repmat('0',1,64);negative('consumed_source_hash_changed',bad);
bad=c;bad.source_provenance_resolution.consumed.bytes=bad.source_provenance_resolution.consumed.bytes+1;negative('consumed_source_bytes_changed',bad);
bad=c;bad.source_provenance_resolution.consumed.path=bad.source_provenance_resolution.declared.path;negative('changed_work_copy_cannot_replace_consumed_source',bad);
bad=c;bad.source_provenance_resolution.declared.sha256=repmat('0',1,64);negative('original_source_declaration_not_rewritten',bad);
bad=c;bad.unchanged_guard_entries(1).raw_bits_hex='00000001';negative('unchanged_original_guard_drift',bad);
bad=c;k=find(strcmp({bad.unchanged_guard_entries.name},'FD_FAIL_R'));assert(isscalar(k));bad.unchanged_guard_entries(k).raw_bits_hex='00000000';negative('failure_detector_disabled',bad);
bad=c;k=find(strcmp({bad.unchanged_guard_entries.name},'MPC_USE_HTE'));bad.unchanged_guard_entries(k).raw_bits_hex='00000000';negative('HTE_disabled',bad);
bad=c;k=find(strcmp({bad.unchanged_guard_entries.name},'MC_ROLLRATE_I'));bad.unchanged_guard_entries(k).raw_bits_hex='00000000';negative('rate_I_changed',bad);
bad=c;k=find(strcmp({bad.unchanged_guard_entries.name},'CA_ROTOR0_PX'));bad.unchanged_guard_entries(k)=[];negative('geometry_guard_not_silently_exempted',bad);
bad=c;k=find(strcmp({bad.output_guard_entries.name},'PWM_MAIN_FUNC1'));bad.output_guard_entries(k).raw_bits_hex='00000065';negative('physical_output_guard_changed',bad);
bad=c;k=find(strcmp({bad.output_guard_entries.name},'HIL_ACT_FUNC1'));bad.output_guard_entries(k).raw_bits_hex='00000065';negative('virtual_output_premutation_guard_changed',bad);
bad=c;bad.failure_detector_changes=1;negative('FD_change_scope_rejected',bad);
bad=c;bad.model_changes=1;negative('model_change_rejected',bad);
bad=c;bad.no_route_hover_I_target=2;negative('runtime_I_relabel_rejected',bad);
bad=c;bad.MPC_USE_HTE_required=0;negative('HTE_contract_disabled',bad);
bad=c;bad.geometry67_guards_unchanged=false;negative('geometry_guard_override_rejected',bad);
bad=c;bad.fresh_premutation_original138_required=false;negative('stale_preflight_allowed_rejected',bad);
bad=c;bad.rollback_all_three_exact_original_bits=false;negative('rollback_omitted_rejected',bad);
bad=c;bad.single_send_with_ACK_and_fresh_typed_readback=false;negative('ACK_readback_omitted_rejected',bad);
bad=c;bad.maximum_apply_passes=2;negative('second_apply_pass_rejected',bad);
bad=c;bad.maximum_restore_passes=1;negative('independent_restorer_must_not_be_blocked',bad);
bad=c;bad.guard_scope='IGNORE_GEOMETRY';negative('ambiguous_active_union_rejected',bad);
bad=c;bad.engineering_margin_is_safety_gate=true;negative('engineering_margin_cannot_be_safety_gate',bad);
bad=c;bad.authority_granted=true;negative('file_not_live_authorization',bad);
roundtrip=jsondecode(jsonencode(c));q=m600check.validateNativeHoverTuningContract(roundtrip);
add('JSON_roundtrip_schema_and_binding_pass',q.passed);
result=struct('schema','HOST_NATIVE_HOVER_TUNING_CONTRACT_TESTS_V1','passed',all([tests.passed]), ...
    'case_count',numel(tests),'cases_passed',sum([tests.passed]),'tests',tests,'negative_failures',failures, ...
    'contract',c,'validation',v,'hardware_actions',0,'COM_UDP_actions',0,'parameter_writes',0, ...
    'authority_granted',false);
mkdir(outputDir);f=fopen(fullfile(outputDir,'HOST_RESULT.json'),'w','n','UTF-8');assert(f>=0);g=onCleanup(@()fclose(f)); %#ok<NASGU>
fprintf(f,'%s\n',jsonencode(result,PrettyPrint=true));assert(result.passed,'m600check:TuningContractTestsFailed');
    function add(name,pass),tests(end+1)=struct('name',name,'passed',logical(pass));end %#ok<AGROW>
    function negative(name,bad)
        check=m600check.validateNativeHoverTuningContract(bad);add(name,~check.passed&&~isempty(check.failure)&&check.hardware_actions==0);
        failures(end+1)=struct('test',name,'failure',check.failure); %#ok<AGROW>
    end
end
