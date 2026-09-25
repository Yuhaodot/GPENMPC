function report = verifySensorAlignedHoverPreparation(context, calibrationContract)
% HOST-only evidence consistency, never current-board or flight admission.
report=struct('schema','M600_SENSOR_ALIGNED_NATIVE_HOVER_PREPARATION_CHECK_V1', ...
    'passed',false,'failure','','flight_admission',false,'required_fresh_preflight',true, ...
    'hardware_actions',0,'references_verified',0,'posthoc_performance_gates_added',false, ...
    'offset_compensation_applied',false,'checks',{{}});
try
    must(isstruct(context)&&isscalar(context),'MISSING_PREPARATION_CONTEXT');
    must(isfield(context,'schema')&&strcmp(context.schema,'M600_SENSOR_ALIGNED_NATIVE_HOVER_PREPARATION_V1'),'PREPARATION_SCHEMA');
    required={'schema','scope','observation_run_root','observation_execution_id','required_fresh_preflight', ...
        'flight_admission','staleness_policy','calibration_contract','expected_identity','observation','runtime_binding','references'};
    must(isequal(sort(fieldnames(context)),sort(required(:))),'PREPARATION_FIELDS');
    must(strcmp(context.scope,'BOUND_OFFLINE_PREPARATION_ONLY')&&isequal(context.flight_admission,false)&& ...
        isequal(context.required_fresh_preflight,true),'PREPARATION_CANNOT_GRANT_FLIGHT');
    root=gpenmpc_external_path('sensor_aligned_observation_fixture');
    must(strcmp(context.observation_run_root,root)&& ...
        strcmp(context.observation_execution_id,'SENSOR_ALIGNED_REFERENCE_OBSERVATION')&& ...
        strcmp(context.staleness_policy,'EXACT_RUN_AND_RUNTIME_BINDING_NOT_WALL_CLOCK_AGE'),'STALE_OR_DIFFERENT_OBSERVATION_CONTEXT');
    expected=struct('uid',gpenmpc_device_identity('uid'),'board_version',56, ...
        'commit','6ea3539157ca358c70a515878b77077af7d4611d','flight_custom_version_hex','000000579153A36E');
    must(same(context.expected_identity,expected),'BOARD_IDENTITY_CHANGED');
    v=context.observation;
    must(v.window_completed&&v.duration_contract_s==45&&v.elapsed_s==45,'INCOMPLETE_OBSERVATION_WINDOW');
    must(v.source_reset_count==0&&v.source_reversal_count==0,'OBSERVATION_RESET_OR_REVERSAL');
    must(~v.fatal_present,'OBSERVATION_FATAL'); must(v.safe_finally,'UNSAFE_OBSERVATION_FINALLY');
    must(isstruct(calibrationContract)&&same(calibrationContract,context.calibration_contract),'CALIBRATION_CONTRACT_CHANGED');
    must(strcmp(calibrationContract.schema,'VIRTUAL_SENSOR_MOUNT_CONTRACT_V2_AUTOCAL_OBSERVED')&& ...
        strcmp(calibrationContract.admission_scope,'DISARMED_SENSOR_OBSERVATION_ONLY')&& ...
        ~calibrationContract.flight_admission&&~calibrationContract.parameter_writes_allowed&& ...
        strcmp(calibrationContract.offset_policy,'NATIVE_DYNAMIC_FINITE_OBSERVED_NO_WIRE_COMPENSATION'),'CALIBRATION_RUNTIME_POLICY');
    dynamic={calibrationContract.parameters(strcmp({calibrationContract.parameters.policy},'NATIVE_DYNAMIC_FINITE_BIAS')).name};
    must(isequal(sort(dynamic),sort({'CAL_GYRO2_XOFF','CAL_GYRO2_YOFF','CAL_GYRO2_ZOFF', ...
        'CAL_MAG1_XOFF','CAL_MAG1_YOFF','CAL_MAG1_ZOFF'})),'DYNAMIC_OFFSET_POLICY_CHANGED');
    refs=context.references(:); labels={refs.label};
    must(numel(unique(labels))==numel(labels),'DUPLICATE_REFERENCE');
    anchors={ ...
        'outer_result',201924,'FB6E9A4ECEC74526BBE1EBEDB01B197E7864A2EBDDD4528616D802EE7B03E887'; ...
        'observer_result',15415,'7AA47527DDECA06DC68DEF03F14E8D7BA5AFCCAC004CFFD962AC344040C6FB59'; ...
        'stream_summary',143213,'1C42373A430A356A081BF4D177D4E8C01564EA4A8445A159096705D6431B2585'; ...
        'attitude_summary',114529,'C5E5498429DF5B811B154C7F17B06E1ACCD7856C26A0A0C6CEB3C5699F21CB95'; ...
        'consumed_plan',41615,'9FC9ED781194E6CC255946B6D03788EA62709424243DDA160138CC2468A00D2D'};
    for k=1:size(anchors,1)
        r=reference(anchors{k,1});
        must(r.bytes==anchors{k,2}&&strcmp(r.sha256,anchors{k,3}),['CHANGED_REFERENCE_OBSERVATION_' upper(r.label)]);
    end
    rtLabels={'model_dll','source_model','core_parameters','sensor_frame_encoder','hil_profile','firmware', ...
        'sensor_mount_checker','sensor_mount_host_tests','sensor_calibration_readonly'};
    other={'serial_preflight','serial_postflight','vendor_prefix','vendor_full_log','raw_observation','observation_rows','attitude_rows'};
    must(isequal(sort(labels),sort([anchors(:,1).' other rtLabels])),'REFERENCE_SET_MISMATCH');
    for k=1:numel(refs)
        r=refs(k); d=dir(r.path);
        must(isscalar(d)&&~d.isdir,['MISSING_FILE_' upper(r.label)]);
        must(d.bytes==r.bytes&&strcmp(hashFile(r.path),r.sha256),['FILE_HASH_OR_BYTES_' upper(r.label)]);
        report.references_verified=report.references_verified+1;
    end
    plan=readRef('consumed_plan'); obs=readRef('observer_result'); health=readRef('stream_summary');
    attitude=readRef('attitude_summary'); outer=readRef('outer_result'); pre=readRef('serial_preflight'); post=readRef('serial_postflight');
    must(strcmp(plan.execution_kind,'INITIALIZATION_OBSERVATION_ONLY')&& ...
        same(calibrationContract,plan.virtual_sensor_mount_contract)&& ...
        same(calibrationContract,obs.config.virtual_sensor_mount_contract),'CONSUMED_CONTRACT_CHANGED');
    for k=1:numel(rtLabels)
        label=rtLabels{k}; r=reference(label); pr=plan.references(strcmp({plan.references.label},label));
        must(isscalar(pr)&&same(r,pr),['RUNTIME_REFERENCE_CHANGED_' upper(label)]);
        must(strcmp(context.runtime_binding.([label '_sha256']),r.sha256),['RUNTIME_BINDING_CHANGED_' upper(label)]);
    end
    must(numel(fieldnames(context.runtime_binding))==numel(rtLabels),'RUNTIME_BINDING_FIELDS');
    must(strcmp(context.runtime_binding.model_dll_sha256,'80C95FA673EC462EB3108016A0F53FE4A7A50A7FD9F016DFB57EE9345A09EAE4'),'WRONG_MODEL_DLL');
    for s={pre,post}
        x=s{1}; must(same(identity(x),expected),'SERIAL_IDENTITY_CHANGED');
        must(x.passed&&isempty(x.failure)&&~x.armed&&x.landed_state==1&&x.COM_closed&& ...
            x.physical_output_path_disabled&&x.virtual_output_path_disabled&& ...
            contains(x.pwm_out_status,'not running'),'SERIAL_SAFETY_NOT_CLOSED');
    end
    must(pre.virtual_sensor_mount_check.passed&& ...
        same(pre.virtual_sensor_mount_check.contract,calibrationContract),'OBSERVED_CALIBRATION_NOT_BOUND');
    must(obs.window_completed&&obs.duration_contract_s==45&&obs.elapsed_observation_s==45&& ...
        strcmp(obs.stop_reason,'OBSERVATION_WINDOW_ENDED')&&isempty(obs.failure)&&isempty(obs.first_fatal)&& ...
        isempty(obs.events)&&~obs.source_reset_performed&&~obs.fault_latch_cleared&& ...
        obs.matlab_sockets_closed&&obs.close_attempted&&isempty(obs.close_failure)&&isempty(obs.evidence_capture_failure)&& ...
        ~obs.injected_io&&obs.formal_rows==0&&~obs.flight_result,'OBSERVATION_NOT_COMPLETE_AND_CLEAN');
    zeroCounts={'parameter_writes','mapping_writes','mode_requests','arm_requests','disarm_requests', ...
        'land_requests','task_requests','setpoint_requests','reboot_requests','COM_open','plant_creations','physical_output_actions'};
    for k=1:numel(zeroCounts), must(obs.counts.(zeroCounts{k})==0,'OBSERVATION_AUTHORITY_NONZERO'); end
    must(outer.safe_to_stop_proved&&outer.final_serial_safety_passed&&outer.coptersim_stopped&& ...
        ~outer.safety_recovery_remaining&&isempty(outer.failure)&&outer.emergency_logical_force_disarm_count==0&& ...
        outer.physical_output_actions==0&&outer.host_explicit_flash_reboot_bootloader_requests==0,'OUTER_FINALLY_NOT_SAFE');
    prefix=readRef('vendor_prefix'); full=readRef('vendor_full_log');
    must(prefix.can_start_disarmed_observer&&full.can_start_disarmed_observer&&isempty(full.first_reject)&& ...
        strcmp(prefix.external_proof.observed_dll_sha256,context.runtime_binding.model_dll_sha256), ...
        'VENDOR_PREFIX_NOT_BOUND');
    for k={'vendor_reboot_count','model_stop_count','sim_start_count'}
        must(prefix.(k{1})==full.(k{1}),'VENDOR_RESET_AFTER_OBSERVER_START');
    end
    must(health.window_completed&&health.duration_contract_s==45&&health.observation_elapsed_s==45&& ...
        isempty(health.original_first_fatal)&&isempty(health.transport_fatal)&& ...
        ~health.fault_latch_cleared&&~health.source_reset_performed,'RAW_STREAM_WINDOW_OR_FATAL');
    rawRef=reference('raw_observation');
    must(fileIdentitySame(health.raw_input,rawRef)&&fileIdentitySame(attitude.input_raw_mat,rawRef),'RAW_SOURCE_MISMATCH');
    must(fileIdentitySame(attitude.input_rows,reference('observation_rows'))&& ...
        fileIdentitySame(attitude.output_csv,reference('attitude_rows')),'ROW_SOURCE_MISMATCH');
    for k={'ATTITUDE','LOCAL_POSITION_NED','ESTIMATOR_STATUS'}
        st=health.streams.(k{1}).declared_observation_window;
        must(st.packet_count>1&&st.source_unique_count>1&&st.source_nonfinite_count==0&& ...
            st.source_reversal_count==0&&st.receive_reversal_count==0,['SOURCE_CONTINUITY_' k{1}]);
    end
    ac=health.snapshot_accounting;
    for k={'estimate_source_reversed_count','truth_source_reversed_count','model_source_reversed_count', ...
            'raw_model_source_reversed_count','attitude_source_reversed_count','model_fault_count','armed_count'}
        must(ac.(k{1})==0,['OBSERVATION_' upper(k{1})]);
    end
    for k={'MAV','TRUTH','TIME','TX'}, must(health.raw_overflow_drops.(k{1})==0,'RAW_OVERFLOW'); end
    must(attitude.input_window_completed&&attitude.input_elapsed_s==45&&isempty(attitude.input_first_fatal)&& ...
        isempty(attitude.input_failure)&&~attitude.truth_or_estimate_modified&& ...
        ~attitude.calibration_offsets_compensated&&~attitude.origin_fitting_performed&& ...
        attitude.accounting.rows_removed==0&&attitude.accounting.raw_rows_count==ac.rows&& ...
        attitude.accounting.output_csv_rows==ac.rows,'ATTITUDE_OBSERVATION_CHANGED_OR_CROPPED');
    report.observation=struct('full_window_rows',ac.rows,'all_initialization_rows_preserved',true, ...
        'attitude_statistics',attitude.partitions.full_window, ...
        'identity_in_udp_not_invented',ac.identity_missing_is_not_invented_as_matching, ...
        'identity_basis','READONLY_SERIAL_BEFORE_AND_AFTER__FRESH_LIVE_IDENTITY_STILL_REQUIRED');
    report.passed=true;
    report.classification='PASS_BOUND_HOST_PREPARATION__FRESH_ACTUAL_PREFLIGHT_REQUIRED__NOT_FLIGHT_ADMISSION';
catch err
    report.failure=err.message;
    report.classification='REJECT_HOST_PREPARATION__NO_FLIGHT_ADMISSION';
end
    function r=reference(label)
        ii=find(strcmp(labels,label)); must(numel(ii)==1,['MISSING_REFERENCE_' upper(label)]); r=refs(ii);
    end
    function v=readRef(label)
        r=reference(label); v=jsondecode(fileread(r.path));
    end
    function must(ok,reason)
        if ~isscalar(ok)||~ok, error('m600check:HoverPreparation','%s',reason); end
        report.checks{end+1}=reason;
    end
end
function b=fileIdentitySame(a,r)
b=strcmp(a.path,r.path)&&a.bytes==r.bytes&&strcmp(a.sha256,r.sha256);
end
function v=identity(x)
v=struct('uid',x.uid,'board_version',x.board_version,'commit',x.commit, ...
    'flight_custom_version_hex',x.flight_custom_version_hex);
end
function yes=same(a,b)
% JSON struct vectors may change row/column; no fields or numbers are dropped.
yes=isequaln(normalize(a),normalize(b));
end
function x=normalize(x)
if isstruct(x)
    x=orderfields(x); x=x(:);
    fs=fieldnames(x); for k=1:numel(x), for j=1:numel(fs), x(k).(fs{j})=normalize(x(k).(fs{j})); end, end
elseif iscell(x)
    x=x(:); for k=1:numel(x), x{k}=normalize(x{k}); end
end
end
function h=hashFile(path)
fid=fopen(path,'rb'); if fid<0, error('m600check:PreparationRead','Cannot read %s',path); end
c=onCleanup(@()fclose(fid)); %#ok<NASGU>
md=java.security.MessageDigest.getInstance('SHA-256');
while true, b=fread(fid,1048576,'*uint8'); if isempty(b), break; end, md.update(b); end
h=upper(reshape(dec2hex(typecast(md.digest(),'uint8'),2).',1,[]));
end
