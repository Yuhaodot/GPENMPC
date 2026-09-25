function report=test_rfly_local_original_inputs(outputRoot)
% Test source composition using retained DLL records and synthetic RLS/ACK metadata.
build=string(fileparts(fileparts(mfilename('fullpath'))));addpath(fullfile(build,'host_runtime'),fullfile(build,'matlab_validation'));
assert(~isfolder(outputRoot));mkdir(outputRoot);checks=struct('name',{},'pass',{});negatives=struct('name',{},'identifier',{});
e=fullfile(gpenmpc_external_path('official_noui_step_snapshot'));
paths=[fullfile(e,'GETTER_RING_RECORDS.bin'),fullfile(e,'GETTER_RING_READ_QPC.bin'),fullfile(e,'GETTER_RING_SECTION_FINAL.bin'), ...
    fullfile(e,'STEP_SNAPSHOT_STATUS_FINAL.bin'),fullfile(e,'RAW_STREAMS.bin'),fullfile(e,'OFFLINE_SENSOR_CONTENT_MATCH.json'), ...
    fullfile(build,'rfly_vendor_integration','full_inner_abi','snapshot_wire_fixture','RLS1_SNAPSHOTS.bin')];
before=arrayfun(@shaFile,paths);
records=reshape(readbin(paths(1)),336,[]);qpc=typecast(readbin(paths(2)),'uint64');
detail=jsondecode(fileread(paths(6)));pairs=detail.mutually_unique_retained_pairs;
sensor=zeros(52,numel(pairs),'uint8');expected=zeros(numel(pairs),1);
for k=1:numel(pairs),sensor(:,k)=uint8(sscanf(pairs(k).equal_sensor52_bytes_hex,'%2x'));expected(k)=pairs(k).retained_getter_index+1;end
match=gpenmpcNative.matchRflyOriginalGetterSensorBytes(records,sensor);
save(fullfile(outputRoot,'MATCH_DIAGNOSTIC.mat'),'match','expected','sensor');
disp(struct('ng',size(records,2),'nw',numel(pairs),'unique',match.mutually_unique_count, ...
    'max_count',max(match.wire_match_counts),'equal_indices',isequal(match.getter_index,expected)));
check('actual3559getter749wire_full_window_matches_exact_retained_C_parser',size(records,2)==3559&&numel(pairs)==749 ...
    &&match.mutually_unique_count==749&&isequal(match.getter_index,expected)&&all(match.wire_match_counts==1));
check('original_QPC_retained_not_relabelled_as_BOARD_or_Java_ns',numel(qpc)==3559&&all(qpc(2:end)>=qpc(1:end-1)));
check('zero_time_or_ordinal_filters',~match.time_or_ordinal_used&&~match.unseen_history_or_future_unique);
m=gpenmpcNative.matchRflyOriginalGetterSensorBytes([records,records(:,expected(1))],sensor);
check('second_identical_getter_invalidates_that_pair',m.getter_index(1)==0&&m.wire_match_counts(1)==2&&m.mutually_unique_count==748);
m=gpenmpcNative.matchRflyOriginalGetterSensorBytes(records,[sensor,sensor(:,1)]);
check('two_source_packets_same_content_are_not_bijective',m.getter_index(1)==0&&m.getter_index(end)==0&&m.mutually_unique_count==748);
changed=sensor;changed(1,1)=bitxor(changed(1,1),uint8(1));m=gpenmpcNative.matchRflyOriginalGetterSensorBytes(records,changed);
check('one_sensor_bit_not_tolerance_matched',m.getter_index(1)==0&&m.mutually_unique_count==748);
task=fullfile(build,'task_packages','cambridge_canonical','MU_CAMBRIDGE_MA_02__CANONICAL_PHYSICAL_TASK.mat');
taskSha='B876546B468265F84BD028AC549B06EF8ADDCB25CBA09CF389A08F4B21290A5F';
policy=struct('expected_session_token',26090501,'payload_by_generation_kg',[2.21;1.75;.98;.55;0], ...
    'mass_by_generation_kg',[11.71;11.25;10.48;10.05;9.5]);
taskSource=struct('path',task,'sha256',taskSha,'configuration_sha256', ...
    'A859433D0AA774013341444A4B9AE971A34F12B4FB89C0A004B2CB3E013FCEBA','environment_policy',policy,'copter_id',1);
window=struct('records',records,'original_read_ns',uint64(9000000000)+(uint64(1):uint64(3559)).'*uint64(1000), ...
    'original_read_clock_domain','EXPLICIT_HOST_FIXTURE_NS_NOT_RETAINED_QPC','original_qpc_retained',qpc, ...
    'ring_section',readbin(paths(3)),'step_status',readbin(paths(4)), ...
    'instrumented_dll_sha256','990850A2F40F3FCC2A6C47E63A4065B60FF49AA39CC4749FF443963B06F2EF7E');
rls=reshape(readbin(paths(7)),382,[]);rx=uint64(10000000000)+(uint64(1):uint64(60)).'*uint64(1000);
[unmatched,originalReceipt]=gpenmpcNative.bindRflyLocalOriginalInputs(rls,rx,window,taskSource,cell(60,1),cell(60,1));
check('actual60RLS_are_different_run_not_forced_to_match_actualNoUI',all(cellfun(@(v)strcmp(v.status,'NO_UNIQUE_SENSOR_CONTENT_MATCH'),unmatched)));
% Compose an RLS-schema fixture with sensor52 wire data for the unit test.
mockRls=rls(:,1);mockRls(277:328)=sensor(:,1);mockRls(351:382)=digest(mockRls(1:350));mockRx=rx(1);
[b,~]=gpenmpcNative.bindRflyLocalOriginalInputs(mockRls,mockRx,window,taskSource,{[]},{[]});v=b{1};g=expected(1);
lag=littleValue(records(273:320,g),'double');
check('MOCK_RLS_envelope_actual_sensor_lag_bits_original_stamps',strcmp(v.status,'MISSING_SOURCE_BOUND_TASK_PHASE') ...
    &&isequal(typecast(v.rotor.original_observation.observed_thrust_n,'uint64'),typecast(lag,'uint64')) ...
    &&v.rotor.original_observation.original_host_receive_ns==window.original_read_ns(g) ...
    &&v.rotor.original_observation.original_board_ingress_us==gpenmpcNative.RflyLocalSnapshotDecoder(mockRls,mockRx).original_receipt_us ...
    &&~v.numerical_inputs_complete&&~v.control_authority);
t=load(task,'physicalTask');r=t.physicalTask.reference;phase=struct('source_rls_sha256',digest(mockRls), ...
    'saved_task_time_s',double(r.global_time_s(1)),'leg_index',double(r.leg_index(1)), ...
    'original_bytes',uint8('EXPLICIT_MOCK_PHASE_OWNER_NOT_ACTUAL_BOARD_FEEDBACK').');
[b,~]=gpenmpcNative.bindRflyLocalOriginalInputs(mockRls,mockRx,window,taskSource,{phase},{[]});
check('missing_actual_ack_never_becomes_payload',strcmp(b{1}.status,'MISSING_SAME_STEP_ACCEPTED_ENVIRONMENT')&&isempty(b{1}.payload) ...
    &&isequal(b{1}.wind.estimate_xy_mps,double(r.wind_estimate_xy_mps(1,:)).'));
time=littleValue(records(265:272,g),'double');env=makeMockEnvironment(time,phase,r,policy);
[b,receipt]=gpenmpcNative.bindRflyLocalOriginalInputs(mockRls,mockRx,window,taskSource,{phase},{env});
cached=taskSource;cached.cached_asset=gpenmpcNative.RflyLocalTaskAsset(taskSource);
[cachedBound,cachedReceipt]=gpenmpcNative.bindRflyLocalOriginalInputs(mockRls,mockRx,window,cached,{phase},{env});
check('cached_task_identical_binding_no_hot_path_file_reload',isequaln(b,cachedBound) ...
    &&isequaln(receipt,cachedReceipt)&&cached.cached_asset.FileReads==1);
badCached=cached;badCached.sha256=repmat('0',1,64);
reject('cached_task_identity_drift_rejected',@()call(window,badCached,phase,env));
check('MOCK_ACK_actual_encoder_decoder_full_numeric_shape',b{1}.numerical_inputs_complete ...
    &&isequal(b{1}.wind.estimate_xy_mps,double(r.wind_estimate_xy_mps(1,:)).') ...
    &&isequal(b{1}.accepted_plant_wind_xy_mps,double(r.actual_wind_xy_mps(1,:)).') ...
    &&b{1}.payload.payload_kg==r.payload_kg(1)&&~b{1}.control_authority&&~b{1}.transport_installed);
ix=find(r.leg_index(1:end-1)==1&r.leg_index(2:end)==1&any(diff(r.wind_estimate_xy_mps)~=0,2),1,'first');assert(~isempty(ix));
fraction=phase;fraction.saved_task_time_s=mean(r.global_time_s(ix:ix+1));
fraction.original_bytes=[phase.original_bytes;typecast(fraction.saved_task_time_s,'uint8').'];
fractionEnv=makeMockEnvironment(time,fraction,r,policy);
[fb,~]=gpenmpcNative.bindRflyLocalOriginalInputs(mockRls,mockRx,window,taskSource,{fraction},{fractionEnv});
windOracle=interp1(double(r.global_time_s),double(r.wind_estimate_xy_mps),fraction.saved_task_time_s,"linear").';
actualOracle=interp1(double(r.global_time_s),double(r.actual_wind_xy_mps),fraction.saved_task_time_s,"linear").';
check('canonical_fractional_task_both_linear_not_left_hold',fb{1}.numerical_inputs_complete ...
    &&isequal(fb{1}.wind.estimate_xy_mps,windOracle)&&isequal(fb{1}.accepted_plant_wind_xy_mps,actualOracle) ...
    &&~isequal(windOracle,double(r.wind_estimate_xy_mps(ix,:)).'));
held=fractionEnv;held.original_frame232(49:64)=typecast(windOracle,'uint8');
reject('estimator_value_cannot_replace_actual_plant_wind',@()call(window,taskSource,fraction,held));
wrong=phase;wrong.source_rls_sha256(1)=bitxor(wrong.source_rls_sha256(1),uint8(1));reject('wrong_exact_source',@()call(window,taskSource,wrong,env));
wrong=phase;wrong.leg_index=2;reject('wrong_task_leg',@()call(window,taskSource,wrong,env));
bad=taskSource;bad.sha256=repmat('0',1,64);reject('wrong_task_bytes',@()call(window,bad,phase,env));
bad=window;bad.step_status(73)=1;reject('sticky_step_fault',@()call(bad,taskSource,phase,env));
bad=window;bad.ring_section(57)=1;reject('ring_dropped',@()call(bad,taskSource,phase,env));
bad=window;bad.step_status(17)=bitxor(bad.step_status(17),uint8(1));reject('different_owner_nonce',@()call(bad,taskSource,phase,env));
bad=window;bad.records(329,g)=0;reject('matched_lag_invalid',@()call(bad,taskSource,phase,env));
bad=window;bad.original_read_ns(2)=bad.original_read_ns(1)-uint64(1);reject('original_read_time_reversed',@()call(bad,taskSource,phase,env));
bad=env;bad.original_frame232(41)=bitxor(bad.original_frame232(41),uint8(1));reject('accepted_payload_not_frame',@()call(window,taskSource,phase,bad));
bad=env;bad.original_frame232(49)=bitxor(bad.original_frame232(49),uint8(1));reject('actual_plant_wind_mismatch',@()call(window,taskSource,phase,bad));
bad=makeMockEnvironment(time+.01,phase,r,policy);reject('nearby_diagnostic_not_same_accepted_step',@()call(window,taskSource,phase,bad));
reject('reused_source_cannot_form_new_window',@()gpenmpcNative.bindRflyLocalOriginalInputs([mockRls,mockRls],[mockRx;mockRx],window,taskSource,{phase,phase},{env,env}));
check('all_original_input_files_unchanged',isequal(before,arrayfun(@shaFile,paths)));
report=struct('scope','ACTUAL_RETAINED749_CONTENT_PAIRS_WITH_EXPLICIT_MOCK_RLS_PHASE_NS_ACK_COMPOSITION', ...
    'checks',numel(checks),'failed',sum(~[checks.pass]),'actual_getters',3559,'actual_wire_pairs',749, ...
    'actual_C_RLS_unmatched_different_run',60,'actual_board_RLS_to_DLL_proven',false,'actual_task_ACK_proven',false, ...
    'checks_detail',checks,'negative_cases',negatives, ...
    'original_paths',paths,'original_sha256',before,'COM',0,'plant_runs',0);
save(fullfile(outputRoot,'RAW.mat'),'report','receipt','originalReceipt','match','b');
f=fopen(fullfile(outputRoot,'RESULT.json'),'w');assert(f>=0);c=onCleanup(@()fclose(f));fprintf(f,'%s\n',jsonencode(report,PrettyPrint=true));clear c
disp(jsonencode(rmfield(report,{'checks_detail','negative_cases','original_paths','original_sha256'})));assert(report.failed==0);
    function call(w,ts,p,e2),gpenmpcNative.bindRflyLocalOriginalInputs(mockRls,mockRx,w,ts,{p},{e2});end
    function check(name,yes),checks(end+1)=struct('name',name,'pass',logical(yes));assert(yes,'test:Check','%s',name);end
    function reject(name,f),id='';try,f();catch ex,id=ex.identifier;end;negatives(end+1)=struct('name',name,'identifier',id);check(name,~isempty(id));end
end
function e=makeMockEnvironment(time,phase,r,policy)
actual=interp1(double(r.global_time_s),double(r.actual_wind_xy_mps),phase.saved_task_time_s,"linear").';
v=struct('schema_version',2,'generation',1,'source_io_time_s',0,'task_reference_time_s',phase.saved_task_time_s, ...
    'payload_kg',double(r.payload_kg(1)),'wind_ned_xy_mps',actual,'mission_phase',0, ...
    'reference_jet_ned',zeros(12,1),'payload_generation',0,'task_clock_paused',true,'session_token',policy.expected_session_token, ...
    'board_min_rx_io_time_s',0,'board_valid_flags',15,'landed_state',1,'continuity_epoch',1,'service_release_generation',0);
frame=gpenmpcTaskIo.encodePlantEnvironmentV2(v,1,policy.payload_by_generation_kg(1));
terrain=zeros(32,1);terrain(3)=time;terrain(5)=1;terrain(7)=1;terrain(26)=1;
ack=struct('valid',true,'task_env_failed',false,'failure_code',0,'session_token',policy.expected_session_token, ...
    'applied_frame_generation',1,'applied_payload_generation',0,'actual_payload_kg',v.payload_kg,'actual_total_mass_kg',policy.mass_by_generation_kg(1));
p=m600check.encodeCopterSimDeliveryDiagnostics(terrain,ack,false);
bytes=[reshape(typecast(int32([1234567890,1]),'uint8'),[],1);reshape(typecast(p,'uint8'),[],1)];
e=struct('original_frame232',frame(:),'original_diagnostic264',bytes,'original_frame_send_ns',uint64(1), ...
    'original_diagnostic_receive_ns',uint64(2),'provenance','EXPLICIT_HOST_MOCK_ACK_NOT_ACTUAL_DLL');
end
function b=readbin(p),f=fopen(p,'rb');assert(f>=0);c=onCleanup(@()fclose(f));b=fread(f,Inf,'*uint8');end
function h=digest(b),m=java.security.MessageDigest.getInstance('SHA-256');m.update(typecast(b(:),'int8'));h=reshape(typecast(m.digest(),'uint8'),[],1);end
function s=shaFile(p),s=string(upper(reshape(dec2hex(digest(readbin(p)),2).',1,[])));end
function v=littleValue(b,k),v=typecast(b(:),k);[~,~,e]=computer;if e=='B',v=swapbytes(v);end;v=v(:);end
