function report=test_rfly_local_streaming_getters(outputRoot,runtimeHoldOnly)
% Test running-window binding with retained getter data and synthetic RLS envelopes.
build=string(fileparts(fileparts(mfilename('fullpath'))));addpath(fullfile(build,'host_runtime'));
if nargin>=2&&runtimeHoldOnly
    if ~isfolder(outputRoot),mkdir(outputRoot);end
    assert(~isfile(fullfile(outputRoot,'RESULT.mat')));
    report=checkRuntimeHold(build,outputRoot);return
end
assert(~isfolder(outputRoot));mkdir(outputRoot);
root=fullfile(gpenmpc_external_path('matlab_noui_original_reader'));
data=load(fullfile(root,'MATLAB_ORIGINAL_GETTERS.mat'));detail=jsondecode(fileread(fullfile(root,'OFFLINE_SENSOR_CONTENT_MATCH.json')));
dll='990850A2F40F3FCC2A6C47E63A4065B60FF49AA39CC4749FF443963B06F2EF7E';
checks=struct('name',{},'pass',{});obj=gpenmpcNative.RflyLocalOriginalGetterBuffer(4096,dll);
base=data.last;records=data.records;reads=data.reads;
for first=1:256:size(records,2)
    last=min(first+255,size(records,2));v=batch(first:last);obj.ingest(v);
end
s=obj.status();check('actual3544_original_records_retained',s.accepted_getters==3544&&s.retained_getters==3544&&~s.failed);
check('live_atomic_counters_not_claimed_coherent',~s.running_snapshot_coherent);
p=detail.mutually_unique_retained_pairs;sensors=zeros(52,numel(p),'uint8');expected=zeros(numel(p),1);
for k=1:numel(p),sensors(:,k)=uint8(sscanf(p(k).equal_sensor52_bytes_hex,'%2x'));expected(k)=p(k).retained_getter_index+1;end
match=gpenmpcNative.matchRflyOriginalGetterSensorBytes(records,sensors);
check('actual749_content_pairs_equal_independent_C_parser',match.mutually_unique_count==749&&isequal(match.getter_index,expected));
task=fullfile(build,'task_packages','cambridge_canonical','MU_CAMBRIDGE_MA_02__CANONICAL_PHYSICAL_TASK.mat');
policy=struct('expected_session_token',26090501,'payload_by_generation_kg',[2.21;1.75;.98;.55;0], ...
    'mass_by_generation_kg',[11.71;11.25;10.48;10.05;9.5]);
ts=struct('path',task,'sha256','B876546B468265F84BD028AC549B06EF8ADDCB25CBA09CF389A08F4B21290A5F', ...
    'configuration_sha256','A859433D0AA774013341444A4B9AE971A34F12B4FB89C0A004B2CB3E013FCEBA', ...
    'environment_policy',policy,'copter_id',1);
ts.cached_asset=gpenmpcNative.RflyLocalTaskAsset(ts);
rls=reshape(readbin(fullfile(build,'rfly_vendor_integration','full_inner_abi','snapshot_wire_fixture','RLS1_SNAPSHOTS.bin')),382,[]);
rx=reads(end)+uint64(1);sourceRows=60;raw=cell(sourceRows,1);
for k=1:sourceRows
    mock=rls(:,k);mock(277:328)=sensors(:,k);mock(351:382)=digest(mock(1:350));
    [value,receipt]=obj.bind(mock,rx+uint64(k),ts,[],[]);raw{k}=value;
    assert(strcmp(value.status,'MISSING_SOURCE_BOUND_TASK_PHASE')&&~value.numerical_inputs_complete);
    g=expected(k);assert(isequal(typecast(value.rotor.original_observation.observed_thrust_n,'uint64'), ...
        typecast(records(273:320,g),'uint64'))&&value.rotor.original_observation.original_host_receive_ns==reads(g));
    assert(receipt.live_rdr_reader_installed&&~receipt.owner.external_owner_quiescence_proven_here ...
        &&~receipt.owner.running_counters_equal_epoch&&~receipt.board_authority);
    obj.retireResolvedSource(value.source.source_generation);
end
s=obj.status();check('60_mock_RLS_to_actual_getter_lag_bit_exact',numel(raw)==60&&s.matched_sources==60);
check('explicit_retirement_accounted_no_raw_overwrite',s.explicitly_retired_getters==expected(60) ...
    &&s.retained_getters+s.explicitly_retired_getters==3544);
check('missing_phase_never_authorized_payload',all(cellfun(@(v)isempty(v.payload)&&~v.numerical_inputs_complete,raw)));
check('single_cached_task_load',ts.cached_asset.FileReads==1);
bad=gpenmpcNative.RflyLocalOriginalGetterBuffer(4096,dll);v=batch(1:10);v.ring_header(81)=1;
reject('sticky_ring_fault',@()bad.ingest(v));check('fault_is_latched',bad.Failed);
reject('fault_cannot_be_recovered_with_good_batch',@()bad.ingest(batch(1:10)));
bad=gpenmpcNative.RflyLocalOriginalGetterBuffer(4096,dll);v=batch(1:10);v.step_status(73)=1;
reject('snapshot_fault',@()bad.ingest(v));
bad=gpenmpcNative.RflyLocalOriginalGetterBuffer(4096,dll);bad.ingest(batch(1:10));v=batch(11:20);v.step_status(17)=bitxor(v.step_status(17),uint8(1));
reject('mixed_owner_nonce',@()bad.ingest(v));
bad=gpenmpcNative.RflyLocalOriginalGetterBuffer(4096,dll);bad.ingest(batch(1:10));
reject('record_gap',@()bad.ingest(batch(12:20)));
bad=gpenmpcNative.RflyLocalOriginalGetterBuffer(4096,dll);bad.ingest(batch(1:10));v=batch(11:20);v.original_read_ns(1)=reads(1)-uint64(1);
reject('original_read_clock_reversal',@()bad.ingest(v));
bad=gpenmpcNative.RflyLocalOriginalGetterBuffer(256,dll);bad.ingest(batch(1:256));
reject('unretired_capacity_overflow',@()bad.ingest(batch(257:260)));
bad=gpenmpcNative.RflyLocalOriginalGetterBuffer(4096,dll);v=batch(1:10);v.snapshot_coherent=true;
reject('pretended_coherent_live_snapshot',@()bad.ingest(v));
bad=gpenmpcNative.RflyLocalOriginalGetterBuffer(4096,dll);v=batch(1:10);v.records(330,1)=1;
reject('actual_record_invalid',@()bad.ingest(v));
% Independently sampled non-equal counters are normal while producer runs.
bad=gpenmpcNative.RflyLocalOriginalGetterBuffer(4096,dll);v=batch(1:10);v.step_status(97:104)=typecast(uint64(3600),'uint8');bad.ingest(v);
check('running_counter_inequality_not_false_fail',~bad.Failed);
check('all_zero_hardware',s.COM==0&&s.plant_runs==0&&~s.board_authority);
report=struct('scope','HOST_REPLAY_ACTUAL_NOUI_RECORDS_WITH_EXPLICIT_MOCK_RLS_ONLY', ...
    'checks',checks,'passed',all([checks.pass]),'actual_records',3544,'actual_sensor_pairs',749,'mock_RLS_rows',60, ...
    'same_live_board_source_proven',false,'hardware_actions',0,'COM',0,'plant_runs',0,'source_sha256',sha(which('gpenmpcNative.RflyLocalOriginalGetterBuffer')));
save(fullfile(outputRoot,'RAW.mat'),'report','raw','s','match');
f=fopen(fullfile(outputRoot,'RESULT.json'),'w','n','UTF-8');assert(f>=0);fprintf(f,'%s\n',jsonencode(report,PrettyPrint=true));fclose(f);
fprintf('Streaming original getter glue %d/%d\n',sum([checks.pass]),numel(checks));assert(report.passed);
    function b=batch(ix),b=base;b.records=records(:,ix);b.original_read_ns=reads(ix);end
    function check(name,value),checks(end+1)=struct('name',name,'pass',logical(value));assert(value,'test:Check','%s',name);end
    function reject(name,f),id='';try,f();catch ex,id=ex.identifier;end;check(name,~isempty(id));end
end
function b=readbin(p),f=fopen(p,'rb');assert(f>=0);c=onCleanup(@()fclose(f));b=fread(f,Inf,'*uint8');end
function h=digest(b),m=java.security.MessageDigest.getInstance('SHA-256');m.update(typecast(b(:),'int8'));h=reshape(typecast(m.digest(),'uint8'),[],1);end
function h=sha(p),h=upper(reshape(dec2hex(digest(readbin(p)),2).',1,[]));end

function report=checkRuntimeHold(build,outputRoot)
% Test retirement and age/order checks with recorded getter bytes
% and synthetic receive times and RLS data.
d=load(fullfile(gpenmpc_external_path('matlab_noui_original_reader'),'MATLAB_ORIGINAL_GETTERS.mat'),'last','records');
b=d.last;
last=find(d.records(329,:)==1&d.records(330,:)==0&d.records(331,:)==0,1);
assert(last>=10);first=last-9;
rls=reshape(readbin(fullfile(build,'rfly_vendor_integration','full_inner_abi','snapshot_wire_fixture','RLS1_SNAPSHOTS.bin')),382,[]);
for k=1:3
    rls(236:350,k)=0;rls(351:382,k)=digest(rls(1:350,k));
    gpenmpcNative.RflyLocalSnapshotDecoder(rls(:,k),uint64(1));
end
ts=struct('path',fullfile(build,'task_packages','cambridge_canonical','MU_CAMBRIDGE_MA_02__CANONICAL_PHYSICAL_TASK.mat'), ...
 'sha256','B876546B468265F84BD028AC549B06EF8ADDCB25CBA09CF389A08F4B21290A5F', ...
 'configuration_sha256','A859433D0AA774013341444A4B9AE971A34F12B4FB89C0A004B2CB3E013FCEBA', ...
 'environment_policy',struct(),'copter_id',1);
ts.cached_asset=gpenmpcNative.RflyLocalTaskAsset(ts);
% Warm parsing and hashing before the timed retirement fixture.
for fixturePass=1:2
    obj=gpenmpcNative.RflyLocalOriginalGetterBuffer(256,'990850A2F40F3FCC2A6C47E63A4065B60FF49AA39CC4749FF443963B06F2EF7E');obj.useRuntimeStateOnly();
    t=gpenmpcNative.rflyOriginalHostMonotonicNs();
    for from=1:256:first-1
        ix=from:min(from+255,first-1);b.records=d.records(:,ix);b.original_read_ns=t-uint64(last-ix+1).';obj.observeOnly(b);
    end
    b.records=d.records(:,first:last);b.original_read_ns=t-uint64(10:-1:1).';obj.ingest(b);
    [v,r]=obj.bind(rls(:,1),t,ts,[],[]);assert(~isempty(v.rotor));
end
obj.retireResolvedSource(v.source.source_generation);s=obj.status();
assert(s.retained_getters==1&&s.explicitly_retired_getters==9);
[v2,r2]=obj.bind(rls(:,2),t+uint64(1),ts,[],[]);
assert(~isempty(v2.rotor)&&isequal(r.original_window.records(:,end),r2.original_window.records));
assert(v2.rotor.original_observation.original_host_receive_ns==v.rotor.original_observation.original_host_receive_ns);
obj.retireResolvedSource(v2.source.source_generation);s=obj.status();assert(s.retained_getters==1&&s.explicitly_retired_getters==9);
pause(.06);[stale,~]=obj.bind(rls(:,3),t+uint64(2),ts,[],[]);assert(isempty(stale.rotor)&&~stale.numerical_inputs_complete);
id='';try,obj.bind(rls(:,2),t+uint64(1),ts,[],[]);catch ex,id=ex.identifier;end
assert(strcmp(id,'gpenmpcNative:GetterSourceOrder'));
report=struct('passed',true,'checks',6,'scope','MOCK_RX_CLOCK_RLS__RETAINED_ACTUAL_GETTER_BYTES__NO_HARDWARE', ...
 'original_getter_time_unchanged',true,'expired_50ms_still_missing',true,'repeated_source_rejected',true,'COM',0,'board_actions',0);
save(fullfile(outputRoot,'RESULT.mat'),'report');disp(jsonencode(report));
end
