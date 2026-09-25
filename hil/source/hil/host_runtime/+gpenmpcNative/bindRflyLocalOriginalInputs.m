function [bound,receipt]=bindRflyLocalOriginalInputs(rls,originalRlsReceiveNs,window,taskSource,phases,environments,runtimeStateOnly)
if nargin<7,runtimeStateOnly=false;end
% Bind retained numerical sources to rotor, phase and environment records.
% window contains RDR336/read_ns, ring_section86176, SSS144, DLL hash and clock domain.
% The owner retains read provenance; hashes alone do not authenticate input bytes.
% phases contains source_rls_sha256, saved_task_time_s, leg_index and original_bytes.
% environments contains diagnostic264 and matching ENV232 records per source.
% Report missing evidence for empty entries.
assert(isa(rls,'uint8')&&size(rls,1)==382&&size(rls,2)>0&&size(rls,2)<=8192);
n=size(rls,2);assert(isa(originalRlsReceiveNs,'uint64')&&numel(originalRlsReceiveNs)==n&&all(originalRlsReceiveNs>0));
assert(iscell(phases)&&numel(phases)==n&&iscell(environments)&&numel(environments)==n);
assert(all(isfield(window,{'records','original_read_ns','original_read_clock_domain','ring_section','step_status','instrumented_dll_sha256'})));
records=window.records;assert(isa(records,'uint8')&&size(records,1)==336&&size(records,2)>0&&size(records,2)<=8192);
ng=size(records,2);assert(isa(window.original_read_ns,'uint64')&&numel(window.original_read_ns)==ng&&all(window.original_read_ns>0) ...
    &&all(window.original_read_ns(2:end)>=window.original_read_ns(1:end-1))&&strlength(string(window.original_read_clock_domain))>0);
% Require the StepCompleteSnapshotStore implementation for atomic step/getter
% pairing, in addition to the RDR1 packet metadata.
assert(any(strcmpi(window.instrumented_dll_sha256,{'990850A2F40F3FCC2A6C47E63A4065B60FF49AA39CC4749FF443963B06F2EF7E', ...
    '9F55AB72F75C3987BD5FB3F612276BDA6DE792E7367EE9F561FC55A75B8D72BD', ...
    'D536EACE85EBA30A6CE07EF5B38FF108B132C9E3B230A3C46E93F3E1C9CA6E12'})), ...
    'gpenmpcNative:LocalInputDll','Unreviewed observer implementation.');
[owner,rawCounters]=validateOwner(window,ng);
assert(isstruct(taskSource)&&all(isfield(taskSource,{'path','sha256','configuration_sha256','environment_policy','copter_id'})));
if isfield(taskSource,'cached_asset')
    assert(isa(taskSource.cached_asset,'gpenmpcNative.RflyLocalTaskAsset'),'gpenmpcNative:LocalInputCachedTask');
    [r,taskDigest]=taskSource.cached_asset.read(taskSource);
else
    asset=gpenmpcNative.RflyLocalTaskAsset(taskSource);[r,taskDigest]=asset.read(taskSource);
end
decoded=cell(n,1);sensor=zeros(52,n,'uint8');
for k=1:n
    decoded{k}=gpenmpcNative.RflyLocalSnapshotDecoder(rls(:,k),originalRlsReceiveNs(k));sensor(:,k)=decoded{k}.original_sensor52;
    if k>1
        a=decoded{k-1};b=decoded{k};assert(isequal(a.identity,b.identity)&&b.source_generation>a.source_generation ...
            &&b.original_sample_us>a.original_sample_us&&b.original_receipt_us>=a.original_receipt_us ...
            &&originalRlsReceiveNs(k)>=originalRlsReceiveNs(k-1),'gpenmpcNative:LocalInputSourceOrder','Repeated or reversed source window.');
    end
end
if runtimeStateOnly
    matches=struct('getter_index',zeros(n,1),'semantics','LATEST_ACTUAL_MODEL_OBSERVATION_AVAILABLE_AT_DECISION_NOT_EXACT_HIL_ASSOCIATION');
    assert(isfield(window,'runtime_decision_ns')&&isa(window.runtime_decision_ns,'uint64'));
    for k=1:n
        assert(decoded{k}.endpoint_event_ordinal==0,'gpenmpcNative:RuntimeProfileMismatch');
        g=find(window.original_read_ns<=window.runtime_decision_ns,1,'last');
        if ~isempty(g)&&window.runtime_decision_ns-window.original_read_ns(g)<=uint64(50000000)
            matches.getter_index(k)=g;
        end
    end
else
    matches=gpenmpcNative.matchRflyOriginalGetterSensorBytes(records,sensor);
end
bound=cell(n,1);
for k=1:n
    s=decoded{k};source=sourceKey(s);v=struct('schema','RFLY_LOCAL_ORIGINAL_INPUT_BINDING_V1', ...
        'status','NO_UNIQUE_SENSOR_CONTENT_MATCH','source',source,'rotor',[],'payload',[],'wind',[], ...
        'numerical_inputs_complete',false,'control_authority',false,'full_ekf_lineage_proven',false, ...
        'clock_mapping_established',false,'transport_installed',false,'matching_scope','SUPPLIED_RDR_RLS_WINDOW_ONLY');
    g=matches.getter_index(k);if g==0,bound{k}=v;continue;end
    b=records(:,g);generation=littleValue(b(9:16),'uint64');session=littleValue(b(17:24),'uint64');
    time=littleValue(b(265:272),'double');lag=littleValue(b(273:320),'double');
    assert(generation>0&&session>0&&isfinite(time)&&time>=0&&all(isfinite(lag))&&all(lag>=0) ...
        &&b(329)==1&&b(330)==0&&b(331)==0,'gpenmpcNative:LocalInputGetter','Matched original step observation failed.');
    association=digest([uint8('RLA1').';rls(:,k);b;digest(window.ring_section);digest(window.step_status); ...
        hashBytes(window.instrumented_dll_sha256);be(window.original_read_ns(g));be(originalRlsReceiveNs(k))]);
    v.rotor=struct('source',source,'value_kind','OriginalPlantLagState','original_observation', ...
        struct('observed_thrust_n',lag,'dll_generation',generation,'dll_session',session, ...
        'original_host_receive_ns',window.original_read_ns(g),'original_board_ingress_us',s.original_receipt_us, ...
        'original_sim_time_s',time,'original_observation_sha',digest(b)), ...
        'verified_association_receipt_sha256',association, ...
        'association_semantics','BIDIRECTIONAL_UNIQUE_SENSOR_CONTENT_WITHIN_SUPPLIED_WINDOW', ...
        'original_read_clock_domain',window.original_read_clock_domain);
    if runtimeStateOnly
        v.matching_scope=matches.semantics;
        v.rotor.association_semantics=matches.semantics;
    end
    v.status='MISSING_SOURCE_BOUND_TASK_PHASE';p=phases{k};
    if isempty(p),bound{k}=v;continue;end
    assert(isstruct(p)&&all(isfield(p,{'source_rls_sha256','saved_task_time_s','leg_index','original_bytes'})) ...
        &&isequal(hashBytes(p.source_rls_sha256),digest(rls(:,k)))&&isa(p.original_bytes,'uint8')&&~isempty(p.original_bytes) ...
        &&isscalar(p.saved_task_time_s)&&isfinite(p.saved_task_time_s)&&p.saved_task_time_s>=0 ...
        &&p.saved_task_time_s<=r.global_time_s(end),'gpenmpcNative:LocalInputPhase','Task phase must bind this exact source.');
    row=find(r.global_time_s<=p.saved_task_time_s,1,'last');
    assert(r.leg_index(row)==p.leg_index,'gpenmpcNative:LocalInputLeg','Saved task row belongs to another leg.');
    windSource=r;if isfield(taskSource,'cached_asset'),windSource=taskSource.cached_asset;end
    [actualWind,windEstimate,windSelection]=gpenmpcNative.canonicalSavedTaskWindAt(windSource,p.saved_task_time_s,p.leg_index);
    schedule=digest([uint8('RLT1').';taskDigest;be(uint64(row));be(p.saved_task_time_s);digest(p.original_bytes);digest(rls(:,k))]);
    v.wind=struct('source',source,'original_estimate_evidence_sha256',schedule,'original_estimate_generation',uint64(row), ...
        'estimate_xy_mps',windEstimate,'semantics','CANONICAL_LINEAR_TASK_ESTIMATE_NOT_PLANT_WIND');
    v.wind_selection=windSelection;
    v.status='MISSING_SAME_STEP_ACCEPTED_ENVIRONMENT';e=environments{k};d=[];
    if isa(e,'gpenmpcNative.RflyLocalEnvironmentLedger')
        [e,d]=e.resolve(time,runtimeStateOnly,taskSource.copter_id,taskSource.environment_policy);
        environments{k}=e;
    end
    if isempty(e),bound{k}=v;continue;end
    assert(isstruct(e)&&all(isfield(e,{'original_frame232','original_diagnostic264','original_frame_send_ns','original_diagnostic_receive_ns'})) ...
        &&isa(e.original_frame232,'uint8')&&numel(e.original_frame232)==232 ...
        &&isa(e.original_frame_send_ns,'uint64')&&isscalar(e.original_frame_send_ns)&&e.original_frame_send_ns>0 ...
        &&isa(e.original_diagnostic_receive_ns,'uint64')&&isscalar(e.original_diagnostic_receive_ns) ...
        &&e.original_diagnostic_receive_ns>=e.original_frame_send_ns,'gpenmpcNative:LocalInputEnvironment','Original frame and ACK evidence missing.');
    f=littleValue(e.original_frame232(9:232),'double');head=littleValue(e.original_frame232(1:8),'uint32');
    if isempty(d)
        % Raw/offline callers have no validated owner cache.
        d=m600check.decodeCopterSimDeliveryDiagnostics(e.original_diagnostic264,taskSource.copter_id,NaN,taskSource.environment_policy);
    end
    a=d.environment_extension;
    held=isfield(e,'binding_semantics')&&strcmp(e.binding_semantics,'EXACT_OR_BRACKETED_MONOTONIC_APPLIED_FRAME_V1');
    runtimeHeld=runtimeStateOnly&&isfield(e,'binding_semantics')&& ...
        strcmp(e.binding_semantics,'LATEST_CAUSAL_APPLIED_ENVIRONMENT_WITH_ORIGINAL_250MS_EXPIRY');
    sameStep=sameBits(a.same_model_time_s,time);
    if held&&~sameStep
        z=m600check.decodeCopterSimDeliveryDiagnostics(e.original_previous_diagnostic264,taskSource.copter_id,NaN,taskSource.environment_policy);
        before=z.environment_extension;
        sameStep=z.packet_valid&&before.mass_ack_valid&&before.same_model_time_s<time&&time<a.same_model_time_s ...
            &&before.applied_frame_generation==a.applied_frame_generation&&before.applied_payload_generation==a.applied_payload_generation ...
            &&before.session_token==a.session_token&&sameBits(before.actual_payload_kg,a.actual_payload_kg) ...
            &&e.original_frame_send_ns<=e.original_previous_diagnostic_receive_ns ...
            &&e.original_previous_diagnostic_receive_ns<=e.original_diagnostic_receive_ns;
    end
    if runtimeHeld
        sameStep=a.same_model_time_s<=time&&time-f(3)<=.25&&time>=f(3);
    end
    windAtFrame=actualWind;phaseMatches=sameBits(f(4),p.saved_task_time_s);
    if held||runtimeHeld
        % Retain the phase latency of the held environment.
        % Control uses the separately selected wind estimate.
        frameRow=find(r.global_time_s<=f(4),1,'last');
        assert(~isempty(frameRow)&&r.leg_index(frameRow)==p.leg_index&&f(4)<=p.saved_task_time_s, ...
            'gpenmpcNative:LocalInputEnvironmentPhase');
        windAtFrame=gpenmpcNative.canonicalSavedTaskWindAt(windSource,f(4),p.leg_index);
        phaseMatches=true;
    end
    windMatches=sameBits(f(6:7),windAtFrame);
    if isfield(taskSource,'manual_disturbance')&&isequal(taskSource.manual_disturbance,true)
        % The RC environment carries applied gust: .7 periodic plus .9 clipped random per axis.
        % Validate generation, session, mass and time separately from the control estimate.
        windMatches=all(abs(f(6:7)-windAtFrame)<=1.6+8*eps(max(1,max(abs(windAtFrame)))));
    end
    assert(d.packet_valid&&a.mass_ack_valid&&head(1)==1234567897&&head(2)==taskSource.copter_id&&all(isfinite(f)) ...
        &&f(1)==2&&f(2)==a.applied_frame_generation&&f(21)==a.applied_payload_generation&&f(23)==a.session_token ...
        &&sameStep&&sameBits(a.actual_payload_kg,f(5)) ...
        &&sameBits(f(5),double(r.payload_kg(row)))&&phaseMatches ...
        &&windMatches, ...
        'gpenmpcNative:LocalInputEnvironmentMismatch','ACK/time/frame/task payload or actual plant wind mismatch.');
    % commitPlantEnvironmentV2 installs payload AND wind under this applied
    % frame generation. Diagnostic alone lacks wind, so retain original frame.
    v.payload=struct('source',source,'task_sha256',taskDigest, ...
        'original_schedule_evidence_sha256',digest([schedule;e.original_frame232(:);e.original_diagnostic264(:)]), ...
        'original_schedule_generation',uint64(row),'payload_kg',a.actual_payload_kg);
    v.accepted_plant_wind_xy_mps=f(6:7);v.task_row=uint64(row);v.leg_index=uint64(p.leg_index);
    v.actual_environment_task_time_s=f(4);v.controller_estimate_task_time_s=p.saved_task_time_s;
    v.environment_phase_lag_s=p.saved_task_time_s-f(4);v.environment_exact_step_diagnostic=sameBits(a.same_model_time_s,time);
    v.status='NUMERICALLY_BOUND_ORIGINAL_INPUTS_NOT_AUTHORITY';v.numerical_inputs_complete=true;bound{k}=v;
end
receipt=struct('schema','RFLY_LOCAL_ORIGINAL_INPUT_WINDOW_V1','matching',matches,'owner',owner,'original_counters',rawCounters, ...
    'original_rls_bytes',rls,'original_rls_receive_ns',originalRlsReceiveNs,'original_window',window, ...
    'original_phase_receipts',{phases},'original_environment_receipts',{environments},'task_sha256',taskDigest, ...
    'live_rdr_reader_installed',isfield(window,'streaming_snapshot')&&window.streaming_snapshot, ...
    'board_binding_transport_installed',false,'task_phase_feedback_wire_installed',false, ...
    'unseen_history_unique',false,'original_timestamps_renewed',false,'board_authority',false);
end
function [o,c]=validateOwner(w,n)
if isfield(w,'streaming_snapshot')&&isequal(w.streaming_snapshot,true)
    [o,c]=gpenmpcNative.validateRflyOriginalStreamingWindow(w,n);return
end
b=w.ring_section;s=w.step_status;assert(isa(b,'uint8')&&numel(b)==86176&&isa(s,'uint8')&&numel(s)==144);
u=littleValue(b(1:24),'uint32');h=littleValue(b(33:72),'uint64');pid=littleValue(s(25:28),'uint32');nonce=littleValue(s(17:24),'uint64');
assert(isequal(u(1:5),uint32([hex2dec('31524452');1;86176;336;256]))&&u(6)>0 ...
    &&pid==u(6)&&nonce==h(5)&&nonce>0&&h(1)==h(2)&&h(3)==h(1)&&h(4)==0&&h(1)>=uint64(n) ...
    &&littleValue(b(29:32),'uint32')==0,'gpenmpcNative:LocalInputRing','Wrong owner, ring loss or unfinished record read.');
errors=littleValue(b(81:84),'uint32');assert(errors==0||(errors==4&&littleValue(b(129:136),'uint64')==1), ...
    'gpenmpcNative:LocalInputRing','Original ring error cannot grant input.');
assert(isequal(littleValue(s(1:16),'uint32'),uint32([hex2dec('31535353');1;144;10]))&&littleValue(s(57:64),'uint64')==0 ...
    &&littleValue(s(33:40),'uint64')==littleValue(s(49:56),'uint64')&&littleValue(s(41:48),'uint64')==0, ...
    'gpenmpcNative:LocalInputSnapshotStatus','Snapshot status owner/mirror inconsistency.');
c=littleValue(s(65:144),'uint64');assert(c(1)==1&&c(2)==0&&c(3)>=1&&c(4)==c(5)&&c(5)==c(6)&&c(6)==c(7) ...
    &&all(c(8:10)==0)&&c(6)==h(3),'gpenmpcNative:LocalInputSnapshotStatus','Step/getter loss or sticky fault.');
events=zeros(n,1,'uint64');for j=1:n,events(j)=littleValue(w.records(1:8,j),'uint64');end
assert(events(1)>0&&all(diff(events)==1)&&events(end)<=h(3),'gpenmpcNative:LocalInputRing','Window record gap/reverse.');
o=struct('producer_pid',pid,'peer_nonce',nonce,'first_window_event',events(1),'last_window_event',events(end), ...
    'observed_total_getters',h(3),'external_owner_quiescence_proven_here',false);
end
function key=sourceKey(s)
key=struct('identity',s.identity,'sample_us',s.original_sample_us,'publication_us',s.original_publication_us, ...
    'original_receipt_us',s.original_receipt_us,'source_generation',s.source_generation,'generation_delta',s.generation_delta, ...
    'sample_delta_us',s.sample_delta_us,'reset_counter',s.reset_counter,'state_and_origin_sha256',s.state_and_origin_sha256);
end
function v=littleValue(b,t),v=typecast(b(:),t);[~,~,e]=computer;if e=='B',v=swapbytes(v);end;v=v(:);end
function b=be(v),[~,~,e]=computer;if e=='L',v=swapbytes(v);end;b=reshape(typecast(v,'uint8'),[],1);end
function h=digest(b),m=java.security.MessageDigest.getInstance('SHA-256');m.update(typecast(b(:),'int8'));h=reshape(typecast(m.digest(),'uint8'),[],1);end
function h=hashBytes(x),if isa(x,'uint8'),h=x(:);else,h=uint8(sscanf(char(x),'%2x'));end;assert(numel(h)==32);end
function y=sameBits(a,b),y=isequal(typecast(a(:),'uint8'),typecast(b(:),'uint8'));end
