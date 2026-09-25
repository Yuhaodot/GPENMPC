function report=test_rfly_same_io_exchange(outputRoot)
% Test same-owner callbacks, asynchronous worker, peer bytes and C/RFC1 commit on loopback.
% Use synthetic registration, board, rotor and wind data.
arguments,outputRoot (1,1) string,end
build=string(fileparts(fileparts(mfilename('fullpath'))));
parent=string(gpenmpc_external_path('native_visual_host_runtime'));
addpath(fullfile(parent,'src'),'-begin');addpath(fullfile(build,'host_runtime'),'-begin');
addpath(fullfile(build,'matlab_validation'),fullfile(build,'m600_coptersim','matlab_validation'));
assert(~isfolder(outputRoot));mkdir(outputRoot);checks=struct('name',{},'pass',{});raw=struct();
observedPool=backgroundPool;poolCapacity=NaN;
if isprop(observedPool,'NumWorkers'),poolCapacity=observedPool.NumWorkers;end
raw.thread_observation=struct('maxNumCompThreads',maxNumCompThreads, ...
    'OMP_NUM_THREADS',getenv('OMP_NUM_THREADS'),'MKL_NUM_THREADS',getenv('MKL_NUM_THREADS'), ...
    'OPENBLAS_NUM_THREADS',getenv('OPENBLAS_NUM_THREADS'),'background_pool_capacity',poolCapacity, ...
    'background_pool_class',class(observedPool),'background_pool_properties',{properties(observedPool)}, ...
    'solver_futures_owned_by_exchange',1,'background_pool_capacity_is_not_active_solver_count',true);
paths={which('gpenmpcNative.advanceRflyCanonicalIoExchange'),which('gpenmpcNative.RflyHostExchangeService'), ...
    which('m600check.makeM600CopterSimIo'),which('gpenmpcNative.RflyTunnelReassembler'), ...
    which('gpenmpcNative.RflyCommittedFeedbackDecoder'),which('gpenmpcNative.warmRflyCanonicalHostFunctions'), ...
    which('gpenmpcNative.BoardOuterService'),[mfilename('fullpath') '.m']};
hashes=cellfun(@fileSha,paths,'UniformOutput',false);
raw.source_fingerprint=struct('sources',{paths},'source_sha256',{hashes});
probe=fullfile(gpenmpc_external_path('linux_io_exchange_fixture'),'actual_io_exchange_probe');
raw.peer_fixture_transport='SAME_UDP_PEER_ORIGINAL_FRAMES_TO_ACTUAL_C_STDIN_NO_CRITICAL_DISK_IPC';
kernel=fullfile(gpenmpc_external_path('full_inner_px4_float_mapping'),'MATLAB_ARGUMENTS_AND_EXPECTED.bin');
states=fullfile(gpenmpc_external_path('full_inner_px4_float_mapping'),'MATLAB_NED_AND_MAPPED_STATE.bin');
values={'wsl.exe','-d','Ubuntu-24.04','--',linux(probe),linux(kernel),linux(states),linux(outputRoot)};
args=javaArray('java.lang.String',numel(values));for k=1:numel(values),args(k)=java.lang.String(values{k});end
builder=java.lang.ProcessBuilder(args);builder.redirectErrorStream(true);process=builder.start();
pc=onCleanup(@()process.destroy()); %#ok<NASGU>
reader=java.io.BufferedReader(java.io.InputStreamReader(process.getInputStream()));writer=java.io.PrintWriter(process.getOutputStream(),true);
raw.cpp_ready=string(reader.readLine());assert(raw.cpp_ready=="READY_ACTUAL_FIXTURE_SESSION_AND_SOURCES");
eb=readbin(fullfile(outputRoot,'ACTUAL_FIXTURE_ECHO119.bin'));now=gpenmpcNative.rflyOriginalHostMonotonicNs();
echo=struct('identity_semantics','BoardRegisteredExecutionSessionGenerationV1', ...
    'host_challenge',decode(eb(6:21),'uint64'),'uid',decode(eb(22:29),'uint64'),'system',eb(30),'component',eb(31), ...
    'board_registration_hrt_us',decode(eb(32:39),'uint64'),'process_session_generation',decode(eb(40:47),'uint64'), ...
    'link_lifecycle_generation',decode(eb(48:55),'uint64'),'configuration_payload_sha256',hex(eb(56:87)));
sessionSha=hex(eb(88:119));
association=struct('schema','GPENMPC_RFLY_REGISTERED_SESSION_ASSOCIATION_V1','registration_result','Registered', ...
    'echo_confirmation_result','Confirmed','echo',echo,'original_host_challenge',echo.host_challenge, ...
    'original_host_receive_ns',now,'execution_session_sha256',sessionSha,'provenance','ACTUAL_EXISTING_SESSION_BOUND_GUARD_HOST_FIXTURE');
e=struct('uid',string(echo.uid),'system_id',1,'component_id',1,'boot_generation',echo.process_session_generation, ...
    'maximum_age_ns',1e8,'maximum_runtime_age_ns',1e8,'canonical_package_root',gpenmpcNative.canonicalAssetRoot(), ...
    'configuration_payload_sha256',echo.configuration_payload_sha256,'initial_payload_kg',2.27, ...
    'async_outer_required',true,'task_identity_sha256',repmat('C',1,64), ...
    'inner_control_contract','CANONICAL_FULL_SE3','full_inner_evidence_scope','BOARD_COMMIT_RFC1','initial_leg_index',1);
e.rfly_board_commit=struct('execution_session_sha256',sessionSha,'identity_semantics',echo.identity_semantics, ...
    'link_lifecycle_generation',echo.link_lifecycle_generation,'board_registration_hrt_us',echo.board_registration_hrt_us, ...
    'reference_max_age_us',uint64(400000),'outer_max_age_us',uint64(400000), ...
    'generated_arm_source_sha256','A47F1C255BDAC1DAE712494BE8D9D66FC4F83138DBA9B0C2E3A31E114D4ABD0F', ...
    'wrapper_matlab_source_sha256','9117D3CDF8E924A253F4C444CDD467E4850D6F11CBF5744B26375B950E2A95B5');
origin=struct('link_lifecycle_generation',echo.link_lifecycle_generation,'execution_session_sha256',sessionSha, ...
    'source_system',uint8(1),'source_component',uint8(1));
source=cell(1,3);for k=1:3,source{k}=readbin(fullfile(outputRoot,"SOURCE_"+k+".bin"));end
p=gpenmpcNative.RflySnapshotSample(source{1},now,e);[state,ok]=gpenmpcNative.px4EstimateState(p,e,now);assert(ok);
coef=zeros(3,8);coef(:,1)=state(1:3);coef(:,2)=[.1;0;0];trajectory=struct('total_duration_s',10,'coefficients_ascending',coef);
cfg=struct('local_mavlink_port',62311,'remote_mavlink_port',62312,'truth_port',62313,'coptersim_time_port',62314, ...
    'target_system',1,'target_component',1,'clock_max_rtt_s',1,'clock_sync_samples',2,'clock_sync_period_s',1, ...
    'clock_max_age_s',5,'clock_max_uncertainty_s',1,'clock_max_utc_drift_s',1,'clock_max_time_heartbeat_age_s',5, ...
    'clock_max_time_heartbeat_lag_s',5,'maximum_truth_lag_s',5,'maximum_raw_records',10000,'state_max_age_s',1, ...
    'live_enabled',true,'outer_preflight_pass',true, ...
    'canonical_exchange',struct('assembly_limit_ns',uint64(2000000000),'completed_queue_capacity',2), ...
    'canonical_rotor_observer',struct('local_port',62315,'maximum_queue',2));
d=mavlinkdialect(fullfile(build,'m600_coptersim','matlab_validation','+m600check','px4_health_events.xml'),2);
board=mavlinkio(d,'SystemID',1,'ComponentID',1);serializer=mavlinkio(d,'SystemID',255,'ComponentID',190);
sc=onCleanup(@()deleteSerializers(board,serializer)); %#ok<NASGU>
expected=struct('source_system',uint8(1),'source_component',uint8(1),'target_system',uint8(255),'target_component',uint8(190), ...
    'uid',echo.uid,'session_generation',echo.process_session_generation,'link_lifecycle_generation',echo.link_lifecycle_generation, ...
    'confirmed_host_rx_ns',gpenmpcNative.rflyOriginalHostMonotonicNs(),'execution_session_sha256',sessionSha, ...
    'configuration_sha256',echo.configuration_payload_sha256);
x=gpenmpcNative.RflyHostExchangeService(build,e,trajectory,association,serializer,d,8);xc=onCleanup(@()x.close()); %#ok<NASGU>
% All actual constructor asset/path initialization has finished, but no IO
% endpoint exists. Only NEW discarded historical objects are warmed; x is
% not passed in, prepared, sampled or otherwise changed by this helper.
raw.discarded_function_warmup=gpenmpcNative.warmRflyCanonicalHostFunctions(build,20);
io=m600check.makeM600CopterSimIo(cfg);ioc=onCleanup(@()io.close()); %#ok<NASGU>
peer=udpport('datagram','IPV4','LocalHost','127.0.0.1','LocalPort',cfg.remote_mavlink_port,'Timeout',.2);
peerc=onCleanup(@()delete(peer)); %#ok<NASGU>
raw.bound=io.bindCanonicalSession(expected);
inputGeneration=63;rotorValid=true;rotorIngests=0;lastHealthSeconds=-Inf;lastHealthArmed=NaN;
rotor=struct('Failed',false,'ingest',@ingestRotor,'current',@rotorCurrent);
wind=struct('observe',@windObserve);
request=struct('origin',origin,'task_time_s',0,'request_startup',false,'maximum_sources_per_poll',1, ...
    'heartbeat_max_age_s',1,'landed_max_age_s',1);
try
health(false);emit(source{1},3,uint64(63));waitCanonical(1,0);
raw.prepare=step(x);
check('actual_callback_gen0_submitted_once_no_inner',~raw.prepare.failed&&raw.prepare.source_messages==1 ...
    &&raw.prepare.inner_messages_sent==0&&x.status().coordinator.state=="PREPARING" ...
    &&x.status().coordinator.actual_solver_submissions==0);
pause(.30);raw.preparation_polls={};timer=tic;
while ~x.status().coordinator.initialized&&toc(timer)<60
    health(false);value=step(x);assert(~value.failed,'gpenmpcNative:SameIoPrepare',jsonencode(value.failure));
    raw.preparation_polls{end+1}=value;pause(.002); %#ok<AGROW>
end
check('same_worker_gen0_diagnostic_not_control',x.status().coordinator.initialized ...
    &&x.status().coordinator.service.physical_runtime.control_commit_count==0&&~x.status().pending_command);
inputGeneration=64;request.request_startup=true;health(false);emit(source{2},3,uint64(64));waitCanonical(1,0);
raw.startup=step(x);assert(~raw.startup.failed,'gpenmpcNative:SameIoStartup',jsonencode(raw.startup.failure));
raw.startup_polls={};timer=tic;
while ~x.status().startup.ready&&toc(timer)<3
    health(false);value=step(x);raw.startup_polls{end+1}=value; %#ok<AGROW>
    assert(~value.failed,'gpenmpcNative:SameIoStartupPoll',jsonencode(value.failure));pause(.001);
    if ~x.status().coordinator.service.async_in_flight&&~x.status().startup.ready,break;end
end
check('actual_async_new_source_startup_no_inner',x.status().startup.ready&&raw.startup.inner_messages_sent==0 ...
    &&x.status().coordinator.actual_solver_submissions==1&&x.status().coordinator.service.phase_s==0);
assert(x.status().startup.ready,'gpenmpcNative:SameIoStartupNotReady','Startup outer solve timed out.');
inputGeneration=65;health(true);emit(source{3},3,uint64(65));waitCanonical(1,0);
raw.flight=step(x);assert(~raw.flight.failed,'gpenmpcNative:SameIoFlight',jsonencode(raw.flight.failure));
raw.flight_return_ns=gpenmpcNative.rflyOriginalHostMonotonicNs();
check('seven_same_owner_actual_send_not_commit',raw.flight.inner_messages_sent==7 ...
    &&x.status().pending_command&&x.status().coordinator.service.physical_runtime.control_commit_count==0);
assert(raw.flight.inner_messages_sent==7,'gpenmpcNative:SameIoNoSend',jsonencode(raw.flight));
packets=raw.flight.events{1}.result.packets;wire=cell(7,1);
for k=1:7,wire{k}=receivePeer();end
raw.peer_read_complete_ns=gpenmpcNative.rflyOriginalHostMonotonicNs();
% Original datagrams, not decoded/re-serialized message substitutes. This
% test's C++ process uses the real generated parser on exactly these bytes.
writer.println('execute-inline');for k=1:7,writer.println(hex(wire{k}));end
line=string(reader.readLine());raw.cpp=jsondecode(line);
raw.cpp_complete_ns=gpenmpcNative.rflyOriginalHostMonotonicNs();
feedback=uint8(sscanf(raw.cpp.feedback_hex,'%2x'));feedback=feedback(:);
assert(numel(feedback)==1112,'gpenmpcNative:SameIoActualFeedbackLength');
% Exact RFC1 output-generation field (bytes65:72). The actual IO reassembler
% and consumer verify full SHA/schema/identity. Avoid a redundant full audit
% decode in this test peer's critical response path; perform it after commit.
emit(feedback,5,decode(feedback(65:72),'uint64'));raw.feedback_emit_ns=gpenmpcNative.rflyOriginalHostMonotonicNs();
waitCanonical(0,1);raw.feedback_complete_ns=gpenmpcNative.rflyOriginalHostMonotonicNs();raw.commit=step(x);
raw.commit_return_ns=gpenmpcNative.rflyOriginalHostMonotonicNs();
persistPeerEvidence(); % Observation archives only AFTER attempted consumption.
assert(~raw.commit.failed,'gpenmpcNative:SameIoCommit',jsonencode(raw.commit.failure));
disp(line);f=gpenmpcNative.RflyCommittedFeedbackDecoder(feedback,gpenmpcNative.rflyOriginalHostMonotonicNs());
payloadEqual=true;for k=1:7
    actual=deserializemsg(d,wire{k});input=deserializemsg(d,packets{k});payloadEqual=payloadEqual&&isequal(actual.Payload,input.Payload);
end
check('actual_peer_wire_payload_not_reserialized_evidence',payloadEqual);
raw.final=x.status();
check('actual_peer_parser_dispatch_C_RFC1_same_callback_commit',raw.cpp.failed==0&&raw.cpp.actual_peer_frames==7 ...
    &&raw.cpp.actual_generated_c_steps==1&&raw.commit.feedback_messages==1 ...
    &&raw.final.coordinator.service.physical_runtime.control_commit_count==1&&~raw.final.pending_command);
check('single_solver_original_generations_and_expiry',raw.final.coordinator.actual_solver_submissions==1 ...
    &&f.token.sample_generation==65&&f.token.outer_valid_until_us==uint64(1410000) ...
    &&f.token.reference_valid_until_us==uint64(1420000));
raw.io=io.evidence();rows=raw.io.raw_mavlink(cellfun(@(v)strcmp(v.topic,'TUNNEL'),raw.io.raw_mavlink));
check('original_first_callback_uint64_not_poll_time',raw.prepare.events{1}.ingress.original_host_receive_ns==rows{1}.original_host_receive_ns ...
    &&raw.flight.events{1}.ingress.original_host_receive_ns==rows{7}.original_host_receive_ns ...
    &&isa(rows{1}.original_host_receive_ns,'uint64')&&~rows{1}.raw_frame_available);
check('actual61_and16_from_C_not_host_oracle', ...
    isequal(f.actual61,raw.commit.events{1}.result.board_commit_observation.actual61) ...
    &&isequal(typecast(f.published_control16,'uint32'), ...
    typecast(raw.commit.events{1}.result.board_commit_observation.published_control16,'uint32')));
check('no_arm_second_owner_or_live_claim',~raw.flight.arm_authorized&&raw.flight.new_endpoints==0 ...
    &&raw.final.owned_io_endpoints==0&&~raw.final.live_registration_proven);
raw.before_host_land=x.status();
raw.host_land=x.suspendForNativeLand(gpenmpcNative.rflyOriginalHostMonotonicNs());
raw.after_host_land=x.status();
check('host_land_suspend_preserves_phase_and_original_source_clock_without_board_actions', ...
    raw.after_host_land.coordinator.service.phase_s==raw.before_host_land.coordinator.service.phase_s ...
    &&raw.after_host_land.coordinator.source_clock.last_time_s==raw.before_host_land.coordinator.source_clock.last_time_s ...
    &&raw.after_host_land.coordinator.service.outer_suspended&&isempty(raw.host_land.packets) ...
    &&~raw.host_land.board_stop_sent&&~raw.host_land.land_command_sent ...
    &&~raw.host_land.board_disarmed_proven&&~raw.host_land.plant_cache_zero_proven&&~raw.host_land.session_release_proven);
x.close();
% Inject failure after three packet sends to test attempt accounting.
mockClosed=false;mockDrains=0;mockSubmitted=0;mockPhase="PREPARING";
mock=struct('status',@mockStatus,'observeDisarmedSnapshot',@mockDrain,'pollPreparation',@mockPoll, ...
    'poll',@mockPoll,'source',@mockSource,'submitted',@mockSubmit,'close',@mockClose);
testIo=io;testIo.pollCanonical=@()struct('bound',true,'failure',[],'completed_queue_capacity',2);
testIo.takeCanonical=@mockTake;testIo.sendCanonicalPackets=@partialSend;
% Supply the retained callback source through the test seam and observe board health by callback.
hasMockSource=true;health(false);rotorValid=false;request.request_startup=false;
raw.invalid_rotor_drain=gpenmpcNative.advanceRflyCanonicalIoExchange(testIo,mock,rotor,wind,request);
check('invalid_rotor_pure_disarmed_drain_not_blocked',~raw.invalid_rotor_drain.failed&&mockDrains==1);
mockPhase="UNPREPARED";hasMockSource=true;
raw.invalid_rotor_prepare=gpenmpcNative.advanceRflyCanonicalIoExchange(testIo,mock,rotor,wind,request);
check('invalid_rotor_prepare_waits_without_consuming_source',~raw.invalid_rotor_prepare.failed&&hasMockSource ...
    &&strcmp(raw.invalid_rotor_prepare.events{1}.kind,'WAIT_ORIGINAL_ROTOR_OBSERVATION'));
mockPhase="PREPARED_PAUSED";rotorValid=true;hasMockSource=true;health(true);
raw.partial=gpenmpcNative.advanceRflyCanonicalIoExchange(testIo,mock,rotor,wind,request);
check('actual_three_sent_then_injected_failure_keeps_original_ledger',raw.partial.failed&&mockClosed ...
    &&raw.partial.inner_messages_sent==3&&raw.partial.partial_send_evidence.messages_attempted==3 ...
    &&raw.partial.partial_send_evidence.messages_send_returned==3&&mockSubmitted==0 ...
    &&~raw.partial.partial_send_evidence.board_commit_proven&&raw.partial.same_io_cleanup_required);
partialWire=cell(3,1);for k=1:3,partialWire{k}=receivePeer();end
check('partial_evidence_original_uint64_rows_retained',all(cellfun(@(v)isa(v.original_host_submit_ns,'uint64') ...
    &&v.send_returned&&v.original_host_send_return_ns>=v.original_host_submit_ns, ...
    raw.partial.partial_send_evidence.original_transmit_rows)));
raw.after=io.evidence();check('failure_does_not_close_same_io_safe_finally_owner',~raw.after.closed);
check('same_owner_closed',io.close());
report=struct('passed',all([checks.pass]),'checks',checks,'actual_peer_frames',raw.cpp.actual_peer_frames, ...
    'actual_command_wire_bytes',sum(cellfun(@numel,wire)),'actual_generated_c_steps',raw.cpp.actual_generated_c_steps, ...
    'actual_host_service_commits',raw.final.coordinator.service.physical_runtime.control_commit_count, ...
    'original_source_generations',[63 64 65],'partial_send_injected_after_real_messages',3, ...
    'rotor_wind_scope','NUMERICAL_ROTOR_WIND_INPUT_FIXTURE', ...
    'board_hrt_uorb_ack_scope','EXPLICIT_HOST_FIXTURE_NO_CLOCK_MAPPING', ...
    'com_opens',0,'simulator_actions',0,'board_actions',0,'arm_commands',0,'publication_authority',false, ...
    'sources',{paths},'source_sha256',{hashes},'sources_unchanged',isequal(hashes,cellfun(@fileSha,paths,'UniformOutput',false)));
report.passed=report.passed&&report.sources_unchanged;
save(fullfile(outputRoot,'RAW.mat'),'report','raw','wire','packets','partialWire','source','association','request');
writejson(fullfile(outputRoot,'RESULT.json'),report);disp(jsonencode(struct('passed',report.passed,'checks',numel(checks))));
assert(report.passed,'gpenmpcNative:SameIoRegression');
catch problem
    persistPeerEvidence();
    raw.source_fingerprint.sources_unchanged=isequal(hashes,cellfun(@fileSha,paths,'UniformOutput',false));
    raw.last_source_profile=x.LastSourceProfile;
    attemptIo=io.evidence();attemptStatus=x.status();failure=struct('identifier',problem.identifier,'message',problem.message); %#ok<NASGU>
    save(fullfile(outputRoot,'ATTEMPT_RAW.mat'),'raw','checks','attemptIo','attemptStatus','failure');
    writejson(fullfile(outputRoot,'ATTEMPT_RESULT.json'),struct('passed',false,'checks',checks,'failure',failure));
    rethrow(problem)
end
    function persistPeerEvidence()
        if exist('wire','var')&&iscell(wire)
            for j=1:numel(wire),if ~isempty(wire{j}),writebin(fullfile(outputRoot,"PEER_FRAME_"+j+".bin"),wire{j});end,end
        end
        if exist('feedback','var')&&~isempty(feedback),writebin(fullfile(outputRoot,'FEEDBACK1112.bin'),feedback);end
    end
    function r=step(exchange),r=gpenmpcNative.advanceRflyCanonicalIoExchange(io,exchange,rotor,wind,request);end
    function check(name,value),checks(end+1)=struct('name',name,'pass',logical(value));end
    function ingestRotor(v),assert(isempty(v.records));rotorIngests=rotorIngests+1;end
    function [v,r]=rotorCurrent(ns,~)
        v=struct('source','HOST_M600_VIRTUAL_ACTUATOR_INTERFACE','valid',rotorValid,'generation',inputGeneration, ...
            'rx_ns',double(ns),'ordering','SOFTWARE_M600_ORDER','rotor_thrust_state_n',ones(6,1)*20, ...
            'thrust_effectiveness',ones(6,1),'state_source','SAME_M600_ACCEPTED_STEP_ROTOR_LAG_STATE','plant_session_id',1);
        r=struct('scope','EXPLICIT_HOST_INPUT_FIXTURE','ingest_calls',rotorIngests);
    end
    function [v,r]=windObserve(~,~,~,ns)
        v=struct('source','FROZEN_TASK_WIND_ESTIMATOR','valid',true,'generation',inputGeneration,'rx_ns',double(ns),'estimate_xy_mps',zeros(2,1));
        r=struct('scope','EXPLICIT_HOST_INPUT_FIXTURE');
    end
    function health(armed)
        % Explicit test peer health events at the same selected 20-Hz cadence,
        % not two callback packets on every solver poll. No source/expiry edit.
        if armed==lastHealthArmed&&io.now()-lastHealthSeconds<.05,return;end
        previous=io.snapshot();
        h=createmsg(d,'HEARTBEAT');h.Payload.type=uint8(2);h.Payload.autopilot=uint8(12);h.Payload.base_mode=uint8(128*armed);
        q=createmsg(d,'EXTENDED_SYS_STATE');q.Payload.landed_state=uint8(1+armed);
        lastHealthSeconds=io.now();lastHealthArmed=armed;
        write(peer,[uint8(serializemsg(board,h)) uint8(serializemsg(board,q))],'uint8','127.0.0.1',cfg.local_mavlink_port);
        timerHealth=tic;while toc(timerHealth)<1
            v=io.snapshot();if v.armed==armed&&v.landed_state==1+armed ...
                    &&v.heartbeat_rx_s>previous.heartbeat_rx_s&&v.extended_rx_s>previous.extended_rx_s,return;end;pause(.001);
        end;error('gpenmpcNative:SameIoHealthTimeout');
    end
    function emit(body,schema,generation)
        frames=cell(ceil(numel(body)/119),1);
        for j=0:numel(frames)-1
            msg=createmsg(d,'TUNNEL');part=[uint8(schema*16+j);be(generation);body(j*119+1:min(j*119+119,numel(body)))];
            msg.Payload.payload_type=uint16(42002);msg.Payload.target_system=uint8(255);msg.Payload.target_component=uint8(190);
            msg.Payload.payload_length=uint8(numel(part));msg.Payload.payload(:)=0;msg.Payload.payload(1:numel(part))=part;
            frames{j+1}=reshape(uint8(serializemsg(board,msg)),[],1);
        end
        write(peer,vertcat(frames{:}),'uint8','127.0.0.1',cfg.local_mavlink_port);
    end
    function waitCanonical(s,f)
        t=tic;while toc(t)<2
            v=io.pollCanonical();if all(v.completed_queue_counts>=[s f]),return;end;pause(.001);
        end;error('gpenmpcNative:SameIoCanonicalTimeout');
    end
    function bytes=receivePeer()
        t=tic;
        while toc(t)<2
            while peer.NumDatagramsAvailable==0&&toc(t)<2,pause(.001);end
            assert(peer.NumDatagramsAvailable>0);row=read(peer,1,'uint8');bytes=uint8(row.Data(:));
            % Use the header only as a filter; the generated parser validates CRC.
            if numel(bytes)>=12&&bytes(1)==253&&isequal(bytes(8:10),uint8([129;1;0])),return;end
            % Retain TIMESYNC packets separately from the seven TUNNEL command fragments.
            if ~isfield(raw,'other_original_peer_frames'),raw.other_original_peer_frames={};end
            raw.other_original_peer_frames{end+1}=struct('bytes',bytes,'crc_checked_by_test_filter',false);
        end
        error('gpenmpcNative:SameIoTunnelPeerTimeout');
    end
    function s=mockStatus()
        s=struct('pending_command',false,'coordinator',struct('state',mockPhase,'initialized',mockPhase=="PREPARED_PAUSED"), ...
            'startup',struct('requested',true,'ready',false));
    end
    function v=mockDrain(varargin),mockDrains=mockDrains+1;v=struct('packets',{{}});end
    function v=mockPoll(varargin),v=struct('scope','EXPLICIT_STEP_BRANCH_FIXTURE');end
    function v=mockSource(varargin)
        v=raw.flight.events{1}.result;
        % Inject fixture context to test send accounting without invoking the exchange or C core.
        ns=gpenmpcNative.rflyOriginalHostMonotonicNs();
        v.context.reference_creation_ns=ns;v.context.outer_creation_ns=ns;
        v.context.reference_expiry_ns=ns+uint64(400000000);v.context.outer_expiry_ns=ns+uint64(400000000);
        v.send_validity=struct('source_host_receive_ns',ns,'maximum_runtime_age_ns',uint64(100000000), ...
            'source_valid_until_ns',ns+uint64(100000000));
    end
    function v=mockSubmit(varargin),mockSubmitted=mockSubmitted+1;v=[];end
    function mockClose(),mockClosed=true;end
    function item=mockTake(kind)
        item=[];if strcmp(kind,'snapshot')&&hasMockSource,item=raw.flight.events{1}.ingress;hasMockSource=false;end
    end
    function v=partialSend(input,context,validity)
        io.sendCanonicalPackets(input(1:3),context,validity);error('gpenmpcNative:InjectedAfterThreeActualSends');v=[]; %#ok<UNRCH>
    end
end
function b=readbin(p),f=fopen(p,'rb');assert(f>0);c=onCleanup(@()fclose(f));b=fread(f,inf,'*uint8');end
function writebin(p,b),f=fopen(p,'wb');assert(f>0);c=onCleanup(@()fclose(f));fwrite(f,b,'uint8');end
function writejson(p,v),f=fopen(p,'w');assert(f>0);c=onCleanup(@()fclose(f));fprintf(f,'%s\n',jsonencode(v,PrettyPrint=true));end
function h=hex(b),h=upper(reshape(dec2hex(b,2).',1,[]));end
function b=be(v),[~,~,e]=computer;if e=='L',v=swapbytes(v);end;b=reshape(typecast(v,'uint8'),[],1);end
function v=decode(b,t),v=typecast(uint8(b),t);[~,~,e]=computer;if e=='L',v=swapbytes(v);end;v=v(:);end
function p=linux(p),p=char(replace(p,'\','/'));p=['/mnt/' lower(p(1)) p(3:end)];end
function deleteSerializers(a,b),delete(a);delete(b);end
function value=fileSha(p),md=java.security.MessageDigest.getInstance('SHA-256');md.update(typecast(readbin(p),'int8'));value=hex(typecast(md.digest(),'uint8'));end
