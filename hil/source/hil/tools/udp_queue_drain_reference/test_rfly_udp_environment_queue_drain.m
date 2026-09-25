function report=test_rfly_udp_environment_queue_drain()
% Replay recorded NoUI diagnostic bytes over isolated loopback UDP.
build=fileparts(fileparts(mfilename('fullpath')));
target=fullfile(build,'tools','UDP_ENVIRONMENT_QUEUE_DRAIN_HOST_ONLY.mat');
assert(~isfile(target),'gpenmpcHost:ExistingEvidence','Do not overwrite evidence.');
addpath(fullfile(build,'tools'),fullfile(build,'host_runtime'),fullfile(build,'matlab_validation'),fullfile(build,'m600_coptersim','matlab_validation'));
addpath(gpenmpc_external_path('native_visual_host_source'),'-end');
q=load(gpenmpc_external_path('udp_loopback_fixture'),'cfg');cfg=q.cfg;
assert(strcmp(cfg.mavlink_transport.scope,'HOST_ONLY_LOOPBACK')&&cfg.truth_port==62293&& ...
    cfg.local_mavlink_port==62291&&cfg.remote_mavlink_port==62292&&cfg.delivery_environment_contract.remote_port==62296);
addpath(fileparts(cfg.mavlink_transport.source.exact_path));
source=fullfile(gpenmpc_external_path('startup_observation'),'STARTUP_COMPATIBILITY_CHECK','STARTUP_OBSERVATION_RAW.mat');
retained=load(source,'rawIo');rows=retained.rawIo.raw_truth_datagrams;
index=find(cellfun(@(x)numel(x.bytes)==264,rows),1,'first');
assert(~isempty(index),'gpenmpcHost:MissingActualDiagnostic','Retained actual startup must contain a 264-byte diagnostic.');
wire=uint8(rows{index}.bytes(:));
ioPath=which('m600check.makeM600CopterSimIo');classPath=which('gpenmpcNative.RflySoleMavlinkTransport');
hashes={m600check.fileSha256(ioPath),m600check.fileSha256(classPath)};
io=[];peer=[];raw=struct();drained=cell(257,1);snapshots=cell(257,1);checks=struct();guard=onCleanup(@finish);
try
    io=m600check.makeM600CopterSimIo(cfg);
    peer=udpport('datagram','IPV4','LocalHost','127.0.0.1','LocalPort',62297);
    for k=1:257
        write(peer,wire,'uint8','127.0.0.1',62293);
        timer=tic;events={};
        while isempty(events)&&toc(timer)<1
            pause(.002);snapshots{k}=io.snapshot();events=io.takeCanonicalEnvironmentRecords();
        end
        assert(numel(events)==1&&strcmp(events{1}.kind,'DIAGNOSTIC_RX')&& ...
            isequal(uint8(events{1}.bytes(:)),wire),'gpenmpcHost:DiagnosticDrain','Exactly one byte-identical diagnostic must be consumed per input.');
        drained{k}=events{1};
        assert(isempty(snapshots{k}.fatal),'gpenmpcHost:UnexpectedFatal','Observed fatal: %s',snapshots{k}.fatal);
    end
    evidence=io.evidence();native=gpenmpc_rfly_udp_transport_mex('status');
    checks.all_257_actual_datagrams_retained_and_drained=numel(evidence.raw_truth_datagrams)==257&& ...
        all(cellfun(@(r)isequal(uint8(r.bytes(:)),wire),evidence.raw_truth_datagrams))&& ...
        numel(drained)==257&&all(cellfun(@(r)r.original_host_receive_ns>0,drained));
    % take() checks the private overflow latch before returning; all 257
    % successful calls and this final empty call prove the latch stayed false.
    checks.queue_overflow_false_and_final_queue_empty=isempty(io.takeCanonicalEnvironmentRecords())&& ...
        isempty(evidence.fatal)&&all(structfun(@(v)v==0,evidence.raw_record_overflow_drops));
    checks.no_environment_tx_or_source_binding=isempty(evidence.raw_environment_transmit_datagrams)&& ...
        isempty(evidence.raw_transmit_messages)&&~evidence.canonical_exchange_bind_attempted&& ...
        isempty(evidence.canonical_exchange_association)&&~evidence.delivery_diagnostic_observer.session_bound&& ...
        native.sent_count==0&&~native.failed;
    assert(all(structfun(@logical,checks)),'gpenmpcHost:Checks','Queue/raw/no-authority check failed.');
    assert(isequal(hashes,{m600check.fileSha256(ioPath),m600check.fileSha256(classPath)}), ...
        'gpenmpcHost:SourceChanged','Production source changed during fixture.');
    raw=struct('drained',{drained},'snapshots',{snapshots},'io',evidence,'native',native,'input_bytes',wire);
    report=struct('scope','HOST_ONLY_ACTUAL_DIAGNOSTIC257_PER_TICK_DRAIN','pass',true,'checks',checks, ...
        'retained_source',source,'retained_source_sha256',m600check.fileSha256(source),'retained_raw_index',index, ...
        'input_count',257,'queue_capacity_unchanged',256,'raw_count',257,'queue_overflow',false, ...
        'queue_overflow_evidence','ALL_257_GUARDED_TAKE_CALLS_RETURNED_AND_FINAL_QUEUE_EMPTY', ...
        'source_unchanged',true,'io_source_sha256',hashes{1},'transport_class_sha256',hashes{2}, ...
        'COM',0,'CopterSim',0,'board',0,'HIL',0,'environment_tx',0,'control_commands',0);
    clear guard
    save(target,'raw','report','cfg');disp(jsonencode(report));
catch ex
    failure=struct('identifier',ex.identifier,'message',ex.message,'stack',ex.stack);
    if ~isempty(io),try,raw.failure_io=io.evidence();catch,end,end
    clear guard
    save(target,'raw','drained','snapshots','checks','cfg','failure');rethrow(ex)
end
    function finish()
        if ~isempty(io),try,io.close();catch,end;io=[];end
        if ~isempty(peer),delete(peer);peer=[];end
    end
end
