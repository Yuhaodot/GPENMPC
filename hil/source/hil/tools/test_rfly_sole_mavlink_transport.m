function report=test_rfly_sole_mavlink_transport(outputRoot,source,ports,selection)
% Test the bound native transport on localhost.
% Source fields are kind, exact_path and sha256; add the MEX folder before running.
if nargin<3,ports=[62371 62372 62373];end
if nargin<4,selection='ALL';end
assert(isnumeric(ports)&&numel(ports)==3&&numel(unique(ports))==3 ...
    &&all(ports>=62200&ports<=62399&ports==fix(ports)));
build=fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(build,'host_runtime'));
assert(~isfile(fullfile(outputRoot,'RESULT.json')),'Existing evidence is read-only.');
if ~isfolder(outputRoot),mkdir(outputRoot);end
checks=struct('name',{},'pass',{});raw={};transport=[];peer=[];wrongPeer=[];
serializer=[];remoteSerializer=[];v1Serializer=[];wrongSerializer=[];
cleanup=onCleanup(@finish); %#ok<NASGU>
dialectPath=fullfile(build,'m600_coptersim','matlab_validation', ...
    '+m600check','px4_health_events.xml');
d=mavlinkdialect(dialectPath,2);
cfg=struct('source',source,'scope','HOST_ONLY_LOOPBACK','allow_loopback',true, ...
    'local_host','127.0.0.1','remote_host','127.0.0.1', ...
    'local_port',ports(1),'remote_port',ports(2), ...
    'local_system',uint8(245),'local_component',uint8(190), ...
    'remote_system',uint8(1),'remote_component',uint8(1),'maximum_poll_datagrams',8);
if strcmp(selection,'INLINE_RECEIVE')
    clear cleanup
    report=inlineReceiveCheck(cfg,d,outputRoot);return
end
if strcmp(selection,'RC_CONTINUOUS_RECEIVE')
    clear cleanup
    report=rcContinuousReceiveCheck(cfg,outputRoot,build);return
end
if strcmp(selection,'RC_REMAINDER')
    cfg.copter_serial_forwarding=true;
    serializer=mavlinkio(d,'SystemID',cfg.local_system,'ComponentID',cfg.local_component);
    remoteSerializer=mavlinkio(d,'SystemID',cfg.remote_system,'ComponentID',cfg.remote_component);
    peer=udpport('datagram','IPV4','LocalHost','127.0.0.1','LocalPort',ports(2));
    transport=gpenmpcNative.RflySoleMavlinkTransport(cfg,d,serializer);
    % Use the invalid header from a 96-byte serial-payload remainder.
    remainder=zeros(96,1,'uint8');remainder(1:15)=uint8([253 48 60 0 0 192 127 0 0 192 127 0 0 192 127]);
    write(peer,remainder,'uint8','127.0.0.1',ports(1));
    frames=struct([]);fragments=struct([]);for k=1:20
        [a,b]=transport.poll();frames=[frames;a];fragments=[fragments;b];
        if ~isempty(fragments),break;end;pause(.002);
    end
    check('payload_magic_not_a_received_control_message',isempty(frames)&&numel(fragments)==1);
    check('all_remainder_bytes_retained',isequal(fragments(1).raw_fragment(:),remainder));
    state=transport.status();check('link_not_faulted_by_unframed_payload',~state.failed);
    ping=createmsg(d,'PING');ping.Payload.seq=uint32(83);ping.Payload.time_usec=uint64(711);
    wire=uint8(serializemsg(remoteSerializer,ping));sendPeer(wire);frames=pollFrames();
    check('following_official_crc_valid_message_received',numel(frames)==1&&frames.validated&&frames.decoded_message.MsgID==4);
    transport.close();delete(transport);transport=[];
    bad=remainder;bad(6:7)=uint8([1 1]);
    rejects(bad,'actual_source_unsupported_flags_still_rejected');
    report=struct('passed',all([checks.pass]),'checks',checks,'test_count',numel(checks),'hardware_actions',0, ...
        'scope','EXISTING_SERIAL_FORWARDER_PAYLOAD_NONADMISSION__LOCALHOST_ONLY');
    fid=fopen(fullfile(outputRoot,'RESULT.json'),'w');assert(fid>=0);fprintf(fid,'%s\n',jsonencode(report));fclose(fid);disp(report);clear cleanup;return
end
assert(strcmp(selection,'ALL'));
try
    serializer=mavlinkio(d,'SystemID',cfg.local_system,'ComponentID',cfg.local_component);
    remoteSerializer=mavlinkio(d,'SystemID',cfg.remote_system,'ComponentID',cfg.remote_component);
    assert(isempty(listConnections(serializer))&&isempty(listConnections(remoteSerializer)));
    % Reject invalid configuration before opening native state.
    bad=cfg;bad.scope='LIVE';constructorFails(bad,'scope_before_open');
    bad=cfg;bad.remote_host='192.0.2.1';constructorFails(bad,'remote_before_open');
    bad=cfg;bad.local_port=62199;constructorFails(bad,'port_before_open');
    bad=cfg;bad.remote_port=bad.local_port;constructorFails(bad,'same_port_before_open');
    bad=cfg;bad.source.sha256=repmat('0',1,64);constructorFails(bad,'hash_before_open');
    bad=cfg;bad.source.exact_path=fullfile(outputRoot,'missing','gpenmpc_rfly_udp_transport_mex.mexw64');
    constructorFails(bad,'path_before_open');
    localProbe=udpport('datagram','IPV4','LocalHost','127.0.0.1','LocalPort',ports(1));
    delete(localProbe);check('failed_constructors_did_not_open_local_port',true);

    peer=udpport('datagram','IPV4','LocalHost','127.0.0.1','LocalPort',ports(2));
    transport=gpenmpcNative.RflySoleMavlinkTransport(cfg,d,serializer);
    before=transport.status();
    check('sole_owner_and_borrowed_unconnected_serializer', ...
        before.open&&~before.closed&&~before.failed ...
        &&strcmp(before.transport_kind,'OFFICIAL_CODEC_SOLE_RAW_UDP') ...
        &&~before.same_existing_mavlinkio&&~before.owns_serializer ...
        &&isempty(listConnections(serializer)));
    m=createmsg(d,'PING');m.Payload.time_usec=uint64(2^53)+uint64(391);m.Payload.seq=uint32(71);
    sent=transport.sendMessage(m);observed=readPeer();after=transport.status();
    check('one_official_serialization_full_wire_actual_send', ...
        after.serializations==before.serializations+uint64(1) ...
        &&after.serialization_attempts==before.serialization_attempts+uint64(1) ...
        &&isequal(observed(:),sent.bytes(:))&&isequal(sent,after.last_send) ...
        &&sent.ok&&sent.attempted&&sent.bytes_complete ...
        &&isa(sent.submit_ns,'uint64')&&isa(sent.return_ns,'uint64') ...
        &&sent.submit_ns<=sent.return_ns&&sent.submit_qpc<=sent.return_qpc);
    [decoded,status]=deserializemsg(d,observed(:).','OutputAllMessages',true);
    check('sent_payload_uint64_above_flintmax',isscalar(decoded)&&status==0 ...
        &&decoded.Payload.time_usec==m.Payload.time_usec);

    ping=createmsg(d,'PING');ping.Payload.time_usec=uint64(2^53)+uint64(711);ping.Payload.seq=uint32(83);
    tunnel=createmsg(d,'TUNNEL');tunnel.Payload.payload_type=uint16(42002);
    tunnel.Payload.target_system=cfg.local_system;tunnel.Payload.target_component=cfg.local_component;
    tunnel.Payload.payload_length=uint8(128);
    bytes=uint8(mod(0:127,256));bytes(1:8)=uint8([0 0 0 0 0 0 248 127]);
    tunnel.Payload.payload(:)=bytes;
    a=uint8(serializemsg(remoteSerializer,ping));b=uint8(serializemsg(remoteSerializer,tunnel));
    sendPeer([a(:);b(:)]);frames=pollFrames();audit=transport.status();raw{end+1}=audit;
    check('two_original_frames_one_datagram',numel(frames)==2 ...
        &&isequal(frames(1).raw_frame,a(:))&&isequal(frames(2).raw_frame,b(:)) ...
        &&all([frames.validated]));
    check('same_datagram_exact_dequeue_time_not_per_frame_now', ...
        isa(frames(1).original_host_receive_ns,'uint64') ...
        &&isequal(frames(1).original_host_receive_ns,frames(2).original_host_receive_ns) ...
        &&isequal(frames(1).dequeueinfo.dequeue_qpc,frames(2).dequeueinfo.dequeue_qpc) ...
        &&frames(1).dequeueinfo.frame_offset==1 ...
        &&frames(2).dequeueinfo.frame_offset==numel(a)+1 ...
        &&frames(1).dequeueinfo.source_port==cfg.remote_port);
    check('payload_bytes_no_nan_canonicalization', ...
        isequal(frames(2).decoded_message.Payload.payload(:),bytes(:)) ...
        &&frames(1).decoded_message.Payload.time_usec==ping.Payload.time_usec);
    check('raw_native_batch_retained',isfield(audit.last_receive,'records') ...
        &&isequal(audit.last_receive.records(1).bytes(:),[a(:);b(:)]) ...
        &&audit.last_receive.records(1).dequeue_ns==frames(1).original_host_receive_ns);

    % Validate a trailing frame independently of the RLS hint.
    % Preserve the next datagram and its receive timestamp.
    lastRls=tunnel;lastRls.Payload.payload(:)=uint8(0);
    lastRls.Payload.payload_length=uint8(34);lastRls.Payload.payload(1)=uint8(163);
    lastRls.Payload.payload(9)=uint8(1);
    rlsWire=uint8(serializemsg(remoteSerializer,lastRls));
    sendPeer([rlsWire(:);a(:)]);sendPeer(a);pause(.005);
    yielded=pollFrames();pendingAudit=transport.status();
    check('rls_hint_yields_at_datagram_not_frame_boundary',numel(yielded)==2 ...
        &&isequal(yielded(1).raw_frame,rlsWire(:))&&isequal(yielded(2).raw_frame,a(:)));
    tail=pollFrames();
    pendingData=pendingAudit.last_receive.records;
    pendingData=pendingData(arrayfun(@(x)~isempty(x.bytes),pendingData));
    check('rls_yield_retains_following_original_datagram',numel(tail)==1 ...
        &&isequal(tail(1).raw_frame,a(:))&&numel(pendingData)==2 ...
        &&tail(1).original_host_receive_ns==pendingData(2).dequeue_ns);

    % Deliver snapshots dequeued together to the latest-value consumer in one poll.
    lastRls.Payload.payload(9)=uint8(2);newRlsWire=uint8(serializemsg(remoteSerializer,lastRls));
    sendPeer(rlsWire);sendPeer(newRlsWire);sendPeer(a);pause(.005);
    newestBatch=pollFrames();latestAudit=transport.status();tail=pollFrames();
    nativeData=latestAudit.last_receive.records;
    nativeData=nativeData(arrayfun(@(x)~isempty(x.bytes),nativeData));
    check('latest_RLS_in_existing_batch_delivered_before_yield',numel(newestBatch)==2 ...
        &&isequal(newestBatch(1).raw_frame,rlsWire(:))&&isequal(newestBatch(2).raw_frame,newRlsWire(:)));
    check('latest_RLS_order_original_times_and_tail_preserved',numel(tail)==1&&numel(nativeData)==3 ...
        &&newestBatch(1).original_host_receive_ns==nativeData(1).dequeue_ns ...
        &&newestBatch(2).original_host_receive_ns==nativeData(2).dequeue_ns ...
        &&tail(1).original_host_receive_ns==nativeData(3).dequeue_ns);

    % A partial RLS at the end of the first native batch must not wait for
    % another caller control/history iteration before receiving its tail.
    for j=1:7,sendPeer(a);end
    partialRls=lastRls;partialRls.Payload.payload_length=uint8(128);
    partialRls.Payload.payload(1)=uint8(160);
    firstWire=uint8(serializemsg(remoteSerializer,partialRls));sendPeer(firstWire);
    tailWires=cell(3,1);
    for j=1:3
        part=partialRls;part.Payload.payload(1)=uint8(160+j);
        if j==3,part.Payload.payload_length=uint8(34);end
        tailWires{j}=uint8(serializemsg(remoteSerializer,part));sendPeer(tailWires{j});
    end
    pause(.005);refilled=pollFrames();
    check('partial_RLS_refills_once_before_caller_history',numel(refilled)==11 ...
        &&isequal(refilled(8).raw_frame,firstWire(:))&&isequal(refilled(11).raw_frame,tailWires{3}(:)));
    check('partial_refill_preserves_real_receive_time_and_crc_source', ...
        all([refilled.validated])&&all([refilled(2:end).original_host_receive_ns]>=[refilled(1:end-1).original_host_receive_ns]) ...
        &&refilled(9).original_host_receive_ns>=refilled(8).original_host_receive_ns);

    % When a completed RLS precedes a newer partial in the same batch,
    % allow one bounded refill for the partial.
    sendPeer(rlsWire);
    for j=1:5,sendPeer(a);end
    sendPeer(firstWire);sendPeer(tailWires{1});
    sendPeer(tailWires{2});sendPeer(tailWires{3});pause(.005);
    latestSplit=pollFrames();
    check('older_complete_does_not_defer_newer_partial_batch_tail',numel(latestSplit)==10 ...
        &&isequal(latestSplit(1).raw_frame,rlsWire(:)) ...
        &&isequal(latestSplit(7).raw_frame,firstWire(:)) ...
        &&isequal(latestSplit(10).raw_frame,tailWires{3}(:)));
    check('newer_partial_refill_original_order_time_crc_preserved',all([latestSplit.validated]) ...
        &&all([latestSplit(2:end).original_host_receive_ns]>=[latestSplit(1:end-1).original_host_receive_ns]));

    d1=mavlinkdialect(dialectPath,1);
    v1Serializer=mavlinkio(d1,'SystemID',cfg.remote_system,'ComponentID',cfg.remote_component);
    p1=createmsg(d1,'PING');p1.Payload.seq=uint32(912);
    c=uint8(serializemsg(v1Serializer,p1));sendPeer(c);f1=pollFrames();
    check('official_mavlink1_frame',numel(f1)==1&&f1.raw_frame(1)==254 ...
        &&f1.decoded_message.Payload.seq==p1.Payload.seq);
    closeReceipt=transport.close();secondClose=transport.close();
    check('actual_close_idempotent',closeReceipt.ok&&closeReceipt.closed ...
        &&secondClose.ok&&secondClose.closed&&secondClose.already_closed);
    check('borrowed_serializer_remains_valid',isvalid(serializer)&&isempty(listConnections(serializer)));
    delete(transport);transport=[];
    p=udpport('datagram','IPV4','LocalHost','127.0.0.1','LocalPort',ports(1));delete(p);
    check('native_close_releases_port',true);

    malformed=a(:);malformed(end)=bitxor(malformed(end),uint8(1));
    rejects(malformed,'bad_crc_retained');
    malformed=rlsWire(:);malformed(end)=bitxor(malformed(end),uint8(1));
    rejects(malformed,'rls_hint_bad_crc_not_exempt');
    rejects(a(1:end-1),'truncated_retained');
    rejects([a(:);uint8(0)],'trailing_byte_retained_partial_success');
    malformed=a(:);malformed(3)=bitor(malformed(3),uint8(2));
    rejects(malformed,'unknown_incompatibility_flag_retained');
    malformed=a(:);malformed(8:10)=uint8([255;255;255]);
    rejects(malformed,'unknown_message_id_retained');
    rejects([uint8(0);a(:)],'leading_bad_magic_not_skipped');
    wrongSerializer=mavlinkio(d,'SystemID',uint8(2),'ComponentID',cfg.remote_component);
    malformed=serializemsg(wrongSerializer,ping);rejects(malformed,'official_valid_wrong_mavlink_source');
    wrongPeer=udpport('datagram','IPV4','LocalHost','127.0.0.1','LocalPort',ports(3));
    rejects(a,'native_wrong_udp_source',wrongPeer);
    rejects(zeros(301,1,'uint8'),'native_oversize_retained');

    transport=gpenmpcNative.RflySoleMavlinkTransport(cfg,d);
    check('internally_owned_unconnected_serializer',transport.status().owns_serializer);
    prior=transport.status();failed=false;
    try,transport.sendMessage(struct('not_a_message',1));catch,failed=true;end
    final=transport.status();raw{end+1}=final;
    check('serializer_failure_has_no_native_send',failed&&final.failed ...
        &&final.native.send_attempt_count==prior.native.send_attempt_count ...
        &&isequaln(final.last_send,prior.last_send));
    transport.close();delete(transport);transport=[];
    report=struct('pass',all([checks.pass]),'checks',checks,'passed',sum([checks.pass]), ...
        'total',numel(checks),'scope','HOST_ONLY_LOOPBACK', ...
        'COM_calls',0,'board_calls',0,'model_runs',0,'solver_calls',0, ...
        'ports',ports,'source',source, ...
        'time_semantics','ORIGINAL_DATAGRAM_DEQUEUE_INTERVAL_NOT_KERNEL_ARRIVAL', ...
        'class_sha256',sha(fullfile(build,'host_runtime','+gpenmpcNative','RflySoleMavlinkTransport.m')), ...
        'test_sha256',sha([mfilename('fullpath') '.m']));
    save(fullfile(outputRoot,'RAW.mat'),'raw','checks','cfg','sent','frames','f1','report');
    writeJson(fullfile(outputRoot,'RESULT.json'),report);
    assert(report.pass,'One or more actual localhost checks failed.');
    clear cleanup % Execute while the nested cleanup's shared handles still exist.
catch ex
    failure=struct('identifier',ex.identifier,'message',ex.message,'stack',ex.stack);
    if ~isempty(transport),try,raw{end+1}=transport.status();catch,end,end
    save(fullfile(outputRoot,'FAILURE_RAW.mat'),'raw','checks','cfg','failure');
    writeJson(fullfile(outputRoot,'FAILURE.json'),failure);
    clear cleanup % Release handles explicitly.
    rethrow(ex)
end

    function check(name,condition)
        checks(end+1)=struct('name',name,'pass',logical(condition));
        assert(condition,'gpenmpcNative:SoleTransportTest','%s',name);
    end
    function constructorFails(candidate,name)
        failed=false;x=[];
        try,x=gpenmpcNative.RflySoleMavlinkTransport(candidate,d,serializer);catch,failed=true;end
        if ~isempty(x),x.close();delete(x);end
        check(name,failed);
    end
    function sendPeer(bytes,sender)
        if nargin<2,sender=peer;end
        write(sender,uint8(bytes(:).'),'uint8','127.0.0.1',ports(1));
    end
    function bytes=readPeer()
        start=tic;
        while peer.NumDatagramsAvailable==0&&toc(start)<2,pause(.002);end
        assert(peer.NumDatagramsAvailable>0,'No actual native send reached peer.');
        datagram=read(peer,1,'uint8');
        if istable(datagram),v=datagram.Data;if iscell(v),v=v{1};end;else,v=datagram(1).Data;end
        bytes=uint8(v(:));
    end
    function frames=pollFrames()
        frames=struct([]);start=tic;
        while isempty(frames)&&toc(start)<2,frames=transport.poll();if isempty(frames),pause(.002);end,end
        assert(~isempty(frames),'No actual datagram was dequeued.');
    end
    function rejects(bytes,name,sender)
        if nargin<3,sender=peer;end
        transport=gpenmpcNative.RflySoleMavlinkTransport(cfg,d,serializer);
        sendPeer(bytes,sender);failed=false;start=tic;
        while ~failed&&toc(start)<2
            try,transport.poll();catch,failed=true;end
            if ~failed,pause(.002);end
        end
        s=transport.status();raw{end+1}=s;
        check(name,failed&&s.failed&&~isempty(fieldnames(s.first_error)));
        records=s.last_receive.records;nonempty=arrayfun(@(x)~isempty(x.bytes),records);
        records=records(nonempty);
        check([name '_raw_preserved'],~isempty(records)&&isequal(records(1).bytes(:),uint8(bytes(:))));
        rawBefore=s.last_receive;first=s.first_error;blocked=false;
        try,transport.poll();catch,blocked=true;end
        after=transport.status();
        check([name '_sticky_no_second_receive'],blocked ...
            &&isequaln(after.last_receive,rawBefore)&&isequaln(after.first_error,first));
        if strcmp(name,'trailing_byte_retained_partial_success')
            check('valid_prefix_and_invalid_tail_both_retained',numel(s.last_frames)==2 ...
                &&s.last_frames(1).validated&&~s.last_frames(2).validated ...
                &&isequal(s.last_frames(2).raw_frame,uint8(0)));
        end
        transport.close();delete(transport);transport=[];
    end
    function finish()
        if ~isempty(transport),try,transport.close();catch,end;try,delete(transport);catch,end,end
        objects={peer,wrongPeer,serializer,remoteSerializer,v1Serializer,wrongSerializer};
        for i=1:numel(objects),if ~isempty(objects{i}),try,delete(objects{i});catch,end,end,end
    end
end
function report=inlineReceiveCheck(cfg,d,outputRoot)
% Test parse and dispatch with localhost telemetry fixtures.
t=[];peer=[];sender=[];observed={};delay=true;delayPrivate=false;
cleanup=onCleanup(@finish);
try
peer=udpport('datagram','IPV4','LocalHost','127.0.0.1','LocalPort',cfg.remote_port);
sender=mavlinkio(d,'SystemID',cfg.remote_system,'ComponentID',cfg.remote_component);
t=gpenmpcNative.RflySoleMavlinkTransport(cfg,d);
[empty,~,pending]=t.poll(@receive);
assert(isempty(empty)&&islogical(pending)&&isscalar(pending)&&~pending);
m=createmsg(d,'PING');m.Payload.seq=uint32(31);a=uint8(serializemsg(sender,m));
for k=1:3,write(peer,a,'uint8','127.0.0.1',cfg.local_port);end
pause(.01);[first,~]=t.poll(@receive);s=t.status();
assert(numel(first)==1&&numel(observed)==1&&isequaln(first(1),observed{1}));
% A 10-ms consumer forces the existing 8-ms cooperative slice to yield
% after its atomic datagram, not after the whole three-callback batch.
delay=false;tail=struct([]);
for pollIndex=1:3
 more=t.poll(@receive);tail=[tail;more];
 if numel(observed)==3,break;end
end
assert(numel(tail)==2&&numel(observed)==3);
for k=1:2
 assert(isequaln(tail(k),observed{k+1})&&tail(k).validated ...
  &&tail(k).original_host_receive_ns==s.last_receive.records(k+1).dequeue_ns);
end
% Deliver the completed CRC-checked state before pursuing a newer message.
% The body and age consumer validates it; retain later bytes in order.
rls=createmsg(d,'TUNNEL');rls.Payload.payload_type=uint16(42002);
rls.Payload.target_system=cfg.local_system;rls.Payload.target_component=cfg.local_component;
rls.Payload.payload_length=uint8(34);rls.Payload.payload(1)=uint8(163);
w=uint8(serializemsg(sender,rls));write(peer,w,'uint8','127.0.0.1',cfg.local_port);
for j=0:3
 rls.Payload.payload_length=uint8(128);if j==3,rls.Payload.payload_length=uint8(34);end
 rls.Payload.payload(1)=uint8(160+j);w=uint8(serializemsg(sender,rls));
 write(peer,w,'uint8','127.0.0.1',cfg.local_port);
end
pause(.01);delay=true;before=numel(observed);
[older,~,pending]=t.poll(@receive);batch=t.status().last_receive.records;
assert(numel(older)==1&&pending&&numel(observed)==before+1);
delay=false;newer=struct([]);
for pollIndex=1:8
 [more,~,pending]=t.poll(@receive);newer=[newer;more];
 if ~pending,break;end
end
assert(~pending&&numel(newer)==4);
for j=1:4
 assert(newer(j).validated&&newer(j).original_host_receive_ns==batch(j+1).dequeue_ns ...
  &&newer(j).decoded_message.Payload.payload(1)==uint8(159+j));
end
[empty,~,pending]=t.poll(@receive);
assert(isempty(empty)&&islogical(pending)&&isscalar(pending)&&~pending);
% A complete state is usable now even if its successor has only one fragment.
% Leave the successor buffered with its original receive time; no CRC bypass.
rls.Payload.payload_length=uint8(34);rls.Payload.payload(1)=uint8(163);
w=uint8(serializemsg(sender,rls));write(peer,w,'uint8','127.0.0.1',cfg.local_port);
rls.Payload.payload_length=uint8(128);rls.Payload.payload(1)=uint8(160);
w=uint8(serializemsg(sender,rls));write(peer,w,'uint8','127.0.0.1',cfg.local_port);
pause(.01);[complete,~,pending]=t.poll(@receive);batch=t.status().last_receive.records;
assert(numel(complete)==1&&~pending&&complete(1).validated);
[partial,~,pending]=t.poll(@receive);
assert(numel(partial)==1&&pending&&partial(1).validated ...
 &&partial(1).original_host_receive_ns==batch(2).dequeue_ns);
for j=1:3
 rls.Payload.payload_length=uint8(128);if j==3,rls.Payload.payload_length=uint8(34);end
 rls.Payload.payload(1)=uint8(160+j);w=uint8(serializemsg(sender,rls));
 write(peer,w,'uint8','127.0.0.1',cfg.local_port);
end
pause(.01);newer=struct([]);
for pollIndex=1:8
 [more,~,pending]=t.poll(@receive);newer=[newer;more];
 if ~pending,break;end
end
assert(~pending&&numel(newer)==3&&all([newer.validated]));
% Retain the latest completed hint across GP/history parsing and bounded callback yields.
rls.Payload.payload_length=uint8(34);rls.Payload.payload(1)=uint8(163);
w=uint8(serializemsg(sender,rls));
write(peer,w,'uint8','127.0.0.1',cfg.local_port);
write(peer,a,'uint8','127.0.0.1',cfg.local_port);
write(peer,w,'uint8','127.0.0.1',cfg.local_port);
pause(.01);[adjacent,~,pending]=t.poll(@receive);
assert(numel(adjacent)==3&&~pending&&all([adjacent.validated]));
gp=rls;gp.Payload.payload_length=uint8(128);gp.Payload.payload(1)=uint8(128);
wg=uint8(serializemsg(sender,gp));
write(peer,w,'uint8','127.0.0.1',cfg.local_port);
write(peer,wg,'uint8','127.0.0.1',cfg.local_port);
write(peer,w,'uint8','127.0.0.1',cfg.local_port);
pause(.01);delayPrivate=true;[beforeGp,~,pending]=t.poll(@receive);batch=t.status().last_receive.records;
assert(numel(beforeGp)==2&&pending&&all([beforeGp.validated]) ...
 &&beforeGp(1).decoded_message.Payload.payload(1)==uint8(163) ...
 &&beforeGp(2).decoded_message.Payload.payload(1)==uint8(128));
delayPrivate=false;[afterPrivateWork,~,pending]=t.poll(@receive);
assert(numel(afterPrivateWork)==1&&~pending&&afterPrivateWork(1).validated ...
 &&afterPrivateWork(1).decoded_message.Payload.payload(1)==uint8(163) ...
 &&afterPrivateWork(1).original_host_receive_ns==batch(3).dequeue_ns);
% Exhaust slices with seven slow history callbacks while a new RLS waits in the socket.
for j=1:8,write(peer,a,'uint8','127.0.0.1',cfg.local_port);end
pause(.01);delay=true;
for j=1:7,one=t.poll(@receive);assert(numel(one)==1);end
for j=0:3
 rls.Payload.payload_length=uint8(128);if j==3,rls.Payload.payload_length=uint8(34);end
 rls.Payload.payload(1)=uint8(160+j);w=uint8(serializemsg(sender,rls));
 write(peer,w,'uint8','127.0.0.1',cfg.local_port);
end
pause(.01);delay=false;
[refilled,~,pending]=t.poll(@receive);
assert(numel(refilled)==5&&~pending&&refilled(1).decoded_message.MsgID==4 ...
 &&refilled(end).decoded_message.Payload.payload(1)==uint8(163) ...
 &&all([refilled.validated]) ...
 &&all([refilled(2:end).original_host_receive_ns]>=[refilled(1:end-1).original_host_receive_ns]));
delivered=numel(observed);
% Yield for a completed GP request, not for its prefix alone.
gp.Payload.payload_length=uint8(81);gp.Payload.payload(1)=uint8(130);
wfinal=uint8(serializemsg(sender,gp));
write(peer,w,'uint8','127.0.0.1',cfg.local_port);
write(peer,wg,'uint8','127.0.0.1',cfg.local_port);
write(peer,wfinal,'uint8','127.0.0.1',cfg.local_port);
write(peer,w,'uint8','127.0.0.1',cfg.local_port);
pause(.01);[priority,~,pending]=t.poll(@receive);batch=t.status().last_receive.records;
assert(numel(priority)==3&&priority(1).decoded_message.Payload.payload(1)==uint8(163) ...
 &&priority(end).decoded_message.Payload.payload(1)==uint8(130)&&pending);
[remaining,~,pending]=t.poll(@receive);
assert(numel(remaining)==1&&~pending&&remaining(1).validated ...
 &&remaining(1).original_host_receive_ns==batch(4).dequeue_ns);
delivered=numel(observed);
bad=a;bad(end)=bitxor(bad(end),uint8(1));write(peer,bad,'uint8','127.0.0.1',cfg.local_port);
pause(.005);rejected=false;try,t.poll(@receive);catch,rejected=true;end
assert(rejected&&numel(observed)==delivered&&t.status().failed);
closed=t.close();assert(closed.closed&&closed.ok);
report=struct('passed',true,'selection','INLINE_RECEIVE','delivered_once',delivered, ...
 'consumer_time_in_work_slice',true,'original_times_preserved',true, ...
 'complete_state_delivered_before_later_batch_work',true,'complete_precedes_unfinished_successor',true, ...
 'latest_complete_hint_across_receipt_work_within_original_slice',true, ...
 'resumed_private_work_preserves_later_RLS_receive_dependency',true, ...
 'history_tail_refills_within_same_work_slice',true, ...
 'bad_crc_never_delivered',true,'COM',0,'board',0,'model',0,'controls',0);
writeJson(fullfile(outputRoot,'INLINE_RECEIVE_RESULT.json'),report);disp(jsonencode(report));
clear cleanup
catch ex
 clear cleanup
 rethrow(ex)
end
 function receive(r)
  observed{end+1}=r;
  if delay||(delayPrivate&&r.decoded_message.MsgID==385&&r.decoded_message.Payload.payload(1)==uint8(128)),pause(.010);end
 end
 function finish()
  if ~isempty(t),try,t.close();catch,end;delete(t);end
  if ~isempty(peer),delete(peer);end
  if ~isempty(sender),delete(sender);end
 end
end
function report=rcContinuousReceiveCheck(cfg,outputRoot,build)
% Test the RC receive branch over localhost UDP.
addpath(fileparts(cfg.source.exact_path),'-begin');
gpDir=fullfile(build,'runtime_assets','gp_wire');addpath(gpDir);
a=gpenmpcNative.loadCanonicalAssets();
binding=struct('kind','CANONICAL_GP_WIRE_MEX','path',fullfile(gpDir,'canonical_gp_wire_mex.mexw64'), ...
 'binary_sha256','A20AB9DAA20749EFB7E549DDBD26DE29AC7F34C2FA5C4A73FB5B4A348D1281D6','receive_inline',true);
prepared=gpenmpcNative.RflyLocalGpService.prepareBackend(a,binding);
f=fopen(fullfile(build,'rfly_vendor_integration','full_inner_abi','snapshot_wire_fixture','RGP1_RGR1_PAIRS.bin'),'rb');
assert(f>=0);bytes=fread(f,310,'*uint8');fclose(f);
q=gpenmpcNative.RflyLocalGpCodec.decodeRequest(bytes);
e=struct('uid',q.identity.uid,'boot_generation',q.identity.boot_generation, ...
 'board_system',q.identity.system,'board_component',q.identity.component,'host_system',uint8(255),'host_component',uint8(190), ...
 'link_lifecycle_generation',uint64(1),'confirmed_host_rx_ns',gpenmpcNative.rflyOriginalHostMonotonicNs(), ...
 'execution_session_sha256',repmat('A',1,64),'gp_backend',binding);
gp=gpenmpcNative.RflyLocalGpService(a,e,prepared);cg=onCleanup(@()gp.close()); %#ok<NASGU>
d=mavlinkdialect(fullfile(build,'m600_coptersim','matlab_validation','+m600check','local_full_inner_dialect','px4_local_full_inner.xml'),2);
cfg.local_system=uint8(255);cfg.local_component=uint8(190);cfg.remote_system=q.identity.system;cfg.remote_component=q.identity.component;
cfg.maximum_poll_datagrams=64;cfg.copter_serial_forwarding=true;
t=gpenmpcNative.RflySoleMavlinkTransport(cfg,d);ct=onCleanup(@()t.close()); %#ok<NASGU>
peer=udpport('datagram','IPV4','LocalHost','127.0.0.1','LocalPort',cfg.remote_port);cp=onCleanup(@()delete(peer)); %#ok<NASGU>
sender=mavlinkio(d,'SystemID',cfg.remote_system,'ComponentID',cfg.remote_component);cs=onCleanup(@()delete(sender)); %#ok<NASGU>
% Isolate queue ordering from cold JIT time with synthetic fixtures.
t.attachInlineGp(gp,true,uint64(0),uint64(4096),true);
s=t.status();assert(s.native.continuous_gp.active&&~s.native.continuous_gp.prediction_enabled);
observed={};slow=true;
ping=createmsg(d,'PING');ping.Payload.seq=uint32(31);send(ping);
rls=createmsg(d,'TUNNEL');rls.Payload.payload_type=uint16(42002);
rls.Payload.target_system=cfg.local_system;rls.Payload.target_component=cfg.local_component;
rls.Payload.payload_length=uint8(34);rls.Payload.payload(1)=uint8(163);send(rls);
pause(.03);s=t.status();assert(s.native.continuous_gp.queued_datagrams==2&&numel(observed)==0);
[first,~,pending]=t.poll(@receive);batch=t.status().last_receive.records;
assert(numel(first)==1&&pending&&numel(batch)==2);
% Keep the dequeued target stable while a newer state arrives during suspension.
send(rls);pause(.02);slow=false;
[second,~,pending]=t.poll(@receive);
assert(numel(second)==1&&~pending&&second.validated ...
 &&second.original_host_receive_ns==batch(2).dequeue_ns);
[third,~,pending]=t.poll(@receive);assert(numel(third)==1&&~pending&&third.validated);
assert(third.original_host_receive_ns>second.original_host_receive_ns);
% Existing partial-state continuation still receives the next fragment.
rls.Payload.payload_length=uint8(128);rls.Payload.payload(1)=uint8(160);send(rls);
pause(.02);[partial,~,pending]=t.poll(@receive);assert(numel(partial)==1&&pending);
for j=1:3
 rls.Payload.payload_length=uint8(128);if j==3,rls.Payload.payload_length=uint8(34);end
 rls.Payload.payload(1)=uint8(160+j);send(rls);
end
pause(.02);tail=struct([]);
for k=1:8
 [more,~,pending]=t.poll(@receive);tail=[tail;more]; %#ok<AGROW>
 if ~pending,break;end
end
assert(numel(tail)==3&&~pending&&all([tail.validated]));
s=t.status();assert(s.native.continuous_gp.queries==0&&s.native.continuous_gp.completed==0 ...
 &&s.native.continuous_gp.replies==0&&gp.Calls==0&&peer.NumDatagramsAvailable==0);
% Wrong-mode GP requests cannot silently enable inference in an RC session.
rls.Payload.payload_length=uint8(128);rls.Payload.payload(1)=uint8(128);
rls.Payload.payload(2:9)=bytes(39:46);rls.Payload.payload(10:128)=bytes(1:119);send(rls);pause(.02);
s=t.status();assert(s.failed&&s.native.continuous_gp.queries==0&&gp.Calls==0);
closed=t.close();assert(closed.closed&&closed.ok);
report=struct('passed',true,'selection','RC_CONTINUOUS_RECEIVE','independent_receiver',true, ...
 'complete_batch_target_fixed',true,'original_timestamps_retained',true,'partial_continuation',true, ...
 'unexpected_gp_rejected_without_inference',true,'GP_calls',gp.Calls,'hardware_actions',0);
writeJson(fullfile(outputRoot,'RC_CONTINUOUS_RECEIVE_RESULT.json'),report);disp(jsonencode(report));
 function send(m),write(peer,uint8(serializemsg(sender,m)),'uint8','127.0.0.1',cfg.local_port);end
 function receive(r),observed{end+1}=r;if slow,pause(.01);end;end
end
function h=sha(path)
f=fopen(path,'rb');assert(f>=0);c=onCleanup(@()fclose(f)); %#ok<NASGU>
m=java.security.MessageDigest.getInstance('SHA-256');
while ~feof(f),v=fread(f,1048576,'*uint8');m.update(typecast(v,'int8'));end
h=upper(reshape(dec2hex(typecast(m.digest(),'uint8'),2).',1,[]));
end
function writeJson(path,value)
f=fopen(path,'w');assert(f>=0);c=onCleanup(@()fclose(f)); %#ok<NASGU>
fwrite(f,jsonencode(value,PrettyPrint=true),'char');
end
