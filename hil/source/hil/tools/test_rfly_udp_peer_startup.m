function report=test_rfly_udp_peer_startup()
% Test two bounded loopback peer-startup cases.
build=fileparts(fileparts(mfilename('fullpath')));
target=fullfile(build,'tools','UDP_PEER_STARTUP_HOST_ONLY.mat');
assert(~isfile(target),'gpenmpcHost:ExistingEvidence','Choose an unused output path.');
mexDir=fullfile(gpenmpc_external_path('rfly_udp_transport_build'),'production');
addpath(fullfile(build,'host_runtime'),mexDir);
source=struct('kind','GPENMPC_RFLY_UDP_TRANSPORT_MEX', ...
    'exact_path',fullfile(mexDir,'gpenmpc_rfly_udp_transport_mex.mexw64'), ...
    'sha256','412250893D93D8035A4D2F357E2862807BF11A1333CE3CF7653A128B3E3EAC32');
cfg=struct('source',source,'scope','HOST_ONLY_LOOPBACK','allow_loopback',true, ...
    'local_host','127.0.0.1','remote_host','127.0.0.1', ...
    'local_port',62281,'remote_port',62282,'local_system',uint8(245), ...
    'local_component',uint8(190),'remote_system',uint8(1), ...
    'remote_component',uint8(1),'maximum_poll_datagrams',8);
d=mavlinkdialect(fullfile(build,'m600_coptersim','matlab_validation', ...
    '+m600check','px4_health_events.xml'),2);
peer=[];transport=[];probe=[];remote=[];raw=struct();
guard=onCleanup(@finish); %#ok<NASGU>
try
    % Explicitly prove these isolated fixture endpoints are available first.
    probe=udpport('datagram','IPV4','LocalHost','127.0.0.1','LocalPort',cfg.local_port);
    peer=udpport('datagram','IPV4','LocalHost','127.0.0.1','LocalPort',cfg.remote_port);
    delete(probe);probe=[];delete(peer);peer=[];
    message=createmsg(d,'TIMESYNC');message.Payload.tc1=int64(0);message.Payload.ts1=int64(123456789);

    transport=gpenmpcNative.RflySoleMavlinkTransport(cfg,d);
    raw.early_send=transport.sendMessage(message);
    started=tic;faultIdentifier='';
    while toc(started)<1
        try,transport.poll();catch ex,faultIdentifier=ex.identifier;break,end
        pause(.005);
    end
    raw.early=transport.status();
    raw.early_elapsed_s=toc(started);raw.early_identifier=faultIdentifier;
    errors=[raw.early.native.last_receive.records.socket_error];
    reproduced=strcmp(faultIdentifier,'gpenmpcNative:UdpTransportReceiveSocket') ...
        &&any(errors==int32(10054))&&raw.early.failed ...
        &&raw.early.native.sent_count==1&&raw.early.native.received_count==0;
    transport.close();delete(transport);transport=[];

    % Sole change: bind actual remote peer before the exact TIMESYNC send.
    peer=udpport('datagram','IPV4','LocalHost','127.0.0.1','LocalPort',cfg.remote_port);
    transport=gpenmpcNative.RflySoleMavlinkTransport(cfg,d);
    raw.ready_send=transport.sendMessage(message);
    started=tic;
    while peer.NumDatagramsAvailable==0&&toc(started)<1,pause(.005);end
    assert(peer.NumDatagramsAvailable==1,'gpenmpcHost:NoPeerReceipt','Peer did not receive exact send.');
    sent=read(peer,1,'uint8');raw.ready_peer_received=sent;
    remote=mavlinkio(d,'SystemID',uint8(1),'ComponentID',uint8(1));
    reply=createmsg(d,'HEARTBEAT');reply.Payload.mavlink_version=uint8(3);
    bytes=serializemsg(remote,reply);
    write(peer,bytes,'uint8','127.0.0.1',cfg.local_port);
    frames=struct([]);started=tic;
    while isempty(frames)&&toc(started)<1,frames=transport.poll();if isempty(frames),pause(.005);end,end
    raw.ready=transport.status();raw.ready_frames=frames;
    readyPass=numel(frames)==1&&frames(1).validated ...
        &&frames(1).decoded_message.MsgID==0&&~raw.ready.failed ...
        &&raw.ready.native.sent_count==1&&raw.ready.native.received_count==1;
    report=struct('scope','HOST_ONLY_PEER_BIND_STARTUP_DIFFERENTIAL', ...
        'closed_peer_reproduced_10054',reproduced,'bound_peer_positive_pass',readyPass, ...
        'pass',reproduced&&readyPass,'ports',[cfg.local_port cfg.remote_port], ...
        'COM',0,'CopterSim',0,'board',0,'HIL',0);
    finish();save(target,'report','raw','cfg');disp(jsonencode(report));
    assert(report.pass,'gpenmpcHost:StartupDifferential','Startup hypothesis not established by both cases.');
catch ex
    if ~isempty(transport),try,raw.failure_native=transport.status();catch,end,end
    failure=struct('identifier',ex.identifier,'message',ex.message,'stack',ex.stack);
    finish();save(target,'raw','cfg','failure');rethrow(ex)
end
    function finish()
        if ~isempty(transport),try,transport.close();catch,end;delete(transport);transport=[];end
        if ~isempty(peer),delete(peer);peer=[];end
        if ~isempty(probe),delete(probe);probe=[];end
        if ~isempty(remote),delete(remote);remote=[];end
    end
end
