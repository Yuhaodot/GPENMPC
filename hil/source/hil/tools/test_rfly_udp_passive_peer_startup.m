function report=test_rfly_udp_passive_peer_startup()
% Test passive startup, valid peer detection and terminal failure through actual IO.
build=fileparts(fileparts(mfilename('fullpath')));target=fullfile(build,'tools','UDP_PASSIVE_PEER_STARTUP_HOST_ONLY.mat');
assert(~isfile(target),'gpenmpcHost:ExistingEvidence','Choose an unused output path.');
addpath(fullfile(build,'tools'),fullfile(build,'host_runtime'),fullfile(build,'matlab_validation'),fullfile(build,'m600_coptersim','matlab_validation'));
addpath(gpenmpc_external_path('native_visual_host_source'),'-end');
% Reuse the prior test's actual IO configuration and 622xx endpoints.
q=load(gpenmpc_external_path('udp_loopback_fixture'),'cfg');cfg=q.cfg;
assert(strcmp(cfg.mavlink_transport.scope,'HOST_ONLY_LOOPBACK')&&cfg.local_mavlink_port==62291&&cfg.remote_mavlink_port==62292);
addpath(fileparts(cfg.mavlink_transport.source.exact_path));
ioPath=which('m600check.makeM600CopterSimIo');classPath=which('gpenmpcNative.RflySoleMavlinkTransport');
sourceBefore={m600check.fileSha256(ioPath),m600check.fileSha256(classPath)};
io=[];peer=[];remote=[];raw=struct();checks=struct();guard=onCleanup(@finish); %#ok<NASGU>
try
    peer=udpport('datagram','IPV4','LocalHost','127.0.0.1','LocalPort',62292);delete(peer);peer=[];
    io=m600check.makeM600CopterSimIo(cfg);
    raw.empty=cell(1,3);
    for k=1:3
        s=io.snapshot();sent=io.sendHeartbeat();n=gpenmpc_rfly_udp_transport_mex('status');
        raw.empty{k}=struct('snapshot',s,'heartbeat_sent',sent,'native',n);
        assert(~sent&&~s.automatic_mavlink_peer_ready&&n.sent_count==0&&~n.failed, ...
            'gpenmpcHost:EarlySend','No automatic write is permitted before actual peer heartbeat.');
        pause(.005);
    end
    checks.unbound_peer_automatic_sends_zero=true;
    peer=udpport('datagram','IPV4','LocalHost','127.0.0.1','LocalPort',62292);
    d=mavlinkdialect(fullfile(build,'m600_coptersim','matlab_validation','+m600check','px4_health_events.xml'),2);
    remote=mavlinkio(d,'SystemID',uint8(1),'ComponentID',uint8(1));
    hb=createmsg(d,'HEARTBEAT');hb.Payload.mavlink_version=uint8(3);
    wire=serializemsg(remote,hb);write(peer,wire,'uint8','127.0.0.1',62291);pause(.02);
    s=io.snapshot();sent=io.sendHeartbeat();n=gpenmpc_rfly_udp_transport_mex('status');
    raw.ready=struct('snapshot',s,'heartbeat_sent',sent,'native',n,'peer_heartbeat_bytes',wire);
    assert(s.automatic_mavlink_peer_ready&&sent&&n.sent_count==2&&n.received_count==1&&~n.failed, ...
        'gpenmpcHost:PeerNotAdmitted','Valid same-peer heartbeat must enable exactly original TIMESYNC and heartbeat.');
    start=tic;while peer.NumDatagramsAvailable<2&&toc(start)<1,pause(.002);end
    assert(peer.NumDatagramsAvailable==2,'gpenmpcHost:PeerReceipts','Peer must observe both original sends.');
    rows=read(peer,2,'uint8');raw.ready_peer_datagrams=rows;ids=zeros(1,2);
    for k=1:2
        bytes=bytesAt(rows,k);[m,status]=deserializemsg(d,bytes.',OutputAllMessage=true);
        assert(isscalar(m)&&status==0&&m.SystemID==255&&m.ComponentID==190);ids(k)=double(m.MsgID);
    end
    checks.valid_peer_enables_original_timesync_and_heartbeat=isequal(ids,[111 0]);
    % One deliberately bad CRC from the same peer must latch the production
    % class/IO fatal. A later valid frame cannot reopen automatic sending.
    bad=serializemsg(remote,hb);bad(end)=bitxor(bad(end),uint8(1));
    write(peer,bad,'uint8','127.0.0.1',62291);pause(.02);
    s=io.snapshot();sent=io.sendHeartbeat();raw.fatal=struct('snapshot',s,'heartbeat_sent',sent,'io',io.evidence(),'bad_wire',bad);
    assert(~isempty(s.fatal)&&~s.automatic_mavlink_peer_ready&&~sent);
    good=serializemsg(remote,hb);write(peer,good,'uint8','127.0.0.1',62291);pause(.02);
    s=io.snapshot();sent=io.sendHeartbeat();n=gpenmpc_rfly_udp_transport_mex('status');
    raw.after_fatal=struct('snapshot',s,'heartbeat_sent',sent,'native',n,'io',io.evidence());
    checks.fatal_and_later_valid_frame_do_not_restart_tx=~isempty(s.fatal)&&~s.automatic_mavlink_peer_ready&&~sent&&n.sent_count==2;
    checks.source_unchanged_during_test=isequal(sourceBefore,{m600check.fileSha256(ioPath),m600check.fileSha256(classPath)});
    report=struct('scope','HOST_ONLY_PRODUCTION_IO_PASSIVE_PEER_STARTUP','pass',all(structfun(@logical,checks)), ...
        'checks',checks,'io_source_sha256',sourceBefore{1},'transport_class_sha256',sourceBefore{2}, ...
        'mex_sha256',cfg.mavlink_transport.source.sha256,'COM',0,'CopterSim',0,'board',0,'HIL',0,'control_commands',0);
    clear guard
    save(target,'raw','report','cfg');disp(jsonencode(report));
catch ex
    failure=struct('identifier',ex.identifier,'message',ex.message,'stack',ex.stack);
    if ~isempty(io),try,raw.failure_io=io.evidence();catch,end,end
    clear guard
    save(target,'raw','checks','cfg','failure');rethrow(ex)
end
    function finish()
        if ~isempty(io),try,io.close();catch,end;io=[];end
        if ~isempty(remote),delete(remote);remote=[];end
        if ~isempty(peer),delete(peer);peer=[];end
    end
end
function b=bytesAt(rows,k)
if istable(rows),v=rows.Data;if iscell(v),b=v{k};else,b=v(k,:);end;else,b=rows(k).Data;end
b=uint8(b(:));
end
