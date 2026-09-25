function report=test_canonical_se3_adapter_loopback(outputRoot)
% Test the production adapter with a localhost SERIAL_CONTROL peer.
arguments,outputRoot (1,1) string,end
build=string(fileparts(fileparts(mfilename('fullpath'))));
outputRoot=string(char(java.io.File(char(outputRoot)).getCanonicalPath()));
assert(startsWith(lower(outputRoot),lower(build+"\evidence\"))&&~isfolder(outputRoot));
mkdir(outputRoot);oldPath=path;pg=onCleanup(@()path(oldPath)); %#ok<NASGU>
addpath(fullfile(build,'host_runtime'),fullfile(build,'matlab_validation'), ...
    fullfile(build,'m600_coptersim','matlab_validation'),'-begin');
ports=48231:48235;checks=struct('name',{},'pass',{});failure='';raw=struct();
io=[];peer=[];encoder=[];wrong=[];dialect=[];queryRows={};peerErrors={};
shellPeer=[];shellPeerStderr='';replyPackets={};cleanup=[];
cg=onCleanup(@finish);
adapterPath=which('m600check.makeM600CopterSimIo');adapterSha=m600check.fileSha256(adapterPath);
helperSha=m600check.fileSha256(which('gpenmpcNative.advanceSe3StatusObservation'));
cfg=configuration();
try
    for p=ports,u=udpport('datagram','IPV4','LocalHost','127.0.0.1','LocalPort',p);delete(u);end
    check('five_high_ports_initially_available',true);
    dialect=mavlinkdialect('common.xml',2);
    encoder=mavlinkio(dialect,'SystemID',231,'ComponentID',77);
    wrong=mavlinkio(dialect,'SystemID',232,'ComponentID',78);
    peer=udpport('datagram','IPV4','LocalHost','127.0.0.1','LocalPort',48235);
    text=fixture(1000000,10,9);replyPackets=encodeText(text,encoder);
    % Warm codecs without opening any additional transport endpoints.
    for k=1:numel(replyPackets),deserializemsg(dialect,replyPackets{k});end
    warm=cfg.canonical_se3_status_observation;warm.now_ns=1e9;
    [ws,~]=gpenmpcNative.advanceSe3StatusObservation([],'INIT',warm);
    [ws,wt]=gpenmpcNative.advanceSe3StatusObservation(ws,'TICK',struct('now_ns',1e9,'boot_generation',7));
    [ws,~]=gpenmpcNative.advanceSe3StatusObservation(ws,'TX_RESULT',struct('now_ns',1e9, ...
        'boot_generation',7,'request_generation',wt.request_generation,'send_succeeded',true));
    for k=1:numel(replyPackets)
        [ws,wr]=gpenmpcNative.advanceSe3StatusObservation(ws,'MESSAGE', ...
            struct('rx_ns',1.010e9,'boot_generation',7,'message',deserializemsg(dialect,replyPackets{k})));
    end
    check('pure_reducer_codec_prewarm_complete_no_io',wr.completed&&ws.status_generation==1);
    payloads={replyPackets,replyPackets,encodeText(fixture(1200000,12,11),encoder), ...
        encodeText(strrep(fixture(1400000,14,13),'    dt_s: 0.004','    unknown_status: 1'),encoder)};
    payload64=cellfun(@(p)char(matlab.net.base64encode([p{:}])),payloads,'UniformOutput',false);
    command=["$ErrorActionPreference='Stop';$payloads=@('",strjoin(string(payload64),"','"), ...
        "');$u=[System.Net.Sockets.UdpClient]::new(48232);try{$u.Client.ReceiveTimeout=10000;", ...
        "[Console]::WriteLine('READY');for($i=0;$i -lt 5;$i++){", ...
        "$ep=[System.Net.IPEndPoint]::new([System.Net.IPAddress]::Loopback,0);$b=$u.Receive([ref]$ep);", ...
        "if($ep.Address.ToString() -ne '127.0.0.1' -or $ep.Port -ne 48231){throw 'unexpected source'};", ...
        "[Console]::WriteLine('RX:'+ [Convert]::ToBase64String($b));", ...
        "if($i -lt 4){$r=[Convert]::FromBase64String($payloads[$i]);", ...
        "[void]$u.Send($r,$r.Length,'127.0.0.1',48231)}}}finally{$u.Dispose()}"];
    command=char(join(command,''));
    shellPeer=System.Diagnostics.Process();shellPeer.StartInfo.FileName= ...
        fullfile(getenv('WINDIR'),'System32','WindowsPowerShell','v1.0','powershell.exe');
    shellPeer.StartInfo.Arguments=['-NoLogo -NoProfile -NonInteractive -Command "',command,'"'];
    shellPeer.StartInfo.UseShellExecute=false;shellPeer.StartInfo.CreateNoWindow=true;
    shellPeer.StartInfo.RedirectStandardOutput=true;shellPeer.StartInfo.RedirectStandardError=true;
    assert(shellPeer.Start());readyTask=shellPeer.StandardOutput.ReadLineAsync();assert(readyTask.Wait(5000));
    assert(strcmp(char(readyTask.Result),'READY'),'Fixed localhost responder failed to initialize.');
    check('bounded_independent_localhost_peer_ready',~shellPeer.HasExited);
    io=m600check.makeM600CopterSimIo(cfg);sendHeartbeat();pause(.03);
    ev=io.evidence();
    check('optional_production_subscription_enabled',ev.canonical_module_observer_enabled&& ...
        any(strcmp(ev.subscribed_topics,'SERIAL_CONTROL')));
    check('constructor_and_receive_do_not_start_query',isempty(ev.raw_transmit_messages)&& ...
        ev.canonical_module_observer_state.queries_prepared==0&&isempty(queryRows));
    tx=io.pollCanonicalModuleStatus();pollReturnNs=round(io.now()*1e9);raw.first_send_receipt=tx;
    raw.first=waitComplete(1);guard=io.canonicalModuleGuard();
    collectQuery();
    check('production_query_send_and_owner_return_recorded',tx.query_prepared&& ...
        tx.owner_send_receipt.snapshot.send_attempts==1&&tx.owner_send_receipt.snapshot.queries_sent==1);
    check('peer_received_exact_fixed_readonly_query',numel(queryRows)==1&&queryIsFixed(queryRows{1}.message));
    atom=raw.first.canonical_module_observer_state.latest;
    check('actual_callback_reassembles_fragmented_listener',numel(replyPackets)>10&& ...
        raw.first.canonical_module_observer_state.chunks_received==numel(replyPackets)&& ...
        isequal(atom.raw_bytes,uint8(text)));
    check('metadata_and_original_board_times_preserved',atom.system_id==231&&atom.component_id==77&& ...
        strcmp(atom.uid,'1234605616436508552')&&atom.boot_generation==7&& ...
        atom.timestamp==1000000&&atom.sample_timestamp==999995&& ...
        atom.reference_timestamp==999996&&atom.segment_timestamp==999997&&atom.listener_age_s==.001);
    check('fresh_callback_produces_mode1_guard',~isempty(guard.guard)&&guard.guard.valid&& ...
        guard.guard.status_generation==1&&guard.guard.rx_ns==atom.rx_ns);
    raw.send_callback_order=struct('send_return_receipt_ns',tx.owner_send_receipt.snapshot.now_ns, ...
        'poll_return_ns',pollReturnNs,'first_callback_rx_ns',atom.first_chunk_rx_ns, ...
        'last_callback_rx_ns',atom.rx_ns, ...
        'callback_before_send_receipt_observed',atom.first_chunk_rx_ns<tx.owner_send_receipt.snapshot.now_ns, ...
        'callback_before_poll_return_observed',atom.first_chunk_rx_ns<pollReturnNs);
    check('immediate_reply_admitted_only_after_send_return', ...
        isempty(raw.first.fatal)&&raw.first.canonical_module_observer_state.tx_confirmed==false&& ...
        (atom.first_chunk_rx_ns>=tx.owner_send_receipt.snapshot.now_ns|| ...
        raw.first.canonical_module_deferred_receive_processed>0));
    g1=io.canonicalModuleGuard();g2=io.canonicalModuleGuard();
    check('guard_reads_keep_source_generation_and_rx',g1.snapshot.status_generation==1&& ...
        g2.snapshot.status_generation==1&&g1.snapshot.latest.rx_ns==atom.rx_ns&&g2.snapshot.latest.rx_ns==atom.rx_ns);
    ev=io.evidence();serialRows=ev.raw_mavlink(cellfun(@(r)strcmp(r.topic,'SERIAL_CONTROL'),ev.raw_mavlink));
    check('raw_receive_metadata_matches_final_atom',round(serialRows{end}.rx_s*1e9)==atom.rx_ns&& ...
        serialRows{end}.message.SystemID==231&&serialRows{end}.message.ComponentID==77&& ...
        serialRows{end}.message.Seq==atom.final_wire_sequence);
    % A second solicited response repeats the board timestamp/payload. Only
    % response accounting advances, not original receive time or generation.
    awaitInterval();tx=io.pollCanonicalModuleStatus();raw.duplicate=waitComplete(2);collectQuery();
    d=raw.duplicate.canonical_module_observer_state;
    check('duplicate_status_reply_no_new_generation_or_rx',tx.query_prepared&&d.status_generation==1&& ...
        d.latest.rx_ns==atom.rx_ns&&d.duplicate_status_responses==1&&d.latest_response.rx_ns>atom.rx_ns);
    pause(.11);stale=io.canonicalModuleGuard();
    check('expired_guard_empty_no_receive_time_substitution',isempty(stale.guard)&& ...
        strcmp(stale.guard_reason,'STATUS_GUARD_STALE')&&stale.snapshot.latest.rx_ns==atom.rx_ns);
    io.pollCanonicalModuleStatus();raw.next=waitComplete(3);collectQuery();a2=raw.next.canonical_module_observer_state.latest;
    check('new_board_timestamp_advances_generation_once',a2.status_generation==2&&a2.timestamp==1200000&& ...
        a2.rx_ns>atom.rx_ns);
    before=io.evidence();wrongPackets=encodeText(fixture(1300000,13,12),wrong);
    for k=1:numel(wrongPackets),write(peer,wrongPackets{k},'uint8','127.0.0.1',cfg.local_mavlink_port);end
    pause(.02);after=io.evidence();
    check('wrong_source_filtered_by_real_client_not_relabelled',numel(after.raw_mavlink)==numel(before.raw_mavlink)&& ...
        after.canonical_module_observer_state.status_generation==2&&isempty(after.fatal));
    awaitInterval();io.pollCanonicalModuleStatus();raw.invalid=waitFatal();collectQuery();
    check('actual_unknown_status_reply_latches_adapter_fatal', ...
        strcmp(raw.invalid.canonical_module_observer_state.first_failure.reason,'STATUS_UNKNOWN_FIELD')&& ...
        contains(raw.invalid.fatal,'CANONICAL_MODULE:STATUS_UNKNOWN_FIELD'));
    rejected=rejects(@()io.pollCanonicalModuleStatus(),'m600check:CanonicalModuleTransportClosed');
    check('fatal_prohibits_additional_query',rejected&&numel(queryRows)==4);
    check('only_listener_messages_no_authority_transmit', ...
        all(cellfun(@(r)queryIsFixed(r.message),raw.invalid.raw_transmit_messages))&& ...
        isempty(raw.invalid.raw_environment_transmit_datagrams)&&isempty(raw.invalid.real_parameter_actions));
    check('enabled_adapter_closes',io.close());io=[];
    disabledCfg=rmfield(cfg,'canonical_se3_status_observation');
    io=m600check.makeM600CopterSimIo(disabledCfg);sendHeartbeat();pause(.02);disabled=io.evidence();
    check('default_path_no_serial_subscription_or_queries',~disabled.canonical_module_observer_enabled&& ...
        ~any(strcmp(disabled.subscribed_topics,'SERIAL_CONTROL'))&&isempty(disabled.raw_transmit_messages));
    check('default_path_rejects_poll_and_guard', ...
        rejects(@()io.pollCanonicalModuleStatus(),'m600check:CanonicalModuleNotConfigured')&& ...
        rejects(@()io.canonicalModuleGuard(),'m600check:CanonicalModuleNotConfigured'));
    check('disabled_adapter_closes',io.close());io=[];
    io=m600check.makeM600CopterSimIo(cfg);sendHeartbeat();pause(.02);
    io.pollCanonicalModuleStatus();pause(.11);raw.timeout=io.canonicalModuleGuard();collectQuery();
    check('actual_no_response_timeout_is_fail_closed',raw.timeout.fatal_latched&& ...
        strcmp(raw.timeout.snapshot.first_failure.reason,'STATUS_QUERY_TIMEOUT')&&isempty(raw.timeout.guard));
    check('timeout_no_automatic_query_restart',rejects(@()io.pollCanonicalModuleStatus(), ...
        'm600check:CanonicalModuleTransportClosed')&&raw.timeout.snapshot.queries_sent==1);
    check('peer_callback_no_exception',isempty(peerErrors));
    check('source_files_unchanged_during_suite',strcmp(adapterSha,m600check.fileSha256(adapterPath))&& ...
        strcmp(helperSha,m600check.fileSha256(which('gpenmpcNative.advanceSe3StatusObservation'))));
catch ex
    failure=getReport(ex,'extended','hyperlinks','off');
    if ~isempty(io),try,raw.failure_evidence=io.evidence();catch,end,end
end
finish();clear cg
check('all_endpoints_closed_and_high_ports_rebound',cleanup.all_closed&&cleanup.ports_rebound);
report=struct('schema','CANONICAL_SE3_REAL_ADAPTER_LOCALHOST_V1','pass',isempty(failure)&&all([checks.pass]), ...
    'checks_total',numel(checks),'checks_passed',nnz([checks.pass]),'checks',checks,'failure',failure, ...
    'cleanup',cleanup,'peer_callback_errors',{peerErrors},'peer_stderr',shellPeerStderr, ...
    'peer_implementation','BOUNDED_DOTNET_LOCALHOST_RESPONDER_FIXED_FIVE_QUERY_PLAN', ...
    'observed_query_count',numel(queryRows), ...
    'adapter_sha256',adapterSha,'helper_sha256',helperSha, ...
    'test_sha256',m600check.fileSha256(mfilename('fullpath')+".m"),'localhost_ports',ports, ...
    'COM_open',0,'board_actions',0,'plant_runs',0,'model_runs',0,'parameter_actions',0,'arm_actions',0, ...
    'claim','Adapter localhost callback tests.');
save(fullfile(outputRoot,'RAW.mat'),'raw','report','queryRows');
fid=fopen(fullfile(outputRoot,'RESULT.json'),'w','n','UTF-8');assert(fid>=0);
fg=onCleanup(@()fclose(fid));fprintf(fid,'%s\n',jsonencode(report,PrettyPrint=true));clear fg
fprintf('CANONICAL_SE3_ADAPTER %d/%d pass=%d\n',report.checks_passed,report.checks_total,report.pass);
if ~isempty(failure),fprintf('%s\n',failure);end
assert(report.pass,'m600check:Se3AdapterLoopbackTest','See RESULT.json.');
    function check(name,yes)
        checks(end+1)=struct('name',name,'pass',logical(yes)); %#ok<AGROW>
        if ~yes,error('m600check:Se3AdapterCheck','Failed: %s',name);end
    end
    function collectQuery()
        readTask=shellPeer.StandardOutput.ReadLineAsync();assert(readTask.Wait(1000));
        line=char(readTask.Result);assert(startsWith(line,'RX:'));
        bytes=matlab.net.base64decode(line(4:end));m=deserializemsg(dialect,bytes);
        assert(isscalar(m)&&queryIsFixed(m),'Unexpected non-listener transmit.');
        queryRows{end+1}=struct('message',m,'wire_bytes',bytes); %#ok<AGROW>
    end
    function sendHeartbeat()
        m=createmsg(dialect,'HEARTBEAT');m.Payload.type=uint8(13);m.Payload.autopilot=uint8(12);
        m.Payload.base_mode=uint8(0);m.Payload.system_status=uint8(3);m.Payload.mavlink_version=uint8(3);
        write(peer,serializemsg(encoder,m),'uint8','127.0.0.1',cfg.local_mavlink_port);
    end
    function e=waitComplete(count)
        clock=tic;
        while toc(clock)<2
            pause(.001);e=io.evidence();s=e.canonical_module_observer_state;
            if s.fatal_latched,error('m600check:Se3RealCallbackFatal','%s',s.first_failure.reason);end
            if s.responses_completed==count,return,end
        end
        error('m600check:Se3RealCallbackTimeout','Response not completed; peer errors: %s',strjoin(peerErrors,' | '));
    end
    function e=waitFatal()
        clock=tic;
        while toc(clock)<2
            pause(.001);e=io.evidence();if e.canonical_module_observer_state.fatal_latched,return,end
        end
        error('m600check:Se3ExpectedFatalMissing','Expected invalid reply was not processed.');
    end
    function awaitInterval()
        e=io.evidence();remaining=(e.canonical_module_observer_state.next_query_ns-round(io.now()*1e9))/1e9;
        if remaining>0,pause(remaining+.001);end
    end
    function finish()
        if ~isempty(cleanup),return,end
        okay=true;errors={};
        if ~isempty(io),try,assert(io.close());catch ex,okay=false;errors{end+1}=ex.message;end;io=[];end
        if ~isempty(peer),try,configureCallback(peer,'off');delete(peer);catch ex,okay=false;errors{end+1}=ex.message;end;peer=[];end
        if ~isempty(shellPeer)
            try
                natural=shellPeer.WaitForExit(2000);
                if ~natural,shellPeer.Kill();assert(shellPeer.WaitForExit(2000));end
                shellPeerStderr=char(shellPeer.StandardError.ReadToEnd());
                if numel(queryRows)==5,assert(natural&&shellPeer.ExitCode==0);end
                shellPeer.Dispose();
            catch ex,okay=false;errors{end+1}=ex.message;end
            shellPeer=[];
        end
        for e={encoder,wrong}
            if ~isempty(e{1}),try,delete(e{1});catch ex,okay=false;errors{end+1}=ex.message;end,end
        end
        encoder=[];wrong=[];bound=true;
        for p=ports
            try,u=udpport('datagram','IPV4','LocalHost','127.0.0.1','LocalPort',p);delete(u);
            catch ex,bound=false;errors{end+1}=ex.message;end
        end
        cleanup=struct('all_closed',okay,'ports_rebound',bound,'errors',{errors});
    end
end
function c=configuration()
c=struct('local_mavlink_port',48231,'remote_mavlink_port',48232,'truth_port',48233, ...
    'coptersim_time_port',48234,'target_system',231,'target_component',77, ...
    'clock_max_rtt_s',2,'clock_sync_samples',3,'clock_sync_period_s',.03,'clock_max_age_s',30, ...
    'clock_max_uncertainty_s',1.1,'clock_max_utc_drift_s',.1,'clock_max_time_heartbeat_age_s',2, ...
    'clock_max_time_heartbeat_lag_s',.25,'maximum_truth_lag_s',2,'maximum_raw_records',2000, ...
    'state_max_age_s',.5,'heartbeat_max_age_s',.3,'landed_max_age_s',.3, ...
    'live_enabled',true,'outer_preflight_pass',true,'command_timeout_s',2,'poll_period_s',.01, ...
    'require_terrain_diagnostic_extension',false);
c.canonical_se3_status_observation=struct('verified_binding',struct( ...
    'uid','1234605616436508552','system_id',231,'component_id',77,'boot_generation',7,'verified',true));
end
function yes=queryIsFixed(m)
yes=double(m.MsgID)==126;if ~yes,return,end
p=m.Payload;yes=double(p.device)==10&&double(p.flags)==6&&double(p.timeout)==0&&double(p.baudrate)==0&& ...
    strcmp(char(p.data(1:double(p.count))),[newline,'listener gpenmpc_se3_control_status 0 1',newline]);
end
function packets=encodeText(text,encoder)
bytes=uint8(text);packets={};
for k=1:70:numel(bytes)
    b=bytes(k:min(end,k+69));m=createmsg(encoder.Dialect,'SERIAL_CONTROL');
    m.Payload.device=uint8(10);m.Payload.flags=uint8(1);m.Payload.timeout=uint16(0);m.Payload.baudrate=uint32(0);
    m.Payload.count=uint8(numel(b));m.Payload.data=zeros(1,70,'uint8');m.Payload.data(1:numel(b))=b;
    packets{end+1}=serializemsg(encoder,m); %#ok<AGROW>
end
end
function b=packet(d,j)
if istable(d),if iscell(d.Data),b=d.Data{j};else,b=d.Data(j,:);end;else,b=d(j).Data;end
b=reshape(uint8(b),1,[]);
end
function yes=rejects(f,id)
yes=false;try,f();catch ex,yes=strcmp(ex.identifier,id);end
end
function text=fixture(timestamp,inputCount,outputCount)
lines={sprintf('nsh> listener gpenmpc_se3_control_status 0 1\nTOPIC: gpenmpc_se3_control_status\n gpenmpc_se3_control_status'), ...
    sprintf('    timestamp: %.0f (0.001 seconds ago)',timestamp), ...
    sprintf('    sample_timestamp: %.0f',timestamp-5),sprintf('    reference_timestamp: %.0f',timestamp-4), ...
    sprintf('    segment_timestamp: %.0f',timestamp-3),sprintf('    input_sample_count: %.0f',inputCount), ...
    sprintf('    output_publish_count: %.0f',outputCount), ...
    '    rejected_sample_count: 0','    reset_count: 1','    dt_s: 0.004','    reference_age_s: 0.002', ...
    '    segment_age_s: 0.003','    hover_thrust: 0.5','    total_mass_kg: 12.9', ...
    '    yaw_setpoint_rad: 0','    control_mode: 1','    failure_reason: 0'};
for f={'active','reference_fresh','segment_fresh','state_valid','hover_thrust_fresh', ...
        'native_position_controller_disabled','native_attitude_rate_allocator_enabled','single_publisher_contract_pass'}
    lines{end+1}=['    ',f{1},': True']; %#ok<AGROW>
end
for f={'robust_acceleration_ned_mps2','normalized_thrust_ned','position_ned_m','velocity_ned_mps', ...
        'reference_position_ned_m','reference_velocity_ned_mps','reference_acceleration_ned_mps2', ...
        'nominal_feedback_acceleration_ned_mps2','drag_feedforward_acceleration_ned_mps2'}
    lines{end+1}=['    ',f{1},': [0, 0, 0]']; %#ok<AGROW>
end
lines{end+1}='    commanded_acceleration_ned_mps2: [0.1, 0.2, -0.3]';lines{end+1}='nsh>';
text=[strjoin(lines,newline),newline];
end
