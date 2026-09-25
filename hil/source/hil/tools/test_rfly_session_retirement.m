function report=test_rfly_session_retirement(outputRoot)
% Test session retirement over UDP with synthetic peer registration and stop reports.
arguments,outputRoot (1,1) string,end
build=string(fileparts(fileparts(mfilename('fullpath'))));
addpath(fullfile(build,'host_runtime'),fullfile(build,'matlab_validation'), ...
    fullfile(build,'m600_coptersim','matlab_validation'));
assert(~isfolder(outputRoot));mkdir(outputRoot);
sourcePaths={which('m600check.makeM600CopterSimIo'),which('gpenmpcNative.RflySessionReleaseDecoder'), ...
    which('gpenmpcNative.RflySessionAssociationDecoder'),which('m600check.decodeCanonicalControlCache'), ...
    which('m600check.inspectCanonicalControlCachePacket'),[mfilename('fullpath') '.m']};
sourceHashes=cellfun(@(p)hex(digest(readbin(p))),sourcePaths,'UniformOutput',false);
checks=struct('name',{},'pass',{});negative=struct('name',{},'identifier',{});
cfg=struct('local_mavlink_port',62421,'remote_mavlink_port',62422,'truth_port',62423,'coptersim_time_port',62424, ...
    'target_system',1,'target_component',1,'clock_max_rtt_s',1,'clock_sync_samples',2,'clock_sync_period_s',1, ...
    'clock_max_age_s',5,'clock_max_uncertainty_s',1,'clock_max_utc_drift_s',1,'clock_max_time_heartbeat_age_s',5, ...
    'clock_max_time_heartbeat_lag_s',5,'maximum_truth_lag_s',5,'maximum_raw_records',2000,'state_max_age_s',1, ...
    'live_enabled',true,'outer_preflight_pass',true,'canonical_exchange', ...
    struct('assembly_limit_ns',uint64(2000000000),'completed_queue_capacity',4));
health=fullfile(build,'m600_coptersim','matlab_validation','+m600check','px4_health_events.xml');
d=mavlinkdialect(health,2);serializer=mavlinkio(d,'SystemID',1,'ComponentID',1);
serializerCleanup=onCleanup(@()delete(serializer)); %#ok<NASGU>
peer=udpport('datagram','IPV4','LocalPort',cfg.remote_mavlink_port,'Timeout',.2);
peerCleanup=onCleanup(@()delete(peer)); %#ok<NASGU>
base=fullfile(gpenmpc_external_path('board_commit_exchange_fixture'));
echoBytes=readbin(fullfile(base,'ACTUAL_FIXTURE_ECHO119.bin'));
sources={readbin(fullfile(base,'SOURCE_1.bin')),readbin(fullfile(base,'SOURCE_2.bin')),readbin(fullfile(base,'SOURCE_3.bin'))};
authorization=struct('source','OperatorUsbIsolationDeclaration', ...
    'physical_setup_record_sha256',repmat('A',1,64),'original_record', ...
    struct('scope','HOST_SESSION_RETIREMENT_FIXTURE'));
io=m600check.makeM600CopterSimIo(cfg);ioCleanup=onCleanup(@()io.close()); %#ok<NASGU>
[a,expected]=register(uint64(0));io.bindCanonicalSession(expected,a);
% Exact reproduction: scalar linear deletion changes a column cell to 1x0.
badColumn={1};badColumn(1)=[];badColumn{end+1,1}=2;
check('reproduced_old_linear_delete_empty_row',numel(badColumn)==2&&isempty(badColumn{1}));
for sourceIndex=1:3
    sendSource(sources{sourceIndex});waitTunnel(3*sourceIndex);
    taken=io.takeCanonical('snapshot');st=io.pollCanonical();
    check(sprintf('sequential_singleton_pop_%d',sourceIndex),isequal(taken.message,sources{sourceIndex}) ...
        &&isequal(st.completed_enqueued,uint64([sourceIndex 0])) ...
        &&isequal(st.completed_taken,uint64([sourceIndex 0]))&&all(st.completed_queue_counts==0));
end
[records,request,text]=releaseRecords(a,uint64(0));
parsed=gpenmpcNative.RflySessionReleaseDecoder(records,request,d);
check('release_original_bytes_and_timestamps',isequal(parsed.original_shell_bytes,uint8(text).') ...
    &&isequal(parsed.original_records,records)&&~any(parsed.raw_frame_available) ...
    &&parsed.first_original_host_receive_ns>=request.original_release_submit_ns);
check('release_report_not_cache_or_authority',parsed.board_context_detached_reported ...
    &&~parsed.requires_independent_plant_cache_zero&&~parsed.plant_cache_zero_proven ...
    &&~parsed.publication_authority&&~parsed.transport_source_authenticated);
variants={ ...
    'pending_not_detached',strrep(text,'RFLY_RELEASE result=4','RFLY_RELEASE result=3'); ...
    'context_not_observed',strrep(text,'context_observed_after_stop=1','context_observed_after_stop=0'); ...
    'not_disarmed',strrep(text,'disarmed=1','disarmed=0'); ...
    'fake_cache_proof',strrep(text,'plant_cache_zero_proven=0','plant_cache_zero_proven=1'); ...
    'board_session_fault',strrep(text,'session_fault=0','session_fault=1'); ...
    'stale_session_generation',regexprep(text,'session_generation=[0-9]+','session_generation=9999'); ...
    'duplicate_release',[text text]; ...
    'truncated_release',text(1:end-1); ...
    'missing_counter',strrep(text,' publication_attempts=0',''); ...
    'overflow_counter',strrep(text,'publication_attempts=0','publication_attempts=18446744073709551616')};
for variant=1:size(variants,1)
    altered=syntheticRecords(variants{variant,2},records(1).original_host_receive_ns);
    reject(@()gpenmpcNative.RflySessionReleaseDecoder(altered,request,d),'gpenmpcNative:Release',variants{variant,1});
end
altered=records;altered(1).original_host_receive_ns=request.original_release_submit_ns-uint64(1);
reject(@()gpenmpcNative.RflySessionReleaseDecoder(altered,request,d),'gpenmpcNative:ReleaseReceiveOrder','before_original_release');
altered=records;altered(1).decoded_message.SystemID=uint8(2);
reject(@()gpenmpcNative.RflySessionReleaseDecoder(altered,request,d),'gpenmpcNative:ReleaseCallback','wrong_original_origin');
retired=io.retireCanonicalSession(records,request);
check('same_io_retired_only_no_own_publication',retired.retired ...
    &&strcmp(retired.status,'RETIRED_NO_OWN_CONTROL_PUBLICATION')&&~io.pollCanonical().bound);
old=io.evidence();check('history_keeps_closed_assembler_counters_raw',numel(old.canonical_session_history)==1 ...
    &&old.canonical_session_history{1}.closed_assembler_evidence.status.closed ...
    &&isequal(old.canonical_session_history{1}.completed_taken,uint64([3 0])) ...
    &&~isempty(old.raw_mavlink)&&isempty(old.canonical_exchange_failure));
[a2,expected2]=register(uint64(1));b2=io.bindCanonicalSession(expected2,a2);
check('actual_same_io_new_session_generation',b2.bound&&b2.retired_session_count==1 ...
    &&a2.echo.process_session_generation>a.echo.process_session_generation ...
    &&a2.echo.link_lifecycle_generation==a.echo.link_lifecycle_generation);
sendSource(sources{1});waitTunnel(12);failed=io.evidence();
check('late_old_session_body_rejected_and_retained',contains(failed.canonical_exchange_failure,'TunnelBodyBinding') ...
    &&failed.canonical_exchange.status.failed&&numel(failed.canonical_session_history)==1);
reject(@()io.retireCanonicalSession(records,request),'m600check:CanonicalExchangeClosed','fault_cannot_retire_and_resume');
stop=io.sendCanonicalSession('stop',struct());drainPeer(stop.messages_send_returned);
check('fault_cleanup_send_preserves_first_fault',stop.messages_send_returned==1 ...
    &&strcmp(io.evidence().canonical_exchange_failure,failed.canonical_exchange_failure));
check('first_owner_closed',io.close());clear ioCleanup
% A separate sequential test owner proves a published Detached report is
% insufficient while the real existing model packet has no cache field.
io=m600check.makeM600CopterSimIo(cfg);ioCleanup=onCleanup(@()io.close()); %#ok<NASGU>
[publishedAssociation,publishedExpected]=register(uint64(0));io.bindCanonicalSession(publishedExpected,publishedAssociation);
[publishedRecords,publishedRequest]=releaseRecords(publishedAssociation,uint64(1));
unavailable=io.retireCanonicalSession(publishedRecords,publishedRequest);
check('published_requires_independent_cache_not_release_bool',~unavailable.retired ...
    &&strcmp(unavailable.status,'PLANT_CACHE_ZERO_UNOBSERVABLE')&&io.pollCanonical().bound ...
    &&~unavailable.plant_cache_zero_proven&&isempty(io.evidence().canonical_session_history));
retainedPublished=io.evidence();check('unavailable_keeps_release_and_model_evidence', ...
    numel(retainedPublished.canonical_retirement_attempts)==1 ...
    &&isequal(retainedPublished.canonical_retirement_attempts{1}.release.original_records,publishedRecords));
check('second_owner_closed',io.close());clear ioCleanup
% Reject reuse of a retired identity on the same owner.
io=m600check.makeM600CopterSimIo(cfg);ioCleanup=onCleanup(@()io.close()); %#ok<NASGU>
[a3,e3]=register(uint64(0));io.bindCanonicalSession(e3,a3);
[r3,q3]=releaseRecords(a3,uint64(0));io.retireCanonicalSession(r3,q3);
reject(@()io.bindCanonicalSession(e3,a3),'m600check:CanonicalSessionReplay','same_old_challenge_cannot_resume');
check('replay_fault_history_survives',numel(io.evidence().canonical_session_history)==1 ...
    &&contains(io.evidence().canonical_exchange_failure,'CanonicalSessionReplay'));
check('third_owner_closed',io.close());clear ioCleanup
% Replay recorded paired DLL cache and rotor packets over localhost.
% The peer's model/UTC/board telemetry is explicitly simulated, and enables
% exercising the real same-IO conservative post-release mapping predicate.
cacheBase=fullfile(gpenmpc_external_path('control_cache_dll'));
cachePackets=reshape(readbin(fullfile(cacheBase,'CACHE216.bin')),216,[]);
rotorPackets=reshape(readbin(fullfile(cacheBase,'ROTOR128.bin')),128,[]);
cfg.canonical_rotor_observer=struct('local_port',62425,'maximum_queue',4);
cfg.canonical_control_cache_observer=struct('session_token',uint64(26090501), ...
    'dll_sha256',uint8(sscanf('D155F1E4A1824B4D10EAB0860FC8946AF616874421FE1F9FB1F1BE05A54285BB','%2x')), ...
    'maximum_source_age_s',5,'maximum_receive_age_s',5);
cfg.heartbeat_max_age_s=5;cfg.landed_max_age_s=5;cfg.clock_sync_period_s=.01;
io=m600check.makeM600CopterSimIo(cfg);ioCleanup=onCleanup(@()io.close()); %#ok<NASGU>
[cacheAssociation,cacheExpectedSession]=register(uint64(0));io.bindCanonicalSession(cacheExpectedSession,cacheAssociation);
for generation=1:6
    input=sendObserverPair(generation);
    check(sprintf('same_socket_raw_rotor_cache_demux_%d',generation),numel(input.records)==1&&numel(input.cache_records)==1 ...
        &&isequal(input.records{1}.bytes(:),rotorPackets(:,generation)) ...
        &&isequal(input.cache_records{1}.bytes(:),cachePackets(:,generation)) ...
        &&~input.cache_observation.valid&&input.received_count==uint64(generation) ...
        &&input.cache_received_count==uint64(generation));
end
[cacheReleaseRecords,cacheReleaseRequest]=releaseRecords(cacheAssociation,uint64(1));
cacheBefore=io.retireCanonicalSession(cacheReleaseRecords,cacheReleaseRequest);
check('no_clock_no_post_stop_cache_credit',~cacheBefore.retired&&io.pollCanonical().bound);
cacheBeforeEvidence=io.evidence();
releaseRows=cacheBeforeEvidence.raw_mavlink(cellfun(@(x)strcmp(x.topic,'SERIAL_CONTROL') ...
    &&x.original_host_receive_ns==cacheReleaseRecords(end).original_host_receive_ns,cacheBeforeEvidence.raw_mavlink));
assert(numel(releaseRows)==1);
% Choose this replay's one fixed model epoch so gen7's original source is at
% or before release. A later packet reception alone must not grant retirement.
modelStartMs=int64(floor((cacheBeforeEvidence.utc_zero_s+releaseRows{1}.rx_s-.07)*1000));
pause(.10);
publishTimeAndGround(modelStartMs,.06,1);pause(.01);io.snapshot();answerTimesync();
publishTimeAndGround(modelStartMs,.07,2);pause(.02);io.snapshot();answerTimesync();
pause(.02);mapped=io.snapshot();answerTimesync();
check('actual_io_original_clock_receipts_bound',mapped.clock_valid&&mapped.model_ready&&mapped.armed==0&&mapped.landed_state==1);
cacheZeroIngress=sendObserverPair(7);
check('actual_zero_step_not_lag_state',cacheZeroIngress.cache_observation.valid ...
    &&cacheZeroIngress.cache_observation.all16_zero&&cacheZeroIngress.cache_observation.generation==7 ...
    &&cacheZeroIngress.cache_observation.input_call_count==71);
beforeBound=io.retireCanonicalSession(cacheReleaseRecords,cacheReleaseRequest);
check('late_rx_of_old_zero_is_not_post_release_step',~beforeBound.retired ...
    &&strcmp(beforeBound.status,'PLANT_CACHE_NOT_PROVEN_AFTER_RELEASE')&&io.pollCanonical().bound);
for generation=8:10
    publishTimeAndGround(modelStartMs,generation*.01,generation);
    pause(.005);sendObserverPair(generation);
end
cacheRetired=io.retireCanonicalSession(cacheReleaseRecords,cacheReleaseRequest);
check('published_same_io_retirement_requires_actual_cache_mapping',cacheRetired.retired&&cacheRetired.plant_cache_zero_proven ...
    &&strcmp(cacheRetired.status,'RETIRED_PUBLISHED_WITH_POST_RELEASE_ACCEPTED_CACHE_ZERO') ...
    &&cacheRetired.cache_evidence.source_io_time_lower_bound_s>cacheRetired.cache_evidence.original_release_receipt_io_s ...
    &&~cacheRetired.cache_evidence.runtime_origin_attested);
cacheIoEvidence=io.evidence();check('cache_all_original_bytes_retained_separate',numel(cacheIoEvidence.raw_cache_datagrams)==10 ...
    &&numel(cacheIoEvidence.raw_rotor_datagrams)==10&&numel(cacheIoEvidence.canonical_session_history)==1);
check('fourth_owner_closed',io.close());clear ioCleanup
cfg=rmfield(cfg,{'canonical_rotor_observer','canonical_control_cache_observer'});
io=m600check.makeM600CopterSimIo(cfg);ioCleanup=onCleanup(@()io.close()); %#ok<NASGU>
[queuedAssociation,queuedExpected]=register(uint64(0));io.bindCanonicalSession(queuedExpected,queuedAssociation);
sendSource(sources{1});waitTunnel(3);
[queuedRecords,queuedRequest]=releaseRecords(queuedAssociation,uint64(0));
reject(@()io.retireCanonicalSession(queuedRecords,queuedRequest),'m600check:CanonicalSessionNotDrained', ...
    'retirement_cannot_drop_original_complete_fifo');
queuedRaw=io.evidence();check('failed_retirement_retains_queued_source',numel(queuedRaw.canonical_completed_queues{1})==1 ...
    &&isequal(queuedRaw.canonical_completed_queues{1}{1}.message,sources{1}) ...
    &&isempty(queuedRaw.canonical_session_history));
check('fifth_owner_closed',io.close());clear ioCleanup
% Test both host lifetimes before sending using the complete seven-frame input fixture.
io=m600check.makeM600CopterSimIo(cfg);ioCleanup=onCleanup(@()io.close()); %#ok<NASGU>
[sendAssociation,sendExpected]=register(uint64(0));io.bindCanonicalSession(sendExpected,sendAssociation);
now=gpenmpcNative.rflyOriginalHostMonotonicNs();
originalContext=struct('reference_creation_ns',now-uint64(10000000),'outer_creation_ns',now-uint64(10000000), ...
    'reference_expiry_ns',now+uint64(1000000000),'outer_expiry_ns',now+uint64(1000000000));
sourceLifetime=struct('source_host_receive_ns',now-uint64(200000000), ...
    'maximum_runtime_age_ns',uint64(100000000),'source_valid_until_ns',now-uint64(100000000));
contextBody=readbin(fullfile(base,'CONTEXT316.bin'));numericBody=readbin(fullfile(base,'COMMAND381.bin'));
inputSerializer=mavlinkio(d,'SystemID',255,'ComponentID',190);inputCleanup=onCleanup(@()delete(inputSerializer)); %#ok<NASGU>
inputPackets=[packInput(contextBody,4,decode(contextBody(5:12)));packInput(numericBody,2,decode(numericBody(1:8)))];
beforeSend=io.evidence();
reject(@()io.sendCanonicalPackets(inputPackets,originalContext,sourceLifetime), ...
    'm600check:CanonicalSendOriginalSourceExpired','source_original_100ms_before_actual_send');
afterSend=io.evidence();check('expired_source_no_control_attempt_prefix_fabricated', ...
    numel(afterSend.raw_transmit_messages)==numel(beforeSend.raw_transmit_messages)&&afterSend.canonical_exchange_messages_sent==0);
check('sixth_owner_closed',io.close());clear ioCleanup
check('actual_sources_unchanged',isequal(sourceHashes,cellfun(@(p)hex(digest(readbin(p))),sourcePaths,'UniformOutput',false)));
report=struct('passed',all([checks.pass]),'checks',checks,'negative_cases',negative, ...
    'scope','ACTUAL_SAME_MAVLINKIO_LOOPBACK_WITH_EXPLICIT_PEER_SESSION_REPORT_FIXTURE', ...
    'live_registration_proven',false,'completed_five_leg_retirement_proven',false, ...
    'board_actions',0,'com_opens',0,'model_runs',0,'maximum_simultaneous_production_io_owners',1, ...
    'cache_actual_dll_bytes',true,'cache_board_and_clock_telemetry_fixture',true, ...
    'cache_fixture_age_bounds_s',5,'cache_source',cacheBase,'source_paths',{sourcePaths},'source_sha256',{sourceHashes});
save(fullfile(outputRoot,'RAW.mat'),'report','a','a2','expected','expected2','parsed','retired','old','failed', ...
    'publishedRecords','publishedRequest','unavailable','retainedPublished','cacheReleaseRecords', ...
    'cacheReleaseRequest','cacheRetired','cacheZeroIngress','cacheIoEvidence','mapped','beforeBound','queuedRaw','beforeSend','afterSend');
f=fopen(fullfile(outputRoot,'RESULT.json'),'w');assert(f>0);c=onCleanup(@()fclose(f));
fprintf(f,'%s\n',jsonencode(report,PrettyPrint=true));clear c
disp(jsonencode(struct('passed',report.passed,'checks',numel(checks))));assert(report.passed);

    function [association,expected]=register(delta)
        echo=echoBytes;challenge=decode(echo(6:21));challenge(2)=challenge(2)+delta;
        echo(6:21)=be(challenge);echo(32:39)=be(decode(echo(32:39))+delta);
        echo(40:47)=be(decode(echo(40:47))+delta);echo(88:119)=digest(echo(1:87));
        req=struct('original_host_challenge',challenge,'uid',decode(echo(22:29)), ...
            'system',echo(30),'component',echo(31),'host_system',uint8(255),'host_component',uint8(190), ...
            'configuration_payload_sha256',hex(echo(56:87)), ...
            'approved_parameter_sha256','CBF017C734B59715B563A61BCB64D8B142A740007C8D610BB6D4BE54808356A8');
        p=struct('challenge',challenge,'origin_ned_m',[0;0;0],'host_system',uint8(255),'host_component',uint8(190));
        tx=io.sendCanonicalSession('prepare',p);drainPeer(tx.messages_send_returned);
        req.original_prepare_submit_ns=tx.original_host_submit_ns(1);
        lineText=sprintf(['RFLY_SESSION state=1 challenge=%s uid=%u system=%u component=%u registration_hrt_us=%u ' ...
            'session_generation=%u link_generation=%u semantics=1 config_sha=%s session_sha=%s parameter_sha=%s ' ...
            'registered=1 echo_confirmed=0 declared_isolation=0 declaration_is_sensor_proof=0 session_fault=0 start_requests=0 stop_requests=0\n'], ...
            hex(echo(6:21)),req.uid,req.system,req.component,decode(echo(32:39)),decode(echo(40:47)), ...
            decode(echo(48:55)),req.configuration_payload_sha256,hex(echo(88:119)),req.approved_parameter_sha256);
        pre=sendText(lineText);
        prepared=gpenmpcNative.RflySessionAssociationDecoder(pre,[],req,[],d);
        tx=io.sendCanonicalSession('confirm',struct('prepared_receipt',prepared, ...
            'physical_setup_record_sha256',authorization.physical_setup_record_sha256));drainPeer(tx.messages_send_returned);
        req.original_confirm_submit_ns=tx.original_host_submit_ns(1);
        req.original_confirm_session_sha256=prepared.execution_session_sha256;
        req.confirmed_physical_setup_record_sha256=authorization.physical_setup_record_sha256;
        confirmed=strrep(strrep(strrep(lineText,'state=1','state=2'),'echo_confirmed=0','echo_confirmed=1'),'declared_isolation=0','declared_isolation=1');
        con=sendText(confirmed);association=gpenmpcNative.RflySessionAssociationDecoder(pre,con,req,authorization,d);
        e=association.echo;expected=struct('source_system',e.system,'source_component',e.component, ...
            'target_system',uint8(255),'target_component',uint8(190),'uid',e.uid,'session_generation',e.process_session_generation, ...
            'link_lifecycle_generation',e.link_lifecycle_generation,'confirmed_host_rx_ns',association.original_host_receive_ns, ...
            'execution_session_sha256',association.execution_session_sha256,'configuration_sha256',e.configuration_payload_sha256);
    end
    function [records,request,text]=releaseRecords(association,published)
        stopTx=io.sendCanonicalSession('stop',struct());drainPeer(stopTx.messages_send_returned);
        releaseTx=io.sendCanonicalSession('release',struct());drainPeer(releaseTx.messages_send_returned);
        request=struct('registered_association',association,'original_stop_submit_ns',stopTx.original_host_submit_ns(1), ...
            'original_release_submit_ns',releaseTx.original_host_submit_ns(1));
        s=char(association.confirm_receipt.original_session_line_bytes.');
        s=strrep(strrep(strrep(s,'state=2','state=4'),'start_requests=0','start_requests=1'),'stop_requests=0','stop_requests=1');
        text=[s sprintf(['RFLY_RELEASE result=4 context_observed_after_stop=1 disarmed=1 virtual_zero_stream_accepted=%u ' ...
            'plant_cache_zero_proven=0 publication_attempts=%u publication_successes=%u raw_evidence_available=0\n'],published>0,published,published)];
        records=sendText(text);
    end
    function records=sendText(text)
        n=ceil(numel(text)/70);before=numel(io.canonicalSessionReceipts());
        for fragment=1:n
            m=createmsg(d,'SERIAL_CONTROL');m.Payload.device=uint8(10);m.Payload.flags=uint8(1);
            part=uint8(text((fragment-1)*70+1:min(fragment*70,numel(text))));
            m.Payload.count=uint8(numel(part));m.Payload.data(:)=0;m.Payload.data(1:numel(part))=part;
            write(peer,uint8(serializemsg(serializer,m)),'uint8','127.0.0.1',cfg.local_mavlink_port);
        end
        started=tic;while numel(io.canonicalSessionReceipts())<before+n&&toc(started)<3,pause(.005);end
        allRecords=io.canonicalSessionReceipts();assert(numel(allRecords)==before+n);records=[allRecords{before+1:end}];
    end
    function records=syntheticRecords(text,start)
        records=struct('decoded_message',{},'original_host_receive_ns',{},'decoded_source',{},'raw_frame_available',{});
        for fragment=1:ceil(numel(text)/70)
            m=createmsg(d,'SERIAL_CONTROL');m.Payload.device=uint8(10);m.Payload.flags=uint8(1);
            part=uint8(text((fragment-1)*70+1:min(fragment*70,numel(text))));m.Payload.count=uint8(numel(part));
            m.Payload.data(:)=0;m.Payload.data(1:numel(part))=part;
            wire=uint8(serializemsg(serializer,m));decoded=deserializemsg(d,wire);
            records(end+1)=struct('decoded_message',decoded,'original_host_receive_ns',start+uint64(fragment), ...
                'decoded_source','ORIGINAL_MAVLINKIO_SERIAL_CONTROL_CALLBACK','raw_frame_available',false); %#ok<AGROW>
        end
    end
    function sendSource(body)
        decoded=gpenmpcNative.RflySnapshotDecoder(body,gpenmpcNative.rflyOriginalHostMonotonicNs());
        for fragment=0:2
            m=createmsg(d,'TUNNEL');part=[uint8(48+fragment);be(uint64(decoded.subscription_generation));body(fragment*119+1:min(fragment*119+119,numel(body)))];
            m.Payload.payload_type=uint16(42002);m.Payload.target_system=uint8(255);m.Payload.target_component=uint8(190);
            m.Payload.payload_length=uint8(numel(part));m.Payload.payload(:)=0;m.Payload.payload(1:numel(part))=part;
            write(peer,uint8(serializemsg(serializer,m)),'uint8','127.0.0.1',cfg.local_mavlink_port);
        end
    end
    function packets=packInput(body,version,generation)
        count=ceil(numel(body)/119);packets=cell(count,1);
        for fragment=0:count-1
            m=createmsg(d,'TUNNEL');part=[uint8(version*16+fragment);be(generation);body(fragment*119+1:min(fragment*119+119,numel(body)))];
            m.Payload.payload_type=uint16(42002);m.Payload.target_system=uint8(1);m.Payload.target_component=uint8(1);
            m.Payload.payload_length=uint8(numel(part));m.Payload.payload(:)=0;m.Payload.payload(1:numel(part))=part;
            packets{fragment+1}=uint8(serializemsg(inputSerializer,m));
        end
    end
    function input=sendObserverPair(generation)
        write(peer,rotorPackets(:,generation),'uint8','127.0.0.1',cfg.canonical_rotor_observer.local_port);
        write(peer,cachePackets(:,generation),'uint8','127.0.0.1',cfg.canonical_rotor_observer.local_port);
        pause(.005);input=io.takeCanonicalRotorRecords();
    end
    function publishTimeAndGround(startMs,modelS,counter)
        nowMs=int64(floor(posixtime(datetime('now','TimeZone','UTC'))*1000));
        t=[le(int32(123456789));le(int32(1));le(startMs);le(nowMs);le(int64(counter))];
        write(peer,t,'uint8','127.0.0.1',cfg.coptersim_time_port);
        p=zeros(32,1);p(3)=modelS;p(4)=1;p(5)=1;p(7)=1;
        write(peer,[le(int32(1234567890));le(int32(1));le(p)],'uint8','127.0.0.1',cfg.truth_port);
        h=createmsg(d,'HEARTBEAT');h.Payload.type=uint8(13);h.Payload.autopilot=uint8(3);
        h.Payload.base_mode=uint8(0);h.Payload.custom_mode=bitor(bitshift(uint32(4),16),bitshift(uint32(6),24));
        write(peer,uint8(serializemsg(serializer,h)),'uint8','127.0.0.1',cfg.local_mavlink_port);
        h=createmsg(d,'EXTENDED_SYS_STATE');h.Payload.landed_state=uint8(1);
        write(peer,uint8(serializemsg(serializer,h)),'uint8','127.0.0.1',cfg.local_mavlink_port);
    end
    function answerTimesync()
        started=tic;while peer.NumDatagramsAvailable==0&&toc(started)<3,pause(.005);end
        assert(peer.NumDatagramsAvailable>0);
        while peer.NumDatagramsAvailable>0
            row=read(peer,1,'uint8');requestMessage=deserializemsg(d,uint8(row.Data));
            if requestMessage.MsgID==111
                reply=createmsg(d,'TIMESYNC');reply.Payload.ts1=requestMessage.Payload.ts1;
                reply.Payload.tc1=int64(round((io.now()+1)*1e9));
                write(peer,uint8(serializemsg(serializer,reply)),'uint8','127.0.0.1',cfg.local_mavlink_port);
            end
        end
        pause(.005);
    end
    function waitTunnel(n)
        started=tic;
        while toc(started)<3
            raw=io.evidence();count=sum(cellfun(@(x)strcmp(x.topic,'TUNNEL'),raw.raw_mavlink));
            if count>=n,return;end;pause(.005);
        end
        error('gpenmpcNative:RetirementReceiveTimeout');
    end
    function drainPeer(n)
        for packet=1:n
            started=tic;while peer.NumDatagramsAvailable==0&&toc(started)<3,pause(.005);end
            assert(peer.NumDatagramsAvailable>0);read(peer,1,'uint8');
        end
    end
    function check(name,yes),checks(end+1)=struct('name',name,'pass',logical(yes));assert(yes,'gpenmpcNative:RetirementTest','%s',name);end
    function reject(fn,prefix,name)
        caught='';try,fn();catch ex,caught=ex.identifier;end
        negative(end+1)=struct('name',name,'identifier',caught);check(name,startsWith(caught,prefix));
    end
end
function b=readbin(p),f=fopen(p,'rb');assert(f>0);c=onCleanup(@()fclose(f));b=fread(f,inf,'*uint8');end
function b=be(v),[~,~,e]=computer;if e=='L',v=swapbytes(v);end;b=reshape(typecast(v(:),'uint8'),[],1);end
function v=decode(b),v=typecast(b(:),'uint64');[~,~,e]=computer;if e=='L',v=swapbytes(v);end;v=v(:);end
function h=hex(b),h=upper(reshape(dec2hex(b,2).',1,[]));end
function b=digest(v),md=java.security.MessageDigest.getInstance('SHA-256');md.update(typecast(v(:),'int8'));b=reshape(typecast(md.digest(),'uint8'),[],1);end
function b=le(v),[~,~,e]=computer;if e=='B',v=swapbytes(v);end;b=reshape(typecast(v(:),'uint8'),[],1);end
