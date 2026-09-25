function report=test_canonical_px4_adapter_loopback(outputRoot)
% Test makeM600CopterSimIo with localhost MAVLink peers.
arguments,outputRoot (1,1) string,end
build=string(fileparts(fileparts(mfilename('fullpath'))));
outputRoot=string(char(java.io.File(char(outputRoot)).getCanonicalPath()));
assert(startsWith(lower(outputRoot),lower(build+"\evidence\"))&&~isfolder(outputRoot), ...
    'Use a fresh BUILD/evidence subdirectory.');
mkdir(outputRoot);oldPath=path;pathGuard=onCleanup(@()path(oldPath)); %#ok<NASGU>
addpath(fullfile(build,'matlab_validation'),fullfile(build,'m600_coptersim','matlab_validation'));
ports=48221:48225;
resources=containers.Map('KeyType','char','ValueType','any');
cleanupGuard=onCleanup(@()release(resources,ports));
checks=struct('name',{},'pass',{});raw=struct();failure='';
adapterPath=which('m600check.makeM600CopterSimIo');adapterSha=m600check.fileSha256(adapterPath);
observerPath=which('m600check.advanceCanonicalPx4Observation');observerSha=m600check.fileSha256(observerPath);
cfg=struct('local_mavlink_port',48221,'remote_mavlink_port',48222,'truth_port',48223, ...
    'coptersim_time_port',48224,'target_system',231,'target_component',77, ...
    'clock_max_rtt_s',2,'clock_sync_samples',3,'clock_sync_period_s',.03,'clock_max_age_s',30, ...
    'clock_max_uncertainty_s',1.1,'clock_max_utc_drift_s',.1,'clock_max_time_heartbeat_age_s',2, ...
    'clock_max_time_heartbeat_lag_s',.25,'maximum_truth_lag_s',2,'maximum_raw_records',1000, ...
    'state_max_age_s',.5,'heartbeat_max_age_s',.3,'landed_max_age_s',.3, ...
    'live_enabled',true,'outer_preflight_pass',true,'command_timeout_s',2,'poll_period_s',.01, ...
    'require_terrain_diagnostic_extension',false);
cfg.canonical_px4_observation=struct('verified_binding',struct( ...
    'uid','1234605616436508552','system_id',231,'component_id',77, ...
    'boot_generation',7,'verified',true),'maximum_queue',8,'maximum_age_ns',5e9);
try
    % Exclusive availability probe before constructing the actual adapter.
    for k=1:numel(ports)
        key=sprintf('probe%d',k);
        resources(key)=udpport('datagram','IPV4','LocalHost','127.0.0.1','LocalPort',ports(k));
    end
    check('five_high_ports_initially_available',true);
    for k=1:numel(ports),key=sprintf('probe%d',k);delete(resources(key));remove(resources,key);end
    dialect=mavlinkdialect('common.xml',2);
    peer=mavlinkio(dialect,'SystemID',231,'ComponentID',77);resources('peer')=peer;
    connect(peer,'UDP','LocalPort',48222);
    wrong=mavlinkio(dialect,'SystemID',232,'ComponentID',78);resources('wrong_peer')=wrong;
    connect(wrong,'UDP','LocalPort',48225);
    io=m600check.makeM600CopterSimIo(cfg);resources('adapter')=io;
    initial=io.canonicalObservation('SNAPSHOT');ev=io.evidence();
    check('real_adapter_optional_odometry_subscription',ev.canonical_px4_observer_enabled&& ...
        any(strcmp(ev.subscribed_topics,'ODOMETRY')));
    check('absence_not_filled_with_zero_observation',isempty(initial.snapshot.odometry)&& ...
        isempty(initial.snapshot.actuator)&&initial.snapshot.odometry_generation==0);
    heartbeat=createmsg(dialect,'HEARTBEAT');heartbeat.Payload.type=uint8(13);
    heartbeat.Payload.autopilot=uint8(12);heartbeat.Payload.base_mode=uint8(0);
    heartbeat.Payload.system_status=uint8(3);heartbeat.Payload.mavlink_version=uint8(3);
    sendudpmsg(peer,heartbeat,'127.0.0.1',48221);
    odom=createmsg(dialect,'ODOMETRY');p=odom.Payload;
    p.time_usec=uint64(2300000);p.frame_id=uint8(1);p.child_frame_id=uint8(1);
    p.x=single(1.25);p.y=single(-2.5);p.z=single(-3.75);p.q=single([1,0,0,0]);
    p.vx=single(.1);p.vy=single(.2);p.vz=single(-.3);
    p.rollspeed=single(.01);p.pitchspeed=single(.02);p.yawspeed=single(-.03);
    p.pose_covariance=NaN(1,21,'single');p.velocity_covariance=NaN(1,21,'single');
    p.reset_counter=uint8(3);p.estimator_type=uint8(8);odom.Payload=p;
    actuator=createmsg(dialect,'HIL_ACTUATOR_CONTROLS');actuator.Payload.time_usec=uint64(2304000);
    actuator.Payload.controls=single([.1,.2,.3,.4,.5,.6,NaN(1,10)]);
    actuator.Payload.mode=uint8(0);actuator.Payload.flags=uint64(0);
    sendudpmsg(peer,odom,'127.0.0.1',48221);sendudpmsg(peer,actuator,'127.0.0.1',48221);
    raw.first=waitFor(io,@(r)r.snapshot.odometry_generation==1&&r.snapshot.actuator_generation==1);
    o=raw.first.snapshot.odometry;a=raw.first.snapshot.actuator;
    check('real_callback_atomically_received_both_streams',~isempty(o)&&~isempty(a)&& ...
        o.message_id==331&&a.message_id==93&&o.source_generation==1&&a.source_generation==1);
    check('actual_mavlink_source_and_explicit_binding',o.system_id==231&&o.component_id==77&& ...
        a.system_id==231&&a.component_id==77&&strcmp(o.uid,'1234605616436508552')&&o.boot_generation==7);
    check('untranslated_ned_actual_payload',isequal(o.position_ned_m,double([p.x;p.y;p.z]))&& ...
        ~o.task_origin_added&&strcmp(o.coordinate_origin,'PX4_LOCAL_NED_UNTRANSLATED'));
    check('atomic_odometry_one_generation_one_receive',o.position_generation==o.attitude_generation&& ...
        o.position_generation==o.rates_generation&&o.position_rx_ns==o.attitude_rx_ns&& ...
        o.rates_rx_ns==o.rx_ns&&o.source_time_ns==2300000000&& ...
        o.atomic_estimate&&~o.plant_truth_used);
    check('hil93_preserves_unused_nan',isequal(a.controls(1:6),double(actuator.Payload.controls(1:6).'))&& ...
        all(isnan(a.controls(7:16)))&&a.source_time_ns==2304000000);
    ev=io.evidence();rawAccepted=ev.raw_mavlink;
    seen=false;
    for k=1:numel(rawAccepted)
        row=rawAccepted{k};
        if strcmp(row.topic,'ODOMETRY')
            seen=round(row.rx_s*1e9)==o.rx_ns&&row.message.SystemID==o.system_id&& ...
                row.message.ComponentID==o.component_id&&row.message.Seq==o.wire_sequence;
        end
    end
    check('raw_callback_metadata_matches_atomic_sample',seen);
    sendudpmsg(peer,odom,'127.0.0.1',48221);
    raw.duplicate=waitFor(io,@(r)r.snapshot.odometry_duplicates==1);
    check('duplicate_callback_no_generation_or_receive_renewal', ...
        raw.duplicate.snapshot.odometry_generation==1&&raw.duplicate.snapshot.odometry.rx_ns==o.rx_ns&& ...
        raw.duplicate.snapshot.odometry_queue_count==1);
    raw.drain=io.canonicalObservation('DRAIN');
    check('actual_drain_keeps_source_and_original_receive',numel(raw.drain.odometry_queue)==1&& ...
        numel(raw.drain.actuator_queue)==1&&raw.drain.odometry_queue{1}.rx_ns==o.rx_ns&& ...
        raw.drain.odometry_queue{1}.source_time_ns==o.source_time_ns);
    raw.empty=io.canonicalObservation('DRAIN');
    check('redrain_does_not_repeat_sample',isempty(raw.empty.odometry_queue)&&isempty(raw.empty.actuator_queue));
    beforeWrong=io.evidence();wrongOdom=odom;wrongOdom.Payload.time_usec=uint64(2330000);
    sendudpmsg(wrong,heartbeat,'127.0.0.1',48221);
    sendudpmsg(wrong,wrongOdom,'127.0.0.1',48221);sendudpmsg(wrong,actuator,'127.0.0.1',48221);
    pause(.20);afterWrong=io.evidence();wr=io.canonicalObservation('SNAPSHOT');
    check('wrong_source_client_filtered_not_valid_or_relabelled', ...
        wr.snapshot.odometry_generation==1&&wr.snapshot.actuator_generation==1&& ...
        numel(afterWrong.raw_mavlink)==numel(beforeWrong.raw_mavlink)&&~wr.fatal_latched);
    next=odom;next.Payload.time_usec=uint64(2337500);
    sendudpmsg(peer,next,'127.0.0.1',48221);
    raw.next=waitFor(io,@(r)r.snapshot.odometry_generation==2);
    check('real_callback_non10ms_gap_is_observation_not_gate', ...
        raw.next.snapshot.odometry.source_gap_ns==37.5e6&&~raw.next.fatal_latched);
    malformed=next;malformed.Payload.time_usec=uint64(2350000);malformed.Payload.q=single([2,0,0,0]);
    sendudpmsg(peer,malformed,'127.0.0.1',48221);
    raw.invalid=waitFor(io,@(r)r.fatal_latched);
    check('actual_bad_payload_permanently_fails_closed',raw.invalid.fatal_latched&& ...
        strcmp(raw.invalid.snapshot.first_failure.reason,'ODOMETRY_QUATERNION_INVALID')&& ...
        isempty(raw.invalid.snapshot.odometry)&&raw.invalid.snapshot.odometry_generation==2);
    repaired=next;repaired.Payload.time_usec=uint64(2360000);
    sendudpmsg(peer,repaired,'127.0.0.1',48221);pause(.08);
    raw.after_fatal=io.canonicalObservation('DRAIN');ev=io.evidence();
    check('later_good_wire_message_cannot_heal_fatal',raw.after_fatal.fatal_latched&& ...
        isempty(raw.after_fatal.odometry_queue)&&raw.after_fatal.snapshot.odometry_generation==2);
    check('adapter_fatal_and_raw_invalid_message_retained', ...
        contains(ev.fatal,'CANONICAL_OBSERVATION:ODOMETRY_QUATERNION_INVALID')&& ...
        ev.canonical_px4_observer_state.fatal_latched&& ...
        any(cellfun(@(row)double(row.message.MsgID)==331&& ...
            double(row.message.Payload.time_usec)==2350000&& ...
            double(row.message.Payload.q(1))==2,ev.raw_mavlink)));
    check('observer_path_sent_no_commands_or_requests',isempty(ev.raw_transmit_messages)&& ...
        isempty(ev.raw_environment_transmit_datagrams)&&isempty(ev.real_parameter_actions));
    raw.enabled_adapter_evidence=ev;
    check('enabled_adapter_close_succeeds',io.close());remove(resources,'adapter');
    cfg=rmfield(cfg,'canonical_px4_observation');
    io=m600check.makeM600CopterSimIo(cfg);resources('adapter')=io;
    disabled=io.evidence();
    check('default_legacy_path_not_subscribed_or_enabled',~disabled.canonical_px4_observer_enabled&& ...
        ~any(strcmp(disabled.subscribed_topics,'ODOMETRY'))&&isempty(disabled.canonical_px4_observer_state));
    check('disabled_observer_interface_rejects',rejects(@()io.canonicalObservation('SNAPSHOT'), ...
        'm600check:CanonicalObserverNotConfigured'));
    check('sources_stayed_byte_identical',strcmp(adapterSha,m600check.fileSha256(adapterPath))&& ...
        strcmp(observerSha,m600check.fileSha256(observerPath)));
catch ex
    failure=getReport(ex,'extended','hyperlinks','off');
    if isKey(resources,'adapter')
        try,currentIo=resources('adapter');raw.failure_evidence=currentIo.evidence();catch,end
    end
end
release(resources,ports);clear cleanupGuard
cleanup=resources('cleanup_report');
check('all_resources_closed_and_five_ports_rebound',cleanup.all_closed&&cleanup.ports_rebound);
report=struct('schema','CANONICAL_PX4_REAL_ADAPTER_LOCALHOST_TEST_V1', ...
    'pass',isempty(failure)&&all([checks.pass]),'checks_total',numel(checks), ...
    'checks_passed',nnz([checks.pass]),'checks',checks,'failure',failure,'cleanup',cleanup, ...
    'adapter_sha256',adapterSha,'observer_sha256',observerSha, ...
    'test_sha256',m600check.fileSha256(mfilename('fullpath')+".m"), ...
    'localhost_ports',ports,'udp_connects',4,'COM_open',0,'board_actions',0, ...
    'RflySim_runs',0,'plant_runs',0,'model_runs',0,'parameter_actions',0,'arm_actions',0, ...
    'claim','MATLAB adapter localhost callback tests.');
save(fullfile(outputRoot,'RAW.mat'),'raw','report');
fid=fopen(fullfile(outputRoot,'RESULT.json'),'w','n','UTF-8');assert(fid>=0);
f=onCleanup(@()fclose(fid));fprintf(fid,'%s\n',jsonencode(report,PrettyPrint=true));clear f
fprintf('CANONICAL_PX4_ADAPTER %d/%d pass=%d\n',report.checks_passed,report.checks_total,report.pass);
if ~isempty(failure),fprintf('%s\n',failure);end
assert(report.pass,'m600check:CanonicalAdapterLoopbackTest','See RESULT.json.');
    function check(name,pass)
        checks(end+1)=struct('name',name,'pass',logical(pass)); %#ok<AGROW>
        if ~pass,error('m600check:CanonicalAdapterLoopbackCheck','Failed: %s',name);end
    end
end
function r=waitFor(io,predicate)
t=tic;r=[];
while toc(t)<4
    pause(.02);r=io.canonicalObservation('SNAPSHOT');
    if predicate(r),return,end
    if r.fatal_latched
        error('m600check:CanonicalAdapterCallbackFatal','%s',r.snapshot.first_failure.reason);
    end
end
error('m600check:CanonicalAdapterCallbackTimeout','Callback did not reach expected observation state.');
end
function yes=rejects(f,id)
yes=false;try,f();catch ex,yes=strcmp(ex.identifier,id);end
end
function release(resources,ports)
if isKey(resources,'cleanup_report'),return,end
allClosed=true;errors={};
keys=resources.keys;
if any(strcmp(keys,'adapter')),keys=[{'adapter'},keys(~strcmp(keys,'adapter'))];end
for k=1:numel(keys)
    key=keys{k};obj=resources(key);
    try
        if strcmp(key,'adapter'),okay=obj.close();assert(okay);
        elseif isa(obj,'mavlinkio'),disconnect(obj);delete(obj);
        else,delete(obj);end
    catch ex,allClosed=false;errors{end+1}=ex.message;end %#ok<AGROW>
    remove(resources,key);
end
bound=true;
for port=ports
    try,p=udpport('datagram','IPV4','LocalHost','127.0.0.1','LocalPort',port);delete(p);
    catch ex,bound=false;errors{end+1}=ex.message;end %#ok<AGROW>
end
resources('cleanup_report')=struct('all_closed',allClosed,'ports_rebound',bound,'errors',{errors});
end
