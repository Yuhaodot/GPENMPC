function report = run_m600_adapter_loopback_api_smoke(outputDir,requireTerrainExtension)
% Test the MATLAB UAV Toolbox and udpport APIs on localhost ports 48171:48175.
% Send synthetic messages through the installed serializer, parser, subscriber and adapter.
% References: mavlinkio/mavlinksub/mavlinkdialect; DllSimCtrlAPI.py and ReqCopterSim.py.
arguments
    outputDir (1,1) string
    requireTerrainExtension (1,1) logical = false
end
assert(~isfolder(outputDir),'gpenmpc:FixtureOutputExists','Preserve prior results.');
mkdir(outputDir);
root=fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(root,'matlab_validation'));
addpath(fullfile(root,'m600_coptersim','matlab_validation'));
adapter=which('m600check.makeM600CopterSimIo');
beforeSha=fileSha(adapter);
% Use ports outside the Windows excluded UDP range.
cfg=struct('local_mavlink_port',48171,'remote_mavlink_port',48172, ...
    'truth_port',48173,'coptersim_time_port',48174,'target_system',231, ...
    'target_component',77,'clock_max_rtt_s',2,'clock_sync_samples',3, ...
    'clock_sync_period_s',0.05,'clock_max_age_s',30, ...
    'clock_max_uncertainty_s',1.1,'clock_max_utc_drift_s',0.1, ...
    'clock_max_time_heartbeat_age_s',2,'clock_max_time_heartbeat_lag_s',0.25, ...
    'maximum_truth_lag_s',2,'maximum_raw_records',10000, ...
    'state_max_age_s',0.5,'heartbeat_max_age_s',3,'live_enabled',true,'outer_preflight_pass',true, ...
    'command_timeout_s',3,'poll_period_s',0.01);
cfg.require_terrain_diagnostic_extension=requireTerrainExtension;
ports=[48171,48172,48173,48174,48175];
assert(~any(ismember(ports,[14550,18570,30101,20005])));
io=[];peer=[];rawSender=[];peerSub=[];peerTimer=[];
peerReceived={};peerSent={};peerError='';peerErrorReport='';checks=struct('name',{},'pass',{});
lostReadCount=0;missingReadCount=0;
failure=struct('identifier','','message','','stack',[]);
snapshots={};positive=[];paramInt=[];paramReal=[];ev=[];
oldClockNegative=[];reverseClockNegative=[];
startUtcMs=int64(0);peerClock=[];counter=int64(0);timerTickCount=0;
diagFreeze=false;diagTime=0;diagFailed=false;originShiftMs=int64(0);
telemetryFreeze=false;telemetrySource=100;telemetryReverse=false;estimatorErrorFlag=false;
emitAttitudeTarget=false;
timeHeartbeatDelayMs=int64(0);timeCounterReverse=false;
cleanupReceipt=struct('adapter_closed',false,'peer_closed',false, ...
    'sender_closed',false,'ports_rebound',false,'errors',{{}});
guard=onCleanup(@closeOwned); %#ok<NASGU>
try
    % All endpoints must be unused before we construct either participant.
    probes=cell(1,numel(ports));
    try
        for k=1:numel(ports)
            probes{k}=udpport('datagram','IPV4','LocalHost','127.0.0.1', ...
                'LocalPort',ports(k),'Timeout',0.01);
        end
    catch err
        for k=1:numel(probes),if ~isempty(probes{k}),delete(probes{k});end;end
        rethrow(err)
    end
    for k=1:numel(probes),delete(probes{k});end
    check('dedicated_ports_available',true);
    dialect=mavlinkdialect(fullfile(fileparts(adapter),'px4_health_events.xml'));
    peer=mavlinkio(dialect,'SystemID',cfg.target_system,'ComponentID',cfg.target_component);
    connect(peer,'UDP','LocalPort',cfg.remote_mavlink_port);
    hostClient=mavlinkclient(peer,255,190);
    peerSub=mavlinksub(peer,hostClient,'BufferSize',1000,'NewMessageFcn',@onPeer);
    rawSender=udpport('datagram','IPV4','LocalHost','127.0.0.1', ...
        'LocalPort',48175,'Timeout',0.01);
    io=m600check.makeM600CopterSimIo(cfg);
    check('actual_api_objects_constructed',true);
    initialEvidence=io.evidence();pair=initialEvidence.utc_pair_calibration;
    check('production_bounded_utc_pair_keeps_all_ten_brackets', ...
        pair.sample_count==10&&numel(pair.samples)==10);
    check('production_utc_uncertainty_uses_measured_selected_bracket', ...
        abs(pair.uncertainty_s-(pair.samples(pair.selected_sample).bracket_s/2+pair.roundoff_allowance_s))<1e-12);
    check('production_utc_pair_preserves_configured_drift_bound', ...
        pair.utc_drift_bound_s==cfg.clock_max_utc_drift_s);
    startUtcMs=int64(floor(posixtime(datetime('now','TimeZone','UTC'))*1000));
    peerClock=tic;
    peerTimer=timer('ExecutionMode','fixedSpacing','Period',0.05, ...
        'BusyMode','drop','TimerFcn',@(~,~)emitPeer,'ErrorFcn',@onTimerError);
    start(peerTimer);
    io.sendHeartbeat();
    t=tic;
    while toc(t)<6
        s=io.snapshot();snapshots{end+1}=s; %#ok<AGROW>
        if ~isempty(peerError),error('gpenmpc:PeerCallback','%s',peerError);end
        if s.clock_valid&&s.model_ready&&~isempty(s.estimate)&& ...
                ~isempty(s.truth)&&~isempty(s.autopilot_version)&&s.native_hover_telemetry.passed
            positive=s;break
        end
        pause(0.02);
    end
    check('actual_callbacks_clock_model_and_state_ready',~isempty(positive));
    assert(~isempty(positive),'gpenmpc:NoPositiveSnapshot', ...
        'Actual adapter did not obtain complete positive API observation.');
    check('heartbeat_disarmed_and_landed',positive.armed==0&&positive.landed_state==1);
    check('actual_SYS_STATUS_callback_observed',~isempty(positive.system_status)&& ...
        positive.system_status.onboard_control_sensors_present==uint32(7));
    healthEvidence=io.evidence();healthRows=healthEvidence.raw_mavlink( ...
        cellfun(@(x)strcmp(x.topic,'EVENT'),healthEvidence.raw_mavlink));
    check('actual_EVENT_callback_keeps_source_sequence_and_40_arguments', ...
        ~isempty(healthRows)&&healthRows{end}.message.Payload.id==uint32(27725120)&& ...
        isequal(reshape(healthRows{end}.message.Payload.arguments,1,[]),uint8(1:40)));
    check('version_payload_uint64_preserved',isa(positive.autopilot_version.uid,'uint64')&& ...
        positive.autopilot_version.uid==uint64(42));
    check('version_receive_time_fresh',isfinite(positive.autopilot_version_rx_s)&& ...
        positive.now_s-positive.autopilot_version_rx_s<2);
    check('estimate_position_schema',isequal(positive.estimate.position_ned_m,[1,2,-3]));
    check('truth_world_position_schema',isequal(positive.truth.position_ned_m,[1,2,-3]));
    check('model_diagnostic_v1',positive.model_diagnostic.decoded.schema_version==1&& ...
        positive.model_diagnostic.decoded.ground_confirmed);
    if requireTerrainExtension
        check('actual_UDP_model_extension_required_present_and_valid', ...
            positive.model_diagnostic.decoded.terrain_extension.present&& ...
            positive.model_diagnostic.decoded.terrain_extension.valid);
    end
    check('source_clock_not_receive_timestamp', ...
        positive.estimate.raw_source_time_s>=100&&positive.truth.raw_source_time_s<30&& ...
        strcmp(positive.estimate.sample.clock_id,'MATLAB_MONOTONIC_MAPPED_SOURCE_TIME'));
    check('actual_ATT_EST_callbacks_current_source_pass',positive.native_hover_telemetry.passed);
    check('actual_optional_EST_nan_preserved', ...
        positive.native_hover_telemetry.optional_estimator_fields.hagl_ratio.is_nan&& ...
        positive.native_hover_telemetry.optional_estimator_fields.tas_ratio.is_nan);
    check('actual_source_progress_recorded_three_streams', ...
        positive.estimate.source_advance_count>=1&&positive.attitude_observation.source_advance_count>=1&& ...
        positive.estimator_status_observation.source_advance_count>=1);
    targetEvidence=io.evidence();
    check('missing_optional_attitude_target_does_not_block_native_telemetry', ...
        positive.native_hover_telemetry.passed&& ...
        ismember('ATTITUDE_TARGET',targetEvidence.subscribed_topics)&& ...
        ~any(cellfun(@(x)strcmp(x.topic,'ATTITUDE_TARGET'),targetEvidence.raw_mavlink)));
    emitAttitudeTarget=true;pause(.12);io.snapshot();targetEvidence=io.evidence();
    targetRows=targetEvidence.raw_mavlink(cellfun(@(x)strcmp(x.topic,'ATTITUDE_TARGET'),targetEvidence.raw_mavlink));
    check('actual_optional_attitude_target_payload_recorded_without_target_invention', ...
        ~isempty(targetRows)&&isequal(targetRows{end}.message.Payload.q,single([1,0,0,0]))&& ...
        targetRows{end}.message.Payload.type_mask==0&& ...
        targetRows{end}.message.Payload.body_roll_rate==single(.125)&& ...
        targetRows{end}.message.Payload.body_pitch_rate==single(-.25)&& ...
        targetRows{end}.message.Payload.body_yaw_rate==single(.375)&& ...
        targetRows{end}.message.Payload.thrust==single(.5));
    paramInt=io.readParameter('HOST_API_INT32');
    paramReal=io.readParameter('HOST_API_REAL');
    paramUint=io.readParameter('HOST_API_UINT');
    check('typed_INT32_roundtrip',paramInt.mav_type==6&&paramInt.decoded==101&& ...
        strcmp(paramInt.raw_bits_hex,'00000065'));
    check('typed_REAL32_roundtrip',paramReal.mav_type==9&&paramReal.decoded==0.125&& ...
        strcmp(paramReal.raw_bits_hex,'3E000000'));
    check('UINT32_NaN_bits_not_numeric_NaN_identity',paramUint.mav_type==5&&paramUint.decoded==4294967295&&strcmp(paramUint.raw_bits_hex,'FFFFFFFF'));
    lost=io.readParameter('HOST_API_LOST');
    check('lost_first_read_response_retried_same_request',lost.decoded==0.125&&lostReadCount==2);
    beforeMissing=io.evidence();beforeMissingClock=numel(beforeMissing.raw_clock_datagrams);
    beforeMissingTruth=numel(beforeMissing.raw_truth_datagrams);
    missingStart=tic;missingFailure='';
    try,io.readParameter('HOST_API_MISS');catch e,missingFailure=e.identifier;end
    missingElapsed=toc(missingStart);
    check('missing_read_stays_fail_closed',strcmp(missingFailure,'m600check:ParameterTimeout'));
    check('read_retry_does_not_extend_original_deadline',missingElapsed>=cfg.command_timeout_s&&missingElapsed<cfg.command_timeout_s+.5);
    check('missing_read_bounded_three_requests',missingReadCount==3);
    afterMissing=io.evidence();
    check('blocking_read_pumps_same_clock_and_truth_sockets', ...
        numel(afterMissing.raw_clock_datagrams)>beforeMissingClock+10&& ...
        numel(afterMissing.raw_truth_datagrams)>beforeMissingTruth+20);
    s=io.snapshot();snapshots{end+1}=s;
    check('blocking_read_does_not_create_false_clock_dequeue_lag', ...
        s.clock_valid&&s.clock_diagnostic.time_heartbeat_lag_pass&& ...
        s.clock_diagnostic.time_heartbeat_lag_s<=.25);
    check('snapshot_now_follows_captured_rx_fields',s.now_s>=s.estimate.rx_s&& ...
        s.now_s>=s.attitude_observation.latest_rx_s&&s.now_s>=s.estimator_status_observation.latest_rx_s);
    % Distinguish an old producer UTC from an unpolled queue while keeping
    % source counters and origins monotonic.
    timeHeartbeatDelayMs=int64(500);pause(.12);s=io.snapshot();snapshots{end+1}=s;
    oldClockNegative=s.clock_diagnostic;
    check('actual_old_clock_currentUTC_over_point25_still_rejected', ...
        ~s.clock_valid&&~s.clock_diagnostic.time_heartbeat_lag_pass&& ...
        s.clock_diagnostic.time_heartbeat_lag_s>.25&& ...
        s.clock_diagnostic.time_heartbeat_dequeue_age_s<.25);
    timeHeartbeatDelayMs=int64(0);pause(.12);s=io.snapshot();snapshots{end+1}=s;
    check('current_clock_packet_only_recovers_nonlatched_lag_not_reset', ...
        s.clock_valid&&s.clock_diagnostic.time_heartbeat_lag_pass&&~s.clock_diagnostic.clock_origin_changed);
    sentEvidence=io.evidence();sentRows=sentEvidence.raw_transmit_messages;
    hbTimes=cellfun(@(x)x.sent_s,sentRows(cellfun(@(x)double(x.message.MsgID)==0,sentRows)));
    check('heartbeat_maintained_during_blocking_reads',numel(hbTimes)>=4&&max(diff(hbTimes))<1.3);
    ev=io.evidence();
    check('timesync_actual_accepted_responses',nnz([ev.timesync_responses.accepted])>=3);
    check('raw_264_and_168_received',any(cellfun(@(x)numel(x.bytes)==264,ev.raw_truth_datagrams))&& ...
        any(cellfun(@(x)numel(x.bytes)==168,ev.raw_truth_datagrams)));
    check('raw_32_time_received',any(cellfun(@(x)numel(x.bytes)==32,ev.raw_clock_datagrams)));
    check('callback_topics_not_captured_as_one_loop_name', ...
        all(ismember({'HEARTBEAT','EXTENDED_SYS_STATE','LOCAL_POSITION_NED', ...
        'AUTOPILOT_VERSION','TIMESYNC','PARAM_VALUE','ATTITUDE','ATTITUDE_TARGET','ESTIMATOR_STATUS'}, ...
        cellfun(@(x)x.topic,ev.raw_mavlink,'UniformOutput',false))));
    telemetryFreeze=true;t=tic;
    while toc(t)<cfg.heartbeat_max_age_s+.25,s=io.snapshot();pause(.02);end
    snapshots{end+1}=s;
    check('actual_duplicate_ATT_EST_cannot_wash_source_staleness', ...
        ~s.native_hover_telemetry.passed&&s.attitude_observation.duplicate_count>1&& ...
        s.estimator_status_observation.duplicate_count>1&& ...
        ~s.native_hover_telemetry.streams.attitude.checks.progress_fresh&& ...
        ~s.native_hover_telemetry.streams.estimator_status.checks.progress_fresh&& ...
        s.now_s-s.attitude_observation.latest_rx_s<cfg.state_max_age_s&& ...
        s.now_s-s.estimator_status_observation.latest_rx_s<cfg.state_max_age_s);
    telemetryFreeze=false;t=tic;
    while toc(t)<2,s=io.snapshot();if s.native_hover_telemetry.passed,break;end;pause(.02);end
    check('actual_new_telemetry_progress_restores_only_nonfault_staleness',s.native_hover_telemetry.passed);
    estimatorErrorFlag=true;pause(.12);s=io.snapshot();snapshots{end+1}=s;
    check('actual_EST_error_flag_rejected',~s.native_hover_telemetry.passed&& ...
        ~s.native_hover_telemetry.checks.estimator_error_flags_clear);
    estimatorErrorFlag=false;pause(.12);s=io.snapshot();
    check('actual_current_EST_clear_reported',s.native_hover_telemetry.passed);
    % Continued duplicate packets must NOT replenish source-progress freshness.
    diagFreeze=true;t=tic;
    while toc(t)<cfg.state_max_age_s+0.25
        s=io.snapshot();pause(0.02);
    end
    snapshots{end+1}=s;
    check('duplicate_diagnostic_does_not_refresh_source',~s.model_ready&& ...
        strcmp(s.model_diagnostic.status,'MODEL_DIAGNOSTIC_SOURCE_PROGRESS_STALE')&& ...
        s.model_diagnostic.receive_age_s<cfg.state_max_age_s);
    diagFreeze=false;t=tic;
    while toc(t)<2
        s=io.snapshot();if s.model_ready,break;end;pause(0.02);
    end
    check('new_source_progress_recovers_nonfault_staleness',s.model_ready);
    % A reported model fault must survive a later healthy packet.
    diagFailed=true;pause(0.1);s=io.snapshot();snapshots{end+1}=s;
    check('actual_264_model_failure_fail_closed',~s.model_ready&& ...
        contains(s.fatal,'MODEL_DIAGNOSTIC:VALID_PACKET_MODEL_FAILURE_LATCHED'));
    diagFailed=false;pause(0.1);s=io.snapshot();snapshots{end+1}=s;
    check('model_failure_stays_latched_after_healthy_packet',~s.model_ready&& ...
        ~isempty(s.first_fatal)&&s.first_fatal.model_diagnostic);
    if requireTerrainExtension
        first=s.model_diagnostic.first_fault.decoded.terrain_extension;
        check('actual_UDP_first_terrain_nonfinite_reason_and_raw_bits_retained', ...
            first.valid&&first.capture_valid&&first.first_reason==2&& ...
            strcmp(first.first_terrain_float64_hex{15},'7FF800000000007B'));
        check('terrain_capture_never_restores_model_health',~s.model_ready);
    end
    check('model_fault_does_not_fabricate_clock_fault',s.clock_valid);
    % Test counter faults before origin shifts, because an origin shift can
    % latch truth-mapping lag first.
    timeCounterReverse=true;pause(.12);s=io.snapshot();snapshots{end+1}=s;
    evidenceReverse=io.evidence();
    reverseClockNegative=s.clock_diagnostic;
    check('actual_clock_counter_reversal_is_first_nonmodel_fatal',~s.clock_valid&& ...
        strcmp(evidenceReverse.non_model_fatal,'COPTERSIM_HEARTBEAT_COUNT_REVERSED'));
    clockRaw=evidenceReverse.raw_clock_datagrams;clockCounters=cellfun(@counterValue,clockRaw);
    check('clock_reversal_preserved_in_raw',any(diff(clockCounters)<0));
    timeCounterReverse=false;pause(.12);s=io.snapshot();snapshots{end+1}=s;
    check('actual_clock_reversal_stays_fatal_after_new_progress', ...
        strcmp(s.clock_diagnostic.non_model_fatal,'COPTERSIM_HEARTBEAT_COUNT_REVERSED')&&~s.clock_valid);
    originShiftMs=int64(1000);pause(0.1);s=io.snapshot();snapshots{end+1}=s;
    check('actual_32_clock_origin_change_invalidates_mapping',~s.clock_valid&&s.clock_diagnostic.clock_origin_changed);
    telemetryReverse=true;pause(.12);s=io.snapshot();snapshots{end+1}=s;
    check('actual_ATT_EST_reversal_latched',s.attitude_observation.source_reversal_latched&& ...
        s.estimator_status_observation.source_reversal_latched);
    telemetryReverse=false;pause(.12);s=io.snapshot();snapshots{end+1}=s;
    check('actual_ATT_EST_reversal_survives_later_healthy_source', ...
        s.attitude_observation.source_reversal_latched&&s.estimator_status_observation.source_reversal_latched&& ...
        ~s.native_hover_telemetry.passed);
    check('fixture_timer_executed',timerTickCount>=5);
catch err
    failure=struct('identifier',err.identifier,'message',err.message,'stack',err.stack);
end
closeOwned();
delete(guard); % execute while captured workspace still exists; idempotent
if ~isempty(io)
    try,ev=io.evidence();
    catch err
        cleanupReceipt.errors{end+1}=['Evidence after close: ' err.message];
        if isempty(failure.identifier)
            failure=struct('identifier',err.identifier,'message',err.message,'stack',err.stack);
        end
    end
end
% Check fixture isolation.
ids=cellfun(@(x)double(x.MsgID),peerReceived);
check('peer_saw_only_heartbeat_timesync_parameter_read',all(ismember(ids,[0,111,20])));
if ~isempty(ev)
    txids=cellfun(@(x)double(x.message.MsgID),ev.raw_transmit_messages);
    check('adapter_sent_no_command_paramset_setpoint',all(ismember(txids,[0,111,20])));
    % Validate periodic renewal over the complete fixture.
    check('production_timesync_continues_after_lock_without_source_refit', ...
        ev.timesync_validation_state.renewal_count>3&& ...
        isequaln(ev.timesync_validation_state.best,ev.fixed_best_timesync)&& ...
        ev.timesync_validation_state.last_validation_receive_s>ev.fixed_best_timesync.receive_s);
end
check('adapter_source_unchanged_during_fixture',strcmp(beforeSha,fileSha(adapter)));
check('owned_endpoints_closed_and_rebound',cleanupReceipt.adapter_closed&& ...
    cleanupReceipt.peer_closed&&cleanupReceipt.sender_closed&&cleanupReceipt.ports_rebound);
check('no_peer_callback_error',isempty(peerError));
report=struct('classification','HOST_API_LOOPBACK_FIXTURE', ...
    'pass',isempty(failure.identifier)&&all([checks.pass]), ...
    'checks',checks,'checks_total',numel(checks),'checks_passed',nnz([checks.pass]), ...
    'failure',failure,'peer_callback_error',peerError,'peer_callback_error_report',peerErrorReport,'config',cfg, ...
    'fixture_bounds_are_not_live_safety_or_science_thresholds',true, ...
    'adapter_path',adapter,'adapter_sha256',beforeSha,'matlab_version',version, ...
    'telemetry_helper_sha256',fileSha(which('m600check.evaluateNativeHoverTelemetry')), ...
    'fixture_source_sha256',fileSha(mfilename('fullpath')+".m"), ...
    'api_paths',struct('mavlinkio',which('mavlinkio'),'mavlinksub',which('mavlinksub'), ...
    'mavlinkclient',which('mavlinkclient'),'mavlinkdialect',which('mavlinkdialect')), ...
    'peer_received_count',numel(peerReceived),'peer_sent_count',numel(peerSent), ...
    'peer_timer_ticks',timerTickCount,'parameter_INT32',paramInt,'parameter_REAL32',paramReal, ...
    'old_current_utc_negative',oldClockNegative,'counter_reversal_negative',reverseClockNegative, ...
    'cleanup',cleanupReceipt,'hardware_actions',0,'COM_open',0,'simulator_launch',0, ...
    'PX4_access',0,'arm_disarm_mode_setpoint_parameter_write',0);
save(fullfile(outputDir,'HOST_API_RAW.mat'),'report','snapshots','positive','ev', ...
    'peerReceived','peerSent','-v7');
fid=fopen(fullfile(outputDir,'HOST_API_RESULT.json'),'w');assert(fid>=0);
fileGuard=onCleanup(@()fclose(fid)); %#ok<NASGU>
fwrite(fid,jsonencode(report,PrettyPrint=true),'char');

    function check(name,value)
        checks(end+1)=struct('name',name,'pass',logical(value));
    end
    function onTimerError(~,event)
        peerError=['TIMER:' event.Data.Message];
    end
    function onPeer(~,messages)
        try
            for j=1:numel(messages)
                m=messages(j);peerReceived{end+1}=m; %#ok<AGROW>
                if m.MsgID==111&&m.Payload.tc1==0
                    r=createmsg(dialect,'TIMESYNC');
                    r.Payload.tc1=int64(round((100+toc(peerClock))*1e9));
                    r.Payload.ts1=int64(m.Payload.ts1);sendPeer(r);
                elseif m.MsgID==20
                    % MATLAB's MAVLink char field decoder space-pads param_id.
                    name=strtrim(strrep(char(m.Payload.param_id),char(0),''));
                    if strcmp(name,'HOST_API_LOST')
                        lostReadCount=lostReadCount+1;if lostReadCount==1,continue;end
                    elseif strcmp(name,'HOST_API_MISS')
                        missingReadCount=missingReadCount+1;continue
                    end
                    r=createmsg(dialect,'PARAM_VALUE');
                    r.Payload.param_id=text16(name);r.Payload.param_count=uint16(2);
                    if strcmp(name,'HOST_API_INT32')
                        r.Payload.param_type=uint8(6);r.Payload.param_value=typecast(int32(101),'single');
                        r.Payload.param_index=uint16(0);
                    elseif any(strcmp(name,{'HOST_API_REAL','HOST_API_LOST'}))
                        r.Payload.param_type=uint8(9);r.Payload.param_value=single(0.125);
                        r.Payload.param_index=uint16(1);
                    elseif strcmp(name,'HOST_API_UINT')
                        r.Payload.param_type=uint8(5);r.Payload.param_value=typecast(uint32(4294967295),'single');
                        r.Payload.param_index=uint16(2);
                    else,error('gpenmpc:UnexpectedRead','Unexpected fixture parameter %s.',name);
                    end
                    sendPeer(r);
                elseif m.MsgID~=0
                    error('gpenmpc:ForbiddenFixtureMessage','Unexpected outbound MsgID=%d.',m.MsgID);
                end
            end
        catch err
            if isempty(peerError)
                peerError=['PEER:' err.identifier ':' err.message];
                peerErrorReport=getReport(err,'extended','hyperlinks','off');
            end
        end
    end
    function sendPeer(m)
        peerSent{end+1}=m;
        sendudpmsg(peer,m,'127.0.0.1',cfg.local_mavlink_port);
    end
    function emitPeer()
        try
            elapsed=toc(peerClock);timerTickCount=timerTickCount+1;counter=counter+1;
            m=createmsg(dialect,'HEARTBEAT');m.Payload.type=uint8(13);
            m.Payload.autopilot=uint8(12);m.Payload.base_mode=uint8(0);
            m.Payload.custom_mode=uint32(1*65536);m.Payload.system_status=uint8(3);
            m.Payload.mavlink_version=uint8(3);sendPeer(m);
            m=createmsg(dialect,'SYS_STATUS');m.Payload.onboard_control_sensors_present=uint32(7);
            m.Payload.onboard_control_sensors_enabled=uint32(7);m.Payload.onboard_control_sensors_health=uint32(3);sendPeer(m);
            m=createmsg(dialect,'EVENT');m.Payload.id=uint32(27725120);
            m.Payload.event_time_boot_ms=uint32(round((100+elapsed)*1000));
            m.Payload.sequence=uint16(mod(timerTickCount,65536));m.Payload.log_levels=uint8(34);
            m.Payload.arguments=uint8(1:40);sendPeer(m);
            m=createmsg(dialect,'EXTENDED_SYS_STATE');m.Payload.landed_state=uint8(1);
            m.Payload.vtol_state=uint8(0);sendPeer(m);
            m=createmsg(dialect,'LOCAL_POSITION_NED');m.Payload.time_boot_ms=uint32(floor((100+elapsed)*1000));
            m.Payload.x=single(1);m.Payload.y=single(2);m.Payload.z=single(-3);
            m.Payload.vx=single(0);m.Payload.vy=single(0);m.Payload.vz=single(0);sendPeer(m);
            if ~telemetryFreeze,telemetrySource=100+elapsed;end
            source=telemetrySource-double(telemetryReverse)*2;
            m=createmsg(dialect,'ATTITUDE');m.Payload.time_boot_ms=uint32(floor(source*1000));
            for name={'roll','pitch','yaw','rollspeed','pitchspeed','yawspeed'},m.Payload.(name{1})=single(0);end
            sendPeer(m);
            if emitAttitudeTarget
                m=createmsg(dialect,'ATTITUDE_TARGET');m.Payload.time_boot_ms=uint32(floor(source*1000));
                m.Payload.q=single([1,0,0,0]);m.Payload.type_mask=uint8(0);
                m.Payload.body_roll_rate=single(.125);m.Payload.body_pitch_rate=single(-.25);
                m.Payload.body_yaw_rate=single(.375);m.Payload.thrust=single(.5);sendPeer(m);
            end
            m=createmsg(dialect,'ESTIMATOR_STATUS');m.Payload.time_usec=uint64(round(source*1e6));
            m.Payload.flags=uint16(959+1024*double(estimatorErrorFlag));
            for name={'vel_ratio','pos_horiz_ratio','pos_vert_ratio','mag_ratio','pos_horiz_accuracy','pos_vert_accuracy'}
                m.Payload.(name{1})=single(.1);
            end
            m.Payload.hagl_ratio=single(NaN);m.Payload.tas_ratio=single(NaN);sendPeer(m);
            m=createmsg(dialect,'AUTOPILOT_VERSION');m.Payload.uid=uint64(42);
            m.Payload.flight_sw_version=uint32(hex2dec('011000FF'));
            m.Payload.vendor_id=uint16(65500);m.Payload.product_id=uint16(65501);
            m.Payload.flight_custom_version=uint8([1,2,3,4,5,6,7,8]);sendPeer(m);
            if ~diagFreeze,diagTime=elapsed;end
            d=zeros(1,32);d(1:7)=[double(diagFailed),double(diagFailed)*2,diagTime,0,1,0,1];
            if requireTerrainExtension
                d(26)=1;
                if diagFailed
                    d(2)=4;d(8:10)=[1,2,0];
                    nanBits=bitor(bitshift(uint64(hex2dec('7FF80000')),32),uint64(123));
                    d(25)=typecast(nanBits,'double');
                end
            end
            diagnostic=[packLittleEndian(int32([1234567890,cfg.target_system])),packLittleEndian(d)];
            floats=zeros(1,24,'single');floats(7)=1; % quaternion w
            truthBytes=[packLittleEndian(int32([123456789,cfg.target_system,5,0])),packLittleEndian(floats), ...
                packLittleEndian(double([elapsed,1,2,-3,0,0,0]))];
            nowMs=int64(floor(posixtime(datetime('now','TimeZone','UTC'))*1000));
            sentCounter=counter;if timeCounterReverse,sentCounter=counter-int64(100);end
            clockBytes=[packLittleEndian(int32([123456789,cfg.target_system])), ...
                packLittleEndian(int64([startUtcMs+originShiftMs,nowMs-timeHeartbeatDelayMs,sentCounter]))];
            write(rawSender,truthBytes,'uint8','127.0.0.1',cfg.truth_port);
            write(rawSender,diagnostic,'uint8','127.0.0.1',cfg.truth_port);
            write(rawSender,clockBytes,'uint8','127.0.0.1',cfg.coptersim_time_port);
        catch err
            if isempty(peerError)
                peerError=['EMIT:' err.identifier ':' err.message];
                peerErrorReport=getReport(err,'extended','hyperlinks','off');
            end
        end
    end
    function closeOwned()
        if ~isempty(peerTimer)
            try,stop(peerTimer);delete(peerTimer);peerTimer=[];catch err,cleanupReceipt.errors{end+1}=err.message;end
        end
        if ~isempty(io)&&~cleanupReceipt.adapter_closed
            try,cleanupReceipt.adapter_closed=io.close();catch err,cleanupReceipt.errors{end+1}=err.message;end
        end
        if ~isempty(peerSub),try,peerSub.NewMessageFcn=[];delete(peerSub);peerSub=[];catch err,cleanupReceipt.errors{end+1}=err.message;end;end
        if ~isempty(peer)&&~cleanupReceipt.peer_closed
            try,disconnect(peer);delete(peer);cleanupReceipt.peer_closed=true;catch err,cleanupReceipt.errors{end+1}=err.message;end
        end
        if ~isempty(rawSender)&&~cleanupReceipt.sender_closed
            try,delete(rawSender);cleanupReceipt.sender_closed=true;catch err,cleanupReceipt.errors{end+1}=err.message;end
        end
        if ~cleanupReceipt.ports_rebound
            try
                for q=1:numel(ports)
                    probe=udpport('datagram','IPV4','LocalHost','127.0.0.1','LocalPort',ports(q),'Timeout',0.01);
                    delete(probe);
                end
                cleanupReceipt.ports_rebound=true;
            catch err,cleanupReceipt.errors{end+1}=err.message;
            end
        end
    end
end
function value=counterValue(row)
value=double(typecast(uint8(row.bytes(25:32)),'int64'));
end
function out=packLittleEndian(v)
[~,~,endian]=computer;if endian=='B',v=swapbytes(v);end
out=reshape(typecast(v,'uint8'),1,[]);
end
function out=text16(v)
out=repmat(char(0),1,16);assert(numel(v)<=16);out(1:numel(v))=v;
end
function sha=fileSha(path)
fid=fopen(path,'rb');assert(fid>=0);guard=onCleanup(@()fclose(fid)); %#ok<NASGU>
bytes=fread(fid,Inf,'*uint8');md=java.security.MessageDigest.getInstance('SHA-256');
md.update(bytes);sha=upper(reshape(dec2hex(typecast(md.digest(),'uint8'),2).',1,[]));
end
