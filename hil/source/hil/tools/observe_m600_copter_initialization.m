function result=observe_m600_copter_initialization(outputDir,cfg,io)
% OBSERVE_M600_COPTER_INITIALIZATION Observe simulator lifecycle for up to 45 s.
% The outer owner manages COM, simulator, preflight and recovery.
% This observer uses the UDP adapter's now, sleep, snapshot, sendHeartbeat,
% evidence and close operations, including bounded TIMESYNC acquisition.
% Model faults remain latched while disarmed observation continues.
% Return to the outer owner on heartbeat loss, unexpected arming, identity change or error.

required={'write_artifacts','live_enabled','outer_preflight_pass', ...
    'poll_period_s','heartbeat_period_s','heartbeat_max_age_s', ...
    'preflight_deadline_s','expected_uid','expected_board_version', ...
    'expected_flight_custom_version_hex'};
assert(all(isfield(cfg,required)),'m600check:ObserverConfig','Missing explicit observation configuration.');
for key={'poll_period_s','heartbeat_period_s','heartbeat_max_age_s','preflight_deadline_s'}
    v=cfg.(key{1});assert(isnumeric(v)&&isscalar(v)&&isfinite(v)&&v>0, ...
        'm600check:ObserverConfig','Observation bounds must be positive and finite.');
end
assert(cfg.heartbeat_period_s==1&&cfg.heartbeat_max_age_s==3, ...
    'm600check:ObserverHeartbeatContract','Preserve existing 1 Hz / 3 s heartbeat contract.');
assert(cfg.preflight_deadline_s<=45&&cfg.poll_period_s<=cfg.heartbeat_period_s, ...
    'm600check:ObserverConfig','Acquisition and poll periods must fit the diagnostic window.');
assert(strcmp(char(cfg.expected_uid),gpenmpc_device_identity('uid'))&&cfg.expected_board_version==56, ...
    'm600check:ObserverIdentityContract','Exact expected Pixhawk identity is required.');
assert(~isempty(regexp(char(cfg.expected_flight_custom_version_hex),'^[0-9A-Fa-f]{16}$','once')), ...
    'm600check:ObserverIdentityContract','Exact eight custom-version bytes are required.');
injected=nargin>=3;
if ~injected
    assert(cfg.live_enabled&&cfg.outer_preflight_pass,'m600check:ObserverNoLivePreflight', ...
        'The sole real adapter can only be constructed after explicit outer live preflight.');
    io=[];
end
if cfg.write_artifacts
    assert(~isfolder(outputDir)&&~isfile(outputDir),'m600check:ObserverOutputExists', ...
        'Refusing to reuse an observation output path.');
    mkdir(outputDir);
end
duration=45;timerStart=NaN;lastNow=NaN;lastHeartbeat=-Inf;
snapshots={};rows=repmat(rowTemplate(),0,1);events=repmat(eventTemplate(),0,1);
firstFatal=[];failure='';stopReason='';transport=[];latest=[];initialIdentity=[];
everFreshHeartbeat=false;completed=false;finalized=false;socketsClosed=false;
closeAttempted=false;evidenceFailure='';closeFailure='';adapterCreations=0;
heartbeatAttempts=0;heartbeatSent=0;
lastSource=struct('estimate',NaN,'truth',NaN,'model',NaN,'raw_model',NaN,'attitude',NaN);
guard=onCleanup(@finish); %#ok<NASGU>
try
    if ~injected,adapterCreations=adapterCreations+1;io=m600check.makeM600CopterSimIo(cfg);end
    assert(isstruct(io)&&all(isfield(io,{'now','sleep','snapshot','sendHeartbeat','evidence','close'})), ...
        'm600check:ObserverIoContract','Observation adapter interface is incomplete.');
    timerStart=io.now();lastNow=timerStart;
    assert(isscalar(timerStart)&&isfinite(timerStart),'m600check:ObserverClock','Host clock unavailable.');
    while true
        now=io.now();
        assert(isscalar(now)&&isfinite(now)&&now>=lastNow,'m600check:ObserverClock','Host clock reversed or invalid.');
        lastNow=now;
        if now-timerStart>=duration,completed=true;stopReason='OBSERVATION_WINDOW_ENDED';break;end
        latest=io.snapshot();now=io.now();lastNow=now;
        snapshots{end+1}=latest; %#ok<AGROW>
        r=inspect(latest,now);rows(end+1,1)=r; %#ok<AGROW>
        if ~isempty(r.fatal),latch('ADAPTER_FATAL',r.fatal,now,fieldOr(latest,'first_fatal',[]));end
        if r.estimate_source_reversed,latch('PX4_SOURCE_REVERSED','Estimate source time reversed.',now,[]);end
        if r.attitude_source_reversed,latch('PX4_ATTITUDE_SOURCE_REVERSED','ATTITUDE source time reversed.',now,[]);end
        if r.truth_source_reversed,latch('COPTER_SOURCE_REVERSED','Truth source time reversed.',now,[]);end
        if r.model_source_reversed,latch('MODEL_SOURCE_REVERSED','Diagnostic source time reversed.',now,[]);end
        if r.raw_model_source_reversed,latch('RAW_MODEL_SOURCE_REVERSED','Decoded diagnostic time reversed; accepted observer time is separate.',now,[]);end
        if r.model_fault,latch('MODEL_FAULT',r.model_reason,now,[]);end
        if r.identity_present&&~r.identity_matches_expected
            stopReason='IDENTITY_MISMATCH_RETURN_TO_OUTER';latch('IDENTITY',stopReason,now,[]);break
        end
        if r.identity_changed
            stopReason='IDENTITY_CHANGED_RETURN_TO_OUTER';latch('IDENTITY',stopReason,now,[]);break
        end
        if r.heartbeat_fresh
            everFreshHeartbeat=true;
            if latest.armed~=0
                stopReason='UNEXPECTED_ARM_STATE_RETURN_TO_OUTER';latch('ARM_STATE',stopReason,now,[]);break
            end
        elseif everFreshHeartbeat
            stopReason='HEARTBEAT_STALE_RETURN_TO_OUTER';latch('HEARTBEAT',stopReason,now,[]);break
        elseif now-timerStart>=cfg.preflight_deadline_s
            stopReason='INITIAL_HEARTBEAT_ACQUISITION_TIMEOUT';latch('HEARTBEAT',stopReason,now,[]);break
        end
        if now-lastHeartbeat>=cfg.heartbeat_period_s
            heartbeatAttempts=heartbeatAttempts+1;io.sendHeartbeat();
            heartbeatSent=heartbeatSent+1;lastHeartbeat=io.now();
        end
        io.sleep(min(cfg.poll_period_s,max(0,duration-(io.now()-timerStart))));
    end
catch problem
    failure=[problem.identifier ': ' problem.message];stopReason='OBSERVER_EXCEPTION_RETURN_TO_OUTER';
    latch('OBSERVER_EXCEPTION',failure,safeNow(),[]);
end
finish();clear guard
elapsed=NaN;if ~isempty(rows)&&isfinite(timerStart),elapsed=rows(end).host_time_s-timerStart;end
if completed,elapsed=duration;end
status='OBSERVATION_ENDED_WITH_RECORDED_FAULTS';
if completed&&isempty(firstFatal)&&isempty(failure)&&socketsClosed
    status='OBSERVATION_WINDOW_RECORDED';
elseif ~completed||~socketsClosed,status='OBSERVATION_STOPPED__OUTER_SAFETY_REQUIRED';end
counts=struct('adapter_creations',adapterCreations,'heartbeat_attempts',heartbeatAttempts, ...
    'heartbeat_sent',heartbeatSent,'parameter_read_requests',0,'parameter_writes',0, ...
    'mapping_writes',0,'mode_requests',0,'arm_requests',0,'disarm_requests',0, ...
    'land_requests',0,'task_requests',0,'setpoint_requests',0,'reboot_requests',0, ...
    'COM_open',0,'plant_creations',0,'physical_output_actions',0);
result=struct('schema','M600_COPTER_INITIALIZATION_OBSERVATION_V1','status',status, ...
    'duration_contract_s',duration,'duration_provenance','PROSPECTIVE_HOST_LIFECYCLE_DIAGNOSTIC_WINDOW', ...
    'window_completed',completed,'elapsed_observation_s',elapsed,'stop_reason',stopReason, ...
    'failure',failure,'first_fatal',firstFatal,'events',events,'rows',rows, ...
    'snapshots',{snapshots},'transport_evidence',transport,'counts',counts, ...
    'initial_observed_identity',initialIdentity,'last_snapshot',latest, ...
    'ever_fresh_heartbeat',everFreshHeartbeat,'close_attempted',closeAttempted, ...
    'matlab_sockets_closed',socketsClosed,'evidence_capture_failure',evidenceFailure, ...
    'close_failure',closeFailure,'injected_io',injected,'outer_final_safety_still_required',true, ...
    'formal_rows',0,'flight_result',false,'initialization_pass_claim',false, ...
    'source_reset_performed',false,'fault_latch_cleared',false,'config',cfg);
if cfg.write_artifacts
    save(fullfile(outputDir,'RAW_INITIALIZATION_OBSERVATION.mat'),'result','-v7.3');
    if ~isempty(rows),writetable(struct2table(rows),fullfile(outputDir,'OBSERVATION_ROWS.csv'));end
    compact=result;compact.snapshots={};compact.transport_evidence=[];compact.rows=[];compact.last_snapshot=[];
    fid=fopen(fullfile(outputDir,'RESULT.json'),'w','n','UTF-8');assert(fid>=0);
    f=onCleanup(@()fclose(fid));fprintf(fid,'%s\n',jsonencode(compact,PrettyPrint=true));clear f
end

    function r=inspect(s,now)
        r=rowTemplate();r.host_time_s=now;r.elapsed_s=now-timerStart;
        assert(isstruct(s)&&all(isfield(s,{'armed','heartbeat_rx_s','clock_valid'})), ...
            'm600check:ObserverSnapshotContract','Snapshot missing essential fields.');
        r.armed=double(s.armed);r.landed_state=double(fieldOr(s,'landed_state',NaN));
        r.heartbeat_age_s=now-double(s.heartbeat_rx_s);
        r.heartbeat_fresh=isfinite(r.heartbeat_age_s)&&r.heartbeat_age_s>=0&& ...
            r.heartbeat_age_s<=cfg.heartbeat_max_age_s&&ismember(r.armed,[0,1]);
        r.clock_valid=logical(s.clock_valid);r.clock_uncertainty_s=fieldOr(s,'clock_uncertainty_s',NaN);
        r.clock_utc_drift_s=fieldOr(s,'clock_utc_drift_s',NaN);r.fatal=char(fieldOr(s,'fatal',''));
        r.model_ready=logical(fieldOr(s,'model_ready',false));d=fieldOr(s,'model_diagnostic',[]);
        if isstruct(d)&&~isempty(d)
            r.model_reason=char(fieldOr(d,'status',''));r.model_source_s=fieldOr(d,'last_source_time_s',NaN);
            r.model_fault=~isempty(fieldOr(d,'fatal_reason',''));
            decoded=fieldOr(d,'decoded',[]);
            if isstruct(decoded)&&~isempty(decoded),r.raw_model_source_s=fieldOr(decoded,'sim_time_s',NaN);end
        end
        attitude=fieldOr(s,'attitude',[]);
        if isstruct(attitude)&&~isempty(attitude)&&isfield(attitude,'time_boot_ms')
            r.attitude_source_s=double(attitude.time_boot_ms)/1000;
        end
        for name={'estimate','truth'}
            v=fieldOr(s,name{1},[]);
            if isstruct(v)&&~isempty(v),r.([name{1} '_source_s'])=fieldOr(v,'raw_source_time_s',NaN);end
        end
        for name={'estimate','truth','model','raw_model','attitude'}
            n=name{1};t=r.([n '_source_s']);r.([n '_source_reversed'])=isfinite(t)&&isfinite(lastSource.(n))&&t<lastSource.(n);
            if isfinite(t),lastSource.(n)=t;end
        end
        v=fieldOr(s,'autopilot_version',[]);
        if isstruct(v)&&~isempty(v)&&all(isfield(v,{'uid','board_version','flight_custom_version'}))
            r.identity_present=true;
            id=struct('uid',sprintf('%u',uint64(v.uid)),'board_version',double(v.board_version), ...
                'flight_custom_version_hex',upper(reshape(dec2hex(uint8(v.flight_custom_version(:)),2).',1,[])));
            r.identity_matches_expected=strcmp(id.uid,char(cfg.expected_uid))&&id.board_version==cfg.expected_board_version&& ...
                strcmpi(id.flight_custom_version_hex,char(cfg.expected_flight_custom_version_hex));
            if isempty(initialIdentity),initialIdentity=id;else,r.identity_changed=~isequal(initialIdentity,id);end
        end
    end
    function latch(domain,reason,time,adapterFirst)
        events(end+1,1)=struct('time_s',time,'domain',domain,'reason',reason); %#ok<AGROW>
        if isempty(firstFatal)
            firstFatal=struct('time_s',time,'domain',domain,'reason',reason,'adapter_first_fatal',adapterFirst);
        end
    end
    function t=safeNow()
        t=lastNow;
        if ~isempty(io),try,t=io.now();catch,end;end
    end
    function finish()
        if finalized,return;end
        finalized=true;
        if isempty(io),return;end
        try,transport=io.evidence();catch problem,evidenceFailure=[problem.identifier ': ' problem.message];end
        closeAttempted=true;
        try
            value=io.close();socketsClosed=islogical(value)&&isscalar(value)&&value;
            if ~socketsClosed,closeFailure='Adapter close did not explicitly confirm true.';end
        catch problem,closeFailure=[problem.identifier ': ' problem.message];end
        if ~isempty(evidenceFailure),latch('EVIDENCE_CAPTURE',evidenceFailure,safeNow(),[]);end
        if ~socketsClosed,latch('SOCKET_CLOSE',closeFailure,safeNow(),[]);end
    end
end
function value=fieldOr(s,name,fallback)
if isstruct(s)&&isscalar(s)&&isfield(s,name),value=s.(name);else,value=fallback;end
end
function r=rowTemplate()
r=struct('host_time_s',NaN,'elapsed_s',NaN,'heartbeat_age_s',Inf,'heartbeat_fresh',false, ...
    'armed',NaN,'landed_state',NaN,'clock_valid',false,'clock_uncertainty_s',NaN, ...
    'clock_utc_drift_s',NaN,'model_ready',false,'model_fault',false,'model_reason','', ...
    'estimate_source_s',NaN,'truth_source_s',NaN,'model_source_s',NaN,'raw_model_source_s',NaN,'attitude_source_s',NaN, ...
    'estimate_source_reversed',false,'truth_source_reversed',false,'model_source_reversed',false, ...
    'raw_model_source_reversed',false,'attitude_source_reversed',false, ...
    'identity_present',false,'identity_matches_expected',false,'identity_changed',false,'fatal','');
end
function e=eventTemplate(),e=struct('time_s',NaN,'domain','','reason','');end
