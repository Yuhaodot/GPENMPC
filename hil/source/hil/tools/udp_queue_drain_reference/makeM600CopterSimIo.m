function io = makeM600CopterSimIo(cfg,operation)
%MAKEM600COPTERSIMIO MATLAB MAVLink/CopterSim observation adapter.
% OPEN creates UDP sockets. VALIDATE_ONLY checks configuration and dependencies.
% Required cfg fields: local_mavlink_port, remote_mavlink_port, truth_port,
% coptersim_time_port, target_system, target_component, clock_max_rtt_s,
% clock_sync_samples, clock_sync_period_s, clock_max_age_s,
% clock_max_uncertainty_s, clock_max_utc_drift_s, clock_max_time_heartbeat_age_s,
% clock_max_time_heartbeat_lag_s, maximum_truth_lag_s, maximum_raw_records,
% state_max_age_s, including model-diagnostic receive and source-progress age.
%
% PX4 TIMESYNC replies carry board HRT in tc1 and the request token in ts1.
% The offset lies in [send-board,receive-board]; the best-RTT midpoint is
% fixed before arm and validated by fresh token-matched responses.
% CopterSim RflyTimeStmp on UDP20005 uses 2i3q with magic 123456789, vehicle
% ID, UTC-millisecond SysStartTime/SysCurrentTime and count.
% Truth time is SysStartTime + DLL simTime, mapped through a measured
% UTC/monotonic pair with millisecond quantization and acquisition uncertainty.
% Origin changes, stale mappings, UTC drift and excessive lag invalidate the
% clock. Midpoint alignment is approximate within the reported interval.
if nargin<2,operation='OPEN';end
assert((ischar(operation)&&isrow(operation))||(isstring(operation)&&isscalar(operation)), ...
    'm600check:AdapterOperation','Expected OPEN or VALIDATE_ONLY.');
operation=char(operation);
assert(ismember(string(operation),["OPEN","VALIDATE_ONLY"]), ...
    'm600check:AdapterOperation','Unknown adapter operation.');
validateOnly=strcmp(operation,'VALIDATE_ONLY');

required={'local_mavlink_port','remote_mavlink_port','truth_port','coptersim_time_port', ...
    'target_system','target_component','clock_max_rtt_s','clock_sync_samples', ...
    'clock_sync_period_s','clock_max_age_s','clock_max_uncertainty_s', ...
    'clock_max_utc_drift_s','clock_max_time_heartbeat_age_s', ...
    'clock_max_time_heartbeat_lag_s','maximum_truth_lag_s','maximum_raw_records','state_max_age_s'};
assert(all(isfield(cfg,required)),'m600check:AdapterConfig','Explicit adapter and clock bounds are required.');
for i=1:numel(required)
    v=cfg.(required{i});
    assert(isnumeric(v)&&isreal(v)&&isscalar(v)&&isfinite(v)&&v>0, ...
        'm600check:AdapterConfig','%s must be positive and finite.',required{i});
end
for name={'local_mavlink_port','remote_mavlink_port','truth_port','coptersim_time_port'}
    v=cfg.(name{1});assert(v==fix(v)&&v<=65535,'m600check:AdapterConfig','Invalid UDP port.');
end
assert(cfg.clock_sync_samples==fix(cfg.clock_sync_samples)&& ...
    cfg.maximum_raw_records==fix(cfg.maximum_raw_records), ...
    'm600check:AdapterConfig','Sample and memory bounds must be integers.');
assert(all(isfield(cfg,{'live_enabled','outer_preflight_pass'})), ...
    'm600check:AdapterConfig','Explicit live and outer-preflight fields are required.');
if ~validateOnly
    assert(cfg.live_enabled&&cfg.outer_preflight_pass,'m600check:AdapterNotAuthorized','Outer preflight required.');
end
% Replace the sole MAVLink transport using the CopterSim localhost endpoint.
rawTransportEnabled=isfield(cfg,'mavlink_transport');
transportKind='ORIGINAL_MAVLINKIO_UDP';rawTransport=[];rawTransportFinal=[];rawDraining=false;
if rawTransportEnabled
    c=cfg.mavlink_transport;
    assert(isstruct(c)&&isscalar(c)&&ismember(string(c.scope),["HOST_ONLY_LOOPBACK","COPTERSIM_EXISTING_MAVLINK"]) ...
        &&isequal(c.allow_loopback,true)&&c.local_port==cfg.local_mavlink_port ...
        &&c.remote_port==cfg.remote_mavlink_port&&isequal(c.local_system,uint8(255)) ...
        &&isequal(c.local_component,uint8(190))&&isequal(c.remote_system,uint8(cfg.target_system)) ...
        &&isequal(c.remote_component,uint8(cfg.target_component)), ...
        'm600check:RawTransportConfig','Exact same-endpoint replacement required.');
    transportKind='OFFICIAL_CODEC_SOLE_RAW_UDP';
end
if ~isfield(cfg,'require_terrain_diagnostic_extension'),cfg.require_terrain_diagnostic_extension=false;end
assert(islogical(cfg.require_terrain_diagnostic_extension)&&isscalar(cfg.require_terrain_diagnostic_extension), ...
    'm600check:TerrainObserverContract','The terrain extension flag must be scalar logical.');
assert(~cfg.require_terrain_diagnostic_extension||~isempty(which('m600check.decodeCopterSimTerrainDiagnostics')), ...
    'm600check:TerrainDiagnosticDependencyMissing','The selected terrain diagnostic decoder is unavailable.');
deliveryEnabled=isfield(cfg,'delivery_environment_contract');deliveryContract=[];deliveryPolicy=[];
if deliveryEnabled
    deliveryContract=cfg.delivery_environment_contract;
    fields={'expected_session_token','initial_payload_kg','payload_by_generation_kg','mass_by_generation_kg','remote_port'};
    assert(isstruct(deliveryContract)&&isscalar(deliveryContract)&&all(isfield(deliveryContract,fields)), ...
        'm600check:DeliveryAdapterContract','Complete delivery contract required before socket creation.');
    assert(deliveryContract.remote_port==fix(deliveryContract.remote_port)&&deliveryContract.remote_port>0&&deliveryContract.remote_port<=65535, ...
        'm600check:DeliveryAdapterContract','Invalid exact CopterSim environment port.');
    deliveryPolicy=struct('expected_session_token',deliveryContract.expected_session_token, ...
        'payload_by_generation_kg',deliveryContract.payload_by_generation_kg, ...
        'mass_by_generation_kg',deliveryContract.mass_by_generation_kg, ...
        'allow_unbound_pre_session',true);
    assert(~isempty(which('m600check.updateCopterSimDeliveryDiagnostic'))&& ...
        ~isempty(which('m600check.encodeCopterSimDeliveryDiagnostics'))&& ...
        ~isempty(which('gpenmpcTaskIo.encodePlantEnvironmentV2')), ...
        'm600check:DeliveryDiagnosticDependencyMissing','The selected delivery diagnostic or environment codec dependency is unavailable.');
end
nativeHoverTuning=[];tuningValidation=[];
canonicalExchangeEnabled=isfield(cfg,'canonical_exchange');canonicalExchange=[];
canonicalLocalMode=false;canonicalChannels=["snapshot","feedback"];
canonicalExchangeExpected=[];canonicalExchangeOrigin=[];canonicalExchangeBindAttempted=false;
canonicalExchangeFailure='';canonicalExchangeUnboundReceived=uint64(0);
canonicalExchangeSendInProgress=false;canonicalExchangeSent=uint64(0);
canonicalCompletedCapacity=0;canonicalCompleted={cell(0,1),cell(0,1)};
canonicalCompletedEnqueued=zeros(1,2,'uint64');canonicalCompletedTaken=zeros(1,2,'uint64');
canonicalSessionLastSend=[];
canonicalLocalInputLastSource=uint64(0);
canonicalLocalWindow=[];
canonicalExchangeAssociation=[];
canonicalSessionHistory={};canonicalRetirementAttempts={};canonicalSessionHistoryCapacity=5;
canonicalRotorEnabled=isfield(cfg,'canonical_rotor_observer');canonicalRotorSocket=[];
canonicalRotorQueue={};canonicalRotorReceived=uint64(0);canonicalRotorFailure='';
canonicalCacheQueue={};canonicalCacheReceived=uint64(0);canonicalCacheState=[];canonicalCacheSample=[];
canonicalCacheEnabled=isfield(cfg,'canonical_control_cache_observer');
if canonicalCacheEnabled
    assert(canonicalRotorEnabled,'m600check:CanonicalCacheSharedSocket','Cache observation must use the same existing rotor socket.');
    cacheExpected=cfg.canonical_control_cache_observer;
    cacheInitialClock=struct('now_ns',uint64(1),'now_wall_time_s',0,'now_sim_time_s',NaN);
    [~,cacheInitialState]=m600check.decodeCanonicalControlCache([],cacheExpected,cacheInitialClock,[]);
    assert(~cacheInitialState.failed,'m600check:CanonicalCacheConfig','Exact cache identity and original age bounds required.');
end
if canonicalRotorEnabled
    c=cfg.canonical_rotor_observer;
    assert(isstruct(c)&&isscalar(c)&&all(isfield(c,{'local_port','maximum_queue'})) ...
        &&isnumeric(c.local_port)&&isscalar(c.local_port)&&isfinite(c.local_port) ...
        &&c.local_port==fix(c.local_port)&&c.local_port>0&&c.local_port<=65535 ...
        &&~ismember(c.local_port,[cfg.local_mavlink_port,cfg.remote_mavlink_port,cfg.truth_port,cfg.coptersim_time_port]) ...
        &&isnumeric(c.maximum_queue)&&isscalar(c.maximum_queue)&&isfinite(c.maximum_queue) ...
        &&c.maximum_queue==fix(c.maximum_queue)&&c.maximum_queue>0 ...
        &&c.maximum_queue<=cfg.maximum_raw_records, ...
        'm600check:CanonicalRotorConfig','Distinct localhost observer port and explicit bounded queue required.');
    assert(~isempty(which('gpenmpcNative.rflyOriginalHostMonotonicNs')), ...
        'm600check:CanonicalRotorDependency','Original monotonic receiver clock required.');
end
if canonicalExchangeEnabled
    c=cfg.canonical_exchange;
    assert(isstruct(c)&&isscalar(c)&&isfield(c,'assembly_limit_ns') ...
        &&isa(c.assembly_limit_ns,'uint64')&&isscalar(c.assembly_limit_ns)&&c.assembly_limit_ns>0, ...
        'm600check:CanonicalExchangeConfig','Explicit HOST assembly resource bound required.');
    if isfield(c,'runtime')
        assert(ismember(string(c.runtime),["LEGACY_HOST_INNER","BOARD_LOCAL_FULL_INNER"]), ...
            'm600check:CanonicalRuntime','Unknown canonical runtime cannot silently use legacy exchange.');
        canonicalLocalMode=string(c.runtime)=="BOARD_LOCAL_FULL_INNER";
    end
    if canonicalLocalMode
        if isfield(c,'require_learning_audit')
            assert(islogical(c.require_learning_audit)&&isscalar(c.require_learning_audit), ...
                'm600check:CanonicalLearningAuditConfig','Explicit scalar logical required.');
        end
        canonicalChannels=["gp_request","snapshot","committed_state"];
        canonicalCompleted={cell(0,1),cell(0,1),cell(0,1)};
        canonicalCompletedEnqueued=zeros(1,3,'uint64');canonicalCompletedTaken=zeros(1,3,'uint64');
        assert(isfield(c,'gp_reply_host_max_age_ns')&&isa(c.gp_reply_host_max_age_ns,'uint64') ...
            &&isscalar(c.gp_reply_host_max_age_ns)&&c.gp_reply_host_max_age_ns>0 ...
            &&~isempty(which('gpenmpcNative.RflyLocalTunnelReassembler')) ...
            &&~isempty(which('gpenmpcNative.validateLocalGpReplyForSend')), ...
            'm600check:CanonicalLocalRuntime','Explicit reply transport bound and local dependencies required.');
    end
    if isfield(c,'completed_queue_capacity')
        v=c.completed_queue_capacity;
        assert(isnumeric(v)&&isreal(v)&&isscalar(v)&&isfinite(v)&&v==fix(v) ...
            &&v>0&&v<=cfg.maximum_raw_records,'m600check:CanonicalCompletedQueueConfig', ...
            'Explicit combined completed-message memory bound required before socket creation.');
        canonicalCompletedCapacity=double(v);
    end
    if isfield(c,'maximum_session_history')
        v=c.maximum_session_history;
        assert(isnumeric(v)&&isscalar(v)&&isfinite(v)&&v==fix(v)&&v>0&&v<=cfg.maximum_raw_records, ...
            'm600check:CanonicalSessionHistoryBound','Explicit non-evicting session history resource bound required.');
        canonicalSessionHistoryCapacity=double(v);
    end
    assert(~isempty(which('gpenmpcNative.RflyTunnelReassembler')) ...
        &&~isempty(which('gpenmpcNative.rflyOriginalHostMonotonicNs')) ...
        &&~isempty(which('gpenmpcNative.RflySessionCommandEncoder')) ...
        &&~isempty(which('m600check.decodeCanonicalExchangePackets')), ...
        'm600check:CanonicalExchangeDependency','Canonical exchange dependencies are required before socket creation.');
end
canonicalObserverEnabled=isfield(cfg,'canonical_px4_observation');
canonicalObserver=[];
canonicalModuleEnabled=isfield(cfg,'canonical_se3_status_observation');
assert(~(canonicalExchangeEnabled&&canonicalModuleEnabled), ...
    'm600check:CanonicalSerialObserverConflict', ...
    'Canonical session replies and the old SE3 shell-status observer cannot both own SERIAL_CONTROL.');
canonicalModule=[];
canonicalModuleSendInProgress=false;canonicalModuleDeferred={};canonicalModuleDeferredBytes=0;
canonicalModuleDeferredTotal=0;canonicalModuleDeferredProcessed=0;
if canonicalModuleEnabled
    assert(~isempty(which('gpenmpcNative.advanceSe3StatusObservation')), ...
        'm600check:CanonicalModuleObserverMissing','The selected SE3 module observer helper is unavailable.');
    c=cfg.canonical_se3_status_observation;
    assert(isstruct(c)&&isscalar(c)&&isfield(c,'verified_binding') ...
        &&c.verified_binding.system_id==cfg.target_system ...
        &&c.verified_binding.component_id==cfg.target_component, ...
        'm600check:CanonicalModuleObserverConfig','Actual module observer source system/component binding is required.');
    if canonicalObserverEnabled
        assert(isequal(c.verified_binding,cfg.canonical_px4_observation.verified_binding), ...
            'm600check:CanonicalModuleBindingMismatch','Module and atomic observation bindings must be identical.');
    end
end
if canonicalObserverEnabled
    % The method backend requires this optional receive path.
    assert(~isempty(which('m600check.advanceCanonicalPx4Observation')), ...
        'm600check:CanonicalObserverMissing','The selected atomic PX4 observation helper is unavailable.');
    c=cfg.canonical_px4_observation;
    assert(isstruct(c)&&isscalar(c)&&all(isfield(c, ...
        {'verified_binding','maximum_queue','maximum_age_ns'})), ...
        'm600check:CanonicalObserverConfig','Complete explicit atomic observer configuration is required.');
    assert(c.verified_binding.system_id==cfg.target_system ...
        &&c.verified_binding.component_id==cfg.target_component, ...
        'm600check:CanonicalObserverSourceMismatch','Atomic observer source must match this IO target.');
end
if isfield(cfg,'native_hover_tuning')||(canonicalLocalMode&&isfield(cfg,'temporary_allocator_geometry'))
    if canonicalLocalMode
        % Load native LAND recovery values from the recorded plan;
        % the full-inner controller uses its own mathematical configuration.
        assert(isfield(cfg,'temporary_allocator_geometry')&&isfield(cfg,'native_hover_tuning') ...
            &&~isempty(which('load_m600_recovery_contracts')), ...
            'm600check:LocalRecoveryContractSource','The bound recovery loader and both contracts are required before transport creation.');
        [originalRecovery,recoverySource]=load_m600_recovery_contracts();
        assert(isequaln(cfg.native_hover_tuning,originalRecovery.native_hover_tuning) ...
            &&isequaln(cfg.temporary_allocator_geometry,originalRecovery.temporary_allocator_geometry), ...
            'm600check:LocalRecoveryContractMismatch','Both recovery contracts must match the SHA-bound recovery plan.');
        tuningValidation=struct('passed',true,'failure','', ...
            'source',recoverySource,'controller_configuration_validated_here',false, ...
            'scope','REFERENCE_APPLICATION_NATIVE_LAND_RECOVERY_VALUES_NOT_LEGACY_CONTROL_SELECTION');
    elseif isfield(cfg.native_hover_tuning,'schema')&& ...
            strcmp(cfg.native_hover_tuning.schema,'TEMPORARY_M600_CANONICAL_DELIVERY_NATIVE_TUNING_V1')
        assert(~isempty(which('validate_m600_delivery_native_tuning_contract')), ...
            'm600check:DeliveryTuningValidatorMissing','The selected legacy delivery tuning validator is unavailable.');
        tuningValidation=validate_m600_delivery_native_tuning_contract(cfg.native_hover_tuning);
    else
        tuningValidation=m600check.validateNativeHoverTuningContract(cfg.native_hover_tuning);
    end
    assert(tuningValidation.passed,'m600check:NativeHoverTuningContract','%s',tuningValidation.failure);
    nativeHoverTuning=cfg.native_hover_tuning;
end
% Resolve the pure dependencies before opening any transport endpoint.
assert(~isempty(which('m600check.decodeCopterSimDiagnostics'))&& ...
    ~isempty(which('m600check.updateCopterSimDiagnostic'))&& ...
    ~isempty(which('m600check.copterSimDiagnosticSnapshot'))&& ...
    ~isempty(which('m600check.sampleUtcMonotonicPair'))&& ...
    ~isempty(which('m600check.evaluateUtcDrift'))&& ...
    ~isempty(which('m600check.advanceFixedTimesyncValidation'))&& ...
    ~isempty(which('m600check.evaluateNativeHoverTelemetry')), ...
    'm600check:DiagnosticDependencyMissing','Add both MATLAB validation package roots before live I/O.');
if validateOnly
    io=struct('schema','M600_COPTERSIM_IO_PURE_CONFIGURATION_CHECK_V1', ...
        'passed',true,'operation','VALIDATE_ONLY','board_local_full_inner',logical(canonicalLocalMode), ...
        'transport_kind',transportKind,'native_hover_tuning_validation',tuningValidation, ...
        'udp_socket_opens',0,'COM_opens',0,'board_actions',0,'plant_actions',0, ...
        'clock_observed',false,'live_authorization_granted',false, ...
        'scope','PURE_CONFIG_AND_DEPENDENCY_GUARDS_ONLY_NOT_TRANSPORT_OR_RUNTIME_SUCCESS');
    return
end
timer=tic;
if canonicalModuleEnabled
    init=cfg.canonical_se3_status_observation;init.now_ns=round(toc(timer)*1e9);
    [canonicalModule,~]=gpenmpcNative.advanceSe3StatusObservation([],'INIT',init);
end
if canonicalObserverEnabled
    init=cfg.canonical_px4_observation;init.now_ns=round(toc(timer)*1e9);
    [canonicalObserver,~]=m600check.advanceCanonicalPx4Observation( ...
        [],'INIT',init);
end
% Take ten clock samples before socket creation to account for initial
% datetime setup separately from per-source timestamps.
utcPair=m600check.sampleUtcMonotonicPair(@()toc(timer), ...
    @()posixtime(datetime('now','TimeZone','UTC')),10,cfg.clock_max_utc_drift_s);
utcZero=utcPair.utc_zero_s;utcPairUncertainty=utcPair.uncertainty_s;
link=[];truthSocket=[];timeSocket=[];subscriptions={};closed=false;fatal='';nonModelFatal='';
firstFatal=[];modelDiagnostic=[];deliveryDiagnostic=[];deliveryLastDecoded=[];deliveryLastReceive=-Inf;
rawMav={};rawTruth={};rawTime={};rawTx={};rawEnvironmentTx={};rawRotor={};rawCache={};
rawForwardingFragments={};
canonicalEnvironmentQueue={};canonicalEnvironmentOverflow=false;
deliveryContinuityEpoch=1;deliveryGroundState=[];
deliveryTransitions=repmat(struct('time_s',0,'ground_disarmed_fresh',false,'armed',NaN,'landed_state',NaN,'epoch',0),0,1);
lastHeartbeatSent=-Inf;
rawDropped=struct('MAV',0,'TRUTH',0,'TIME',0,'TX',0,'ENV_TX',0,'ROTOR',0,'CACHE',0);
hb=[];extended=[];estimate=[];truth=[];attitude=[];actuator=[];autopilotVersion=[];
systemStatus=[];systemStatusRx=-Inf;
localObserver=emptyObserver();attitudeObserver=emptyObserver();estimatorObserver=emptyObserver();
autopilotVersionRx=-Inf;
hbRx=-Inf;extRx=-Inf;estimateRx=-Inf;truthRx=-Inf;
acks=repmat(struct('command',0,'result',0,'received_at_s',0,'payload',struct()),0,1);
parameters=repmat(struct('name','','received_at_s',0,'payload',struct()),0,1);
realParameterActions=repmat(struct('name','','original_raw_bits_hex','','target_raw_bits_hex','', ...
    'expected_current_raw_bits_hex','','before_send_guard_invoked',false,'before_send_guard_passed',false, ...
    'attempted',false,'send_returned',false,'ack_verified',false,'readback_verified',false,'error',''),0,1);
syncSent=repmat(struct('token',int64(0),'time_s',0),0,1);
syncResponses=repmat(struct('token',int64(0),'send_s',0,'receive_s',0, ...
    'board_time_s',0,'rtt_s',0,'offset_lower_s',0,'offset_upper_s',0,'accepted',false),0,1);
best=[];syncLocked=false;lastSync=-Inf;startUtc=NaN;timeHeartbeatRx=-Inf;
clockValidation=[];clockValidationReceipt=[];clockValidationResponses={};
clockValidationConfig=cfg;
clockValidationConfig.clock_fixed_extra_uncertainty_s=utcPairUncertainty+0.0005;
timeHeartbeatLag=Inf;timeHeartbeatCurrentUtc=NaN;timeCounter=-Inf;timeOriginChanged=false;
timeStartupUninitializedCount=0;
try
    % Extend the dialect with EVENT messages 410--413 while preserving
    % the parent control-message definitions.
    healthDialectPath=fullfile(fileparts(mfilename('fullpath')),'px4_health_events.xml');
    if canonicalLocalMode&&isfield(cfg,'local_mavlink_dialect')
        localDialectBinding=cfg.local_mavlink_dialect;
        assert(strcmpi(m600check.fileSha256(localDialectBinding.path),localDialectBinding.sha256) ...
            &&strcmpi(m600check.fileSha256(fullfile(fileparts(localDialectBinding.path),'common_local_full_inner.xml')),localDialectBinding.common_sha256), ...
            'm600check:LocalDialectIdentity','Exact PX4 telemetry definition derivative required before open.');
        healthDialectPath=localDialectBinding.path;
    end
    dialect=mavlinkdialect(healthDialectPath);
    % Prepare the public message shape once for high-rate fragment exchange.
    canonicalTunnelTemplate=[];
    if canonicalExchangeEnabled,canonicalTunnelTemplate=createmsg(dialect,'TUNNEL');end
    link=mavlinkio(dialect,'SystemID',255,'ComponentID',190);
    if rawTransportEnabled
        rawTransport=gpenmpcNative.RflySoleMavlinkTransport(cfg.mavlink_transport,dialect,link);
    else
        connect(link,'UDP','LocalPort',cfg.local_mavlink_port);
        client=mavlinkclient(link,cfg.target_system,cfg.target_component);
    end
    % ATTITUDE_TARGET is optional. Missing observations remain unavailable.
    topics={'HEARTBEAT','EXTENDED_SYS_STATE','LOCAL_POSITION_NED','ATTITUDE','ATTITUDE_TARGET', ...
        'ESTIMATOR_STATUS','HIL_ACTUATOR_CONTROLS','STATUSTEXT','COMMAND_ACK','PARAM_VALUE','TIMESYNC', ...
        'AUTOPILOT_VERSION','SYSTEM_TIME','HIGHRES_IMU','GPS_RAW_INT','VIBRATION', ...
        'SYS_STATUS','EVENT','CURRENT_EVENT_SEQUENCE','RESPONSE_EVENT_ERROR'};
    if canonicalObserverEnabled,topics{end+1}='ODOMETRY';end
    if canonicalModuleEnabled||canonicalExchangeEnabled,topics{end+1}='SERIAL_CONTROL';end
    if canonicalExchangeEnabled,topics{end+1}='TUNNEL';end
    topicIds=zeros(1,numel(topics),'uint32');
    if rawTransportEnabled
        for i=1:numel(topics),shape=createmsg(dialect,topics{i});topicIds(i)=uint32(shape.MsgID);end
    end
    if ~rawTransportEnabled
        for i=1:numel(topics)
            name=topics{i};
            subscriptions{end+1}=mavlinksub(link,client,name,'BufferSize',200, ...
                'NewMessageFcn',@(~,msg)onMav(name,msg)); %#ok<AGROW>
        end
    end
    truthSocket=udpport('datagram','IPV4','LocalPort',cfg.truth_port,'Timeout',0.01);
    timeSocket=udpport('datagram','IPV4','LocalPort',cfg.coptersim_time_port,'Timeout',0.01);
    % Subscribe using the MATLAB multicast group-address API.
    configureMulticast(timeSocket,'224.0.0.10');
    if canonicalRotorEnabled
        % Receive the DLL observer through its configured endpoint.
        canonicalRotorSocket=udpport('datagram','IPV4','LocalHost','127.0.0.1', ...
            'LocalPort',cfg.canonical_rotor_observer.local_port,'Timeout',0.01);
    end
catch problem
    closeAll();rethrow(problem);
end
io=struct('now',@nowS,'sleep',@(s)pause(s),'snapshot',@snapshot, ...
    'sendHeartbeat',@sendHeartbeat,'sendSetpoint',@sendSetpoint, ...
    'requestCommand',@requestCommand,'commandAck',@commandAck, ...
    'readParameter',@readParameter,'setIntegerParameter',@setIntegerParameter, ...
    'setRealParameter',@setRealParameter, ...
    'setNativeHoverTuningParameter',@setNativeHoverTuningParameter, ...
    'sendPlantEnvironment',@sendPlantEnvironment, ...
    'takeCanonicalEnvironmentRecords',@takeCanonicalEnvironmentRecords, ...
    'requestPrearmHealthReport',@requestPrearmHealthReport,'healthReport',@healthReport, ...
    'canonicalObservation',@canonicalObservation, ...
    'pollCanonicalModuleStatus',@pollCanonicalModuleStatus, ...
    'canonicalModuleGuard',@canonicalModuleGuard, ...
    'bindCanonicalSession',@bindCanonicalSession,'retireCanonicalSession',@retireCanonicalSession, ...
    'takeCanonical',@takeCanonical, ...
    'pollCanonical',@pollCanonical,'sendCanonicalPackets',@sendCanonicalPackets, ...
    'sendCanonicalLocalGp',@sendCanonicalLocalGp, ...
    'sendCanonicalLocalInput',@sendCanonicalLocalInput, ...
    'beginCanonicalLocalWindow',@beginCanonicalLocalWindow, ...
    'sendCanonicalLocalWindowChunk',@sendCanonicalLocalWindowChunk, ...
    'sendCanonicalSession',@sendCanonicalSession, ...
    'canonicalSessionReceipts',@canonicalSessionReceipts, ...
    'takeCanonicalRotorRecords',@takeCanonicalRotorRecords, ...
    'close',@closeAll,'evidence',@evidence);

    function t=nowS(),t=toc(timer);end
    function r=takeCanonicalRotorRecords()
        drainCanonicalRotorSocket();
        nowNs=gpenmpcNative.rflyOriginalHostMonotonicNs();t=nowS();
        r=struct('schema','RFLY_SAME_IO_RAW_ROTOR_OBSERVATIONS_V1', ...
            'records',{canonicalRotorQueue},'cache_records',{canonicalCacheQueue},'original_poll_ns',nowNs, ...
            'original_io_poll_s',t,'failure',canonicalRotorFailure, ...
            'received_count',canonicalRotorReceived,'cache_received_count',canonicalCacheReceived, ...
            'cache_observation',canonicalCacheSample, ...
            'independent_model_clock',m600check.copterSimDiagnosticSnapshot(modelDiagnostic,t,cfg.state_max_age_s), ...
            'clock_source','OFFICIAL_COPTERSIM_MODEL_DIAGNOSTIC_NOT_ROTOR_PACKET', ...
            'rotor_decode_or_origin_attestation_performed',false,'publication_authority',false);
        canonicalRotorQueue={};canonicalCacheQueue={};
    end
    function drainCanonicalRotorSocket()
        assert(canonicalRotorEnabled&&~closed,'m600check:CanonicalRotorNotOpen', ...
            'The existing IO owner must have an open optional rotor receiver.');
        % The independent clock comes from the official model diagnostic,
        % never from the rotor datagram being judged. A caller must wait for
        % a suitable clock observation rather than invent a newer sim time.
        drainTime();drainTruth();
        if isempty(canonicalRotorFailure)
            n=canonicalRotorSocket.NumDatagramsAvailable;
            n=min(n,cfg.canonical_rotor_observer.maximum_queue-numel(canonicalRotorQueue)-numel(canonicalCacheQueue)+1);
            if n>0
                datagrams=read(canonicalRotorSocket,n,'uint8');
                for j=1:n
                    originalNs=gpenmpcNative.rflyOriginalHostMonotonicNs();rx=nowS();
                    bytes=datagramBytes(datagrams,j);
                    row=struct('bytes',bytes,'original_host_receive_ns',originalNs, ...
                        'original_io_receive_s',rx,'sender_address',char(datagrams(j).SenderAddress), ...
                        'sender_port',double(datagrams(j).SenderPort), ...
                        'receive_semantics','MATLAB_DEQUEUE_NOT_KERNEL_ARRIVAL');
                    isCache=numel(bytes)>=8&&isequal(reshape(bytes(1:8),[],1),uint8('M6CACHE1').');
                    if isCache
                        appendRaw('CACHE',row);canonicalCacheReceived=canonicalCacheReceived+uint64(1);
                    else
                        appendRaw('ROTOR',row);canonicalRotorReceived=canonicalRotorReceived+uint64(1);
                    end
                    if ~strcmp(row.sender_address,'127.0.0.1')
                        canonicalRotorFailure='ROTOR_NONLOCAL_SOURCE';
                    elseif numel(canonicalRotorQueue)+numel(canonicalCacheQueue)>=cfg.canonical_rotor_observer.maximum_queue
                        canonicalRotorFailure='ROTOR_OBSERVATION_QUEUE_OVERFLOW';
                    elseif rawDropped.ROTOR>0||rawDropped.CACHE>0
                        canonicalRotorFailure='ROTOR_RAW_MEMORY_BOUND';
                    else
                        if isCache
                            canonicalCacheQueue{end+1}=row;
                            if canonicalCacheEnabled,observeCanonicalCache(row);end
                        else,canonicalRotorQueue{end+1}=row;
                        end
                    end
                    if ~isempty(canonicalRotorFailure)
                        latchFatal(canonicalRotorFailure,false);break
                    end
                end
            end
        end
        if canonicalCacheEnabled,observeCanonicalCache([]);end
    end
    function observeCanonicalCache(row)
        cacheNow=nowS();modelClock=m600check.copterSimDiagnosticSnapshot(modelDiagnostic,cacheNow,cfg.state_max_age_s);
        sourceNow=NaN;if modelClock.model_ready,sourceNow=modelClock.last_source_time_s;end
        cacheClock=struct('now_ns',gpenmpcNative.rflyOriginalHostMonotonicNs(), ...
            'now_wall_time_s',cacheNow,'now_sim_time_s',sourceNow);
        cacheBytes=[];
        if ~isempty(row)
            cacheBytes=row.bytes;cacheClock.original_host_receive_ns=row.original_host_receive_ns;
            cacheClock.receive_wall_time_s=row.original_io_receive_s;
        end
        [canonicalCacheSample,canonicalCacheState]=m600check.decodeCanonicalControlCache( ...
            cacheBytes,cacheExpected,cacheClock,canonicalCacheState);
        if canonicalCacheState.failed
            if isempty(canonicalRotorFailure),canonicalRotorFailure=canonicalCacheState.failure_reason;end
            latchFatal(canonicalRotorFailure,false);
        end
    end
    function r=bindCanonicalSession(expected,association)
        assert(canonicalExchangeEnabled,'m600check:CanonicalExchangeNotConfigured','Optional exchange is not configured.');
        try
            assert(~closed&&isempty(fatal)&&isempty(canonicalExchangeFailure) ...
                &&~canonicalExchangeBindAttempted,'m600check:CanonicalExchangeBindOnce','An existing or failed binding cannot be replaced.');
            canonicalExchangeBindAttempted=true;
            assert(isequal(expected.source_system,uint8(cfg.target_system)) ...
                &&isequal(expected.source_component,uint8(cfg.target_component)) ...
                &&isequal(expected.target_system,uint8(255)) ...
                &&isequal(expected.target_component,uint8(190)), ...
                'm600check:CanonicalExchangeEndpoints','Registered endpoints must match this existing IO.');
            if nargin>=2||~isempty(canonicalSessionHistory)
                assert(nargin>=2,'m600check:CanonicalSessionAssociation', ...
                    'A subsequent binding requires its actual new registration receipts.');
                association=checkedCanonicalAssociation(association,expected);
                for historyIndex=1:numel(canonicalSessionHistory)
                    old=canonicalSessionHistory{historyIndex};prior=old.release.registered_association;
                    assert(~isequal(association.echo.host_challenge,prior.echo.host_challenge) ...
                        &&~strcmpi(association.execution_session_sha256,prior.execution_session_sha256) ...
                        &&association.echo.process_session_generation>prior.echo.process_session_generation ...
                        &&association.echo.board_registration_hrt_us>prior.echo.board_registration_hrt_us ...
                        &&association.echo.link_lifecycle_generation==prior.echo.link_lifecycle_generation ...
                        &&association.echo.uid==prior.echo.uid ...
                        &&association.original_request.original_prepare_submit_ns>old.release.last_original_host_receive_ns, ...
                        'm600check:CanonicalSessionReplay','Old challenge/session or changed same-link lifetime cannot be rebound.');
                end
            end
            if canonicalLocalMode
                localExpected=expected;localExpected.boot_generation=expected.session_generation;
                localExpected.require_learning_audit=true;
                if isfield(cfg.canonical_exchange,'require_learning_audit')
                    localExpected.require_learning_audit=cfg.canonical_exchange.require_learning_audit;
                end
                canonicalExchange=gpenmpcNative.RflyLocalTunnelReassembler(localExpected,cfg.canonical_exchange.assembly_limit_ns);
            else
                canonicalExchange=gpenmpcNative.RflyTunnelReassembler(expected,cfg.canonical_exchange.assembly_limit_ns);
            end
            canonicalExchangeExpected=expected;
            canonicalExchangeAssociation=[];
            if nargin>=2,canonicalExchangeAssociation=association;end
            canonicalLocalInputLastSource=uint64(0);
            canonicalLocalWindow=[];
            canonicalExchangeOrigin=struct('link_lifecycle_generation',expected.link_lifecycle_generation, ...
                'execution_session_sha256',expected.execution_session_sha256);
            r=pollCanonical();
        catch problem,failCanonicalExchange(problem);rethrow(problem);end
    end
    function r=retireCanonicalSession(records,request)
        % Retire a binding after observed task/callback detachment;
        % retain the first fault and assembler lifetime.
        assert(canonicalExchangeEnabled&&~isempty(canonicalExchange), ...
            'm600check:CanonicalExchangeUnbound','An existing bound session is required.');
        try
            assert(~closed&&isempty(fatal)&&isempty(canonicalExchangeFailure)&&~canonicalExchangeSendInProgress, ...
                'm600check:CanonicalExchangeClosed','Failed/closed/in-flight exchange cannot be recycled.');
            assert(numel(canonicalSessionHistory)<canonicalSessionHistoryCapacity ...
                &&numel(canonicalRetirementAttempts)<cfg.maximum_raw_records, ...
                'm600check:CanonicalSessionHistoryFull','Never evict old sessions to permit replay.');
            release=gpenmpcNative.RflySessionReleaseDecoder(records,request,dialect);
            checkedCanonicalAssociation(release.registered_association,canonicalExchangeExpected);
            assertCanonicalOriginalRecords(records);
            assertCanonicalOriginalCommand('stop',request.original_stop_submit_ns);
            assertCanonicalOriginalCommand('release',request.original_release_submit_ns);
            state=canonicalExchange.poll(gpenmpcNative.rflyOriginalHostMonotonicNs());
            assert(~state.failed&&~state.closed&&~any(state.active)&&~any(state.ready) ...
                &&all(cellfun(@isempty,canonicalCompleted)), ...
                'm600check:CanonicalSessionNotDrained','Partial, ready or queued original messages must not be discarded.');
            r=struct('schema','M600_SAME_IO_SESSION_RETIREMENT_V1','retired',false, ...
                'status','PLANT_CACHE_ZERO_UNOBSERVABLE','release',release, ...
                'assembler_evidence',canonicalExchange.evidence(),'completed_queues',{canonicalCompleted}, ...
                'completed_enqueued',canonicalCompletedEnqueued,'completed_taken',canonicalCompletedTaken, ...
                'same_existing_mavlinkio',~rawTransportEnabled,'same_existing_transport_owner',true, ...
                'transport_kind',transportKind,'network_inflight_datagrams_drained_proven',false, ...
                'plant_cache_zero_proven',false,'board_disarm_and_detach_report_only',true, ...
                'model_diagnostic_observer',modelDiagnostic,'original_retire_host_ns',gpenmpcNative.rflyOriginalHostMonotonicNs());
            % M6CACHE1 supplies the accepted-step input sample for cache checks.
            if release.requires_independent_plant_cache_zero
                [cachePass,cacheReceipt]=canonicalCacheRetirementEvidence(release,request);
                r.cache_evidence=cacheReceipt;
                if ~cachePass
                    r.status=cacheReceipt.status;canonicalRetirementAttempts{end+1}=r;return
                end
                r.plant_cache_zero_proven=true;r.status='RETIRED_PUBLISHED_WITH_POST_RELEASE_ACCEPTED_CACHE_ZERO';
            else
                r.status='RETIRED_NO_OWN_CONTROL_PUBLICATION';
            end
            % Cache and clock collection may dispatch callbacks;
            % recheck the source queue immediately before retirement.
            state=canonicalExchange.poll(gpenmpcNative.rflyOriginalHostMonotonicNs());
            assert(~state.failed&&~state.closed&&~any(state.active)&&~any(state.ready) ...
                &&all(cellfun(@isempty,canonicalCompleted))&&~canonicalExchangeSendInProgress, ...
                'm600check:CanonicalSessionNotDrained','New original ingress arrived during retirement evidence collection.');
            r.assembler_evidence=canonicalExchange.evidence();
            r.retired=true;
            canonicalExchange.close();r.closed_assembler_evidence=canonicalExchange.evidence();
            r.expected=canonicalExchangeExpected;
            canonicalSessionHistory{end+1}=r;canonicalRetirementAttempts{end+1}=r;
            canonicalExchange=[];canonicalExchangeExpected=[];canonicalExchangeOrigin=[];canonicalExchangeAssociation=[];
            canonicalExchangeBindAttempted=false;
            canonicalCompleted=repmat({cell(0,1)},1,numel(canonicalChannels));
            canonicalCompletedEnqueued=zeros(1,numel(canonicalChannels),'uint64');canonicalCompletedTaken=zeros(1,numel(canonicalChannels),'uint64');
        catch problem,failCanonicalExchange(problem);rethrow(problem);end
    end
    function [yes,e]=canonicalCacheRetirementEvidence(release,request)
        yes=false;e=struct('status','PLANT_CACHE_ZERO_UNOBSERVABLE','proven',false, ...
            'runtime_origin_attested',false,'physical_isolation_proven',false);
        if ~canonicalCacheEnabled,return;end
        % Snapshot obtains independently validated model/time telemetry; the
        % cache socket drain preserves new rotor rows for the caller's provider.
        safety=snapshot();drainCanonicalRotorSocket();
        e.actual_snapshot=safety;e.cache_observation=canonicalCacheSample;e.cache_state=canonicalCacheState;
        assert(isempty(fatal)&&~canonicalCacheState.failed,'m600check:CanonicalCacheRetirementFault','Cache or existing IO first fault is terminal.');
        if isempty(canonicalCacheSample)||~canonicalCacheSample.valid
            e.status='PLANT_CACHE_NOT_FRESH_OR_INDEPENDENT_CLOCK_PENDING';return
        end
        if ~canonicalCacheSample.all16_zero,e.status='PLANT_CACHE_NONZERO';return;end
        e.original_packet=canonicalCacheState.bytes;
        stopRow=canonicalOriginalCommand('stop',request.original_stop_submit_ns);
        releaseIoRx=canonicalOriginalReceiveIoTime(release.original_records(end));
        c=safety.clock_diagnostic;
        if ~safety.clock_valid||~safety.model_ready||~isfinite(releaseIoRx) ...
            ||~all(isfinite([c.coptersim_start_utc_s,c.utc_zero_s,c.uncertainty_s, ...
                c.utc_pair_uncertainty_s,c.utc_drift_s,c.utc_drift_uncertainty_s]))
            e.status='PLANT_CACHE_POST_STOP_CLOCK_UNPROVEN';return
        end
        % RflyTimeStmp has one-millisecond format quantization.
        mapUncertainty=max(c.uncertainty_s,c.utc_pair_uncertainty_s+.001) ...
            +abs(c.utc_drift_s)+c.utc_drift_uncertainty_s;
        lower=canonicalCacheSample.sim_time_s+(c.coptersim_start_utc_s-c.utc_zero_s)-mapUncertainty;
        e.source_io_time_lower_bound_s=lower;e.mapping_uncertainty_s=mapUncertainty;
        e.original_stop_io_send_s=stopRow.sent_s;e.original_release_receipt_io_s=releaseIoRx;
        e.mapping_scope='EXISTING_BOUNDED_APPROXIMATE_IO_MAPPING_NOT_EXACT_CLOCK_IDENTITY';
        if lower<=stopRow.sent_s||lower<=releaseIoRx ...
            ||canonicalCacheSample.original_host_receive_ns<=release.last_original_host_receive_ns
            e.status='PLANT_CACHE_NOT_PROVEN_AFTER_RELEASE';return
        end
        if ~(safety.armed==0&&safety.landed_state==1&&safety.model_diagnostic.decoded.ground_confirmed)
            e.status='PLANT_CACHE_FRESH_DUAL_GROUND_DISARM_NOT_OBSERVED';return
        end
        freshNow=nowS();
        if ~isfield(cfg,'heartbeat_max_age_s')||~isfield(cfg,'landed_max_age_s') ...
            ||freshNow<safety.heartbeat_rx_s||freshNow<safety.extended_rx_s ...
            ||freshNow-safety.heartbeat_rx_s>cfg.heartbeat_max_age_s ...
            ||freshNow-safety.extended_rx_s>cfg.landed_max_age_s
            e.status='PLANT_CACHE_BOARD_GROUND_EVIDENCE_STALE';return
        end
        observeCanonicalCache([]);
        assert(~canonicalCacheState.failed,'m600check:CanonicalCacheRetirementFault','Original cache observation expired or failed.');
        if ~canonicalCacheSample.valid,e.status='PLANT_CACHE_NOT_FRESH_OR_INDEPENDENT_CLOCK_PENDING';return;end
        yes=true;e.status='POST_RELEASE_ACCEPTED_STEP_ALL16_INPUT_CACHE_ZERO';e.proven=true;
    end
    function a=checkedCanonicalAssociation(a,expected)
        a=gpenmpcNative.RflySessionAssociationDecoder(a.prepare_receipt.original_frames, ...
            a.confirm_receipt.original_frames,a.original_request,a.physical_declaration,dialect);
        assertCanonicalOriginalRecords(a.prepare_receipt.original_frames);
        assertCanonicalOriginalRecords(a.confirm_receipt.original_frames);
        prepareAction='prepare';if a.local_full_inner,prepareAction='prepare_local';end
        assertCanonicalOriginalCommand(prepareAction,a.original_request.original_prepare_submit_ns);
        assertCanonicalOriginalCommand('confirm',a.original_request.original_confirm_submit_ns);
        e=a.echo;
        if isfield(expected,'local_full_inner')&&expected.local_full_inner
            assert(a.local_full_inner&&a.original_request.leg_index==expected.leg_index ...
                &&strcmpi(a.original_request.task_sha256,expected.task_sha256), ...
                'm600check:CanonicalLocalSessionAssociation','Local full-inner task/leg receipt required.');
        else
            assert(~a.local_full_inner,'m600check:CanonicalLocalSessionAssociation', ...
                'Local association must be explicitly selected by this IO profile.');
        end
        assert(e.uid==expected.uid&&e.system==expected.source_system&&e.component==expected.source_component ...
            &&e.process_session_generation==expected.session_generation ...
            &&e.link_lifecycle_generation==expected.link_lifecycle_generation ...
            &&a.original_host_receive_ns==expected.confirmed_host_rx_ns ...
            &&strcmpi(a.execution_session_sha256,expected.execution_session_sha256) ...
            &&strcmpi(e.configuration_payload_sha256,expected.configuration_sha256), ...
            'm600check:CanonicalSessionAssociation','Actual registration does not match this bound session.');
    end
    function assertCanonicalOriginalRecords(records)
        actual=canonicalSessionReceipts();last=0;
        for index=1:numel(records)
            found=0;
            for original=last+1:numel(actual)
                if isequaln(records(index),actual{original}),found=original;break;end
            end
            assert(found>0,'m600check:CanonicalSessionOriginalRecords', ...
                'Only this same IO original callback records, in order, are accepted.');
            last=found;
        end
    end
    function assertCanonicalOriginalCommand(action,submit)
        canonicalOriginalCommand(action,submit);
    end
    function selected=canonicalOriginalCommand(action,submit)
        found=false;
        for index=1:numel(rawTx)
            row=rawTx{index};
            if isfield(row,'canonical_session_action')&&strcmp(row.canonical_session_action,action) ...
                &&isequal(row.original_host_submit_ns,submit)&&row.send_returned
                found=true;selected=row;break
            end
        end
        assert(found,'m600check:CanonicalSessionOriginalCommand','Exact same-IO original command send required.');
    end
    function t=canonicalOriginalReceiveIoTime(record)
        t=NaN;
        for originalIndex=1:numel(rawMav)
            row=rawMav{originalIndex};
            if strcmp(row.topic,'SERIAL_CONTROL')&&isfield(row,'original_host_receive_ns') ...
                &&isequal(row.original_host_receive_ns,record.original_host_receive_ns)
                if rawTransportEnabled
                    matched=isfield(record,'raw_frame')&&isequal(row.raw_frame,record.raw_frame);
                else
                    matched=isfield(record,'decoded_message')&&isequal(row.decoded_message,record.decoded_message);
                end
                if matched,t=row.rx_s;return;end
            end
        end
    end
    function r=takeCanonical(channel)
        drainRawMavlink();
        assert(canonicalExchangeEnabled&&~isempty(canonicalExchange), ...
            'm600check:CanonicalExchangeUnbound','An externally confirmed session must bind this same IO first.');
        try
            assert(~closed&&isempty(fatal)&&isempty(canonicalExchangeFailure),'m600check:CanonicalExchangeClosed','Closed or failed exchange cannot consume live results.');
            nowNs=gpenmpcNative.rflyOriginalHostMonotonicNs();
            if canonicalCompletedCapacity==0
                r=canonicalExchange.take(channel,nowNs);
            else
                % Validate partial-message lifetime using its receive timestamp;
                % ingress failures retain queued data and block consumption.
                canonicalExchange.poll(nowNs);
                takeChannelIndex=find(canonicalChannels==string(channel));
                assert(isscalar(takeChannelIndex),'m600check:CanonicalCompletedChannel');
                r=[];
                if ~isempty(canonicalCompleted{takeChannelIndex})
                    r=canonicalCompleted{takeChannelIndex}{1};
                    % Preserve 0-by-1 after the last pop. Linear deletion
                    % makes 1-by-0; later {end+1,1} would insert an empty row.
                    canonicalCompleted{takeChannelIndex}(1,:)=[];
                    canonicalCompletedTaken(takeChannelIndex)=canonicalCompletedTaken(takeChannelIndex)+uint64(1);
                end
            end
        catch problem,failCanonicalExchange(problem);rethrow(problem);end
    end
    function r=pollCanonical()
        drainRawMavlink();
        assert(canonicalExchangeEnabled,'m600check:CanonicalExchangeNotConfigured','Optional exchange is not configured.');
        r=struct('bound',~isempty(canonicalExchange),'bind_attempted',canonicalExchangeBindAttempted, ...
            'failure',canonicalExchangeFailure,'unbound_messages_retained',canonicalExchangeUnboundReceived, ...
            'same_existing_mavlinkio',~rawTransportEnabled,'same_existing_transport_owner',true, ...
            'transport_kind',transportKind,'additional_connections',0,'status',[], ...
            'completed_queue_capacity',canonicalCompletedCapacity, ...
            'completed_queue_counts',cellfun(@numel,canonicalCompleted), ...
            'completed_enqueued',canonicalCompletedEnqueued,'completed_taken',canonicalCompletedTaken);
        r.board_local_full_inner=canonicalLocalMode;r.channels=canonicalChannels;
        r.retired_session_count=numel(canonicalSessionHistory);
        r.maximum_session_history=canonicalSessionHistoryCapacity;
        if isempty(canonicalExchange),return;end
        try
            r.status=canonicalExchange.poll(gpenmpcNative.rflyOriginalHostMonotonicNs());
        catch problem,failCanonicalExchange(problem);rethrow(problem);end
    end
    function r=sendCanonicalPackets(packets,originalContext,sendValidity)
        assert(~canonicalLocalMode,'m600check:CanonicalLegacySendDisabled', ...
            'The board-local inner must not accept legacy per-tick HOST numerical commands.');
        assert(canonicalExchangeEnabled&&~isempty(canonicalExchange), ...
            'm600check:CanonicalExchangeUnbound','Registered session binding is required.');
        ownsSend=false;
        try
            assert(~closed&&isempty(fatal)&&isempty(canonicalExchangeFailure) ...
                &&~canonicalExchangeSendInProgress,'m600check:CanonicalExchangeClosed','Failed, closed or recursive send is rejected.');
            hasOriginalContext=nargin>=2;
            if hasOriginalContext
                for contextField={'reference_creation_ns','outer_creation_ns','reference_expiry_ns','outer_expiry_ns'}
                    value=originalContext.(contextField{1});
                    assert(isa(value,'uint64')&&isscalar(value)&&value>0, ...
                        'm600check:CanonicalSendOriginalContext','Exact original uint64 HOST context required.');
                end
                assert(originalContext.reference_creation_ns<=originalContext.reference_expiry_ns ...
                    &&originalContext.outer_creation_ns<=originalContext.outer_expiry_ns, ...
                    'm600check:CanonicalSendOriginalContext','Original lifetime reversed.');
            else,originalContext=[];
            end
            hasSendValidity=nargin>=3;
            if hasSendValidity
                for validityField={'source_host_receive_ns','maximum_runtime_age_ns','source_valid_until_ns'}
                    value=sendValidity.(validityField{1});
                    assert(isa(value,'uint64')&&isscalar(value)&&value>0, ...
                        'm600check:CanonicalSendOriginalSource','Exact original source lifetime fields required.');
                end
                assert(sendValidity.maximum_runtime_age_ns<=intmax('uint64')-sendValidity.source_host_receive_ns ...
                    &&sendValidity.source_valid_until_ns==sendValidity.source_host_receive_ns+sendValidity.maximum_runtime_age_ns, ...
                    'm600check:CanonicalSendOriginalSource','Original source expiry overflow or mismatch.');
            else,sendValidity=[];
            end
            [messages,decoded]=m600check.decodeCanonicalExchangePackets(packets,dialect,canonicalExchangeExpected);
            assert(numel(rawTx)+numel(messages)<=cfg.maximum_raw_records, ...
                'm600check:CanonicalExchangeEvidenceBound','No send without original attempted-message evidence.');
            canonicalExchangeSendInProgress=true;ownsSend=true;
            submitted=zeros(numel(messages),1,'uint64');returned=submitted;
            for k=1:numel(messages)
                assert(~closed&&isempty(fatal),'m600check:CanonicalExchangeClosed','The existing owner failed before the next send.');
                submitted(k)=gpenmpcNative.rflyOriginalHostMonotonicNs();
                if hasOriginalContext,checkCanonicalSendDeadline(submitted(k),originalContext);end
                if hasSendValidity,checkCanonicalSourceDeadline(submitted(k),sendValidity);end
                row=struct('sent_s',nowS(),'message',messages{k}, ...
                    'original_host_submit_ns',submitted(k),'original_host_send_return_ns',uint64(0), ...
                    'canonical_input_serialized_frame',packets{k},'send_returned',false, ...
                    'send_attempted',false,'original_context',originalContext,'original_source_validity',sendValidity, ...
                    'input_frame_is_final_wire',false,'actual_wire_bytes_available',false);
                appendRaw('TX',row);index=numel(rawTx);
                % msgToPacket finalizes identity and sequence from this link.
                % Recheck the original deadline after receipt allocation.
                submitted(k)=gpenmpcNative.rflyOriginalHostMonotonicNs();
                if hasOriginalContext,checkCanonicalSendDeadline(submitted(k),originalContext);end
                if hasSendValidity,checkCanonicalSourceDeadline(submitted(k),sendValidity);end
                rawTx{index}.original_host_submit_ns=submitted(k);rawTx{index}.send_attempted=true;
                transmitMessage(messages{k},index);
                returned(k)=gpenmpcNative.rflyOriginalHostMonotonicNs();
                rawTx{index}.send_returned=true;rawTx{index}.original_host_send_return_ns=returned(k);
                canonicalExchangeSent=canonicalExchangeSent+uint64(1);
            end
            canonicalExchangeSendInProgress=false;
            r=struct('decoded_input',decoded,'messages_submitted',numel(messages), ...
                'original_host_submit_ns',submitted,'original_host_send_return_ns',returned, ...
                'same_existing_mavlinkio',~rawTransportEnabled,'same_existing_transport_owner',true, ...
                'transport_kind',transportKind,'final_sequence_generated_by_existing_link',true, ...
                'original_context_checked_per_frame',hasOriginalContext,'original_context',originalContext, ...
                'original_source_checked_per_frame',hasSendValidity,'original_source_validity',sendValidity, ...
                'actual_wire_bytes_available',rawTransportEnabled,'board_receipt_proven',false,'control_authority',false);
        catch problem
            if ownsSend,canonicalExchangeSendInProgress=false;end
            failCanonicalExchange(problem);rethrow(problem)
        end
    end
    function r=sendCanonicalLocalInput(bytes,originalSource)
        r=sendCanonicalLocalGp(bytes,originalSource,true);
    end
    function r=beginCanonicalLocalWindow(window)
        % Prepare the RWW1 payload bytes.
        try
            assert(canonicalLocalMode&&~isempty(canonicalExchange)&&~closed&&isempty(fatal) ...
                &&isempty(canonicalExchangeFailure)&&~canonicalExchangeSendInProgress, ...
                'm600check:CanonicalLocalUnbound','One healthy bound local owner is required.');
            assert(isfield(cfg.canonical_exchange,'local_window_host_max_age_ns'), ...
                'm600check:CanonicalLocalWindowBound','Explicit original window transport bound required.');
            if isempty(canonicalLocalWindow)
                canonicalLocalWindow=gpenmpcNative.RflyLocalWindowTransfer(canonicalExchangeExpected, ...
                    cfg.canonical_exchange.local_window_host_max_age_ns);
            end
            r=canonicalLocalWindow.begin(window);
        catch problem,failCanonicalExchange(problem);rethrow(problem);end
    end
    function r=sendCanonicalLocalWindowChunk(maxFragments)
        % Send a bounded chunk so source and GP service can run between calls.
        ownsSend=false;
        try
            assert(canonicalLocalMode&&~isempty(canonicalExchange)&&~isempty(canonicalLocalWindow) ...
                &&~closed&&isempty(fatal)&&isempty(canonicalExchangeFailure)&&~canonicalExchangeSendInProgress, ...
                'm600check:CanonicalLocalUnbound');
            assert(isnumeric(maxFragments)&&isscalar(maxFragments)&&isfinite(maxFragments) ...
                &&maxFragments==fix(maxFragments)&&maxFragments>=1&&maxFragments<=16, ...
                'm600check:CanonicalLocalWindowChunk','One to sixteen fragments per service call.');
            before=canonicalLocalWindow.evidence();count=min(maxFragments,245-before.next_fragment);
            assert(count>0&&numel(rawTx)+count<=cfg.maximum_raw_records, ...
                'm600check:CanonicalLocalWindowUnavailable','Completed transfers are not automatically resent.');
            canonicalExchangeSendInProgress=true;ownsSend=true;
            for k=1:count
                assert(~closed&&isempty(fatal)&&isempty(canonicalExchangeFailure),'m600check:CanonicalExchangeClosed');
                p=canonicalLocalWindow.frame();message=localTunnelMessage(p);
                row=struct('sent_s',nowS(),'message',message,'canonical_local_reference_window',true, ...
                    'original_host_submit_ns',uint64(0),'original_host_send_return_ns',uint64(0), ...
                    'send_attempted',false,'send_returned',false,'actual_wire_bytes_available',false);
                appendRaw('TX',row);index=numel(rawTx);
                submitted=gpenmpcNative.rflyOriginalHostMonotonicNs();binding=canonicalLocalWindow.attempt(submitted);
                rawTx{index}.original_host_submit_ns=submitted;rawTx{index}.send_attempted=true;
                rawTx{index}.original_window_validity=binding;
                transmitMessage(message,index);
                returned=gpenmpcNative.rflyOriginalHostMonotonicNs();
                rawTx{index}.send_returned=true;rawTx{index}.original_host_send_return_ns=returned;
                canonicalExchangeSent=canonicalExchangeSent+uint64(1);canonicalLocalWindow.sent(returned);
            end
            canonicalExchangeSendInProgress=false;r=canonicalLocalWindow.evidence();
            r.chunk_messages_send_returned=count;r.same_existing_mavlinkio=~rawTransportEnabled;
            r.same_existing_transport_owner=true;r.transport_kind=transportKind;
        catch problem
            if ownsSend,canonicalExchangeSendInProgress=false;end
            if ~isempty(canonicalLocalWindow),canonicalLocalWindow.fail(problem.identifier);end
            failCanonicalExchange(problem);rethrow(problem);
        end
    end
    function r=sendCanonicalLocalGp(replyBytes,originalRequest,taskInput)
        % Send the RGR1 numerical reply associated with this owner and RGP1.
        assert(canonicalLocalMode&&~isempty(canonicalExchange), ...
            'm600check:CanonicalLocalUnbound','A bound board-local exchange is required.');
        if nargin<3,taskInput=false;end
        ownsSend=false;
        try
            assert(~closed&&isempty(fatal)&&isempty(canonicalExchangeFailure)&&~canonicalExchangeSendInProgress, ...
                'm600check:CanonicalExchangeClosed','Failed/closed/reentrant send is rejected.');
            e=canonicalExchangeExpected;now=gpenmpcNative.rflyOriginalHostMonotonicNs();
            if taskInput
                assert(isfield(cfg.canonical_exchange,'local_input_host_max_age_ns'), ...
                    'm600check:CanonicalLocalInputBound','Explicit input transport bound required.');
                [payloads,binding]=gpenmpcNative.validateLocalTaskInputForSend(replyBytes,originalRequest,e, ...
                    cfg.canonical_exchange.local_input_host_max_age_ns,now);
                assert(binding.original_source_generation>canonicalLocalInputLastSource, ...
                    'm600check:CanonicalLocalInputReplay','No duplicate or regressed input source.');
            else
                [payloads,binding]=gpenmpcNative.validateLocalGpReplyForSend(replyBytes,originalRequest,e, ...
                    cfg.canonical_exchange.gp_reply_host_max_age_ns,now);
            end
            % Match every fragment to the callback ledger before sending.
            assert(isfield(originalRequest,'original_callback_indices') ...
                &&numel(originalRequest.original_callback_indices)==numel(originalRequest.decoded_fragments), ...
                'm600check:CanonicalLocalOriginalCallback','Original callback indices required, not a full-history search.');
            last=0;
            for j=1:numel(originalRequest.decoded_fragments)
                index=originalRequest.original_callback_indices(j);
                assert(isa(index,'uint64')&&index>last&&index<=uint64(numel(rawMav)), ...
                    'm600check:CanonicalLocalOriginalCallback');
                row=rawMav{double(index)};
                assert(strcmp(row.topic,'TUNNEL')&&isfield(row,'original_host_receive_ns') ...
                    &&isequal(row.original_host_receive_ns,originalRequest.fragment_rx_ns(j)) ...
                    &&isequaln(row.decoded_message,originalRequest.decoded_fragments{j}), ...
                    'm600check:CanonicalLocalOriginalCallback','Exact indexed original callback/time/message required.');
                last=index;
            end
            count=numel(payloads);
            assert(numel(rawTx)+count<=cfg.maximum_raw_records,'m600check:CanonicalExchangeEvidenceBound');
            canonicalExchangeSendInProgress=true;ownsSend=true;
            if taskInput,canonicalLocalInputLastSource=binding.original_source_generation;end
            submitted=zeros(count,1,'uint64');returned=submitted;
            for j=1:count
                assert(~closed&&isempty(fatal)&&isempty(canonicalExchangeFailure),'m600check:CanonicalExchangeClosed');
                p=payloads{j};message=localTunnelMessage(p);
                row=struct('sent_s',nowS(),'message',message,'canonical_local_gp_reply',~taskInput, ...
                    'canonical_local_task_inputs',taskInput, ...
                    'original_host_submit_ns',uint64(0),'original_host_send_return_ns',uint64(0), ...
                    'send_attempted',false,'send_returned',false,'actual_wire_bytes_available',false, ...
                    'original_source_validity',binding);
                if ~taskInput,row.original_request_sha256=binding.original_request_sha256;end
                appendRaw('TX',row);index=numel(rawTx);
                submitted(j)=gpenmpcNative.rflyOriginalHostMonotonicNs();
                assert(submitted(j)>=binding.original_host_receive_ns&&submitted(j)<=binding.valid_until_host_ns, ...
                    'm600check:CanonicalLocalGpExpired','Original GP reply transport interval expired.');
                rawTx{index}.original_host_submit_ns=submitted(j);rawTx{index}.send_attempted=true;
                transmitMessage(message,index);
                returned(j)=gpenmpcNative.rflyOriginalHostMonotonicNs();
                rawTx{index}.send_returned=true;rawTx{index}.original_host_send_return_ns=returned(j);
                canonicalExchangeSent=canonicalExchangeSent+uint64(1);
                assert(~closed&&isempty(fatal)&&isempty(canonicalExchangeFailure) ...
                    &&returned(j)<=binding.valid_until_host_ns,'m600check:CanonicalLocalGpExpired', ...
                    'Late/failed actual send remains recorded and cannot continue.');
            end
            canonicalExchangeSendInProgress=false;
            r=struct('messages_send_returned',count,'submitted_ns',submitted,'returned_ns',returned, ...
                'binding',binding,'same_existing_mavlinkio',~rawTransportEnabled, ...
                'same_existing_transport_owner',true,'transport_kind',transportKind,'board_receipt_proven',false, ...
                'control_authority',false,'actual_wire_bytes_available',rawTransportEnabled);
        catch problem
            if ownsSend,canonicalExchangeSendInProgress=false;end
            failCanonicalExchange(problem);rethrow(problem)
        end
    end
    function message=localTunnelMessage(p)
        % Keep the official same mavlinkio serializer, sequence and socket.
        % Copy-on-write prevents a later fragment changing an archived row.
        message=canonicalTunnelTemplate;
        message.Payload.target_system(:)=p.target_system(:);
        message.Payload.target_component(:)=p.target_component(:);
        message.Payload.payload_type(:)=p.payload_type(:);
        message.Payload.payload_length(:)=p.payload_length(:);
        message.Payload.payload(:)=p.payload(:);
    end
    function checkCanonicalSendDeadline(t,context)
        assert(t>=context.reference_creation_ns&&t>=context.outer_creation_ns ...
            &&t<=context.reference_expiry_ns&&t<=context.outer_expiry_ns, ...
            'm600check:CanonicalSendOriginalDeadline','Original reference/outer lifetime expired before this actual frame send.');
    end
    function checkCanonicalSourceDeadline(t,validity)
        assert(t>=validity.source_host_receive_ns&&t<=validity.source_valid_until_ns, ...
            'm600check:CanonicalSendOriginalSourceExpired','Original source lifetime expired before this actual frame send.');
    end
    function records=canonicalSessionReceipts()
        drainRawMavlink();
        assert(canonicalExchangeEnabled,'m600check:CanonicalExchangeNotConfigured','Optional exchange is not configured.');
        records={};
        for k=1:numel(rawMav)
            item=rawMav{k};
            if strcmp(item.topic,'SERIAL_CONTROL')&&isfield(item,'original_host_receive_ns')
                if rawTransportEnabled
                    % Raw and decoded alternatives use separate parser branches.
                    records{end+1}=struct('raw_frame',item.raw_frame, ...
                        'original_host_receive_ns',item.original_host_receive_ns, ...
                        'decoded_source',item.decoded_source,'raw_frame_available',true); %#ok<AGROW>
                else
                    records{end+1}=struct('decoded_message',item.decoded_message, ...
                        'original_host_receive_ns',item.original_host_receive_ns, ...
                        'decoded_source',item.decoded_source,'raw_frame_available',false); %#ok<AGROW>
                end
            end
        end
    end
    function r=sendCanonicalSession(action,value)
        assert(canonicalExchangeEnabled,'m600check:CanonicalExchangeNotConfigured','Optional exchange is not configured.');
        ownsSend=false;
        try
            % Allow cleanup and read-only requests while retaining the first fault.
            name=string(action);cleanup=ismember(name,["stop","status","release","evidence_failed","evidence_interrupted","evidence_local"]);
            if ismember(name,["stream_snapshot","stream_feedback","stream_local"])&&isfield(value,'enabled')
                cleanup=isequal(value.enabled,false);
            end
            assert(~closed&&~canonicalExchangeSendInProgress ...
                &&(cleanup||(isempty(fatal)&&isempty(canonicalExchangeFailure))), ...
                'm600check:CanonicalExchangeClosed','Only explicit cleanup/read-only requests survive exchange failure.');
            target=struct('system',uint8(cfg.target_system),'component',uint8(cfg.target_component));
            % Validate every supplied command against the encoder whitelist.
            [messages,encoding]=gpenmpcNative.RflySessionCommandEncoder(action,value,dialect,target);
            assert(numel(rawTx)+numel(messages)<=cfg.maximum_raw_records,'m600check:CanonicalExchangeEvidenceBound','No send without original attempted-message evidence.');
            canonicalSessionLastSend=struct('encoding',encoding,'messages_attempted',0,'messages_send_returned',0, ...
                'original_host_submit_ns',zeros(numel(messages),1,'uint64'), ...
                'original_host_send_return_ns',zeros(numel(messages),1,'uint64'), ...
                'partial',false,'error','','same_existing_mavlinkio',~rawTransportEnabled, ...
                'same_existing_transport_owner',true,'transport_kind',transportKind,'board_acknowledged',false, ...
                'final_sequence_generated_by_existing_link',true,'actual_wire_bytes_available',rawTransportEnabled);
            canonicalExchangeSendInProgress=true;ownsSend=true;
            for k=1:numel(messages)
                assert(~closed&&(cleanup||isempty(fatal)),'m600check:CanonicalExchangeClosed','The existing owner cannot continue this request.');
                submitted=gpenmpcNative.rflyOriginalHostMonotonicNs();
                row=struct('sent_s',nowS(),'message',messages{k},'canonical_session_action',char(name), ...
                    'original_host_submit_ns',submitted,'original_host_send_return_ns',uint64(0), ...
                    'send_returned',false,'actual_wire_bytes_available',false);
                appendRaw('TX',row);index=numel(rawTx);
                canonicalSessionLastSend.messages_attempted=k;
                canonicalSessionLastSend.original_host_submit_ns(k)=submitted;
                transmitMessage(messages{k},index);
                returned=gpenmpcNative.rflyOriginalHostMonotonicNs();
                rawTx{index}.send_returned=true;rawTx{index}.original_host_send_return_ns=returned;
                canonicalSessionLastSend.messages_send_returned=k;
                canonicalSessionLastSend.original_host_send_return_ns(k)=returned;
            end
            canonicalExchangeSendInProgress=false;r=canonicalSessionLastSend;
        catch problem
            if ownsSend,canonicalExchangeSendInProgress=false;end
            if ownsSend
                canonicalSessionLastSend.partial=canonicalSessionLastSend.messages_attempted>0;
                canonicalSessionLastSend.error=[problem.identifier ': ' problem.message];
            end
            failCanonicalExchange(problem);rethrow(problem)
        end
    end
    function failCanonicalExchange(problem)
        if isempty(canonicalExchangeFailure),canonicalExchangeFailure=[problem.identifier ': ' problem.message];end
        latchFatal(['CANONICAL_EXCHANGE:' canonicalExchangeFailure],false);
    end
    function updateDeliveryContinuity(t)
        if ~deliveryEnabled||isempty(hb)||isempty(extended),return;end
        armedNow=double(bitand(uint8(hb.base_mode),uint8(128))~=0);
        landedNow=double(extended.landed_state);
        fresh=isfinite(t)&&t>=hbRx&&t>=extRx&&t-hbRx<=cfg.heartbeat_max_age_s&&t-extRx<=cfg.landed_max_age_s;
        groundNow=fresh&&armedNow==0&&landedNow==1;
        if isempty(deliveryGroundState)
            deliveryGroundState=groundNow;
            deliveryTransitions(end+1,1)=struct('time_s',t,'ground_disarmed_fresh',groundNow, ...
                'armed',armedNow,'landed_state',landedNow,'epoch',deliveryContinuityEpoch); %#ok<AGROW>
        elseif groundNow~=deliveryGroundState
            deliveryContinuityEpoch=deliveryContinuityEpoch+1;
            deliveryGroundState=groundNow;
            deliveryTransitions(end+1,1)=struct('time_s',t,'ground_disarmed_fresh',groundNow, ...
                'armed',armedNow,'landed_state',landedNow,'epoch',deliveryContinuityEpoch); %#ok<AGROW>
        end
    end
    function appendRaw(which,value)
        switch which
            case 'MAV',n=numel(rawMav);
            case 'TRUTH',n=numel(rawTruth);
            case 'TIME',n=numel(rawTime);
            case 'TX',n=numel(rawTx);
            case 'ENV_TX',n=numel(rawEnvironmentTx);
            case 'ROTOR',n=numel(rawRotor);
            case 'CACHE',n=numel(rawCache);
        end
        if n>=cfg.maximum_raw_records
            rawDropped.(which)=rawDropped.(which)+1;
            latchFatal('RAW_MEMORY_RECORD_BOUND_EXCEEDED',false);return
        end
        switch which
            case 'MAV',rawMav{end+1}=value;
            case 'TRUTH',rawTruth{end+1}=value;
            case 'TIME',rawTime{end+1}=value;
            case 'TX',rawTx{end+1}=value;
            case 'ENV_TX',rawEnvironmentTx{end+1}=value;
            case 'ROTOR',rawRotor{end+1}=value;
            case 'CACHE',rawCache{end+1}=value;
        end
    end
    function drainRawMavlink()
        if ~rawTransportEnabled||closed||rawDraining||isempty(rawTransport),return;end
        rawDraining=true;unlock=onCleanup(@unlockRawDrain); %#ok<NASGU>
        try
            [records,fragments]=rawTransport.poll();
            retainForwardingFragments(fragments);
            for ri=1:numel(records)
                m=records(ri).decoded_message;where=find(topicIds==uint32(m.MsgID));
                name=['UNSUBSCRIBED_' sprintf('%u',uint32(m.MsgID))];
                if isscalar(where),name=topics{where};end
                onMav(name,m,records(ri));
                if ~isempty(fatal),break;end
            end
        catch problem
            % Retain the complete received batch and latch receive failures.
            try
                transportState=rawTransport.status();
                if isfield(transportState,'last_forwarding_fragments')
                    retainForwardingFragments(transportState.last_forwarding_fragments);
                end
            catch
                % Preserve the first error and complete native batch.
            end
            latchFatal(['RAW_MAVLINK:' problem.identifier],false);
            if canonicalExchangeEnabled,failCanonicalExchange(problem);end
        end
    end
    function unlockRawDrain(),rawDraining=false;end
    function retainForwardingFragments(fragments)
        assert(numel(rawForwardingFragments)+numel(fragments)<=cfg.maximum_raw_records, ...
            'm600check:RawForwardingMemoryBound','No unvalidated byte fragment may be dropped.');
        for k=1:numel(fragments),rawForwardingFragments{end+1}=fragments(k);end
    end
    function onMav(topic,messages,originalRaw)
        try
            for j=1:numel(messages)
                originalHostNs=[];
                if nargin>=3
                    assert(rawTransportEnabled&&isscalar(messages),'m600check:RawMessageOwner');
                    originalHostNs=originalRaw.original_host_receive_ns;
                elseif canonicalExchangeEnabled&&ismember(topic,{'TUNNEL','SERIAL_CONTROL'})
                    % Timestamp the first MATLAB callback with the monotonic clock.
                    originalHostNs=gpenmpcNative.rflyOriginalHostMonotonicNs();
                end
                m=messages(j);p=m.Payload;rx=nowS();
                raw=struct('topic',topic,'rx_s',rx,'message',m);
                if ~isempty(originalHostNs)
                    raw.original_host_receive_ns=originalHostNs;
                    raw.decoded_message=m;raw.raw_frame_available=false;
                    raw.decoded_source=['ORIGINAL_MAVLINKIO_' topic '_CALLBACK'];
                    if nargin>=3
                        raw.raw_frame_available=true;raw.raw_frame=originalRaw.raw_frame;
                        raw.decoded_source='RAW_OWNER_DATAGRAM_DEQUEUE_OFFICIAL_DECODER';
                        raw.transport_record=originalRaw;
                    end
                end
                appendRaw('MAV',raw);
                % Filter consumption to the bound board source; retain other raw messages.
                if nargin>=3&&(m.SystemID~=cfg.target_system||m.ComponentID~=cfg.target_component),continue;end
                if canonicalExchangeEnabled&&strcmp(topic,'TUNNEL')
                    if isempty(canonicalExchange)
                        canonicalExchangeUnboundReceived=canonicalExchangeUnboundReceived+uint64(1);
                    else
                        % Enqueue each complete callback message in the bounded FIFO
                        % before ingesting the next fragment.
                        try
                            if canonicalLocalMode
                                processingNs=gpenmpcNative.rflyOriginalHostMonotonicNs();
                                localOrigin=canonicalExchangeOrigin;
                                localOrigin.original_callback_index=uint64(numel(rawMav));
                                ingress=canonicalExchange.ingest(m,originalHostNs,processingNs,localOrigin);
                            else
                                processingNs=originalHostNs;
                                ingress=canonicalExchange.ingest(m,originalHostNs,canonicalExchangeOrigin);
                            end
                            if canonicalCompletedCapacity>0&&strcmp(ingress.status,'COMPLETE_RETAINED')
                                assert(sum(cellfun(@numel,canonicalCompleted))<canonicalCompletedCapacity, ...
                                    'm600check:CanonicalCompletedQueueOverflow', ...
                                    'No overwrite/drop: completed ingress and queued originals are retained.');
                                ingressChannelIndex=find(canonicalChannels==string(ingress.completed_channel));
                                item=canonicalExchange.take(ingress.completed_channel,processingNs);
                                canonicalCompleted{ingressChannelIndex}{end+1,1}=item;
                                canonicalCompletedEnqueued(ingressChannelIndex)=canonicalCompletedEnqueued(ingressChannelIndex)+uint64(1);
                            end
                        catch problem,failCanonicalExchange(problem);rethrow(problem);end
                    end
                end
                if canonicalObserverEnabled&&ismember(topic,{'ODOMETRY','HIL_ACTUATOR_CONTROLS'})
                    [canonicalObserver,observationReceipt]= ...
                        m600check.advanceCanonicalPx4Observation( ...
                            canonicalObserver,'MESSAGE', ...
                            struct('message',m,'rx_ns',round(rx*1e9)));
                    if observationReceipt.fatal_latched
                        latchFatal(['CANONICAL_OBSERVATION:' ...
                            canonicalObserver.first_failure.reason],false);
                    end
                end
                if canonicalModuleEnabled&&strcmp(topic,'SERIAL_CONTROL')
                    if canonicalModuleSendInProgress
                        % sendudpmsg can dispatch a re-entrant MATLAB callback.
                        % Hold the exact receive record until its send return
                        % is known; never admit an unconfirmed response.
                        bytes=max(1,double(p.count));
                        if numel(canonicalModuleDeferred)>=cfg.maximum_raw_records|| ...
                                canonicalModuleDeferredBytes+bytes>canonicalModule.maximum_response_bytes
                            latchFatal('CANONICAL_MODULE_SEND_CALLBACK_BUFFER_OVERFLOW',false);
                        else
                            canonicalModuleDeferred{end+1}=struct('message',m,'rx_ns',round(rx*1e9)); %#ok<AGROW>
                            canonicalModuleDeferredBytes=canonicalModuleDeferredBytes+bytes;
                            canonicalModuleDeferredTotal=canonicalModuleDeferredTotal+1;
                        end
                    else
                        consumeCanonicalModuleMessage(m,round(rx*1e9),false);
                    end
                end
                switch topic
                    case 'HEARTBEAT',hb=p;hbRx=rx;updateDeliveryContinuity(rx);
                    case 'EXTENDED_SYS_STATE',extended=p;extRx=rx;updateDeliveryContinuity(rx);
                    case 'LOCAL_POSITION_NED'
                        localObserver=observeSource(localObserver,p,double(p.time_boot_ms)/1000,rx,'LOCAL');
                        if isempty(estimate)||double(p.time_boot_ms)>double(estimate.time_boot_ms)
                            estimate=p;estimateRx=rx;
                        end
                    case 'ATTITUDE'
                        attitude=p;
                        attitudeObserver=observeSource(attitudeObserver,p,double(p.time_boot_ms)/1000,rx,'ATTITUDE');
                    case 'ESTIMATOR_STATUS'
                        estimatorObserver=observeSource(estimatorObserver,p,double(p.time_usec)/1e6,rx,'ESTIMATOR');
                    case 'HIL_ACTUATOR_CONTROLS',actuator=p;
                    case 'SYS_STATUS',systemStatus=p;systemStatusRx=rx;
                    case 'AUTOPILOT_VERSION',autopilotVersion=p;autopilotVersionRx=rx;
                    case 'COMMAND_ACK'
                        acks(end+1,1)=struct('command',double(p.command),'result',double(p.result), ...
                            'received_at_s',rx,'payload',p);
                    case 'PARAM_VALUE'
                        parameters(end+1,1)=struct('name',cleanText(p.param_id),'received_at_s',rx,'payload',p);
                    case 'TIMESYNC',acceptTimesync(p,rx);
                end
            end
        catch problem,latchFatal(['MAV_CALLBACK:' problem.message],false);end
    end
    function o=observeSource(o,p,source,rx,name)
        o.payload=p;o.latest_rx_s=rx;o.receive_count=o.receive_count+1;
        if ~isfinite(source)||source<0
            o.source_invalid_latched=true;
            latchFatal(['PX4_' name '_SOURCE_TIME_INVALID'],false);return
        end
        if isfinite(o.raw_source_time_s)&&source<o.raw_source_time_s
            o.source_reversal_latched=true;
            latchFatal(['PX4_' name '_SOURCE_TIME_REVERSED'],false);
        elseif ~isfinite(o.raw_source_time_s)||source>o.raw_source_time_s
            if isfinite(o.raw_source_time_s),o.source_advance_count=o.source_advance_count+1;end
            o.raw_source_time_s=source;o.rx_s=rx;o.source_progress_rx_s=rx;
            o.source_generation_count=o.source_generation_count+1;
        else
            o.duplicate_count=o.duplicate_count+1;
        end
    end
    function acceptTimesync(p,rx)
        if p.tc1<=0,return;end
        if syncLocked
            [clockValidation,clockValidationReceipt]=m600check.advanceFixedTimesyncValidation( ...
                clockValidation,'RESPONSE',struct('ts1_token',int64(p.ts1), ...
                'tc1_ns',int64(p.tc1),'receive_s',rx),clockValidationConfig);
            clockValidationResponses{end+1}=clockValidationReceipt;
            if clockValidationReceipt.fatal_latched
                latchFatal(['TIMESYNC:' clockValidationReceipt.first_failure],false);
            end
            return
        end
        k=find([syncSent.token]==int64(p.ts1),1,'last');
        if isempty(k),return;end
        if any([syncResponses.token]==int64(p.ts1)),return;end
        tx=syncSent(k).time_s;board=double(p.tc1)/1e9;rtt=rx-tx;
        accepted=isfinite(board)&&board>=0&&rtt>=0&&rtt<=cfg.clock_max_rtt_s;
        row=struct('token',int64(p.ts1),'send_s',tx,'receive_s',rx, ...
            'board_time_s',board,'rtt_s',rtt,'offset_lower_s',tx-board, ...
            'offset_upper_s',rx-board,'accepted',accepted);
        syncResponses(end+1,1)=row;
        if accepted&&(isempty(best)||rtt<best.rtt_s),best=row;end
        if nnz([syncResponses.accepted])>=cfg.clock_sync_samples
            syncLocked=true;
            [clockValidation,clockValidationReceipt]=m600check.advanceFixedTimesyncValidation( ...
                [],'INIT',struct('best',best,'last_accepted_board_time_s',board, ...
                'last_issued_token',syncSent(end).token,'now_s',rx),clockValidationConfig);
        end
    end
    function s=snapshot()
        drainRawMavlink();
        drainTime();drainTruth();
        % Wait for a CRC-valid, source-matched heartbeat before sending to
        % the NoUI UDP peer, avoiding traffic to an unbound endpoint.
        if automaticPeerReady()&&nowS()-lastSync>=cfg.clock_sync_period_s
            token=int64(round(nowS()*1e9))+1;tx=nowS();
            maySend=true;
            if syncLocked
                [clockValidation,clockValidationReceipt]=m600check.advanceFixedTimesyncValidation( ...
                    clockValidation,'SENT',struct('token',token,'send_s',tx),clockValidationConfig);
                maySend=clockValidationReceipt.request_registered;
            end
            if maySend
                syncSent(end+1,1)=struct('token',token,'time_s',tx);
                p=createmsg(dialect,'TIMESYNC');p.Payload.tc1=int64(0);p.Payload.ts1=token;
                send(p);
            end
            lastSync=tx;
        end
        utcBefore=nowS();utcObserved=posixtime(datetime('now','TimeZone','UTC'));utcAfter=nowS();
        drift=m600check.evaluateUtcDrift(utcBefore,utcObserved,utcAfter,utcPair,cfg.clock_max_utc_drift_s);
        utcDrift=drift.observed_drift_s;utcDriftUncertainty=drift.combined_uncertainty_s;
        uncertainty=Inf;mappingValid=false;mappingAge=Inf;mappingNow=nowS();
        if ~isempty(best)
            uncertainty=best.rtt_s/2+utcPairUncertainty+0.0005;
            mappingAge=mappingNow-best.receive_s;
            validationPassed=false;
            if syncLocked
                [clockValidation,clockValidationReceipt]=m600check.advanceFixedTimesyncValidation( ...
                    clockValidation,'POLL',struct('now_s',mappingNow),clockValidationConfig);
                uncertainty=clockValidationReceipt.uncertainty_s;
                mappingAge=clockValidationReceipt.validation_age_s;
                validationPassed=clockValidationReceipt.mapping_valid;
            end
            mappingValid=syncLocked&&isfinite(startUtc)&&~timeOriginChanged&& ...
                validationPassed&& ...
                mappingAge<=cfg.clock_max_age_s&& ...
                uncertainty<=cfg.clock_max_uncertainty_s&& ...
                drift.passed&& ...
                mappingNow-timeHeartbeatRx<=cfg.clock_max_time_heartbeat_age_s&& ...
                timeHeartbeatLag>=-(utcPairUncertainty+0.001)&& ...
                timeHeartbeatLag<=cfg.clock_max_time_heartbeat_lag_s;
        end
        updateDeliveryContinuity(nowS());
        model=m600check.copterSimDiagnosticSnapshot(modelDiagnostic,nowS(),cfg.state_max_age_s);
        s=struct('now_s',nowS(),'armed',NaN,'landed_state',NaN,'main_mode',NaN, ...
            'sub_mode',NaN,'heartbeat_rx_s',hbRx,'extended_rx_s',extRx, ...
            'estimate',[],'truth',[],'clock_valid',mappingValid&&isempty(nonModelFatal), ...
            'model_ready',model.model_ready,'model_diagnostic',model, ...
            'clock_uncertainty_s',uncertainty,'clock_utc_drift_s',utcDrift, ...
            'clock_utc_drift_uncertainty_s',utcDriftUncertainty,'clock_utc_drift_evidence',drift, ...
            'fatal',fatal,'first_fatal',firstFatal,'attitude',attitude,'actuator',actuator);
        s.autopilot_version=autopilotVersion;s.autopilot_version_rx_s=autopilotVersionRx;
        s.automatic_mavlink_peer_ready=automaticPeerReady();
        % SYS_STATUS supplies a partial component summary;
        % retain EVENT and sequence/error messages in raw_mavlink.
        s.system_status=systemStatus;s.system_status_rx_s=systemStatusRx;
        if ~isempty(hb)
            s.armed=double(bitand(uint8(hb.base_mode),uint8(128))~=0);
            s.main_mode=double(bitand(bitshift(uint32(hb.custom_mode),-16),uint32(255)));
            s.sub_mode=double(bitand(bitshift(uint32(hb.custom_mode),-24),uint32(255)));
        end
        if ~isempty(extended),s.landed_state=double(extended.landed_state);end
        if ~isempty(estimate)&&~isempty(best)
            source=double(estimate.time_boot_ms)/1000;
            mapped=source+(best.offset_lower_s+best.offset_upper_s)/2;
            pos=double([estimate.x,estimate.y,estimate.z]);vel=double([estimate.vx,estimate.vy,estimate.vz]);
            sample=stateRecord(mapped,estimateRx,pos,vel);
            s.estimate=struct('raw_source_time_s',source,'rx_s',estimateRx, ...
                'position_ned_m',pos,'velocity_ned_mps',vel,'sample',sample);
            meta=mapObserver(localObserver);
            for key={'mapped_source_time_s','latest_rx_s','source_progress_rx_s','receive_count', ...
                    'source_generation_count','source_advance_count','duplicate_count', ...
                    'source_reversal_latched','source_invalid_latched'}
                s.estimate.(key{1})=meta.(key{1});
            end
        end
        s.attitude_observation=mapObserver(attitudeObserver);
        s.estimator_status_observation=mapObserver(estimatorObserver);
        s.delivery_board_continuity_epoch=deliveryContinuityEpoch;
        s.delivery_environment=deliverySnapshot(s.now_s);
        if ~isempty(truth)&&isfinite(startUtc)
            mapped=truth.time_s+startUtc-utcZero;
            sample=stateRecord(mapped,truthRx,truth.position_ned_m,truth.velocity_ned_mps);
            s.truth=struct('raw_source_time_s',truth.time_s,'rx_s',truthRx, ...
                'position_ned_m',truth.position_ned_m,'velocity_ned_mps',truth.velocity_ned_mps,'sample',sample);
            lag=truthRx-mapped;
            if lag<-(utcPairUncertainty+0.0005)||lag>cfg.maximum_truth_lag_s
                s.clock_valid=false;
                latchFatal('SIMULATION_WALL_TIME_MAPPING_LAG_OUTSIDE_CALLER_BOUND',false);
                s.fatal=fatal;s.first_fatal=firstFatal;
            end
        end
        % Timestamp after copying receive/source fields and dispatching callbacks.
        s.now_s=nowS();
        s.clock_diagnostic=struct('fixed_best_timesync',best,'sync_locked',syncLocked, ...
            'periodic_validation',clockValidationReceipt, ...
            'mapping_age_basis','LAST_MATCHED_FIXED_MAPPING_VALIDATION__SOURCE_MAPPING_NEVER_REFIT', ...
            'mapping_evaluated_at_s',mappingNow,'mapping_age_s',mappingAge,'mapping_max_age_s',cfg.clock_max_age_s, ...
            'mapping_valid_before_fatal',mappingValid,'clock_valid',s.clock_valid, ...
            'uncertainty_s',uncertainty,'uncertainty_max_s',cfg.clock_max_uncertainty_s, ...
            'utc_pair_uncertainty_s',utcPairUncertainty,'utc_zero_s',utcZero, ...
            'utc_drift_s',utcDrift,'utc_drift_uncertainty_s',utcDriftUncertainty, ...
            'utc_drift_max_s',cfg.clock_max_utc_drift_s,'utc_drift_pass',drift.passed, ...
            'coptersim_start_utc_s',startUtc, ...
            'time_heartbeat_current_utc_s',timeHeartbeatCurrentUtc, ...
            'time_heartbeat_counter',timeCounter,'time_heartbeat_dequeue_rx_s',timeHeartbeatRx, ...
            'time_heartbeat_dequeue_age_s',s.now_s-timeHeartbeatRx, ...
            'time_heartbeat_age_at_mapping_evaluation_s',mappingNow-timeHeartbeatRx, ...
            'time_heartbeat_max_age_s',cfg.clock_max_time_heartbeat_age_s, ...
            'time_heartbeat_lag_s',timeHeartbeatLag, ...
            'time_heartbeat_min_lag_s',-(utcPairUncertainty+0.001), ...
            'time_heartbeat_lag_bound_s',cfg.clock_max_time_heartbeat_lag_s, ...
            'time_heartbeat_lag_pass',timeHeartbeatLag>=-(utcPairUncertainty+0.001)&& ...
                timeHeartbeatLag<=cfg.clock_max_time_heartbeat_lag_s, ...
            'clock_origin_changed',timeOriginChanged,'non_model_fatal',nonModelFatal, ...
            'receive_timestamp_semantics','MATLAB_DEQUEUE_NOW_NOT_KERNEL_NETWORK_ARRIVAL');
        s.native_hover_telemetry=m600check.evaluateNativeHoverTelemetry(s,cfg);
        if canonicalObserverEnabled
            s.canonical_px4_observation=canonicalObservation('SNAPSHOT');
        end
        if canonicalModuleEnabled
            s.canonical_module_observation=canonicalModuleGuard();
        end
    end
    function r=canonicalObservation(operation)
        assert(canonicalObserverEnabled, ...
            'm600check:CanonicalObserverNotConfigured', ...
            'No verified current-method observer binding exists.');
        if nargin<1,operation='SNAPSHOT';end
        assert(ismember(string(operation),["SNAPSHOT","DRAIN"]), ...
            'm600check:CanonicalObserverReadOnlyOperation');
        [canonicalObserver,r]=m600check.advanceCanonicalPx4Observation( ...
            canonicalObserver,operation,struct('now_ns',round(nowS()*1e9)));
        if r.fatal_latched
            latchFatal(['CANONICAL_OBSERVATION:' ...
                canonicalObserver.first_failure.reason],false);
        end
    end
    function r=pollCanonicalModuleStatus()
        % On explicit request, send one read-only listener over the owned MAVLink link.
        assert(canonicalModuleEnabled,'m600check:CanonicalModuleNotConfigured', ...
            'No verified SE3 listener observer binding exists.');
        assert(~closed&&isempty(nonModelFatal),'m600check:CanonicalModuleTransportClosed', ...
            'The listener transport is closed or failed.');
        boot=canonicalModule.binding.boot_generation;
        [canonicalModule,r]=gpenmpcNative.advanceSe3StatusObservation( ...
            canonicalModule,'TICK',struct('now_ns',round(nowS()*1e9),'boot_generation',boot));
        if r.query_prepared
            requestGeneration=r.request_generation;
            m=createmsg(dialect,'SERIAL_CONTROL');
            fields=fieldnames(r.tx_message.Payload);
            for k=1:numel(fields),m.Payload.(fields{k})=r.tx_message.Payload.(fields{k});end
            canonicalModuleSendInProgress=true;
            try
                send(m);
            catch problem
                [canonicalModule,~]=gpenmpcNative.advanceSe3StatusObservation( ...
                    canonicalModule,'TX_RESULT',struct('now_ns',round(nowS()*1e9), ...
                    'boot_generation',boot,'request_generation',requestGeneration,'send_succeeded',false));
                canonicalModuleSendInProgress=false;
                latchFatal('CANONICAL_MODULE_QUERY_SEND_FAILED',false);rethrow(problem);
            end
            [canonicalModule,sent]=gpenmpcNative.advanceSe3StatusObservation( ...
                canonicalModule,'TX_RESULT',struct('now_ns',round(nowS()*1e9), ...
                'boot_generation',boot,'request_generation',requestGeneration,'send_succeeded',true));
            r.owner_send_receipt=sent;
            % Consume after successful TX_RESULT, retaining receive time and
            % using dispatch time for age checks. Drain re-entrant chunks in order.
            if ~canonicalModule.fatal_latched&&isempty(nonModelFatal)
                k=1;
                while k<=numel(canonicalModuleDeferred)
                    item=canonicalModuleDeferred{k};
                    consumeCanonicalModuleMessage(item.message,item.rx_ns,true);
                    canonicalModuleDeferredProcessed=canonicalModuleDeferredProcessed+1;
                    if canonicalModule.fatal_latched||~isempty(nonModelFatal),break;end
                    k=k+1;
                end
                if ~canonicalModule.fatal_latched&&isempty(nonModelFatal)
                    canonicalModuleDeferred={};canonicalModuleDeferredBytes=0;
                end
            end
            canonicalModuleSendInProgress=false;
        end
        if canonicalModule.fatal_latched
            latchFatal(['CANONICAL_MODULE:' canonicalModule.first_failure.reason],false);
        end
    end
    function consumeCanonicalModuleMessage(m,rxNs,deferred)
        e=struct('message',m,'rx_ns',rxNs,'boot_generation',canonicalModule.binding.boot_generation);
        if deferred,e.processing_now_ns=round(nowS()*1e9);end
        [canonicalModule,moduleReceipt]=gpenmpcNative.advanceSe3StatusObservation(canonicalModule,'MESSAGE',e);
        if moduleReceipt.fatal_latched
            latchFatal(['CANONICAL_MODULE:' canonicalModule.first_failure.reason],false);
        end
    end
    function r=canonicalModuleGuard()
        assert(canonicalModuleEnabled,'m600check:CanonicalModuleNotConfigured', ...
            'No verified SE3 listener observer binding exists.');
        [canonicalModule,r]=gpenmpcNative.advanceSe3StatusObservation( ...
            canonicalModule,'GUARD',struct('now_ns',round(nowS()*1e9), ...
            'boot_generation',canonicalModule.binding.boot_generation));
        if r.fatal_latched
            latchFatal(['CANONICAL_MODULE:' canonicalModule.first_failure.reason],false);
        end
        if ~isempty(nonModelFatal)
            r.guard=[];r.guard_reason='ADAPTER_FATAL_NO_MODULE_GUARD';r.fatal_latched=true;
        end
    end
    function o=mapObserver(o)
        o.mapped_source_time_s=NaN;
        if ~isempty(best)&&isfinite(o.raw_source_time_s)
            o.mapped_source_time_s=o.raw_source_time_s+(best.offset_lower_s+best.offset_upper_s)/2;
        end
    end
    function d=deliverySnapshot(t)
        d=struct('enabled',deliveryEnabled,'present',false,'fresh',false, ...
            'receive_age_s',Inf,'maximum_age_s',cfg.state_max_age_s, ...
            'decoded',[],'observer_state',deliveryDiagnostic, ...
            'can_continue_task',false,'mass_ack_valid',false, ...
            'task_fault_latched',false,'first_fault',struct(), ...
            'board_continuity_epoch',deliveryContinuityEpoch, ...
            'board_ground_disarmed_fresh',false);
        if ~deliveryEnabled||isempty(deliveryLastDecoded),return;end
        d.present=true;d.receive_age_s=t-deliveryLastReceive;
        d.fresh=isfinite(d.receive_age_s)&&d.receive_age_s>=0&&d.receive_age_s<=cfg.state_max_age_s;
        d.decoded=deliveryLastDecoded;d.observer_state=deliveryDiagnostic;
        if isfield(deliveryLastDecoded,'stateful_delivery')
            q=deliveryLastDecoded.stateful_delivery;
            d.task_fault_latched=q.task_fault_latched;d.first_fault=q.first_fault;
            d.can_continue_task=d.fresh&&q.can_continue_task;
            d.mass_ack_valid=d.fresh&&q.mass_ack_valid;
        end
        if ~isempty(deliveryGroundState),d.board_ground_disarmed_fresh=deliveryGroundState;end
    end
    function sample=stateRecord(t,rx,pos,vel)
        sample=struct('clock_id','MATLAB_MONOTONIC_MAPPED_SOURCE_TIME','time_s',t, ...
            'received_at_s',rx,'position_ned_m',pos,'velocity_ned_mps',vel);
    end
    function drainTime()
        n=timeSocket.NumDatagramsAvailable;
        if n==0,return;end
        datagrams=read(timeSocket,n,'uint8');
        for j=1:n
            bytes=datagramBytes(datagrams,j);rx=nowS();appendRaw('TIME',struct('rx_s',rx,'bytes',bytes));
            if numel(bytes)~=32,continue;end
            ints=readLE(bytes(1:8),'int32');vals=readLE(bytes(9:32),'int64');
            if ints(1)~=123456789||ints(2)~=cfg.target_system,continue;end
            % Retain the NoUI [-1,-1,0] startup placeholder until a valid origin.
            % Once running, keep the established clock origin and sample lifetimes.
            if canonicalLocalMode&&isequal(vals(:),int64([-1;-1;0]))
                if ~isfinite(startUtc)&&~syncLocked
                    timeStartupUninitializedCount=timeStartupUninitializedCount+1;
                    rawTime{end}.classification='PRESTART_UNINITIALIZED_TIME__NOT_CLOCK_EVIDENCE';
                    continue
                end
                latchFatal('COPTERSIM_RUNNING_CLOCK_BECAME_UNINITIALIZED',false);continue
            end
            candidate=double(vals(1))/1000;
            if isfinite(startUtc)&&candidate~=startUtc,timeOriginChanged=true;end
            if double(vals(3))<timeCounter,latchFatal('COPTERSIM_HEARTBEAT_COUNT_REVERSED',false);end
            startUtc=candidate;timeHeartbeatRx=rx;timeCounter=double(vals(3));
            timeHeartbeatCurrentUtc=double(vals(2))/1000;
            timeHeartbeatLag=utcZero+rx-timeHeartbeatCurrentUtc;
        end
    end
    function events=takeCanonicalEnvironmentRecords()
        assert(canonicalLocalMode&&deliveryEnabled,'m600check:LocalEnvironmentNotBound');
        drainTruth();
        assert(~canonicalEnvironmentOverflow,'m600check:LocalEnvironmentQueueOverflow');
        events=canonicalEnvironmentQueue;canonicalEnvironmentQueue={};
    end
    function enqueueCanonicalEnvironment(event)
        if ~canonicalLocalMode||~deliveryEnabled,return;end
        if numel(canonicalEnvironmentQueue)>=256
            canonicalEnvironmentOverflow=true;latchFatal('LOCAL_ENVIRONMENT_QUEUE_OVERFLOW',false);return
        end
        canonicalEnvironmentQueue{end+1}=event;
    end
    function drainTruth()
        n=truthSocket.NumDatagramsAvailable;
        if n==0,return;end
        datagrams=read(truthSocket,n,'uint8');
        for j=1:n
            bytes=datagramBytes(datagrams,j);rx=nowS();originalReceiveNs=uint64(0);
            if canonicalLocalMode,originalReceiveNs=gpenmpcNative.rflyOriginalHostMonotonicNs();end
            appendRaw('TRUTH',struct('rx_s',rx,'bytes',bytes,'original_host_receive_ns',originalReceiveNs));
            % Decode the 264-byte ii32d outCopterData after retaining the datagram.
            if numel(bytes)==264
                enqueueCanonicalEnvironment(struct('kind','DIAGNOSTIC_RX','bytes',bytes(:), ...
                    'original_host_receive_ns',originalReceiveNs));
                if deliveryEnabled
                    [modelDiagnostic,deliveryDiagnostic,deliveryLastDecoded]= ...
                        m600check.updateCopterSimDeliveryDiagnostic(modelDiagnostic,deliveryDiagnostic, ...
                        bytes,rx,cfg.target_system,deliveryPolicy);
                    deliveryLastReceive=rx;
                    if isfield(deliveryLastDecoded,'stateful_delivery')&& ...
                            deliveryLastDecoded.stateful_delivery.task_fault_latched
                        latchFatal(['DELIVERY_ENVIRONMENT:' deliveryLastDecoded.stateful_delivery.first_fault.reason],false);
                    end
                else
                    modelDiagnostic=m600check.updateCopterSimDiagnostic( ...
                        modelDiagnostic,bytes,rx,cfg.target_system,cfg.require_terrain_diagnostic_extension);
                end
                if ~isempty(modelDiagnostic.fatal_reason)
                    latchFatal(modelDiagnostic.fatal_reason,true);
                end
                continue
            end
            % Other standard packets, e.g. 32B geographic origin, coexist.
            if ~ismember(numel(bytes),[112,168,200]),continue;end
            decoded=m600check.decodeTruthPacket(bytes,struct('expected_copter_id',cfg.target_system,'expected_vehicle_type',5));
            if ~decoded.valid,latchFatal(['TRUTH_DECODE:' decoded.reason],false);continue;end
            if ~isempty(truth)&&decoded.time_s<truth.time_s,latchFatal('TRUTH_TIME_REVERSED',false);end
            if isempty(truth)||decoded.time_s>truth.time_s,truth=decoded;truthRx=rx;end
        end
    end
    function latchFatal(reason,isModel)
        if isempty(fatal)
            fatal=reason;firstFatal=struct('received_at_s',nowS(),'reason',reason,'model_diagnostic',isModel);
        end
        % Evaluate model health separately from clock validity.
        if ~isModel&&isempty(nonModelFatal),nonModelFatal=reason;end
    end
    function send(message)
        row=struct('sent_s',nowS(),'message',message,'send_attempted',true,'send_returned',false, ...
            'actual_wire_bytes_available',false);
        appendRaw('TX',row);index=numel(rawTx);
        transmitMessage(message,index);rawTx{index}.send_returned=true;
    end
    function transmitMessage(message,index)
        if ~rawTransportEnabled
            sendudpmsg(link,message,'127.0.0.1',cfg.remote_mavlink_port);return
        end
        assert(index>=1&&index<=numel(rawTx),'m600check:RawTransmitLedger');
        try
            sent=rawTransport.sendMessage(message);
            rawTx{index}.transport_receipt=sent;
            rawTx{index}.actual_wire_bytes_available=true;
        catch problem
            % The native owner records attempted bytes, WSA result and QPC
            % interval even when sending throws.
            try
                state=rawTransport.status();rawTx{index}.transport_failure_state=state;
                rawTx{index}.transport_receipt=state.last_send;
            catch retainedError,rawTx{index}.transport_evidence_error=retainedError.identifier;end
            rethrow(problem)
        end
    end
    function receipt=sendPlantEnvironment(value)
        assert(deliveryEnabled,'m600check:DeliveryEnvironmentNotBound', ...
            'No 28-D task environment transmission without an explicit contract.');
        [bytes,frame]=gpenmpcTaskIo.encodePlantEnvironmentV2( ...
            value,cfg.target_system,deliveryContract.initial_payload_kg);
        sentAt=nowS();originalSendNs=uint64(0);
        if canonicalLocalMode,originalSendNs=gpenmpcNative.rflyOriginalHostMonotonicNs();end
        appendRaw('ENV_TX',struct('sent_s',sentAt,'frame',frame,'bytes',bytes,'original_host_send_ns',originalSendNs));
        write(truthSocket,bytes,'uint8','127.0.0.1',deliveryContract.remote_port);
        enqueueCanonicalEnvironment(struct('kind','ENV_TX','bytes',bytes(:), ...
            'original_host_send_ns',originalSendNs,'send_returned',true));
        receipt=struct('sent_s',sentAt,'generation',value.generation, ...
            'payload_generation',value.payload_generation,'service_release_generation',value.service_release_generation, ...
            'bytes',numel(bytes),'remote_port',deliveryContract.remote_port);
    end
    function ready=automaticPeerReady()
        ready=~(canonicalLocalMode&&rawTransportEnabled)||(~isempty(hb)&&isempty(fatal));
    end
    function sent=sendHeartbeat()
        sent=false;
        if ~automaticPeerReady(),return,end
        m=createmsg(dialect,'HEARTBEAT');m.Payload.type=uint8(6);m.Payload.autopilot=uint8(8);
        m.Payload.base_mode=uint8(0);m.Payload.custom_mode=uint32(0);
        m.Payload.system_status=uint8(4);m.Payload.mavlink_version=uint8(3);send(m);
        lastHeartbeatSent=nowS();sent=true;
    end
    function sendSetpoint(p)
        % Numeric vectors use the hover API. Reference structs carry the
        % delivery position, velocity, acceleration and yaw jet.
        % Retain jerk separately because MAVLink message 84 has no jerk field.
        if isstruct(p)
            requiredReference={'position_ned_m','velocity_ned_mps', ...
                'acceleration_ned_mps2','yaw_rad'};
            assert(isscalar(p)&&all(isfield(p,requiredReference)), ...
                'm600check:CanonicalReferenceSchema', ...
                'Canonical setpoint requires position, velocity, acceleration and yaw.');
            position=double(p.position_ned_m(:).');
            velocity=double(p.velocity_ned_mps(:).');
            acceleration=double(p.acceleration_ned_mps2(:).');
            yaw=double(p.yaw_rad);
            assert(numel(position)==3&&numel(velocity)==3&&numel(acceleration)==3&& ...
                all(isfinite([position,velocity,acceleration,yaw])), ...
                'm600check:CanonicalReferenceFinite', ...
                'Canonical MAVLink reference must be finite 3-D p/v/a plus yaw.');
        else
            position=double(p(:).');velocity=zeros(1,3);acceleration=zeros(1,3);yaw=0;
            assert(numel(position)==3&&all(isfinite(position)), ...
                'm600check:HoverReferenceFinite','Hover position must be a finite 3-vector.');
        end
        m=createmsg(dialect,'SET_POSITION_TARGET_LOCAL_NED');
        m.Payload.time_boot_ms=uint32(mod(round(nowS()*1000),2^32));
        m.Payload.x=single(position(1));m.Payload.y=single(position(2));m.Payload.z=single(position(3));
        m.Payload.vx=single(velocity(1));m.Payload.vy=single(velocity(2));m.Payload.vz=single(velocity(3));
        m.Payload.afx=single(acceleration(1));m.Payload.afy=single(acceleration(2));m.Payload.afz=single(acceleration(3));
        m.Payload.yaw=single(yaw);m.Payload.yaw_rate=single(0);
        m.Payload.type_mask=uint16(2048);m.Payload.coordinate_frame=uint8(1);
        m.Payload.target_system=uint8(cfg.target_system);m.Payload.target_component=uint8(cfg.target_component);send(m);
    end
    function requestCommand(cmd,p)
        m=createmsg(dialect,'COMMAND_LONG');
        for j=1:7,m.Payload.(sprintf('param%d',j))=single(p(j));end
        m.Payload.command=uint16(cmd);m.Payload.confirmation=uint8(0);
        m.Payload.target_system=uint8(cfg.target_system);m.Payload.target_component=uint8(cfg.target_component);send(m);
    end
    function a=commandAck(cmd,sent)
        drainRawMavlink();
        a=[];k=find([acks.command]==cmd&[acks.received_at_s]>=sent,1,'last');
        if ~isempty(k),a=acks(k);end
    end
    function request=requestPrearmHealthReport()
        % Explicitly request VEHICLE_CMD_RUN_PREARM_CHECKS to update
        % health_and_arming_checks without arming or changing mode.
        t=nowS();
        assert(~isempty(hb)&&t>=hbRx&&t-hbRx<=cfg.heartbeat_max_age_s&& ...
            bitand(uint8(hb.base_mode),uint8(128))==0, ...
            'm600check:HealthRequestRequiresFreshDisarmedHeartbeat');
        assert(~isempty(estimate)&&t>=estimateRx&&t-estimateRx<=cfg.state_max_age_s, ...
            'm600check:HealthRequestRequiresFreshBootTimestamp');
        request=struct('sent_s',t,'source_system',cfg.target_system, ...
            'source_component',cfg.target_component, ...
            'minimum_event_boot_ms',double(estimate.time_boot_ms));
        requestCommand(401,zeros(1,7));
    end
    function report=healthReport(request)
        drainRawMavlink();
        assert(~isempty(which('reduce_m600_px4_health_report')), ...
            'm600check:HealthReducerMissing');
        metadata=gpenmpc_external_path('px4_event_metadata');
        metadataSha='951C8B738A4FF0142E2AC30C2D0E105255DCECC889002DD86E29140CCB75A837';
        report=reduce_m600_px4_health_report(rawMav,request,metadata,metadataSha, ...
            'overflow',rawDropped.MAV>0);
        % Empty or partial reports remain UNKNOWN; evaluate health separately from ACK.
        report.command_ack=commandAck(401,request.sent_s);
    end
    function p=readParameter(name)
        m=createmsg(dialect,'PARAM_REQUEST_READ');m.Payload.param_index=int16(-1);
        m.Payload.target_system=uint8(cfg.target_system);m.Payload.target_component=uint8(cfg.target_component);
        m.Payload.param_id=fieldText(name,16);sent=nowS();send(m);p=waitParameter(name,sent,m);
    end
    function p=setIntegerParameter(name,value)
        before=readParameter(name);assert(before.mav_type==6,'m600check:MappingType','Expected INT32 mapping.');
        m=createmsg(dialect,'PARAM_SET');m.Payload.param_value=typecast(int32(value),'single');
        m.Payload.param_type=uint8(6);m.Payload.param_id=fieldText(name,16);
        m.Payload.target_system=uint8(cfg.target_system);m.Payload.target_component=uint8(cfg.target_component);
        sent=nowS();send(m);ack=waitParameter(name,sent);
        assert(ack.mav_type==6&&ack.decoded==value,'m600check:MappingACK','Mapping ACK mismatch.');
        p=readParameter(name);
    end
    function p=setRealParameter(name,rawBitsHex,expectedCurrentRawBitsHex,beforeSendGuard)
        % Apply CA geometry through exact REAL32 values.
        % The outer owner authorizes the candidate and restores parameters.
        assert(nargin==4,'m600check:RealParameterContract', ...
            'Exact expected current bits and an immediate pre-send safety callback are required.');
        assert(isa(beforeSendGuard,'function_handle'),'m600check:RealParameterGuard', ...
            'A fresh independent safety callback is required.');
        assert((ischar(name)&&isrow(name))||(isstring(name)&&isscalar(name)), ...
            'm600check:RealParameterName','Expected one exact parameter name.');
        name=char(name);
        assert(~isempty(regexp(name,'^CA_ROTOR[0-5]_P[XY]$','once')), ...
            'm600check:RealParameterName','Only twelve CA_ROTOR0..5_PX/PY fields are writable.');
        p=setRealParameterCore(name,rawBitsHex,expectedCurrentRawBitsHex,beforeSendGuard);
    end
    function p=setNativeHoverTuningParameter(name,rawBitsHex,expectedCurrentRawBitsHex,beforeSendGuard)
        % Separate exact three-field service, never an expansion of geometry.
        % The full provenance contract is checked before opening sockets.
        assert(nargin==4&&~isempty(nativeHoverTuning),'m600check:NativeHoverTuningNotBound', ...
            'The dedicated three-parameter contract and fresh guard are required.');
        assert((ischar(name)&&isrow(name))||(isstring(name)&&isscalar(name)), ...
            'm600check:NativeHoverTuningName','Expected one exact approved parameter name.');
        name=char(name);entries=nativeHoverTuning.entries;
        index=find(strcmp({entries.name},name));
        assert(isscalar(index)&&any(strcmp(name,{'MC_ROLL_P','MC_PITCH_P','MPC_THR_HOVER'})), ...
            'm600check:NativeHoverTuningName','Only the three contract entries are writable.');
        e=entries(index);
        assert(((ischar(rawBitsHex)&&isrow(rawBitsHex))||(isstring(rawBitsHex)&&isscalar(rawBitsHex)))&& ...
            ((ischar(expectedCurrentRawBitsHex)&&isrow(expectedCurrentRawBitsHex))|| ...
            (isstring(expectedCurrentRawBitsHex)&&isscalar(expectedCurrentRawBitsHex))), ...
            'm600check:NativeHoverTuningBits','Expected scalar raw-bit text for both endpoints.');
        assert(any(strcmpi(char(rawBitsHex),{e.original_raw_bits_hex,e.target_raw_bits_hex}))&& ...
            any(strcmpi(char(expectedCurrentRawBitsHex),{e.original_raw_bits_hex,e.target_raw_bits_hex})), ...
            'm600check:NativeHoverTuningOutsideExactPair','Only exact original/selected REAL32 endpoints are allowed.');
        assert(isa(beforeSendGuard,'function_handle'),'m600check:RealParameterGuard','Fresh safety callback required.');
        p=setRealParameterCore(name,rawBitsHex,expectedCurrentRawBitsHex,beforeSendGuard);
    end
    function p=setRealParameterCore(name,rawBitsHex,expectedCurrentRawBitsHex,beforeSendGuard)
        % Private transport implementation. Both public services above lock
        % their own disjoint names/pairs before reaching this common code.
        assert((ischar(rawBitsHex)&&isrow(rawBitsHex))||(isstring(rawBitsHex)&&isscalar(rawBitsHex)), ...
            'm600check:RealParameterBits','Expected one eight-digit REAL32 hex identity.');
        rawBitsHex=upper(char(rawBitsHex));
        assert(~isempty(regexp(rawBitsHex,'^[0-9A-F]{8}$','once')), ...
            'm600check:RealParameterBits','Expected exactly eight hexadecimal digits.');
        value=typecast(uint32(hex2dec(rawBitsHex)),'single');
        assert(isfinite(value),'m600check:RealParameterNonfinite','NaN/Inf REAL32 target rejected before any request.');
        assert((ischar(expectedCurrentRawBitsHex)&&isrow(expectedCurrentRawBitsHex))|| ...
            (isstring(expectedCurrentRawBitsHex)&&isscalar(expectedCurrentRawBitsHex)), ...
            'm600check:RealParameterExpectedBits','Expected current identity must be eight-digit REAL32 hex.');
        expectedCurrentRawBitsHex=upper(char(expectedCurrentRawBitsHex));
        assert(~isempty(regexp(expectedCurrentRawBitsHex,'^[0-9A-F]{8}$','once')), ...
            'm600check:RealParameterExpectedBits','Expected current identity must be exactly eight hex digits.');
        assert(isfinite(typecast(uint32(hex2dec(expectedCurrentRawBitsHex)),'single')), ...
            'm600check:RealParameterExpectedNonfinite','Expected current NaN/Inf is not permitted.');
        row=struct('name',name,'original_raw_bits_hex','','target_raw_bits_hex',rawBitsHex, ...
            'expected_current_raw_bits_hex',expectedCurrentRawBitsHex, ...
            'before_send_guard_invoked',false,'before_send_guard_passed',false, ...
            'attempted',false,'send_returned',false,'ack_verified',false,'readback_verified',false,'error','');
        realParameterActions(end+1,1)=row;k=numel(realParameterActions);
        try
            before=readParameter(name);realParameterActions(k).original_raw_bits_hex=before.raw_bits_hex;
            assert(before.mav_type==9&&isfinite(before.decoded), ...
                'm600check:RealParameterOriginalType','Original service field must be finite REAL32.');
            assert(strcmp(before.raw_bits_hex,expectedCurrentRawBitsHex), ...
                'm600check:RealParameterCurrentMismatch','Last internal read differs from exact expected current bits.');
            m=createmsg(dialect,'PARAM_SET');m.Payload.param_id=fieldText(name,16);
            m.Payload.param_value=value;m.Payload.param_type=uint8(9);
            m.Payload.target_system=uint8(cfg.target_system);m.Payload.target_component=uint8(cfg.target_component);
            % Run fresh disarmed/landed and zero-output checks after the
            % blocking parameter read and immediately before SET.
            realParameterActions(k).before_send_guard_invoked=true;
            okay=beforeSendGuard();
            assert(islogical(okay)&&isscalar(okay)&&okay,'m600check:RealParameterUnsafeWriteGuard', ...
                'Immediate pre-send independent safety callback did not return logical true.');
            realParameterActions(k).before_send_guard_passed=true;
            % send() records the exact attempted wire message BEFORE the API
            % call. Refuse the write if that evidence cannot be retained.
            assert(numel(rawTx)<cfg.maximum_raw_records,'m600check:RealParameterEvidenceBound', ...
                'No PARAM_SET without available attempted-message evidence capacity.');
            sent=nowS();realParameterActions(k).attempted=true;
            send(m);realParameterActions(k).send_returned=true;
            % Send PARAM_SET once and retain the initial acknowledgement deadline.
            ack=waitParameter(name,sent);
            assert(ack.mav_type==9&&strcmp(ack.raw_bits_hex,rawBitsHex), ...
                'm600check:RealParameterACK','REAL32 PARAM_SET acknowledgment bits/type mismatch.');
            realParameterActions(k).ack_verified=true;
            p=readParameter(name);
            assert(p.mav_type==9&&strcmp(p.raw_bits_hex,rawBitsHex), ...
                'm600check:RealParameterReadback','Independent REAL32 reread bits/type mismatch.');
            realParameterActions(k).readback_verified=true;
            p.real_write_receipt=realParameterActions(k);
        catch problem
            realParameterActions(k).error=[problem.identifier ': ' problem.message];
            rethrow(problem)
        end
    end
    function p=waitParameter(name,sent,readOnlyRetry)
        % Retry identical PARAM_REQUEST_READ once per second within its deadline;
        % PARAM_SET is sent once.
        if nargin<3,readOnlyRetry=[];end
        t=nowS();lastReadSent=sent;
        while nowS()-t<cfg.command_timeout_s
            % Drain synchronous UDP streams during parameter reads, preserving
            % observations, the original deadline and the 0.25 s lag limit.
            drainRawMavlink();drainTime();drainTruth();
            k=find(strcmp({parameters.name},name)&[parameters.received_at_s]>=sent,1,'last');
            if ~isempty(k)
                raw=single(parameters(k).payload.param_value);type=double(parameters(k).payload.param_type);
                decoded=double(raw);
                if type==6,decoded=double(typecast(raw,'int32'));
                elseif type==5,decoded=double(typecast(raw,'uint32'));end
                p=struct('name',name,'mav_type',type,'decoded',decoded, ...
                    'raw_bits_hex',upper(dec2hex(typecast(raw,'uint32'),8)));return
            end
            if nowS()-lastHeartbeatSent>=1,sendHeartbeat();end
            if ~isempty(readOnlyRetry)&&nowS()-lastReadSent>=1
                assert(double(readOnlyRetry.MsgID)==20,'m600check:ReadRetryOnly','Only read requests can be retried.');
                send(readOnlyRetry);lastReadSent=nowS();
            end
            pause(cfg.poll_period_s);
        end
        error('m600check:ParameterTimeout','No fresh parameter response for %s.',name);
    end
    function yes=closeAll()
        if closed,yes=true;return;end
        yes=true;
        for j=1:numel(subscriptions)
            try,subscriptions{j}.NewMessageFcn=[];delete(subscriptions{j});catch,yes=false;end
        end
        if ~isempty(canonicalExchange),canonicalExchange.close();end
        if ~isempty(rawTransport)
            try
                rawTransport.close();rawTransportFinal=rawTransport.status();
                yes=yes&&isequal(rawTransportFinal.closed,true);
            catch problem
                yes=false;
                try,rawTransportFinal=rawTransport.status();catch,rawTransportFinal=struct('closed',false,'error',problem.identifier);end
            end
        end
        if ~isempty(link)
            try
                if ~rawTransportEnabled,disconnect(link);end
                delete(link);
            catch,yes=false;end
        end
        if ~isempty(truthSocket),try,delete(truthSocket);catch,yes=false;end;end
        if ~isempty(timeSocket),try,delete(timeSocket);catch,yes=false;end;end
        if ~isempty(canonicalRotorSocket),try,delete(canonicalRotorSocket);catch,yes=false;end;end
        closed=yes;
    end
    function r=evidence()
        r=struct('subscribed_topics',{topics}, ...
            'automatic_mavlink_peer_ready',automaticPeerReady(), ...
            'startup_send_policy','BOARD_LOCAL_RAW_AUTOMATIC_TX_AFTER_SOURCE_MATCHED_VALID_HEARTBEAT__FAULT_LATCH_UNCHANGED', ...
            'same_existing_mavlinkio',~rawTransportEnabled,'same_existing_transport_owner',true, ...
            'transport_kind',transportKind,'additional_connections',0, ...
            'attitude_target_observation_policy', ...
            'OPTIONAL_RAW_ONLY__NO_STREAM_REQUEST__NO_CONTROL_OR_ADMISSION_DEPENDENCY', ...
            'raw_mavlink',{rawMav},'raw_truth_datagrams',{rawTruth},'raw_clock_datagrams',{rawTime}, ...
            'raw_forwarding_fragments',{rawForwardingFragments}, ...
            'health_dialect_path',healthDialectPath, ...
            'health_observation_policy','EXACT_PX4_EVENT_PROTOCOL_RAW__SYS_STATUS_IS_NOT_FULL_ARM_APPROVAL', ...
            'raw_transmit_messages',{rawTx},'raw_environment_transmit_datagrams',{rawEnvironmentTx}, ...
            'timesync_responses',syncResponses,'fixed_best_timesync',best, ...
            'time_startup_uninitialized_count',timeStartupUninitializedCount, ...
            'timesync_validation_state',clockValidation,'timesync_validation_responses',{clockValidationResponses}, ...
            'utc_zero_s',utcZero,'utc_pair_uncertainty_s',utcPairUncertainty, ...
            'utc_pair_calibration',utcPair,'coptersim_start_utc_s',startUtc, ...
            'clock_origin_changed',timeOriginChanged,'fatal',fatal,'first_fatal',firstFatal, ...
            'non_model_fatal',nonModelFatal,'model_diagnostic_observer',modelDiagnostic,'closed',closed, ...
            'delivery_environment_enabled',deliveryEnabled,'delivery_environment_contract',deliveryContract, ...
            'delivery_diagnostic_observer',deliveryDiagnostic,'delivery_last_decoded',deliveryLastDecoded, ...
            'delivery_board_continuity_epoch',deliveryContinuityEpoch,'delivery_board_transitions',deliveryTransitions, ...
            'raw_record_overflow_drops',rawDropped, ...
            'canonical_rotor_observer_enabled',canonicalRotorEnabled, ...
            'canonical_rotor_received',canonicalRotorReceived, ...
            'canonical_rotor_failure',canonicalRotorFailure, ...
            'raw_rotor_datagrams',{rawRotor}, ...
            'canonical_cache_observer_enabled',canonicalCacheEnabled, ...
            'canonical_cache_received',canonicalCacheReceived,'raw_cache_datagrams',{rawCache}, ...
            'canonical_cache_observer_state',canonicalCacheState,'canonical_cache_observation',canonicalCacheSample, ...
            'canonical_px4_observer_enabled',canonicalObserverEnabled, ...
            'canonical_px4_observer_state',canonicalObserver, ...
            'canonical_module_observer_enabled',canonicalModuleEnabled, ...
            'canonical_module_observer_state',canonicalModule, ...
            'canonical_module_send_in_progress',canonicalModuleSendInProgress, ...
            'canonical_module_deferred_receive_records',{canonicalModuleDeferred}, ...
            'canonical_module_deferred_receive_bytes',canonicalModuleDeferredBytes, ...
            'canonical_module_deferred_receive_total',canonicalModuleDeferredTotal, ...
            'canonical_module_deferred_receive_processed',canonicalModuleDeferredProcessed, ...
            'canonical_exchange_enabled',canonicalExchangeEnabled, ...
            'canonical_exchange_bind_attempted',canonicalExchangeBindAttempted, ...
            'canonical_exchange_expected',canonicalExchangeExpected, ...
            'canonical_exchange_association',canonicalExchangeAssociation, ...
            'canonical_exchange_failure',canonicalExchangeFailure, ...
            'canonical_exchange_unbound_received',canonicalExchangeUnboundReceived, ...
            'canonical_exchange_messages_sent',canonicalExchangeSent, ...
            'canonical_exchange_send_in_progress',canonicalExchangeSendInProgress, ...
            'canonical_completed_queue_capacity',canonicalCompletedCapacity, ...
            'canonical_completed_queues',{canonicalCompleted}, ...
            'canonical_completed_enqueued',canonicalCompletedEnqueued,'canonical_completed_taken',canonicalCompletedTaken, ...
            'canonical_session_last_send',canonicalSessionLastSend, ...
            'canonical_session_history',{canonicalSessionHistory}, ...
            'canonical_retirement_attempts',{canonicalRetirementAttempts}, ...
            'canonical_session_history_capacity',canonicalSessionHistoryCapacity, ...
            'real_parameter_actions',realParameterActions, ...
            'telemetry_source_observers',struct('local_position',localObserver, ...
                'attitude',attitudeObserver,'estimator_status',estimatorObserver), ...
            'timing_claim','MEASURED_INTERVAL_BOUNDED_APPROXIMATE_ALIGNMENT_NOT_EXACT_SIMULTANEITY');
        if ~isempty(canonicalExchange),r.canonical_exchange=canonicalExchange.evidence();end
        if ~isempty(canonicalLocalWindow),r.canonical_local_window=canonicalLocalWindow.evidence();end
        if rawTransportEnabled
            r.raw_transport=rawTransportFinal;
            if ~isempty(rawTransport),r.raw_transport=rawTransport.status();end
            r.raw_transport_replaces_mavlinkio_connection=true;
            r.actual_mavlink_subscription_count=numel(subscriptions);
        end
    end
end

function o=emptyObserver()
o=struct('payload',struct(),'raw_source_time_s',NaN,'rx_s',-Inf,'latest_rx_s',-Inf, ...
    'source_progress_rx_s',-Inf,'receive_count',0,'source_generation_count',0, ...
    'source_advance_count',0,'duplicate_count',0, ...
    'source_reversal_latched',false,'source_invalid_latched',false);
end

function bytes=datagramBytes(value,index)
if istable(value)
    data=value.Data;
    if iscell(data),bytes=data{index};else,bytes=data(index,:);end
else,bytes=value(index).Data;end
bytes=reshape(uint8(bytes),1,[]);
end
function value=readLE(bytes,type)
value=typecast(reshape(uint8(bytes),1,[]),type);[~,~,e]=computer;
if e=='B',value=swapbytes(value);end
end
function value=cleanText(value),value=char(strtrim(strrep(char(value),char(0),'')));end
function value=fieldText(text,n)
text=char(text);assert(numel(text)<=n,'m600check:TextLength','Field too long.');
value=repmat(char(0),1,n);value(1:numel(text))=text;
end
