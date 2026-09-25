function value=makePlantEnvironmentFrameFromSnapshot(snapshot,reference, ...
        payloadKg,payloadGeneration,releaseGeneration,frameGeneration, ...
        taskTimeS,taskClockPaused,lifecyclePhase,sessionToken,identityValid, ...
        heartbeatFresh,extendedStateFresh,canonicalPlantWind)
%MAKEPLANTENVIRONMENTFRAMEFROMSNAPSHOT Bind live PX4 state to model time.
% Receive timestamps are mapped into the CopterSim clock by subtracting
% their current host age from the latest model time. No plant state is fed
% back into PX4 control; the plant reference jet is caller supplied.
arguments
    snapshot (1,1) struct
    reference (1,1) struct
    payloadKg (1,1) double {mustBeFinite,mustBeNonnegative}
    payloadGeneration (1,1) double {mustBeInteger,mustBeNonnegative}
    releaseGeneration (1,1) double {mustBeInteger,mustBeNonnegative}
    frameGeneration (1,1) double {mustBeInteger,mustBePositive}
    taskTimeS (1,1) double {mustBeFinite,mustBeNonnegative}
    taskClockPaused (1,1) logical
    lifecyclePhase (1,1) string
    sessionToken (1,1) double {mustBeInteger,mustBePositive}
    identityValid (1,1) logical
    heartbeatFresh (1,1) logical
    extendedStateFresh (1,1) logical
    canonicalPlantWind (1,1) struct = struct()
end
required={'position_ned_m','velocity_ned_mps','acceleration_ned_mps2','jerk_ned_mps3'};
assert(all(isfield(reference,required)),'gpenmpcTaskIo:ReferenceSchema', ...
    'The reference must contain position, velocity, acceleration and jerk.');
jet=[reshape(double(reference.position_ned_m),3,1); ...
    reshape(double(reference.velocity_ned_mps),3,1); ...
    reshape(double(reference.acceleration_ned_mps2),3,1); ...
    reshape(double(reference.jerk_ned_mps3),3,1)];
assert(all(isfinite(jet)),'gpenmpcTaskIo:ReferenceFinite', ...
    'The reference jet must be finite.');
assert(all(isfield(snapshot,{'now_s','heartbeat_rx_s','extended_rx_s', ...
    'armed','landed_state','clock_valid','model_diagnostic', ...
    'delivery_board_continuity_epoch'})),'gpenmpcTaskIo:SnapshotSchema', ...
    'The board/model snapshot is missing a required field.');
assert(isfield(snapshot.model_diagnostic,'decoded')&& ...
    isstruct(snapshot.model_diagnostic.decoded)&& ...
    isfield(snapshot.model_diagnostic.decoded,'sim_time_s'), ...
    'gpenmpcTaskIo:ModelTimeMissing','The CopterSim model timestamp is unavailable.');
nowS=double(snapshot.now_s);modelS=double(snapshot.model_diagnostic.decoded.sim_time_s);
hb=double(snapshot.heartbeat_rx_s);ex=double(snapshot.extended_rx_s);
assert(all(isfinite([nowS,modelS,hb,ex]))&&modelS>=0&&nowS>=hb&&nowS>=ex, ...
    'gpenmpcTaskIo:ClockMappingInvalid','The host-to-model receive-time mapping is invalid.');
boardSourceS=max(0,modelS-(nowS-min(hb,ex)));
% Freshness is evaluated by the live owner against its independently frozen
% HEARTBEAT and EXTENDED_SYS_STATE receive-age bounds.  Never manufacture
% valid bits merely because the two receive timestamps exist.
flags=uint32(0);
flags=bitset(flags,1,heartbeatFresh);flags=bitset(flags,2,extendedStateFresh);
flags=bitset(flags,3,identityValid);flags=bitset(flags,4,logical(snapshot.clock_valid));
armed=double(snapshot.armed);
observedLanded=double(snapshot.landed_state);
assert(ismember(armed,[0,1])&&ismember(observedLanded,0:4), ...
    'gpenmpcTaskIo:BoardStateInvalid','The board arm/landed state is outside the MAVLink domain.');
% MAV_LANDED_STATE_LANDING=4 is valid PX4 telemetry. The unchanged ENV2
% model ABI only represents 0..3; conservatively encode LANDING as IN_AIR,
% never ON_GROUND. Retain the actual enum separately and in the raw snapshot.
% This grants no ground dwell/unload permission and changes no model physics.
modelLanded=observedLanded;
if observedLanded==4,modelLanded=2;end
flags=bitset(flags,5,armed==1);
value=struct('schema_version',2,'generation',frameGeneration, ...
    'source_io_time_s',modelS,'task_reference_time_s',taskTimeS, ...
    'payload_kg',payloadKg,'wind_ned_xy_mps',wind(reference,canonicalPlantWind), ...
    'mission_phase',phaseCode(lifecyclePhase), ...
    'reference_jet_ned',jet,'payload_generation',payloadGeneration, ...
    'task_clock_paused',taskClockPaused,'session_token',sessionToken, ...
    'board_min_rx_io_time_s',boardSourceS,'board_valid_flags',double(flags), ...
    'landed_state',modelLanded,'observed_mavlink_landed_state',observedLanded, ...
    'continuity_epoch',double(snapshot.delivery_board_continuity_epoch), ...
    'service_release_generation',releaseGeneration);
end

function value=wind(reference,canonical)
% Empty final argument preserves the historical estimate-or-zero interface.
% Explicit canonical calls MUST supply actual plant wind; never fall back.
if ~isempty(fieldnames(canonical))
    assert(all(isfield(canonical,{'schema','actual_wind_ned_xy_mps'})) ...
        &&strcmp(canonical.schema,'RFLY_EXPLICIT_CANONICAL_PLANT_WIND_V1') ...
        &&isa(canonical.actual_wind_ned_xy_mps,'double') ...
        &&isreal(canonical.actual_wind_ned_xy_mps)&&isequal(size(canonical.actual_wind_ned_xy_mps),[2,1]) ...
        &&all(isfinite(canonical.actual_wind_ned_xy_mps)), ...
        'gpenmpcTaskIo:CanonicalActualWind','Explicit finite actual plant wind is required.');
    value=canonical.actual_wind_ned_xy_mps;return
end
value=zeros(2,1);
if isfield(reference,'estimated_wind_ned_mps')
    x=double(reference.estimated_wind_ned_mps(:));
    assert(numel(x)>=2&&all(isfinite(x(1:2))),'gpenmpcTaskIo:WindFinite', ...
        'The horizontal wind estimate must contain two finite components.');
    value=x(1:2);
end
end

function value=phaseCode(name)
switch upper(char(name))
    case 'PREARM',value=0;
    case 'INITIAL_TAKEOFF',value=1;
    case 'TRANSIT',value=2;
    case 'DELIVERY_DESCENT',value=3;
    case {'NATIVE_LAND','FINAL_NATIVE_LAND'},value=4;
    case {'GROUND_CONFIRM','SERVICE_GROUNDED_DISARMED'},value=5;
    case {'WAIT_OFFBOARD','REARM_ADMISSION','SERVICE_ASCENT','COMPLETE'},value=6;
    otherwise,error('gpenmpcTaskIo:LifecyclePhase','Unknown lifecycle phase %s',name);
end
end
