function [s,r]=advanceCanonicalPx4Observation(s,op,e)
%ADVANCECANONICALPX4OBSERVATION Pure single-owner decoded-message reducer.
% INIT: verified_binding (uid/system_id/component_id/boot_generation/verified),
% maximum_queue, maximum_age_ns, now_ns. MESSAGE: message, rx_ns. SNAPSHOT and
% DRAIN: now_ns. DRAIN clears queued records, returns only fresh records and
% explicitly counts expired records. No receive/source times are rewritten.
% MATLAB deserializemsg/callback fields: MsgID,SystemID,ComponentID,Seq,Payload.
% ODOMETRY is one atomic PX4 estimate: no LOCAL_POSITION+ATTITUDE assembly,
% task-origin translation, clock fitting, interpolation or sample manufacture.
% Receive age and source gaps describe stream timing independently of
% source-to-host clock alignment. No exact-10ms observer admission gate.
% UID/boot identity is supplied by an independently verified caller binding.
% This function cannot itself prove that a received message is authenticated.
% Quaternion norm tolerance 1e-5 retains the existing LiveBoardState numeric
% representation check.
op=upper(string(op));
assert(isscalar(op)&&isstruct(e)&&isscalar(e), ...
    'm600check:CanonicalObservationEvent','Scalar operation/event required.');
r=receipt(char(op));
if isempty(s)
    assert(op=="INIT",'m600check:CanonicalObservationInit','Initialize once per verified owner.');
    b=validateInit(e);
    stream=struct('generation',0,'last_source_time_ns',NaN,'last_rx_ns',NaN, ...
        'last_payload',[],'last_sample',[],'queue',{{}}, ...
        'duplicates',0,'expired_queue_samples',0,'max_source_gap_ns',0,'max_receive_gap_ns',0);
    s=struct('schema','CANONICAL_PX4_ATOMIC_OBSERVER_V1','binding',b, ...
        'maximum_queue',double(e.maximum_queue),'maximum_age_ns',double(e.maximum_age_ns), ...
        'quaternion_norm_tolerance',1e-5,'last_event_ns',double(e.now_ns), ...
        'fatal_latched',false,'first_failure',[],'failure_count',0, ...
        'message_count',0,'ignored_message_count',0,'accepted_message_count',0, ...
        'odometry',stream,'actuator',stream);
    r.reason='INITIALIZED_FROM_EXPLICIT_VERIFIED_BINDING';
    r.snapshot=snapshot(s,double(e.now_ns));return
end
assert(isstruct(s)&&isscalar(s)&&isfield(s,'schema')&& ...
    strcmp(s.schema,'CANONICAL_PX4_ATOMIC_OBSERVER_V1'), ...
    'm600check:CanonicalObservationState','Existing observer state required.');
if op=="INIT"
    [s,r]=fail(s,r,'OWNER_REINITIALIZATION_FORBIDDEN',e);return
end
if op=="MESSAGE",timeField='rx_ns';else,timeField='now_ns';end
if ~isfield(e,timeField)||~finiteScalar(e.(timeField))||e.(timeField)<0
    [s,r]=fail(s,r,'HOST_TIMESTAMP_INVALID',e);return
end
now=double(e.(timeField));
if now<s.last_event_ns
    [s,r]=fail(s,r,'HOST_TIMESTAMP_REVERSED',e);return
end
s.last_event_ns=now;
if s.fatal_latched
    r.reason='ALREADY_FATAL_NO_NEW_SAMPLE';r.snapshot=snapshot(s,now);
    r.fatal_latched=true;return
end
switch op
    case 'MESSAGE'
        s.message_count=s.message_count+1;
        if ~isfield(e,'message')||~isstruct(e.message)||~isscalar(e.message)
            [s,r]=fail(s,r,'DECODED_MESSAGE_SCHEMA_INVALID',e);return
        end
        m=e.message;
        if ~isfield(m,'MsgID')||~integerIn(m.MsgID,0,16777215)
            [s,r]=fail(s,r,'MESSAGE_ID_INVALID',e);return
        end
        if ~ismember(double(m.MsgID),[331,93])
            s.ignored_message_count=s.ignored_message_count+1;
            r.reason='NON_TARGET_MESSAGE_NOT_AN_ESTIMATE';r.snapshot=snapshot(s,now);return
        end
        if ~all(isfield(m,{'SystemID','ComponentID','Seq','Payload'}))|| ...
                ~integerIn(m.SystemID,1,255)||~integerIn(m.ComponentID,1,255)|| ...
                ~integerIn(m.Seq,0,255)||~isstruct(m.Payload)||~isscalar(m.Payload)
            [s,r]=fail(s,r,'DECODED_SOURCE_OR_PAYLOAD_INVALID',e);return
        end
        if double(m.SystemID)~=s.binding.system_id||double(m.ComponentID)~=s.binding.component_id
            [s,r]=fail(s,r,'MESSAGE_SOURCE_IDENTITY_MISMATCH',e);return
        end
        p=m.Payload;
        if ~isfield(p,'time_usec')||~integerIn(p.time_usec,1,floor(flintmax/1000))
            [s,r]=fail(s,r,'SOURCE_TIMESTAMP_INVALID_OR_PRECISION_UNSAFE',e);return
        end
        if double(m.MsgID)==331
            kind='odometry';why=validateOdometry(p,s.quaternion_norm_tolerance);
        else
            kind='actuator';why=validateActuator(p);
        end
        if ~isempty(why),[s,r]=fail(s,r,why,e);return,end
        previous=s.(kind);sourceNs=double(p.time_usec)*1000;
        if previous.generation>0
            if strcmp(kind,'odometry')&&p.reset_counter~=previous.last_payload.reset_counter
                [s,r]=fail(s,r,'ODOMETRY_RESET_COUNTER_CHANGED',e);return
            end
            if sourceNs<previous.last_source_time_ns
                [s,r]=fail(s,r,'SOURCE_TIMESTAMP_REVERSED',e);return
            elseif sourceNs==previous.last_source_time_ns
                if ~isequaln(p,previous.last_payload)
                    [s,r]=fail(s,r,'DUPLICATE_TIMESTAMP_PAYLOAD_CHANGED',e);return
                end
                s.(kind).duplicates=s.(kind).duplicates+1;
                r.duplicate=true;r.reason='DUPLICATE_NO_GENERATION_OR_FRESHNESS_CREDIT';
                r.snapshot=snapshot(s,now);return
            end
        end
        if numel(previous.queue)>=s.maximum_queue
            [s,r]=fail(s,r,'BOUNDED_QUEUE_OVERFLOW_NO_OVERWRITE',e);return
        end
        generation=previous.generation+1;
        sourceGap=sourceNs-previous.last_source_time_ns;rxGap=now-previous.last_rx_ns;
        if strcmp(kind,'odometry')
            value=odometrySample(m,s.binding,now,sourceNs,generation,sourceGap,rxGap);
        else
            value=actuatorSample(m,s.binding,now,sourceNs,generation,sourceGap,rxGap);
        end
        s.(kind).generation=generation;s.(kind).last_source_time_ns=sourceNs;
        s.(kind).last_rx_ns=now;s.(kind).last_payload=p;s.(kind).last_sample=value;
        s.(kind).queue{end+1}=value;
        if isfinite(sourceGap),s.(kind).max_source_gap_ns=max(previous.max_source_gap_ns,sourceGap);end
        if isfinite(rxGap),s.(kind).max_receive_gap_ns=max(previous.max_receive_gap_ns,rxGap);end
        s.accepted_message_count=s.accepted_message_count+1;
        r.accepted=true;r.kind=kind;r.sample=value;r.reason='ATOMIC_SOURCE_SAMPLE_ACCEPTED';
    case 'DRAIN'
        for kind={'odometry','actuator'}
            name=kind{1};queue=s.(name).queue;fresh=false(size(queue));
            for k=1:numel(queue)
                age=now-queue{k}.rx_ns;fresh(k)=age>=0&&age<=s.maximum_age_ns;
            end
            r.([name '_queue'])=queue(fresh);
            r.([name '_expired_count'])=nnz(~fresh);
            s.(name).expired_queue_samples=s.(name).expired_queue_samples+nnz(~fresh);
            s.(name).queue={};
        end
        r.reason='QUEUE_DRAINED_FRESH_ONLY_NO_BACKFILL';
    case 'SNAPSHOT'
        r.reason='OBSERVED_SNAPSHOT_NO_GENERATION';
    otherwise
        [s,r]=fail(s,r,'OPERATION_INVALID',e);return
end
r.snapshot=snapshot(s,now);r.fatal_latched=s.fatal_latched;
end

function b=validateInit(e)
assert(all(isfield(e,{'verified_binding','maximum_queue','maximum_age_ns','now_ns'}))&& ...
    isstruct(e.verified_binding)&&isscalar(e.verified_binding), ...
    'm600check:CanonicalObservationBinding','Explicit identity and queue/age bounds required.');
b=e.verified_binding;
assert(all(isfield(b,{'uid','system_id','component_id','boot_generation','verified'}))&& ...
    islogical(b.verified)&&isscalar(b.verified)&&b.verified&& ...
    integerIn(b.system_id,1,255)&&integerIn(b.component_id,1,255)&& ...
    integerIn(b.boot_generation,0,flintmax), ...
    'm600check:CanonicalObservationBinding','Unambiguous verified source/boot identity required.');
if isa(b.uid,'uint64')&&isscalar(b.uid)&&b.uid>0
    uid=char(string(b.uid));
elseif (ischar(b.uid)&&isrow(b.uid))||(isstring(b.uid)&&isscalar(b.uid))
    uid=char(b.uid);
elseif isa(b.uid,'double')&&integerIn(b.uid,1,flintmax)
    uid=sprintf('%.0f',b.uid);
else
    error('m600check:CanonicalObservationBinding','UID must be a lossless decimal identity.');
end
assert(~isempty(regexp(uid,'^[1-9][0-9]{0,19}$','once'))&& ...
    (numel(uid)<20||strcmp(uid,'18446744073709551615')|| ...
    firstLexLess(uid,'18446744073709551615')), ...
    'm600check:CanonicalObservationBinding','UID is not a nonzero uint64 decimal identity.');
assert(integerIn(e.maximum_queue,1,4096)&&finiteScalar(e.maximum_age_ns)&&e.maximum_age_ns>0&& ...
    finiteScalar(e.now_ns)&&e.now_ns>=0, ...
    'm600check:CanonicalObservationConfiguration','Explicit bounded queue and receive-age policy required.');
b=struct('uid',uid,'system_id',double(b.system_id),'component_id',double(b.component_id), ...
    'boot_generation',double(b.boot_generation),'verified',true);
end
function yes=firstLexLess(a,b)
k=find(a~=b,1);yes=~isempty(k)&&a(k)<b(k);
end
function why=validateOdometry(p,tolerance)
why='';
scalars={'x','y','z','vx','vy','vz','rollspeed','pitchspeed','yawspeed'};
needed=[scalars,{'q','frame_id','child_frame_id','estimator_type','reset_counter', ...
    'pose_covariance','velocity_covariance'}];
if ~all(isfield(p,needed)),why='ODOMETRY_REQUIRED_FIELDS_MISSING';return,end
for k=1:numel(scalars)
    if ~finiteScalar(p.(scalars{k})),why='ODOMETRY_STATE_NONFINITE_OR_SHAPE';return,end
end
if ~numericVector(p.q,4)||~all(isfinite(p.q(:)))||abs(norm(double(p.q(:)))-1)>tolerance
    why='ODOMETRY_QUATERNION_INVALID';return
end
if ~integerIn(p.frame_id,1,1)||~integerIn(p.child_frame_id,1,1)|| ...
        ~integerIn(p.estimator_type,8,8)||~integerIn(p.reset_counter,0,255)
    why='ODOMETRY_FRAME_ESTIMATOR_OR_RESET_INVALID';return
end
for name={'pose_covariance','velocity_covariance'}
    if ~numericVector(p.(name{1}),21)||any(isinf(p.(name{1})(:)))
        why='ODOMETRY_COVARIANCE_SHAPE_OR_INFINITY';return
    end
end
allowed=[needed,{'time_usec','quality'}];
if ~all(ismember(fieldnames(p),allowed)),why='ODOMETRY_UNEXPECTED_PAYLOAD_FIELD';return,end
if isfield(p,'quality')&&~integerIn(p.quality,-1,100),why='ODOMETRY_QUALITY_INVALID';end
end
function why=validateActuator(p)
why='';
if ~all(isfield(p,{'controls','mode','flags'}))||~numericVector(p.controls,16)|| ...
        ~all(isfinite(p.controls(1:6)))||any(isinf(p.controls(7:16)))|| ...
        ~integerIn(p.mode,0,255)||~(isa(p.flags,'uint64')&&isscalar(p.flags))|| ...
        ~all(ismember(fieldnames(p),{'time_usec','controls','mode','flags'}))
    why='HIL_ACTUATOR_REQUIRED_CHANNELS_OR_PAYLOAD_INVALID';
end
end
function x=odometrySample(m,b,rx,t,g,sg,rg)
p=m.Payload;x=commonSample(m,b,rx,t,g,sg,rg);
x.source='PX4_EKF2_MAVLINK_ODOMETRY_331';x.atomic_estimate=true;x.plant_truth_used=false;
x.coordinate_origin='PX4_LOCAL_NED_UNTRANSLATED';x.task_origin_added=false;
x.odometry_frame_id=double(p.frame_id);x.odometry_child_frame_id=double(p.child_frame_id);
x.odometry_estimator_type=double(p.estimator_type);x.odometry_reset_counter=double(p.reset_counter);
x.position_ned_m=double([p.x;p.y;p.z]);x.velocity_ned_mps=double([p.vx;p.vy;p.vz]);
x.quaternion_wxyz_body_to_ned=double(p.q(:));
% PX4 v1.16 streams/ODOMETRY.hpp explicitly copies current body rates.
x.omega_frd_rad_s=double([p.rollspeed;p.pitchspeed;p.yawspeed]);
x.position_rx_ns=rx;x.attitude_rx_ns=rx;x.rates_rx_ns=rx;
x.position_generation=g;x.attitude_generation=g;x.rates_generation=g;
x.position_valid=true;x.attitude_valid=true;x.rates_valid=true;
x.pose_covariance=p.pose_covariance;x.velocity_covariance=p.velocity_covariance;
end
function x=actuatorSample(m,b,rx,t,g,sg,rg)
x=commonSample(m,b,rx,t,g,sg,rg);x.source='PX4_HIL_ACTUATOR_CONTROLS_93';
x.mavpackettype='HIL_ACTUATOR_CONTROLS';x.controls=double(m.Payload.controls(:));
x.mode=m.Payload.mode;x.flags=m.Payload.flags;
x.required_six_channels_finite=true;x.unused_channels_nan=isnan(x.controls(7:16));
end
function x=commonSample(m,b,rx,t,g,sg,rg)
x=struct('uid',b.uid,'boot_generation',b.boot_generation, ...
    'system_id',double(m.SystemID),'component_id',double(m.ComponentID), ...
    'src_system',double(m.SystemID),'src_component',double(m.ComponentID), ...
    'message_id',double(m.MsgID),'wire_sequence',double(m.Seq), ...
    'time_usec',m.Payload.time_usec,'source_time_ns',t,'sample_timestamp_ns',t, ...
    'rx_ns',rx,'generation',g,'source_generation',g,'observation_generation',g, ...
    'source_gap_ns',sg,'receive_gap_ns',rg,'source_identity_verified_binding',true);
end
function r=receipt(op)
r=struct('operation',op,'accepted',false,'duplicate',false,'kind','','reason','', ...
    'sample',[],'snapshot',[],'odometry_queue',{{}},'actuator_queue',{{}}, ...
    'odometry_expired_count',0,'actuator_expired_count',0,'fatal_latched',false);
end
function out=snapshot(s,now)
out=struct('binding',s.binding,'fatal_latched',s.fatal_latched,'first_failure',s.first_failure, ...
    'failure_count',s.failure_count,'message_count',s.message_count, ...
    'accepted_message_count',s.accepted_message_count,'ignored_message_count',s.ignored_message_count, ...
    'maximum_queue',s.maximum_queue,'maximum_age_ns',s.maximum_age_ns, ...
    'age_basis','HOST_RECEIVE_AGE__SOURCE_CLOCK_ALIGNMENT_EXTERNAL','now_ns',now);
for kind={'odometry','actuator'}
    n=kind{1};v=s.(n);present=~isempty(v.last_sample);age=now-v.last_rx_ns;
    fresh=present&&~s.fatal_latched&&isfinite(age)&&age>=0&&age<=s.maximum_age_ns;
    out.(n)=[];if fresh,out.(n)=v.last_sample;end
    out.([n '_present'])=present;out.([n '_fresh'])=fresh;out.([n '_age_ns'])=age;
    out.([n '_generation'])=v.generation;out.([n '_queue_count'])=numel(v.queue);
    queuedExpired=0;
    for k=1:numel(v.queue)
        queuedAge=now-v.queue{k}.rx_ns;
        queuedExpired=queuedExpired+double(queuedAge<0||queuedAge>s.maximum_age_ns);
    end
    out.([n '_queued_expired_count'])=queuedExpired;
    out.([n '_duplicates'])=v.duplicates;out.([n '_expired_total'])=v.expired_queue_samples;
    out.([n '_last_source_time_ns'])=v.last_source_time_ns;
    out.([n '_last_rx_ns'])=v.last_rx_ns;
    out.([n '_max_source_gap_ns'])=v.max_source_gap_ns;
    out.([n '_max_receive_gap_ns'])=v.max_receive_gap_ns;
end
end
function [s,r]=fail(s,r,why,e)
s.failure_count=s.failure_count+1;
if ~s.fatal_latched
    s.fatal_latched=true;f=struct('reason',why,'event_ns',s.last_event_ns, ...
        'message_id',NaN,'system_id',NaN,'component_id',NaN);
    if isfield(e,'rx_ns')&&finiteScalar(e.rx_ns),f.event_ns=double(e.rx_ns);
    elseif isfield(e,'now_ns')&&finiteScalar(e.now_ns),f.event_ns=double(e.now_ns);end
    if isfield(e,'message')&&isstruct(e.message)&&isscalar(e.message)
        for pair={'MsgID','message_id';'SystemID','system_id';'ComponentID','component_id'}.'
            if isfield(e.message,pair{1})&&finiteScalar(e.message.(pair{1}))
                f.(pair{2})=double(e.message.(pair{1}));
            end
        end
    end
    s.first_failure=f;
end
r.reason=why;r.fatal_latched=true;r.snapshot=snapshot(s,s.last_event_ns);
end
function yes=finiteScalar(x)
yes=isnumeric(x)&&isreal(x)&&isscalar(x)&&isfinite(x);
end
function yes=integerIn(x,low,high)
yes=finiteScalar(x)&&double(x)>=low&&double(x)<=high&&double(x)==fix(double(x));
end
function yes=numericVector(x,n)
yes=isnumeric(x)&&isreal(x)&&isvector(x)&&numel(x)==n;
end
