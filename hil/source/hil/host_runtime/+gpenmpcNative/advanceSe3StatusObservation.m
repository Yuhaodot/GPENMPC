function [s,r]=advanceSe3StatusObservation(s,op,e)
% ADVANCESE3STATUSOBSERVATION Advance a read-only NSH listener transaction.
% INIT takes verified_binding and now_ns; later events include the current boot_generation.
% TICK may return tx_message. TX_RESULT records the owner's send result.
% MESSAGE accepts decoded SERIAL_CONTROL with the original rx_ns.
% Optional processing_now_ns records dispatch delay without renewing receipt time.
% GUARD and SNAPSHOT are read-only. The command is listener gpenmpc_se3_control_status 0 1.
% Limits: interval 50 ms, timeout 90 ms, guard 100 ms, buffer 32768 bytes.
% Guard age includes time since request preparation because printing time is not host-aligned.
op=upper(string(op));assert(isscalar(op)&&isstruct(e)&&isscalar(e), ...
    'gpenmpcNative:Se3ObserverEvent','Scalar operation/event required.');
r=emptyReceipt(char(op));
if isempty(s)
    assert(op=="INIT",'gpenmpcNative:Se3ObserverInit','INIT required.');
    b=verifiedBinding(e);assert(ns(e.now_ns),'gpenmpcNative:Se3ObserverTime','Exact host nanoseconds required.');
    s=struct('schema','MATLAB_PURE_SE3_READONLY_LISTENER_V1','binding',b, ...
        'query','listener gpenmpc_se3_control_status 0 1', ...
        'query_interval_ns',50e6,'query_timeout_ns',90e6,'maximum_guard_age_ns',100e6, ...
        'maximum_response_bytes',32768,'last_event_ns',double(e.now_ns), ...
        'query_open',false,'tx_confirmed',false,'request_generation',0,'query_started_ns',NaN, ...
        'next_query_ns',0,'buffer',uint8([]),'first_chunk_rx_ns',NaN, ...
        'last_chunk',[],'latest',[],'latest_response',[], ...
        'fatal_latched',false,'first_failure',[],'failure_count',0, ...
        'queries_prepared',0,'send_attempts',0,'queries_sent',0, ...
        'responses_completed',0,'chunks_received',0,'bytes_received',0, ...
        'duplicate_chunks',0,'duplicate_status_responses',0,'status_generation',0,'guards_issued',0, ...
        'inactive_reset_increment_total',0,'inactive_reset_increment_observations',0);
    r.reason='INITIALIZED_VERIFIED_SOURCE';r.snapshot=status(s,double(e.now_ns));return
end
assert(isstruct(s)&&isscalar(s)&&isfield(s,'schema')&& ...
    strcmp(s.schema,'MATLAB_PURE_SE3_READONLY_LISTENER_V1'), ...
    'gpenmpcNative:Se3ObserverState','Existing state required.');
if op=="INIT",[s,r]=fail(s,r,'OWNER_REINITIALIZATION_FORBIDDEN');return,end
timeField='now_ns';if op=="MESSAGE",timeField='rx_ns';end
if ~isfield(e,timeField)||~ns(e.(timeField))
    [s,r]=fail(s,r,'INVALID_MONOTONIC_TIME');return
end
now=double(e.(timeField));received=now;
if op=="MESSAGE"&&isfield(e,'processing_now_ns')
    if ~ns(e.processing_now_ns)||double(e.processing_now_ns)<received
        [s,r]=fail(s,r,'INVALID_DEFERRED_PROCESSING_TIME');return
    end
    now=double(e.processing_now_ns);
end
if now<s.last_event_ns,[s,r]=fail(s,r,'HOST_TIME_REVERSED');return,end
s.last_event_ns=now;
if ~isfield(e,'boot_generation')||~integer(e.boot_generation,0,flintmax)|| ...
        double(e.boot_generation)~=s.binding.boot_generation
    [s,r]=fail(s,r,'VERIFIED_BOOT_GENERATION_CHANGED_OR_ABSENT');return
end
if s.fatal_latched,r.reason='ALREADY_FATAL_NO_QUERY_OR_GUARD';r.fatal_latched=true;r.snapshot=status(s,now);return,end
if any(isfield(e,{'query','command','data','tx_message','payload'}))
    [s,r]=fail(s,r,'ARBITRARY_QUERY_INPUT_FORBIDDEN');return
end
if s.query_open&&now-s.query_started_ns>s.query_timeout_ns
    % Record a late send return before latching the query deadline.
    if op=="TX_RESULT"&&sendReceiptValid(s,e)
        s.send_attempts=s.send_attempts+1;
        if e.send_succeeded,s.queries_sent=s.queries_sent+1;s.tx_confirmed=true;end
    end
    [s,r]=fail(s,r,'STATUS_QUERY_TIMEOUT');return
end
switch op
    case 'TICK'
        if s.query_open,r.reason='ONE_QUERY_ALREADY_PENDING';
        elseif now<s.next_query_ns,r.reason='QUERY_INTERVAL_NOT_DUE';
        else
            bytes=uint8([newline,s.query,newline]);data=zeros(1,70,'uint8');data(1:numel(bytes))=bytes;
            r.tx_message=struct('MsgID',uint32(126),'Payload',struct( ...
                'device',uint8(10),'flags',uint8(6),'timeout',uint16(0), ...
                'baudrate',uint32(0),'count',uint8(numel(bytes)),'data',data));
            s.request_generation=s.request_generation+1;s.queries_prepared=s.queries_prepared+1;
            s.query_open=true;s.tx_confirmed=false;s.query_started_ns=now;
            s.buffer=uint8([]);s.first_chunk_rx_ns=NaN;s.last_chunk=[];
            r.request_generation=s.request_generation;r.query_prepared=true;
            r.reason='FIXED_READONLY_QUERY_PREPARED_NOT_SENT';
        end
    case 'TX_RESULT'
        if ~sendReceiptValid(s,e)
            [s,r]=fail(s,r,'SEND_RECEIPT_CONTRACT_MISMATCH');return
        end
        s.send_attempts=s.send_attempts+1;
        if ~e.send_succeeded,[s,r]=fail(s,r,'STATUS_QUERY_SEND_FAILED');return,end
        s.tx_confirmed=true;s.queries_sent=s.queries_sent+1;r.reason='OWNER_SEND_RETURN_RECORDED';
    case 'MESSAGE'
        if ~isfield(e,'message')||~isstruct(e.message)||~isscalar(e.message)|| ...
                ~all(isfield(e.message,{'MsgID','SystemID','ComponentID','Seq','Payload'}))
            [s,r]=fail(s,r,'SERIAL_CONTROL_MESSAGE_SCHEMA');return
        end
        m=e.message;
        if ~integer(m.MsgID,126,126),[s,r]=fail(s,r,'NON_SERIAL_CONTROL_DISPATCH');return,end
        if ~integer(m.SystemID,1,255)||~integer(m.ComponentID,1,255)|| ...
                double(m.SystemID)~=s.binding.system_id||double(m.ComponentID)~=s.binding.component_id
            [s,r]=fail(s,r,'SERIAL_CONTROL_SOURCE_MISMATCH');return
        end
        p=m.Payload;
        if ~isstruct(p)||~isscalar(p)||~all(isfield(p,{'device','flags','count','data','timeout','baudrate'}))|| ...
                ~integer(m.Seq,0,255)||~integer(p.device,0,255)||~integer(p.flags,0,255)|| ...
                ~integer(p.count,0,70)||~isa(p.data,'uint8')||~isvector(p.data)||numel(p.data)~=70|| ...
                ~integer(p.timeout,0,65535)||~integer(p.baudrate,0,4294967295)|| ...
                ~all(ismember(fieldnames(p),{'device','flags','count','data','timeout','baudrate','target_system','target_component'}))
            [s,r]=fail(s,r,'SERIAL_CONTROL_PAYLOAD_INVALID');return
        end
        if p.device~=10,r.reason='NON_SHELL_DEVICE_IGNORED';r.snapshot=status(s,now);return,end
        if p.flags~=1||p.timeout~=0||p.baudrate~=0
            [s,r]=fail(s,r,'SERIAL_CONTROL_REPLY_FLAGS_INVALID');return
        end
        if p.count==0,r.reason='EMPTY_REPLY_NO_CREDIT';r.snapshot=status(s,now);return,end
        if ~s.query_open||~s.tx_confirmed
            [s,r]=fail(s,r,'UNSOLICITED_OR_UNCONFIRMED_SHELL_RESPONSE');return
        end
        bytes=reshape(p.data(1:double(p.count)),1,[]);
        chunk=struct('Seq',m.Seq,'Payload',p);
        if isequaln(chunk,s.last_chunk)
            s.duplicate_chunks=s.duplicate_chunks+1;r.reason='DUPLICATE_CHUNK_NO_APPEND';
            r.snapshot=status(s,now);return
        end
        s.last_chunk=chunk;s.chunks_received=s.chunks_received+1;s.bytes_received=s.bytes_received+double(p.count);
        if numel(s.buffer)+numel(bytes)>s.maximum_response_bytes
            [s,r]=fail(s,r,'STATUS_RESPONSE_OVERSIZE');return
        end
        s.buffer=[s.buffer,bytes];if isnan(s.first_chunk_rx_ns),s.first_chunk_rx_ns=received;end
        % Validate a topic only after its line is complete. UART/MAVLink may
        % split anywhere, including in the middle of the topic name.
        text=clean(s.buffer);headers=regexp(text,'(?m)^[ \t]*TOPIC:[ \t]*([^\n]+)\n','tokens');
        if numel(headers)>1,[s,r]=fail(s,r,'MULTIPLE_STATUS_RESPONSES_AMBIGUOUS');return,end
        if numel(headers)==1
            if isempty(regexp(strtrim(headers{1}{1}), ...
                    '^gpenmpc_se3_control_status(?: instance 0 #1)?$','once'))
                [s,r]=fail(s,r,'STATUS_TOPIC_OR_INSTANCE_MISMATCH');return
            end
            at=strfind(text,'TOPIC:');after=text(at(1):end);
            if ~isempty(regexp(after,'(?m)^\s*nsh>\s*$','once'))
                [atom,why]=parseAtom(text);
                if ~isempty(why),[s,r]=fail(s,r,why);return,end
                atom.source='PX4_UORB_GPENMPC_SE3_CONTROL_STATUS';
                atom.transport='PX4_SERIAL_CONTROL_NSH_LISTENER';atom.plant_truth_used=false;
                atom.uid=s.binding.uid;atom.system_id=double(m.SystemID);atom.component_id=double(m.ComponentID);
                atom.boot_generation=s.binding.boot_generation;atom.rx_ns=received;
                atom.first_chunk_rx_ns=s.first_chunk_rx_ns;atom.query_started_ns=s.query_started_ns;
                atom.request_generation=s.request_generation;atom.final_wire_sequence=double(m.Seq);
                atom.raw_text=text;atom.raw_bytes=s.buffer;atom.valid=false;
                duplicate=false;
                if ~isempty(s.latest)
                    if atom.timestamp<s.latest.timestamp,[s,r]=fail(s,r,'STATUS_TIMESTAMP_REVERSED');return,end
                    for field={'input_sample_count','output_publish_count','rejected_sample_count'}
                        if atom.(field{1})<s.latest.(field{1})
                            [s,r]=fail(s,r,'STATUS_COUNTER_REGRESSION');return
                        end
                    end
                    if atom.reset_count<s.latest.reset_count
                        [s,r]=fail(s,r,'STATUS_RESET_COUNT_REGRESSION');return
                    elseif atom.reset_count>s.latest.reset_count
                        % PX4 resets the controller while disarmed or outside Offboard.
                        % Treat that as lifecycle state, not an active nominal guard or boot reset.
                        if atom.active&&atom.control_mode==1&&s.latest.active&&s.latest.control_mode==1
                            [s,r]=fail(s,r,'CONTINUOUS_ACTIVE_MODE1_RESET_COUNT_CHANGED');return
                        end
                        s.inactive_reset_increment_total=s.inactive_reset_increment_total+atom.reset_count-s.latest.reset_count;
                        s.inactive_reset_increment_observations=s.inactive_reset_increment_observations+1;
                    end
                    if atom.timestamp==s.latest.timestamp
                        [numberNames,booleanNames,vectorNames]=fieldNames();fields=[numberNames,booleanNames,vectorNames];
                        duplicate=all(cellfun(@(n)isequaln(atom.(n),s.latest.(n)),fields));
                        if ~duplicate,[s,r]=fail(s,r,'SAME_TIMESTAMP_CHANGED_STATUS');return,end
                    end
                end
                if duplicate
                    s.duplicate_status_responses=s.duplicate_status_responses+1;
                    atom.status_generation=s.status_generation;
                    r.reason='DUPLICATE_STATUS_NO_GENERATION_OR_RX_RENEWAL';
                else
                    s.status_generation=s.status_generation+1;atom.status_generation=s.status_generation;
                    s.latest=atom;r.reason='COMPLETE_STATUS_OBSERVED';
                end
                s.latest_response=atom;s.responses_completed=s.responses_completed+1;
                s.query_open=false;s.tx_confirmed=false;s.next_query_ns=now+s.query_interval_ns;
                s.buffer=uint8([]);r.completed=true;r.atom=atom;
            else,r.reason='BOUNDED_PARTIAL_RESPONSE';end
        elseif contains(text,'never published')||contains(text,'command not found')
            [s,r]=fail(s,r,'STATUS_NOT_PUBLISHED_OR_COMMAND_UNAVAILABLE');return
        else,r.reason='AWAITING_STATUS_TOPIC';end
    case {'GUARD','SNAPSHOT'}
        r.reason='READONLY_STATUS';
    otherwise
        [s,r]=fail(s,r,'UNKNOWN_OPERATION');return
end
[r.guard,r.guard_reason]=activeGuard(s,now);
if op=="GUARD"&&~isempty(r.guard),s.guards_issued=s.guards_issued+1;end
r.snapshot=status(s,now);r.fatal_latched=s.fatal_latched;
end

function [a,why]=parseAtom(text)
a=struct();why='';[numbers,booleans,vectors]=fieldNames();allNames=[numbers,booleans,vectors];
rows=regexp(text,'(?m)^\s*([A-Za-z_][A-Za-z_0-9]*):\s*([^\n]+)','tokens');
observed={};values={};
for k=1:numel(rows)
    name=rows{k}{1};if strcmp(name,'TOPIC'),continue,end
    if ~ismember(name,allNames),why='STATUS_UNKNOWN_FIELD';return,end
    if ismember(name,observed),why='STATUS_DUPLICATE_FIELD';return,end
    observed{end+1}=name;values{end+1}=strtrim(rows{k}{2}); %#ok<AGROW>
end
if ~all(ismember(allNames,observed)),why='STATUS_REQUIRED_FIELD_MISSING_OR_TRUNCATED';return,end
for k=1:numel(allNames)
    name=allNames{k};value=values{strcmp(observed,name)};
    if ismember(name,booleans)
        if strcmpi(value,'true'),a.(name)=true;
        elseif strcmpi(value,'false'),a.(name)=false;
        else,why='STATUS_BOOLEAN_UNKNOWN';return,end
    elseif ismember(name,vectors)
        token=regexp(value,'^\[([^\]]+)\]$','tokens','once');
        if isempty(token),why='STATUS_VECTOR_SHAPE_OR_NONFINITE';return,end
        x=str2double(split(string(token{1}),','));
        if numel(x)~=3||~all(isfinite(x)),why='STATUS_VECTOR_SHAPE_OR_NONFINITE';return,end
        a.(name)=reshape(x,3,1);
    else
        if strcmp(name,'timestamp')
            token=regexp(value,'^([0-9]+)[ \t]+[(]([0-9.eE+\-]+) seconds ago[)]$','tokens','once');
            if isempty(token),why='STATUS_LISTENER_AGE_MISSING';return,end
            x=str2double(token{1});
        else
            x=str2double(value);
        end
        if ~isfinite(x),why='STATUS_NUMBER_NONFINITE';return,end
        a.(name)=x;
    end
end
age=regexp(values{strcmp(observed,'timestamp')}, ...
    '^[0-9]+[ \t]+[(]([0-9.eE+\-]+) seconds ago[)]$','tokens','once');
if isempty(age),why='STATUS_LISTENER_AGE_MISSING';return,end
a.listener_age_s=str2double(age{1});
if ~isfinite(a.listener_age_s)||a.listener_age_s<0,why='STATUS_LISTENER_AGE_INVALID';return,end
for n={'timestamp','sample_timestamp','reference_timestamp','segment_timestamp', ...
        'input_sample_count','output_publish_count','rejected_sample_count','reset_count'}
    if ~integer(a.(n{1}),0,floor(flintmax/1000)),why='STATUS_INTEGER_PRECISION_OR_DOMAIN';return,end
end
if a.timestamp<=0||any([a.sample_timestamp,a.reference_timestamp,a.segment_timestamp]>a.timestamp)|| ...
        ~integer(a.control_mode,0,2)||~integer(a.failure_reason,0,9)
    why='STATUS_ENUM_OR_TIMESTAMP_DOMAIN';return
end
a.board_timestamp_ns=a.timestamp*1000;a.sample_timestamp_ns=a.sample_timestamp*1000;
a.reference_timestamp_ns=a.reference_timestamp*1000;a.segment_timestamp_ns=a.segment_timestamp*1000;
end
function [g,why]=activeGuard(s,now)
g=[];why='STATUS_GUARD_NOT_OBSERVED';
if s.fatal_latched,why='STATUS_OBSERVER_FATAL';return,end
if isempty(s.latest),return,end
a=s.latest;rxAge=now-a.rx_ns;
ageUpper=a.listener_age_s+(now-a.query_started_ns)/1e9;
if rxAge<0||rxAge>s.maximum_guard_age_ns||ageUpper<0||ageUpper>s.maximum_guard_age_ns/1e9
    why='STATUS_GUARD_STALE';return
end
fields={'active','state_valid','reference_fresh','segment_fresh', ...
    'native_position_controller_disabled','native_attitude_rate_allocator_enabled','single_publisher_contract_pass'};
if a.control_mode~=1||a.failure_reason~=0||~all(cellfun(@(n)a.(n),fields))|| ...
        any(abs(a.robust_acceleration_ned_mps2)>1e-7)||a.dt_s<=0
    why='STATUS_GUARD_ATOMS_NOT_ADMITTED';return
end
g=a;g.valid=true;g.receive_age_s=rxAge/1e9;g.listener_age_upper_bound_s=ageUpper;
g.age_basis='PRINTED_BOARD_AGE_PLUS_ELAPSED_FROM_REQUEST_PREPARATION';
g.nominal_mode_observed=true;g.robust_acceleration_zero=true;
why='FRESH_MODE1_SINGLE_PUBLISHER_GUARD';
end
function x=status(s,now)
x=struct('query',s.query,'query_open',s.query_open,'tx_confirmed',s.tx_confirmed, ...
    'request_generation',s.request_generation,'status_generation',s.status_generation, ...
    'queries_prepared',s.queries_prepared,'send_attempts',s.send_attempts,'queries_sent',s.queries_sent, ...
    'responses_completed',s.responses_completed,'chunks_received',s.chunks_received,'bytes_received',s.bytes_received, ...
    'buffer_bytes',numel(s.buffer),'maximum_response_bytes',s.maximum_response_bytes, ...
    'duplicate_chunks',s.duplicate_chunks,'duplicate_status_responses',s.duplicate_status_responses, ...
    'inactive_reset_increment_total',s.inactive_reset_increment_total, ...
    'inactive_reset_increment_observations',s.inactive_reset_increment_observations, ...
    'guards_issued',s.guards_issued,'fatal_latched',s.fatal_latched,'first_failure',s.first_failure, ...
    'failure_count',s.failure_count,'query_interval_ns',s.query_interval_ns,'query_timeout_ns',s.query_timeout_ns, ...
    'maximum_guard_age_ns',s.maximum_guard_age_ns,'binding',s.binding, ...
    'latest',s.latest,'latest_response',s.latest_response,'now_ns',now,'hardware_actions',0,'COM_owner_count',0);
end
function [s,r]=fail(s,r,why)
s.failure_count=s.failure_count+1;
if ~s.fatal_latched
    s.fatal_latched=true;s.first_failure=struct('reason',why,'event_ns',s.last_event_ns, ...
        'request_generation',s.request_generation,'raw_response_prefix',s.buffer);
end
r.reason=why;r.fatal_latched=true;r.snapshot=status(s,s.last_event_ns);
end
function r=emptyReceipt(op)
r=struct('operation',op,'reason','','query_prepared',false,'request_generation',0, ...
    'tx_message',[],'completed',false,'atom',[],'guard',[],'guard_reason','', ...
    'fatal_latched',false,'snapshot',[]);
end
function b=verifiedBinding(e)
assert(isfield(e,'verified_binding')&&isstruct(e.verified_binding)&&isscalar(e.verified_binding)&& ...
    isfield(e,'now_ns'),'gpenmpcNative:Se3ObserverBinding','Explicit verified identity required.');
b=e.verified_binding;
assert(all(isfield(b,{'uid','system_id','component_id','boot_generation','verified'}))&& ...
    isequal(b.verified,true)&&integer(b.system_id,1,255)&&integer(b.component_id,1,255)&& ...
    integer(b.boot_generation,0,flintmax),'gpenmpcNative:Se3ObserverBinding','Verified source/boot required.');
if isa(b.uid,'uint64')&&isscalar(b.uid)&&b.uid>0,uid=char(string(b.uid));
elseif (ischar(b.uid)&&isrow(b.uid))||(isstring(b.uid)&&isscalar(b.uid)),uid=char(b.uid);
else,error('gpenmpcNative:Se3ObserverBinding','UID must be lossless uint64 or decimal text.');end
assert(~isempty(regexp(uid,'^[1-9][0-9]{0,19}$','once')), ...
    'gpenmpcNative:Se3ObserverBinding','Invalid UID decimal identity.');
if numel(uid)==20
    maximum='18446744073709551615';k=find(uid~=maximum,1);
    assert(isempty(k)||uid(k)<maximum(k),'gpenmpcNative:Se3ObserverBinding','UID exceeds uint64.');
end
b=struct('uid',uid,'system_id',double(b.system_id),'component_id',double(b.component_id), ...
    'boot_generation',double(b.boot_generation),'verified',true);
end
function text=clean(raw)
text=char(raw);text=regexprep(text,[char(27),'\[[0-9;?]*[ -/]*[@-~]'],'');text=strrep(text,char(13),'');
end
function yes=integer(x,a,b)
yes=isnumeric(x)&&isreal(x)&&isscalar(x)&&isfinite(x)&&double(x)>=a&&double(x)<=b&&double(x)==fix(double(x));
end
function yes=ns(x),yes=integer(x,0,flintmax);end
function yes=sendReceiptValid(s,e)
yes=s.query_open&&~s.tx_confirmed&&isfield(e,'request_generation')&& ...
    integer(e.request_generation,1,flintmax)&&e.request_generation==s.request_generation&& ...
    isfield(e,'send_succeeded')&&islogical(e.send_succeeded)&&isscalar(e.send_succeeded);
end
function [n,b,v]=fieldNames()
n={'timestamp','sample_timestamp','reference_timestamp','segment_timestamp', ...
    'input_sample_count','output_publish_count','rejected_sample_count','reset_count', ...
    'dt_s','reference_age_s','segment_age_s','hover_thrust','total_mass_kg','yaw_setpoint_rad','control_mode','failure_reason'};
b={'active','reference_fresh','segment_fresh','state_valid','hover_thrust_fresh', ...
    'native_position_controller_disabled','native_attitude_rate_allocator_enabled','single_publisher_contract_pass'};
v={'robust_acceleration_ned_mps2','commanded_acceleration_ned_mps2','normalized_thrust_ned', ...
    'position_ned_m','velocity_ned_mps','reference_position_ned_m','reference_velocity_ned_mps', ...
    'reference_acceleration_ned_mps2','nominal_feedback_acceleration_ned_mps2','drag_feedforward_acceleration_ned_mps2'};
end
