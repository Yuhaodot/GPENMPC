function [messages,receipt]=RflySessionCommandEncoder(action,value,dialect,target)
% Encode the supported NSH session and stream commands.
assert(isstruct(value)&&isscalar(value));action=string(action);assert(isscalar(action));
for f={'system','component'},v=target.(f{1});assert(isa(v,'uint8')&&isscalar(v)&&v>0);end
switch action
    case {"prepare","prepare_local","prepare_local_rc"}
        names={'challenge','origin_ned_m','host_system','host_component'};
        if action~="prepare",names{end+1}='leg_index';end
        exactFields(value,names);
        c=value.challenge;assert(isa(c,'uint64')&&numel(c)==2&&any(c),'gpenmpcNative:SessionCommandChallenge');
        p=value.origin_ned_m;assert(isa(p,'double')&&numel(p)==3&&all(isfinite(p)),'gpenmpcNative:SessionCommandOrigin');
        for f={'host_system','host_component'},v=value.(f{1});assert(isa(v,'uint8')&&isscalar(v)&&v>0);end
        command=sprintf('gpenmpc_rfly_session prepare /dev/ttyACM0 %s %.17g %.17g %.17g %u %u', ...
            upper(reshape(dec2hex(c(:),16).',1,[])),p(1),p(2),p(3),value.host_system,value.host_component);
        if action~="prepare"
            assert(isa(value.leg_index,'uint8')&&isscalar(value.leg_index)&&ismember(value.leg_index,uint8(1:5)), ...
                'gpenmpcNative:LocalSessionLeg','Original explicit task leg 1..5 required.');
            command=sprintf('%s %u',command,value.leg_index);
        end
        if action=="prepare_local_rc"
            assert(value.leg_index==1,'gpenmpcNative:OperatorLeg','USB operator component uses one explicit leg.');
            command=strrep(command,'gpenmpc_rfly_session prepare ','gpenmpc_rfly_session prepare_rc ');
        end
    case "confirm"
        exactFields(value,{'prepared_receipt','physical_setup_record_sha256'});
        p=value.prepared_receipt;
        assert(isstruct(p)&&isscalar(p)&&string(p.schema)=="GPENMPC_RFLY_PREPARED_SESSION_RECEIPT_V1" ...
            &&string(p.echo_confirmation_result)=="NOT_CONFIRMED" ...
            &&string(p.registration_result)=="Registered"&&isfield(p,'execution_session_sha256'), ...
            'gpenmpcNative:SessionCommandPreparation','Original parsed unconfirmed prepare receipt required.');
        command=sprintf('gpenmpc_rfly_session confirm %s %s',sha(p.execution_session_sha256),sha(value.physical_setup_record_sha256));
    case {"start","stop","status","release"}
        exactFields(value,{});command=['gpenmpc_rfly_session ' char(action)];
    case "module_status"
        exactFields(value,{});command='gpenmpc_rfly_canonical_local status';
    case {"prearm_ekf_stop","prearm_ekf_start","prearm_ekf_status"}
        % Fixed native commands only. The sole IO enforces disarmed,
        % unregistered startup and a single stop/start pair.
        exactFields(value,{});command=['ekf2 ' char(extractAfter(action,'prearm_ekf_'))];
    case {"evidence_failed","evidence_interrupted"}
        exactFields(value,{});command=['gpenmpc_rfly_session evidence ' char(extractAfter(action,'evidence_'))];
    case "evidence_local"
        exactFields(value,{});command='gpenmpc_rfly_session evidence';
    case {"stream_snapshot","stream_feedback","stream_local"}
        exactFields(value,{'enabled'});assert(islogical(value.enabled)&&isscalar(value.enabled));
        if action=="stream_snapshot",name='GPENMPC_PRIVATE_SNAPSHOT';
        elseif action=="stream_feedback",name='GPENMPC_COMMITTED_FEEDBACK';
        else,name='GPENMPC_LOCAL_WIRE';end
        % interval=-1 checks the outbox on each MAVLink loop; zero disables it.
        % Fragment production remains tied to source and commit events.
        rate=0;if value.enabled,rate=-1;end
        command=sprintf('mavlink stream -d /dev/ttyACM0 -s %s -r %d',name,rate);
    otherwise,error('gpenmpcNative:SessionCommandNotAllowed','Only the explicit session/stream operations are allowed.');
end
assert(~contains(command,newline)&&~contains(command,char(13))&&~contains(command,';') ...
    &&~contains(command,'&')&&~contains(command,'|'),'gpenmpcNative:SessionCommandInjection');
bytes=uint8([command newline]).';assert(numel(bytes)<=256,'gpenmpcNative:SessionCommandBound');
messages=cell(ceil(numel(bytes)/70),1);targetExplicit=true;
for k=1:numel(messages)
    m=createmsg(dialect,'SERIAL_CONTROL');m.Payload.device=uint8(10);m.Payload.flags=uint8(6);
    m.Payload.timeout=uint16(0);m.Payload.baudrate=uint32(0);part=bytes((k-1)*70+1:min(k*70,numel(bytes)));
    m.Payload.count=uint8(numel(part));m.Payload.data(:)=uint8(0);m.Payload.data(1:numel(part))=part;
    if all(isfield(m.Payload,{'target_system','target_component'}))
        m.Payload.target_system=target.system;m.Payload.target_component=target.component;
    else
        targetExplicit=false; % The base dialect has no extension fields.
    end
    messages{k}=m;
end
receipt=struct('schema','RFLY_EXACT_SESSION_COMMAND_BYTES_V1','action',char(action), ...
    'original_command',command,'original_command_bytes',bytes,'fragments',numel(messages), ...
    'target',target,'explicit_mavlink_target_supported',targetExplicit, ...
    'existing_single_link_required',true,'io_sent',false,'board_acknowledged',false,'hardware_actions',0);
if action=="confirm"
    receipt.original_confirm_session_sha256=sha(value.prepared_receipt.execution_session_sha256);
    receipt.confirmed_physical_setup_record_sha256=sha(value.physical_setup_record_sha256);
end
end
function exactFields(s,names)
assert(isequal(sort(fieldnames(s)),sort(names(:))),'gpenmpcNative:SessionCommandFields','Unexpected/missing fields.');
end
function s=sha(v)
s=char(v);assert(~isempty(regexp(s,'^[0-9a-fA-F]{64}$','once'))&&any(s~='0'), ...
    'gpenmpcNative:SessionCommandDigest');s=upper(s);
end
