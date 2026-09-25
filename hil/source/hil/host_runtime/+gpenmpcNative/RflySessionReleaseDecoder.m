function r=RflySessionReleaseDecoder(records,request,dialect)
% Parse paired RFLY_SESSION and RFLY_RELEASE records.
a=request.registered_association;
a=gpenmpcNative.RflySessionAssociationDecoder(a.prepare_receipt.original_frames, ...
    a.confirm_receipt.original_frames,a.original_request,a.physical_declaration,dialect);
stop=exactU64(request.original_stop_submit_ns);submit=exactU64(request.original_release_submit_ns);
assert(a.original_host_receive_ns<stop&&stop<submit,'gpenmpcNative:ReleaseCommandOrder', ...
    'Original confirmed session, stop and release commands must be ordered.');
q=a.original_request;[shell,times,rawAvailable]=originalBytes(records,q,dialect);
assert(submit<=times(1),'gpenmpcNative:ReleaseReceiveOrder','Release receipt predates original release command.');
text=char(shell.');
local=a.local_full_inner;sessionLabel='RFLY_SESSION';releaseLabel='RFLY_RELEASE';retired=4;
sessionNames={'state','challenge','uid','system','component','registration_hrt_us','session_generation', ...
    'link_generation','semantics','config_sha','session_sha'};
if local
    sessionLabel='RFLY_LOCAL_SESSION';releaseLabel='RFLY_LOCAL_RELEASE';retired=5;
    sessionNames=[sessionNames,{'task_sha','leg'}];stopField='quiescent';
else
    sessionNames{end+1}='parameter_sha';stopField='context_observed_after_stop';
end
sessionNames=[sessionNames,{'registered','echo_confirmed','declared_isolation', ...
    'declaration_is_sensor_proof','session_fault','start_requests','stop_requests'}];
releaseNames={'result',stopField,'disarmed','virtual_zero_stream_accepted', ...
    'plant_cache_zero_proven','publication_attempts','publication_successes'};
if ~local,releaseNames{end+1}='raw_evidence_available';end
[session,sessionBytes,sessionAt]=line(text,sessionLabel,sessionNames);
[release,releaseBytes,releaseAt]=line(text,releaseLabel,releaseNames);
assert(sessionAt<releaseAt,'gpenmpcNative:ReleaseLineOrder','Actual entry prints session before release.');
c=a.confirm_receipt.parsed_fields;
immutable={'challenge','uid','system','component','registration_hrt_us','session_generation', ...
    'link_generation','semantics','config_sha','session_sha'};
if local,immutable=[immutable,{'task_sha','leg'}];else,immutable{end+1}='parameter_sha';end
for k=1:numel(immutable)
    name=immutable{k};assert(isequal(session.(name),c.(name)), ...
        'gpenmpcNative:ReleaseSessionMismatch','Release differs from registered %s.',name);
end
% ApplicationState::Retired=4, ApplicationResult::Detached=4 in the real TU.
assert(session.state==retired&&release.result==4&&session.registered==1 ...
    &&session.echo_confirmed==1&&session.declared_isolation==1 ...
    &&session.declaration_is_sensor_proof==0&&session.session_fault==0 ...
    &&session.start_requests>0&&session.stop_requests>0, ...
    'gpenmpcNative:ReleaseNotDetached','Only the exact healthy retired, previously started/stopped session is reusable.');
assert(release.(stopField)==1&&release.disarmed==1 ...
    &&ismember(release.virtual_zero_stream_accepted,uint64([0 1])) ...
    &&release.plant_cache_zero_proven==0 ...
    &&release.publication_successes<=release.publication_attempts, ...
    'gpenmpcNative:ReleaseEvidence','Actual stopped/disarmed release evidence is inconsistent.');
if ~local,assert(ismember(release.raw_evidence_available,uint64([0 1])),'gpenmpcNative:ReleaseEvidence');end
assert(release.publication_attempts==0||release.virtual_zero_stream_accepted==1, ...
    'gpenmpcNative:ReleaseVirtualZeroMissing','Published controls require actual stream zero acceptance.');
r=struct('schema','GPENMPC_RFLY_SESSION_RELEASE_RECEIPT_V1','result','Detached', ...
    'registered_association',a,'local_full_inner',local,'original_request',request,'original_records',records, ...
    'original_shell_bytes',shell,'original_session_line_bytes',sessionBytes, ...
    'original_release_line_bytes',releaseBytes,'parsed_session',session,'parsed_release',release, ...
    'first_original_host_receive_ns',times(1),'last_original_host_receive_ns',times(end), ...
    'original_frame_receive_ns',times,'raw_frame_available',rawAvailable, ...
    'mavlink_crc_checked_here',all(rawAvailable),'transport_source_authenticated',false, ...
    'board_context_detached_reported',true,'board_disarmed_reported',true, ...
    'plant_cache_zero_proven',false,'requires_independent_plant_cache_zero',release.publication_attempts>0, ...
    'publication_authority',false,'hardware_actions',0);
end

function [fields,bytes,at]=line(text,label,names)
starts=regexp(text,['(?m)^' label ' ']);
[at,~,~,lines]=regexp(text,['(?m)^' label ' [^\r\n]*(?:\r?\n)']);
assert(numel(starts)==1&&numel(lines)==1,'gpenmpcNative:ReleaseLine', ...
    'Exactly one complete unambiguous %s line required.',label);
bytes=uint8(lines{1}).';tokens=strsplit(regexprep(lines{1},'\r?\n$',''),' ');
assert(numel(tokens)==numel(names)+1,'gpenmpcNative:ReleaseFields','Missing or extra release fields.');
fields=struct();
for k=1:numel(names)
    token=regexp(tokens{k+1},'^([a-z_]+)=([0-9A-Fa-f]+)$','tokens','once');
    assert(~isempty(token)&&strcmp(token{1},names{k}),'gpenmpcNative:ReleaseFields','Malformed/duplicated field.');
    name=names{k};value=token{2};
    if strcmp(name,'challenge')
        assert(numel(value)==32,'gpenmpcNative:ReleaseFields','Exact challenge required.');
        fields.(name)=[integer(value(1:16),16);integer(value(17:32),16)];
    elseif endsWith(name,'_sha')
        assert(numel(value)==64&&any(value~='0'),'gpenmpcNative:ReleaseFields','Exact nonzero SHA required.');
        fields.(name)=upper(value);
    else,fields.(name)=integer(value,10);
    end
end
end

function [shell,times,rawAvailable]=originalBytes(records,q,dialect)
assert(isstruct(records)&&~isempty(records),'gpenmpcNative:ReleaseRecords','Original SERIAL_CONTROL records required.');
parts=cell(numel(records),1);times=zeros(numel(records),1,'uint64');rawAvailable=false(numel(records),1);
for k=1:numel(records)
    record=records(k);times(k)=exactU64(record.original_host_receive_ns);
    if k>1,assert(times(k)>=times(k-1),'gpenmpcNative:ReleaseReceiveOrder','Original receive time reversed.');end
    targetsKnown=false;targets=zeros(1,2,'uint8');
    if isfield(record,'raw_frame')&&~isempty(record.raw_frame)
        assert(~isfield(record,'decoded_message')||isempty(record.decoded_message), ...
            'gpenmpcNative:ReleaseFrame','Ambiguous original raw/decoded source.');
        raw=record.raw_frame;assert(isa(raw,'uint8')&&isvector(raw)&&numel(raw)>=12,'gpenmpcNative:ReleaseFrame','Original frame shape invalid.');
        raw=raw(:);n=double(raw(2));signed=bitand(raw(3),uint8(1))~=0;
        assert(raw(1)==253&&bitand(raw(3),uint8(254))==0&&numel(raw)==12+n+13*double(signed) ...
            &&raw(6)==q.system&&raw(7)==q.component&&isequal(raw(8:10),uint8([126;0;0]))&&n<=81, ...
            'gpenmpcNative:ReleaseFrame','Wrong original frame tuple/shape.');
        [message,status]=deserializemsg(dialect,raw.',OutputAllMessage=true);
        assert(numel(message)==1&&status==0&&message.MsgID==126,'gpenmpcNative:ReleaseFrameCrc','Official original-frame decoder rejected CRC.');
        payload=zeros(81,1,'uint8');payload(1:n)=raw(11:10+n);targets=payload(80:81).';
        targetsKnown=true;rawAvailable(k)=true;
    else
        assert(all(isfield(record,{'decoded_message','decoded_source','raw_frame_available'})) ...
            &&string(record.decoded_source)=="ORIGINAL_MAVLINKIO_SERIAL_CONTROL_CALLBACK" ...
            &&isequal(record.raw_frame_available,false),'gpenmpcNative:ReleaseCallback','Original callback provenance required.');
        message=record.decoded_message;
        assert(isstruct(message)&&isscalar(message)&&all(isfield(message,{'MsgID','SystemID','ComponentID','Seq','Payload'})) ...
            &&message.MsgID==126&&message.SystemID==q.system&&message.ComponentID==q.component ...
            &&isscalar(message.Seq)&&isnumeric(message.Seq)&&isfinite(message.Seq) ...
            &&message.Seq>=0&&message.Seq<=255&&fix(message.Seq)==message.Seq, ...
            'gpenmpcNative:ReleaseCallback','Original callback header tuple required.');
        if all(isfield(message.Payload,{'target_system','target_component'}))
            targets=[message.Payload.target_system message.Payload.target_component];targetsKnown=true;
        end
    end
    p=message.Payload;
    assert(p.device==10&&p.flags==1&&p.baudrate==0&&p.timeout==0&&p.count>=0&&p.count<=70 ...
        &&fix(p.count)==p.count&&isa(p.data,'uint8')&&numel(p.data)==70,'gpenmpcNative:ReleaseShell','Actual shell reply shape required.');
    assert(~targetsKnown||((targets(1)==0||targets(1)==q.host_system) ...
        &&(targets(2)==0||targets(2)==q.host_component)),'gpenmpcNative:ReleaseTarget','Reply targets a different HOST.');
    parts{k}=reshape(p.data(1:double(p.count)),[],1);
end
shell=vertcat(parts{:});
end
function v=exactU64(v)
assert(isa(v,'uint64')&&isscalar(v)&&v>0,'gpenmpcNative:ReleaseExactUint64','Original positive uint64 required.');
end
function v=integer(s,base)
v=uint64(0);b=uint64(base);
for c=s
    if c>='0'&&c<='9',d=uint64(c-'0');elseif c>='a'&&c<='f',d=uint64(c-'a'+10);
    elseif c>='A'&&c<='F',d=uint64(c-'A'+10);else,error('gpenmpcNative:ReleaseInteger');end
    assert(d<b&&v<=idivide(intmax('uint64')-d,b,'floor'),'gpenmpcNative:ReleaseInteger','Integer overflow/malformed.');
    v=v*b+d;
end
end
