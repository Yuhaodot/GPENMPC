function observed=RflyLocalStartReceiptDecoder(frames,request,dialect,serviceIo)
% Parse post-submit SERIAL_CONTROL records from the bound owner.
if nargin<4,serviceIo=@() [];end
observed=[];parts=cell(numel(frames),1);times=zeros(numel(frames),1,'uint64');
slice=tic;
for k=1:numel(frames)
    % Reuse validated owner messages without recursive IO drains per shell fragment.
    % Yield during fallback parsing through the service callback.
    if toc(slice)>=.005,serviceIo();slice=tic;end
    times(k)=frames(k).original_host_receive_ns;
    assert(isa(times(k),'uint64')&&times(k)>0);
    if k>1,assert(times(k)>=times(k-1),'gpenmpcNative:StartReceiveOrder','Receive order regressed.');end
    if isfield(frames,'transport_record')&&~isempty(frames(k).transport_record)
        record=frames(k).transport_record;
        assert(isfield(frames,'decoded_message') ...
            &&strcmp(frames(k).decoded_source,'RAW_OWNER_DATAGRAM_DEQUEUE_OFFICIAL_DECODER') ...
            &&isequal(record.validated,true)&&record.official_parse_status==0 ...
            &&record.original_host_receive_ns==times(k) ...
            &&isequal(record.raw_frame,frames(k).raw_frame) ...
            &&isequaln(record.decoded_message,frames(k).decoded_message), ...
            'gpenmpcNative:StartCachedFrame','Cached message must be the original validated sole-owner frame.');
        message=record.decoded_message;
    elseif isfield(frames,'raw_frame')&&~isempty(frames(k).raw_frame)
        raw=frames(k).raw_frame(:);assert(isa(raw,'uint8'));
        [message,status]=deserializemsg(dialect,raw.',OutputAllMessage=true);
        assert(status==0&&numel(message)==1,'gpenmpcNative:StartFrameCrc','Invalid original frame.');
    else
        assert(isfield(frames,'decoded_source') ...
            &&strcmp(frames(k).decoded_source,'ORIGINAL_MAVLINKIO_SERIAL_CONTROL_CALLBACK') ...
            &&isequal(frames(k).raw_frame_available,false),'gpenmpcNative:StartCallbackOrigin','Invalid callback origin.');
        message=frames(k).decoded_message;
    end
    assert(isstruct(message)&&isscalar(message)&&message.MsgID==126 ...
        &&message.SystemID==request.system&&message.ComponentID==request.component, ...
        'gpenmpcNative:StartFrameSource','Invalid message or source.');
    p=message.Payload;
    assert(p.device==10&&p.flags==1&&p.baudrate==0&&p.timeout==0 ...
        &&p.count>=0&&p.count<=70&&fix(p.count)==p.count ...
        &&isa(p.data,'uint8')&&numel(p.data)==70,'gpenmpcNative:StartShellPayload','Invalid shell payload.');
    if all(isfield(p,{'target_system','target_component'}))
        assert((p.target_system==0||p.target_system==request.host_system) ...
            &&(p.target_component==0||p.target_component==request.host_component), ...
            'gpenmpcNative:StartFrameTarget','Foreign shell target.');
    end
    parts{k}=reshape(p.data(1:double(p.count)),[],1);
end
bytes=vertcat(parts{:});shell=char(bytes.');
lines=regexp(shell,'(?m)^RFLY_LOCAL_START [^\r\n]*\r?\n','match');
assert(numel(lines)<=1,'gpenmpcNative:StartAmbiguous','More than one start receipt.');
if isempty(lines),return,end
line=strtrim(lines{1});fields=struct();tokens=strsplit(line,' ');
for k=2:numel(tokens)
    pair=regexp(tokens{k},'^([a-z_]+)=(-?[0-9]+)$','tokens','once');
    assert(~isempty(pair)&&~isfield(fields,pair{1}),'gpenmpcNative:StartFields','Malformed or duplicated field.');
    fields.(pair{1})=str2double(pair{2});
end
assert(all(isfield(fields,{'acquire_attempted','acquire_result','reason','start_return', ...
    'task_created','retained_first_acquire','diagnostic_only'})) ...
    &&fields.retained_first_acquire==1&&fields.diagnostic_only==1, ...
    'gpenmpcNative:StartFields','Incomplete first-acquire diagnostic.');
observed=fields;observed.original_line=line;observed.original_shell_bytes=bytes;
observed.original_frames=frames;observed.first_receive_ns=times(1);observed.last_receive_ns=times(end);
observed.running_inner_loop_proven=false; % Confirmed by subsequent commits.
end
