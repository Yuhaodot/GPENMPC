function result=test_rfly_local_start_receipt(resultPath)
% Test receipt parsing with retained negative messages and synthetic diagnostic lines.
build=fileparts(fileparts(mfilename('fullpath')));addpath(fullfile(build,'host_runtime'));
if nargin<1,resultPath=fullfile(build,'tools','LOCAL_START_RECEIPT_HOST_RESULT.mat');end
d=mavlinkdialect(fullfile(build,'m600_coptersim','matlab_validation','+m600check','px4_health_events.xml'),2);
request=struct('system',uint8(1),'component',uint8(1),'host_system',uint8(255),'host_component',uint8(190));
failed=['RFLY_LOCAL_START acquire_attempted=1 acquire_result=1 reason=5 start_return=-13 ' ...
    'task_created=0 retained_first_acquire=1 diagnostic_only=1' newline];
pass=replace(failed,{'acquire_result=1','reason=5','start_return=-13','task_created=0'}, ...
    {'acquire_result=0','reason=0','start_return=0','task_created=1'});
a=gpenmpcNative.RflyLocalStartReceiptDecoder(makeFrames(failed),request,d);
assert(a.start_return==-13&&a.acquire_result==1&&a.task_created==0&&~a.running_inner_loop_proven);
b=gpenmpcNative.RflyLocalStartReceiptDecoder(makeFrames(pass),request,d);
assert(b.task_created==1&&b.start_return==0&&b.acquire_result==0&&~b.running_inner_loop_proven);
assert(isempty(gpenmpcNative.RflyLocalStartReceiptDecoder(makeFrames(pass(1:end-1)),request,d)));
actualOld=['ERROR [gpenmpc_local_inner] session/evidence unavailable or rejected (1)' newline ...
    'ERROR [gpenmpc_local_inner] Task start failed (-13)' newline];
assert(isempty(gpenmpcNative.RflyLocalStartReceiptDecoder(makeFrames(actualOld),request,d)));
bad=makeFrames(pass);bad(1).decoded_message.SystemID=uint8(2);
mustReject(@()gpenmpcNative.RflyLocalStartReceiptDecoder(bad,request,d),'gpenmpcNative:StartFrameSource');
mustReject(@()gpenmpcNative.RflyLocalStartReceiptDecoder(makeFrames([pass pass]),request,d),'gpenmpcNative:StartAmbiguous');
% Require cache and raw-fallback parsing to agree.
codec=mavlinkio(d,SystemID=1,ComponentID=1);cleanup=onCleanup(@()delete(codec)); %#ok<NASGU>
cached=makeFrames(pass);rawOnly=rmfield(cached,'decoded_message');
for k=1:numel(cached)
    m=createmsg(d,'SERIAL_CONTROL');m.Payload=cached(k).decoded_message.Payload;
    bytes=serializemsg(codec,m);[decoded,status]=deserializemsg(d,bytes,OutputAllMessage=true);assert(status==0);
    cached(k).raw_frame=bytes(:);cached(k).raw_frame_available=true;
    cached(k).decoded_source='RAW_OWNER_DATAGRAM_DEQUEUE_OFFICIAL_DECODER';cached(k).decoded_message=decoded;
    cached(k).transport_record=struct('validated',true,'official_parse_status',status, ...
        'original_host_receive_ns',cached(k).original_host_receive_ns,'raw_frame',bytes(:),'decoded_message',decoded);
    rawOnly(k).raw_frame=bytes(:);rawOnly(k).raw_frame_available=true;rawOnly(k).decoded_source=cached(k).decoded_source;
end
c=gpenmpcNative.RflyLocalStartReceiptDecoder(cached,request,d);
raw=gpenmpcNative.RflyLocalStartReceiptDecoder(rawOnly,request,d);
assert(isequal(c.original_shell_bytes,raw.original_shell_bytes)&&c.task_created==1);
bad=cached;bad(1).transport_record.validated=false;
mustReject(@()gpenmpcNative.RflyLocalStartReceiptDecoder(bad,request,d),'gpenmpcNative:StartCachedFrame');
bad=cached;bad(1).raw_frame(end)=bitxor(bad(1).raw_frame(end),uint8(1));
mustReject(@()gpenmpcNative.RflyLocalStartReceiptDecoder(bad,request,d),'gpenmpcNative:StartCachedFrame');
bad=cached;bad(1).decoded_message.SystemID=uint8(2);
mustReject(@()gpenmpcNative.RflyLocalStartReceiptDecoder(bad,request,d),'gpenmpcNative:StartCachedFrame');
result=struct('passed',true,'cases',10,'COM',0,'board_actions',0, ...
    'scope','START_RECEIPT_PARSE','old_start_error_not_success',true);
assert(~isfile(resultPath),'Choose an unused output path.');save(resultPath,'result');disp(result);
end
function frames=makeFrames(text)
bytes=uint8(char(text));parts=ceil(numel(bytes)/70);rows=cell(parts,1);
for k=1:parts
    data=zeros(1,70,'uint8');chunk=bytes((k-1)*70+1:min(k*70,numel(bytes)));data(1:numel(chunk))=chunk;
    p=struct('device',uint8(10),'flags',uint8(1),'baudrate',uint32(0),'timeout',uint16(0), ...
        'count',uint8(numel(chunk)),'data',data);
    message=struct('MsgID',126,'SystemID',uint8(1),'ComponentID',uint8(1),'Seq',uint8(k),'Payload',p);
    rows{k}=struct('original_host_receive_ns',uint64(k),'raw_frame',uint8([]), ...
        'raw_frame_available',false,'decoded_source','ORIGINAL_MAVLINKIO_SERIAL_CONTROL_CALLBACK', ...
        'decoded_message',message); %#ok<AGROW>
end
frames=vertcat(rows{:});
end
function mustReject(f,id)
try,f();catch e,assert(strcmp(e.identifier,id));return,end
error('gpenmpcTest:ExpectedReject','Expected %s',id);
end
