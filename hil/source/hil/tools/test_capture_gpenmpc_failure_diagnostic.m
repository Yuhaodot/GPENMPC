function result=test_capture_gpenmpc_failure_diagnostic()
% Test diagnostic parsing and projection with synthetic fixtures.
command='gpenmpc_rfly_session prepare_rc /dev/ttyACM0 1234 0 0 0 255 190 1';
session=struct('prepare_send',struct('encoding',struct('original_command',command, ...
    'action','prepare_local_rc'),'original_host_submit_ns',uint64([11;12]), ...
    'original_host_send_return_ns',uint64([13;14]),'messages_attempted',2, ...
    'messages_send_returned',2,'partial',false));
text=['RFLY_LOCAL_PREPARE_DIAG stage=4 reason=11 hil_us=10' newline ...
    'RFLY_LOCAL_PREPARE_ACQUISITION attempts=1 stop_reason=3' newline ...
    'RFLY_LOCAL_SESSION state=0 registered=0 session_fault=0' newline ...
    'RFLY_LOCAL_RAW_GUARD retained_snapshot=1' newline];
rows=makeRows(text);summary=capture_gpenmpc_failure_diagnostic(session,struct('raw_mavlink',{rows}));
assert(strcmp(summary.serial_control_text,text));
assert(strcmp(summary.commands.prepare_send.original_command,command));
assert(isequal(summary.commands.prepare_send.original_host_submit_ns,uint64([11;12])));
assert(numel(summary.diagnostic_lines)==4&&~summary.truncated&&summary.hardware_actions==0);
% Actual demo records may keep decoded_message instead of message.
cached=rows;for k=1:numel(cached),cached{k}.decoded_message=cached{k}.message;cached{k}=rmfield(cached{k},'message');end
other=capture_gpenmpc_failure_diagnostic(struct(),struct('raw_mavlink',{cached}));
assert(strcmp(other.serial_control_text,text));
largeText=repmat('x',1,21000);limited=capture_gpenmpc_failure_diagnostic(struct(),struct('raw_mavlink',{makeRows(largeText)}));
assert(limited.truncated&&numel(limited.serial_control_text)<=16384&&limited.serial_record_count<=256);
assert(strcmp(limited.serial_control_text,largeText(end-numel(limited.serial_control_text)+1:end)));
empty=capture_gpenmpc_failure_diagnostic(struct(),struct());assert(isempty(empty.serial_control_text));
bad=rows;bad{1}.message.Payload.count=uint8(71);
filtered=capture_gpenmpc_failure_diagnostic(struct(),struct('raw_mavlink',{bad}));
assert(filtered.serial_record_count==numel(rows)-1);
result=struct('passed',true,'cases',6,'hardware_actions',0,'control_changes',0);disp(result);
end
function rows=makeRows(text)
rows=cell(1,ceil(numel(text)/70));
for k=1:numel(rows)
    part=uint8(text((k-1)*70+1:min(k*70,numel(text))));
    payload=struct('device',uint8(10),'flags',uint8(1),'count',uint8(numel(part)),'data',part);
    rows{k}=struct('topic','SERIAL_CONTROL','message',struct('Payload',payload), ...
        'original_host_receive_ns',uint64(100+k));
end
end
