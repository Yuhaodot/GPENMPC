function summary=capture_gpenmpc_failure_diagnostic(session,evidence)
% Project retained command and shell records for failure diagnostics.
maxCharacters=16384;maxRecords=256;maxCommandCharacters=512;
summary=struct('schema','GPENMPC_FAILURE_SESSION_DIAGNOSTIC_V1', ...
    'commands',struct(),'serial_control_text','','diagnostic_lines',{{}}, ...
    'serial_record_count',0,'first_original_host_receive_ns',uint64(0), ...
    'last_original_host_receive_ns',uint64(0),'truncated',false, ...
    'maximum_text_characters',maxCharacters,'maximum_serial_records',maxRecords, ...
    'source','EXISTING_RECEIVER_RECORDS_BEFORE_RECOVERY','hardware_actions',0);
for name={'prepare_send','confirm_send','start_send'}
    key=name{1};if ~isfield(session,key),continue,end
    source=session.(key);command=struct();
    if isfield(source,'encoding')&&isfield(source.encoding,'original_command')
        text=char(source.encoding.original_command);
        command.original_command=text(1:min(numel(text),maxCommandCharacters));
        command.command_truncated=numel(text)>maxCommandCharacters;
        if isfield(source.encoding,'action'),command.action=source.encoding.action;end
    end
    for field={'original_host_submit_ns','original_host_send_return_ns', ...
            'messages_attempted','messages_send_returned','partial'}
        if isfield(source,field{1}),command.(field{1})=source.(field{1});end
    end
    summary.commands.(key)=command;
end
if ~isfield(evidence,'raw_mavlink'),return,end
records=evidence.raw_mavlink;chunks={};characters=0;
for index=numel(records):-1:1
    row=records{index};
    if ~isfield(row,'topic')||~strcmp(row.topic,'SERIAL_CONTROL'),continue,end
    message=[];
    if isfield(row,'message')&&isstruct(row.message),message=row.message;
    elseif isfield(row,'decoded_message')&&isstruct(row.decoded_message),message=row.decoded_message;end
    if isempty(message)||~isfield(message,'Payload'),continue,end
    payload=message.Payload;
    if ~all(isfield(payload,{'device','flags','count','data'}))||payload.device~=10||payload.flags~=1,continue,end
    count=double(payload.count);
    if ~isscalar(count)||~isfinite(count)||count<0||count>70||count~=fix(count)||numel(payload.data)<count,continue,end
    if summary.serial_record_count>=maxRecords||characters>=maxCharacters
        summary.truncated=true;break
    end
    text=char(reshape(uint8(payload.data(1:count)),1,[]));
    remaining=maxCharacters-characters;
    if numel(text)>remaining,text=text(end-remaining+1:end);summary.truncated=true;end
    chunks{end+1}=text;characters=characters+numel(text); %#ok<AGROW>
    summary.serial_record_count=summary.serial_record_count+1;
    if isfield(row,'original_host_receive_ns')
        if summary.serial_record_count==1,summary.last_original_host_receive_ns=row.original_host_receive_ns;end
        summary.first_original_host_receive_ns=row.original_host_receive_ns;
    end
end
if ~isempty(chunks),summary.serial_control_text=[chunks{end:-1:1}];end
% Keep the complete receive text; these extracts are diagnostic only.
summary.diagnostic_lines=regexp(summary.serial_control_text, ...
    '(?:RFLY_LOCAL_PREPARE[^\s]*|RFLY_LOCAL_RAW_GUARD|RFLY_LOCAL_SESSION|RFLY_LOCAL_START|LOCAL_IO|LOCAL_CTX|LOCAL_AUTH)[^\r\n]*','match');
end
