function result=test_px4_health_dialect(outputPath)
% Test the PX4 health dialect.
root=fileparts(fileparts(mfilename('fullpath')));
xml=fullfile(root,'m600_coptersim','matlab_validation','+m600check','px4_health_events.xml');
parent=mavlinkdialect('common.xml');health=mavlinkdialect(xml);
sourceRoot=gpenmpc_external_path('px4_source_root');
checks=struct('name',{},'pass',{});
topics={'HEARTBEAT','EXTENDED_SYS_STATE','LOCAL_POSITION_NED','ATTITUDE','ATTITUDE_TARGET', ...
    'ESTIMATOR_STATUS','HIL_ACTUATOR_CONTROLS','STATUSTEXT','COMMAND_ACK','PARAM_VALUE', ...
    'TIMESYNC','AUTOPILOT_VERSION','SYSTEM_TIME','HIGHRES_IMU','GPS_RAW_INT','VIBRATION', ...
    'COMMAND_LONG','SET_POSITION_TARGET_LOCAL_NED','PARAM_REQUEST_READ','PARAM_SET','SYS_STATUS'};
for k=1:numel(topics)
    name=topics{k};
    record(['existing_definition_unchanged_' name],isequaln(msginfo(parent,name),msginfo(health,name))&& ...
        isequaln(createmsg(parent,name),createmsg(health,name)));
end
expected=[410 53 160;411 3 106;412 6 33;413 7 77];
names={'EVENT','CURRENT_EVENT_SEQUENCE','REQUEST_EVENT','RESPONSE_EVENT_ERROR'};
for k=1:4
    name=names{k};id=expected(k,1);len=expected(k,2);extra=expected(k,3);
    header=fileread(fullfile(sourceRoot,'build','px4_fmu-v6c_default','mavlink','common', ...
        ['mavlink_msg_' lower(name) '.h']));
    crcToken=regexp(header,['#define MAVLINK_MSG_ID_' name '_CRC\s+(\d+)'],'tokens','once');
    lenToken=regexp(header,['#define MAVLINK_MSG_ID_' name '_LEN\s+(\d+)'],'tokens','once');
    record(['exact_px4_generated_wire_identity_' name],str2double(crcToken{1})==extra&&str2double(lenToken{1})==len);
    payload=uint8(1:len);packet=frame(id,payload,extra);[msg,status]=deserializemsg(health,packet);
    record(['actual_decoder_accepts_' name],numel(msg)==1&&all(status==0)&&double(msg.MsgID)==id);
    if id==410
        record('event_arguments_retained_all_40_bytes',isequal(reshape(msg.Payload.arguments,1,[]),uint8(14:53)));
        record('event_id_time_sequence_and_source_preserved', ...
            msg.Payload.id==uint32(hex2dec('04030201'))&& ...
            msg.Payload.event_time_boot_ms==uint32(hex2dec('08070605'))&& ...
            msg.Payload.sequence==uint16(hex2dec('0A09')));
        record('event_json_serializes_without_loss',contains(jsonencode(msg),'arguments'));
        badPacket=packet;badPacket(end)=bitxor(badPacket(end),uint8(1));
        [bad,badStatus]=deserializemsg(health,badPacket);
        record('bad_crc_not_accepted_as_event',isempty(bad)||all(badStatus~=0));
    end
end
result=struct('schema','HOST_PX4_HEALTH_DIALECT_TEST','passed',all([checks.pass]), ...
    'passed_checks',nnz([checks.pass]),'total_checks',numel(checks),'checks',checks, ...
    'hardware_actions',0,'udp_open',0,'arm_requests',0, ...
    'claim','Offline message-compatibility tests.');
if nargin>0
    assert(~isfile(outputPath),'Output already exists');
    folder=fileparts(outputPath);if ~isfolder(folder),mkdir(folder);end
    f=fopen(outputPath,'w');assert(f>=0);c=onCleanup(@()fclose(f));
    fprintf(f,'%s',jsonencode(result,PrettyPrint=true));clear c;
end
fprintf('PX4 health dialect: %d/%d; hardware=0\n',result.passed_checks,result.total_checks);
assert(result.passed,'m600check:HealthDialectFailed','Health observation definitions failed.');
    function record(name,pass)
        checks(end+1)=struct('name',name,'pass',logical(pass));
    end
end
function bytes=frame(id,payload,extra)
% Standard MAVLink2 frame, using CRC extra taken from exact PX4 generated C.
header=uint8([numel(payload),0,0,7,1,1,bitand(id,255),bitand(bitshift(id,-8),255),bitshift(id,-16)]);
crc=uint16(65535);input=[header,payload,uint8(extra)];
for b=input
    tmp=bitxor(uint8(b),uint8(bitand(crc,255)));
    tmp=bitxor(tmp,uint8(bitand(bitshift(uint16(tmp),4),255)));
    crc=bitxor(bitxor(bitshift(crc,-8),bitshift(uint16(tmp),8)), ...
        bitxor(bitshift(uint16(tmp),3),bitshift(uint16(tmp),-4)));
end
bytes=[uint8(253),header,payload,uint8(bitand(crc,255)),uint8(bitshift(crc,-8))];
end
