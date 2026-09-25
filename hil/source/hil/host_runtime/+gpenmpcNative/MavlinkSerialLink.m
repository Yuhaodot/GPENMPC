classdef MavlinkSerialLink < handle
    % MAVLINKSERIALLINK Own MAVLink 2/common over USB serial.
    % Use UAV Toolbox codecs with MATLAB serialport.
    % Open the port explicitly after construction.

    properties (SetAccess=private)
        Dialect
        Serializer
        Port = []
        Endpoint (1,1) string = ""
        Baud (1,1) double = 921600
        TargetSystem (1,1) double = 0
        TargetComponent (1,1) double = 0
        RxBuffer (1,:) uint8 = uint8([])
        Inbox (1,:) cell = cell(1,0)
        TxMessageCount (1,1) uint64 = uint64(0)
        TxByteCount (1,1) uint64 = uint64(0)
        RxFrameCount (1,1) uint64 = uint64(0)
        RxByteCount (1,1) uint64 = uint64(0)
        RxDiscardedByteCount (1,1) uint64 = uint64(0)
        RxInvalidFrameCount (1,1) uint64 = uint64(0)
        MaximumWriteWallS (1,1) double = 0
        CommandGeneration (1,1) uint64 = uint64(0)
    end

    properties (Constant)
        SourceSystem = 245
        SourceComponent = 190
        ArmedFlag = 128
        LandedOnGround = 1
        LocalNedFrame = 1
        AutopilotEstimator = 8
        RoutePayloadType = 42001
        RouteMagic = uint32(hex2dec('31415452'))
        RouteVersion = uint8(2)
        RouteBodyBytes = 124
        RouteChunkBytes = 112
    end

    methods
        function obj=MavlinkSerialLink()
            obj.Dialect=mavlinkdialect("common.xml",2);
            obj.Serializer=mavlinkio(obj.Dialect, ...
                SystemID=obj.SourceSystem,ComponentID=obj.SourceComponent, ...
                ComponentType="MAV_TYPE_GCS", ...
                AutopilotType="MAV_AUTOPILOT_INVALID");
        end

        function heartbeat=open(obj,endpoint,baud,heartbeatTimeoutS)
            arguments
                obj
                endpoint (1,1) string
                baud (1,1) double {mustBeInteger,mustBePositive}=921600
                heartbeatTimeoutS (1,1) double {mustBePositive}=15
            end
            assert(~obj.isOpen(),'gpenmpcNative:MavlinkSerialOwner', ...
                'The MATLAB MAVLink serial owner is already open.');
            % After reboot, PnP can precede MATLAB port discovery.
            % Wait for enumeration within the startup deadline; do not retry open or write.
            endpointWait=tic;
            available=string(serialportlist("available"));
            while ~any(strcmpi(available,endpoint))&&toc(endpointWait)<heartbeatTimeoutS
                pause(min(.1,max(0,heartbeatTimeoutS-toc(endpointWait))));
                available=string(serialportlist("available"));
            end
            assert(any(strcmpi(available,endpoint)), ...
                'gpenmpcNative:MavlinkSerialPnP', ...
                'The exact serial endpoint is not available.');
            obj.Endpoint=endpoint;obj.Baud=baud;
            obj.Port=serialport(char(endpoint),baud,Timeout=0.05);
            flush(obj.Port);
            try
                remaining=heartbeatTimeoutS-toc(endpointWait);
                assert(remaining>0,'gpenmpcNative:MavlinkSerialTimeout', ...
                    'Serial startup deadline elapsed before fresh heartbeat.');
                heartbeat=obj.waitForMessage("HEARTBEAT",remaining,[]);
                obj.TargetSystem=double(heartbeat.SystemID);
                obj.TargetComponent=double(heartbeat.ComponentID);
                assert(obj.TargetSystem>0&&obj.TargetComponent>0, ...
                    'gpenmpcNative:MavlinkSerialTarget','Invalid MAVLink target.');
            catch exception
                obj.close();
                rethrow(exception)
            end
        end

        function close(obj)
            port=obj.Port;obj.Port=[];
            if ~isempty(port)
                try,flush(port);catch,end
                try,delete(port);catch,end
            end
            obj.Endpoint="";
        end

        function value=isOpen(obj)
            value=~isempty(obj.Port);
            if value
                try,value=isvalid(obj.Port);catch,value=false;end
            end
        end

        function messages=drain(obj)
            obj.requireOpen();
            obj.pump();
            messages=obj.Inbox;
            obj.Inbox=cell(1,0);
        end

        function message=waitForMessage(obj,name,timeoutS,predicate)
            arguments
                obj
                name (1,1) string
                timeoutS (1,1) double {mustBePositive}
                predicate=[]
            end
            deadline=tic;
            message=[];
            while toc(deadline)<timeoutS
                obj.pump();
                match=0;
                for k=1:numel(obj.Inbox)
                    candidate=obj.Inbox{k};
                    if obj.messageName(candidate)~=name,continue,end
                    if ~isempty(predicate)&&~predicate(candidate),continue,end
                    match=k;break
                end
                if match>0
                    message=obj.Inbox{match};
                    obj.Inbox(match)=[];
                    return
                end
                pause(0.001);
            end
            error('gpenmpcNative:MavlinkSerialTimeout', ...
                'Timeout waiting for MAVLink %s.',name);
        end

        function sendMessage(obj,name,payload)
            obj.requireOpen();
            message=createmsg(obj.Dialect,name);
            names=fieldnames(payload);
            for k=1:numel(names)
                field=names{k};
                assert(isfield(message.Payload,field), ...
                    'gpenmpcNative:MavlinkPayloadField', ...
                    'Unknown %s payload field %s.',name,field);
                template=message.Payload.(field);
                value=payload.(field);
                if ischar(template)
                    encoded=char(value);encoded=encoded(:).';
                    assert(numel(encoded)<=numel(template), ...
                        'gpenmpcNative:MavlinkPayloadText', ...
                        'Text exceeds MAVLink payload field.');
                    target=repmat(char(0),size(template));
                    target(1:numel(encoded))=encoded;
                else
                    target=cast(value,'like',template);
                    assert(isequal(size(target),size(template)), ...
                        'gpenmpcNative:MavlinkPayloadShape', ...
                        'Unexpected %s.%s payload shape.',name,field);
                end
                message.Payload.(field)=target;
            end
            buffer=uint8(serializemsg(obj.Serializer,message));
            started=tic;
            write(obj.Port,buffer,"uint8");
            wallS=toc(started);
            obj.MaximumWriteWallS=max(obj.MaximumWriteWallS,wallS);
            obj.TxMessageCount=obj.TxMessageCount+uint64(1);
            obj.TxByteCount=obj.TxByteCount+uint64(numel(buffer));
        end

        function generation=commandLong(obj,command,parameters)
            values=double(parameters(:).');
            assert(numel(values)<=7&&all(isfinite(values)), ...
                'gpenmpcNative:MavlinkCommandParameters', ...
                'MAV_CMD accepts at most seven finite parameters.');
            values(end+1:7)=0;
            payload=struct('param1',values(1),'param2',values(2), ...
                'param3',values(3),'param4',values(4), ...
                'param5',values(5),'param6',values(6),'param7',values(7), ...
                'command',uint16(command),'target_system',uint8(obj.TargetSystem), ...
                'target_component',uint8(obj.TargetComponent),'confirmation',uint8(0));
            obj.sendMessage("COMMAND_LONG",payload);
            obj.CommandGeneration=obj.CommandGeneration+uint64(1);
            generation=obj.CommandGeneration;
        end

        function generation=requestMessage(obj,messageId)
            generation=obj.commandLong(512,double(messageId));
        end

        function generation=requestMessageInterval(obj,messageId,hz)
            assert(isfinite(hz)&&hz>0,'gpenmpcNative:MavlinkInterval', ...
                'Message frequency must be positive.');
            generation=obj.commandLong(511,[double(messageId),1e6/double(hz)]);
        end

        function generation=requestOffboard(obj)
            generation=obj.commandLong(176,[1,6,0]);
        end

        function generation=requestNativeLand(obj)
            generation=obj.commandLong(176,[1,4,6]);
        end

        function generation=requestArm(obj,armed,force)
            arguments
                obj
                armed (1,1) logical
                force (1,1) logical=false
            end
            generation=obj.commandLong(400, ...
                [double(armed),double(force)*21196]);
        end

        function sendHeartbeat(obj)
            obj.sendMessage("HEARTBEAT",struct( ...
                'custom_mode',uint32(0),'type',uint8(6),'autopilot',uint8(8), ...
                'base_mode',uint8(0),'system_status',uint8(4), ...
                'mavlink_version',uint8(3)));
        end

        function sendNeutralManualControl(obj)
            obj.sendMessage("MANUAL_CONTROL",struct('x',int16(0), ...
                'y',int16(0),'z',int16(500),'r',int16(0), ...
                'buttons',uint16(0),'target',uint8(obj.TargetSystem)));
        end

        function sendSetpoint(obj,timeBootMs,position,velocity,acceleration,yaw)
            p=double(position(:));v=double(velocity(:));a=double(acceleration(:));
            assert(numel(p)==3&&numel(v)==3&&numel(a)==3 ...
                &&all(isfinite([p;v;a;double(yaw)])), ...
                'gpenmpcNative:MavlinkSetpoint','Invalid Offboard reference.');
            obj.sendMessage("SET_POSITION_TARGET_LOCAL_NED",struct( ...
                'time_boot_ms',uint32(mod(double(timeBootMs),2^32)), ...
                'x',p(1),'y',p(2),'z',p(3), ...
                'vx',v(1),'vy',v(2),'vz',v(3), ...
                'afx',a(1),'afy',a(2),'afz',a(3), ...
                'yaw',double(yaw),'yaw_rate',0, ...
                'type_mask',uint16(2048), ...
                'target_system',uint8(obj.TargetSystem), ...
                'target_component',uint8(obj.TargetComponent), ...
                'coordinate_frame',uint8(obj.LocalNedFrame)));
        end

        function chunks=segmentPayload(obj,sequence,position,yaw,wind, ...
                payloadRemainingKg,initialPayloadKg)
            %#ok<INUSL>
            p=double(position(:));w=double(wind(:));
            assert(numel(p)==3&&numel(w)==3 ...
                &&all(isfinite([p;w;double(yaw);double(payloadRemainingKg)])), ...
                'gpenmpcNative:MavlinkSegment','Invalid segment metadata.');
            values=[repmat(p(:).',1,6),1,0,1,double(yaw),double(yaw),0.5, ...
                w(:).',9.5,double(payloadRemainingKg), ...
                max(double(initialPayloadKg)-double(payloadRemainingKg),0)];
            assert(numel(values)==30,'gpenmpcNative:MavlinkSegmentCount', ...
                'The GPENMPC segment body must contain 30 float fields.');
            body=[typecast(single(values),'uint8'),uint8([0,0,0,1])];
            assert(numel(body)==obj.RouteBodyBytes, ...
                'gpenmpcNative:MavlinkSegmentBytes','Unexpected segment bytes.');
            checksum=gpenmpcNative.MavlinkSerialLink.crc32(body);
            chunks=cell(1,2);
            for index=0:1
                offset=index*obj.RouteChunkBytes;
                part=body(offset+1:min(offset+obj.RouteChunkBytes,numel(body)));
                header=[typecast(obj.RouteMagic,'uint8'),obj.RouteVersion, ...
                    uint8(index),uint8(2),uint8(obj.RouteBodyBytes), ...
                    typecast(uint32(sequence),'uint8'),typecast(checksum,'uint8')];
                chunks{index+1}=uint8([header,part]);
            end
        end

        function sendSegmentChunk(obj,chunk)
            chunk=uint8(chunk(:).');
            assert(numel(chunk)<=128,'gpenmpcNative:MavlinkTunnelLength', ...
                'GPENMPC TUNNEL payload exceeds 128 bytes.');
            data=zeros(1,128,'uint8');data(1:numel(chunk))=chunk;
            obj.sendMessage("TUNNEL",struct( ...
                'payload_type',uint16(obj.RoutePayloadType), ...
                'target_system',uint8(obj.TargetSystem), ...
                'target_component',uint8(obj.TargetComponent), ...
                'payload_length',uint8(numel(chunk)),'payload',data));
        end

        function counts=sendHilSensors(obj,simTimeS,snapshot,logicalArmed, ...
                sensorSequence,sendGps)
            t=double(simTimeS);sensor=snapshot.sensor;
            specificForce=double(sensor.specific_force_body_mps2(:))+[ ...
                0.0120*sin(2*pi*13.7*t); ...
                0.0100*sin(2*pi*17.3*t+0.4); ...
                0.0140*sin(2*pi*19.1*t+0.9)];
            gyro=double(sensor.gyro_body_frd_rad_s(:))+[ ...
                0.0025*sin(2*pi*11.7*t+0.2); ...
                0.0022*sin(2*pi*15.1*t+0.7); ...
                0.0028*sin(2*pi*18.7*t+1.1)];
            magnetic=double(sensor.magnetic_body_gauss(:))+[ ...
                0.00008*sin(2*pi*0.73*t); ...
                0.00007*sin(2*pi*0.91*t+0.5); ...
                0.00008*sin(2*pi*1.09*t+1.0)];
            timestamp=uint64(round((t+0.001)*1e6));
            fields=uint32(hex2dec('1FFF'));
            if sensorSequence==0,fields=bitshift(uint32(1),31);end
            obj.sendMessage("HIL_SENSOR",struct( ...
                'time_usec',timestamp,'xacc',specificForce(1), ...
                'yacc',specificForce(2),'zacc',specificForce(3), ...
                'xgyro',gyro(1),'ygyro',gyro(2),'zgyro',gyro(3), ...
                'xmag',magnetic(1),'ymag',magnetic(2),'zmag',magnetic(3), ...
                'abs_pressure',double(sensor.pressure_mbar)+ ...
                    0.0060*sin(2*pi*13.7*t), ...
                'diff_pressure',0,'pressure_alt',double(sensor.pressure_alt_m), ...
                'temperature',20.0+0.0020*sin(2*pi*7.9*t+0.6), ...
                'fields_updated',fields));
            counts=struct('hil_sensor',1,'hil_state',0,'hil_gps',0, ...
                'logical_arm_observed',logical(logicalArmed));
            if ~sendGps,return,end
            origin=[-34.518;-75.710;0];
            position=double(snapshot.position_ned_m(:))-origin;
            velocity=double(snapshot.velocity_ned_mps(:));
            latitude=47.397742+position(1)/111111.0;
            longitude=8.545594+position(2)/(111111.0*cosd(47.397742));
            altitude=488.0-position(3);
            speed=norm(velocity);course=atan2(velocity(2),velocity(1));
            if course<0,course=course+2*pi;end
            obj.sendMessage("HIL_GPS",struct( ...
                'time_usec',timestamp,'lat',int32(round(latitude*1e7)), ...
                'lon',int32(round(longitude*1e7)), ...
                'alt',int32(round(altitude*1000)), ...
                'eph',uint16(50),'epv',uint16(80), ...
                'vel',uint16(max(0,min(65535,round(speed*100)))), ...
                'vn',int16(round(velocity(1)*100)), ...
                've',int16(round(velocity(2)*100)), ...
                'vd',int16(round(velocity(3)*100)), ...
                'cog',uint16(round(double(speed>0.05)*rad2deg(course)*100)), ...
                'fix_type',uint8(3),'satellites_visible',uint8(14), ...
                'id',uint8(0)));
            counts.hil_gps=1;
        end

        function row=requestParam(obj,name,timeoutS)
            arguments
                obj
                name (1,1) string
                timeoutS (1,1) double {mustBePositive}=2.5
            end
            encoded=char(name);assert(strlength(name)<=16, ...
                'gpenmpcNative:MavlinkParamName','Parameter name too long.');
            obj.sendMessage("PARAM_REQUEST_READ",struct( ...
                'param_index',int16(-1), ...
                'target_system',uint8(obj.TargetSystem), ...
                'target_component',uint8(obj.TargetComponent), ...
                'param_id',encoded));
            predicate=@(message)strcmp(obj.cleanText(message.Payload.param_id),name);
            message=obj.waitForMessage("PARAM_VALUE",timeoutS,predicate);
            raw=single(message.Payload.param_value);
            mavType=double(message.Payload.param_type);
            if mavType==6
                decoded=double(typecast(raw,'int32'));
            elseif mavType==5
                decoded=double(typecast(raw,'uint32'));
            else
                decoded=double(raw);
            end
            row=struct('name',name,'mav_type',mavType, ...
                'raw_float',double(raw),'raw_bits_hex',upper(dec2hex(typecast(raw,'uint32'),8)), ...
                'decoded',decoded);
        end

        function identity=collectIdentity(obj,timeoutS)
            arguments
                obj
                timeoutS (1,1) double {mustBePositive}=8
            end
            ids=[148,2,245];
            for id=ids,obj.requestMessage(id);end
            latest=struct();deadline=tic;lastVersionRequest=0;completeVersion=false;
            while toc(deadline)<timeoutS
                messages=obj.drain();
                for k=1:numel(messages)
                    message=messages{k};name=obj.messageName(message);
                    if any(name==["AUTOPILOT_VERSION","SYSTEM_TIME","EXTENDED_SYS_STATE"])
                        latest.(char(name))=message.Payload;
                    end
                end
                % SYSTEM_TIME is optional; require AUTOPILOT_VERSION and EXTENDED_SYS_STATE
                % for identity and landed-state checks.
                % A compatible reply may omit uid2. Request the complete identity again
                % within the timeout rather than inferring GUID bytes.
                completeVersion=isfield(latest,'AUTOPILOT_VERSION') ...
                    &&isfield(latest.AUTOPILOT_VERSION,'uid2') ...
                    &&numel(latest.AUTOPILOT_VERSION.uid2)==18 ...
                    &&any(latest.AUTOPILOT_VERSION.uid2~=0);
                if completeVersion&&isfield(latest,'EXTENDED_SYS_STATE')
                    break
                end
                if ~completeVersion&&toc(deadline)-lastVersionRequest>=.25
                    obj.requestMessage(148);lastVersionRequest=toc(deadline);
                end
                pause(0.002);
            end
            assert(completeVersion&&all(isfield(latest, ...
                {'AUTOPILOT_VERSION','EXTENDED_SYS_STATE'})), ...
                'gpenmpcNative:MavlinkIdentityTimeout', ...
                'Required complete board identity was not observed.');
            versionText=obj.shellCommand("ver all",3.5);
            uptimeText=obj.shellCommand("uptime",2.5);
            commit=regexp(versionText,'PX4 git-hash:\s*([0-9a-fA-F]{40})', ...
                'tokens','once');
            architecture=regexp(versionText,'HW arch:\s*(\S+)', ...
                'tokens','once');
            systemTime=[];
            if isfield(latest,'SYSTEM_TIME'),systemTime=latest.SYSTEM_TIME;end
            identity=struct('autopilot_version',latest.AUTOPILOT_VERSION, ...
                'system_time',systemTime, ...
                'extended_sys_state',latest.EXTENDED_SYS_STATE, ...
                'ver_all',versionText,'uptime',uptimeText, ...
                'parsed_commit',lower(firstToken(commit)), ...
                'parsed_hw_arch',firstToken(architecture));
        end

        function response=shellCommand(obj,command,timeoutS)
            arguments
                obj
                command (1,1) string
                timeoutS (1,1) double {mustBePositive}=2.5
            end
            allow=["gpenmpc_tunnel_bridge start","gpenmpc_tunnel_bridge stop", ...
                "gpenmpc_tunnel_bridge status","gpenmpc_se3_control start", ...
                "gpenmpc_se3_control stop","gpenmpc_se3_control status", ...
                "gpenmpc_trajectory_exec stop", ...
                "listener gpenmpc_se3_control_status 0 1", ...
                "listener failsafe_flags 0 1","listener vehicle_status 0 1", ...
                "listener offboard_control_mode 0 1","ver all","uptime","dmesg", ...
                "pwm_out status","ls /fs/microsd","param save","param status","mtd status", ...
                "bsondump /fs/mtd_params","bsondump /fs/microsd/parameters_backup.bson", ...
                "gpenmpc_trajectory_exec status","gpenmpc_rfly_canonical status", ...
                "gpenmpc_rfly_canonical_local status","gpenmpc_rfly_session status", ...
                "px4io status","px4io stop","dshot status", ...
                "board_adc status","board_adc start -n", ...
                "logger status","logger on","logger stop", ...
                "listener system_power -n 2"];
            assert(any(command==allow),'gpenmpcNative:MavlinkShellAllowlist', ...
                'Shell command is not allowlisted.');
            obj.sendSerialControl(uint8(newline));pause(0.08);
            obj.sendSerialControl(unicode2native(char(command+newline),'US-ASCII'));
            data=uint8([]);started=tic;lastData=tic;
            while toc(started)<timeoutS
                obj.pump();
                kept=cell(1,0);
                for k=1:numel(obj.Inbox)
                    message=obj.Inbox{k};
                    if obj.messageName(message)=="SERIAL_CONTROL" ...
                            &&double(message.Payload.device)==10 ...
                            &&double(message.Payload.count)>0
                        count=double(message.Payload.count);
                        data=[data,uint8(message.Payload.data(1:count))]; %#ok<AGROW>
                        lastData=tic;
                    else
                        kept{end+1}=message; %#ok<AGROW>
                    end
                end
                obj.Inbox=kept;
                text=native2unicode(data,'UTF-8');
                echoAt=strfind(lower(text),lower(char(command)));
                if ~isempty(echoAt)
                    promptAfter=strfind(text(echoAt(end)+strlength(command):end),'nsh>');
                    if ~isempty(promptAfter)&&toc(lastData)>0.15,break,end
                end
                pause(0.002);
            end
            response=string(native2unicode(data,'UTF-8'));
            assert(contains(lower(response),lower(command))&&contains(response,"nsh>"), ...
                'gpenmpcNative:MavlinkShellCompletion', ...
                'Shell command completion was not observed.');
        end

        function sendSerialControl(obj,data)
            bytes=uint8(data(:).');
            while ~isempty(bytes)
                count=min(70,numel(bytes));chunk=zeros(1,70,'uint8');
                chunk(1:count)=bytes(1:count);bytes(1:count)=[];
                obj.sendMessage("SERIAL_CONTROL",struct('baudrate',uint32(0), ...
                    'timeout',uint16(0),'device',uint8(10),'flags',uint8(6), ...
                    'count',uint8(count),'data',chunk));
            end
        end

        function status=counters(obj)
            status=struct('endpoint',obj.Endpoint,'baud',obj.Baud, ...
                'target_system',obj.TargetSystem, ...
                'target_component',obj.TargetComponent, ...
                'tx_messages',double(obj.TxMessageCount), ...
                'tx_bytes',double(obj.TxByteCount), ...
                'rx_frames',double(obj.RxFrameCount), ...
                'rx_bytes',double(obj.RxByteCount), ...
                'rx_discarded_bytes',double(obj.RxDiscardedByteCount), ...
                'rx_invalid_frames',double(obj.RxInvalidFrameCount), ...
                'maximum_write_wall_s',obj.MaximumWriteWallS, ...
                'open',obj.isOpen());
        end
    end

    methods (Access=private)
        function requireOpen(obj)
            assert(obj.isOpen(),'gpenmpcNative:MavlinkSerialClosed', ...
                'The MATLAB MAVLink serial owner is closed.');
        end

        function pump(obj)
            obj.requireOpen();
            available=double(obj.Port.NumBytesAvailable);
            if available>0
                bytes=uint8(read(obj.Port,available,"uint8"));bytes=bytes(:).';
                obj.RxByteCount=obj.RxByteCount+uint64(numel(bytes));
                obj.RxBuffer=[obj.RxBuffer,bytes];
            end
            [messages,remainder,audit]= ...
                gpenmpcNative.MavlinkSerialLink.parseFrames( ...
                    obj.Dialect,obj.RxBuffer);
            obj.RxBuffer=remainder;
            obj.RxDiscardedByteCount=obj.RxDiscardedByteCount+ ...
                uint64(audit.discarded_bytes);
            obj.RxInvalidFrameCount=obj.RxInvalidFrameCount+ ...
                uint64(audit.invalid_frames);
            obj.RxFrameCount=obj.RxFrameCount+uint64(numel(messages));
            obj.Inbox=[obj.Inbox,messages];
        end
    end

    methods (Static)
        function [messages,remainder,audit]=parseFrames(dialect,buffer)
            bytes=uint8(buffer(:).');messages=cell(1,0);
            discarded=0;invalid=0;
            while ~isempty(bytes)
                start=find(bytes==hex2dec('FD')|bytes==hex2dec('FE'),1);
                if isempty(start)
                    discarded=discarded+numel(bytes);bytes=uint8([]);break
                end
                if start>1
                    discarded=discarded+start-1;bytes(1:start-1)=[];
                end
                if numel(bytes)<2,break,end
                payloadBytes=double(bytes(2));
                if bytes(1)==hex2dec('FD')
                    if numel(bytes)<3,break,end
                    signed=bitand(bytes(3),uint8(1))~=0;
                    frameBytes=12+payloadBytes+13*double(signed);
                else
                    frameBytes=8+payloadBytes;
                end
                if numel(bytes)<frameBytes,break,end
                frame=bytes(1:frameBytes);bytes(1:frameBytes)=[];
                try
                    [decoded,status]=deserializemsg(dialect,frame, ...
                        OutputAllMessage=true);
                    if numel(decoded)==1&&numel(status)==1&&status==0
                        messages{end+1}=decoded; %#ok<AGROW>
                    else
                        invalid=invalid+1;
                    end
                catch
                    invalid=invalid+1;
                end
            end
            remainder=bytes;
            audit=struct('discarded_bytes',discarded, ...
                'invalid_frames',invalid,'messages',numel(messages), ...
                'remainder_bytes',numel(remainder));
        end

        function value=messageName(message)
            persistent dialect names
            if isempty(dialect)
                dialect=mavlinkdialect("common.xml",2);
                names=containers.Map('KeyType','double','ValueType','char');
            end
            id=double(message.MsgID);
            if isKey(names,id),value=string(names(id));return,end
            info=msginfo(dialect,id);value=string(info.MessageName);
            names(id)=char(value);
        end

        function value=cleanText(input)
            value=string(deblank(strrep(char(input),char(0),'')));
        end

        function value=modeName(customMode)
            number=uint32(customMode);
            main=double(bitand(bitshift(number,-16),uint32(255)));
            sub=double(bitand(bitshift(number,-24),uint32(255)));
            if main==3&&sub==0,value="POSCTL";
            elseif main==4&&sub==3,value="AUTO_LOITER";
            elseif main==4&&sub==6,value="AUTO_LAND";
            elseif main==6&&sub==0,value="OFFBOARD";
            else,value="PX4_MAIN_"+main+"_SUB_"+sub;
            end
        end

        function checksum=crc32(bytes)
            crc=java.util.zip.CRC32;
            crc.update(typecast(uint8(bytes(:)),'int8'));
            checksum=uint32(crc.getValue());
        end
    end
end

function value=firstToken(tokens)
if isempty(tokens),value="";else,value=string(tokens{1});end
end
