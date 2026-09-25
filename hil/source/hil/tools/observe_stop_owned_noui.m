function result=observe_stop_owned_noui(root,ownedPid,expectedExe,afterUserReplug,afterObservedFailedPlant,observationName,standardDisarmIfModelGround)
% Observe recovery through the validated safety-frame assembler and retain raw data.
build=fileparts(fileparts(mfilename('fullpath')));
if nargin<4,afterUserReplug=false;end
if nargin<5,afterObservedFailedPlant=false;end
if nargin<6,observationName='';end
if nargin<7,standardDisarmIfModelGround=false;end
assert(islogical(standardDisarmIfModelGround)&&isscalar(standardDisarmIfModelGround));
assert(islogical(afterUserReplug)&&isscalar(afterUserReplug));
assert(islogical(afterObservedFailedPlant)&&isscalar(afterObservedFailedPlant));
addpath(fullfile(build,'host_runtime'),fullfile(build,'matlab_validation'), ...
    fullfile(build,'m600_coptersim','matlab_validation'));
assert(isfolder(root)&&isscalar(ownedPid)&&ownedPid>0);
proc=System.Diagnostics.Process.GetProcessById(int32(ownedPid));
assert(~proc.HasExited&&strcmpi(char(proc.ProcessName),'CopterSimNoUI'));
assert(startsWith(string(expectedExe),string(fullfile(build,'live'))) ...
    &&strcmpi(char(proc.MainModule.FileName),char(expectedExe)));
if afterObservedFailedPlant
    % This release path requires an observed failed software model and fresh
    % board identity, disarmed and grounded state.
    prior=load(fullfile(root,'POST_FAILURE_UDP_OBSERVATION_EXISTING_ASSEMBLER.mat'),'result','ground','hb','ext');
    assert(~prior.result.safe_observation&&prior.ground.model_failed ...
        &&bitand(uint8(prior.hb.base_mode),uint8(128))==0&&prior.ext.landed_state==1);
    evidenceName='FAILED_MODEL_SAFE_SHUTDOWN_OBSERVATION.mat';
elseif afterUserReplug
    evidenceName='POST_USER_REPLUG_UDP_OBSERVATION_EXISTING_ASSEMBLER.mat';
else
    evidenceName='POST_FAILURE_UDP_OBSERVATION_EXISTING_ASSEMBLER.mat';
end
if ~isempty(observationName)
    % Retain the earlier no-device result before observing the reconnected board.
    assert(afterUserReplug&&~afterObservedFailedPlant&&~isempty(regexp(char(observationName), ...
        '^POST_USER_REPLUG_UDP_OBSERVATION_EXISTING_ASSEMBLER_[0-9]{3}\.mat$','once')));
    evidenceName=char(observationName);
end
if standardDisarmIfModelGround
    assert(~afterUserReplug&&~afterObservedFailedPlant&&isempty(observationName));
    prior=load(fullfile(root,'POST_FAILURE_UDP_OBSERVATION_EXISTING_ASSEMBLER.mat'),'result','ground','hb','ext');
    assert(~prior.result.safe_observation&&~prior.ground.model_failed&&prior.ground.ground_confirmed ...
        &&bitand(uint8(prior.hb.base_mode),uint8(128))~=0&&prior.ext.landed_state==4);
    evidenceName='POST_NATIVE_STANDARD_DISARM_RECOVERY.mat';
end
assert(~isfile(fullfile(root,evidenceName)));
d=mavlinkdialect(fullfile(build,'m600_coptersim','matlab_validation','+m600check','px4_health_events.xml'),2);
encoder=mavlinkio(d,'SystemID',255,'ComponentID',190);
u=[];truth=[];raw={};decoded={};ground=[];hb=[];ext=[];version=[];hbAt=-Inf;extAt=-Inf;vAt=-Inf;gAt=-Inf;rxBuffer=uint8([]);
result=struct('safe_observation',false,'owned_NoUI_stopped',false,'COM_open_before_NoUI_stop',0, ...
    'arm_requests',0,'mode_requests',0,'parameter_writes_before_NoUI_stop',0,'failure','', ...
    'after_user_replug',afterUserReplug,'after_observed_failed_plant',afterObservedFailedPlant, ...
    'failed_model_shutdown_only',false);
result.standard_disarm_requests=0;result.force_disarm_requests=0;result.disarm_ack=[];
guard=onCleanup(@closeUdp);watch=tic;lastIdentityRequest=-Inf;
policy=struct('expected_session_token',26090501,'payload_by_generation_kg',[2.21;1.75;.98;.55;0], ...
    'mass_by_generation_kg',[11.71;11.25;10.48;10.05;9.5],'allow_unbound_pre_session',true);
try
    u=udpport('datagram','IPV4','LocalHost','127.0.0.1','LocalPort',14550,'Timeout',0.01);
    truth=udpport('datagram','IPV4','LocalPort',30101,'Timeout',0.01);
    for id=[148 245 0]
        m=createmsg(d,'COMMAND_LONG');m.SystemID=uint8(255);m.ComponentID=uint8(190);
        m.Payload.target_system=uint8(1);m.Payload.target_component=uint8(1);
        m.Payload.command=uint16(512);m.Payload.confirmation=uint8(0);m.Payload.param1=single(id);
        bytes=serializemsg(encoder,m);write(u,bytes,'uint8','127.0.0.1',18570);
        raw{end+1}=struct('direction','READ_ONLY_REQUEST','time_s',toc(watch),'bytes',bytes); %#ok<AGROW>
    end
    while toc(watch)<12
        % Refresh the on-demand UID request to obtain concurrent identity evidence
        % within the 2 s observation window.
        if toc(watch)-lastIdentityRequest>=1
            m=createmsg(d,'COMMAND_LONG');m.SystemID=uint8(255);m.ComponentID=uint8(190);
            m.Payload.target_system=uint8(1);m.Payload.target_component=uint8(1);
            m.Payload.command=uint16(512);m.Payload.confirmation=uint8(0);m.Payload.param1=single(148);
            bytes=serializemsg(encoder,m);write(u,bytes,'uint8','127.0.0.1',18570);
            lastIdentityRequest=toc(watch);
            raw{end+1}=struct('direction','READ_ONLY_IDENTITY_REFRESH','time_s',lastIdentityRequest,'bytes',bytes); %#ok<AGROW>
        end
        n=u.NumDatagramsAvailable;
        if n>0
            rows=read(u,n,'uint8');
            for k=1:n
                bytes=bytesAt(rows,k);received=toc(watch);
                % Use the safety reader's frame assembler and CRC decoder with bounded
                % remainder storage. Retain received bytes before parsing.
                raw{end+1}=struct('direction','MAVLINK_RX','time_s',received,'bytes',bytes); %#ok<AGROW>
                rxBuffer=[rxBuffer bytes(:).'];
                [messageCells,rxBuffer,frameAudit]=gpenmpcNative.MavlinkSerialLink.parseFrames(d,rxBuffer);
                raw{end}.frame_audit=frameAudit;
                messages=[messageCells{:}];status=zeros(1,numel(messages));
                for j=1:numel(messages)
                    q=messages(j);decoded{end+1}=q; %#ok<AGROW>
                    if status(j)~=0||q.SystemID~=1||q.ComponentID~=1,continue,end
                    if q.MsgID==0,hb=q.Payload;hbAt=received;
                    elseif q.MsgID==245,ext=q.Payload;extAt=received;
                    elseif q.MsgID==148,version=q.Payload;vAt=received;
                    elseif q.MsgID==77&&q.Payload.command==400&&result.standard_disarm_requests>0
                        result.disarm_ack=q;
                    end
                end
            end
        end
        n=truth.NumDatagramsAvailable;
        if n>0
            rows=read(truth,n,'uint8');
            for k=1:n
                bytes=bytesAt(rows,k);received=toc(watch);
                raw{end+1}=struct('direction','MODEL_RX','time_s',received,'bytes',bytes); %#ok<AGROW>
                if numel(bytes)==264
                    q=m600check.decodeCopterSimDeliveryDiagnostics(bytes,1,NaN,policy);
                    if q.packet_valid,ground=q;gAt=received;end
                end
            end
        end
        now=toc(watch);
        if standardDisarmIfModelGround&&result.standard_disarm_requests==0 ...
                &&~isempty(hb)&&~isempty(ext)&&~isempty(version)&&~isempty(ground) ...
                &&max([now-hbAt now-extAt now-vAt now-gAt])<2 ...
                &&strcmp(sprintf('%u',uint64(version.uid)),gpenmpc_device_identity('uid')) ...
                &&version.board_version==56&&version.product_id==56 ...
                &&strcmp(upper(reshape(dec2hex(uint8(version.uid2),2).',1,[])),gpenmpc_device_identity('px4_guid')) ...
                &&bitand(uint8(hb.base_mode),uint8(128))~=0&&ext.landed_state==4 ...
                &&bitand(bitshift(uint32(hb.custom_mode),-16),uint32(255))==4 ...
                &&bitand(bitshift(uint32(hb.custom_mode),-24),uint32(255))==6 ...
                &&ground.ground_confirmed&&~ground.model_failed
            % Request ordinary disarm during grounded AUTO_LAND recovery.
            % Commander retains its disarm checks.
            m=createmsg(d,'COMMAND_LONG');m.SystemID=uint8(255);m.ComponentID=uint8(190);
            m.Payload.target_system=uint8(1);m.Payload.target_component=uint8(1);
            m.Payload.command=uint16(400);m.Payload.confirmation=uint8(0);
            for parameter=1:7,m.Payload.(sprintf('param%d',parameter))=single(0);end
            result.standard_disarm_requests=1;
            bytes=serializemsg(encoder,m);
            raw{end+1}=struct('direction','STANDARD_NONFORCE_DISARM','time_s',now,'bytes',bytes);
            write(u,bytes,'uint8','127.0.0.1',18570);
        end
        if ~isempty(hb)&&~isempty(ext)&&~isempty(version)&&~isempty(ground) ...
                &&max([now-hbAt now-extAt now-vAt now-gAt])<2 ...
                &&strcmp(sprintf('%u',uint64(version.uid)),gpenmpc_device_identity('uid')) ...
                &&version.board_version==56&&version.product_id==56 ...
                &&strcmp(upper(reshape(dec2hex(uint8(version.uid2),2).',1,[])),gpenmpc_device_identity('px4_guid')) ...
                &&bitand(uint8(hb.base_mode),uint8(128))==0&&ext.landed_state==1 ...
                &&(ground.ground_confirmed||((afterUserReplug||afterObservedFailedPlant)&&ground.model_failed))
            % A failed frozen plant cannot establish landing after a board power cycle.
            % Fresh board disarmed and grounded state permits releasing the stale producer.
            result.failed_model_shutdown_only=~ground.ground_confirmed;
            result.safe_observation=true;break
        end
        pause(.002);
    end
catch ex,result.failure=getReport(ex,'extended','hyperlinks','off');end
closeUdp();clear guard
save(fullfile(root,evidenceName),'result','raw','decoded','hb','ext','version','ground','hbAt','extAt','vAt','gAt');
assert(result.safe_observation,'gpenmpcRecovery:NoSafeObservation','Retain NoUI: no fresh same-board disarmed/PX4-ground/model-ground observation. %s',result.failure);
proc.Kill();proc.WaitForExit(10000);result.owned_NoUI_stopped=proc.HasExited;proc.Dispose();
assert(result.owned_NoUI_stopped,'gpenmpcRecovery:Owner','Only our already-disarmed ground NoUI must exit.');
if afterObservedFailedPlant||standardDisarmIfModelGround
    % Record process release in this recovery result.
    save(fullfile(root,evidenceName),'result','raw','decoded','hb','ext','version','ground','hbAt','extAt','vAt','gAt');
end
    function closeUdp()
        if ~isempty(u),delete(u);u=[];end
        if ~isempty(truth),delete(truth);truth=[];end
    end
end
function b=bytesAt(rows,k)
if istable(rows)
    v=rows.Data;if iscell(v),b=v{k};else,b=v(k,:);end
else,b=rows(k).Data;end
b=uint8(b(:));
end
