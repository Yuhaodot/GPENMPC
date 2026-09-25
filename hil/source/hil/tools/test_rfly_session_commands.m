function report=test_rfly_session_commands(outputRoot,scope)
% Test session commands with the MAVLink codec.
arguments,outputRoot (1,1) string,scope (1,1) string="ENCODER",end
build=string(fileparts(fileparts(mfilename('fullpath'))));addpath(fullfile(build,'host_runtime'));
assert(~isfile(fullfile(outputRoot,'RESULT.json')));if ~isfolder(outputRoot),mkdir(outputRoot);end
if scope=="COMPONENT_LOOP_ONLY"||scope=="ENV_SERVICE_ORDER_ONLY"||scope=="EKF_PREARM_ONLY"||scope=="EKF_PROVISIONING_ONLY"||scope=="COMPONENT_HISTORY_ORDER_ONLY"||scope=="AUXILIARY_ATTITUDE_STREAM_ONLY"||scope=="GROUND_FINALIZER_ONLY"
    if scope=="COMPONENT_LOOP_ONLY",report=checkComponentCallbacks();
    elseif scope=="EKF_PREARM_ONLY",report=checkPrearmEstimator();
    elseif scope=="EKF_PROVISIONING_ONLY",report=checkEkfProvisioning(outputRoot);
    elseif scope=="COMPONENT_HISTORY_ORDER_ONLY",report=checkComponentHistoryOrder(build);
    elseif scope=="AUXILIARY_ATTITUDE_STREAM_ONLY",report=checkAuxiliaryAttitudeStream(build);
    elseif scope=="GROUND_FINALIZER_ONLY",report=checkGroundFinalizer(build);
    else,report=checkEnvironmentServiceOrder();end
    save(fullfile(outputRoot,'RAW.mat'),'report');
    f=fopen(fullfile(outputRoot,'RESULT.json'),'wt');assert(f>0);c=onCleanup(@()fclose(f));
    fprintf(f,'%s\n',jsonencode(report));disp(jsonencode(report));return
end

function report=checkGroundFinalizer(build)
% Test the production finalizer branch with in-memory IO.
source=fileread(fullfile(build,'tools','run_m600_board_local_short_hil.m'));
first=strfind(source,'            if stateFresh()&&latest.armed==1');
last=strfind(source,'            safeGround=stateFresh()&&latest.armed==0&&latest.landed_state==1&&ground();');
assert(isscalar(first)&&isscalar(last)&&first<last);
body=source(first:last+numel('            safeGround=stateFresh()&&latest.armed==0&&latest.landed_state==1&&ground();')-1);
cfg=struct('mode_timeout_s',1,'land_timeout_s',1,'disarm_timeout_s',1);
c=struct('poll_period_s',.001);io=struct('now',@()0,'sleep',@(x)[]);
t=0; % The production branch assigns this timer inside the static workspace.
latest=struct('armed',1,'landed_state',1);fresh=true;plantGround=true;reject=false;calls=[];
counts=struct('land_requests',0,'disarm_requests',0);safeGround=false;
eval(body);assert(isequal(calls,400)&&safeGround&&counts.land_requests==0);
latest.armed=1;latest.landed_state=2;plantGround=false;calls=[];
eval(body);assert(isequal(calls,[176 400])&&safeGround);
latest.armed=1;latest.landed_state=1;plantGround=false;calls=[];
eval(body);assert(isequal(calls,[176 400])&&safeGround); % PX4 alone is insufficient.
latest.armed=0;calls=[];eval(body);assert(isempty(calls)&&safeGround);
latest.armed=1;fresh=false;calls=[];eval(body);assert(isempty(calls)&&~safeGround);
fresh=true;reject=true;calls=[];id='';
try,eval(body);catch ex,id=ex.identifier;end
assert(strcmp(id,'fixture:DisarmRejected')&&isequal(calls,400)&&latest.armed==1);
report=struct('passed',true,'checks',6,'scope','PRODUCTION_GROUND_FINALIZER_BRANCH', ...
    'hardware_actions',0,'force_requests',0,'stale_state_sends_nothing',true, ...
    'disarm_rejection_does_not_fabricate_safe_state',true);
    function yes=stateFresh(),yes=fresh;end
    function yes=ground(),yes=plantGround;end
    function yes=isDisarmed(),yes=latest.armed==0;end
    function yes=nativeLandMode(),yes=true;end
    function safetyPump(),end
    function command(id,parameters,timeout,condition,label) %#ok<INUSD>
        calls(end+1)=id;
        if id==176
            assert(isequal(parameters,[1 4 6 0 0 0 0]));latest.landed_state=1;plantGround=true;
        else
            assert(id==400&&isequal(parameters,[0 0 0 0 0 0 0]));
            if reject,error('fixture:DisarmRejected','Fixture ordinary disarm was rejected.');end
            latest.armed=0;
        end
        assert(condition());
    end
end

function report=checkAuxiliaryAttitudeStream(build)
% Test auxiliary telemetry setup with an in-memory command callback.
source=fileread(fullfile(build,'tools','run_m600_board_local_short_hil.m'));
first=strfind(source,'    sessionRaw.attitude_telemetry_interval_us=50000;');
last=strfind(source,"    waitUntil(@ready,cfg.preflight_deadline_s,'PREFLIGHT_AFTER_TELEMETRY_CONFIGURATION');");
assert(isscalar(first)&&isscalar(last)&&first<last);body=source(first:last-1);
sessionRaw=struct();cfg=struct('command_timeout_s',3);calls={};reject=false;
eval(body);
assert(numel(calls)==1&&calls{1}.id==511&&isequal(calls{1}.parameters,[30 50000 0 0 0 0 0]));
assert(sessionRaw.attitude_telemetry_interval_us==50000&&calls{1}.condition);
reject=true;id='';try,eval(body);catch ex,id=ex.identifier;end
assert(strcmp(id,'fixture:CommandAckRejected'));
% Verify that auxiliary telemetry setup contains only the interval command.
assert(~contains(body,'source_max_age')&&~contains(body,'local_input_host_max_age') ...
    &&~contains(body,'requestArm')&&~contains(body,'setParameter'));
report=struct('passed',true,'checks',4,'scope','PRODUCTION_AUXILIARY_ATTITUDE_STREAM_SETUP_ONLY', ...
    'command_id',511,'message_id',30,'interval_us',50000,'ack_rejection_propagates',true, ...
    'hardware_actions',0);
    function command(id,parameters,timeout,condition,label)
        assert(timeout==3&&strcmp(label,'SET_AUXILIARY_ATTITUDE_TELEMETRY_20HZ'));
        calls{end+1}=struct('id',id,'parameters',parameters,'condition',condition());
        assert(~reject,'fixture:CommandAckRejected','Rejected setup cannot reach pre-arm.');
    end
end
assert(scope=="ENCODER");
d=mavlinkdialect('common.xml',2);codec=mavlinkio(d,SystemID=255,ComponentID=190);
cleanup=onCleanup(@()delete(codec)); %#ok<NASGU>
target=struct('system',uint8(1),'component',uint8(1));checks=struct('name',{},'pass',{});raw=struct();
high=bitor(bitshift(uint64(1),63),uint64(123));low=bitshift(uint64(1),53)+uint64(9);
v=struct('challenge',[high;low],'origin_ned_m',[7;-11;-0.0], ...
    'host_system',uint8(255),'host_component',uint8(190));
[messages,r]=gpenmpcNative.RflySessionCommandEncoder('prepare',v,d,target);
check('exact_uint64_challenge',contains(r.original_command,'800000000000007B0020000000000009'));
check('fixed_device_and_no_auto_actions',startsWith(r.original_command,'gpenmpc_rfly_session prepare /dev/ttyACM0 ') ...
    &&endsWith(r.original_command,' 255 190')&&~r.io_sent&&~r.board_acknowledged);
check('all_prepare_chunks_official_codec',roundtrip(messages,r));raw.prepare=r;
session='1234567890ABCDEF1234567890ABCDEF1234567890ABCDEF1234567890ABCDEF';
prepared=struct('schema','GPENMPC_RFLY_PREPARED_SESSION_RECEIPT_V1','registration_result','Registered', ...
    'echo_confirmation_result','NOT_CONFIRMED','execution_session_sha256',session);
v2=struct('prepared_receipt',prepared,'physical_setup_record_sha256',repmat('A',1,64));
[messages,r]=gpenmpcNative.RflySessionCommandEncoder('confirm',v2,d,target);raw.confirm=r;
check('confirm_records_actual_original_arguments',strcmp(r.original_confirm_session_sha256,session) ...
    &&strcmp(r.confirmed_physical_setup_record_sha256,repmat('A',1,64))&&roundtrip(messages,r));
    for action=["start","stop","status","module_status","release","evidence_failed","evidence_interrupted"]
    [messages,r]=gpenmpcNative.RflySessionCommandEncoder(action,struct(),d,target);
    check(char(action)+"_exact_existing_entry_codec",roundtrip(messages,r)&&r.hardware_actions==0);raw.(action)=r;
end
for action=["stream_snapshot","stream_feedback"]
    for enabled=[false,true]
        [messages,r]=gpenmpcNative.RflySessionCommandEncoder(action,struct('enabled',enabled),d,target);
        expected=' -r 0';if enabled,expected=' -r -1';end
        check(action+"_enabled_"+enabled,endsWith(r.original_command,expected)&&roundtrip(messages,r));
    end
end
rejected('unlisted_arm',@()gpenmpcNative.RflySessionCommandEncoder('arm',struct(),d,target));
rejected('unlisted_mode',@()gpenmpcNative.RflySessionCommandEncoder('mode',struct(),d,target));
rejected('arbitrary_shell',@()gpenmpcNative.RflySessionCommandEncoder('status; reboot',struct(),d,target));
rejected('extra_argument',@()gpenmpcNative.RflySessionCommandEncoder('status',struct('cmd','reboot'),d,target));
rejected('double_challenge',@()gpenmpcNative.RflySessionCommandEncoder('prepare',set(v,'challenge',double(v.challenge)),d,target));
rejected('zero_challenge',@()gpenmpcNative.RflySessionCommandEncoder('prepare',set(v,'challenge',uint64([0;0])),d,target));
rejected('nan_origin',@()gpenmpcNative.RflySessionCommandEncoder('prepare',set(v,'origin_ned_m',[0;NaN;0]),d,target));
rejected('infinite_origin',@()gpenmpcNative.RflySessionCommandEncoder('prepare',set(v,'origin_ned_m',[0;Inf;0]),d,target));
rejected('arbitrary_uart',@()gpenmpcNative.RflySessionCommandEncoder('prepare',set(v,'device','/dev/ttyS0'),d,target));
rejected('absent_confirmation',@()gpenmpcNative.RflySessionCommandEncoder('confirm',set(v2,'prepared_receipt',struct()),d,target));
rejected('already_confirmed_not_original_prepare',@()gpenmpcNative.RflySessionCommandEncoder('confirm', ...
    set(v2,'prepared_receipt',set(prepared,'echo_confirmation_result','Confirmed')),d,target));
rejected('forged_schema',@()gpenmpcNative.RflySessionCommandEncoder('confirm', ...
    set(v2,'prepared_receipt',set(prepared,'schema','ARBITRARY_SELF_GRANT')),d,target));
rejected('digest_shell_injection',@()gpenmpcNative.RflySessionCommandEncoder('confirm', ...
    set(v2,'physical_setup_record_sha256',[repmat('A',1,64) '; reboot']),d,target));
rejected('zero_declaration_digest',@()gpenmpcNative.RflySessionCommandEncoder('confirm',set(v2,'physical_setup_record_sha256',repmat('0',1,64)),d,target));
rejected('stream_arbitrary_rate',@()gpenmpcNative.RflySessionCommandEncoder('stream_snapshot',struct('enabled',true,'rate',99999),d,target));
rejected('stream_nonlogical',@()gpenmpcNative.RflySessionCommandEncoder('stream_snapshot',struct('enabled',1),d,target));
rejected('broadcast_target',@()gpenmpcNative.RflySessionCommandEncoder('status',struct(),d,set(target,'system',uint8(0))));
report=struct('scope','HOST_ONLY_EXACT_NSH_COMMAND_ENCODER_OFFICIAL_CODEC','passed',all([checks.pass]), ...
    'checks',checks,'checks_total',numel(checks),'connections',0,'hardware_actions',0, ...
    'no_command_sent',true,'no_board_ack_proven',true);
save(fullfile(outputRoot,'RAW.mat'),'raw','report');
f=fopen(fullfile(outputRoot,'RESULT.json'),'wt');assert(f>0);c=onCleanup(@()fclose(f));fprintf(f,'%s\n',jsonencode(report,PrettyPrint=true));clear c
disp(jsonencode(struct('passed',report.passed,'checks',report.checks_total,'hardware_actions',0)));
assert(report.passed);
    function check(n,p),checks(end+1)=struct('name',char(n),'pass',logical(p));end
    function rejected(n,fn),ok=false;try,fn();catch,ok=true;end;check(n,ok);end
    function ok=roundtrip(messages,receipt)
        assembled=uint8([]);ok=numel(messages)==receipt.fragments;
        for j=1:numel(messages)
            wire=serializemsg(codec,messages{j});[back,status]=deserializemsg(d,wire);p=back.Payload;
            ok=ok&&status==0&&back.MsgID==126&&back.SystemID==255&&back.ComponentID==190 ...
                &&p.device==10&&p.flags==6&&p.timeout==0&&p.baudrate==0&&p.count>0&&p.count<=70;
            assembled=[assembled;p.data(1:double(p.count)).']; %#ok<AGROW>
        end
        ok=ok&&isequal(assembled(:),receipt.original_command_bytes(:)) ...
            &&sum(assembled==10)==1&&assembled(end)==10&&all(assembled>=10)&all(assembled<128);
    end
end
function report=checkEkfProvisioning(outputRoot)
src=fileread(fullfile(fileparts(mfilename('fullpath')),'m600_local_application_handoff.m'));
match=regexp(src,' function setEkfInstanceCount\(desired\)\r?\n([\s\S]*?)\r?\n end\r?\nend\r?\nfunction s=utc','tokens','once');
assert(~isempty(match));body=match{1};checks=0;
outputPath=fullfile(outputRoot,'MOCK_ONLY_ATTEMPT');
link=struct('requestParam',@read,'drain',@drain,'requestMessage',@request,'waitForMessage',@wait,'sendMessage',@send,'shellCommand',@shell);
q=[];expected='';e=[];b=[];p=[];desired=1;current=3;valueType=6;armed=false;badEcho=false;badRead=false;writes=0;result=struct(); %#ok<NASGU>
saveFailure=false;saveIncomplete=false;saves=0;savedText='';savedAt=[]; %#ok<NASGU>
for scenario=1:10
    current=3;desired=1;valueType=6;armed=false;badEcho=false;badRead=false;writes=0;result=struct('parameter_writes',0);
    saveFailure=scenario==9;saveIncomplete=scenario==10;saves=0;
    if scenario==2,current=1;desired=3;end
    if scenario==3,desired=3;end
    if scenario==4,current=2;end
    if scenario==5,valueType=9;end
    if scenario==6,armed=true;end
    if scenario==7,badEcho=true;end
    if scenario==8,badRead=true;end
    caught='';try,eval(body);catch ex,caught=ex.identifier;end
    if scenario<=3
        assert(isempty(caught)&&current==desired&&writes==double(scenario~=3)&&saves==1&&result.ekf_instance_save_verified, ...
            'fixture:EkfProvisioning','Scenario %d failed: %s',scenario,caught);
    else
        assert(startsWith(caught,'gpenmpc:EkfInstance'));
        assert(writes==double(scenario>=7));
        assert(saves==double(scenario>=9));
    end
    checks=checks+1;
end
report=struct('passed',true,'checks',checks,'scope','ACTUAL_SINGLE_HIL_IMU_APPLY_RESTORE_BODY_NO_IO', ...
    'hardware_actions',0,'control_commits',0);
    function r=read(varargin)
        v=current;if badRead&&writes>0,v=2;end
        r=struct('name','EKF2_MULTI_IMU','mav_type',valueType,'raw_bits_hex',dec2hex(uint32(v),8));
    end
    function drain(),end
    function request(varargin),end
    function m=wait(name,varargin)
        if strcmp(name,'EXTENDED_SYS_STATE'),payload=struct('landed_state',1);
        elseif strcmp(name,'HEARTBEAT'),payload=struct('base_mode',uint8(128*double(armed)));
        else
            v=current;if badEcho,v=2;end
            payload=struct('param_type',uint8(6),'param_value',typecast(int32(v),'single'));
        end
        m=struct('Payload',payload);
    end
    function send(name,payload)
        assert(strcmp(name,'PARAM_SET')&&strcmp(payload.param_id,'EKF2_MULTI_IMU')&&payload.param_type==6 ...
            &&payload.target_system==1&&payload.target_component==1);
        current=double(typecast(payload.param_value,'int32'));writes=writes+1;
    end
    function text=shell(command,timeout)
        assert(strcmp(command,'param save')&&timeout==5);saves=saves+1;
        text=sprintf('nsh> param save\nnsh> ');
        if saveFailure,text=sprintf('nsh> param save\nERROR [param] Param save failed (-1)\nnsh> ');end
        if saveIncomplete,text=sprintf('nsh> param save\n');end
    end
end
function report=checkPrearmEstimator()
% Execute only the affected production pre-arm body with no-I/O callbacks.
src=fileread(fullfile(fileparts(mfilename('fullpath')),'run_m600_board_local_short_hil.m'));
body=extractBetween(string(src),"    function restartPrearmEstimator()","    function frames=framesAfter(submit)");
assert(numel(body)==1);body=regexprep(char(body),'\s+end\s*$','');
d=mavlinkdialect('common.xml',2);codec=mavlinkio(d,SystemID=255,ComponentID=190);
clean=onCleanup(@()delete(codec)); %#ok<NASGU>
target=struct('system',uint8(1),'component',uint8(1));checks=0;
for action=["prearm_ekf_stop","prearm_ekf_start","prearm_ekf_status"]
    [messages,r]=gpenmpcNative.RflySessionCommandEncoder(action,struct(),d,target);
    assert(numel(messages)==1&&strcmp(r.original_command,['ekf2 ' char(extractAfter(action,'prearm_ekf_'))]));
    [m,s]=deserializemsg(d,serializemsg(codec,messages{1}));
    assert(s==0&&m.Payload.device==10&&isequal(uint8(m.Payload.data(1:double(m.Payload.count))).',r.original_command_bytes));
    checks=checks+1;
    caught=false;try,gpenmpcNative.RflySessionCommandEncoder(action,struct('cmd','arm'),d,target);catch,caught=true;end
    assert(caught);checks=checks+1;
end
service=[];registered=[];counts=struct('commits',0,'arm_requests',0);
cfg=struct('command_timeout_s',.2,'preflight_deadline_s',45);c=struct('poll_period_s',.01);
io=struct('now',@clock,'sleep',@sleep,'sendCanonicalSession',@send,'canonicalSessionReceipts',@receipts);
t=0;preflightStarted=0;phase='';sessionRaw=struct();latest=struct('armed',0,'landed_state',1);
caseName='';calls={};lastCommand='';
sent=[];begin=0;completed=false;frames={};bytes=uint8([]);f=0;p=struct();text='';echo=[];tail=''; %#ok<NASGU>
for scenario=["success","stop_unconfirmed","start_failed","no_prompt","armed","acquisition_expired"]
    caseName=char(scenario);t=0;preflightStarted=0;calls={};sessionRaw=struct();latest.armed=0;
    if scenario=="armed",latest.armed=1;end
    if scenario=="acquisition_expired",t=45;end
    caught='';try,eval(body);catch ex,caught=ex.identifier;end
    if scenario=="success"
        assert(isempty(caught)&&isequal(calls,{'prearm_ekf_stop','prearm_ekf_start','prearm_ekf_status'}), ...
            'test:PrearmSuccess','Production pre-arm body failed: %s',caught);
        assert(sessionRaw.prearm_ekf_initialization.prearm_ekf_status.completed);
    else
        assert(startsWith(caught,'gpenmpcShort:Estimator'));
        if scenario=="armed"||scenario=="acquisition_expired",assert(isempty(calls));
        elseif scenario=="start_failed",assert(numel(calls)==2);
        else,assert(isequal(calls,{'prearm_ekf_stop'}));end
    end
    assert(t<46&&counts.commits==0&&counts.arm_requests==0);checks=checks+1;
end
report=struct('passed',true,'checks',checks,'scope','ACTUAL_PREARM_EKF_INITIALIZATION_BODY_AND_FIXED_COMMAND_CODEC_NO_IO', ...
    'hardware_actions',0,'control_commits',0);
    function v=clock(),v=t;end
    function sleep(dt),t=t+dt;end
    function pump(),t=t+.02;end
    function yes=stateFresh(),yes=true;end
    function yes=ground(),yes=true;end
    function sent=send(action,value)
        calls{end+1}=char(action);
        [~,encoding]=gpenmpcNative.RflySessionCommandEncoder(action,value,d,target);
        lastCommand=encoding.original_command;
        sent=struct('original_host_submit_ns',uint64(1000),'encoding',encoding);
    end
    function frames=receipts(varargin)
        response='';
        if strcmp(lastCommand,'ekf2 stop'),response='INFO [ekf2] stopping ekf2 instance 0';end
        if strcmp(lastCommand,'ekf2 status'),response='ekf2:0 EKF dt: 0.0100s, attitude: 1, local position: 1, global position: 1';end
        if strcmp(caseName,'stop_unconfirmed'),response='unexpected';end
        if strcmp(caseName,'start_failed')&&strcmp(lastCommand,'ekf2 start'),response='ERROR [ekf2] start failed';end
        prompt='nsh>';if strcmp(caseName,'no_prompt'),prompt='';end
        bytes=uint8([lastCommand newline response newline prompt]);frames={};
        for n=1:70:numel(bytes)
            part=bytes(n:min(n+69,numel(bytes)));
            frames{end+1}=struct('decoded_message',struct('Payload',struct('device',10,'count',numel(part),'data',part)));
        end
    end
end
function report=checkEnvironmentServiceOrder()
% Test the command prefix with no-IO call spies.
text=fileread(fullfile(fileparts(mfilename('fullpath')),'run_m600_board_local_short_hil.m'));
body=extractBetween(string(text),"    function serviceRuntimeIo()","        b=getterMex('drain');");
assert(numel(body)==1);body=char(body);
trace={};latest=struct();c=struct('component_initialization',true,'runtime_state_only',true);finalized=false;faultAt='';
fullRuntime=false;
io=struct('snapshot',@snapshot);checks=0;
eval(body);assert(isequal(trace,{'model','env_model','full','env_full'}));checks=checks+1;
trace={};eval(body);assert(isequal(trace,{'model','env_model','full','env_full'}));checks=checks+1;
c.component_initialization=false;trace={};eval(body);assert(isequal(trace,{'model','env_model'}));checks=checks+1;
c.runtime_state_only=false;trace={};eval(body);assert(isequal(trace,{'full','env_full'}));checks=checks+1;
c.component_initialization=true;c.runtime_state_only=true;faultAt='model';trace={};
try,eval(body);error('test:MissingReject');catch e,assert(strcmp(e.identifier,'gpenmpcShort:OriginalIoFatal'));end
assert(isequal(trace,{'model'}));checks=checks+1;
faultAt='full';trace={};
try,eval(body);error('test:MissingReject');catch e,assert(strcmp(e.identifier,'gpenmpcShort:OriginalIoFatal'));end
assert(isequal(trace,{'model','env_model','full'}));checks=checks+1;
report=struct('passed',true,'checks',checks,'scope','ACTUAL_SERVICE_PREFIX_EXECUTED_WITH_NO_IO_SPIES', ...
    'continuous_component_order',true,'real_fatal_not_ignored',true, ...
    'hardware_actions',0,'control_commits',0);
    function s=snapshot(~,readMav)
        if nargin<2||readMav,kind='full';else,kind='model';end
        trace{end+1}=kind;s=struct('fatal','','kind',kind);
        if strcmp(faultAt,kind),s.fatal='ACTUAL_FAULT_FIXTURE';end
    end
    function heartbeat(~),end
    function refreshEnvironment(),trace{end+1}=['env_' latest.kind];end
end
function s=set(s,k,v),s.(k)=v;end
function report=checkComponentCallbacks()
latest=struct('fresh',true,'main_mode',1,'armed',0);firstCommit=NaN;sessionRaw=struct();
old=@()latest.main_mode==6;current=@offboardDisarmed;commit=@hasFirstCommit;
assert(~old()&&~current()&&~commit()&&~dispatch());
latest.main_mode=6;assert(~old()&&current());
latest.fresh=false;assert(~current());latest.fresh=true;latest.armed=1;assert(~current());
firstCommit=12.3;assert(commit());sessionRaw.start_observation=[];assert(~dispatch());
sessionRaw.start_observation=struct('task_created',false);assert(~dispatch());
sessionRaw.start_observation.task_created=true;assert(dispatch());
report=struct('passed',true,'scope','AFFECTED_MATLAB_CALLBACK_CAPTURE_AND_PRESTART_DISPATCH', ...
    'old_callback_retains_old_state',true,'nested_callback_reads_current_state',true, ...
    'stale_and_wrong_arm_state_rejected',true,'no_method_dispatch_before_task_created',true, ...
    'hardware_actions',0,'control_commits',0);
    function yes=offboardDisarmed(),yes=latest.fresh&&latest.main_mode==6&&latest.armed==0;end
    function yes=hasFirstCommit(),yes=isfinite(firstCommit);end
    function yes=dispatch(),yes=isfield(sessionRaw,'start_observation')&&~isempty(sessionRaw.start_observation)&&sessionRaw.start_observation.task_created;end
end

function report=checkComponentHistoryOrder(build)
% Test dequeue, decode, send guards and counters with retained RLC2 bytes;
% replace transport and phase ownership with spies.
a=load(fullfile(gpenmpc_external_path('prestart_control_stop'),'CONTROL_STOP_EXISTING_RAW_SUBSET.mat'),'sel');
ids=find(cellfun(@(q)isfield(q,'committed')&&~isempty(q.committed),a.sel.methodRaw));
assert(~isempty(ids));record=a.sel.methodRaw{ids(1)}.committed{1}.raw;
source=fileread(fullfile(build,'host_runtime','+gpenmpcNative','RflyLocalMethodService.m'));
start=strfind(source,"                if obj.ComponentInitialization&&isfield(event,'input_send')");
stop=strfind(source,"                mark=obj.now();event.window=obj.serviceWindow");
assert(numel(start)==1&&numel(stop)==1&&start<stop);body=source(start:stop-1);
queue={record};takeCalls=0;ingestCalls=0;rejectPhase=false;
obj=struct('ComponentInitialization',true,'Io',struct('takeCanonical',@take), ...
    'Phase',struct('ingest',@ingest),'now',@()uint64(900000000), ...
    'LastCommitSource',uint64(0),'Counts',struct('commits',0));
event=struct;mark=uint64(0);k=0;c=[];dc=[]; %#ok<NASGU>
event=struct('committed',{{}},'work_timing_ns',struct());eval(body);
assert(takeCalls==0&&ingestCalls==0&&numel(queue)==1&&obj.Counts.commits==0);
event.input_send=struct('messages_send_returned',5);eval(body);
assert(takeCalls==0&&numel(queue)==1&&obj.Counts.commits==0);
event.input_send.messages_send_returned=6;eval(body);
expected=gpenmpcNative.RflyLocalCommittedDecoder(record.message,record.original_host_receive_ns);
assert(isempty(queue)&&ingestCalls==1&&obj.Counts.commits==double(expected.joint_installs) ...
    &&isequal(event.committed{1}.raw,record));
queue={record};obj.ComponentInitialization=false;before=takeCalls;eval(body);
assert(takeCalls==before&&numel(queue)==1);obj.ComponentInitialization=true;
bad=record;bad.message(99)=bitxor(bad.message(99),uint8(1));queue={bad};id='';
try,eval(body);catch ex,id=ex.identifier;end
assert(strcmp(id,'gpenmpcNative:LocalCommittedDigest')&&ingestCalls==1);
queue={record};rejectPhase=true;id='';
try,eval(body);catch ex,id=ex.identifier;end
assert(strcmp(id,'fixture:PhaseRejected')&&obj.Counts.commits==double(expected.joint_installs));
report=struct('passed',true,'checks',6,'scope','AFFECTED_PRODUCTION_COMPONENT_HISTORY_ORDER_REAL_RLC2', ...
    'no_input_and_partial_send_leave_original_queue_untouched',true, ...
    'complete_send_then_real_decoder_and_phase',true, ...
    'bad_digest_and_phase_rejection_retained',true,'hardware_actions',0,'live_commits',0);
    function v=take(channel,varargin) %#ok<INUSD>
        assert(strcmp(channel,'committed_state'));takeCalls=takeCalls+1;v=[];
        if ~isempty(queue),v=queue{1};queue(1)=[];end
    end
    function ingest(v)
        assert(isequal(v,record));
        assert(~rejectPhase,'fixture:PhaseRejected','Phase rejection remains fatal.');
        ingestCalls=ingestCalls+1;
    end
end
