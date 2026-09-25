function result=m600_local_recovery_route(outputPath,operation,sourcePath)
% Apply and restore the native-LAND recovery parameter using a dedicated journal.
% APPLY records the fresh INT32 value before changing 0 to 4.
% RESTORE uses that same journal, including after a partial apply.
if nargin<3,sourcePath='';end
operation=char(string(operation));
assert(any(strcmp(operation,{'VALIDATE','APPLY','RESTORE'})), ...
    'gpenmpcRecoveryRoute:Operation','Select explicit APPLY, RESTORE or pure VALIDATE.');
if strcmp(operation,'VALIDATE')
    for value=[0 4]
        p=payload(value);
        assert(isa(p.param_value,'single')&&typecast(p.param_value,'int32')==value, ...
            'gpenmpcRecoveryRoute:Encoding','Typed INT32 PARAM_SET bits changed.');
    end
    result=struct('schema','GPENMPC_LOCAL_RECOVERY_ROUTE_VALIDATION_V1','passed',true, ...
        'parameter','COM_OBL_RC_ACT','mav_type',6,'bit_checks',2,'hardware_actions',0,'COM_open',0);
    return
end
outputPath=char(string(outputPath));sourcePath=char(string(sourcePath));
assert(~isempty(regexp(outputPath,'^[A-Za-z]:[\\/]','once'))&&isfolder(fileparts(outputPath)) ...
    &&~isfile(outputPath)&&~isfile([outputPath '.pending'])&&~isfile([outputPath '.previous']), ...
    'gpenmpcRecoveryRoute:Output','Use a fresh absolute receipt path in an existing directory.');
assert(isfile(sourcePath),'gpenmpcRecoveryRoute:Source','The original source receipt is missing.');
build=fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(build,'host_runtime'),fullfile(build,'m600_coptersim','matlab_validation'));
source=jsondecode(fileread(sourcePath));
sourceBinding=struct('path',sourcePath,'sha256',m600check.fileSha256(sourcePath));
if strcmp(operation,'APPLY')
    validateHandoff(source);
else
    assert(strcmp(source.schema,'GPENMPC_LOCAL_RECOVERY_ROUTE_V1')&&strcmp(source.operation,'APPLY') ...
        &&strcmp(source.uid,gpenmpc_device_identity('uid'))&&islogical(source.attempted)&&isscalar(source.attempted), ...
        'gpenmpcRecoveryRoute:ApplySource','RESTORE requires the exact APPLY journal from this operation family, including partial failure.');
    if source.attempted
        assert(isValue(source.original_current,0)&&isValue(source.before,0)&&isValue(source.target,4), ...
            'gpenmpcRecoveryRoute:Original','Attempted APPLY must retain its actual original INT32 zero and requested INT32 four.');
    end
end
noSimulator();
result=struct('schema','GPENMPC_LOCAL_RECOVERY_ROUTE_V1','operation',operation, ...
    'scope','EXPLICIT_ENGINEERING_NATIVE_LAND_RECOVERY_ROUTE__ADDITIONAL_TO_UNCHANGED_167_AND_21', ...
    'uid',gpenmpc_device_identity('uid'),'source',sourceBinding,'port','COM3','baud',921600, ...
    'original_current',[],'before',[],'target',[],'attempted',false,'send_returned',false, ...
    'echo',[],'independent_readback',[],'parameter_writes',0,'passed',false,'COM_closed',true, ...
    'identity',[],'guard',[],'physical_parameters',[],'driver_status',[], ...
    'disposition','NOT_ATTEMPTED','arm_mode_requests',0,'reboot_requests',0,'driver_stop_requests',0, ...
    'other_parameter_writes',0,'physical_output_actions',0,'failure','','cleanup_failure','', ...
    'journal_sequence',0,'started_utc',utc(),'updated_utc','','link_counters',[]);
if strcmp(operation,'RESTORE'),result.original_current=source.original_current;end
reserve=System.IO.FileStream(outputPath,System.IO.FileMode.CreateNew,System.IO.FileAccess.Write,System.IO.FileShare.None);
reserve.Dispose();link=[];finished=false;guard=onCleanup(@finish);
try
    persist();if strcmp(operation,'APPLY'),freshHandoff(source);end
    noSimulator();link=gpenmpcNative.MavlinkSerialLink;
    h=link.open('COM3',921600,15);result.COM_closed=false;
    assert(link.TargetSystem==1&&link.TargetComponent==1&&own(h), ...
        'gpenmpcRecoveryRoute:Endpoint','The live serial target must be system/component 1/1.');
    result.identity=link.collectIdentity(10);checkIdentity(result.identity);
    readGuards();
    current=readParam('COM_OBL_RC_ACT');result.before=current;
    if strcmp(operation,'APPLY')
        result.original_current=current;result.target=typed(4);persist();
        assert(isValue(current,0),'gpenmpcRecoveryRoute:OriginalCurrent', ...
            'APPLY requires current INT32 0; observed type %g bits %s.',current.mav_type,current.raw_bits_hex);
        desired=4;needsWrite=true;
    else
        if source.attempted,result.target=source.original_current;else,result.target=typed(0);end
        persist();
        assert(isValue(current,0)||(source.attempted&&isValue(current,4)), ...
            'gpenmpcRecoveryRoute:UnknownThirdValue', ...
            'RESTORE requires the recorded original or attempted target; observed type %g bits %s.',current.mav_type,current.raw_bits_hex);
        desired=0;needsWrite=~isValue(current,0);
    end
    result.guard=freshState();safeWatch=tic;
    if needsWrite
        noSimulator();
        result.attempted=true;result.disposition='ATTEMPT_JOURNALED_BEFORE_SEND';persist();
        assert(toc(safeWatch)<3,'gpenmpcRecoveryRoute:ExpiredGuard', ...
            'Fresh exact-board disarmed/landed observation expired before PARAM_SET; no SET sent.');
        link.sendMessage('PARAM_SET',payload(desired)); % Single attempt.
        result.parameter_writes=1;result.send_returned=true;result.disposition='SEND_RETURNED';persist();
        m=link.waitForMessage('PARAM_VALUE',3,@(x)own(x)&&strcmp(gpenmpcNative.MavlinkSerialLink.cleanText(x.Payload.param_id),'COM_OBL_RC_ACT'));
        result.echo=typedMessage(m);persist();
        assert(isValue(result.echo,desired),'gpenmpcRecoveryRoute:Echo', ...
            'Original PARAM_SET echo differs from requested typed value; uncertain mutation remains journaled.');
    else
        result.disposition='ALREADY_ORIGINAL_NO_WRITE';
    end
    result.independent_readback=readParam('COM_OBL_RC_ACT');persist();
    assert(isValue(result.independent_readback,desired),'gpenmpcRecoveryRoute:Readback', ...
        'Independent PARAM_REQUEST_READ did not prove requested original raw bits.');
    result.final_guard=freshState();
    result.passed=true;
    if needsWrite,result.disposition='ECHO_AND_INDEPENDENT_READBACK_EXACT';end
catch ex,result.failure=getReport(ex,'extended','hyperlinks','off');end
finish();clear guard
disp(jsonencode(struct('operation',operation,'passed',result.passed,'COM_closed',result.COM_closed, ...
    'attempted',result.attempted,'parameter_writes',result.parameter_writes,'disposition',result.disposition)));

    function r=readParam(name)
        link.drain();
        link.sendMessage('PARAM_REQUEST_READ',struct('param_index',int16(-1), ...
            'target_system',uint8(1),'target_component',uint8(1),'param_id',name));
        m=link.waitForMessage('PARAM_VALUE',3,@(x)own(x)&&strcmp(gpenmpcNative.MavlinkSerialLink.cleanText(x.Payload.param_id),name));
        r=typedMessage(m);
    end
    function readGuards()
        rows=struct('name',{},'mav_type',{},'raw_bits_hex',{});
        names=[{'RA_CTRL_MODE','SYS_HITL'},cellstr(compose('PWM_MAIN_FUNC%d',1:8)),cellstr(compose('PWM_AUX_FUNC%d',1:8))];
        expected=[0 1 zeros(1,16)];
        for j=1:numel(names)
            r=readParam(names{j});rows(end+1)=r;result.physical_parameters=rows; %#ok<AGROW>
            assert(r.mav_type==6&&strcmpi(r.raw_bits_hex,dec2hex(uint32(expected(j)),8)), ...
                'gpenmpcRecoveryRoute:PhysicalGuard','%s expected INT32 %g, observed type %g bits %s.',names{j},expected(j),r.mav_type,r.raw_bits_hex);
        end
        drivers=struct('name',{},'original_text',{});
        for name={'pwm_out','px4io','dshot'}
            original=char(link.shellCommand(string(name{1})+" status",3));
            drivers(end+1)=struct('name',name{1},'original_text',original);result.driver_status=drivers; %#ok<AGROW>
            assert(~isempty(regexp(original,'not running|command not found','once')), ...
                'gpenmpcRecoveryRoute:Driver','Driver %s is running or its stopped state is unknown.',name{1});
        end
    end
    function g=freshState()
        link.drain();link.requestMessage(148);link.requestMessage(245);watch=tic;
        v=link.waitForMessage('AUTOPILOT_VERSION',3,@own);
        e=link.waitForMessage('EXTENDED_SYS_STATE',3,@own);
        h=link.waitForMessage('HEARTBEAT',3,@own);
        assert(toc(watch)<3&&validVersion(v.Payload) ...
            &&isequal(v.Payload.flight_custom_version,result.identity.autopilot_version.flight_custom_version) ...
            &&isequal(v.Payload.flight_sw_version,result.identity.autopilot_version.flight_sw_version) ...
            &&bitand(uint8(h.Payload.base_mode),uint8(128))==0&&double(e.Payload.landed_state)==1, ...
            'gpenmpcRecoveryRoute:FreshSafety','Fresh same UID/GUID, disarmed and on-ground state was not proved.');
        g=struct('observed_utc',utc(),'identity',v,'heartbeat',h,'extended_state',e,'observation_window_s',toc(watch));
    end
    function persist()
        result.journal_sequence=result.journal_sequence+1;result.updated_utc=utc();
        bytes=unicode2native(jsonencode(result,PrettyPrint=true),'UTF-8');tmp=[outputPath '.pending'];
        f=System.IO.FileStream(tmp,System.IO.FileMode.CreateNew,System.IO.FileAccess.Write,System.IO.FileShare.None);
        cleanup=onCleanup(@()f.Dispose());f.Write(uint8(bytes),int32(0),int32(numel(bytes)));f.Flush(true);f.Dispose();clear cleanup
        System.IO.File.Replace(tmp,outputPath,[outputPath '.previous'],true);
    end
    function finish()
        if finished,return,end
        finished=true;
        if ~isempty(link)
            try,link.close();result.COM_closed=~link.isOpen();result.link_counters=link.counters();
            catch ex,result.COM_closed=false;result.cleanup_failure=getReport(ex,'extended','hyperlinks','off');end
        end
        result.passed=result.passed&&result.COM_closed;
        try,persist();catch ex
            result.passed=false;result.cleanup_failure=[result.cleanup_failure newline getReport(ex,'extended','hyperlinks','off')];
            warning('gpenmpcRecoveryRoute:Journal','Final persistence failed; last atomic attempt receipt remains at %s.',outputPath);
        end
    end
end

function p=payload(value)
assert(isscalar(value)&&any(value==[0 4]),'gpenmpcRecoveryRoute:Value','Only exact INT32 zero/four is allowed.');
p=struct('target_system',uint8(1),'target_component',uint8(1),'param_id','COM_OBL_RC_ACT', ...
    'param_type',uint8(6),'param_value',typecast(int32(value),'single'));
end
function r=typed(value)
r=struct('name','COM_OBL_RC_ACT','mav_type',6,'raw_bits_hex',dec2hex(typecast(int32(value),'uint32'),8));
end
function yes=isValue(r,value)
expected=typed(value);
yes=isstruct(r)&&isscalar(r)&&all(isfield(r,{'name','mav_type','raw_bits_hex'})) ...
    &&strcmp(r.name,'COM_OBL_RC_ACT')&&r.mav_type==6&&strcmpi(r.raw_bits_hex,expected.raw_bits_hex);
end
function r=typedMessage(m)
r=struct('name',char(gpenmpcNative.MavlinkSerialLink.cleanText(m.Payload.param_id)), ...
    'mav_type',double(m.Payload.param_type),'raw_bits_hex',dec2hex(typecast(single(m.Payload.param_value),'uint32'),8));
end
function yes=own(m),yes=double(m.SystemID)==1&&double(m.ComponentID)==1;end
function yes=validVersion(v)
yes=strcmp(sprintf('%u',uint64(v.uid)),gpenmpc_device_identity('uid'))&&double(v.board_version)==56 ...
    &&double(v.product_id)==56&&strcmp(upper(reshape(dec2hex(uint8(v.uid2),2).',1,[])),gpenmpc_device_identity('px4_guid'));
end
function checkIdentity(i)
assert(validVersion(i.autopilot_version)&&strcmp(i.parsed_hw_arch,'PX4_FMU_V6C') ...
    &&strcmpi(i.parsed_commit,'6ea3539157ca358c70a515878b77077af7d4611d'), ...
    'gpenmpcRecoveryRoute:Identity','Current exact UID/GUID, board/product, hardware architecture or PX4 commit differs.');
end
function noSimulator()
assert(System.Diagnostics.Process.GetProcessesByName('CopterSim').Length==0 ...
    &&System.Diagnostics.Process.GetProcessesByName('CopterSimNoUI').Length==0, ...
    'gpenmpcRecoveryRoute:Simulator','Stop CopterSim/NoUI before using this helper.');
end
function validateHandoff(h)
assert(strcmp(h.schema,'GPENMPC_APPLICATION_FLASH_HANDOFF_V1')&&any(strcmp(h.operation,{'READONLY','SAFE_STOP_PX4IO','SAFE_STOP_PX4IO_START_BOARD_ADC'})) ...
    &&h.passed&&h.COM_closed&&strcmp(h.uid,gpenmpc_device_identity('uid'))&&h.disarmed&&h.landed ...
    &&h.usb_only&&h.all_modules_stopped&&h.pwm_out_stopped&&h.physical_path_disabled&&h.virtual_path_disabled ...
    &&h.parameter_writes==0&&h.arm_mode_requests==0&&h.physical_output_actions==0 ...
    &&h.controlled_application_reboot_to_bootloader_count==0&&h.bootloader_firmware_writes==0 ...
    &&h.application_flash_attempts==0&&~h.usb_reenumeration_pending&&~h.reboot_ack_accepted ...
    &&isscalar(h.driver_stop_attempts)&&any(h.driver_stop_attempts==[0 1]) ...
    &&h.driver_stop_verified==h.driver_stop_attempts&&(~strcmp(h.operation,'READONLY')||h.driver_stop_attempts==0), ...
    'gpenmpcRecoveryRoute:Handoff','Fresh original postboot read-only/verified px4io-stop handoff and all zero-action safety guards are required.');
if strcmp(h.operation,'SAFE_STOP_PX4IO_START_BOARD_ADC')
    assert(all(isfield(h,{'board_adc_start_attempts','board_adc_start_verified','board_adc_power_witness'})) ...
        &&(h.board_adc_start_attempts==1||(h.board_adc_start_attempts==0 ...
          &&isfield(h,'board_adc_reused')&&isequal(h.board_adc_reused,true)))&&h.board_adc_start_verified==1 ...
        &&isstruct(h.board_adc_power_witness)&&isscalar(h.board_adc_power_witness) ...
        &&h.board_adc_power_witness.passed, ...
        'gpenmpcRecoveryRoute:PowerObserver','Dedicated HITL handoff lacks verified board_adc -n USB-only system_power evidence.');
end
freshHandoff(h);
end
function freshHandoff(h)
fmt="yyyy-MM-dd'T'HH:mm:ss.SSS'Z'";now=datetime('now','TimeZone','UTC');
created=datetime(h.handoff_created_utc,'InputFormat',fmt,'TimeZone','UTC');
expiry=datetime(h.expires_utc,'InputFormat',fmt,'TimeZone','UTC');
assert(now>=created&&now<=expiry&&seconds(expiry-created)<=60.001, ...
    'gpenmpcRecoveryRoute:HandoffAge','Original postboot handoff expired before opening COM.');
end
function s=utc(),s=char(datetime('now','TimeZone','UTC','Format',"yyyy-MM-dd'T'HH:mm:ss.SSS'Z'"));end
