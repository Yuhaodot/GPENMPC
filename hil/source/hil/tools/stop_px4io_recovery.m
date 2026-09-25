function result=stop_px4io_recovery(root,afterUserReplug)
% Stop the driver during recovery after a pre-arm abort and USB reconnect.
build=string(fileparts(fileparts(mfilename('fullpath'))));
if nargin<1,root=fullfile(gpenmpc_external_path('serial_recovery'));end
if nargin<2,afterUserReplug=true;end
assert(islogical(afterUserReplug)&&isscalar(afterUserReplug));
root=string(root);
assert(gpenmpc_is_session_directory(root,build)&&isfolder(root));
old=jsondecode(fileread(fullfile(root,'OUTER_SHORT_RESULT.json')));
if afterUserReplug
    output=fullfile(root,'PX4IO_STOP_AFTER_USER_REPLUG.json');
    evidence=fullfile(root,'DIRECT_SERIAL_AFTER_USER_REPLUG.mat');
else
    output=fullfile(root,'PX4IO_STOP_AFTER_PREARM_TRANSPORT_FAILURE.json');
    if old.short.counts.arm_requests>0
        output=fullfile(root,'PX4IO_STOP_AFTER_FRESH_NATIVE_STATE.json');
    end
    evidence=fullfile(root,'DIRECT_SERIAL_AFTER_OWNED_BRIDGE_RELEASE.mat');
end
if isfile(output)
    prior=jsondecode(fileread(output));
    assert(~prior.passed&&prior.COM_closed&&prior.driver_stop_attempts==0 ...
        &&prior.parameter_writes==0&&contains(prior.failure,'New or unresolved SD crash evidence'));
    % Require the completed reversible archive and fresh SD, state and PWM checks
    % before the driver-stop recovery action.
    output=replace(output,'.json','_AFTER_EXACT_ARCHIVE.json');
end
assert(~isfile(output),'gpenmpcRecovery:Existing','Do not repeat this recovery action.');
addpath(fullfile(build,'host_runtime'),fullfile(build,'m600_coptersim','matlab_validation'));
fresh=load(evidence,'result');
postReplug=isfield(fresh.result,'after_user_replug')&&isequal(fresh.result.after_user_replug,true);
assert(postReplug==afterUserReplug);
recoveryRequired=(isfield(old,'external_action_required')&&old.external_action_required) ...
    ||(~old.safe&&old.short.safe_ground&&old.short.counts.arm_requests==0&&old.short.counts.offboard_requests==0);
% Use a fresh disarmed and landed observation after exclusive owner release.
assert(recoveryRequired ...
    &&fresh.result.safe_observation&&fresh.result.disarmed&&fresh.result.landed&&fresh.result.COM_closed ...
    &&System.Diagnostics.Process.GetProcessesByName('CopterSimNoUI').Length==0);
result=struct('schema','GPENMPC_RECOVERY_PX4IO_STOP_V1','passed',false,'COM_closed',true, ...
    'driver_stop_attempts',0,'driver_stop_verified',0,'parameter_writes',0, ...
    'arm_mode_requests',0,'reboot_flash',0,'physical_output_actions',0,'failure','');
link=gpenmpcNative.MavlinkSerialLink;guard=onCleanup(@finish);
try
    hb=link.open('COM3',921600,15);result.COM_closed=false;
    result.identity=link.collectIdentity(10);v=result.identity.autopilot_version;
    assert(link.TargetSystem==1&&link.TargetComponent==1&&uint64(v.uid)==gpenmpc_device_identity('uid_uint64') ...
        &&double(v.board_version)==56&&strcmpi(result.identity.parsed_commit,'6ea3539157ca358c70a515878b77077af7d4611d'));
    result.parameters=cell(1,32);
    names=[compose("HIL_ACT_FUNC%d",1:16),compose("PWM_MAIN_FUNC%d",1:8),compose("PWM_AUX_FUNC%d",1:8)];
    expected=[101:106 zeros(1,26)];
    for k=1:32
        q=link.requestParam(names(k),3);result.parameters{k}=q;
        assert(q.mav_type==6&&strcmpi(q.raw_bits_hex,dec2hex(uint32(expected(k)),8)), ...
            'gpenmpcRecovery:Mapping','Unexpected recovery mapping %s.',names(k));
    end
    result.driver_status=struct();
    for name=["pwm_out","dshot"]
        t=char(link.shellCommand(name+" status",3));result.driver_status.(char(name))=t;
        assert(~isempty(regexp(t,'not running|command not found','once')),'gpenmpcRecovery:Driver','%s is running.',name);
    end
    before=char(link.shellCommand('px4io status',3));result.driver_status.px4io_before=before;
    if ~isempty(regexp(before,'not running','once'))
        result.driver_stop_verified=1;result.passed=true;finish();clear guard;return
    end
    pwm=regexp(before,'(?m)^\s*pwm:\s*\[([^\]]+)\]','tokens','once');
    assert(~isempty(pwm),'gpenmpcRecovery:ActualPwm','PX4IO did not report actual output.');
    actual=sscanf(strrep(pwm{1},',',' '),'%f');result.actual_pwm_before=actual;
    assert(numel(actual)==8&&all(actual==0)&&~isempty(regexp(before,'arming_fmu_armed:\s*False','once')) ...
        &&~isempty(regexp(before,'arming_lockdown:\s*True','once')),'gpenmpcRecovery:ActualPwm','PX4IO actual output guard failed.');
    link.drain();link.requestMessage(245);ext=link.waitForMessage('EXTENDED_SYS_STATE',3,[]);
    link.drain();last=link.waitForMessage('HEARTBEAT',3,[]);
    assert(double(ext.Payload.landed_state)==1&&bitand(uint8(last.Payload.base_mode),uint8(128))==0, ...
        'gpenmpcRecovery:State','Fresh disarmed/landed guard failed.');
    result.sd_listing=char(link.shellCommand('ls /fs/microsd',3));
    result.sd_diagnostics=m600check.inspectSdDiagnosticListing(result.sd_listing);
    assert(result.sd_diagnostics.passed,'gpenmpcRecovery:SdFault','New or unresolved SD crash evidence; preserve before maintenance.');
    result.driver_stop_attempts=1;writeResult();
    result.stop_response=char(link.shellCommand('px4io stop',3));
    after=char(link.shellCommand('px4io status',3));result.driver_status.px4io_after=after;
    assert(~isempty(regexp(after,'not running','once')),'gpenmpcRecovery:Stop','PX4IO did not stop.');
    result.driver_stop_verified=1;result.passed=true;
catch ex
    result.failure=getReport(ex,'extended','hyperlinks','off');
end
finish();clear guard
disp(result)
    function writeResult()
        f=fopen(output,'w','n','UTF-8');assert(f>=0);c=onCleanup(@()fclose(f));
        fprintf(f,'%s\n',jsonencode(result,PrettyPrint=true));clear c
    end
    function finish()
        link.close();result.COM_closed=~link.isOpen();result.link_counters=link.counters();
        result.passed=result.passed&&result.COM_closed;writeResult();
    end
end
