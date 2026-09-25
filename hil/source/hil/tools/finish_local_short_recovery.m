function result=finish_local_short_recovery(root,afterUserReplug,afterObservedFailedPlant,afterPrearmShellTimeout)
% Finish recovery of the pre-arm transaction after the simulator exits safely.
root=string(root);build=string(fileparts(fileparts(mfilename('fullpath'))));
addpath(fullfile(build,'tools'),fullfile(build,'host_runtime'),fullfile(build,'matlab_validation'),fullfile(build,'m600_coptersim','matlab_validation'));
if nargin<2,afterUserReplug=false;end
if nargin<3,afterObservedFailedPlant=false;end
if nargin<4,afterPrearmShellTimeout=false;end
assert(islogical(afterPrearmShellTimeout)&&isscalar(afterPrearmShellTimeout));
if afterPrearmShellTimeout
    assert(~afterUserReplug&&~afterObservedFailedPlant);
    result=recoverPrearmShellAndFinish(root,build);return
end
assert(islogical(afterUserReplug)&&isscalar(afterUserReplug));
assert(islogical(afterObservedFailedPlant)&&isscalar(afterObservedFailedPlant));
old=jsondecode(fileread(fullfile(root,'OUTER_SHORT_RESULT.json')));
assert(old.external_action_required);
if afterObservedFailedPlant
    observed=load(fullfile(root,'FAILED_MODEL_SAFE_SHUTDOWN_OBSERVATION.mat'),'result');
    assert(observed.result.after_observed_failed_plant&&observed.result.owned_NoUI_stopped);
    suffix='FAILED_MODEL_DISARMED';
elseif afterUserReplug
    observed=load(fullfile(root,'DIRECT_SERIAL_AFTER_USER_REPLUG.mat'),'result');
    assert(observed.result.after_user_replug&&observed.result.disarmed&&observed.result.landed&&observed.result.COM_closed);
    suffix='USER_REPLUG';
elseif isfile(fullfile(root,'DIRECT_SERIAL_AFTER_OWNED_BRIDGE_RELEASE.mat'))
    observed=load(fullfile(root,'DIRECT_SERIAL_AFTER_OWNED_BRIDGE_RELEASE.mat'),'result');
    q=observed.result;
    assert(q.read_only&&q.safe_observation&&q.COM_closed&&~q.after_user_replug ...
        &&q.parameter_writes==0&&q.arm_mode_requests==0&&q.reboot_flash==0 ...
        &&q.disarmed&&q.landed ...
        &&strcmp(sprintf('%u',uint64(q.identity.autopilot_version.uid)),gpenmpc_device_identity('uid')) ...
        &&q.identity.autopilot_version.board_version==56 ...
        &&bitand(uint8(q.last_heartbeat.Payload.base_mode),uint8(128))==0 ...
        &&q.extended_state.Payload.landed_state==1);
    % Confirm current native safety and recheck identity and output state for each action.
    suffix='FRESH_NATIVE_STATE';
else
    observed=load(fullfile(root,'POST_FAILURE_UDP_OBSERVATION_EXISTING_ASSEMBLER.mat'),'result','hb','ext','ground','version');
    later=fullfile(root,'POST_NATIVE_STANDARD_DISARM_RECOVERY.mat');
    if isfile(later)
        candidate=load(later,'result','hb','ext','ground','version');
        assert(candidate.result.safe_observation&&candidate.result.owned_NoUI_stopped ...
            &&candidate.result.standard_disarm_requests<=1&&candidate.result.force_disarm_requests==0);
        observed=candidate;
    end
    if old.short.counts.arm_requests==0&&old.short.counts.offboard_requests==0
        suffix='PREARM_TRANSPORT_FAILURE';
        if ~observed.result.safe_observation
            % A failed simulator transport can release COM while its process remains alive.
            % Use an exclusive serial observation to check release.
            directPath=fullfile(root,'DIRECT_SERIAL_RECOVERY_OBSERVATION.mat');
            releasedPath=fullfile(root,'DIRECT_SERIAL_AFTER_OWNED_BRIDGE_RELEASE.mat');
            if isfile(releasedPath),directPath=releasedPath;end
            info=dir(directPath);
            assert(numel(info)==1);
            % This observation establishes owner release; each recovery action obtains
            % fresh UID, state and output measurements before writing.
            direct=load(directPath,'result');q=direct.result;
            assert(q.read_only&&q.safe_observation&&q.COM_closed&&~q.after_user_replug ...
                &&q.parameter_writes==0&&q.arm_mode_requests==0&&q.reboot_flash==0 ...
                &&q.disarmed&&q.landed ...
                &&strcmp(sprintf('%u',uint64(q.identity.autopilot_version.uid)),gpenmpc_device_identity('uid')) ...
                &&q.identity.autopilot_version.board_version==56 ...
                &&bitand(uint8(q.last_heartbeat.Payload.base_mode),uint8(128))==0 ...
                &&q.extended_state.Payload.landed_state==1);
            observed=direct;
        end
    else
        % A later observer can confirm native ground/disarm and simulator release
        % when the initial bounded recovery was inconclusive. Preserve both observations.
        assert(observed.result.safe_observation ...
            &&~observed.result.after_user_replug&&~observed.result.after_observed_failed_plant ...
            &&observed.version.uid==gpenmpc_device_identity('uid_uint64') ...
            &&bitand(uint8(observed.hb.base_mode),uint8(128))==0 ...
            &&observed.ext.landed_state==1&&observed.ground.ground_confirmed);
        suffix='LATER_NATIVE_GROUND_OBSERVATION';
    end
end
function result=recoverPrearmShellAndFinish(root,build)
% Recover a pre-arm blocked NSH command through one application reboot.
% Require successful identity, state and mapping reads.
old=jsondecode(fileread(fullfile(root,'OUTER_SHORT_RESULT.json')));
pre=jsondecode(fileread(fullfile(root,'NEW_APPLICATION_READONLY.json')));
uploadPath=gpenmpc_external_path('recovery_application_upload_receipt');
upload=jsondecode(fileread(uploadPath));
assert(~old.safe&&old.short.safe_ground&&upload.application_crc_stage_passed ...
    &&strcmp(old.short.failure.identifier,'gpenmpcShort:EstimatorCommandTimeout') ...
    &&old.short.counts.arm_requests==0&&old.short.counts.offboard_requests==0&&old.short.counts.commits==0);
assert(pre.passed&&pre.COM_closed&&pre.physical_path_disabled&&pre.pwm_out_stopped);
assert(System.Diagnostics.Process.GetProcessesByName('CopterSimNoUI').Length==0);
path=fullfile(root,'PREARM_SHELL_APPLICATION_RECOVERY.mat');
if isfile(path)
    prior=load(path,'result');
    assert(prior.result.reboot_attempts==0&&prior.result.COM_closed ...
        &&contains(prior.result.failure,'Current native identity differs'));
    % Retain the JSON UINT64 conversion failure from the no-write attempt.
    path=fullfile(root,'PREARM_SHELL_APPLICATION_RECOVERY_NATIVE_UID.mat');
end
assert(~isfile(path));
result=struct('passed',false,'reboot_attempts',0,'parameter_writes',0,'arm_requests',0, ...
    'application_flash_attempts',0,'bootloader_writes',0,'physical_output_actions',0,'COM_closed',true,'failure','');
link=gpenmpcNative.MavlinkSerialLink;cleanup=onCleanup(@()link.close());
try
    result.first_heartbeat=link.open('COM3',921600,10);result.COM_closed=false;
    link.drain();link.requestMessage(148);result.version=link.waitForMessage('AUTOPILOT_VERSION',3,[]);
    v=result.version.Payload;expected=pre.raw_identity.autopilot_version;
    assert(link.TargetSystem==1&&link.TargetComponent==1&&isa(v.uid,'uint64') ...
        &&strcmp(sprintf('%u',v.uid),pre.uid)&&strcmp(pre.uid,gpenmpc_device_identity('uid')) ...
        &&v.board_version==56&&v.product_id==56 ...
        &&isequal(uint8(v.uid2(:)),uint8(expected.uid2(:))) ...
        &&isequal(uint8(v.flight_custom_version(:)),uint8(expected.flight_custom_version(:))), ...
        'gpenmpcRecovery:NativeIdentity','Current native identity differs from the verified upload/preflight.');
    names=[compose("HIL_ACT_FUNC%d",1:16),compose("PWM_MAIN_FUNC%d",1:8),compose("PWM_AUX_FUNC%d",1:8)];
    expectedValues=[101:106 zeros(1,26)];result.mappings=cell(1,numel(names));
    for k=1:numel(names)
        q=link.requestParam(names(k),3);result.mappings{k}=q;
        assert(q.mav_type==6&&strcmpi(q.raw_bits_hex,dec2hex(uint32(expectedValues(k)),8)), ...
            'gpenmpcRecovery:NativeMapping','Unexpected current mapping %s; no reboot.',names(k));
    end
    result.estimator_parameters=cell(1,4);names=["SENS_IMU_MODE","EKF2_MULTI_IMU","SENS_MAG_MODE","EKF2_MULTI_MAG"];
    for k=1:numel(names),result.estimator_parameters{k}=link.requestParam(names(k),3);end
    link.drain();link.requestMessage(245);result.ext=link.waitForMessage('EXTENDED_SYS_STATE',3,[]);
    link.drain();result.heartbeat=link.waitForMessage('HEARTBEAT',3,[]);
    assert(result.ext.Payload.landed_state==1&&bitand(uint8(result.heartbeat.Payload.base_mode),uint8(128))==0, ...
        'gpenmpcRecovery:NativeState','Fresh disarmed/on-ground is mandatory; no reboot if unknown.');
    result.reboot_attempts=1;save(path,'result');
    link.commandLong(246,[1 0 0 0 0 0 0]);
    result.ack=link.waitForMessage('COMMAND_ACK',3,@(m)double(m.Payload.command)==246);
    assert(result.ack.Payload.result==0,'gpenmpcRecovery:NativeRebootAck');
    link.close();result.COM_closed=true;save(path,'result');pause(8);
    % A new boot restores the ordinary shell. All original shell/SD/PWM and
    % semantic guards are mandatory again before any restore write/flash.
    observed=observe_serial_recovery(true,false,root);
    assert(observed.safe_observation&&observed.COM_closed);
    result.driver=stop_px4io_recovery(root,false);assert(result.driver.passed);
    [c,~]=load_m600_recovery_contracts();
    result.parameters=m600_local_short_parameter_setup(fullfile(root,'PARAMETER_RESTORE_AFTER_SHELL_RECOVERY.json'), ...
        'RESTORE','',c.temporary_allocator_geometry,c.native_hover_tuning,fullfile(root,'TEMPORARY_PARAMETER_SETUP.json'));
    assert(result.parameters.passed);
    result.route=m600_local_recovery_route(fullfile(root,'NATIVE_ROUTE_RESTORE_AFTER_SHELL_RECOVERY.json'), ...
        'RESTORE',fullfile(root,'NATIVE_RECOVERY_ROUTE_SETUP.json'));assert(result.route.passed);
    [~,transaction]=fileparts(root);transaction=char(transaction);
    hp=fullfile(root,'RESTORE_REFERENCE_HANDOFF_AFTER_SHELL_RECOVERY.json');
    result.handoff=m600_local_application_handoff(hp,'REBOOT_RESTORE_REFERENCE',transaction);assert(result.handoff.passed);
    cmd=sprintf('"%s" -B "%s" --application restore_reference --handoff "%s" --transaction-id "%s" --journal "%s" --result "%s"', ...
        gpenmpc_install_path('rfly','Python38\python.exe'),fullfile(build,'tools','gpenmpc_application_upload_once.py'),hp,transaction, ...
        fullfile(root,'RESTORE_REFERENCE_UPLOAD.jsonl'),fullfile(root,'RESTORE_REFERENCE_UPLOAD.json'));
    result.application_flash_attempts=1;save(path,'result');
    [code,log]=system(cmd);disp(log);assert(code==0,'gpenmpcRecovery:RestoreUpload');pause(8);
    result.final=m600_local_application_handoff(fullfile(root,'FINAL_SAFETY_AFTER_SHELL_RECOVERY.json'),'SAFE_STOP_PX4IO',transaction);
    assert(result.final.passed&&result.final.COM_closed&&strcmp(result.final.commander_recovery_route.raw_bits_hex,'00000000'));
    result.passed=true;
catch ex
    result.failure=getReport(ex,'extended','hyperlinks','off');
end
link.close();result.COM_closed=~link.isOpen();save(path,'result');clear cleanup
disp(struct('safe',result.passed,'COM_closed',result.COM_closed,'failure',result.failure));
end
assert(observed.result.safe_observation);
assert(System.Diagnostics.Process.GetProcessesByName('CopterSimNoUI').Length==0);
[~,transaction,extension]=fileparts(root);transaction=char(transaction);
assert(isempty(extension)&&~isempty(regexp(transaction,'^manual_session_[0-9]{3}$','once')));
assert(~isfile(fullfile(root,"FINAL_SAFETY_AFTER_"+suffix+".json")));
[c,~]=load_m600_recovery_contracts();
parameterOutput=fullfile(root,"PARAMETER_RESTORE_AFTER_"+suffix+".json");
if afterUserReplug||strcmp(suffix,'FRESH_NATIVE_STATE')||(strcmp(suffix,'PREARM_TRANSPORT_FAILURE')&&isfile(parameterOutput))
    % Physical reboot restarts px4io. Stop only that driver using the
    % existing fresh disarmed/landed, zero physical PWM and exact virtual
    % mapping guards before restoring the temporary parameters.
    if afterUserReplug
        stopPath=fullfile(root,'PX4IO_STOP_AFTER_USER_REPLUG.json');
    elseif strcmp(suffix,'FRESH_NATIVE_STATE')
        stopPath=fullfile(root,'PX4IO_STOP_AFTER_FRESH_NATIVE_STATE.json');
    else
        stopPath=fullfile(root,'PX4IO_STOP_AFTER_PREARM_TRANSPORT_FAILURE.json');
        failed=jsondecode(fileread(parameterOutput));
        assert(~failed.passed&&failed.COM_closed&&failed.write_attempts==0 ...
            &&failed.parameter_writes==0&&contains(failed.failure,'Physical driver status is unknown or running: px4io.'));
    end
    if isfile(stopPath)
        result.driver=jsondecode(fileread(stopPath));
        if ~result.driver.passed
            assert(result.driver.COM_closed&&result.driver.driver_stop_attempts==0 ...
                &&contains(result.driver.failure,'New or unresolved SD crash evidence'));
            result.driver=stop_px4io_recovery(root,afterUserReplug);
        end
    else
        result.driver=stop_px4io_recovery(root,afterUserReplug);
    end
    assert(result.driver.passed&&result.driver.COM_closed&&result.driver.driver_stop_verified==1);
    if isfile(parameterOutput)
        failed=jsondecode(fileread(parameterOutput));
        assert(~failed.passed&&failed.COM_closed&&failed.write_attempts==0 ...
            &&failed.parameter_writes==0&&contains(failed.failure,'Physical driver status is unknown or running: px4io.'));
        % Preserve the pre-stop failure before recovery from the verified driver-state change.
        parameterOutput=fullfile(root,"PARAMETER_RESTORE_AFTER_"+suffix+"_PX4IO_STOP.json");
    end
end
result.parameters=m600_local_short_parameter_setup(parameterOutput, ...
    'RESTORE','',c.temporary_allocator_geometry,c.native_hover_tuning,fullfile(root,'TEMPORARY_PARAMETER_SETUP.json'));
assert(result.parameters.passed,'gpenmpcRecovery:Restore21','Exact temporary21 restore incomplete.');
result.route=m600_local_recovery_route(fullfile(root,'NATIVE_RECOVERY_ROUTE_RESTORE.json'),'RESTORE',fullfile(root,'NATIVE_RECOVERY_ROUTE_SETUP.json'));
assert(result.route.passed,'gpenmpcRecovery:RestoreRoute','Extra actual-original route restore incomplete.');
hp=fullfile(root,'RESTORE_REFERENCE_FLASH_HANDOFF.json');
result.handoff=m600_local_application_handoff(hp,'REBOOT_RESTORE_REFERENCE',transaction);assert(result.handoff.passed);
cmd=sprintf('"%s" -B "%s" --application restore_reference --handoff "%s" --transaction-id "%s" --journal "%s" --result "%s"', ...
    gpenmpc_install_path('rfly','Python38\python.exe'),fullfile(build,'tools','gpenmpc_application_upload_once.py'),hp,transaction, ...
    fullfile(root,'RESTORE_REFERENCE_UPLOAD.jsonl'),fullfile(root,'RESTORE_REFERENCE_UPLOAD.json'));
[code,log]=system(cmd);disp(log);assert(code==0,'gpenmpcRecovery:RestoreUpload','Restore application did not return verified success.');
pause(8);
result.final=m600_local_application_handoff(fullfile(root,"FINAL_SAFETY_AFTER_"+suffix+".json"),'SAFE_STOP_PX4IO',transaction);
assert(result.final.passed&&result.final.COM_closed&&strcmp(result.final.commander_recovery_route.raw_bits_hex,'00000000'));
disp(struct('safe',result.final.passed,'COM_closed',result.final.COM_closed,'restored_application','REFERENCE_APPLICATION'));
end
