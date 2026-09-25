function info=start_gpenmpc_usb_manual(operation,durationS)
% Start, inspect or recover the USB manual-control session.
if nargin==0,operation="START";end
if nargin<2,durationS=0;end % Zero runs until an explicit stop.
assert(isscalar(durationS)&&ismember(durationS,[0 20 120]));
operation=string(operation);assert(isscalar(operation)&&ismember(operation,["START","CHECK_ONLY","CHECK_INPUT","RESTORE_ORIGINAL","RESUME_RESET"]));
toolsRoot=fileparts(mfilename('fullpath'));build=fileparts(toolsRoot);
logBase=gpenmpc_log_root();
calibration=fullfile(toolsRoot,'usb_rc_runtime','fs_i6s_calibration.mat');
assert(isfile(calibration),'gpenmpcManual:Paths','RC calibration is unavailable.');
if operation~="CHECK_ONLY"&&~isfolder(logBase),mkdir(logBase);end
addpath(toolsRoot,fullfile(build,'host_runtime'),fullfile(build,'m600_coptersim','matlab_validation'));
assert(strcmpi(m600check.fileSha256(calibration), ...
 '622F6A22DFE2E6B9E224F93D50DF7EE2B97AED53F0EA1D068B99672F44F682CA'), ...
 'gpenmpcManual:Calibration','Selected physical-axis calibration changed.');
assert(strcmpi(which('execute_m600_board_local_short'),fullfile(toolsRoot,'execute_m600_board_local_short.m')));
info=struct('operation',operation,'entry',which('execute_m600_board_local_short'), ...
 'rc_duration_s',durationS,'calibration',calibration,'log_base',logBase, ...
 'component_only',true,'direction_live_validation_pending',true,'save_full_raw',false);
if operation=="CHECK_ONLY",disp(info);return;end % Report configuration.
if operation=="CHECK_INPUT"
 % Read the USB input once.
 addpath(fullfile(toolsRoot,'usb_rc_runtime'),'-begin');
 inputCleanup=onCleanup(@()gpenmpc_usb_joystick_mex('close')); %#ok<NASGU>
 rflyRoot=getenv('GPENMPC_RFLY_ROOT');
 assert(~isempty(rflyRoot)&&isfolder(rflyRoot),'gpenmpcManual:Installation', ...
  'Set GPENMPC_RFLY_ROOT to the RflySim/PX4PSP installation directory.');
 sdlPath=fullfile(rflyRoot,'QGroundControl','SDL2.dll');
 assert(isfile(sdlPath),'gpenmpcManual:SDL2','SDL2 library was not found: %s',sdlPath);
 usb=gpenmpc_usb_joystick_mex('open',sdlPath);
 saved=load(calibration,'calibration');
 rc=gpenmpcNative.normalizeUsbRcInput(usb,saved.calibration,usb.host_read_qpc_s,.1);
 assert(rc.valid,'gpenmpcShort:RcInput','USB input is unavailable: %s',rc.reason);
 info.input_ready=true;return;
end

% Hold an exclusive file handle to prevent concurrent session launches.
lockPath=fullfile(toolsRoot,'usb_rc_runtime','manual_session.lock');
try
 launchLock=System.IO.FileStream(lockPath,System.IO.FileMode.OpenOrCreate, ...
     System.IO.FileAccess.ReadWrite,System.IO.FileShare.None);
catch ex
 error('gpenmpcManual:Busy','The session lock is unavailable. Check the active session before starting.\n%s',ex.message);
end
lockCleanup=onCleanup(@()launchLock.Dispose());
entries=dir(fullfile(logBase,'manual_session_*'));last=0;
for k=1:numel(entries)
 token=regexp(entries(k).name,'^manual_session_([0-9]{3})$','tokens','once');
 if entries(k).isdir&&~isempty(token),last=max(last,str2double(token{1}));end
end
assert(last<999,'gpenmpcManual:RunNames','The session numbering range is exhausted.');
if operation=="RESUME_RESET"
 % Resume parameter recovery for the same session under the launch lock.
 runRoot=fullfile(logBase,sprintf('manual_session_%03d',last));
 info.run_root=runRoot;
 diary(fullfile(runRoot,'MANUAL_RESET_RESUME.log'));diaryCleanup=onCleanup(@()diary('off')); %#ok<NASGU>
 info.result=resumeManualReset(runRoot);
 assignin('base','gpenmpc_manual_result',info.result);
 return
end
runRoot=fullfile(logBase,sprintf('manual_session_%03d',last+1));
assert(~isfolder(runRoot),'gpenmpcManual:FreshOutput','Run directory already exists.');
[ok,msg]=mkdir(runRoot);assert(ok,'%s',msg);
diary(fullfile(runRoot,'MATLAB_RUN.log'));diaryCleanup=onCleanup(@()diary('off')); %#ok<NASGU>
if operation=="RESTORE_ORIGINAL"
 info.run_root=runRoot;info.result=restoreOriginal(build,runRoot);
 assignin('base','gpenmpc_manual_result',info.result);
 if ~info.result.safe,assignin('base','gpenmpc_manual_launch_lock',lockCleanup);end
 return
end
fprintf('\nPreparing the manual session...\n');
if durationS==0
 fprintf('End flight: Ctrl+Shift+L.\n');
else
 fprintf('Session duration: %g s.\n',durationS);
end
result=[];liveInvoked=false;
try
 fprintf('Checking transmitter connection...\n');
 start_gpenmpc_usb_manual('CHECK_INPUT');
 fprintf('Transmitter connected. Preparing simulation...\n');
 % Prepare the 3D view before starting hardware setup.
 info.visualization=ensure_gpenmpc_rfly_view();
 % Prepare and run the session in one invocation.
 liveInvoked=true;
 result=execute_m600_board_local_short(runRoot,"PREPARE_AND_LIVE",calibration,durationS,false,true);
 % Keep any returned native safety owner in this interactive MATLAB session.
 assignin('base','gpenmpc_manual_result',result);
 info.result=result;info.run_root=runRoot;
 if isequal(result.safe,true)
  fprintf('\nSession complete; safe standby confirmed. Summary: %s\n',runRoot);
  fprintf('Select Start manual session on the dashboard to begin again.\n');
 else
  resetStopPath=fullfile(runRoot,'MANUAL_RESET_DRIVER_STOP.json');
  if isfield(result,'short')&&isstruct(result.short)&&isfield(result.short,'manual_reset_requested') ...
    &&result.short.manual_reset_requested&&isfield(result,'manual_application_reset') ...
    &&result.manual_application_reset.passed&&result.manual_application_reset.COM_closed ...
    &&isfile(resetStopPath)
   stopped=jsondecode(fileread(resetStopPath));
   if stopped.COM_closed&&System.Diagnostics.Process.GetProcessesByName('CopterSimNoUI').Length==0 ...
      &&System.Diagnostics.Process.GetProcessesByName('CopterSim').Length==0
    % Flight owner is gone; allow the same panel to launch recovery only.
    % safe remains false, so Start stays disabled until actual restoration.
    result.retry_reset_available=true;info.result=result;
    assignin('base','gpenmpc_manual_result',result);
    fprintf(2,'Reset connection interrupted. Select Reset simulation to reconnect and continue recovery.\n');
    return
   end
  end
  assignin('base','gpenmpc_manual_launch_lock',lockCleanup);
  fprintf(2,'\nRecovery is unconfirmed. Keep MATLAB and USB connected until recovery completes.\nLog: %s\n',runRoot);
 end
catch ex
 info.run_root=runRoot;info.error=ex;
 preparationOnly=~liveInvoked||strcmp(ex.identifier,'gpenmpcShort:Preparation');
 if preparationOnly
  % Release the launch lock when preparation fails before a board transaction.
  result=struct('safe',true,'safety_basis','NO_BOARD_TRANSACTION_STARTED','preparation_failed',true,'board_actions',0, ...
   'short_completed',false,'raw_storage_pending',false, ...
   'failure',struct('identifier',ex.identifier,'message',ex.message));
  if ~isempty(ex.cause),result.failure.cause_identifier=ex.cause{1}.identifier;end
  info.result=result;
  f=fopen(fullfile(runRoot,'OUTER_SHORT_RESULT.json'),'w');assert(f>=0);fc=onCleanup(@()fclose(f));
  fwrite(f,jsonencode(result,PrettyPrint=true),'char');clear fc
  if strcmp(ex.identifier,'gpenmpcRc:DeviceIdentity')&&contains(ex.message,'found 0')
   fprintf(2,'Transmitter disconnected. Turn it on, connect USB, then select Start manual session.\n');
  else
   fprintf(2,'Preparation failed: %s\nResolve the issue before restarting.\n',ex.message);
  end
  return;
 end
 fprintf(2,'\nSession stopped: %s\n',ex.message);
 % Preserve the interactive workspace for recovery after an error.
 assignin('base','gpenmpc_manual_launch_error',ex);
 assignin('base','gpenmpc_manual_launch_lock',lockCleanup);
 fprintf(2,'Keep this window open and check the error and recovery status before restarting.\n');
end
end

function result=resumeManualReset(runRoot)
path=fullfile(runRoot,'OUTER_SHORT_RESULT.json');
result=jsondecode(fileread(path));
assert(~result.safe&&result.manual_retain_requested&&result.short.manual_reset_requested ...
 &&isfile(fullfile(runRoot,'MANUAL_RESET_REQUEST')),'gpenmpcManual:ResetScope');
assert(any(strcmpi(result.installed_application_sha256, ...
 {'39C752E75D032B833802D204758BB3CDA2E2BFED225F7E018C92DC08B45035CD','D3AE53EED37783C32EC7DBC6F225D11AACED85E137D4FBC629FF07ADFE2029F5','022EE000838DBCBF7180CC8079677FD01829BFC271244365EE30F549A756C1DC','F09C2EA76D9F42BA44B640C269BE10A30AFF5F2251FCC12EC1D4F1593FD43BBE','4C1C5F47D6B50C7233D60DDA37F67C9F9887C59D2A3CE615D4731571A686E3B4','178DE0A5DC16D161DF5B6E4D64CAE7DA0E094EA7E295BB5E239F9CD50D0D84D0', ...
 'AF571397E67013D23D59EF2827B879B76A408271713891686790BD64661B4743'})),'gpenmpcManual:ResetApplication');
expectedBuild='Sep 14 2026 10:33:50';
if strcmpi(result.installed_application_sha256,'178DE0A5DC16D161DF5B6E4D64CAE7DA0E094EA7E295BB5E239F9CD50D0D84D0')
 expectedBuild='Sep 24 2026 01:46:50';
elseif strcmpi(result.installed_application_sha256,'AF571397E67013D23D59EF2827B879B76A408271713891686790BD64661B4743')
 expectedBuild='Sep 25 2026 17:26:15';
end
fprintf('USB_RC_RESETTING: Reconnecting to complete reset...\n');
result.manual_reset_driver_stop=reset_m600_manual_application(runRoot,expectedBuild,true);
[contracts,~]=load_m600_recovery_contracts();
parameterPath=fullfile(runRoot,'PARAMETER_RESTORE_AFTER_LINK_RETURN.json');
routePath=fullfile(runRoot,'ROUTE_RESTORE_AFTER_LINK_RETURN.json');
finalPath=fullfile(runRoot,'FINAL_SAFETY_AFTER_LINK_RETURN.json');
% Preserve recovery records and restore values from this session's APPLY receipt.
for name={parameterPath,routePath,finalPath}
 for suffix={'','.pending','.previous'}
  old=[name{1} suffix{1}];
  if isfile(old),movefile(old,[old '.' char(datetime('now','Format','yyyyMMdd_HHmmss_SSS')) '.previous']);end
 end
end
result.parameter_restore=m600_local_short_parameter_setup(parameterPath,'RESTORE','', ...
 contracts.temporary_allocator_geometry,contracts.native_hover_tuning,fullfile(runRoot,'TEMPORARY_PARAMETER_SETUP.json'));
assert(result.parameter_restore.passed&&result.parameter_restore.COM_closed,'gpenmpcManual:ResetParameters');
result.recovery_route_restore=m600_local_recovery_route(routePath,'RESTORE',fullfile(runRoot,'NATIVE_RECOVERY_ROUTE_SETUP.json'));
assert(result.recovery_route_restore.passed&&result.recovery_route_restore.COM_closed,'gpenmpcManual:ResetRoute');
[~,transaction]=fileparts(runRoot);
result.postflight=m600_local_application_handoff(finalPath,'SAFE_KEEP_HIL_APPLICATION',transaction,false,false,expectedBuild);
assert(result.postflight.passed&&result.postflight.COM_closed&&result.postflight.disarmed ...
 &&result.postflight.landed&&result.postflight.physical_path_disabled ...
 &&strcmpi(result.postflight.commander_recovery_route.raw_bits_hex,'00000000'),'gpenmpcManual:ResetFinalState');
result.safe=true;result.manual_firmware_retained=true;result.manual_reset_completed=true;
result.raw_storage_pending=false;
copyfile(path,[path '.' char(datetime('now','Format','yyyyMMdd_HHmmss_SSS')) '.previous']);
pending=[path '.reset_pending'];f=fopen(pending,'w');assert(f>=0);c=onCleanup(@()fclose(f));
fwrite(f,jsonencode(result,PrettyPrint=true),'char');clear c
movefile(pending,path,'f');
reset_m600_standby_view(runRoot);
fprintf('USB_RC_RESET_DONE: Reset complete. Ready to start.\n');
end

function result=restoreOriginal(build,runRoot)
% Restore the baseline application through the guarded uploader.
[~,transaction]=fileparts(runRoot);
result=struct('maintenance_only',true,'safe',false,'short_completed',false, ...
 'manual_firmware_retained',false,'original_firmware_restored',false,'failures',{{}});
try
 for name={'CopterSim','CopterSimNoUI','QGroundControl'}
  assert(System.Diagnostics.Process.GetProcessesByName(name{1}).Length==0,'gpenmpcManual:MaintenanceBusy','End the current simulator/serial session before maintenance.');
 end
 fprintf('USB_RC_MAINTENANCE: Restoring the baseline application.\n');
 handoff=fullfile(runRoot,'RESTORE_REFERENCE_FLASH_HANDOFF.json');
 guard=m600_local_application_handoff(handoff,'REBOOT_RESTORE_REFERENCE',transaction);
 assert(guard.passed,'gpenmpcManual:RestoreGuard','Original restore guard failed.');
 rp=fullfile(runRoot,'RESTORE_REFERENCE_UPLOAD.json');jp=fullfile(runRoot,'RESTORE_REFERENCE_UPLOAD.jsonl');
 cmd=sprintf('"%s" -B "%s" --application restore_reference --handoff "%s" --transaction-id %s --journal "%s" --result "%s"', ...
  gpenmpc_install_path('rfly','Python38\python.exe'),fullfile(build,'tools','gpenmpc_application_upload_once.py'),handoff,transaction,jp,rp);
 [code,log]=system(cmd);fprintf('%s',log);
 assert(isfile(rp),'gpenmpcManual:RestoreReceipt','Uploader did not return its result.');
 result.restore_upload=jsondecode(fileread(rp));result.restore_upload.process_return_code=code;
 assert(result.restore_upload.application_crc_stage_passed,'gpenmpcManual:RestoreCrc','Original firmware CRC was not verified.');
 result.original_firmware_restored=true;
 % Existing handoff waits for fresh PnP enumeration and checks same board.
 result.postflight=m600_local_application_handoff(fullfile(runRoot,'FINAL_READONLY_SAFETY.json'),'SAFE_STOP_PX4IO',transaction);
 result.safe=result.postflight.passed&&result.postflight.COM_closed;
catch ex
 result.failures={struct('identifier',ex.identifier,'message',ex.message)};
 fprintf(2,'Baseline application restoration failed: %s\n',ex.message);
end
f=fopen(fullfile(runRoot,'OUTER_SHORT_RESULT.json'),'w');assert(f>=0);c=onCleanup(@()fclose(f));
fwrite(f,jsonencode(result,PrettyPrint=true),'char');
end
