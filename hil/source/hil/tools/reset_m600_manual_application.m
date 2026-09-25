function result=reset_m600_manual_application(runRoot,expectedBuild,afterReboot)
% Reset the virtual session after its simulator and IO owners release resources.
if nargin<3,afterReboot=false;end
assert(islogical(afterReboot)&&isscalar(afterReboot));
prepared=jsondecode(fileread(fullfile(runRoot,'SHORT_ENTRY_PREPARED.json')));
assert(prepared.operator_reference&&isfile(fullfile(runRoot,'MANUAL_RESET_REQUEST')), ...
 'gpenmpcReset:Scope','An explicit reset of this manual HIL session is required.');
outputName='MANUAL_APPLICATION_RESET.json';
if afterReboot
 prior=jsondecode(fileread(fullfile(runRoot,outputName)));
 assert(prior.passed&&prior.COM_closed&&prior.application_reboots==1, ...
  'gpenmpcReset:RebootNotConfirmed');
 outputName='MANUAL_RESET_DRIVER_STOP.json';
end
for name={'CopterSim','CopterSimNoUI'}
 assert(System.Diagnostics.Process.GetProcessesByName(name{1}).Length==0, ...
  'gpenmpcReset:SimulatorStillRunning');
end
link=gpenmpcNative.MavlinkSerialLink;cleanup=onCleanup(@()link.close()); %#ok<NASGU>
result=struct('passed',false,'force_disarm_requests',0,'application_reboots',0, ...
 'firmware_uploads',0,'physical_output_writes',0,'driver_stop_requests',0,'COM_closed',false);
try
 % After reboot, a visible COM name may belong to a disappearing USB instance.
 % Retry only the read/identity connection; do not repeat state-changing commands.
 connectionClock=tic;result.connection_attempts=0;
 while true
  result.connection_attempts=result.connection_attempts+1;
  try
   link.open('COM3',921600,15);identity=link.collectIdentity(10);break
  catch connectionError
   link.close();
   transient=startsWith(connectionError.identifier,'transportlib:') ...
    ||ismember(string(connectionError.identifier),["gpenmpcNative:MavlinkSerialPnP","gpenmpcNative:MavlinkSerialTimeout"]);
   if ~afterReboot||~transient||toc(connectionClock)>=25,rethrow(connectionError);end
   fprintf('USB_RC_RESETTING: Reconnecting the flight controller...\n');
   pause(.25);link=gpenmpcNative.MavlinkSerialLink;
  end
 end
 v=identity.autopilot_version;
 stamp=regexp(char(identity.ver_all),'Build datetime:\s*([^\r\n]+)','tokens','once');
 guid=upper(reshape(dec2hex(uint8(v.uid2),2).',1,[]));
 assert(strcmp(sprintf('%u',uint64(v.uid)),gpenmpc_device_identity('uid'))&&double(v.board_version)==56 ...
  &&strcmp(guid,gpenmpc_device_identity('px4_guid')) ...
  &&strcmp(identity.parsed_commit,'6ea3539157ca358c70a515878b77077af7d4611d') ...
  &&strcmp(identity.parsed_hw_arch,'PX4_FMU_V6C')&&~isempty(stamp) ...
  &&strcmp(strtrim(stamp{1}),expectedBuild),'gpenmpcReset:BoardIdentity');
 result.identity=identity;
 hitl=link.requestParam('SYS_HITL',3);
 assert(hitl.mav_type==6&&strcmpi(hitl.raw_bits_hex,'00000001'),'gpenmpcReset:NotHil');
 for name=[compose("PWM_MAIN_FUNC%d",1:8),compose("PWM_AUX_FUNC%d",1:8)]
  value=link.requestParam(name,3);
  assert(value.mav_type==6&&strcmpi(value.raw_bits_hex,'00000000'),'gpenmpcReset:PhysicalMapping');
 end
 for name=["pwm_out","dshot"]
  status=char(link.shellCommand(name+" status",3));
  assert(~isempty(regexp(status,'not running|command not found','once')),'gpenmpcReset:PhysicalDriver');
 end
 status=char(link.shellCommand('px4io status',3));
 if isempty(regexp(status,'not running|command not found','once'))
  fields=regexp(status,'(?m)^\s*pwm:\s*\[([^\]]+)\]','tokens','once');
  assert(~isempty(fields),'gpenmpcReset:PhysicalPwmUnknown');
  values=sscanf(strrep(fields{1},',',' '),'%f');
  assert(numel(values)==8&&all(values==0)&&contains(status,'arming_lockdown: True'), ...
   'gpenmpcReset:PhysicalPwmNotZero');
 end
 link.drain();hb=link.waitForMessage('HEARTBEAT',3,[]);
 assert(bitand(uint8(hb.Payload.base_mode),uint8(32))~=0,'gpenmpcReset:HilHeartbeat');
 if afterReboot
  % A reboot restarts PX4IO even with physical functions disabled.
  % Stop it before the parameter-restore driver checks.
  link.requestMessage(245);ext=link.waitForMessage('EXTENDED_SYS_STATE',3,[]);
  assert(bitand(uint8(hb.Payload.base_mode),uint8(128))==0 ...
   &&double(ext.Payload.landed_state)==1,'gpenmpcReset:PostRebootState');
  result.disarmed_heartbeat=hb;result.extended_state=ext;
  if isempty(regexp(status,'not running|command not found','once'))
   assert(~isempty(regexp(status,'arming_fmu_armed:\s*False','once')), ...
    'gpenmpcReset:PhysicalDriverArmed');
   result.driver_stop_requests=1;
   result.driver_stop_response=char(link.shellCommand('px4io stop',3));
  end
  result.driver_status=char(link.shellCommand('px4io status',3));
  assert(~isempty(regexp(result.driver_status,'not running','once')), ...
   'gpenmpcReset:DriverStopUnconfirmed');
  result.passed=true;
 else
 if bitand(uint8(hb.Payload.base_mode),uint8(128))~=0
  % Explicit HIL Reset may stop a failed virtual plant after output-isolation checks.
  result.force_disarm_requests=1;link.requestArm(false,true);
  ack=link.waitForMessage('COMMAND_ACK',3,@(m)double(m.Payload.command)==400);
  assert(double(ack.Payload.result)==0,'gpenmpcReset:DisarmRejected');
  link.drain();hb=link.waitForMessage('HEARTBEAT',3,[]);
 end
 assert(bitand(uint8(hb.Payload.base_mode),uint8(128))==0,'gpenmpcReset:DisarmUnconfirmed');
 result.disarmed_heartbeat=hb;
 result.application_reboots=1;link.commandLong(246,[1 0 0 0 0 0 0]);
 ack=link.waitForMessage('COMMAND_ACK',3,@(m)double(m.Payload.command)==246);
 assert(double(ack.Payload.result)==0,'gpenmpcReset:RebootRejected');
 result.reboot_ack=ack;result.passed=true;
 end
catch ex
 result.failure=struct('identifier',ex.identifier,'message',ex.message); 
end
link.close();result.COM_closed=~link.isOpen();
if isfile(fullfile(runRoot,outputName))
 copyfile(fullfile(runRoot,outputName),fullfile(runRoot,[outputName '.' char(datetime('now','Format','yyyyMMdd_HHmmss_SSS')) '.previous']));
end
f=fopen(fullfile(runRoot,outputName),'w');assert(f>=0);
fileCleanup=onCleanup(@()fclose(f)); %#ok<NASGU>
fwrite(f,jsonencode(result,PrettyPrint=true),'char');
assert(result.passed&&result.COM_closed,'gpenmpcReset:Incomplete','Simulation reset incomplete. Check the flight-controller connection.');
end
