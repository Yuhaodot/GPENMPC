function r=inspect_m600_sim_calibration(outputDir)
% Read simulator calibration and safety parameters.
assert(~isfile(fullfile(outputDir,'RAW_SIM_CALIBRATION.mat'))&&~isfile(fullfile(outputDir,'SIM_CALIBRATION.json')), ...
 'm600check:ExistingDiagnostic','Completed diagnostic evidence must not be overwritten.');
if ~isfolder(outputDir),mkdir(outputDir);end
root=fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(root,'host_runtime'),fullfile(root,'m600_coptersim','matlab_validation'));
p=System.Diagnostics.Process.GetProcessesByName('CopterSim');
assert(p.Length==0,'m600check:ExistingCopterOwner','Do not open COM while a CopterSim owner exists.');
link=gpenmpcNative.MavlinkSerialLink;cleanup=onCleanup(@()link.close());
r=struct('status','READ_ONLY_SIM_CALIBRATION_DIAGNOSTIC','passed',false,'failure','', ...
 'COM_open_attempts',0,'COM_closed',false,'parameter_writes',0,'mapping_writes',0, ...
 'arm_disarm_mode_requests',0,'reboot_flash_bootloader',0,'CopterSim_start_count',0,'rows',[]);
rows=struct('name',{},'value',{},'error',{},'read_utc',{});
try
 r.COM_open_attempts=1;link.open("COM3",921600,15);r.identity=link.collectIdentity(10);
 v=r.identity.autopilot_version;
 assert(strcmp(sprintf('%u',uint64(v.uid)),gpenmpc_device_identity('uid'))&&double(v.board_version)==56&& ...
  strcmpi(r.identity.parsed_commit,'6ea3539157ca358c70a515878b77077af7d4611d'),'m600check:IdentityMismatch','Exact board identity mismatch.');
 safeState();
 names=["CAL_GYRO2_ID","CAL_GYRO2_XOFF","CAL_GYRO2_YOFF","CAL_GYRO2_ZOFF","CAL_GYRO2_PRIO","CAL_GYRO2_ROT", ...
 "CAL_MAG1_ID","CAL_MAG1_XOFF","CAL_MAG1_YOFF","CAL_MAG1_ZOFF", ...
 "CAL_MAG1_XSCALE","CAL_MAG1_YSCALE","CAL_MAG1_ZSCALE", ...
 "CAL_MAG1_XODIAG","CAL_MAG1_YODIAG","CAL_MAG1_ZODIAG", ...
 "CAL_MAG1_XCOMP","CAL_MAG1_YCOMP","CAL_MAG1_ZCOMP","CAL_MAG1_ROT","CAL_MAG1_PRIO", ...
 "CAL_MAG1_ROLL","CAL_MAG1_PITCH","CAL_MAG1_YAW","CAL_MAG_COMP_TYP", ...
 "IMU_GYRO_CAL_EN","SENS_MAG_AUTOCAL","TC_G_ENABLE",compose("TC_G%d_ID",0:3), ...
 "SENS_BOARD_ROT","SENS_BOARD_X_OFF","SENS_BOARD_Y_OFF","SENS_BOARD_Z_OFF"];
 for k=1:numel(names)
  value=[];err='';try,link.drain();value=link.requestParam(names(k),3);catch e,err=[e.identifier ': ' e.message];end
  rows(end+1)=struct('name',char(names(k)),'value',value,'error',err, ...
   'read_utc',char(datetime('now','TimeZone','UTC','Format',"yyyy-MM-dd'T'HH:mm:ss.SSS'Z'")));
 end
 guards=[compose("HIL_ACT_FUNC%d",1:16),compose("PWM_MAIN_FUNC%d",1:8),"RA_CTRL_MODE"];
 values=cell(1,numel(guards));
 for k=1:numel(guards)
  link.drain();values{k}=link.requestParam(guards(k),3);
  assert(values{k}.mav_type==6&&values{k}.decoded==0,'m600check:OutputIsolationMismatch','Output guard must be typed INT32 zero.');
 end
 r.final_typed_zero=vertcat(values{:});
 r.pwm_out=char(link.shellCommand("pwm_out status",3));
 assert(contains(r.pwm_out,'not running'),'m600check:PhysicalOutputNotVerified','Physical PWM producer must be stopped.');
 safeState();r.passed=true;
catch e,r.failure=[e.identifier ': ' e.message];end
link.close();r.COM_closed=~link.isOpen();r.link_counters=link.counters();clear cleanup
r.rows=rows;save(fullfile(outputDir,'RAW_SIM_CALIBRATION.mat'),'r');
fid=fopen(fullfile(outputDir,'SIM_CALIBRATION.json'),'w','n','UTF-8');assert(fid>=0);g=onCleanup(@()fclose(fid));
fprintf(fid,'%s\n',jsonencode(r,PrettyPrint=true));clear g
disp(struct('passed',r.passed,'rows',numel(rows),'COM_closed',r.COM_closed,'write_count',0));
 function safeState()
  link.drain();link.requestMessage(245);ext=link.waitForMessage("EXTENDED_SYS_STATE",3,[]);
  link.drain();hb=link.waitForMessage("HEARTBEAT",3,[]);
  r.armed=bitand(uint8(hb.Payload.base_mode),uint8(128))~=0;r.landed_state=double(ext.Payload.landed_state);
  assert(~r.armed&&r.landed_state==1,'m600check:NotDisarmedOnGround','Board must be disarmed and on ground.');
 end
end
