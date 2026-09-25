function result=m600_local_application_handoff(outputPath,operation,transactionId,startLoggerBeforeHil,stopLoggerBeforeHil,expectedBuildDatetime)
% Use one MATLAB serial owner; READONLY is the default operation.
% REBOOT sends an application-to-bootloader request without flashing.
% The HIL startup profile selects one IMU and restores all three for recovery.
% SAFE_STOP_PX4IO operations require disarmed, grounded and zero-mapping checks.
% SAFE_STOP_PX4IO_START_BOARD_ADC then starts the stock FMUv6C system_power publisher.
if nargin<2,operation='READONLY';end
if nargin<3,transactionId='manual_session_001';end
if nargin<4,startLoggerBeforeHil=false;end
if nargin<5,stopLoggerBeforeHil=false;end
if nargin<6,expectedBuildDatetime='';end
restartRetainedHil=strcmp(operation,'RESTART_RETAINED_HIL_APPLICATION');
if restartRetainedHil
    assert(~isempty(expectedBuildDatetime),'gpenmpc:RetainedRestartIdentity');
    operation='SAFE_STOP_PX4IO_START_BOARD_ADC';
end
keepHilApplication=strcmp(operation,'SAFE_KEEP_HIL_APPLICATION');
% Use the read/stop guard while retaining the application.
if keepHilApplication,operation='SAFE_STOP_PX4IO';end
assert(islogical(stopLoggerBeforeHil)&&isscalar(stopLoggerBeforeHil)&& ...
 (~stopLoggerBeforeHil||(strcmp(operation,'SAFE_STOP_PX4IO_START_BOARD_ADC')&&~startLoggerBeforeHil)));
assert(islogical(startLoggerBeforeHil)&&isscalar(startLoggerBeforeHil)&& ...
 (~startLoggerBeforeHil||strcmp(operation,'SAFE_STOP_PX4IO_START_BOARD_ADC')));
assert(isscalar(string(operation))&&ismember(string(operation),["READONLY","SAFE_RESTORE_EKF_INSTANCE_ONLY","SAFE_STOP_PX4IO","SAFE_STOP_PX4IO_START_BOARD_ADC","REBOOT_GPENMPC","REBOOT_HIL_IMU","REBOOT_RESTORE_REFERENCE"]), ...
 'gpenmpc:HandoffOperation','Select one exact read-only, px4io safe-stop, or declared application reboot operation.');
rebootOperation=any(strcmp(operation,{'REBOOT_GPENMPC','REBOOT_RESTORE_REFERENCE'}));
nativeImuReboot=strcmp(operation,'REBOOT_HIL_IMU');
if nativeImuReboot
 uploadPath=fullfile(fileparts(outputPath),'GPENMPC_UPLOAD.json');
 expectedRun='fmuv6c';expectedSha='AF571397E67013D23D59EF2827B879B76A408271713891686790BD64661B4743';
 uploaded=jsondecode(fileread(uploadPath));
 assert(uploaded.application_crc_stage_passed&&strcmp(uploaded.application,expectedRun) ...
  &&strcmp(uploaded.transaction_id,transactionId)&&strcmpi(uploaded.image_sha256,expectedSha), ...
  'gpenmpc:StorageRepairImageRequired','Only after this transaction loaded the fixed SPI application.');
end
startBoardAdc=strcmp(operation,'SAFE_STOP_PX4IO_START_BOARD_ADC');
allowPx4ioStop=rebootOperation||nativeImuReboot||strcmp(operation,'SAFE_STOP_PX4IO')||startBoardAdc||strcmp(operation,'SAFE_RESTORE_EKF_INSTANCE_ONLY');
assert(~isfile(outputPath),'gpenmpc:ExistingHandoff','Choose an unused output path.');
build=fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(build,'host_runtime'),fullfile(build,'m600_coptersim','matlab_validation'));
parent=fullfile(getenv('HIL_RECOVERY_RECORDS'),'SERIAL_PREFLIGHT.json');
assert(~isempty(getenv('HIL_RECOVERY_RECORDS')),'gpenmpc:RecoveryRecords','Configure HIL_RECOVERY_RECORDS.');
assert(strcmpi(m600check.fileSha256(parent),'073E7CE165D8D158BC329CFE96AEC4F968A6847A2A0AB9D90C1C127D3CB65EBB'), ...
 'gpenmpc:ParentSnapshotHash','Reference application parameter snapshot hash differs.');
p=jsondecode(fileread(parent));target=struct('name',{},'mav_type',{},'raw_bits_hex',{});
for k=1:numel(p.parameters),take(p.parameters(k));end
for k=1:numel(p.native_hover_parameter_union.rows),take(p.native_hover_parameter_union.rows(k).observed);end
assert(numel(target)==167&&~any(strcmp({target.name},'_HASH_CHECK')), ...
 'gpenmpc:Original167Contract','Expected exactly 167 selected writable parameters without _HASH_CHECK.');
link=gpenmpcNative.MavlinkSerialLink;guard=onCleanup(@()link.close());
result=struct('schema','GPENMPC_APPLICATION_FLASH_HANDOFF_V1','transaction_id',char(transactionId), ...
 'operation',char(operation),'passed',false,'port','COM3','COM_closed',false,'parameter_writes',0, ...
 'arm_mode_requests',0,'controlled_application_reboot_to_bootloader_count',0,'bootloader_firmware_writes',0, ...
 'physical_output_actions',0,'application_flash_attempts',0,'selected_count',167,'full_catalog_proven',false, ...
 'usb_reenumeration_pending',false,'reboot_ack_accepted',false, ...
 'driver_stop_attempts',0,'driver_stop_verified',0,'board_adc_start_attempts',0, ...
 'board_adc_start_verified',0,'board_adc_power_witness',struct([]));
try
 % Read current Windows PnP state before opening COM and allow re-enumeration
 % to finish after reboot. Retry absence within the 25 s lifecycle budget;
 % reject ambiguous or mismatched devices immediately.
 pnpWait=tic;result.pnp_empty_observations={};
 while true
  result.pre_reboot_pnp=readFreshPnp();
  device=result.pre_reboot_pnp.parsed.devices;
  if ~isempty(device)||toc(pnpWait)>=25,break;end
  result.pnp_empty_observations{end+1}=result.pre_reboot_pnp;
  pause(min(.25,max(0,25-toc(pnpWait))));
 end
 assert(isstruct(device)&&isscalar(device)&&strcmp(device.status,'OK')&& ...
  ~isempty(regexp(device.friendly_name,'\(COM3\)$','once'))&&strlength(string(device.instance_id))>8, ...
  'gpenmpc:PreRebootPnp','Exactly one present/started COM3 PnP device is required before COM open.');
 result.pre_reboot_pnp_instance=char(device.instance_id);
 link.open('COM3',921600,15);identity=link.collectIdentity(10);v=identity.autopilot_version;
 result.uid=sprintf('%u',uint64(v.uid));result.board_id=double(v.board_version);
 result.px4_guid=upper(reshape(dec2hex(uint8(v.uid2),2).',1,[]));result.commit=char(identity.parsed_commit);
 result.raw_identity=identity;
 if ~isempty(expectedBuildDatetime)
  observedBuild=regexp(char(identity.ver_all),'Build datetime:\s*([^\r\n]+)','tokens','once');
  assert(~isempty(observedBuild)&&strcmp(strtrim(observedBuild{1}),char(expectedBuildDatetime)), ...
   'gpenmpc:InstalledBuildDiffers','Installed application build differs; no automatic reflash or parameter writes.');
  result.installed_build_datetime=observedBuild{1};
 end
 result.keep_hil_application=keepHilApplication;
 assert(strcmp(result.uid,gpenmpc_device_identity('uid'))&&result.board_id==56&&double(v.product_id)==56 ...
  &&strcmp(result.px4_guid,gpenmpc_device_identity('px4_guid')) ...
  &&strcmpi(result.commit,'6ea3539157ca358c70a515878b77077af7d4611d') ...
  &&strcmp(identity.parsed_hw_arch,'PX4_FMU_V6C'),'gpenmpc:ApplicationIdentity','Exact board/commit differs.');
 rows=target;changed={};
 for k=1:numel(target)
  r=link.requestParam(string(target(k).name),3);rows(k)=struct('name',char(r.name),'mav_type',r.mav_type,'raw_bits_hex',char(r.raw_bits_hex));
  if r.mav_type~=target(k).mav_type||~strcmpi(r.raw_bits_hex,target(k).raw_bits_hex),changed{end+1}=r.name;end %#ok<AGROW>
 end
 result.selected_parameters=rows;result.semantic_changed=changed;
 % Read the dedicated recovery-route parameter through the current handle.
 result.commander_recovery_route=link.requestParam('COM_OBL_RC_ACT',3);
 % A restore request may need the original values written later; neither
 % current nonzero output mapping nor uncertain safety is permitted here.
 if ~strcmp(operation,'REBOOT_RESTORE_REFERENCE'),assert(isempty(changed),'gpenmpc:CurrentParametersDiffer','Current reference parameter values differ.');end
 names=[compose("HIL_ACT_FUNC%d",1:16),compose("PWM_MAIN_FUNC%d",1:8)];
 for k=1:numel(names)
  n=char(names(k));r=rows(strcmp({rows.name},n));
  assert(isscalar(r)&&r.mav_type==6&&strcmp(r.raw_bits_hex,'00000000'), ...
   'gpenmpc:OutputMappingNotZero','Mapping %s is missing, duplicated, wrong-type, or nonzero.',n);
 end
 result.aux_parameters=cell(1,8);
 for k=1:8
  r=link.requestParam(sprintf('PWM_AUX_FUNC%d',k),3);result.aux_parameters{k}=r;
  assert(r.mav_type==6&&strcmp(r.raw_bits_hex,'00000000'), ...
   'gpenmpc:PhysicalAuxNotZero','PWM_AUX_FUNC%d is wrong-type or nonzero.',k);
 end
 result.virtual_path_disabled=true;result.physical_path_disabled=true;
 result.pwm_out_status=char(link.shellCommand('pwm_out status',3));
 assert(~isempty(regexp(result.pwm_out_status,'\[pwm_out\]\s+not running','once')), ...
  'gpenmpc:PhysicalPwmUnknown','pwm_out not-running status was not observed.');result.pwm_out_stopped=true;
 modules={'gpenmpc_se3_control','gpenmpc_tunnel_bridge','gpenmpc_trajectory_exec','gpenmpc_rfly_canonical','gpenmpc_rfly_canonical_local','dshot','px4io'};
 result.module_names=modules;
 result.module_status=cell(size(modules));
 result.recovery_only_nonexecuting_closing=false;
 for k=1:numel(modules)
  text=char(link.shellCommand(string(modules{k})+" status",3));result.module_status{k}=text;
  if strcmp(modules{k},'px4io')&&isempty(regexp(text,'not running|command not found','once'))&&allowPx4ioStop
   result.px4io_before_stop=text;
   pwm=regexp(text,'(?m)^\s*pwm:\s*\[([^\]]+)\]','tokens','once');
   assert(~isempty(pwm),'gpenmpc:Px4ioPwmUnknown','Running px4io did not report its actual PWM values.');
   actualPwm=sscanf(strrep(pwm{1},',',' '),'%f');
   assert(numel(actualPwm)==8&&all(actualPwm==0)&& ...
    ~isempty(regexp(text,'arming_fmu_armed:\s*False','once'))&& ...
    ~isempty(regexp(text,'arming_lockdown:\s*True','once')), ...
    'gpenmpc:Px4ioStopUnsafe','Stop requires eight actual PWM zeros, FMU disarmed and lockdown.');
   link.drain();link.requestMessage(148);freshIdentity=link.waitForMessage('AUTOPILOT_VERSION',3,[]);
   freshVersion=freshIdentity.Payload;
   assert(uint64(freshVersion.uid)==uint64(v.uid)&&double(freshVersion.board_version)==56&& ...
    double(freshVersion.product_id)==56&&isequal(uint8(freshVersion.uid2(:)),uint8(v.uid2(:)))&& ...
    isequal(uint8(freshVersion.flight_custom_version(:)),uint8(v.flight_custom_version(:))), ...
    'gpenmpc:Px4ioStopIdentity','Fresh same-owner identity differs before px4io stop.');
   stopNames=[compose("HIL_ACT_FUNC%d",1:16),compose("PWM_MAIN_FUNC%d",1:8),compose("PWM_AUX_FUNC%d",1:8)];
   stopRows=cell(1,numel(stopNames));
   for j=1:numel(stopNames)
    q=link.requestParam(stopNames(j),3);stopRows{j}=q;
    assert(q.mav_type==6&&strcmpi(q.raw_bits_hex,'00000000'), ...
     'gpenmpc:Px4ioStopMapping','Fresh mapping %s must remain typed zero before driver stop.',stopNames(j));
   end
   link.drain();link.requestMessage(245);stopExt=link.waitForMessage('EXTENDED_SYS_STATE',3,[]);
   link.drain();stopHb=link.waitForMessage('HEARTBEAT',3,[]);
   assert(double(stopExt.Payload.landed_state)==1&&bitand(uint8(stopHb.Payload.base_mode),uint8(128))==0, ...
    'gpenmpc:Px4ioStopState','Fresh landed/on-ground and disarmed witnesses are required before driver stop.');
   result.px4io_stop_guard=struct('utc',utc(),'identity',freshIdentity,'parameters',{stopRows}, ...
    'heartbeat',stopHb,'extended_state',stopExt,'actual_pwm',actualPwm);
   result.driver_stop_attempts=result.driver_stop_attempts+1;
   save(string(outputPath)+'.mat','result'); % Count the actual call attempt before sending it.
   result.px4io_stop_response=char(link.shellCommand('px4io stop',3));
   text=char(link.shellCommand('px4io status',3));result.px4io_after_stop=text;result.module_status{k}=text;
   assert(~isempty(regexp(text,'not running','once')), ...
    'gpenmpc:Px4ioStopNotVerified','px4io stop did not produce an explicit not-running status.');
   result.driver_stop_verified=result.driver_stop_verified+1;
  end
  closingRecovery=strcmp(operation,'REBOOT_RESTORE_REFERENCE')&&strcmp(modules{k},'gpenmpc_rfly_canonical_local') ...
   &&~isempty(regexp(text,'\[gpenmpc_rfly_canonical_local\]\s+task=running phase=3 reason=[1-5] route=[01] plant_evidence=[0-3](?:\s|$)','once'));
  if closingRecovery
   % Closing follows authority revocation in the selected module.
   % Require fresh same-handle disarmed, grounded and zero-output checks
   % before this recovery-only application reboot.
   result.recovery_only_nonexecuting_closing=true;
  end
  assert(closingRecovery||~isempty(regexp(text,'not running|task=stopped|command not found','once')), ...
   'gpenmpc:ModuleNotStopped','Module status not stopped/absent: %s',modules{k});
 end
 result.all_modules_stopped=~result.recovery_only_nonexecuting_closing;
 if startBoardAdc
  % SYS_HITL skips rc.board_sensors, which normally starts board_adc.
  % After confirming px4io stopped, start the stock publisher and require two
  % fresh system_power samples before releasing the serial owner.
  result.board_adc_before_start=char(link.shellCommand('board_adc status',3));
  result.board_adc_reused=false;
  if ~isempty(regexp(result.board_adc_before_start,'(?i)\[board_adc\]\s+not running(?:\s|$)','once'))
  result.board_adc_start_attempts=1;
  save(string(outputPath)+'.mat','result'); % Persist attempt before the side effect.
  % Use -n to publish system_power without adding adc_report to the HIL sensor graph.
  result.board_adc_start_response=char(link.shellCommand('board_adc start -n',3));
  else
   result.board_adc_reused=true;
  end % Fresh status/power witness below must still pass for a reused driver.
  result.board_adc_status=char(link.shellCommand('board_adc status',3));
  result.board_adc_system_power=char(link.shellCommand('listener system_power -n 2',3));
  result.board_adc_power_witness=m600check.evaluateBoardAdcPowerWitness( ...
   result.board_adc_status,result.board_adc_system_power);
  assert(result.board_adc_power_witness.passed,'gpenmpc:BoardAdcPowerWitness', ...
   'board_adc did not prove two fresh USB-only system_power samples: %s', ...
   strjoin(string(result.board_adc_power_witness.failures),','));
  result.board_adc_start_verified=1;
 end
 result.sd_diagnostics=m600check.inspectSdDiagnosticListing(link.shellCommand('ls /fs/microsd',3));
 assert(result.sd_diagnostics.passed,'gpenmpc:SdCrashBoundary','Current crash log or incomplete SD listing prevents handoff.');
 outputRows=struct('name',{},'payload',{});link.drain();link.requestMessage(93);link.requestMessage(375);watch=tic;
 while toc(watch)<2
  messages=link.drain();for k=1:numel(messages)
   name=gpenmpcNative.MavlinkSerialLink.messageName(messages{k});
   if any(name==["HIL_ACTUATOR_CONTROLS","ACTUATOR_OUTPUT_STATUS"]),outputRows(end+1)=struct('name',char(name),'payload',messages{k}.Payload);end %#ok<AGROW>
  end;pause(.002);
 end
 result.output_observation=m600check.evaluateVirtualOutputEvidence(outputRows);
 assert(~result.output_observation.any_nonzero_or_nonfinite,'gpenmpc:NonzeroOutput','Observed actuator output is nonzero, nonfinite, or unrecognized.');
 link.drain();link.requestMessage(245);ext=link.waitForMessage('EXTENDED_SYS_STATE',3,[]);
 link.drain();hb=link.waitForMessage('HEARTBEAT',3,[]);
 result.disarmed=bitand(uint8(hb.Payload.base_mode),uint8(128))==0;result.landed=double(ext.Payload.landed_state)==1;
 result.raw_last_heartbeat=hb;result.raw_extended_state=ext;
 assert(result.disarmed&&result.landed,'gpenmpc:NotDisarmedLanded','Fresh heartbeat and extended state did not confirm disarmed/on-ground.');
 if stopLoggerBeforeHil
  % Remove the stock file logger from the HIL scheduling path.
  % Host record collection stays enabled; an application reboot restores the logger lifecycle.
  result.logger_before=char(link.shellCommand('logger status',3));
  if contains(result.logger_before,'Running in mode:')
  result.logger_stop_attempts=1;save(string(outputPath)+'.mat','result');
  result.logger_stop_response=char(link.shellCommand('logger stop',5));
  else
   assert(~isempty(regexp(result.logger_before,'(?i)\[logger\]\s+not running','once')), ...
    'gpenmpc:LoggerUnavailable','Logger status is neither running nor explicitly stopped.');
  end
  result.logger_after=char(link.shellCommand('logger status',3));
  assert(~isempty(regexp(result.logger_after,'(?i)\[logger\]\s+not running','once')), ...
   'gpenmpc:LoggerNotStopped','The file logger must actually stop before this host-recorded short test.');
  result.logger_stopped_before_hil=true;
  result.logging_scope='COMPONENT_ONLY_FILE_LOGGER_EXCLUSION__HOST_RAW_RETAINED__RESTORED_BY_REFERENCE_APPLICATION_BOOT';
  link.drain();hb=link.waitForMessage('HEARTBEAT',3,[]);
  assert(bitand(uint8(hb.Payload.base_mode),uint8(128))==0,'gpenmpc:LoggerArmingInvariant','Stopping a recorder must not arm the board.');
 elseif startLoggerBeforeHil
  % Start the PX4 file logger before control to write formats and parameters.
  % The temporary logging override is cleared by the recovery reboot.
  result.logger_before=char(link.shellCommand('logger status',3));
  assert(contains(result.logger_before,'Running in mode:'),'gpenmpc:LoggerUnavailable','Existing stock logger must be running.');
  result.logger_on_attempts=1;save(string(outputPath)+'.mat','result');
  result.logger_on_response=char(link.shellCommand('logger on',3));
  result.logger_after=char(link.shellCommand('logger status',3));
  % The logger applies its override on a new vehicle_status, published at 2 Hz.
  % Poll status until file logging starts; do not repeat the logging command.
  loggerWait=tic;
  while ~(contains(result.logger_after,'Full File Logging Running:')&&contains(result.logger_after,'Log file:'))&&toc(loggerWait)<2
   pause(.1);
   result.logger_after=char(link.shellCommand('logger status',3));
  end
  assert(contains(result.logger_after,'Full File Logging Running:')&&contains(result.logger_after,'Log file:'), ...
   'gpenmpc:LoggerNotStarted','No HIL start until the existing file logger reports active.');
  result.logger_started_before_hil=true;
  link.drain();hb=link.waitForMessage('HEARTBEAT',3,[]);
  assert(bitand(uint8(hb.Payload.base_mode),uint8(128))==0,'gpenmpc:LoggerArmingInvariant','File logging must not arm the board.');
 end
 result.guard_observed_utc=utc();result.usb_only=true;
 result.isolation_provenance='USB-only setup with propulsion and actuator power removed; verify the physical wiring separately from zero mappings and stopped PWM readback.';
 result.same_transaction_fresh=true;result.post_reboot_usb_chain_observed=false;
 % For a storage-only repair, retain EKF=3 and avoid parameter writes or save
 % commands under the affected SPI driver. Apply the identity and zero-output guards.
 if nativeImuReboot
  setEkfInstanceCount(1);
 elseif strcmp(operation,'REBOOT_RESTORE_REFERENCE')||strcmp(operation,'SAFE_RESTORE_EKF_INSTANCE_ONLY')
  setEkfInstanceCount(3);
 else
  result.ekf_instance_count=link.requestParam('EKF2_MULTI_IMU',3);
  desired=3;if startBoardAdc||keepHilApplication,desired=1;end
   if strcmp(operation,'REBOOT_GPENMPC')&&result.ekf_instance_count.mav_type==6 ...
    &&strcmp(result.ekf_instance_count.raw_bits_hex,'00000001')
   % Update directly from the provisioned HIL image.
   % The baseline recovery path expects three IMUs; preserve the current value for this update.
   predecessorBuild=regexp(char(identity.ver_all),'Build datetime:\s*([^\r\n]+)','tokens','once');
   knownPredecessor={'Sep 14 2026 10:33:50','Sep 24 2026 01:46:50'};
   assert(~isempty(predecessorBuild)&&any(strcmp(strtrim(predecessorBuild{1}),knownPredecessor)), ...
    'gpenmpc:RetainedPredecessorIdentity','Single-IMU upgrade must start from the known retained HIL build.');
   desired=1;
  end
  assert(result.ekf_instance_count.mav_type==6&&strcmp(result.ekf_instance_count.raw_bits_hex,dec2hex(uint32(desired),8)), ...
   'gpenmpc:EkfInstanceReadback','Explicit current HIL/original EKF instance count differs.');
 end
 if nativeImuReboot||restartRetainedHil
  % Complete save and readback with the corrected driver before application reboot.
  result.controlled_native_application_reboot_count=1;save(string(outputPath)+'.mat','result');
  link.drain();link.commandLong(246,[1 0 0 0 0 0 0]);
  ack=link.waitForMessage('COMMAND_ACK',3,@(m)double(m.Payload.command)==246);
  result.reboot_ack=ack;result.reboot_ack_accepted=double(ack.Payload.result)==0;
  assert(result.reboot_ack_accepted,'gpenmpc:NativeRebootRejected');
 end
 if rebootOperation
  if strcmp(operation,'REBOOT_GPENMPC'),result.application='fmuv6c';result.expected_image_sha256='AF571397E67013D23D59EF2827B879B76A408271713891686790BD64661B4743';
  else,result.application='restore_reference';result.expected_image_sha256='7722616157AF96E3493D1827F01A0713D92373946C669B854B8F043552892FC7';end
  result.reboot_requested_utc=utc();result.controlled_application_reboot_to_bootloader_count=1;result.usb_reenumeration_pending=true;
  save(string(outputPath)+'.mat','result'); % Record attempt before side effect.
  link.drain();link.commandLong(246,[3 0 0 0 0 0 0]);
  ack=link.waitForMessage('COMMAND_ACK',3,@(m)double(m.Payload.command)==246);
  result.reboot_ack=ack;result.reboot_ack_accepted=double(ack.Payload.result)==0;
  assert(result.reboot_ack_accepted,'gpenmpc:RebootNotAccepted','Application-to-bootloader request ACK was not accepted.');
 end
 result.passed=true;
catch ex,result.first_failure=getReport(ex,'extended','hyperlinks','off');end
link.close();result.COM_closed=~link.isOpen();result.prior_com_owner_released=result.COM_closed;
result.link_counters=link.counters();clear guard
% Use one timestamp to define the 60 s lease and avoid skew between clock reads.
handoffCreated=datetime('now','TimeZone','UTC','Format',"yyyy-MM-dd'T'HH:mm:ss.SSS'Z'");
result.handoff_created_utc=char(handoffCreated);
result.expires_utc=char(handoffCreated+seconds(60));
result.passed=result.passed&&result.COM_closed;
save(string(outputPath)+'.mat','result');
f=fopen(outputPath,'w','n','UTF-8');assert(f>=0,'gpenmpc:HandoffOutputOpen','Could not create the handoff result file.');c=onCleanup(@()fclose(f));fprintf(f,'%s\n',jsonencode(result,PrettyPrint=true));clear c
disp(jsonencode(struct('passed',result.passed,'operation',operation,'COM_closed',result.COM_closed,'reboot_attempts',result.controlled_application_reboot_to_bootloader_count)));
 function take(r)
  row=struct('name',char(r.name),'mav_type',double(r.mav_type),'raw_bits_hex',upper(char(r.raw_bits_hex)));
  at=find(strcmp({target.name},row.name));if isempty(at),target(end+1)=row;
  else,assert(isscalar(at)&&isequal(target(at),row),'gpenmpc:ParentParameterDuplicate','Duplicate parameter %s has conflicting semantics.',row.name);end
 end
 function setEkfInstanceCount(desired)
  % Provision one IMU for the single simulated sensor so task_spawn does not
  % wait for three instances while holding the module mutex.
  assert(ismember(desired,[1 3]));
  q=link.requestParam('EKF2_MULTI_IMU',3);result.ekf_instance_before=q;
  assert(q.mav_type==6&&ismember(string(q.raw_bits_hex),["00000001","00000003"]), ...
   'gpenmpc:EkfInstanceUnknown','Only observed original3 or candidate1 may be restored.');
  % A CRC-verified update can retain the provisioned value of one IMU.
  % Read back and save it independently; first-time provisioning starts from three.
  if desired==1&&~nativeImuReboot
   assert(strcmp(q.raw_bits_hex,'00000003'),'gpenmpc:EkfInstanceOriginal','Initial provisioning must start from the original instance count.');
  end
  expected=dec2hex(uint32(desired),8);result.ekf_instance_target=desired;
  if ~strcmp(q.raw_bits_hex,expected)
   link.drain();link.requestMessage(245);e=link.waitForMessage('EXTENDED_SYS_STATE',3,[]);
   link.drain();b=link.waitForMessage('HEARTBEAT',3,[]);
   assert(e.Payload.landed_state==1&&bitand(uint8(b.Payload.base_mode),uint8(128))==0, ...
    'gpenmpc:EkfInstanceState','Never change estimator provisioning while armed or not on ground.');
   result.ekf_instance_write_attempts=1;save(string(outputPath)+'.mat','result');
   link.sendMessage('PARAM_SET',struct('target_system',uint8(1),'target_component',uint8(1), ...
    'param_id','EKF2_MULTI_IMU','param_type',uint8(6),'param_value',typecast(int32(desired),'single')));
   result.parameter_writes=result.parameter_writes+1;save(string(outputPath)+'.mat','result');
   result.ekf_instance_echo=link.waitForMessage('PARAM_VALUE',3,@(m) ...
    m.SystemID==1&&m.ComponentID==1&&strcmp(gpenmpcNative.MavlinkSerialLink.cleanText(m.Payload.param_id),'EKF2_MULTI_IMU'));
   p=result.ekf_instance_echo.Payload;
   assert(p.param_type==6&&strcmp(dec2hex(typecast(single(p.param_value),'uint32'),8),expected), ...
    'gpenmpc:EkfInstanceEcho','Native typed parameter echo does not match the requested instance count.');
  end
  result.ekf_instance_count=link.requestParam('EKF2_MULTI_IMU',3);
  assert(result.ekf_instance_count.mav_type==6&&strcmp(result.ekf_instance_count.raw_bits_hex,expected), ...
   'gpenmpc:EkfInstanceReadback','Independent typed instance-count readback differs.');
  % PARAM_VALUE confirms RAM state, not persistence across reboot.
  % Use the blocking param_save_default(true) command to complete the save.
  result.ekf_instance_save_attempts=1;save(string(outputPath)+'.mat','result');
  result.ekf_instance_save_response=char(link.shellCommand('param save',5));
  savedText=regexprep(result.ekf_instance_save_response,'\x1b\[[0-9;]*[A-Za-z]','');
  savedAt=strfind(savedText,'param save');
  result.ekf_instance_save_verified=~isempty(savedAt)&& ...
   contains(savedText(savedAt(end)+numel('param save'):end),'nsh>')&& ...
   isempty(regexpi(savedText,'error|failed|not found|timeout','once'));
  assert(result.ekf_instance_save_verified,'gpenmpc:EkfInstancePersistence', ...
   'Do not reboot before the native blocking parameter save completes successfully.');
 end
end
function s=utc(),s=char(datetime('now','TimeZone','UTC','Format',"yyyy-MM-dd'T'HH:mm:ss.SSS'Z'"));end

function evidence=readFreshPnp()
% Inspect the owned process state.
script="$ErrorActionPreference='Stop'; [Console]::OutputEncoding=[System.Text.UTF8Encoding]::new(); " + ...
 "$d=@(Get-PnpDevice -PresentOnly -ErrorAction Stop | Where-Object {$_.FriendlyName -match '\(COM3\)$'} | " + ...
 "ForEach-Object {[ordered]@{instance_id=$_.InstanceId;friendly_name=$_.FriendlyName;status=$_.Status}}); " + ...
 "[ordered]@{observed_utc=[DateTime]::UtcNow.ToString('o');process_id=$PID;devices=$d} | ConvertTo-Json -Depth 4 -Compress";
startInfo=System.Diagnostics.ProcessStartInfo;
startInfo.FileName='powershell.exe';
startInfo.Arguments=['-NoProfile -NonInteractive -Command "' char(script) '"'];
startInfo.UseShellExecute=false;startInfo.CreateNoWindow=true;
startInfo.RedirectStandardOutput=true;startInfo.RedirectStandardError=true;
startInfo.StandardOutputEncoding=System.Text.Encoding.UTF8;startInfo.StandardErrorEncoding=System.Text.Encoding.UTF8;
process=System.Diagnostics.Process;process.StartInfo=startInfo;
cleanup=onCleanup(@()process.Dispose());
evidence=struct('probe_started_utc',utc(),'probe_completed_utc','','process_id',0, ...
 'exit_code',NaN,'stdout','','stderr','','read_only',true,'COM_open_count',0);
assert(process.Start(),'gpenmpc:PnpProcessStart','Could not start the owned read-only PnP query.');
evidence.process_id=double(process.Id);
out=process.StandardOutput.ReadToEndAsync();err=process.StandardError.ReadToEndAsync();
if ~process.WaitForExit(int32(8000))
 process.Kill();process.WaitForExit(int32(1000));
 error('gpenmpc:PnpProcessTimeout','Owned read-only PnP query exceeded eight seconds before COM open.');
end
evidence.exit_code=double(process.ExitCode);evidence.stdout=char(out.Result);evidence.stderr=char(err.Result);
evidence.probe_completed_utc=utc();
assert(evidence.exit_code==0,'gpenmpc:PnpQueryFailed','Read-only PnP query failed: %s',evidence.stderr);
evidence.parsed=jsondecode(evidence.stdout);
assert(evidence.parsed.process_id==evidence.process_id,'gpenmpc:PnpProcessIdentity','PnP output process identity differs.');
clear cleanup
end
