function result=execute_m600_board_local_short(outputRoot,operation,rcCalibrationPath,rcDurationS,saveFullRaw,retainManualFirmware)
% Prepare or execute the board-local session and its recovery.
% PREPARE initializes assets and the isolated loopback environment sender.
% LIVE manages application handoff, simulator ownership and restoration.
arguments
 outputRoot (1,1) string
 operation (1,1) string {mustBeMember(operation,["PREPARE","LIVE","PREPARE_AND_LIVE","VALIDATE_CLEANUP"])}="PREPARE"
 rcCalibrationPath (1,1) string=""
 rcDurationS (1,1) double {mustBeMember(rcDurationS,[0 20 120])}=20
 saveFullRaw (1,1) logical=true
 retainManualFirmware (1,1) logical=false
end
assert(~retainManualFirmware||strlength(rcCalibrationPath)>0,'gpenmpcShort:FirmwareRetentionScope', ...
 'Firmware retention is selected only by the manual demonstration entry.');
assert(saveFullRaw||strlength(rcCalibrationPath)>0,'gpenmpcShort:RecordingScope', ...
 'No full recording is permitted only in the explicit manual demonstration.');
assert(strlength(rcCalibrationPath)>0||rcDurationS==20, ...
 'gpenmpcShort:RcDurationScope','Extended operator duration applies only to the explicit USB reference component.');
% Limit BLAS concurrency for the MATLAB IO owner.
maxNumCompThreads(1);
if operation~="VALIDATE_CLEANUP"
 % Store session recordings in the configured HIL log directory.
 canonicalOutput=string(java.io.File(char(outputRoot)).getCanonicalPath());
 approvedLogPrefix=string(gpenmpc_log_root())+filesep;
 assert(startsWith(lower(canonicalOutput),lower(approvedLogPrefix)), ...
  'gpenmpcShort:LogStorageLocation','New PREPARE/LIVE recordings must use the configured HIL log directory.');
end
if operation=="VALIDATE_CLEANUP"
 result=validateCapturedOuterCleanup();
 assert(isfolder(outputRoot),'gpenmpcShort:ExistingRoot','Use the existing HOST check directory.');
 writeJson(fullfile(outputRoot,'CLEANUP_CAPTURE_RESULT.json'),result);disp(jsonencode(result));return
end
build=string(fileparts(fileparts(mfilename('fullpath'))));
rflyRoot=getenv('GPENMPC_RFLY_ROOT');
assert(~isempty(rflyRoot)&&isfolder(rflyRoot),'gpenmpcShort:Installation', ...
 'Set GPENMPC_RFLY_ROOT to the RflySim/PX4PSP installation directory.');
candidateSha='AF571397E67013D23D59EF2827B879B76A408271713891686790BD64661B4743';
candidateBuild='Sep 25 2026 17:26:15';
if strlength(rcCalibrationPath)>0
 assert(isfile(fullfile(fileparts(fileparts(build)),'firmware','fmuv6c','px4_fmu-v6c_default.px4')), ...
  'gpenmpcShort:ApplicationPreparing','Application update in progress. Wait before starting.');
end
addpath(fullfile(build,'tools'),fullfile(build,'host_runtime'),fullfile(build,'matlab_validation'), ...
 fullfile(build,'m600_coptersim','matlab_validation'));
assert(isfolder(outputRoot),'gpenmpcShort:ExistingRoot','Use the existing activity directory.');
resultPath=fullfile(outputRoot,'OUTER_SHORT_RESULT.json');
assert(~isfile(resultPath),'gpenmpcShort:Consumed','Use a new live-run directory.');
try % Preparation is strictly before the first board transaction below.
asset=gpenmpcNative.loadCanonicalAssets();
taskPath=fullfile(build,'task_packages','cambridge_canonical','MU_CAMBRIDGE_MA_02__CANONICAL_PHYSICAL_TASK.mat');
taskSha='B876546B468265F84BD028AC549B06EF8ADDCB25CBA09CF389A08F4B21290A5F';
bundle=gpenmpcNative.loadRflyCanonicalDeliveryTask(taskPath,taskSha);
% Prepare the reference in task coordinates before the simulator starts.
% The estimator and ground checks establish the registered origin later.
[initialTrajectory,initialReferenceBinding]=gpenmpcNative.bindRflyCanonicalInitialTakeoffTrajectory(bundle.legs{1},zeros(3,1));
% Warm the environment evaluator before starting its real-time owner.
for referencePhase=[0 .02 20 20.02 25.02]
 initialTrajectory.environment_jet_fcn(referencePhase);
end
[contracts,proof]=load_m600_recovery_contracts();
postbootPath=fullfile(outputRoot,'NEW_APPLICATION_READONLY.json');
cfg=make_m600_board_local_short_config('M600_CANONICAL_DIAGNOSTIC_SAME_6DOF_MODEL',postbootPath,asset);
if strlength(rcCalibrationPath)>0
 cfg.display_truth_port=30251; % Optional nonblocking display copy.
 % Manual commands are limited to 5 m/s horizontal and 1.5 m/s vertical.
 % The 10 m/s divergence guard is a separate diagnostic bound.
 cfg.abort_truth_speed_mps=10;
 assert(isfile(rcCalibrationPath),'gpenmpcShort:RcCalibration','Physical USB axis calibration required.');
 calibrationData=load(rcCalibrationPath,'calibration');
operatorSha='78BA9A5E29F825D4A594A1D91EDC9012E0BA88A16E52D06C8E86B6A797246EA7';
 assert(strcmpi(m600check.fileSha256(fullfile(build,'rfly_vendor_integration','CanonicalOperatorReference.hpp')),operatorSha));
 boardSource=fileread(fullfile(build,'rfly_vendor_integration','px4_runtime','CanonicalLocalSessionEntry.cpp'));
 boardReference=regexp(boardSource,'operator_reference_sha\[32\]\s*=\s*\{([^}]+)\}','tokens','once');
 boardBytes=regexp(boardReference{1},'0x([0-9a-fA-F]{2})','tokens');
 boardHex=cellfun(@(x)x{1},boardBytes,'UniformOutput',false);
 assert(strcmpi([boardHex{:}],operatorSha),'gpenmpcShort:OperatorReferenceBinding', ...
  'Host reference and board registration must agree before any device transaction.');
 % Set the manual session duration; zero runs until an explicit stop.
 cfg.local_short.duration_s=rcDurationS;
 cfg.local_short.save_full_raw=saveFullRaw;
 cfg.local_short.component_initialization=true;
 cfg.local_short.service_cfg.component_initialization=true;
 cfg.local_short.service_cfg.operator_reference=calibrationData.calibration;
 cfg.local_short.service_cfg.operator_reference_sha256=operatorSha;
 cfg.local_short.manual_reset_path=fullfile(outputRoot,'MANUAL_RESET_REQUEST');
 addpath(fullfile(build,'tools','usb_rc_runtime'),'-begin');
 rcCleanup=onCleanup(@()gpenmpc_usb_joystick_mex('close')); %#ok<NASGU>
  sdlPath=fullfile(rflyRoot,'QGroundControl','SDL2.dll');
  assert(isfile(sdlPath),'gpenmpcShort:SDL2','SDL2 library was not found: %s',sdlPath);
  usb=gpenmpc_usb_joystick_mex('open',sdlPath);
 rc=gpenmpcNative.normalizeUsbRcInput(usb,calibrationData.calibration,usb.host_read_qpc_s,.1);
 assert(rc.valid,'gpenmpcShort:RcInput','USB input is not available: %s',rc.reason);
end
cfg.temporary_allocator_geometry=contracts.temporary_allocator_geometry;
cfg.native_hover_tuning=contracts.native_hover_tuning;
getSource=struct('exact_path',char(fullfile(build,'runtime_assets','state_reader','rfly_original_getter_mex.mexw64')));
getSource.sha256=m600check.fileSha256(getSource.exact_path);
getSource.model_sha256='D536EACE85EBA30A6CE07EF5B38FF108B132C9E3B230A3C46E93F3E1C9CA6E12';
assert(strcmpi(getSource.sha256,'E6A9F65749C68A926913B07ACCFBE4004888013DF502431F09FB249F97E90F55'));
addpath(fileparts(getSource.exact_path),fileparts(cfg.local_short.service_cfg.gp_backend.path), ...
 fileparts(cfg.local_short.service_cfg.input_codec_backend.path),fileparts(cfg.mavlink_transport.source.exact_path),'-begin');
assert(strcmpi(which('rfly_original_getter_mex'),getSource.exact_path));
assert(strcmpi(m600check.fileSha256(cfg.mavlink_transport.source.exact_path),cfg.mavlink_transport.source.sha256));
assert(strcmpi(m600check.fileSha256(cfg.local_short.service_cfg.gp_backend.path),cfg.local_short.service_cfg.gp_backend.binary_sha256));
assert(strcmpi(m600check.fileSha256(cfg.local_short.service_cfg.input_codec_backend.path),cfg.local_short.service_cfg.input_codec_backend.binary_sha256));
% Validate IO construction before board setup.
ioConfiguration=m600check.makeM600CopterSimIo(cfg,'VALIDATE_ONLY');
assert(ioConfiguration.passed,'gpenmpcShort:IoConfiguration','Actual IO configuration did not pass its pure construction guards.');
% Load the launch-only helper before opening receivers or starting NoUI.
% It preserves all arguments/environment but prevents inherited MATLAB UDP
% handles from surviving the sole host owner's exit.
noInheritDll=fullfile(build,'tools','NoInheritProcess.dll');
assert(strcmpi(m600check.fileSha256(noInheritDll),'16595B9ED1B00481994D4781DA72251CB9DAD6E4DFAF99520F681B280C1B34DF'), ...
 'gpenmpcShort:LaunchHelper','Use the inheritance-tested launch-only helper.');
NET.addAssembly(char(noInheritDll));
parentRuntime=getenv('GPENMPC_COPTERSIM_RUNTIME');
if isempty(parentRuntime)
 parentRuntime=fullfile(gpenmpc_external_path('official_noui_step_snapshot'),'runtime');
end
% Use a stable executable path for Windows application and firewall rules.
for runtimeOwner={'CopterSim','CopterSimNoUI'}
 assert(System.Diagnostics.Process.GetProcessesByName(runtimeOwner{1}).Length==0, ...
  'gpenmpcShort:RuntimeBusy','Leave the running simulator and its assets untouched.');
end
runtime=fullfile(build,'live','COPTERSIM_M600_RUNTIME');
runtimeCreated=~isfolder(runtime);
if runtimeCreated
 assert(ismember(operation,["PREPARE","PREPARE_AND_LIVE"]),'gpenmpcShort:PrepareRuntime','Prepare the fixed runtime before LIVE.');
 assert(isfolder(parentRuntime),'gpenmpcShort:CopterSimRuntime', ...
  'Set GPENMPC_COPTERSIM_RUNTIME to the verified CopterSimNoUI runtime directory.');
 [ok,msg]=copyfile(parentRuntime,runtime);assert(ok,'%s',msg);
end
% Load the heightfield and calibration used by the 3D scene.
mapRoot=fullfile(runtime,'external','map');if ~isfolder(mapRoot),mkdir(mapRoot);end
for mapName=["3DDisplay.txt","3DDisplay.png"]
  mapSource=fullfile(rflyRoot,'CopterSim','external','map',mapName);
 mapTarget=fullfile(mapRoot,mapName);
 if ~isfile(mapTarget),[ok,msg]=copyfile(mapSource,mapTarget);assert(ok,'%s',msg);end
 assert(strcmpi(m600check.fileSha256(mapSource),m600check.fileSha256(mapTarget)), ...
  'gpenmpcShort:TerrainIdentity','Retain the exact vendor terrain calibration and heightfield.');
end
exe=fullfile(runtime,'CopterSimNoUI.exe');model=fullfile(runtime,'external','model','GPENMPC_M600_Diagnostic.dll');
terrainObserver=fullfile(build,'m600_coptersim','model','GPENMPC_M600_Diagnostic.dll');
assert(strcmpi(m600check.fileSha256(terrainObserver),getSource.model_sha256));
if ~strcmpi(m600check.fileSha256(model),getSource.model_sha256)
 assert(strcmpi(m600check.fileSha256(model),'990850A2F40F3FCC2A6C47E63A4065B60FF49AA39CC4749FF443963B06F2EF7E'));
 assert(ismember(operation,["PREPARE","PREPARE_AND_LIVE"])&&runtimeCreated, ...
  'gpenmpcShort:RuntimeModelIdentity','Initialize a new idle runtime.');
 [ok,msg]=copyfile(terrainObserver,model);assert(ok,'%s',msg);
end
% Ground calibration is independent of the aircraft and estimator readings.
% Compare local estimator coordinates with ground-relative plant coordinates.
cfg.initial_world_ground_ned_m=[0;0;terrainAtOrigin(mapRoot)];
cfg.bound_provenance.origin='Apply the 0.5 m residual check after the fixed vendor-map ground translation.';
assert(strcmpi(m600check.fileSha256(exe),'94B81EFB44058176DD5353669D9C28FC5331CC8411AB9EA3F2D27C1E8C343241'));
assert(strcmpi(m600check.fileSha256(model),getSource.model_sha256));
% Use HITLRunNoUI.bat positional arguments with the selected DLL and PX4_HITL mode 0.
simArgs='1 1 -1 GPENMPC_M600_Diagnostic 0 3DDisplay 0 0 0 0 3:921600 2 0';
task=struct('path',taskPath,'sha256',taskSha,'configuration_sha256',char(asset.binding.effective_configuration_payload_sha256), ...
 'environment_policy',rmfield(cfg.delivery_environment_contract,{'initial_payload_kg','remote_port'}),'copter_id',1);
task.cached_asset=gpenmpcNative.RflyLocalTaskAsset(task);
setupRecordPath=getenv('HIL_SETUP_RECORD');
assert(~isempty(setupRecordPath)&&isfile(setupRecordPath), ...
 'gpenmpcShort:HardwareSetupRecord','Configure HIL_SETUP_RECORD with the hardware setup record.');
original=fileread(setupRecordPath);
auth=struct('source','OperatorUsbIsolationDeclaration', ...
 'original_record',original,'physical_setup_record_sha256',textSha(original));
% Construct official codec before NoUI starts filling the original ring.
dialect=mavlinkdialect(cfg.local_mavlink_dialect.path,2); %#ok<NASGU>
% Warm receive and phase decoders using retained records before hardware setup.
oldRx=load(fullfile(fileparts(fileparts(build)),'assets','runtime','receiver_warmup.mat'));
oldC=gpenmpcNative.RflyLocalCommittedDecoder(oldRx.records{1}.message,oldRx.records{1}.original_host_receive_ns);
oldReg=struct('identity',oldC.identity,'leg_index',oldC.leg_index, ...
 'task_sha256',oldRx.association.confirm_receipt.parsed_fields.task_sha, ...
 'configuration_sha256',hexBytes(oldC.configuration_sha256), ...
 'reference_asset_sha256',hexBytes(oldC.reference_asset_sha256), ...
 'execution_session_sha256',oldRx.association.execution_session_sha256);
oldPhase=gpenmpcNative.RflyLocalPhaseView(bundle,initialTrajectory,oldReg,oldRx.association);
for oldIndex=1:numel(oldRx.records)
 oldPhase.ingest(oldRx.records{oldIndex});oldPhase.view();
end
oldPhase.suspend('NATIVE_LAND');oldPhase.view();
clear oldRx oldC oldReg oldPhase oldIndex
% Warm the start-receipt parser before the live heartbeat interval.
oldStart=load(fullfile(build,'runtime_assets','receiver','EXISTING_START_PARSE_TIMING.mat'),'frames','req');
for oldIndex=1:2
 oldObserved=gpenmpcNative.RflyLocalStartReceiptDecoder(oldStart.frames,oldStart.req,dialect);
 assert(oldObserved.task_created==1);
end
clear oldStart oldObserved oldIndex
pureResources=struct('task_source',task,'getter_source',getSource,'decoder_dialect',dialect, ...
 'assets',asset,'initial_trajectory',initialTrajectory,'preconstruct_outer',true);
pureObjects=prepare_m600_board_local_short_objects(cfg,pureResources);
% Own the idle worker from preparation through cleanup; bind its session later.
process=[];retainOuterOwner=false;
outerCleanupGate=containers.Map('KeyType','char','ValueType','logical');outerCleanupGate('join_allowed')=true;
outerOwnerCleanup=captureOuterCleanup(pureObjects.outer,outerCleanupGate); %#ok<NASGU>
% Warm the encoder and send validator with a retained successful input.
inputInitialization=initializeRetainedInput(build,pureObjects.method_backends.input_codec, ...
 cfg.canonical_exchange.gp_reply_host_max_age_ns,cfg,asset,pureObjects.method_backends.gp);
prepared=struct('operation','PREPARE','passed',true,'board_actions',0,'task_sha256',taskSha, ...
 'configuration_sha256',task.configuration_sha256,'getter_source',getSource,'recovery_source',proof, ...
 'simulator_executable',exe,'simulator_working_directory',runtime, ...
 'simulator_arguments',simArgs,'model_sha256',m600check.fileSha256(model), ...
 'candidate_application_sha256',candidateSha, ...
 'restore_application_sha256','7722616157AF96E3493D1827F01A0713D92373946C669B854B8F043552892FC7');
prepared.pure_objects=pureObjects.receipt;
prepared.initial_reference=initialReferenceBinding;
prepared.initial_world_ground_ned_m=cfg.initial_world_ground_ned_m;
prepared.retained_input_initialization=inputInitialization;
prepared.short_file_logger_exclusion=true;
prepared.component_initialization=cfg.local_short.component_initialization;
prepared.planned_active_s=cfg.local_short.duration_s;
prepared.operator_reference=strlength(rcCalibrationPath)>0;
prepared.operator_calibration_path=char(rcCalibrationPath);
if prepared.operator_reference
 prepared.operator_calibration_sha256=m600check.fileSha256(rcCalibrationPath);
 prepared.reference_source='USB_OPERATOR_BODY_HEADING_VELOCITY_AND_YAW_RATE';
 prepared.complete_enmpc_gp_method=false;
 prepared.operator_reference_sha256=operatorSha;
end
if operation=="PREPARE"
 pureObjects.outer.close();prepared.outer_closed=pureObjects.outer.status();
 clear outerOwnerCleanup
 writeJson(fullfile(outputRoot,'SHORT_ENTRY_PREPARED.json'),prepared);result=prepared;disp(jsonencode(result));return
end
if operation=="PREPARE_AND_LIVE"
 % Same invocation, same verified objects. Avoid loading/constructing the
 % identical assets twice and closing/recreating the idle outer worker.
 writeJson(fullfile(outputRoot,'SHORT_ENTRY_PREPARED.json'),prepared);
 fprintf('USB_RC_PREPARED: Assets and interfaces ready; entering application / parameter setup.\n');
end
assert(isfile(fullfile(outputRoot,'SHORT_ENTRY_PREPARED.json')),'gpenmpcShort:PrepareFirst','Construct the exact entry before hardware.');
priorPrepared=jsondecode(fileread(fullfile(outputRoot,'SHORT_ENTRY_PREPARED.json')));
assert(isfield(priorPrepared,'planned_active_s')&&priorPrepared.planned_active_s==prepared.planned_active_s, ...
 'gpenmpcShort:PreparedDuration','PREPARE and LIVE must use the same declared duration.');
assert(strcmpi(priorPrepared.simulator_executable,exe) ...
 &&isfield(priorPrepared,'simulator_working_directory') ...
 &&strcmpi(priorPrepared.simulator_working_directory,runtime), ...
 'gpenmpcShort:PreparedRuntimeIdentity','Repeat HOST PREPARE after a runtime-path change, before any board action.');
assert(isfield(priorPrepared,'operator_reference')&&priorPrepared.operator_reference==prepared.operator_reference, ...
 'gpenmpcShort:PreparedReferenceMode','PREPARE and LIVE must select the same reference mode.');
if prepared.operator_reference
 assert(strcmpi(priorPrepared.operator_calibration_sha256,prepared.operator_calibration_sha256) ...
  &&strcmpi(priorPrepared.operator_reference_sha256,prepared.operator_reference_sha256), ...
  'gpenmpcShort:PreparedOperatorInput','PREPARE and LIVE must use the same physical calibration and reference generator.');
end
for name={'CopterSim','CopterSimNoUI','QGroundControl'}
 assert(System.Diagnostics.Process.GetProcessesByName(name{1}).Length==0,'gpenmpcShort:CompetingOwner','Competing simulator/QGC must not be active.');
end
% CopterSim alone selects this map and publishes this vehicle's 3DOutput.
[~,transaction,extension]=fileparts(outputRoot);transaction=char(transaction);
assert(isempty(extension)&&~isempty(regexp(transaction,'^manual_session_[0-9]{3}$','once')));
process=[];reader=[];io=[];short=[];safetyOwner=[];setup=[];upload=[];restored=[];post=[];failures={};
routeSetup=[];routeRestored=[];routePath=fullfile(outputRoot,'NATIVE_RECOVERY_ROUTE_SETUP.json');
setupPath=fullfile(outputRoot,'TEMPORARY_PARAMETER_SETUP.json');
result=struct('schema','GPENMPC_BOARD_LOCAL_SHORT_OUTER_V1','firmware_verified',false, ...
 'board_inner_running',false,'short_completed',false,'task_completed',false,'safe',false, ...
 'physical_output_actions',0,'bootloader_firmware_writes',0,'host_inner_steps',0);
result.manual_firmware_retained=false;result.firmware_reused=false;
result.application_uploaded_this_run=false;result.original_firmware_restored=false;
result.manual_retain_requested=retainManualFirmware;
reuseSource='';startupPostVerified=false;
if retainManualFirmware
 previous=dir(fullfile(fileparts(outputRoot),'manual_session_*','OUTER_SHORT_RESULT.json'));
 previous=previous(~strcmp({previous.folder},char(outputRoot)));
 if ~isempty(previous)
  [~,order]=sort({previous.folder});previous=previous(order(end:-1:1));
  for previousIndex=1:numel(previous)
  previousPath=fullfile(previous(previousIndex).folder,previous(previousIndex).name);prior=jsondecode(fileread(previousPath));
  % Skip preparation failures that occurred before device access.
  if isfield(prior,'preparation_failed')&&prior.preparation_failed&&isfield(prior,'board_actions')&&prior.board_actions==0
   continue;
  end
  % Recovery can complete after the original failed transaction returned.
  % Retain that failure file; use only its own actual restoration records.
  recoveredPath=fullfile(previous(previousIndex).folder,'FINAL_SAFETY_AFTER_LINK_RETURN.json');
  restoredPath=fullfile(previous(previousIndex).folder,'PARAMETER_RESTORE_AFTER_LINK_RETURN.json');
  routeRestoredPath=fullfile(previous(previousIndex).folder,'ROUTE_RESTORE_AFTER_LINK_RETURN.json');
  if ~prior.safe&&isfield(prior,'manual_retain_requested')&&prior.manual_retain_requested ...
    &&all(isfile({recoveredPath,restoredPath,routeRestoredPath}))
   r=jsondecode(fileread(recoveredPath));p=jsondecode(fileread(restoredPath));q=jsondecode(fileread(routeRestoredPath));
   sameTransaction=p.passed&&p.COM_closed&&q.passed&&q.COM_closed ...
    &&r.passed&&r.COM_closed&&r.disarmed&&r.landed&&r.physical_path_disabled ...
    &&strcmpi(r.installed_build_datetime,candidateBuild)&&strcmp(r.uid,gpenmpc_device_identity('uid')) ...
    &&strcmpi(prior.installed_application_sha256,candidateSha) ...
    &&strcmpi(p.source.path,fullfile(previous(previousIndex).folder,'TEMPORARY_PARAMETER_SETUP.json')) ...
    &&strcmpi(p.source.sha256,m600check.fileSha256(p.source.path)) ...
    &&strcmpi(q.source.path,fullfile(previous(previousIndex).folder,'NATIVE_RECOVERY_ROUTE_SETUP.json')) ...
    &&strcmpi(q.source.sha256,m600check.fileSha256(q.source.path));
   if sameTransaction
    reuseSource=recoveredPath;break;
   end
  end
  if isfield(prior,'manual_firmware_retained')&&prior.manual_firmware_retained&&prior.safe ...
    &&isfield(prior,'installed_application_sha256')&&strcmpi(prior.installed_application_sha256,candidateSha)
   reuseSource=previousPath;
  end
  break; % Never skip a real board transaction or an unknown recovery result.
  end
 end
end
result.simulator_executable=exe;result.simulator_working_directory=runtime;
% Match the MATLAB IO owner's priority to the simulator carrier.
hostProcess=System.Diagnostics.Process.GetCurrentProcess();
originalHostPriority=hostProcess.PriorityClass;
hostPriorityCleanup=onCleanup(@()restoreHostPriority(hostProcess,originalHostPriority)); %#ok<NASGU>
result.host_schedule=struct('pid',double(hostProcess.Id),'before',char(originalHostPriority.ToString()), ...
 'requested','AboveNormal','other_processes_changed',false,'resource_hold',false);
hostProcess.PriorityClass=System.Diagnostics.ProcessPriorityClass.AboveNormal;
hostProcess.Refresh();result.host_schedule.actual=char(hostProcess.PriorityClass.ToString());
assert(strcmp(result.host_schedule.actual,'AboveNormal'),'gpenmpcShort:HostSchedule');
catch preparationError
 failure=MException('gpenmpcShort:Preparation','%s',preparationError.message);
 throw(addCause(failure,preparationError));
end
cleanup=onCleanup(@finish);finalized=false;
try
 % Perform the guarded application handoff and single upload with CRC verification.
 if isempty(reuseSource)
 handoffPath=fullfile(outputRoot,'GPENMPC_FLASH_HANDOFF.json');
 h=m600_local_application_handoff(handoffPath,'REBOOT_GPENMPC',transaction);
 assert(h.passed,'gpenmpcShort:FlashHandoff','Preflash safety/handoff failed.');
 upload=uploadApplication('fmuv6c',handoffPath,'GPENMPC_UPLOAD');
 result.firmware_verified=upload.application_crc_stage_passed;
 result.application_uploaded_this_run=isfield(upload,'application_upload_attempts')&&upload.application_upload_attempts>0;
 assert(result.firmware_verified,'gpenmpcShort:Upload','Application upload not verified; recovery first.');
 waitForCom();
 % Apply the corrected SPI driver before persisting single-IMU provisioning.
 result.imu_provisioning=m600_local_application_handoff(fullfile(outputRoot,'FIXED_DRIVER_IMU_REBOOT.json'), ...
  'REBOOT_HIL_IMU',transaction);
 assert(result.imu_provisioning.passed&&result.imu_provisioning.ekf_instance_save_verified, ...
  'gpenmpcShort:FixedDriverImuProvisioning','No HIL before native save succeeds.');
 waitForCom();
 else
  % Verify the installed application before reuse.
        fprintf('USB_RC_FIRMWARE_REUSE: Initializing the installed flight controller firmware.\n');
  restart=m600_local_application_handoff(fullfile(outputRoot,'RETAINED_APPLICATION_RESTART.json'), ...
      'RESTART_RETAINED_HIL_APPLICATION',transaction,false,true,candidateBuild);
  assert(restart.passed&&restart.COM_closed&&restart.controlled_native_application_reboot_count==1 ...
      &&restart.reboot_ack_accepted,'gpenmpcShort:RetainedRestart','Retained application restart not acknowledged.');
  result.retained_application_restart=restart;
  waitForCom();
 end
 % Disable the board file logger; host record retention follows saveFullRaw.
 if prepared.operator_reference
  assert(cfg.local_short.component_initialization&&cfg.local_short.service_cfg.component_initialization);
 else
  assert(~cfg.local_short.component_initialization&&~cfg.local_short.service_cfg.component_initialization, ...
   'gpenmpcShort:FullMethodSelection','Both host selectors must use the original outer and GP services.');
 end
 post=m600_local_application_handoff(postbootPath,'SAFE_STOP_PX4IO_START_BOARD_ADC',transaction,false,true,candidateBuild);
 assert(post.passed,'gpenmpcShort:PostbootSafety','New application postboot safety failed.');
 startupPostVerified=true;
 result.firmware_reused=~isempty(reuseSource);result.firmware_reuse_source=reuseSource;
 result.application_crc_checked_this_run=result.application_uploaded_this_run&&result.firmware_verified;
 result.firmware_verified=true; % Fresh build/board identity plus original verified installation.
 result.installed_application_sha256=candidateSha;
 result.firmware_identity_basis='Verified installation record plus fresh exact board GUID, commit and build datetime; no new CRC on reuse';
 % Temporarily select native LAND for offboard loss and restore its original value.
 routeSetup=m600_local_recovery_route(routePath,'APPLY',postbootPath);
 assert(routeSetup.passed,'gpenmpcShort:RecoveryRoute','Native LAND recovery route not established.');
 setup=m600_local_short_parameter_setup(setupPath,'APPLY',postbootPath, ...
  contracts.temporary_allocator_geometry,contracts.native_hover_tuning,'');
 assert(setup.passed,'gpenmpcShort:ParameterSetup','Temporary native-recovery setup failed.');
 cfg.live_enabled=true;cfg.outer_preflight_pass=true;
 % Prepare receivers before launching the serial simulator, then enter the loop.
 io=m600check.makeM600CopterSimIo(cfg);
 resources=struct('assets',asset,'bundle',bundle,'task_source',task,'authorization',auth,'decoder_dialect',dialect, ...
  'getter_source',getSource,'challenge',uint64([0;0]), ...
  'applied_parameters',struct('source_path',char(setupPath),'source_sha256',m600check.fileSha256(setupPath)));
 resources.initial_trajectory=initialTrajectory;resources.initial_reference_binding=initialReferenceBinding;
 resources.prepared_outer=pureObjects.outer;
 resources.prepared_method_backends=pureObjects.method_backends;
 rngBytes=zeros(1,16,'uint8');rng=System.Security.Cryptography.RandomNumberGenerator.Create();
 netBytes=NET.createArray('System.Byte',16);rng.GetBytes(netBytes);rng.Dispose();rngBytes(:)=uint8(netBytes);
 resources.challenge=typecast(rngBytes,'uint64');
 % Construct buffers and services before the simulator fills its rings.
 resources.short_objects=prepare_m600_board_local_short_objects(cfg,resources,io);
 reader=rfly_original_getter_mex('open');resources.getter_open_receipt=reader;
 si=System.Diagnostics.ProcessStartInfo;si.FileName=char(exe);si.Arguments=simArgs;si.WorkingDirectory=char(runtime);
 si.UseShellExecute=false;si.CreateNoWindow=true;si.WindowStyle=System.Diagnostics.ProcessWindowStyle.Hidden;
 vars=si.EnvironmentVariables;
 vars.Remove('GPENMPC_CLOCK_RING_SECTION');vars.Add('GPENMPC_CLOCK_RING_SECTION',reader.ring_name);
 vars.Remove('GPENMPC_STEP_SNAPSHOT_SECTION');vars.Add('GPENMPC_STEP_SNAPSHOT_SECTION',reader.status_name);
 for key={'OMP_NUM_THREADS','MKL_NUM_THREADS','OPENBLAS_NUM_THREADS'},vars.Remove(key{1});vars.Add(key{1},'1');end
 process=GPENMPC.HostDiagnostics.NoInheritProcess.Start(si);outerCleanupGate('join_allowed')=false;
 resources.owned_simulator_pid=double(process.Id);
 % Raise the owned simulator carrier's scheduling priority.
 result.carrier_schedule=struct('pid',double(process.Id), ...
  'before',char(process.PriorityClass.ToString()),'requested','AboveNormal', ...
  'other_processes_changed',false,'resource_hold',false);
 process.PriorityClass=System.Diagnostics.ProcessPriorityClass.AboveNormal;
 process.Refresh();result.carrier_schedule.actual=char(process.PriorityClass.ToString());
 assert(strcmp(result.carrier_schedule.actual,'AboveNormal'), ...
  'gpenmpcShort:CarrierSchedule','Owned carrier scheduling change was not applied.');
 [short,safetyOwner]=run_m600_board_local_short_hil(fullfile(outputRoot,'SHORT_HIL'),cfg,io,resources);
 result.board_inner_running=short.counts.commits>0;result.short_completed=short.window_completed;
catch ex
 failures{end+1}=struct('stage','execution','identifier',ex.identifier,'message',ex.message,'stack',ex.stack);
 % Capture the first transport failure before recovery can report a secondary one.
 % evidence() reads retained records without receiving or transmitting.
 if ~isempty(io)&&saveFullRaw
  try,firstIo=io.evidence();save(fullfile(outputRoot,'FIRST_EXECUTION_IO_EVIDENCE.mat'),'firstIo','ex');
  catch evidenceEx,record('first_execution_evidence',evidenceEx);end
 end
end
finish();clear cleanup
% Complete recovery before optional raw-data serialization.
result.raw_storage_pending=~isempty(safetyOwner)&&isfield(safetyOwner,'persistRaw');
retainRawOwner=false;
% Publish the final recovery result once, before optional storage.
if result.raw_storage_pending&&isfield(short,'io_closed')&&short.io_closed
 try,short=safetyOwner.persistRaw();result.raw_storage_pending=false;
 catch storageEx,record('raw_storage',storageEx);result.raw_storage_error=storageEx.message;retainRawOwner=true;end
end
result.upload=upload;result.short=short;result.parameter_setup=setup;result.parameter_restore=restored;
result.recovery_route_setup=routeSetup;result.recovery_route_restore=routeRestored;
result.temporary_parameter_plan=struct('reference_geometry_tuning_mapping',21,'additional_commander_recovery_route',1, ...
 'additional_hil_estimator_instance_count',1,'total',23);
result.postflight=post;result.failures=failures;
result.preconstructed_outer=pureObjects.outer.status();
writeJson(resultPath,result);disp(jsonencode(struct('firmware_verified',result.firmware_verified, ...
 'board_inner_running',result.board_inner_running,'short_completed',result.short_completed,'safe',result.safe)));
% Keep the safety owner's handle reachable; JSON stores its status only.
if retainOuterOwner,result.retained_outer_owner=pureObjects.outer;end
if retainRawOwner,result.retained_raw_owner=safetyOwner;end % Retain unsaved arrays in the interactive result.
clear outerOwnerCleanup
 function finish()
  if finalized,return,end;finalized=true;
  % Stop method effects now, but NEVER join while native safety is pumping.
  pureObjects.outer.suspendForNativeLand();
  % End requests LAND. Explicit HIL Reset checks output isolation, stops the
  % virtual model and resets the application.
  manualReset=requestedManualReset();
  if ~manualReset&&~isempty(process)&&~process.HasExited
   if isempty(short)||~short.safe_ground
    if ~continueSafety()
     if prepared.operator_reference&&retainManualFirmware&&startupPostVerified
        fprintf('USB_RC_RESET_AVAILABLE: Select Reset simulation to return to the initial state.\n');
      while ~requestedManualReset(),pause(.1);end
      manualReset=true;
     else
     result.external_action_required=true;result.retained_simulator_pid=double(process.Id);
     result.safe=false;retainOuterOwner=true;
     % Retain the first failure and raw records when safety continuation is exhausted.
     if saveFullRaw,try
      if ~isempty(safetyOwner),retainedFailure=safetyOwner.raw();
      else,retainedFailure=struct('rawIo',io.evidence());end
      save(fullfile(outputRoot,'RETAINED_NATIVE_SAFETY_FAILURE.mat'),'retainedFailure','-v7.3');
     catch evidenceEx,record('retain_native_safety_failure',evidenceEx);end,end
     failures{end+1}=struct('stage','safety_boundary','identifier','gpenmpcShort:UnconfirmedGround', ...
      'message','Preserved NoUI sensor owner; no parameter restore/flash while arm or landing safety is unknown.');
     return
     end
    end
   end
  end
  % Release this session's transport and simulator before serial recovery.
  outerIoStopped=isempty(io);outerProducerStopped=isempty(process);outerGetterStopped=isempty(reader);
  if ~isempty(io),try,outerIoStopped=isequal(io.close(),true);catch ex,record('close_io',ex);end,end
  if ~isempty(process)
   try
    if ~process.HasExited,process.Kill();process.WaitForExit(10000);end
    outerProducerStopped=logical(process.HasExited);process.Dispose();
   catch ex,record('stop_owned_NoUI',ex);end
  end
  if ~isempty(reader)
   last=[];closed=[];
   try,last=rfly_original_getter_mex('drain');catch ex,record('drain_getter',ex);end
   try,closed=rfly_original_getter_mex('close');outerGetterStopped=true;catch ex,record('close_getter',ex);end
   if saveFullRaw,try,save(fullfile(outputRoot,'GETTER_FINAL.mat'),'last','closed');catch ex,record('save_getter',ex);end,end
  end
  % Join the worker after transport, simulator and getter shutdown is confirmed.
  outerCleanupGate('join_allowed')=outerIoStopped&&outerProducerStopped&&outerGetterStopped;
  if ~outerCleanupGate('join_allowed')
   retainOuterOwner=true;result.external_action_required=true;result.safe=false;
   failures{end+1}=struct('stage','outer_join_boundary','identifier','gpenmpcShort:ProducerStopUnconfirmed', ...
    'message','Retained original outer owner without joining: IO/NoUI/getter shutdown is not confirmed.');
   return
  end
  try,pureObjects.outer.close();catch outerCloseEx,record('close_preconstructed_outer',outerCloseEx);end
  if manualReset
   try
    fprintf('USB_RC_RESETTING: Resetting the flight controller and simulator connection...\n');
    result.manual_application_reset=reset_m600_manual_application(outputRoot,candidateBuild);
    waitForCom();
    result.manual_reset_driver_stop=reset_m600_manual_application(outputRoot,candidateBuild,true);
    result.manual_reset_completed=false;
   catch ex,record('manual_application_reset',ex);result.safe=false;return;end
  end
  if isfile(setupPath)
   try,restored=m600_local_short_parameter_setup(fullfile(outputRoot,'PARAMETER_RESTORE.json'),'RESTORE','', ...
     contracts.temporary_allocator_geometry,contracts.native_hover_tuning,setupPath);catch ex,record('restore_parameters',ex);end
  end
  if isfile(routePath)
   try,routeRestored=m600_local_recovery_route(fullfile(outputRoot,'NATIVE_RECOVERY_ROUTE_RESTORE.json'),'RESTORE',routePath);
   catch ex,record('restore_extra_recovery_route',ex);end
  end
  if retainManualFirmware&&startupPostVerified
   try
    post=m600_local_application_handoff(fullfile(outputRoot,'FINAL_READONLY_SAFETY.json'), ...
     'SAFE_KEEP_HIL_APPLICATION',transaction,false,false,candidateBuild);
    result.recovery_route_after_reference_boot=post.commander_recovery_route.mav_type==6 ...
     &&strcmpi(post.commander_recovery_route.raw_bits_hex,'00000000');
    result.safe=post.passed&&post.COM_closed&&(~isfile(setupPath)||(~isempty(restored)&&restored.passed)) ...
     &&(~isfile(routePath)||(~isempty(routeRestored)&&routeRestored.passed))&&result.recovery_route_after_reference_boot;
    result.manual_firmware_retained=result.safe;
    if manualReset,result.manual_reset_completed=result.safe;end
    result.installed_application_sha256=candidateSha;
    if result.safe
        if manualReset
            try,reset_m600_standby_view(outputRoot);catch viewEx,record('reset_standby_view',viewEx);end
        fprintf('USB_RC_RESET_DONE: Reset complete. Ready to restart.\n');
        end
        fprintf('USB_RC_STANDBY: Disarmed; session interfaces released. HIL application retained.\n');
    else
        fprintf(2,'USB_RC_RECOVERY: Parameter restoration is incomplete. Complete recovery before starting.\n');
    end
   catch ex,record('retained_application_safety',ex);end
  elseif applicationWasAttempted()
   try
    waitForCom();hp=fullfile(outputRoot,'RESTORE_REFERENCE_FLASH_HANDOFF.json');
    restoreGuard=m600_local_application_handoff(hp,'REBOOT_RESTORE_REFERENCE',transaction);
    assert(restoreGuard.passed,'gpenmpcShort:RestoreSafety','No blind restore flash if safe state cannot be verified.');
    result.restore_upload=uploadApplication('restore_reference',hp,'RESTORE_REFERENCE_UPLOAD');
    assert(result.restore_upload.application_crc_stage_passed,'gpenmpcShort:RestoreUpload','Original application CRC not verified.');
    result.original_firmware_restored=true;
    waitForCom();post=m600_local_application_handoff(fullfile(outputRoot,'FINAL_READONLY_SAFETY.json'),'SAFE_STOP_PX4IO',transaction);
    routeAfterBoot=post.commander_recovery_route;
    result.recovery_route_after_reference_boot=routeAfterBoot.mav_type==6&&strcmpi(routeAfterBoot.raw_bits_hex,'00000000');
    result.safe=post.passed&&post.COM_closed&&result.recovery_route_after_reference_boot ...
     &&(~isfile(routePath)||(~isempty(routeRestored)&&routeRestored.passed));
   catch ex,record('restore_application_and_safety',ex);end
  else
   % Restore temporary IMU provisioning even if upload stopped before erasing the application.
    hp=fullfile(outputRoot,'GPENMPC_FLASH_HANDOFF.json.mat');
   if isfile(hp)
    priorHandoff=load(hp,'result');
    if isfield(priorHandoff.result,'ekf_instance_write_attempts')&&priorHandoff.result.ekf_instance_write_attempts>0
     try
      post=m600_local_application_handoff(fullfile(outputRoot,'FINAL_EKF_PROVISIONING_RESTORE.json'),'SAFE_RESTORE_EKF_INSTANCE_ONLY',transaction);
      result.safe=post.passed&&post.COM_closed;
     catch ex,record('restore_preupload_ekf_provisioning',ex);end
    end
   end
  end
 end
 function yes=applicationWasAttempted()
  yes=~isempty(upload)&&isfield(upload,'application_upload_attempts')&&upload.application_upload_attempts>0;
  % Record the upload attempt immediately before invoking the uploader.
   marker=fullfile(outputRoot,'GPENMPC_FLASH_HANDOFF.json.upload_consumed');
  yes=yes||isfile(marker);
   journal=fullfile(outputRoot,'GPENMPC_UPLOAD.jsonl');
  if isfile(journal),yes=yes||contains(fileread(journal),'APPLICATION_UPLOAD_CALL_ATTEMPT');end
 end
 function yes=continueSafety()
  yes=false;if isempty(io),return,end
  landSent=~isempty(short)&&short.counts.land_requests>0;disarmSent=false;
  safetyRows={};t=tic;
  try
   while toc(t)<60&&~process.HasExited
    if requestedManualReset(),return;end
    if ~isempty(safetyOwner),s=safetyOwner.pump();
    else,s=io.snapshot();io.sendHeartbeat();b=rfly_original_getter_mex('drain');end %#ok<NASGU>
    now=io.now();fresh=isfinite(s.heartbeat_rx_s)&&isfinite(s.extended_rx_s) ...
     &&now-s.heartbeat_rx_s<=cfg.heartbeat_max_age_s&&now-s.extended_rx_s<=cfg.landed_max_age_s;
    onGround=s.model_ready&&isfield(s.model_diagnostic,'decoded') ...
     &&isfield(s.model_diagnostic.decoded,'ground_confirmed')&&s.model_diagnostic.decoded.ground_confirmed;
    safetyRows{end+1}=struct('time_s',now,'fresh',fresh,'armed',s.armed,'landed',s.landed_state,'plant_ground',onGround); %#ok<AGROW>
    if fresh&&s.armed==0&&s.landed_state==1&&onGround
     yes=true;
     if ~isempty(safetyOwner),short=safetyOwner.finishAfterSafe();yes=short.safe_ground;end
     break
    end
    if fresh&&s.armed==1
     if ~landSent,io.requestCommand(176,[1 4 6 0 0 0 0]);landSent=true;end
     if s.landed_state==1&&onGround&&~disarmSent
      io.requestCommand(400,[0 0 0 0 0 0 0]);disarmSent=true;
     end
    end
    pause(.02);
   end
  catch ex,record('continuous_native_safety',ex);end
  result.outer_safety_land_requested=landSent&&(~isempty(short)&&short.counts.land_requests==0);
  result.outer_standard_disarm_requested=disarmSent;
  save(fullfile(outputRoot,'OUTER_NATIVE_SAFETY.mat'),'safetyRows','yes','landSent','disarmSent');
 end
 function yes=requestedManualReset()
  yes=retainManualFirmware&&prepared.operator_reference&&startupPostVerified ...
   &&isfile(fullfile(outputRoot,'MANUAL_RESET_REQUEST'));
 end
 function r=uploadApplication(application,handoff,label)
  label=string(label);
  rp=fullfile(outputRoot,label+'.json');jp=fullfile(outputRoot,label+'.jsonl');
   pythonPath=fullfile(rflyRoot,'Python38','python.exe');
   assert(isfile(pythonPath),'gpenmpcShort:Python','RflySim Python was not found: %s',pythonPath);
   cmd=sprintf('"%s" -B "%s" --application %s --handoff "%s" --transaction-id %s --journal "%s" --result "%s"', ...
    pythonPath,fullfile(build,'tools','gpenmpc_application_upload_once.py'),application,handoff,transaction,jp,rp);
  [code,log]=system(cmd);writeText(fullfile(outputRoot,label+'_stdout.txt'),log);
  assert(isfile(rp),'gpenmpcShort:UploadReceipt','Uploader did not produce its atomic receipt: %s',log);
  r=jsondecode(fileread(rp));r.process_return_code=code;
 end
 function waitForCom()
  % Query fresh PnP state because serialportlist can cache a removed COM port.
  % Require two consecutive matching Present/OK observations before UID verification.
  script="$ErrorActionPreference='Stop'; [Console]::OutputEncoding=[System.Text.UTF8Encoding]::new(); " + ...
   "$d=@(Get-PnpDevice -PresentOnly -ErrorAction Stop | Where-Object {$_.FriendlyName -match '\(COM3\)$'} | " + ...
   "ForEach-Object {[ordered]@{instance_id=$_.InstanceId;friendly_name=$_.FriendlyName;status=$_.Status}}); " + ...
   "[ordered]@{observed_utc=[DateTime]::UtcNow.ToString('o');process_id=$PID;devices=$d} | ConvertTo-Json -Depth 4 -Compress";
  wait=struct('passed',false,'COM_open_count',0,'reboot_requests',0,'observations',{{}},'elapsed_s',0);
  if ~isfield(result,'pnp_reenumeration_waits'),result.pnp_reenumeration_waits={};end
  result.pnp_reenumeration_waits{end+1}=wait;waitIndex=numel(result.pnp_reenumeration_waits);
  previous='';consecutive=0;t=tic;
  while toc(t)<24 % Reserve one second for terminating only a timed-out probe.
   si=System.Diagnostics.ProcessStartInfo;si.FileName='powershell.exe';
   si.Arguments=['-NoProfile -NonInteractive -Command "' char(script) '"'];
   si.UseShellExecute=false;si.CreateNoWindow=true;
   si.RedirectStandardOutput=true;si.RedirectStandardError=true;
   si.StandardOutputEncoding=System.Text.Encoding.UTF8;si.StandardErrorEncoding=System.Text.Encoding.UTF8;
   probe=System.Diagnostics.Process;probe.StartInfo=si;dispose=onCleanup(@()probe.Dispose());
   assert(probe.Start(),'gpenmpcShort:PnpStart','Could not start the owned read-only PnP query.');
   out=probe.StandardOutput.ReadToEndAsync();err=probe.StandardError.ReadToEndAsync();
   budget=int32(max(1,floor(1000*min(8,24-toc(t)))));
   if ~probe.WaitForExit(budget)
    probe.Kill();probe.WaitForExit(int32(1000));
    wait.elapsed_s=toc(t);wait.failure='READ_ONLY_PNP_QUERY_TIMEOUT';
    result.pnp_reenumeration_waits{waitIndex}=wait;
    error('gpenmpcShort:PnpTimeout','PnP query timed out.');
   end
   observation=struct('process_id',double(probe.Id),'exit_code',double(probe.ExitCode), ...
    'stdout',char(out.Result),'stderr',char(err.Result));clear dispose
   wait.observations{end+1}=observation;wait.elapsed_s=toc(t);
   result.pnp_reenumeration_waits{waitIndex}=wait;
   assert(observation.exit_code==0,'gpenmpcShort:PnpQuery','Read-only PnP query failed: %s',observation.stderr);
   parsed=jsondecode(observation.stdout);
   assert(parsed.process_id==observation.process_id,'gpenmpcShort:PnpIdentity','PnP output process identity differs.');
   device=parsed.devices;
   valid=isstruct(device)&&isscalar(device)&&strcmp(device.status,'OK') ...
    &&~isempty(regexp(device.friendly_name,'\(COM3\)$','once'))&&strlength(string(device.instance_id))>8;
   if valid
    if strcmp(previous,device.instance_id),consecutive=consecutive+1;else,consecutive=1;end
    previous=char(device.instance_id);
   else,previous='';consecutive=0;end
   if consecutive==2&&toc(t)<=25
    wait.passed=true;wait.instance_id=previous;wait.consecutive_present_ok=2;wait.elapsed_s=toc(t);
    result.pnp_reenumeration_waits{waitIndex}=wait;return
   end
   pause(min(.25,max(0,24-toc(t))));
  end
  wait.elapsed_s=toc(t);result.pnp_reenumeration_waits{waitIndex}=wait;
  error('gpenmpcShort:COMReenumeration','Exact COM3 Present/OK identity must be observed twice within 25 seconds.');
 end
 function record(stage,ex),failures{end+1}=struct('stage',stage,'identifier',ex.identifier,'message',ex.message);end
end
function cleanup=captureOuterCleanup(owner,gate)
% Capture these handle VALUES in an independent local-function workspace.
% Neither normal return nor exception cleanup reads the exiting executor's
% nested variables. The mutable permit remains false during native safety.
cleanup=onCleanup(@()closeCapturedOuter(owner,gate));
end
function closeCapturedOuter(owner,gate)
if gate('join_allowed'),owner.close();else,owner.suspendForNativeLand();end
end
function report=validateCapturedOuterCleanup()
% Exercise cleanup callbacks with counters and no transport or worker.
counts=containers.Map({'close','suspend'},{0,0});
owner=struct('close',@()bumpCleanupCounter(counts,'close'), ...
 'suspendForNativeLand',@()bumpCleanupCounter(counts,'suspend'));
gate=containers.Map('KeyType','char','ValueType','logical');gate('join_allowed')=true;
cleanupCaptureScope(owner,gate,false,false);
normal=counts('close')==1&&counts('suspend')==0;
gate('join_allowed')=false;
try,cleanupCaptureScope(owner,gate,true,false);catch ex,assert(strcmp(ex.identifier,'gpenmpcShort:CleanupProbe'));end
denied=counts('close')==1&&counts('suspend')==1;
try,cleanupCaptureScope(owner,gate,true,true);catch ex,assert(strcmp(ex.identifier,'gpenmpcShort:CleanupProbe'));end
allowed=counts('close')==2&&counts('suspend')==1;
report=struct('scope','HOST_ONLY_CLEANUP_CAPTURE_NO_WORKER_OR_IO','passed',normal&&denied&&allowed, ...
 'prepare_return_closes',normal,'exception_unknown_safety_does_not_join',denied, ...
 'exception_uses_updated_captured_permit',allowed,'close_callbacks',counts('close'), ...
 'suspend_callbacks',counts('suspend'),'solver_calls',0,'COM_opens',0,'socket_opens',0,'NoUI_starts',0);
assert(report.passed,'gpenmpcShort:CleanupCapture','Value-captured cleanup or mutable permit failed.');
end
function cleanupCaptureScope(owner,gate,throwNow,allowAfterCapture)
cleanup=captureOuterCleanup(owner,gate); %#ok<NASGU>
if allowAfterCapture,gate('join_allowed')=true;end
if throwNow,error('gpenmpcShort:CleanupProbe','Controlled HOST-only exceptional scope exit.');end
end
function bumpCleanupCounter(counts,key),counts(key)=counts(key)+1;end
function h=textSha(s)
m=java.security.MessageDigest.getInstance('SHA-256');m.update(typecast(uint8(unicode2native(char(s),'UTF-8')),'int8'));
h=upper(reshape(dec2hex(typecast(m.digest(),'uint8'),2).',1,[]));
end
function receipt=initializeRetainedInput(build,backend,gpReplyHostMaxAgeNs,cfg,assets,preparedGp)
% Warm ENV2 encoding with retained data before the live model lease.
oldEnv=load(fullfile(fileparts(fileparts(build)),'assets','environment', ...
 'initial_environment.mat'),'initialEnvironment');
oldEnvFrame=oldEnv.initialEnvironment;
for envInitIndex=1:8
 [envInitBytes,envInitFields]=gpenmpcTaskIo.encodePlantEnvironmentV2(oldEnvFrame,1,2.21);
 assert(numel(envInitBytes)==232&&envInitFields(3)==oldEnvFrame.source_io_time_s);
end
% Warm the complete IO sender on isolated loopback ports before hardware setup.
% Discard its retained-data packets and IO instance after initialization.
initCfg=cfg;initCfg.live_enabled=true;initCfg.outer_preflight_pass=true;
initCfg.local_mavlink_port=62291;initCfg.remote_mavlink_port=62292;
initCfg.truth_port=62293;initCfg.coptersim_time_port=62294;
if isfield(initCfg,'canonical_rotor_observer'),initCfg.canonical_rotor_observer.local_port=62295;end
initCfg.delivery_environment_contract.remote_port=62296;
initCfg.mavlink_transport.scope='HOST_ONLY_LOOPBACK';
initCfg.mavlink_transport.local_port=62291;initCfg.mavlink_transport.remote_port=62292;
assert(cfg.delivery_environment_contract.remote_port~=62296);
initPeer=udpport('datagram','IPV4','LocalHost','127.0.0.1','LocalPort',62296,'Timeout',1);
initPeerCleanup=onCleanup(@()delete(initPeer));
initIo=m600check.makeM600CopterSimIo(initCfg);
initIoCleanup=onCleanup(@()initIo.close());
for envInitIndex=1:8,initIo.sendPlantEnvironment(oldEnvFrame);end
initDatagrams=read(initPeer,8,'uint8');
for envInitIndex=1:8
 assert(isequal(uint8(initDatagrams(envInitIndex).Data(:)),envInitBytes(:)));
end
if ~cfg.local_short.component_initialization
% Warm the complete GP request/reply path on the isolated loopback owner.
pairPath=fullfile(build,'rfly_vendor_integration','full_inner_abi', ...
 'snapshot_wire_fixture','RGP1_RGR1_PAIRS.bin');
pairFile=fopen(pairPath,'rb');assert(pairFile>=0);pairClose=onCleanup(@()fclose(pairFile));
pairs=reshape(fread(pairFile,596*8,'*uint8'),596,8);clear pairClose
q=gpenmpcNative.RflyLocalGpCodec.decodeRequest(pairs(1:310,1));
initMavPeer=udpport('datagram','IPV4','LocalHost','127.0.0.1','LocalPort',62292,'Timeout',1);
initMavPeerCleanup=onCleanup(@()delete(initMavPeer));
initDialect=mavlinkdialect(cfg.local_mavlink_dialect.path,2);
initBoard=mavlinkio(initDialect,'SystemID',double(q.identity.system),'ComponentID',double(q.identity.component));
initBoardCleanup=onCleanup(@()delete(initBoard));
initNow=gpenmpcNative.rflyOriginalHostMonotonicNs();
initExpected=struct('source_system',q.identity.system,'source_component',q.identity.component, ...
 'target_system',uint8(255),'target_component',uint8(190),'uid',q.identity.uid, ...
 'session_generation',q.identity.boot_generation,'link_lifecycle_generation',uint64(7), ...
 'confirmed_host_rx_ns',initNow,'execution_session_sha256',repmat('B',1,64), ...
 'configuration_sha256',hexBytes(q.configuration_sha256));
initServiceExpected=struct('uid',q.identity.uid,'boot_generation',q.identity.boot_generation, ...
 'board_system',q.identity.system,'board_component',q.identity.component, ...
 'host_system',uint8(255),'host_component',uint8(190),'link_lifecycle_generation',uint64(7), ...
 'confirmed_host_rx_ns',initNow,'execution_session_sha256',repmat('B',1,64),'gp_backend',preparedGp.binding);
initGp=gpenmpcNative.RflyLocalGpService(assets,initServiceExpected,preparedGp);
initGpCleanup=onCleanup(@()initGp.close());initIo.bindCanonicalSession(initExpected);
if initGp.ReceiveInlineEnabled
 % Warm the selected continuous receive path.
 initIo.attachCanonicalInlineGp(initGp,true,cfg.local_short.service_cfg.source_max_age_ns);
end
gpPathSeconds=nan(8,2);gpInitExpired=0;gpInitSendExpired=0;gpInitReturned=0;
gpInitRetired=false(8,1);gpEarlierReadbacks=[];
for gpInitIndex=1:8
 body=pairs(1:310,gpInitIndex);q=gpenmpcNative.RflyLocalGpCodec.decodeRequest(body);
 initQueryWires=cell(1,3);
 for part=1:3
  message=createmsg(initDialect,'TUNNEL');message.Payload.target_system=uint8(255);
  message.Payload.target_component=uint8(190);message.Payload.payload_type=uint16(42002);
  count=min(119,310-119*(part-1));message.Payload.payload_length=uint8(9+count);
  message.Payload.payload(1)=uint8(128+part-1);
  message.Payload.payload(2:9)=reshape(typecast(swapbytes(q.output_generation),'uint8'),[],1);
  message.Payload.payload(10:9+count)=body(119*(part-1)+1:119*(part-1)+count);
  initQueryWires{part}=uint8(serializemsg(initBoard,message));
 end
 % Encode fixture fragments before starting their receive interval.
 for part=1:3
  write(initMavPeer,initQueryWires{part},'uint8','127.0.0.1',62291);
 end
 initWait=tic;item=[];
 if initGp.ReceiveContinuousEnabled
  % The native worker predicts and transmits independently of MATLAB.
  % Wait for the peer reply without transmitting a duplicate.
  while toc(initWait)<1
   initStatus=initIo.pollCanonical(true);
   assert(isempty(initStatus.failure),'gpenmpcShort:GpInitializationRx','%s',initStatus.failure);
   if initMavPeer.NumDatagramsAvailable>=3,break,end
  end
  initNative=gpenmpc_rfly_udp_transport_mex('status');
  assert(initMavPeer.NumDatagramsAvailable==3&&~initNative.failed ...
   &&initNative.continuous_gp.replies==gpInitIndex, ...
   'gpenmpcShort:GpInitializationRx','Selected continuous GP did not reply in HOST preparation.');
  computed=initNative.continuous_gp.last_result;
  gpPathSeconds(gpInitIndex,1)=toc(initWait);
  gpPathSeconds(gpInitIndex,2)=0; % The native path sent the reply.
  expectedGp=gpenmpcNative.RflyLocalGpCodec.decodeReply(pairs(311:596,gpInitIndex));
  assert(isequal(computed.reply_bytes(1:110),pairs(311:420,gpInitIndex)) ...
   &&max(abs(computed.result18(:)-expectedGp.result18))<=1e-10 ...
   &&computed.result18(15)==expectedGp.result18(15));
  returned=read(initMavPeer,3,'uint8');joined=zeros(286,1,'uint8');
  for part=1:3
   decoded=deserializemsg(initDialect,uint8(returned(part).Data(:)));p=decoded.Payload;count=double(p.payload_length)-9;
   assert(decoded.MsgID==385&&p.payload_type==42002&&p.payload(1)==uint8(144+part-1));
   joined(119*(part-1)+1:119*(part-1)+count)=p.payload(10:9+count);
  end
  assert(isequal(joined,computed.reply_bytes),'gpenmpcShort:GpInitializationReturn','Original reply bytes differ.');
  assert(isempty(initIo.takeCanonical('gp_request',false)),'gpenmpcShort:GpInitializationRx','No duplicate GP request consumer.');
  gpInitReturned=gpInitReturned+1;if gpInitReturned==4,break;end
  continue
 end
 while isempty(item)&&toc(initWait)<1
  initStatus=initIo.pollCanonical(true);item=initIo.takeCanonical('gp_request',false);
  if initStatus.status.messages_completed(1)>=gpInitIndex,break;end
 end
 if isempty(item)
  % Retire expired fixture requests and try the next retained request.
  % Each request retains its original timestamps; at most eight are attempted.
  assert(initStatus.status.messages_completed(1)==gpInitIndex&& ...
   initStatus.status.ignored_nonprivate>gpInitExpired,'gpenmpcShort:GpInitializationRx', ...
   'Loopback request %d not delivered: %s',gpInitIndex,jsonencode(initStatus));
  gpInitExpired=gpInitExpired+1;gpInitRetired(gpInitIndex)=true;continue
 end
 assert(isequal(item.message,body),'gpenmpcShort:GpInitializationRx','Original bytes changed.');
 mark=tic;
 if initGp.ReceiveInlineEnabled
  assert(isfield(item,'inline_gp')&&~isempty(item.inline_gp),'gpenmpcShort:GpInitializationInline');
  computed=item.inline_gp.computed;
 elseif initGp.AsyncEnabled
  % Warm the selected asynchronous begin/poll/fetch/encode path.
  initGp.begin(item.message,item.original_host_receive_ns, ...
   gpenmpcNative.rflyOriginalHostMonotonicNs(),item.origin);
  computed=[];
  while isempty(computed)&&toc(mark)<5
   computed=initGp.poll();
   if isempty(computed),pause(.001);end
  end
  assert(~isempty(computed)&&~initGp.isPending(),'gpenmpcShort:GpInitializationAsync', ...
   'Selected asynchronous HOST preparation timed out.');
 else
  computed=initGp.process(item.message,item.original_host_receive_ns, ...
   gpenmpcNative.rflyOriginalHostMonotonicNs(),item.origin);
 end
 gpPathSeconds(gpInitIndex,1)=toc(mark);
 mark=tic;sent=initIo.sendCanonicalLocalGp(computed.reply_bytes,item);gpPathSeconds(gpInitIndex,2)=toc(mark);
 expectedGp=gpenmpcNative.RflyLocalGpCodec.decodeReply(pairs(311:596,gpInitIndex));
 assert(isequal(computed.reply_bytes(1:110),pairs(311:420,gpInitIndex)) ...
  &&max(abs(computed.result18(:)-expectedGp.result18))<=1e-10 ...
  &&computed.result18(15)==expectedGp.result18(15));
 if sent.messages_send_returned==0
  assert(isfield(sent,'status')&&strcmp(sent.status,'EXPIRED_UNSENT_GP_NOT_USED')&&all(sent.returned_ns==0), ...
   'gpenmpcShort:GpInitializationSend','Only an actually unsent, expired fixture reply may retire.');
  gpInitSendExpired=gpInitSendExpired+1;continue
 end
 assert(sent.messages_send_returned==3);
 matched=false;
 for originalReply=1:gpInitIndex
  returned=read(initMavPeer,3,'uint8');joined=zeros(286,1,'uint8');
  for part=1:3
   decoded=deserializemsg(initDialect,uint8(returned(part).Data(:)));p=decoded.Payload;count=double(p.payload_length)-9;
   assert(decoded.MsgID==385&&p.payload_type==42002&&p.payload(1)==uint8(144+part-1));
   joined(119*(part-1)+1:119*(part-1)+count)=p.payload(10:9+count);
  end
  if isequal(joined,computed.reply_bytes),matched=true;break,end
  % Accept an earlier inline reply only when it exactly matches a retired fixture.
  previous=find(all(pairs(311:596,:)==joined,1));
  assert(initGp.ReceiveInlineEnabled&&isscalar(previous)&&previous<gpInitIndex ...
   &&gpInitRetired(previous)&&~ismember(previous,gpEarlierReadbacks), ...
   'gpenmpcShort:GpInitializationReturn','Reply is neither current nor an exact earlier retired fixture.');
  gpEarlierReadbacks(end+1)=previous; %#ok<AGROW>
  fprintf('HOST preparation retained early reply for retired fixture %d before current %d.\n',previous,gpInitIndex);
 end
 assert(matched,'gpenmpcShort:GpInitializationReturn','Matching current loopback reply was not received.');
 gpInitReturned=gpInitReturned+1;if gpInitReturned==4,break;end
end
% Synchronize native counters at stop and retain each reply.
initGp.close();
assert(gpInitReturned==4&&initGp.Calls==initGp.Completed);
gpInitCalls=double(initGp.Calls);gpSelectedContinuous=initGp.ReceiveContinuousEnabled;
clear initGpCleanup initGp initBoardCleanup initBoard initMavPeerCleanup initMavPeer
end
initIo.close();clear initIoCleanup initIo initPeerCleanup initPeer
clear oldEnv oldEnvFrame envInitIndex envInitBytes envInitFields initCfg initDatagrams
fixture=fullfile(build,'runtime_assets','receiver','RETAINED_SENT_INPUT.mat');
if isfile(fixture)
 old=load(fixture,'item','bytes','expected','sendNs');
else
 parent=fullfile(gpenmpc_external_path('bound_task_input_preparation'),'SHORT_HIL','RAW_BOARD_LOCAL_SHORT_HIL.mat');
 raw=load(parent,'methodRaw','rawIo');
 ix=find(cellfun(@(x)isfield(x,'input_send')&&~isempty(x.input_send) ...
     &&x.input_send.messages_send_returned==6,raw.methodRaw),1);
 assert(~isempty(ix),'gpenmpcShort:RetainedInput','A real successful six-part input is required.');
 item=raw.methodRaw{ix}.source;generation=raw.methodRaw{ix}.input_send.binding.original_source_generation;
 rows=raw.rawIo.raw_transmit_messages;
 use=cellfun(@(x)isfield(x,'canonical_local_task_inputs')&&x.canonical_local_task_inputs ...
     &&isfield(x,'original_source_validity')&&x.original_source_validity.original_source_generation==generation,rows);
 rows=rows(use);assert(numel(rows)==6&&all(cellfun(@(x)x.send_returned,rows)));
 bytes=zeros(647,1,'uint8');
 for j=1:6
  p=rows{j}.message.Payload;n=min(119,647-(j-1)*119);
  assert(p.payload(1)==uint8(208+j-1));bytes((j-1)*119+1:(j-1)*119+n)=p.payload(10:9+n);
 end
 expected=raw.rawIo.canonical_exchange_expected;sendNs=rows{1}.original_host_submit_ns;
 save(fixture,'item','bytes','expected','sendNs');
 old=struct('item',item,'bytes',bytes,'expected',expected,'sendNs',sendNs);clear raw rows
end
u=gpenmpcNative.RflyLocalTaskCodec.decode(old.bytes);
bound=struct('schema','RFLY_LOCAL_ORIGINAL_INPUT_BINDING_V1','numerical_inputs_complete',true, ...
 'control_authority',false,'source',u.source,'rotor',u.rotor,'payload',u.payload,'wind',u.wind,'leg_index',u.leg_index);
reg=struct('identity',u.source.identity,'leg_index',u.leg_index,'task_sha256',hexBytes(u.task_sha256), ...
 'configuration_sha256',hexBytes(u.configuration_sha256),'reference_asset_sha256',hexBytes(u.reference_asset_sha256), ...
 'execution_session_sha256',hexBytes(u.execution_session_sha256));
elapsed=zeros(3,2);
for j=1:3
 t=tic;encoded=backend.adapter(bound,old.item.message,old.item.original_host_receive_ns,u.outer,reg,backend.encoder);
 elapsed(j,1)=toc(t);assert(isequal(encoded,old.bytes),'gpenmpcShort:RetainedInput','Original sent bytes changed.');
 t=tic;[payloads,binding]=gpenmpcNative.validateLocalTaskInputForSend(encoded,old.item,old.expected,uint64(50000000),old.sendNs);
 elapsed(j,2)=toc(t);assert(numel(payloads)==6&&binding.original_source_generation==u.source.source_generation);
end
rejected=false;
try
 gpenmpcNative.validateLocalTaskInputForSend(encoded,old.item,old.expected,uint64(50000000), ...
     old.item.original_host_receive_ns+uint64(50000001));
catch ex,rejected=strcmp(ex.identifier,'gpenmpcNative:LocalTaskSendExpired');end
assert(rejected,'gpenmpcShort:RetainedExpiry','Actual expired input must still be rejected.');
receipt=struct('fixture_kind','LOCAL_INPUT_CONTINUITY','source_generation',u.source.source_generation, ...
 'elapsed_encode_validate_s',elapsed,'byte_exact',true,'expired_rejected',rejected, ...
 'board_actions',0,'live_authority',false,'live_timestamps_renewed',false);
if cfg.local_short.component_initialization
 receipt.initialization_gp_calls=0;
 receipt.gp_initialization_scope='NOT_SELECTED_IN_SE3_COMPONENT_MODE';
 return
end
% Warm GP reply parsing and send validation before starting the simulator.
gp=load(fullfile(build,'runtime_assets','gp_reply','RETAINED_GP_REPLY.mat'));
for j=1:3
 [gpPayloads,~]=gpenmpcNative.validateLocalGpReplyForSend(gp.reply,gp.item,gp.expected,gpReplyHostMaxAgeNs,gp.sendNs);
 assert(numel(gpPayloads)==3);
end
rejected=false;
try
 gpenmpcNative.validateLocalGpReplyForSend(gp.reply,gp.item,gp.expected,gpReplyHostMaxAgeNs, ...
     gp.item.original_host_receive_ns+gpReplyHostMaxAgeNs+uint64(1));
catch ex,rejected=strcmp(ex.identifier,'gpenmpcNative:LocalGpSendExpired');end
assert(rejected,'gpenmpcShort:RetainedGpExpiry','Actual expired GP reply must still be rejected.');
receipt.retained_gp_parse_send_initialized=true;
receipt.retained_gp_expired_rejected=true;receipt.initialization_gp_calls=gpInitCalls;
receipt.gp_same_io_initialization_process_send_s=gpPathSeconds(1:gpInitIndex,:);
receipt.gp_same_io_initialization_expired_not_used=gpInitExpired;
receipt.gp_same_io_initialization_unsent_expired=gpInitSendExpired;
receipt.gp_same_io_initialization_returned=gpInitReturned;
receipt.gp_same_io_earlier_retired_reply_readbacks=gpEarlierReadbacks;
receipt.gp_same_io_initialization_scope='HOST_ONLY_RETAINED_BYTES_ACTUAL_LOOPBACK_NO_LIVE_AUTHORITY';
receipt.gp_selected_continuous_history_initialized=gpSelectedContinuous;
[~,boundary]=gpenmpcNative.validateLocalGpReplyForSend(gp.reply,gp.item,gp.expected,gpReplyHostMaxAgeNs, ...
 gp.item.original_host_receive_ns+gpReplyHostMaxAgeNs);
assert(boundary.original_host_receive_ns==gp.item.original_host_receive_ns&& ...
 boundary.valid_until_host_ns==gp.item.original_host_receive_ns+gpReplyHostMaxAgeNs&& ...
 ~boundary.control_authority&&~boundary.control_publication_expiry_used);
receipt.gp_reply_host_max_age_ns=gpReplyHostMaxAgeNs;
receipt.gp_transport_boundary_no_numerical_authority=true;
end
function text=hexBytes(bytes),text=upper(reshape(dec2hex(bytes,2).',1,[]));end
function z=terrainAtOrigin(mapRoot)
% Vendor LoadPngData.m calibration + getTerrainAltData.m interpolation.
% Unreal calibration is cm/up; the sole plant consumes metres/NED-down.
p=readmatrix(fullfile(mapRoot,'3DDisplay.txt'));p=p(:);assert(numel(p)==9&&all(isfinite(p)));
h=double(imread(fullfile(mapRoot,'3DDisplay.png')))-32768;
[nr,nc]=size(h);sx=(p(1)-p(4))/(nc-1);sy=(p(2)-p(5))/(nr-1);
col=double(int32((p(7)-p(4))/sx+1));row=double(int32((p(8)-p(5))/sy+1));
h0=h(1,1);h1=h(end,end);h2=h(row,col);
if abs(h2-h1)<=abs(h2-h0)
 assert(abs(h0-h2)>10);sz=(p(6)-p(9))/(h0-h2);
else
 assert(abs(h2-h1)>10);sz=(p(3)-p(9))/(h1-h2);
end
pixel=interp2(h,(0-p(4))/sx+1,(0-p(5))/sy+1,'linear');
z=-(p(6)+(pixel-h0)*sz)/100;assert(isfinite(z));
end
function writeJson(p,value)
assert(~isfile(p),'gpenmpcShort:NoOverwrite','Choose a new execution-receipt path.');writeText(p,jsonencode(value,PrettyPrint=true));
end
function restoreHostPriority(process,priority)
process.PriorityClass=priority;
end
function writeText(p,s)
f=fopen(p,'w','n','UTF-8');assert(f>=0);c=onCleanup(@()fclose(f));fprintf(f,'%s\n',s); %#ok<NASGU>
end
