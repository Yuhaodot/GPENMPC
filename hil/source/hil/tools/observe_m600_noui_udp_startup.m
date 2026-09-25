function [result,owner]=observe_m600_noui_udp_startup(outputRoot,restore_referenceReceiptPath)
% Observe NoUI startup without issuing parameter, flight or firmware commands.
% The parent owns safety shutdown through observe_stop_owned_noui.
assert(nargout==2,'gpenmpcObserve:RetainOwner','Retain both result and owner outputs for safety recovery.');
build=string(fileparts(fileparts(mfilename('fullpath'))));outputRoot=string(outputRoot);
assert(isfolder(outputRoot),'gpenmpcObserve:ExistingRoot','Use the existing startup diagnostic directory.');
resultPath=fullfile(outputRoot,'STARTUP_OBSERVATION_RESULT.json');
rawPath=fullfile(outputRoot,'STARTUP_OBSERVATION_RAW.mat');
assert(~isfile(resultPath)&&~isfile(rawPath),'gpenmpcObserve:ExistingEvidence','Use a new observation directory.');
addpath(fullfile(build,'tools'),fullfile(build,'host_runtime'),fullfile(build,'matlab_validation'),fullfile(build,'m600_coptersim','matlab_validation'));
addpath(gpenmpc_external_path('native_visual_host_source'),'-end');
assert(~isempty(which('observe_stop_owned_noui')),'gpenmpcObserve:SafetyStopMissing','Parent-owned fresh UDP safety/stop entry must exist before starting.');
r=jsondecode(fileread(restore_referenceReceiptPath));
% Normalize the application label from retained upload receipts.
if strcmp(r.application,'restore005'),r.application='restore_reference';end
assert(strcmp(r.application,'restore_reference')&&r.application_crc_stage_passed&&r.port_closed ...
    &&strcmpi(r.image_sha256,'7722616157AF96E3493D1827F01A0713D92373946C669B854B8F043552892FC7') ...
    &&strcmp(r.observed_bootloader_sn,gpenmpc_device_identity('bootloader_sn_display'))&&r.observed_board_id==56, ...
    'gpenmpcObserve:RestoreIdentity','Reference application actual upload/CRC and same-board receipt required.');
asset=gpenmpcNative.loadCanonicalAssets();
prePath=fullfile(outputRoot,'OBSERVATION_READONLY_PREFLIGHT.json');
cfg=make_m600_board_local_short_config('M600_CANONICAL_DIAGNOSTIC_SAME_6DOF_MODEL',prePath,asset);
[contracts,~]=load_m600_recovery_contracts();
cfg.temporary_allocator_geometry=contracts.temporary_allocator_geometry;cfg.native_hover_tuning=contracts.native_hover_tuning;
getterSource=struct('exact_path',char(fullfile(gpenmpc_external_path('local_original_getter_mex'),'rfly_original_getter_mex.mexw64')), ...
    'sha256','FD876F30B4337CC3AAD10B1D2531CF5388020401963EA39422328E9C430D3433');
addpath(fileparts(getterSource.exact_path),fileparts(cfg.mavlink_transport.source.exact_path));
task=struct('path',fullfile(build,'task_packages','cambridge_canonical','MU_CAMBRIDGE_MA_02__CANONICAL_PHYSICAL_TASK.mat'), ...
    'sha256','B876546B468265F84BD028AC549B06EF8ADDCB25CBA09CF389A08F4B21290A5F', ...
    'configuration_sha256',char(asset.binding.effective_configuration_payload_sha256), ...
    'environment_policy',rmfield(cfg.delivery_environment_contract,{'initial_payload_kg','remote_port'}),'copter_id',1);
task.cached_asset=gpenmpcNative.RflyLocalTaskAsset(task);
d=mavlinkdialect(cfg.local_mavlink_dialect.path,2);
resources=struct('task_source',task,'getter_source',getterSource,'decoder_dialect',d);
pure=prepare_m600_board_local_short_objects(cfg,resources); %#ok<NASGU>
validation=m600check.makeM600CopterSimIo(cfg,'VALIDATE_ONLY');assert(validation.passed);
dll=fullfile(build,'tools','NoInheritProcess.dll');
assert(strcmpi(m600check.fileSha256(dll),'16595B9ED1B00481994D4781DA72251CB9DAD6E4DFAF99520F681B280C1B34DF'));
NET.addAssembly(char(dll));
runtime=fullfile(outputRoot,'runtime');
if ~isfolder(runtime),[ok,msg]=copyfile(fullfile(gpenmpc_external_path('official_noui_step_snapshot'),'runtime'),runtime);assert(ok,'%s',msg);end
exe=fullfile(runtime,'CopterSimNoUI.exe');
assert(strcmpi(m600check.fileSha256(exe),'94B81EFB44058176DD5353669D9C28FC5331CC8411AB9EA3F2D27C1E8C343241'));
assert(strcmpi(m600check.fileSha256(fullfile(runtime,'external','model','GPENMPC_M600_Diagnostic.dll')), ...
    '990850A2F40F3FCC2A6C47E63A4065B60FF49AA39CC4749FF443963B06F2EF7E'));
result=struct('schema','REFERENCE_APPLICATION_ACTUAL_NOUI_STARTUP_OBSERVATION_V1','entered',false,'diagnostic_window_s',10, ...
    'elapsed_s',0,'safe',false,'owned_pid',0,'heartbeat_attempts',0,'snapshot_calls',0,'no_ui_launch_attempts',0, ...
    'parameter_writes',0,'mapping_writes',0,'application_flash',0,'reboot',0,'arm',0,'disarm',0,'mode',0, ...
    'session',0,'module_commands',0,'control_commits',0,'environment_step_calls',0,'scientific_rows',0, ...
    'restore_reference_receipt',char(restore_referenceReceiptPath),'restore_reference_receipt_sha256',m600check.fileSha256(restore_referenceReceiptPath));
io=[];reader=[];process=[];objects=[];owner=[];snapshots={};getterRaw={};firstFailure=[];lastSnapshot=[];rawIo=[];pre=[];
[~,transaction]=fileparts(outputRoot);failures={};finalized=false;
guard=onCleanup(@finish);
try
    for name={'CopterSim','CopterSimNoUI','QGroundControl'}
        assert(System.Diagnostics.Process.GetProcessesByName(name{1}).Length==0,'gpenmpcObserve:ExistingOwner','Existing simulator/QGC must be released.');
    end
    pre=m600_local_application_handoff(prePath,'READONLY',char(transaction));
    assert(pre.passed&&pre.COM_closed&&pre.disarmed&&pre.landed&&isempty(pre.semantic_changed) ...
        &&pre.parameter_writes==0&&pre.driver_stop_attempts==0&&pre.all_modules_stopped ...
        &&pre.commander_recovery_route.mav_type==6&&strcmp(pre.commander_recovery_route.raw_bits_hex,'00000000'), ...
        'gpenmpcObserve:FreshRecoverySafety','Fresh reference parameters, zero output mapping, disabled recovery route and stopped drivers required.');
    cfg.live_enabled=true;cfg.outer_preflight_pass=true;
    io=m600check.makeM600CopterSimIo(cfg);
    objects=prepare_m600_board_local_short_objects(cfg,resources,io);
    reader=rfly_original_getter_mex('open');
    si=System.Diagnostics.ProcessStartInfo;si.FileName=char(exe);si.WorkingDirectory=char(runtime);
    si.Arguments='1 1 -1 GPENMPC_M600_Diagnostic 0 LowGPU 0 0 0 0 3:921600 2 0';
    si.UseShellExecute=false;si.CreateNoWindow=true;vars=si.EnvironmentVariables;
    vars.Remove('GPENMPC_CLOCK_RING_SECTION');vars.Add('GPENMPC_CLOCK_RING_SECTION',reader.ring_name);
    vars.Remove('GPENMPC_STEP_SNAPSHOT_SECTION');vars.Add('GPENMPC_STEP_SNAPSHOT_SECTION',reader.status_name);
    for key={'OMP_NUM_THREADS','MKL_NUM_THREADS','OPENBLAS_NUM_THREADS'},vars.Remove(key{1});vars.Add(key{1},'1');end
    result.no_ui_launch_attempts=1;
    process=GPENMPC.HostDiagnostics.NoInheritProcess.Start(si);result.owned_pid=double(process.Id);result.entered=true;
    owner=struct('process',process,'owned_pid',result.owned_pid,'io',io,'getter',reader,'objects',objects);
    timer=tic;lastHb=-Inf;
    while toc(timer)<10
        assert(~process.HasExited,'gpenmpcObserve:PeerExited','Owned NoUI exited during observation.');
        result.snapshot_calls=result.snapshot_calls+1;lastSnapshot=io.snapshot();
        native=gpenmpc_rfly_udp_transport_mex('status');
        if native.failed||~isempty(lastSnapshot.fatal)
            retainFirst('SNAPSHOT_OR_NATIVE_RECEIVE',native,[]);break
        end
        if lastSnapshot.automatic_mavlink_peer_ready&&io.now()-lastHb>=cfg.local_short.heartbeat_period_s
            result.heartbeat_attempts=result.heartbeat_attempts+1;io.sendHeartbeat();lastHb=io.now();
        end
        % Drain the bounded delivery queue during observation; raw bytes remain in IO evidence.
        io.takeCanonicalEnvironmentRecords();
        b=rfly_original_getter_mex('drain');
        if size(b.records,2)>0||b.failed,getterRaw{end+1}=b;end %#ok<AGROW>
        snapshots{end+1}=struct('elapsed_s',toc(timer),'io_s',lastSnapshot.now_s, ...
            'armed',lastSnapshot.armed,'landed_state',lastSnapshot.landed_state, ...
            'received',native.received_count,'sent',native.sent_count); %#ok<AGROW>
        if b.failed,retainFirst('GETTER_FAILURE',native,[]);break,end
        io.sleep(cfg.local_short.poll_period_s);
    end
    result.elapsed_s=toc(timer);
catch ex
    failures{end+1}=struct('identifier',ex.identifier,'message',ex.message,'stack',ex.stack);
    if ~isempty(io),try,native=gpenmpc_rfly_udp_transport_mex('status');retainFirst('THROWN_EXCEPTION',native,ex);catch,end,end
end
finish();clear guard
writeJson(resultPath,result);
    function retainFirst(stage,native,exception)
        if ~isempty(firstFailure),return,end
        firstFailure=struct('stage',stage,'native',native,'snapshot',lastSnapshot,'exception',exception);
        if exist('timer','var'),result.elapsed_s=toc(timer);end
        rawIo=io.evidence();
        save(fullfile(outputRoot,'FIRST_STARTUP_FAILURE_RAW.mat'),'firstFailure','rawIo','getterRaw','snapshots','result','-v7.3');
    end
    function finish()
        if finalized,return,end;finalized=true;
        if ~isempty(io),try,rawIo=io.evidence();catch ex,failures{end+1}=struct('identifier',ex.identifier,'message',ex.message);end,end
        try,save(rawPath,'result','firstFailure','rawIo','getterRaw','snapshots','lastSnapshot','pre','cfg','failures','-v7.3');
        catch ex,failures{end+1}=struct('identifier',ex.identifier,'message',ex.message);end
        released=true;
        if ~isempty(io),try,released=io.close();catch,released=false;end,end
        if ~isempty(reader),try,closedGetter=rfly_original_getter_mex('close');catch,released=false;end,end %#ok<NASGU>
        result.observer_handles_released=logical(released);
        if ~isempty(process)
            owner=struct('process',process,'owned_pid',result.owned_pid,'io',io,'getter',reader,'objects',objects);
            if released
                try
                    stop=observe_stop_owned_noui(outputRoot,result.owned_pid,exe);result.safety_stop=stop;
                    assert(stop.safe_observation&&stop.owned_NoUI_stopped&&process.HasExited,'gpenmpcObserve:SafetyStopNotProven','Verified safety and owned NoUI exit are required.');
                    post=m600_local_application_handoff(fullfile(outputRoot,'OBSERVATION_READONLY_POSTFLIGHT.json'),'READONLY',char(transaction));
                    result.postflight=post;result.safe=post.passed&&post.COM_closed&&post.disarmed&&post.landed ...
                        &&post.selected_count==167&&isempty(post.semantic_changed)&&post.all_modules_stopped ...
                        &&post.virtual_path_disabled&&post.physical_path_disabled&&post.pwm_out_stopped ...
                        &&~post.output_observation.any_nonzero_or_nonfinite ...
                        &&post.commander_recovery_route.mav_type==6&&strcmp(post.commander_recovery_route.raw_bits_hex,'00000000');
                    assert(result.safe,'gpenmpcObserve:FinalSafety','Final reference application and zero-output safety were not confirmed.');
                    process.Dispose();owner=[];
                catch ex,failures{end+1}=struct('identifier',ex.identifier,'message',ex.message);end
            end
        end
        result.failure_count=numel(failures);result.failures=failures;
        result.retained_owned_process=~isempty(owner);result.transport_failure_observed=~isempty(firstFailure);
        result.claim='Startup observation.';
    end
end
function writeJson(path,value)
assert(~isfile(path),'gpenmpcObserve:ExistingResult','Choose an unused output path.');
f=fopen(path,'w');assert(f>=0,'gpenmpcObserve:WriteResult','Cannot open new result.');
c=onCleanup(@()fclose(f));fwrite(f,unicode2native(jsonencode(value,PrettyPrint=true),'UTF-8'),'uint8'); %#ok<NASGU>
end
