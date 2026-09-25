function report=test_rfly_udp_lifecycle_startup()
% Compare constructor startup lifecycles.
build=fileparts(fileparts(mfilename('fullpath')));
target=fullfile(build,'tools','UDP_LIFECYCLE_STARTUP_HOST_ONLY_FIXED.mat');
assert(~isfile(target),'gpenmpcHost:ExistingEvidence','Choose an unused output path.');
addpath(fullfile(build,'tools'),fullfile(build,'host_runtime'),fullfile(build,'matlab_validation'), ...
    fullfile(build,'m600_coptersim','matlab_validation'));
addpath(gpenmpc_external_path('native_visual_host_source'),'-end');
assets=gpenmpcNative.loadCanonicalAssets();
cfg=make_m600_board_local_short_config('M600_CANONICAL_DIAGNOSTIC_SAME_6DOF_MODEL','HOST_ONLY_NO_BOARD_RECEIPT',assets);
[contracts,~]=load_m600_recovery_contracts();
cfg.temporary_allocator_geometry=contracts.temporary_allocator_geometry;
cfg.native_hover_tuning=contracts.native_hover_tuning;
% Remap host endpoints and allow constructor opening for the diagnostic fixture.
cfg.local_mavlink_port=62291;cfg.remote_mavlink_port=62292;
cfg.truth_port=62293;cfg.coptersim_time_port=62294;
if isfield(cfg,'canonical_rotor_observer'),cfg.canonical_rotor_observer.local_port=62295;end
cfg.delivery_environment_contract.remote_port=62296;
cfg.mavlink_transport.scope='HOST_ONLY_LOOPBACK';
cfg.mavlink_transport.local_port=62291;cfg.mavlink_transport.remote_port=62292;
cfg.live_enabled=true;cfg.outer_preflight_pass=true;
getterSource=struct('exact_path',fullfile(gpenmpc_external_path('local_original_getter_mex'),'rfly_original_getter_mex.mexw64'), ...
    'sha256','FD876F30B4337CC3AAD10B1D2531CF5388020401963EA39422328E9C430D3433');
addpath(fileparts(getterSource.exact_path),fileparts(cfg.mavlink_transport.source.exact_path));
task=struct('path',fullfile(build,'task_packages','cambridge_canonical','MU_CAMBRIDGE_MA_02__CANONICAL_PHYSICAL_TASK.mat'), ...
    'sha256','B876546B468265F84BD028AC549B06EF8ADDCB25CBA09CF389A08F4B21290A5F', ...
    'configuration_sha256',char(assets.binding.effective_configuration_payload_sha256), ...
    'environment_policy',rmfield(cfg.delivery_environment_contract,{'initial_payload_kg','remote_port'}),'copter_id',1);
task.cached_asset=gpenmpcNative.RflyLocalTaskAsset(task);
d=mavlinkdialect(fullfile(build,'m600_coptersim','matlab_validation','+m600check','px4_health_events.xml'),2);
resources=struct('task_source',task,'getter_source',getterSource,'decoder_dialect',d);
pure=prepare_m600_board_local_short_objects(cfg,resources);
NET.addAssembly(char(fullfile(build,'tools','NoInheritProcess.dll')));
io=[];peer=[];remote=[];reader=[];child=[];objects=[];
raw=struct('stage',{},'native',{},'snapshot',{});failure=struct();
guard=onCleanup(@finish); %#ok<NASGU>
try
    peer=udpport('datagram','IPV4','LocalHost','127.0.0.1','LocalPort',62292);
    remote=mavlinkio(d,'SystemID',uint8(1),'ComponentID',uint8(1));
    io=m600check.makeM600CopterSimIo(cfg);
    witness('AFTER_ACTUAL_IO_WITH_ORIGINAL_OPTIONAL_SOCKET_SET');
    objects=prepare_m600_board_local_short_objects(cfg,resources,io);
    witness('AFTER_ACTUAL_ENVIRONMENT_OBJECT_CONSTRUCTION');
    reader=rfly_original_getter_mex('open');
    witness('AFTER_ORIGINAL_GETTER_OPEN');
    si=System.Diagnostics.ProcessStartInfo;
    si.FileName=fullfile(getenv('SystemRoot'),'System32','WindowsPowerShell','v1.0','powershell.exe');
    si.Arguments='-NoProfile -NonInteractive -Command "Start-Sleep -Seconds 3"';
    si.WorkingDirectory=build;si.UseShellExecute=false;si.CreateNoWindow=true;
    vars=si.EnvironmentVariables;
    vars.Remove('GPENMPC_CLOCK_RING_SECTION');vars.Add('GPENMPC_CLOCK_RING_SECTION',reader.ring_name);
    vars.Remove('GPENMPC_STEP_SNAPSHOT_SECTION');vars.Add('GPENMPC_STEP_SNAPSHOT_SECTION',reader.status_name);
    for name={'OMP_NUM_THREADS','MKL_NUM_THREADS','OPENBLAS_NUM_THREADS'},vars.Remove(name{1});vars.Add(name{1},'1');end
    child=GPENMPC.HostDiagnostics.NoInheritProcess.Start(si);
    witness('AFTER_SAME_NO_INHERIT_LAUNCH_HARMLESS_CHILD');
    assert(child.WaitForExit(10000),'gpenmpcHost:ChildDeadline','Owned waiting fixture did not terminate.');
    childExit=child.ExitCode;
    witness('AFTER_HARMLESS_CHILD_EXIT');
    pass=all(arrayfun(@(r)~r.native.failed&&r.native.open&&r.native.received_count>=1,raw))&&childExit==0;
    report=struct('scope','HOST_ONLY_EXACT_CONSTRUCTOR_LIFECYCLE_DIFFERENTIAL', ...
        'pass',pass,'witness_points',numel(raw),'child_exit',childExit, ...
        'production_mex_sha256',cfg.mavlink_transport.source.sha256, ...
        'ports',62291:62296,'getter_open',true,'real_environment_constructor',~isempty(objects.environment_service), ...
        'COM',0,'CopterSim',0,'HIL',0,'board',0,'control_commands',0, ...
        'environment_step_calls',0);
    clear guard
    save(target,'report','raw','pure','cfg','failure');disp(jsonencode(report));
catch ex
    failure=struct('identifier',ex.identifier,'message',ex.message,'stack',ex.stack);
    try,lastNative=gpenmpc_rfly_udp_transport_mex('status');catch,lastNative=struct();end
    clear guard
    save(target,'raw','cfg','failure','lastNative');rethrow(ex)
end
    function witness(label)
        message=createmsg(d,'HEARTBEAT');message.Payload.mavlink_version=uint8(3);
        bytes=serializemsg(remote,message);
        write(peer,bytes,'uint8','127.0.0.1',62291);
        pause(.02);s=io.snapshot();native=gpenmpc_rfly_udp_transport_mex('status');
        raw(end+1)=struct('stage',label,'native',native,'snapshot',s);
        assert(~native.failed,'gpenmpcHost:LifecycleSocketFailure','Native fault after %s.',label);
    end
    function finish()
        if ~isempty(child),try,if ~child.HasExited,child.Kill();child.WaitForExit(2000);end;child.Dispose();catch,end;child=[];end
        if ~isempty(io),try,io.close();catch,end;io=[];end
        if ~isempty(reader),try,r=rfly_original_getter_mex('close');catch,end;reader=[];end %#ok<NASGU>
        if ~isempty(remote),delete(remote);remote=[];end
        if ~isempty(peer),delete(peer);peer=[];end
    end
end
