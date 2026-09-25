function result=run_no_board_canonical_delivery_udp_probe(outputDir,runtime,expectedDllSha)
% Probe MATLAB -> CopterSim 2i28d -> generated DLL -> ii32d in mode 3.
root=fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(root,'matlab_validation'),fullfile(root,'m600_coptersim','matlab_validation'),fullfile(root,'host_runtime'));
if nargin<2
    runtime=gpenmpc_external_path('passive_ground_coptersim_runtime');
end
if nargin<3
    expectedDllSha='03FD4D18DBCDBE501213AC3B16D0AEF31CB1AF4F9A03C6F80DF6E99CB5BD9AFA';
end
exe=fullfile(runtime,'CopterSim.exe');
dll=fullfile(runtime,'external','model','GPENMPC_M600_Canonical.dll');
assert(~isfolder(outputDir)&&~isfile(outputDir),'gpenmpc:FreshOutput','Fresh output required.');
assert(isfile(exe)&&isfile(dll));
assert(strcmpi(m600check.fileSha256(exe),'BD3139D6D4BC665C254AD7C3C17618CF91D18138205D671FD88B428C0C017917'));
assert(strcmpi(m600check.fileSha256(dll),expectedDllSha));
existing=System.Diagnostics.Process.GetProcessesByName('CopterSim');
assert(existing.Length==0,'gpenmpc:CopterAlreadyRunning','Do not interfere with another CopterSim owner.');
mkdir(outputDir);
rows=zeros(12000,10);rowCount=0;sent=0;received=0;invalid=0;previousSim=NaN;
packetBytes=zeros(12000,1);packetMagic=zeros(12000,1);packetStatus=cell(12000,1);
targetPackets=0;targetInvalid=0;nonTargetPackets=0;
checks=struct('name',{},'pass',{});failure='';proc=[];sock=[];forceKilled=false;
policy=struct('expected_session_token',26090501, ...
    'payload_by_generation_kg',[2.21;1.75;.98;.55;0], ...
    'mass_by_generation_kg',[11.71;11.25;10.48;10.05;9.5], ...
    'allow_unbound_pre_session',true);
latest=[];initialObserved=false;healthyBound=false;unloadAck=false;beginSim=NaN;
try
    sock=udpport('datagram','IPV4','LocalPort',30101,'Timeout',.02);
    psi=System.Diagnostics.ProcessStartInfo;
    psi.FileName=exe;
    psi.Arguments='1 1 -1 GPENMPC_M600_Canonical 3 LowGPU 0 0 0 0 1 2';
    psi.WorkingDirectory=runtime;psi.UseShellExecute=false;psi.CreateNoWindow=true;
    psi.WindowStyle=System.Diagnostics.ProcessWindowStyle.Hidden;
    proc=System.Diagnostics.Process;proc.StartInfo=psi;
    assert(proc.Start(),'gpenmpc:CopterStart','Owned CopterSim did not start.');
    started=tic;lastSendSim=-Inf;generation=0;
    while toc(started)<25
        assert(~proc.HasExited,'gpenmpc:CopterExited','Owned CopterSim exited early.');
        drain();
        if ~initialObserved
            if ~isempty(latest)&&latest.packet_valid&&latest.environment_extension.valid&& ...
                    latest.environment_extension.initial_not_applied
                initialObserved=true;
            end
        elseif ~unloadAck&&~isempty(latest)&&latest.packet_valid
            sim=latest.sim_time_s;
            if sim>lastSendSim+.012
                generation=generation+1;
                if isnan(beginSim),beginSim=sim;end
                release=sim-beginSim>=8.10;
                payload=2.21;payloadGeneration=0;releaseGeneration=0;
                if release,payload=1.75;payloadGeneration=1;releaseGeneration=1;end
                value=struct('schema_version',2,'generation',generation, ...
                    'source_io_time_s',sim,'task_reference_time_s',0, ...
                    'payload_kg',payload,'wind_ned_xy_mps',zeros(2,1), ...
                    'mission_phase',2,'reference_jet_ned',zeros(12,1), ...
                    'payload_generation',payloadGeneration,'task_clock_paused',true, ...
                    'session_token',26090501,'board_min_rx_io_time_s',sim, ...
                    'board_valid_flags',15,'landed_state',1,'continuity_epoch',1, ...
                    'service_release_generation',releaseGeneration);
                bytes=gpenmpcTaskIo.encodePlantEnvironmentV2(value,1,2.21);
                write(sock,bytes,'uint8','127.0.0.1',30100);sent=sent+1;lastSendSim=sim;
            end
            if latest.environment_extension.valid&&latest.environment_extension.status_code==0&& ...
                    latest.environment_extension.applied_frame_generation>0
                healthyBound=true;
            end
            if latest.environment_extension.mass_ack_valid&& ...
                    latest.environment_extension.applied_payload_generation==1&& ...
                    abs(latest.environment_extension.actual_payload_kg-1.75)<1e-12&& ...
                    abs(latest.environment_extension.actual_total_mass_kg-11.25)<1e-12
                unloadAck=true;break
            end
        end
        pause(.002);
    end
    drain();
    check('initial_tag2_unbound_observed',initialObserved);
    check('MATLAB_sent_2i28d_frames',sent>0);
    check('generated_DLL_bound_expected_session',healthyBound);
    check('same_CopterSim_plant_advanced_over_8s',~isempty(latest)&&latest.sim_time_s>8);
    check('one_unload_actual_mass_ACK',unloadAck);
    check('all_target_delivery_packets_valid',targetPackets>0&&targetInvalid==0);
    check('multiplexed_non_target_packets_explicitly_classified',nonTargetPackets==received-targetPackets);
    check('no_model_or_environment_failure',~isempty(latest)&&~latest.model_failed&&~latest.environment_extension.task_env_failed);
catch err
    failure=getReport(err,'extended','hyperlinks','off');
end
cleanup();
remaining=System.Diagnostics.Process.GetProcessesByName('CopterSim');released=remaining.Length==0;
portReleased=false;
try,t=udpport('datagram','IPV4','LocalPort',30101);clear t;portReleased=true;catch,end
checkAfter('owned_CopterSim_released',released);
checkAfter('UDP_30101_released',portReleased);
columns={'rx_index','sim_time_s','packet_valid','model_failed','ground','env_present','env_valid','frame_generation','payload_generation','env_status'};
writetable(array2table(rows(1:rowCount,:),'VariableNames',columns),fullfile(outputDir,'DELIVERY_UDP_ROWS.csv'));
packetTable=table((1:received).',packetBytes(1:received),packetMagic(1:received),packetStatus(1:received), ...
    'VariableNames',{'rx_index','datagram_bytes','int32_magic','decode_status'});
writetable(packetTable,fullfile(outputDir,'PACKET_CLASSES.csv'));
allPassed=isempty(failure)&&all([checks.pass]);status='FAIL_HOST_MATLAB_COPTERSIM_DELIVERY_UDP';
if allPassed,status='PASS_HOST_MATLAB_COPTERSIM_DELIVERY_UDP';end
result=struct('schema','HOST_MATLAB_COPTERSIM_CANONICAL_DELIVERY_UDP_V1', ...
    'status',status,'pass',allPassed, ...
    'checks_total',numel(checks),'checks_passed',sum([checks.pass]),'checks',checks,'failure',failure, ...
    'runtime',runtime,'runtime_dll_sha256',m600check.fileSha256(dll), ...
    'copter_exe_sha256',m600check.fileSha256(exe),'process_force_kill_used',forceKilled, ...
    'packets_received',received,'target_delivery_packets',targetPackets,'target_delivery_invalid',targetInvalid, ...
    'multiplexed_non_target_packets',nonTargetPackets,'all_decoder_invalid_including_non_target',invalid,'environment_frames_sent',sent, ...
    'normal_unload_ACK',unloadAck,'actual_payload_kg',fieldOrNaN('actual_payload_kg'), ...
    'actual_total_mass_kg',fieldOrNaN('actual_total_mass_kg'), ...
    'COM_open',0,'board_actions',0,'MAVLink_actions',0,'actuator_actions',0, ...
    'claim','CopterSim UDP and generated-DLL probe.');
fid=fopen(fullfile(outputDir,'RESULT.json'),'w','n','UTF-8');assert(fid>=0);c=onCleanup(@()fclose(fid));fprintf(fid,'%s\n',jsonencode(result,PrettyPrint=true));clear c;
disp(jsonencode(struct('pass',result.pass,'checks',sprintf('%d/%d',result.checks_passed,result.checks_total), ...
    'packets_received',received,'frames_sent',sent,'payload_kg',result.actual_payload_kg,'mass_kg',result.actual_total_mass_kg,'failure',failure)));
assert(result.pass,'gpenmpc:DeliveryUdpProbe','See RESULT.json and DELIVERY_UDP_ROWS.csv.');

    function drain()
        n=sock.NumDatagramsAvailable;
        if n<=0,return;end
        data=read(sock,n,'uint8');
        for q=1:numel(data)
            bytes=uint8(data(q).Data(:));received=received+1;
            decoded=m600check.decodeCopterSimDeliveryDiagnostics(bytes,1,previousSim,policy);
            packetBytes(received)=numel(bytes);magic=NaN;
            if numel(bytes)>=4,magic=double(typecast(bytes(1:4),'int32'));end
            packetMagic(received)=magic;packetStatus{received}=decoded.status;
            isTarget=numel(bytes)==264&&magic==1234567890;
            if isTarget
                targetPackets=targetPackets+1;
                if ~decoded.packet_valid,targetInvalid=targetInvalid+1;end
            else
                nonTargetPackets=nonTargetPackets+1;
            end
            if decoded.packet_valid,previousSim=decoded.sim_time_s;else,invalid=invalid+1;end
            latest=decoded;rowCount=rowCount+1;
            assert(rowCount<=size(rows,1),'gpenmpc:Rows','Bounded row buffer exhausted.');
            e=decoded.environment_extension;
            rows(rowCount,:)=[received,decoded.sim_time_s,double(decoded.packet_valid),double(decoded.model_failed), ...
                double(decoded.ground_confirmed),double(e.present),double(e.valid),e.applied_frame_generation, ...
                e.applied_payload_generation,e.status_code];
        end
    end
    function check(name,ok),checks(end+1)=struct('name',name,'pass',islogical(ok)&&isscalar(ok)&&ok);end
    function checkAfter(name,ok),check(name,ok);end
    function v=fieldOrNaN(name)
        v=NaN;if ~isempty(latest)&&latest.environment_extension.present,v=latest.environment_extension.(name);end
    end
    function cleanup()
        if ~isempty(sock),sock=[];end
        if ~isempty(proc)
            try
                if ~proc.HasExited
                    proc.CloseMainWindow();
                    if ~proc.WaitForExit(3000),proc.Kill();forceKilled=true;proc.WaitForExit(5000);end
                end
            catch,try,if ~proc.HasExited,proc.Kill();forceKilled=true;end,catch,end,end
            try,proc.Dispose();catch,end
        end
    end
end
