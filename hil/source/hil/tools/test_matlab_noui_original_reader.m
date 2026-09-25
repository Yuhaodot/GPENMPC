function report=test_matlab_noui_original_reader(outputRoot)
% Test the MATLAB consumer with retained NoUI getter records
% and a disarmed software peer.
arguments,outputRoot (1,1) string,end
build=string(fileparts(fileparts(mfilename('fullpath'))));assert(~isfolder(outputRoot));mkdir(outputRoot);
oldPath=path;pg=onCleanup(@()path(oldPath)); %#ok<NASGU>
addpath(fullfile(build,'host_runtime'),fullfile(gpenmpc_external_path('local_original_getter_mex')));
diary(fullfile(outputRoot,'MATLAB_DIARY.txt'));dg=onCleanup(@()diary('off')); %#ok<NASGU>
parent=fullfile(gpenmpc_external_path('official_noui_step_snapshot'),'runtime');runtime=fullfile(outputRoot,'runtime');
assert(strcmp(sha(fullfile(parent,'CopterSimNoUI.exe')),'94B81EFB44058176DD5353669D9C28FC5331CC8411AB9EA3F2D27C1E8C343241'));
assert(strcmp(sha(fullfile(parent,'external','model','GPENMPC_M600_Diagnostic.dll')),'990850A2F40F3FCC2A6C47E63A4065B60FF49AA39CC4749FF443963B06F2EF7E'));
assert(System.Diagnostics.Process.GetProcessesByName('CopterSimNoUI').Length==0 ...
    &&System.Diagnostics.Process.GetProcessesByName('CopterSim').Length==0,'No second plant allowed.');
[ok,msg]=copyfile(parent,runtime);assert(ok,'%s',msg);
source=fullfile(build,'tools','probe_coptersim_software_carrier.cpp');exe=fullfile(outputRoot,'matlab_reader_peer.exe');
cxx=gpenmpc_install_path('llvm','bin\clang++.exe');
command=sprintf('"%s" -std=c++14 -O2 -Wall -Wextra -Werror -Wno-address-of-packed-member -static -municode -DGPENMPC_EXTERNAL_RDR_MATLAB_CONSUMER=1 -I"%s" "%s" -lws2_32 -liphlpapi -o "%s"', ...
    cxx,fullfile(build,'rfly_vendor_integration','application_integration','build_fmuv6c'),source,exe);
[rc,log]=system(command);write(fullfile(outputRoot,'COMPILE_LOG.txt'),log);assert(rc==0,'%s',log);
process=[];reader=[];
try
% Warm the reader and library startup.
warm=rfly_original_getter_mex('open');wc=rfly_original_getter_mex('close'); %#ok<NASGU>
reader=rfly_original_getter_mex('open');
si=System.Diagnostics.ProcessStartInfo;si.FileName=char(exe);
si.Arguments=sprintf('"%s" "%s" "%s" GPENMPC_M600_Diagnostic RDR1_SSS1',fullfile(runtime,'CopterSimNoUI.exe'),runtime,outputRoot);
si.UseShellExecute=false;si.CreateNoWindow=true;si.WindowStyle=System.Diagnostics.ProcessWindowStyle.Hidden;
environment=si.EnvironmentVariables;
environment.Remove('GPENMPC_CLOCK_RING_SECTION');environment.Add('GPENMPC_CLOCK_RING_SECTION',reader.ring_name);
environment.Remove('GPENMPC_STEP_SNAPSHOT_SECTION');environment.Add('GPENMPC_STEP_SNAPSHOT_SECTION',reader.status_name);
for key={'OMP_NUM_THREADS','MKL_NUM_THREADS','OPENBLAS_NUM_THREADS'}
    environment.Remove(key{1});environment.Add(key{1},'1');
end
assert(environment.ContainsKey('GPENMPC_CLOCK_RING_SECTION')&&environment.ContainsKey('GPENMPC_STEP_SNAPSHOT_SECTION'));
% Preallocate bounded short-diagnostic storage. No silent record eviction.
records=zeros(336,8192,'uint8');reads=zeros(8192,1,'uint64');n=0;fail=false;last=[];
process=System.Diagnostics.Process.Start(si);started=tic;
while ~process.HasExited&&toc(started)<25
    take();pause(.002);
end
if ~process.HasExited,process.Kill();process.WaitForExit();error('gpenmpc:NoUiReaderTimeout','Owned fake-peer diagnostic exceeded its 25s resource bound.');end
take();rc=process.ExitCode;
closed=rfly_original_getter_mex('close');reader=[];
records=records(:,1:n);reads=reads(1:n);save(fullfile(outputRoot,'MATLAB_ORIGINAL_GETTERS.mat'),'records','reads','last','closed');
child=jsondecode(fileread(fullfile(outputRoot,'NUMERICAL_RESULT.json')));
ring=jsondecode(fileread(fullfile(outputRoot,'GETTER_RING_OBSERVATION.json')));
status=jsondecode(fileread(fullfile(outputRoot,'STEP_SNAPSHOT_OBSERVATION.json')));
fh=fopen(fullfile(outputRoot,'MATLAB_GETTER_RECORDS.bin'),'wb');assert(fh>=0);fwrite(fh,records,'uint8');fclose(fh);
f=fopen(fullfile(outputRoot,'GETTER_RING_SECTION_FINAL.bin'),'rb');assert(f>=0);finalRing=fread(f,Inf,'*uint8');fclose(f);
producer=typecast(finalRing(21:24),'uint32');consumer=typecast(finalRing(73:76),'uint32');
checks=struct('helper_exit_zero',rc==0,'no_reader_fault',~fail&&~closed.failed,'records_nonempty',n>0, ...
    'all_actual_getters_consumed',ring.complete_capture&&n==ring.events&&n==double(closed.total_read), ...
    'step_status_complete',status.observation_complete,'separate_real_matlab_consumer', ...
    producer==child.owned_pid&&consumer==System.Diagnostics.Process.GetCurrentProcess().Id&&producer~=consumer, ...
    'raw_event_order',all(typecast(reshape(records(1:8,:),[],1),'uint64')==uint64((1:n)')), ...
    'original_read_time_monotone',all(diff(reads)>=0),'single_model_peer_released',child.owned_process_released, ...
    'ports_released',child.udp_ports_released,'no_board_no_nonzero_input',child.COM_requested==0&&child.PX4_processes_launched==0&&child.nonzero_actuator_inputs==0);
report=struct('scope','MATLAB_OFFICIAL_NOUI_GETTER_READER', ...
    'checks',checks,'passed',all(structfun(@logical,checks)),'actual_records',n,'actual_hil_sensor_messages',child.hil_sensor_messages, ...
    'hypothesis','Moving only the original shared-memory consumer to MATLAB preserves complete actual step/getter observations without clock renewal.', ...
    'source_sha256',sha(source),'reader_binary_sha256',sha(which('rfly_original_getter_mex')), ...
    'model_sha256',sha(fullfile(runtime,'external','model','GPENMPC_M600_Diagnostic.dll')), ...
    'board_source_binding_proven',false,'hardware_actions',0,'board',0,'COM',0,'nonzero_output',0);
write(fullfile(outputRoot,'RESULT.json'),jsonencode(report,PrettyPrint=true));
fprintf('Actual NoUI MATLAB reader: %d records, %d/%d\n',n,sum(structfun(@logical,checks)),numel(fieldnames(checks)));assert(report.passed);
catch problem
    finish();rethrow(problem)
end
finish();
    function take()
        v=rfly_original_getter_mex('drain');count=size(v.records,2);assert(n+count<=8192,'No raw capture eviction.');
        records(:,n+(1:count))=v.records;reads(n+(1:count))=v.original_read_ns;n=n+count;fail=fail||v.failed;last=v;
    end
    function finish()
        if ~isempty(process),try,if ~process.HasExited,process.Kill();process.WaitForExit();end;process.Dispose();catch,end,end
        if ~isempty(reader),try,v=rfly_original_getter_mex('close'); %#ok<NASGU>
        catch,end,end
    end
end
function h=sha(p),f=fopen(p,'rb');assert(f>=0);c=onCleanup(@()fclose(f));b=fread(f,Inf,'*uint8');m=java.security.MessageDigest.getInstance('SHA-256');m.update(typecast(b,'int8'));h=upper(reshape(dec2hex(typecast(m.digest(),'uint8'),2).',1,[]));end %#ok<NASGU>
function write(p,t),f=fopen(p,'w','n','UTF-8');assert(f>=0);c=onCleanup(@()fclose(f));fprintf(f,'%s\n',t);end %#ok<NASGU>
