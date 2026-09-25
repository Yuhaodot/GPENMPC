function info=ensure_gpenmpc_rfly_view(timeoutS,launcherPath,presentWindow)
% Reuse or open the installed RflySim3D view without starting a flight session.
% Readiness requires its child window and an observed UDP 20010 listener.
% GetActiveUdpListeners does not report listener ownership.
if nargin<1||isempty(timeoutS),timeoutS=35;end
% Only explicit viewer actions move focus to the verified renderer window.
if nargin<3||isempty(presentWindow),presentWindow=false;end
assert(islogical(presentWindow)&&isscalar(presentWindow),'gpenmpcView:PresentArgument');
rflyRoot=getenv('GPENMPC_RFLY_ROOT');
assert(~isempty(rflyRoot)&&isfolder(rflyRoot),'gpenmpcView:Installation', ...
    'Set GPENMPC_RFLY_ROOT to the RflySim/PX4PSP installation directory.');
officialLauncher=char(System.IO.Path.GetFullPath(fullfile(rflyRoot,'RflySim3D','RflySim3D.exe')));
if nargin<2||isempty(launcherPath),launcherPath=officialLauncher;end
validateattributes(timeoutS,{'numeric'},{'scalar','real','finite','positive'});
launcherPath=char(string(launcherPath));
assert(isrow(launcherPath)&&~isempty(launcherPath),'gpenmpcView:LauncherPath', ...
    'A single RflySim3D launcher path is required.');
launcherPath=char(System.IO.Path.GetFullPath(launcherPath));
if ~isfile(launcherPath)
    error('gpenmpcView:LauncherMissing','RflySim3D was not found: %s',launcherPath);
end
assert(strcmpi(launcherPath,officialLauncher),'gpenmpcView:UnexpectedLauncher', ...
    'Use the installed RflySim3D launcher: %s',officialLauncher);
viewRoot=fileparts(launcherPath);
childPath=fullfile(viewRoot,'RflySim3D','Binaries','Win64','RflySim3D.exe');
assert(isfile(childPath),'gpenmpcView:ChildMissing','RflySim3D runtime was not found: %s',childPath);

watch=tic;
mutex=System.Threading.Mutex(false,'Local\GPENMPCHILRflyViewStart');
lockHeld=false;
try
    lockHeld=mutex.WaitOne(int32(min(2147483647,ceil(double(timeoutS)*1000))));
catch ex
    % An abandoned mutex is acquired by the thread receiving this exception.
    if contains(ex.message,'AbandonedMutexException')
        lockHeld=true;
    else
        mutex.Dispose();rethrow(ex);
    end
end
if ~lockHeld
    mutex.Dispose();
    error('gpenmpcView:StartupBusy','RflySim3D startup is still busy after %.1f s. Try again when it finishes.',toc(watch));
end
lockCleanup=onCleanup(@()releaseViewMutex(mutex)); %#ok<NASGU>

instances=officialProcesses();
checkSingleInstance(instances);
launched=false;launcherPid=NaN;
if isempty(instances)
    if toc(watch)>=timeoutS
        error('gpenmpcView:StartupTimeout','Timed out waiting to start RflySim3D.');
    end
    startInfo=System.Diagnostics.ProcessStartInfo;
    startInfo.FileName=launcherPath;
    startInfo.WorkingDirectory=viewRoot;
    startInfo.UseShellExecute=false;
    startInfo.CreateNoWindow=true;
    startInfo.WindowStyle=System.Diagnostics.ProcessWindowStyle.Normal;
    try
        started=System.Diagnostics.Process.Start(startInfo);
        launcherPid=double(started.Id);
        started.Dispose();
    catch ex
        error('gpenmpcView:LaunchFailed','Could not open RflySim3D: %s',ex.message);
    end
    launched=true;
end

listenerSeen=false;listenerError='';
while true
    instances=officialProcesses();
    checkSingleInstance(instances);
    listenerSeen=false;listenerError='';
    try
        network=System.Net.NetworkInformation.IPGlobalProperties.GetIPGlobalProperties();
        endpoints=network.GetActiveUdpListeners();
        for k=1:double(endpoints.Length)
            if double(endpoints(k).Port)==20010,listenerSeen=true;break;end
        end
    catch ex
        listenerError=ex.message;
    end
    ready=find([instances.is_child]&[instances.window_handle]~=0,1);
    if ~isempty(ready)&&listenerSeen
        q=instances(ready);
        info=struct('launched',launched,'reused',~launched,'pid',q.pid, ...
            'window',q.window_handle,'elapsed_s',toc(watch),'launcher_pid',launcherPid, ...
            'launcher_path',launcherPath,'child_path',childPath, ...
            'udp_port',20010,'udp_listener_observed',true,'udp_listener_ownership_checked',false);
        info.presentation_requested=presentWindow;
        if presentWindow
            helper=fullfile(fileparts(mfilename('fullpath')),'RflyViewWindow.dll');
            assert(isfile(helper),'gpenmpcView:DisplayHelperMissing','Display helper was not found: %s',helper);
            NET.addAssembly(helper);
            shown=GPENMPC.Display.RflyViewWindow.Present(int32(q.pid),int64(q.window_handle));
            info.window_visible=logical(shown.Visible);
            info.window_restored=logical(shown.Restored);
            info.window_repositioned=logical(shown.Repositioned);
            info.window_foreground=logical(shown.Foreground);
        end
        return
    end
    if toc(watch)>=timeoutS,break;end
    pause(min(.15,max(0,double(timeoutS)-toc(watch))));
end
ids=sprintf('%g ',[instances.pid]);
if isempty(ids),ids='none';end
error('gpenmpcView:StartupTimeout', ...
    ['RflySim3D did not become ready within %.1f s (official process IDs: %s; ' ...
     'runtime window: %d; UDP 20010 listener: %d). Existing processes were left running. %s'], ...
    toc(watch),strtrim(ids),~isempty(ready),listenerSeen,listenerError);

    function rows=officialProcesses()
        rows=struct('pid',{},'path',{},'window_handle',{},'is_child',{});
        processes=System.Diagnostics.Process.GetProcessesByName('RflySim3D');
        unknown={};
        for j=1:double(processes.Length)
            process=processes(j);
            try
                process.Refresh();
                if ~process.HasExited
                    actualPath=char(System.IO.Path.GetFullPath(char(process.MainModule.FileName)));
                    if strcmpi(actualPath,launcherPath)||strcmpi(actualPath,childPath)
                        rows(end+1)=struct('pid',double(process.Id),'path',actualPath, ...
                            'window_handle',double(process.MainWindowHandle.ToInt64()), ...
                            'is_child',strcmpi(actualPath,childPath)); %#ok<AGROW>
                    end
                end
            catch ex
                % Treat inaccessible live processes as present to avoid duplicate launch.
                exited=false;
                try,exited=process.HasExited;catch,end
                if ~exited,unknown{end+1}=ex.message;end %#ok<AGROW>
            end
            process.Dispose();
        end
        if ~isempty(unknown)
            error('gpenmpcView:ProcessInspection', ...
                'Unable to verify the existing RflySim3D process. %s',unknown{1});
        end
    end
    function checkSingleInstance(rows)
        if nnz([rows.is_child])>1||nnz(~[rows.is_child])>1
            error('gpenmpcView:MultipleInstances', ...
                'Multiple RflySim3D instances are running (IDs: %s). Select one before starting.', ...
                strtrim(sprintf('%g ',[rows.pid])));
        end
    end
end

function releaseViewMutex(mutex)
% Capture the acquired handle for cleanup.
try,mutex.ReleaseMutex();catch,end
mutex.Dispose();
end
