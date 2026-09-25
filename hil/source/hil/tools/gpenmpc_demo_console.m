function fig=gpenmpc_demo_console(initialMode,language,options)
% Monitor the HIL session with explicit Start and Restore actions.
% Display refresh is up to 5 Hz; the board control loop runs independently.
if nargin<1,initialMode="LIVE";end
if nargin<2,language="EN";end
if nargin<3,options=struct();end
language=upper(string(language));assert(ismember(language,["ZH","EN"]));
% Retain the language argument for existing callers; this interface uses English.
language="EN";
build=fileparts(fileparts(mfilename('fullpath')));
defaults=struct('LogRoot',gpenmpc_log_root(), ...
 'ReplayFile',getenv('GPENMPC_REPLAY_FILE'), ...
 'Visible','on','ViewerEnabled',true,'DisplayPort',30251, ...
 'LockFile',fullfile(build,'tools','usb_rc_runtime','manual_session.lock'));
assert(isstruct(options)&&isscalar(options)&&all(ismember(fieldnames(options),fieldnames(defaults))));
for key=fieldnames(defaults).',if ~isfield(options,key{1}),options.(key{1})=defaults.(key{1});end,end
assert(ismember(string(options.Visible),["on","off"])&&isscalar(options.ViewerEnabled)&&islogical(options.ViewerEnabled));
assert(isscalar(options.DisplayPort)&&isfinite(options.DisplayPort)&&options.DisplayPort==fix(options.DisplayPort)&&options.DisplayPort>=0&&options.DisplayPort<=65535);
logBase=options.LogRoot;
addpath(fullfile(build,'m600_coptersim','matlab_validation'));
existing=findall(groot,'Type','figure','Tag','GPENMPCDemoConsole');
if ~isempty(existing)
    fig=existing(1);if string(options.Visible)=="on",figure(fig);end
    if string(initialMode)=="LIVE"&&isappdata(fig,'GPENMPCShow3DView')
        showView=getappdata(fig,'GPENMPCShow3DView');showView();
    end
    return;
end
screen=get(groot,'ScreenSize');bg=[.965 .968 .971];ink=[.12 .18 .23];muted=[.39 .44 .49];font='Microsoft YaHei';
fig=figure('Name','GPENMPC HIL','NumberTitle','off','Visible',options.Visible, ...
 'Tag','GPENMPCDemoConsole','Color',bg,'MenuBar','none','ToolBar','none', ...
 'Position',[25 40 min(1510,screen(3)-60) min(950,screen(4)-80)],'CloseRequestFcn',@closeConsole);
setappdata(fig,'ExplicitStartUI',true);
mode="WAIT";socket=[];tick=[];clock=tic;lastRx=-Inf;launchProcess=[];launchAt=-Inf;launchBaseline='';runtimeLog='';
t=[];heightM=[];vel=[];angles=[];rotors=[];windNE=[];bodyRates=[];firstTime=NaN;
groundNed=NaN;groundRun='';displayEnv=[];displayDiagnostic=[];
tr=[];replayBreaks=[];plotBreakRows=[];replayPaused=true;replayPosition=0;replayClock=tic;
runRoot='';resetRunRoot='';resetAppliedRoot='';busy=false;runPhase="IDLE";lastStatus=-Inf;outcome=[];monitorError='';inputError='';ownedModels={};
preparingView=false;viewError='';
label([.023 .941 .74 .038],'GPENMPC HIL',20,ink,true);
modeLabel=label([.023 .902 .80 .029],'',11,muted,false);
statusBox=label([.023 .822 .953 .071],'',11,ink,false);statusBox.Tag='GPENMPCDemoStatus';
label([.023 .773 .20 .025],'SESSION',11,muted,true);
startButton=button('Start manual session',.713,@(~,~)startManual());startButton.Tag='GPENMPCManualStart';
monitorButton=button('Live data',.654,@(~,~)liveMode());monitorButton.Tag='GPENMPCLiveMonitor';
resetButton=button('Reset simulation',.595,@(~,~)resetManual());resetButton.Tag='GPENMPCManualReset';
label([.023 .537 .198 .044],sprintf('End flight: Ctrl + Shift + L\nLand and return to standby'),10,ink,false);
label([.023 .489 .20 .026],'SYSTEM & RECORDS',11,muted,true);
button('System diagram',.430,@(~,~)openModel(false));
button('M600 Model',.371,@(~,~)openModel(true));
replayButton=button('Load simulation record',.312,@(~,~)replayMode());
pauseButton=button('Play simulation recording',.253,@(~,~)pauseReplay());pauseButton.Tag='GPENMPCReplayToggle';pauseButton.Enable='off';
button('Open status logs',.194,@(~,~)openRecords());
restoreButton=button('Restore original firmware',.135,@(~,~)startManual("RESTORE_ORIGINAL"));
restoreButton.TooltipString='Maintenance: restore the previously saved original firmware.';
inputLabel=label([.023 .028 .198 .094],sprintf('FS-i6S → MATLAB → Pixhawk 6C\nM600 · CopterSim · RflySim3D'),9,muted,false);
axesList=gobjects(6,1);series=cell(6,1);
for k=1:6
 row=floor((k-1)/2);col=mod(k-1,2);
 axesList(k)=axes(fig,'Units','normalized','Position',[.278+.367*col .624-.252*row .312 .152], ...
  'FontName',font,'FontSize',10,'Box','off','Color','white','XColor',muted,'YColor',muted);
end
footer=label([.26 .010 .715 .044],'',9,muted,false);
setappdata(fig,'GPENMPCConsoleAxes',axesList);
tick=timer('ExecutionMode','fixedSpacing','Period',.2,'BusyMode','drop','TimerFcn',@(~,~)refresh(),'ErrorFcn',@displayError);
setappdata(fig,'GPENMPCViewerTimer',tick);
setappdata(fig,'GPENMPCShow3DView',@showViewExplicitly);
resetPlots(false);
if string(initialMode)=="REPLAY",replayMode();elseif string(initialMode)=="LIVE",liveMode();end
refreshRunStatus(true);start(tick);
if string(initialMode)=="LIVE",showViewExplicitly();end
    function h=label(pos,txt,size,color,bold)
        h=uicontrol(fig,'Style','text','Units','normalized','Position',pos,'String',txt,'FontName',font, ...
         'FontSize',size,'ForegroundColor',color,'BackgroundColor',bg,'HorizontalAlignment','left');
        if bold,h.FontWeight='bold';end
    end
    function h=button(txt,y,callback)
        h=uicontrol(fig,'Style','pushbutton','Units','normalized','Position',[.023 y .194 .048], ...
         'String',txt,'FontName',font,'FontSize',10,'ForegroundColor',ink,'BackgroundColor','white','Callback',callback);
    end
    function resetPlots(isReplay)
        counts=[1 3 3 6 2 3];colors=[0 .36 .64;.84 .33 .1;0 .55 .4;.66 .43 .67;.8 .65 0;.25 .25 .25];
        titles={'Height above takeoff ground','Velocity (North / East / Down)','Body attitude','Six virtual inputs received by model','Disturbance','Angular-rate response'};
        units={'m','m/s','deg','Normalized input','m/s','deg/s'};
        modeLabel.String='Manual Hardware in the Loop · Robust SE(3) flight control';
        footer.String='M600 six-degree-of-freedom dynamics · Live flight and environment response';
        if isReplay
            counts=[1 3 3 6 3 1];
            modeLabel.String='Simulation playback · GP-enhanced energy-aware eNMPC with robust SE(3) control';
            footer.String='Cambridge delivery mission · Tracking, learned compensation and optimization response';
            inputLabel.String='Simulation recording · Play / Pause';
            titles={'Position tracking error','Velocity (software frame)','Attitude tracking error','Rotor commands (modelled)','GP physical authority','eNMPC solution time'};
            units{4}='N';units{5}='Weight';units{6}='s';
        end
        for j=1:6
            ax=axesList(j);cla(ax);legend(ax,'off');axis(ax,'on');hold(ax,'on');colororder(ax,colors);
            series{j}=plot(ax,NaN(2,counts(j)),NaN(2,counts(j)),'LineWidth',1.4);
            title(ax,titles{j},'FontSize',11,'FontWeight','normal','Color',ink);ylabel(ax,units{j});xlabel(ax,'Time / s');
            grid(ax,'on');ax.GridAlpha=.12;ax.XLimMode='auto';ax.YLimMode='auto';
        end
        legend(axesList(2),{'N / x','E / y','D / z'},'Location','northwest','Orientation','horizontal','Box','off');
        legend(axesList(3),{'roll','pitch','yaw'},'Location','northwest','Orientation','horizontal','Box','off');
        legend(axesList(4),{'1','2','3','4','5','6'},'Location','northwest','Orientation','horizontal','Box','off');
        if isReplay
            legend(axesList(5),{'F1','F2','F3'},'Location','northwest','Orientation','horizontal','Box','off');
            yline(axesList(6),.28,'--','0.28 s','HandleVisibility','off');
        else
            inputLabel.String=sprintf('FS-i6S → MATLAB → Pixhawk 6C\nM600 · CopterSim · RflySim3D');
            legend(axesList(5),{'North','East'},'Location','northwest','Orientation','horizontal','Box','off');
            legend(axesList(6),{'p','q','r'},'Location','northwest','Orientation','horizontal','Box','off');
        end
    end
    function liveMode()
        if mode=="LIVE"&&~isempty(socket),return;end
        closeSocket();mode="LIVE";tr=[];clearCurves();resetPlots(false);pauseButton.Enable='off';
        monitorError='';
        try
            socket=udpport('datagram','IPV4','LocalHost','127.0.0.1','LocalPort',options.DisplayPort,'Timeout',.01);
            setappdata(fig,'GPENMPCDisplayPort',socket.LocalPort);
        catch ex,mode="WAIT";monitorError=ex.message;statusBox.String=ex.message;end
    end
    function yes=ownerBusy()
        yes=false;p=options.LockFile;
        if isfile(p)
            try,h=System.IO.FileStream(p,System.IO.FileMode.Open,System.IO.FileAccess.ReadWrite,System.IO.FileShare.None);h.Dispose();
            catch,yes=true;end
        end
        if ~isempty(launchProcess),try,yes=yes||~launchProcess.HasExited;catch,end;end
        yes=yes||(toc(clock)-launchAt<8);
    end
    function startManual(operation)
        if nargin<1,operation="START";end
        refreshRunStatus(true);if busy||preparingView,return;end
        if operation=="START",liveMode();if mode~="LIVE",return;end;end
        if operation=="START"
            if ~prepareView(),refreshRunStatus(true);return;end
            statusBox.String='Checking transmitter…';drawnow;
            try
                start_gpenmpc_usb_manual('CHECK_INPUT');inputError='';
            catch ex
                if strcmp(ex.identifier,'gpenmpcRc:DeviceIdentity')&&contains(ex.message,'found 0')
                    inputError='Transmitter not connected · Power on, connect USB, then select Start.';
                else,inputError=['Transmitter check: ' ex.message];end
                refreshRunStatus(true);return;
            end
        end
        runtimeLog=fullfile(logBase,'Demo_UI_ARTIFACTS',['RUNTIME_' char(datetime('now','Format','yyyyMMdd_HHmmss_SSS')) '.log']);
        args=sprintf('-nosplash -nodesktop -sd "%s" -logfile "%s" -r "addpath(fullfile(pwd,''tools'')); rcResult=start_gpenmpc_usb_manual(''%s''); if isfield(rcResult,''result'') && ((isequal(rcResult.result.safe,true) && (~isfield(rcResult.result,''raw_storage_pending'') || ~rcResult.result.raw_storage_pending)) || (isfield(rcResult.result,''retry_reset_available'') && rcResult.result.retry_reset_available)), exit; end;"',build,runtimeLog,operation);
        if operation=="RESUME_RESET"
            args=sprintf('-nosplash -sd "%s" -logfile "%s" -batch "addpath(fullfile(pwd,''tools'')); start_gpenmpc_usb_manual(''RESUME_RESET'');"',build,runtimeLog);
        end
        try
            matlabExecutable=fullfile(matlabroot,'bin','matlab.exe');
            assert(isfile(matlabExecutable),'gpenmpcDemo:MatlabExecutable','MATLAB executable was not found: %s',matlabExecutable);
            spec=System.Diagnostics.ProcessStartInfo(matlabExecutable,args);spec.UseShellExecute=false;
            spec.WindowStyle=System.Diagnostics.ProcessWindowStyle.Hidden;spec.CreateNoWindow=true;
            launchBaseline=runRoot;if operation=="RESUME_RESET",launchBaseline='';end
            launchProcess=System.Diagnostics.Process.Start(spec);launchAt=toc(clock);busy=true;
            startButton.Enable='off';startButton.String='Starting…';resetButton.Enable='off';
            if operation=="RESTORE_ORIGINAL"
                statusBox.String='Opening firmware maintenance...';
            else
                statusBox.String='Starting simulation…';
            end
        catch ex,statusBox.String=ex.message;end
    end
    function ready=prepareView()
        % Display preparation runs before the independent hardware owner.
        ready=false;if preparingView,return;end
        preparingView=true;viewError='';
        startButton.Enable='off';resetButton.Enable='off';restoreButton.Enable='off';
        statusBox.String='Opening 3D view…';drawnow;
        try
            receipt=ensure_gpenmpc_rfly_view([],[],true);
            setappdata(fig,'GPENMPC3DView',receipt);ready=true;
        catch ex
            viewError=['Unable to open 3D view: ' ex.message];
        end
        preparingView=false;
    end
    function showViewExplicitly()
        % Prepare the display without changing the flight owner's Start/Reset state.
        % This action is not invoked by the monitor timer.
        if ~options.ViewerEnabled||preparingView,return;end
        prepareView();refreshRunStatus(true);
    end
    function resetManual()
        refreshRunStatus(true);
        if busy
            if ~ismember(runPhase,["RUNNING","NO_DATA","RECOVERING","RESET_AVAILABLE","RESETTING"]),return;end
            preparedPath=fullfile(runRoot,'SHORT_ENTRY_PREPARED.json');
            if ~isfile(preparedPath),return;end
            preparedView=jsondecode(fileread(preparedPath));
            if ~isfield(preparedView,'operator_reference')||~preparedView.operator_reference,return;end
            % Request reset from the owner of this exact session; the panel does not open COM.
            requestPath=fullfile(runRoot,'MANUAL_RESET_REQUEST');
            if ~isfile(requestPath)
                f=fopen(requestPath,'w');assert(f>=0,'gpenmpcDemo:ResetRequest','Unable to submit the reset request.');
                fprintf(f,'RESET_SIMULATION\n');fclose(f);
            end
            resetRunRoot=runRoot;resetButton.Enable='off';
            statusBox.String='Resetting simulation · Ready to restart when complete.';
            return;
        end
        if ~isempty(runRoot)&&(~isstruct(outcome)||~isfield(outcome,'safe')||~isequal(outcome.safe,true))
            if runPhase=="RESET_AVAILABLE",startManual("RESUME_RESET");end
            return
        end
        resetRunRoot=runRoot;
        if ~isempty(runRoot),reset_m600_standby_view(runRoot);end
        clearCurves();resetPlots(false);liveMode();
        inputError='';statusBox.String='Ready · Select Start for the next session.';
    end
    function clearCurves()
        t=[];heightM=[];vel=[];angles=[];rotors=[];windNE=[];bodyRates=[];firstTime=NaN;lastRx=-Inf;
        displayEnv=[];displayDiagnostic=[];
    end
    function refresh()
        if ~isgraphics(fig),return;end
        if mode=="LIVE"&&~isempty(socket)
            n=socket.NumDatagramsAvailable;
            if n>0
                dg=read(socket,n,'uint8');
                for at=max(1,n-24):n
                    if istable(dg),b=dg.Data{at};else,b=dg(at).Data;end
                    b=uint8(b(:));
                    if numel(b)==232
                        header=typecast(b(1:8),'int32');v=typecast(b(9:end),'double');
                        if isequal(header(:),int32([1234567897;1]))&&all(isfinite(v))&&v(1)==2,displayEnv=v;end
                        continue
                    elseif numel(b)==264
                        header=typecast(b(1:8),'int32');v=typecast(b(9:end),'double');
                        if isequal(header(:),int32([1234567890;1]))&&all(isfinite(v([1:7 26:32])))&&v(26)==2,displayDiagnostic=v;end
                        continue
                    end
                    s=m600check.decodeTruthPacket(uint8(b),struct('expected_copter_id',1,'expected_vehicle_type',5));
                    if ~s.valid,continue;end
                    if ~isempty(t)&&s.time_s<firstTime+t(end),clearCurves();end
                    if isnan(firstTime),firstTime=s.time_s;end
                    tt=s.time_s-firstTime;if ~isempty(t)&&tt<=t(end),continue;end
                    t(end+1,1)=tt;heightM(end+1,1)=groundNed-s.position_ned_m(3);vel(end+1,:)=s.velocity_ned_mps;
                    angles(end+1,:)=rad2deg(s.euler_rad);rotors(end+1,:)=s.motor_rpm(1:6)/1000;lastRx=toc(clock);
                    windNE(end+1,:)=[NaN NaN];bodyRates(end+1,:)=[NaN NaN NaN];
                    if s.angular_rate_present,bodyRates(end,:)=rad2deg(s.angular_rate_body_radps);end
                    if ~isempty(displayEnv)&&~isempty(displayDiagnostic)
                        e=displayEnv;d=displayDiagnostic;
                        % Leave gaps for missing or stale wind samples without changing control timestamps.
                        if d(1)==0&&d(32)==0&&d(27)==e(23)&&d(28)==e(2)&&d(28)>0 ...
                          &&d(30)==e(5)&&abs(s.time_s-d(3))<=.25
                            windNE(end,:)=e(6:7);
                        end
                    end
                end
                if numel(t)>1500,ids=numel(t)-1499:numel(t);t=t(ids);heightM=heightM(ids);vel=vel(ids,:);angles=angles(ids,:);rotors=rotors(ids,:);windNE=windNE(ids,:);bodyRates=bodyRates(ids,:);end
                if ~isempty(t),setSeries(1,t,heightM);setSeries(2,t,vel);setSeries(3,t,angles);setSeries(4,t,rotors);setSeries(5,t,windNE);setSeries(6,t,bodyRates);end
            end
        end
        refreshRunStatus(false); % Run state is observed even in WAIT and REPLAY.
        if mode=="REPLAY",renderReplay();end
        drawnow limitrate nocallbacks;
    end
    function refreshRunStatus(force)
        if ~force&&toc(clock)-lastStatus<1,return;end
        lastStatus=toc(clock);busy=ownerBusy();outcome=[];runPhase="IDLE";
        msg='Standby · Select Start manual session.';
        try
            logs=dir(fullfile(logBase,'manual_session_*','MATLAB_RUN.log'));
            if ~isempty(logs)
                [~,ix]=sort({logs.folder});log=logs(ix(end));runRoot=log.folder;[~,runName]=fileparts(runRoot);
                if ~strcmp(groundRun,runRoot)
                    groundNed=NaN;preparedPath=fullfile(runRoot,'SHORT_ENTRY_PREPARED.json');
                    if isfile(preparedPath)
                        preparedView=jsondecode(fileread(preparedPath));
                        if isfield(preparedView,'initial_world_ground_ned_m'),groundNed=preparedView.initial_world_ground_ned_m(3);groundRun=runRoot;end
                    end
                end
                txt=fileread(fullfile(runRoot,log.name));p=fullfile(runRoot,'OUTER_SHORT_RESULT.json');
                if isfile(p),outcome=jsondecode(fileread(p));end
                if isstruct(outcome)&&isfield(outcome,'safe')&&outcome.safe
                    pending=isfield(outcome,'raw_storage_pending')&&outcome.raw_storage_pending;
                    if pending&&~busy,runPhase="STOPPED";msg='Board restored · Last program did not finish cleanly; no current flight.';
                    elseif pending,runPhase="SAVING";msg='Board restored · Saving complete records on E:.';
                    elseif isfield(outcome,'manual_reset_completed')&&outcome.manual_reset_completed
                        if ~strcmp(resetAppliedRoot,runRoot),clearCurves();resetPlots(false);resetAppliedRoot=runRoot;end
                        resetRunRoot=runRoot;
                        runPhase="IDLE";msg='Reset complete · Select Start manual session.';
                    elseif isfield(outcome,'preparation_failed')&&outcome.preparation_failed
                        runPhase="INPUT_WAIT";msg='Preparation incomplete · Ready to retry.';
                        if isfield(outcome,'failure')&&contains(outcome.failure.message,'FS-i6S')&&contains(outcome.failure.message,'found 0')
                            msg='Previous start did not detect the transmitter · Select Start to check again.';
                        end
                    elseif isfield(outcome,'maintenance_only')&&outcome.maintenance_only
                        runPhase="COMPLETE";msg='Original firmware restored · Standby.';
                    elseif isfield(outcome,'short_completed')&&outcome.short_completed
                        runPhase="COMPLETE";msg='Session ended · Disarmed and interfaces released.';
                    elseif isfield(outcome,'short')&&isstruct(outcome.short)&&isfield(outcome.short,'counts') ...
                            &&isfield(outcome.short.counts,'arm_requests')&&outcome.short.counts.arm_requests==0
                        runPhase="INPUT_WAIT";msg='Preparation incomplete · Back in standby, ready to retry.';
                    else,runPhase="STOPPED";msg='Session stopped with an error; safety cleanup completed.';end
                elseif ~busy&&isstruct(outcome)&&isfield(outcome,'safe')&&~outcome.safe ...
                        &&isfield(outcome,'short')&&isfield(outcome.short,'manual_reset_requested')&&outcome.short.manual_reset_requested
                    runPhase="RESET_AVAILABLE";msg='Reset connection interrupted · Select Reset simulation to reconnect.';
                elseif busy&&isfile(fullfile(runRoot,'MANUAL_RESET_REQUEST'))
                    runPhase="RESETTING";msg='Resetting simulation · Ready to restart when complete.';
                elseif contains(txt,'USB_RC_RESET_AVAILABLE:')
                    runPhase="RESET_AVAILABLE";msg='Flight stopped · Select Reset simulation to return to the initial state.';
                elseif contains(txt,'USB_RC_MAINTENANCE:')
                    runPhase="MAINTENANCE";msg='Restoring original firmware.';
                elseif contains(txt,'USB_RC_STOP:')||isfile(fullfile(runRoot,'SHORT_HIL','RESULT.json'))
                    runPhase="RECOVERING";msg='Control stopped · Landing, disarming and releasing session interfaces.';
                    if contains(txt,'USB_RC_STOP: Preparation incomplete')
                        msg='Preparation incomplete · Returning to standby.';
                    end
                elseif contains(txt,'USB_RC_ACTIVE:')
                    if toc(clock)-lastRx<1,runPhase="RUNNING";msg='Running · Sticks are live. Ctrl + Shift + L ends the session and lands.';
                    else,runPhase="NO_DATA";msg='Live state reception interrupted · Waiting for connection.';end
                elseif busy,runPhase="PREPARING";msg='Preparing flight · Waiting for ready status.';
                elseif isempty(outcome),runPhase="UNCONFIRMED";msg='Previous session status needs checking · Not ready.';end
                if busy&&ismember(runPhase,["COMPLETE","STOPPED","INPUT_WAIT","IDLE"])
                    runPhase="RELEASING";msg='Closing session connections · Returning to standby.';
                end
                detail=progressText(txt,runRoot);statusBox.TooltipString=runRoot;
                if ismember(runPhase,["INPUT_WAIT","RESETTING","RESET_AVAILABLE"]),detail='';end
                if runPhase=="STOPPED"
                    fpath=fullfile(runRoot,'SHORT_HIL','RESULT.json');
                    if isfile(fpath)
                        s=jsondecode(fileread(fpath));
                        if isfield(s,'failure')&&isstruct(s.failure)&&isfield(s.failure,'message')
                            detail=sprintf('%s | %s','Cause',s.failure.message);
                            if contains(s.failure.message,'BOARD_LOCAL_CONTEXT_STOPPED:7')
                                if isfield(s,'counts')&&s.counts.commits>0
                                    span=s.last_commit_io_s-s.first_commit_io_s;
                                    detail=sprintf('Onboard exchange stopped during flight (code 7) · %d confirmed commits over %.1f s.',s.counts.commits,span);
                                else
                                    detail='Onboard exchange stopped before first control commit (code 7).';
                                end
                                if isfield(s,'board_stop_lines')&&~isempty(s.board_stop_lines),statusBox.TooltipString=strjoin(string(s.board_stop_lines),newline);end
                            end
                        end
                    end
                end
                if busy
                    created=System.IO.File.GetCreationTime(fullfile(runRoot,log.name));elapsed=System.DateTime.Now.Subtract(created);
                    detail=sprintf('%s | %s %.0f s',detail,'Session elapsed',elapsed.TotalSeconds);
                end
                if runPhase=="RUNNING"&&~isempty(t),detail=sprintf('%s %.1f s | %s','Model time',firstTime+t(end),'Receiving live state');end
                if ~busy&&strcmp(runRoot,resetRunRoot)&&isstruct(outcome)&&isfield(outcome,'safe')&&isequal(outcome.safe,true)
                    runPhase="IDLE";msg='Standby · Select Start manual session.';detail='';
                end
                statusBox.String=sprintf('%s\n%s',msg,detail);
            else,statusBox.String=msg;end
            if busy&&runPhase=="IDLE",runPhase="PREPARING";statusBox.String='Starting runtime process…';end
            if isfinite(launchAt)&&strcmp(runRoot,launchBaseline)
                if busy
                    runPhase="PREPARING";statusBox.String=sprintf('%s %.0f s','Loading runtime ·',toc(clock)-launchAt);
                else
                    runPhase="LAUNCH_FAILED";statusBox.String='Runtime failed to start; no new board session. Error log is on E:.';statusBox.TooltipString=runtimeLog;
                end
            end
        catch ex,runPhase="UNKNOWN";statusBox.String=['Status unavailable: ' ex.message];end
        if ~isempty(monitorError)
            runPhase="MONITOR_ERROR";statusBox.String=['Live monitor not connected: ' monitorError];
        end
        if ~busy&&~isempty(inputError),runPhase="INPUT_WAIT";statusBox.String=inputError;end
        % Preserve unfinished recovery status if display updates fail.
        if ~busy&&~isempty(viewError),statusBox.String=sprintf('%s\n%s',statusBox.String,viewError);end
        if preparingView,runPhase="PREPARING_VIEW";statusBox.String='Opening 3D view…';end
        unavailable=busy||preparingView||ismember(runPhase,["PREPARING","RECOVERING","RESETTING","RESET_AVAILABLE","MAINTENANCE","SAVING","RUNNING","NO_DATA","UNCONFIRMED","UNKNOWN","MONITOR_ERROR"]);
        resetAvailable=(busy&&ismember(runPhase,["RUNNING","NO_DATA","RECOVERING"]))||runPhase=="RESET_AVAILABLE";
        startButton.Enable=onoff(~unavailable);resetButton.Enable=onoff(~unavailable||resetAvailable);replayButton.Enable=onoff(~unavailable);
        restoreButton.Enable=onoff(~unavailable);
        if unavailable
            if runPhase=="RUNNING",startButton.String='Manual session running';
            elseif ismember(runPhase,["RECOVERING","SAVING"]),startButton.String='Finishing session';
            else,startButton.String='Session in progress';end
        else,startButton.String='Start manual session';end
        statusBox.BackgroundColor=[.89 .93 .96];
        if runPhase=="RUNNING",statusBox.BackgroundColor=[.86 .94 .89];
        elseif ~isempty(viewError)||ismember(runPhase,["STOPPED","INPUT_WAIT","UNKNOWN","UNCONFIRMED","NO_DATA"]),statusBox.BackgroundColor=[.98 .93 .84];end
        setappdata(fig,'GPENMPCObservedRunState',struct('phase',runPhase,'busy',busy,'run_root',runRoot,'mode',mode));
        monitorButton.Enable=onoff(mode~="LIVE"||isempty(socket));
    end
    function txt=progressText(log,root)
        if isfile(fullfile(root,'FINAL_READONLY_SAFETY.json')),txt='Final safety record written';
        elseif isfile(fullfile(root,'RESTORE_REFERENCE_UPLOAD.json')),txt='Original firmware uploaded; verifying';
        elseif isfile(fullfile(root,'PARAMETER_RESTORE.json')),txt='Temporary parameters restored; verifying disarm and interface release';
        elseif isfile(fullfile(root,'TEMPORARY_PARAMETER_SETUP.json')),txt='Parameters set; establishing sensor / control loop';
        elseif contains(log,'REBOOT_HIL_IMU'),txt='Checking sensors and parameters';
        elseif contains(log,'USB_RC_FIRMWARE_REUSE'),txt='Initializing flight controller and session parameters';
        elseif contains(log,'REBOOT_GPENMPC'),txt='Starting flight controller; checking connection';
        elseif ~isempty(dir(fullfile(root,'*_UPLOAD_stdout.txt'))),txt='Uploading application; waiting for reconnect';
        elseif isfile(fullfile(root,'SHORT_ENTRY_PREPARED.json')),txt='Host ready; application and parameter setup';
        else,txt='Loading existing assets, interfaces and RC calibration';end
    end
    function replayMode()
        refreshRunStatus(true);if busy,return;end
        closeSocket();mode="LOADING";statusBox.String='Loading simulation recording…';drawnow;
        path=options.ReplayFile;
        try
            loaded=load(path,'trace');tr=loaded.trace;
            assert(string(tr.mission_id)=="MU_CAMBRIDGE_MA_02"&&string(tr.planner_id)=="P_ENERGY_WIND_PAYLOAD");
            assert(string(tr.method_id)=="B2_ENMPC_GP_MEAN_TOTAL_TUBE"&&tr.command_continuity_enabled);
            assert(tr.outer_period_s==.30&&tr.prediction_step_s==.20&&tr.prediction_horizon_s==1.60);
            dt=diff(double(tr.global_time_s(:)));replayBreaks=find(dt>2.5*median(dt(dt>0)))+1;
            resetPlots(true);mode="REPLAY";replayPaused=true;replayPosition=0;replayClock=tic;pauseButton.Enable='on';renderReplay();
        catch ex,mode="WAIT";statusBox.String=ex.message;end
    end
    function pauseReplay()
        if mode~="REPLAY",return;end
        if ~replayPaused,replayPosition=replayPosition+4*toc(replayClock);else,replayClock=tic;end
        replayPaused=~replayPaused;
        if replayPaused,pauseButton.String='Resume recorded playback';else,pauseButton.String='Pause recorded playback';end
    end
    function renderReplay()
        elapsed=replayPosition;if ~replayPaused,elapsed=elapsed+4*toc(replayClock);end
        times=double(tr.global_time_s(:));last=find(times<=times(1)+elapsed,1,'last');if isempty(last),last=1;end
        breaks=replayBreaks(replayBreaks<=last);ids=unique([1:10:last last reshape(breaks,1,[]) reshape(breaks-1,1,[])]);
        x=times(ids)-times(1);plotBreakRows=ismember(ids,breaks);
        setSeries(1,x,slice(tr.position_error_norm_m,ids));setSeries(2,x,slice(tr.velocity_mps,ids));
        setSeries(3,x,rad2deg(slice(tr.attitude_error_rad,ids)));setSeries(4,x,slice(tr.rotor_command_n,ids));
        setSeries(5,x,slice(tr.gp_physical_axis_authority_f,ids));
        solve=slice(tr.solver_last_elapsed_s,ids);available=slice(tr.solver_elapsed_available,ids);solve(~available)=NaN;setSeries(6,x,solve);
        state='Playing';if replayPaused,state='Paused';elseif last==numel(times),state='Record ended';end
        statusBox.String=sprintf('%s · %s · %.2f / %.2f s\n%s','Simulation playback ×4',state,times(last),times(end), ...
         'Cambridge · GP-enhanced energy-aware eNMPC with robust SE(3) control');
    end
    function value=slice(value,ids)
        n=numel(tr.global_time_s);if size(value,1)==n,value=double(value(ids,:));elseif size(value,2)==n,value=double(value(:,ids)).';else,error('Trace dimension mismatch');end
    end
    function setSeries(j,x,y)
        assert(size(y,1)==numel(x)&&size(y,2)==numel(series{j}));if mode=="REPLAY",y(plotBreakRows,:)=NaN;end
        for ch=1:size(y,2),set(series{j}(ch),'XData',x,'YData',y(:,ch));end
    end
    function openRecords()
        path=logBase;if isfolder(runRoot),path=runRoot;end;winopen(path);
    end
    function value=onoff(yes),value='off';if yes,value='on';end,end
    function closeSocket(),if ~isempty(socket),try,delete(socket);catch,end;socket=[];end,end
    function displayError(~,event)
        statusBox.String='Monitor update failed. Check the session status.';
        try,warning('%s',event.Data.message);catch,end
    end
    function closeConsole(varargin)
        for at=1:numel(ownedModels)
            name=ownedModels{at};
            if bdIsLoaded(name)&&strcmp(get_param(name,'Dirty'),'on')
                open_system(name);
                statusBox.String='A model has unsaved edits. Save or discard them in its window before closing the panel.';
                return
            end
        end
        if ~isempty(tick),try,stop(tick);delete(tick);catch,end;end
        closeSocket();
        for at=1:numel(ownedModels)
            name=ownedModels{at};
            if bdIsLoaded(name)&&strcmp(get_param(name,'Dirty'),'off'),close_system(name,0);end
        end
        if isgraphics(fig),delete(fig);end
    end
    function openModel(dynamics)
        if dynamics,name=open_gpenmpc_m600_display(false,language);else,name=open_gpenmpc_demo_connections(language);end
        ownedModels=unique([ownedModels {char(name)}]);
    end
end
