function test_gpenmpc_demo_display(operation)
% Test display handling with loopback data, an absent viewer and replay plots.
% OFFLINE is the default; display modes use isolated hidden figures.
% DIAGRAM_ONLY opens Simulink; LOOPBACK_ONLY requires the free display port 30251.
if nargin<1,operation="OFFLINE";end
operation=string(operation);
assert(isscalar(operation)&&ismember(operation,["OFFLINE","LOOPBACK_ONLY","REPLAY_ONLY","DIAGRAM_ONLY", ...
 "REPEAT_DEMO","REPEAT_PREFLIGHT_ONLY","INTERACTION_ONLY"]));
build=fileparts(fileparts(mfilename('fullpath')));addpath(fullfile(build,'m600_coptersim','matlab_validation'));
if ismember(operation,["REPEAT_DEMO","REPEAT_PREFLIGHT_ONLY"])
    checkRepeatDemo(build,string(operation)=="REPEAT_PREFLIGHT_ONLY");return
end
if operation=="INTERACTION_ONLY"
    checkInteraction(build,false);return;
end
if operation=="DIAGRAM_ONLY"
    name=open_gpenmpc_demo_connections();clean=onCleanup(@()close_system(name,0));
    assert(strcmp(get_param(name,'BlockDiagramType'),'library'));
    assert(getSimulinkBlockHandle([name '/EstimateOut'])~=-1);
    assert(strcmp(get_param([name '/EstimateOut'],'GotoTag'),'ESTIMATE'));
    fprintf('Diagram model tests passed.\n');return
end
function checkRepeatDemo(build,preflightOnly)
% Check display paths and firmware-maintenance branches.
for file={'gpenmpc_demo_console.m','start_gpenmpc_usb_manual.m','execute_m600_board_local_short.m', ...
 'm600_local_application_handoff.m','run_m600_board_local_short_hil.m', ...
 'm600_local_short_parameter_setup.m','m600_local_recovery_route.m'}
 issues=checkcode(fullfile(build,'tools',file{1}),'-id');
 bad=strcmp({issues.id},'PARSE');if any(bad),disp(issues(bad));end
 assert(~any(bad),file{1});
end
for file={'makeDisplayTruthMirror.m','makeM600CopterSimIo.m'}
 issues=checkcode(fullfile(build,'m600_coptersim','matlab_validation','+m600check',file{1}),'-id');
 assert(~any(strcmp({issues.id},'PARSE')),file{1});
end
owner=fileread(fullfile(build,'tools','execute_m600_board_local_short.m'));
install=extractBetween(owner,' if isempty(reuseSource)', ' if prepared.operator_reference');
assert(isscalar(install));split=strsplit(install{1},sprintf('\n else\n'));
upload=regexp(split{1},'uploadApplication\(''([^'']+)''','tokens','once');
assert(numel(split)==2&&~isempty(upload)&&contains(split{1},"'REBOOT_HIL_IMU'"));
distribution=fileparts(fileparts(build));
metadata=jsondecode(fileread(fullfile(distribution,'firmware',upload{1},'firmware.json')));
assert(strcmp(metadata.board,'PX4 FMUv6C')&&metadata.board_id==56);
assert(~contains(split{2},'uploadApplication(')&&contains(split{2},"'RESTART_RETAINED_HIL_APPLICATION'"));
keep=extractBetween(owner,'  if retainManualFirmware&&startupPostVerified','  elseif applicationWasAttempted()');
assert(isscalar(keep)&&~contains(keep{1},'uploadApplication(')&&contains(keep{1},'SAFE_KEEP_HIL_APPLICATION'));
fprintf('Firmware branch tests cover installation, provisioning and reuse.\n');
for file={'m600_local_short_parameter_setup.m','m600_local_recovery_route.m'}
 source=fileread(fullfile(build,'tools',file{1}));
 guards=regexp(source,'assert\(all\(isfield\(h,\{''board_adc_start_attempts''[\s\S]*?\);','match');
 assert(numel(guards)==1);
 h=struct('board_adc_start_attempts',0,'board_adc_start_verified',1, ...
  'board_adc_reused',true,'board_adc_power_witness',struct('passed',true));
 eval(guards{1});h.board_adc_power_witness.passed=false;rejected=false;
 try,eval(guards{1});catch,rejected=true;end;assert(rejected);
end
fprintf('Repeated parameter setup accepts reused ADC only with a fresh valid power witness; failed witness rejected.\n');
if preflightOnly,return;end

options=gpenmpc_demo_test_environment();
fig=gpenmpc_demo_console('LIVE','ZH',options);clean=onCleanup(@()close(fig));
assert(isequal(fig,gpenmpc_demo_console('LIVE','ZH',options)));pause(.3);
assert(~getappdata(fig,'GPENMPCObservedRunState').busy);
ax=getappdata(fig,'GPENMPCConsoleAxes');assert(numel(ax)==6);
statusControl=findall(fig,'Tag','GPENMPCDemoStatus');
assert(isscalar(statusControl));
assert(getappdata(fig,'GPENMPCObservedRunState').phase=="IDLE");
sender=udpport('datagram','IPV4','LocalHost','127.0.0.1');mc=onCleanup(@()delete(sender));
port=getappdata(fig,'GPENMPCDisplayPort');assert(port>0&&port~=30251);
truth=zeros(112,1,'uint8');truth(1:12)=typecast(int32([1234567891 1 5]),'uint8');
env=zeros(28,1);env(1)=2;env(2)=77;env(5)=2.21;env(6:7)=[1.25;-.8];env(23)=26090501;
envBytes=[typecast(int32([1234567897;1]),'uint8');typecast(env,'uint8')];
diag=zeros(32,1);diag(3)=10;diag(7)=1;diag(26)=2;diag(27)=env(23);diag(28)=env(2);diag(30)=2.21;diag(31)=11.71;
db=[typecast(int32([1234567890;1]),'uint8');typecast(diag,'uint8')];
truth(105:112)=typecast(double(10),'uint8');
for packet={envBytes,db,truth},write(sender,packet{1},'uint8','127.0.0.1',port);end
pause(.35);lines=findall(ax(5),'Type','line');
values=arrayfun(@(h)h.YData(end),lines);assert(isequal(sort(values(:)),sort([1.25;-.8])));
% Leave a gap for mismatched application generations.
diag(28)=78;diag(3)=10.3;db=[typecast(int32([1234567890;1]),'uint8');typecast(diag,'uint8')];
truth(105:112)=typecast(double(10.3),'uint8');
for packet={envBytes,db,truth},write(sender,packet{1},'uint8','127.0.0.1',port);end
pause(.35);
assert(all(arrayfun(@(h)isnan(h.YData(end)),lines)));
runRoot=fullfile(options.LogRoot,'manual_session_001');mkdir(runRoot);mkdir(fullfile(runRoot,'SHORT_HIL'));
writeFixture(fullfile(runRoot,'MATLAB_RUN.log'),'USB_RC_STOP: fixture');
writeFixture(fullfile(runRoot,'OUTER_SHORT_RESULT.json'),jsonencode(struct('safe',true)));
result=struct('failure',struct('message','BOARD_LOCAL_CONTEXT_STOPPED:7'), ...
 'counts',struct('commits',17),'first_commit_io_s',1,'last_commit_io_s',3.5);
writeFixture(fullfile(runRoot,'SHORT_HIL','RESULT.json'),jsonencode(result));pause(1.2);
assert(getappdata(fig,'GPENMPCObservedRunState').phase=="STOPPED");
statusText=string(statusControl.String);assert(any(contains(statusText,'17'))&&any(contains(statusText,'2.5')));
fprintf('Display packet and generation-mismatch tests passed.\n');
clear mc clean
end
files={fullfile(build,'tools','gpenmpc_demo_console.m'),fullfile(build,'tools','open_gpenmpc_demo_connections.m'), ...
 fullfile(build,'m600_coptersim','matlab_validation','+m600check','makeDisplayTruthMirror.m')};
for k=1:numel(files)
 findings=checkcode(files{k},'-id');
 assert(~any(strcmp({findings.id},'PARSE')),sprintf('Parse error: %s',files{k}));
end
checkDiagram(build);
if operation=="OFFLINE"
 checkRepeatDemo(build,true);checkInteraction(build,true);checkRebootStatus(build);
 disabled=m600check.makeDisplayTruthMirror(0);assert(disabled.status().disabled);disabled.close();
 fprintf('Offline display tests passed.\n');return
end

function checkInteraction(build,guardsOnly)
source=fileread(fullfile(build,'tools','prepare_m600_board_local_short_objects.m'));
guard=regexp(source,'assert\(all\(isfield\(c,[\s\S]*?Keep the original short-run bounds and independent full raw retention\.''\);','match','once');
assert(~isempty(guard));
c=struct('duration_s',0,'poll_period_s',.001,'heartbeat_period_s',.05,'preparation_timeout_s',10, ...
 'getter_capacity',16,'environment_ledger_capacity',16,'raw_capacity',1000, ...
 'component_initialization',true,'service_cfg',struct('operator_reference',struct()));
eval(guard);c.duration_s=120;eval(guard);c.duration_s=20;eval(guard);c.duration_s=40;eval(guard);
c.duration_s=0;c.component_initialization=false;rejected=false;
try,eval(guard);catch ex,rejected=strcmp(ex.identifier,'gpenmpcShort:Bounds');end
assert(rejected);c.component_initialization=true;c.service_cfg=struct();rejected=false;
try,eval(guard);catch ex,rejected=strcmp(ex.identifier,'gpenmpcShort:Bounds');end
assert(rejected);
if guardsOnly,return;end
options=gpenmpc_demo_test_environment();
fig=gpenmpc_demo_console('LIVE','ZH',options);clean=onCleanup(@()closeTestPanel());pause(.3);
assert(isempty(findobj(fig,'Type','line','-and','YData',1))); % Require an empty live display.
assert(getappdata(fig,'GPENMPCObservedRunState').phase=="IDLE");
assert(isscalar(findall(fig,'Tag','GPENMPCDemoStatus')));
close(fig);fig=gpenmpc_demo_console('REPLAY','ZH',options);
curves=findall(fig,'Type','line');before=get(curves,'XData');pause(1);after=get(curves,'XData');assert(isequaln(before,after));
play=findall(fig,'Tag','GPENMPCReplayToggle');assert(isscalar(play)&&strcmp(play.Enable,'on'));
feval(play.Callback,play,[]);pause(1);afterPlay=get(curves,'XData');assert(~isequaln(after,afterPlay));
feval(play.Callback,play,[]);pause(.25);paused=get(curves,'XData');pause(.5);assert(isequaln(paused,get(curves,'XData')));
checkDiagram(build);clear clean
fprintf('Display interaction tests passed.\n');
end
if operation=="LOOPBACK_ONLY"
sink=udpport('datagram','IPV4','LocalHost','127.0.0.1','LocalPort',30251,'Timeout',.1);
sinkCleanup=onCleanup(@()delete(sink));
mirror=m600check.makeDisplayTruthMirror(30251);mirrorCleanup=onCleanup(@()mirror.close());
bytes=zeros(1,112,'uint8');bytes(1:12)=typecast(int32([1234567891 1 5]),'uint8');
bytes(105:112)=typecast(double(10),'uint8');
assert(m600check.decodeTruthPacket(bytes).valid);
mirror.send(bytes,1);mirror.send(bytes,1.01);mirror.send(bytes,1.25);pause(.05);
packets=read(sink,sink.NumDatagramsAvailable,'uint8');count=0;
for k=1:numel(packets)
 if istable(packets),b=packets.Data{k};else,b=packets(k).Data;end
 if numel(b)==112,assert(isequal(uint8(b(:)),bytes(:)));count=count+1;end
end
state=mirror.status();assert(count==2&&state.sent==2&&~state.disabled);
disp(state);clear sinkCleanup sink;
mirror.send(bytes,1.5); % An absent renderer is non-fatal.
clear mirrorCleanup mirror;
fprintf('Display mirror loopback tests passed.\n');return
end
options=gpenmpc_demo_test_environment();
fig=gpenmpc_demo_console('REPLAY','ZH',options);clean=onCleanup(@()close(fig));pause(1);drawnow;
assert(getappdata(fig,'GPENMPCObservedRunState').mode=="REPLAY",'Replay fixture failed to load.');
ax=findall(fig,'Type','axes');assert(numel(ax)==6);
for k=1:numel(ax)
 curves=findall(ax(k),'Type','line');assert(any(arrayfun(@(h)any(isfinite(h.YData)),curves)));
end
checkDiagram(build);clear clean
fprintf('Replay plot tests passed.\n');
end
function closeTestPanel()
fig=findall(groot,'Type','figure','Tag','GPENMPCDemoConsole');
if ~isempty(fig),close(fig);end
end
function checkDiagram(build)
source=fileread(fullfile(build,'tools','open_gpenmpc_demo_connections.m'));
assert(contains(source,"new_system(name,'Library')"));
assert(contains(source,"tag('EstimateOut','Goto','ESTIMATE'")&&contains(source,"tag('EstimateIn','From','ESTIMATE'"));
assert(contains(source,"wire('FlightController',2,'EstimateOut',1")&&contains(source,"wire('EstimateIn',1,'HostReference',2"));
end
function writeFixture(path,text)
fid=fopen(path,'w');assert(fid>=0);clean=onCleanup(@()fclose(fid));fprintf(fid,'%s',text);
end
function checkRebootStatus(build)
source=fileread(fullfile(build,'tools','gpenmpc_demo_console.m'));
body=extractBetween(source,'    function txt=progressText(log,root)','    function replayMode()');
assert(isscalar(body));body=regexprep(body{1},'\s*end\s*$','');
log='REBOOT_GPENMPC';root=tempname;T=@(~,en)en;txt=''; %#ok<NASGU>
eval(body);
assert(strcmp(txt,'Starting flight controller; checking connection'));
end
