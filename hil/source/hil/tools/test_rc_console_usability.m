function test_rc_console_usability()
% Test console layout and persistence ordering.
build=fileparts(fileparts(mfilename('fullpath')));toolsRoot=fullfile(build,'tools');
for file={'gpenmpc_demo_console.m','save_m600_short_raw.m','run_m600_board_local_short_hil.m','execute_m600_board_local_short.m','start_gpenmpc_usb_manual.m','open_gpenmpc_demo_connections.m','open_gpenmpc_m600_display.m'}
 findings=checkcode(fullfile(toolsRoot,file{1}),'-id');assert(~any(strcmp({findings.id},'PARSE')),file{1});
end
owner=fileread(fullfile(toolsRoot,'execute_m600_board_local_short.m'));
assert(strfind(owner,'finish();clear cleanup')<strfind(owner,'short=safetyOwner.persistRaw()'));
assert(contains(owner,'PREPARE_AND_LIVE')&&contains(owner,'prepared);'));
entry=fileread(fullfile(toolsRoot,'start_gpenmpc_usb_manual.m'));
assert(numel(strfind(entry,'result=execute_m600_board_local_short('))==1);
f=gpenmpc_demo_console('IDLE','EN');cleanup=onCleanup(@()closeIfOpen(f));pause(1.1);
assert(contains(f.Name,'GPENMPC'));
start=findall(f,'Tag','GPENMPCManualStart');reset=findall(f,'Tag','GPENMPCManualReset');
assert(strcmp(start.Enable,'on'));
% Use the existing exclusive launch lock.
p=fullfile(toolsRoot,'usb_rc_runtime','manual_session.lock');
h=System.IO.FileStream(p,System.IO.FileMode.Open,System.IO.FileAccess.ReadWrite,System.IO.FileShare.None);
lockCleanup=onCleanup(@()h.Dispose());pause(1.2);
assert(strcmp(start.Enable,'off')&&strcmp(reset.Enable,'off'));
clear lockCleanup;pause(1.2);assert(strcmp(start.Enable,'on'));
close(f);clear cleanup
for lang=["ZH","EN"]
 name=open_gpenmpc_demo_connections(lang);
 assert(strcmp(get_param(name,'BlockDiagramType'),'library'));
 assert(numel(find_system(name,'SearchDepth',1,'BlockType','SubSystem'))==6);
 assert(getSimulinkBlockHandle([name '/SensorOut'])~=-1);
 assert(startsWith(get_param(name,'FileName'),'E:'));close_system(name,0);
 name=open_gpenmpc_m600_display(false,lang);
 assert(startsWith(get_param(name,'FileName'),'E:'));
 source=fullfile(build,'m600_coptersim','model','generated','GPENMPC_M600_Canonical.slx');
 load_system(source);old='GPENMPC_M600_Canonical';
 a=find_system(old,'SearchDepth',1,'Type','Block');b=find_system(name,'SearchDepth',1,'Type','Block');
 assert(numel(a)==numel(b));
 aNames=cellfun(@(s)extractAfter(s,strlength(old)+1),a,'UniformOutput',false);
 bNames=cellfun(@(s)extractAfter(s,strlength(name)+1),b,'UniformOutput',false);assert(isequal(sort(aNames),sort(bNames)));
 assert(numel(find_system(old,'FindAll','on','Type','line'))==numel(find_system(name,'FindAll','on','Type','line')));
 close_system(old,0);close_system(name,0);
end
fprintf('Console usability tests passed: labels, explicit start, duplicate-start protection, reset locking and recovery-before-save.\n');
end
function closeIfOpen(f),if isgraphics(f),close(f);end,end
