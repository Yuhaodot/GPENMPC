function name=open_gpenmpc_m600_display(relayout,language) %#ok<INUSD>
% Open a layout-only copy of the CopterSim source model.
if nargin==0,relayout=false;end
% The optional language argument is retained for callers; the display is English.
toolsRoot=fileparts(mfilename('fullpath'));
source=fullfile(toolsRoot,'GPENMPC_M600_Display.slx');
name='M600_Model';
displayRoot=gpenmpc_external_path('demo_artifact_root');
if ~isfolder(displayRoot),mkdir(displayRoot);end
target=fullfile(displayRoot,[name '.slx']);
fresh=~isfile(target);
if fresh
 [ok,msg]=copyfile(source,target);assert(ok,'%s',msg);
end
load_system(target);
wasClean=strcmp(get_param(name,'Dirty'),'off');
if fresh||relayout
 assert(strcmp(get_param(name,'Dirty'),'off'),'gpenmpcDemo:PreserveDrawing','Save unsaved model edits before relayout.');
 blocks=find_system(name,'SearchDepth',1,'Type','Block');
 for k=1:numel(blocks)
  set_param(blocks{k},'FontName','Microsoft YaHei','FontSize','16');
 end
 % Give the real 19-output dynamics function enough room for all port labels.
 set_param([name '/CurrentM600_10ms'],'Position',[390 150 640 750],'BackgroundColor','[0.87 0.94 0.91]');
 set_param([name '/SensorOutput'],'Position',[1050 180 1270 360],'BackgroundColor','[0.91 0.95 1]');
 set_param([name '/3DOutput'],'Position',[1050 600 1270 760],'BackgroundColor','[0.97 0.94 0.86]');
 % Update display layout.
 Simulink.BlockDiagram.arrangeSystem(name,'FullLayout','true');
 positions=cell2mat(get_param(blocks,'Position'));
 if fresh
 caption=modelCaption();
 annotations=find_system(name,'SearchDepth',1,'FindAll','on','Type','annotation');
 a=[];
 for k=1:numel(annotations)
  candidate=get_param(annotations(k),'Object');
  if startsWith(string(candidate.Text),"M600 Model")
   a=candidate;break;
  end
 end
 if isempty(a),a=Simulink.Annotation(name,caption);else,a.Text=caption;end
 a.Position=[min(positions(:,1)) min(positions(:,2))-115];
 a.FontName='Microsoft YaHei';a.FontSize=14;
 end
 save_system(name,target);
end
% Update the heading only in a clean display copy, preserving unsaved edits.
if wasClean&&~fresh
 annotations=find_system(name,'SearchDepth',1,'FindAll','on','Type','annotation');
 for k=1:numel(annotations)
  a=get_param(annotations(k),'Object');text=string(a.Text);
  if startsWith(text,"M600 SIX-DEGREE-OF-FREEDOM DYNAMICS")
   a.Text=modelCaption();
  end
 end
end
open_system(name);set_param(name,'Location',[45 55 1845 970]);drawnow;
set_param(name,'ZoomFactor','FitSystem');
if wasClean,save_system(name,target);end % Persist the clean display layout.
end
function caption=modelCaption()
caption=sprintf('M600 Model\nRotor response · Translation and rotation · Simulated sensors · 3D output');
end
