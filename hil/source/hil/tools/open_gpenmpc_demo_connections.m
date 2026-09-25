function name=open_gpenmpc_demo_connections(language) %#ok<INUSD>
% Open the teaching connection library without changing the executable model.
% The optional language argument is retained for callers; the diagram is English.
name='Hexarotor_HIL';
target=fullfile(gpenmpc_external_path('host_recording_root'),'Demo_UI_ARTIFACTS');
if ~isfolder(target),mkdir(target);end
if bdIsLoaded(name)
 assert(strcmp(get_param(name,'Dirty'),'off'),'Save your diagram edits before reopening.');
 set_param(name,'Lock','off');open_system(name);set_param(name,'ZoomFactor','FitSystem');set_param(name,'Lock','on');save_system(name);return;
end
new_system(name,'Library');set_param(name,'Lock','off');
block('Transmitter',[30 175 185 275],sprintf('FS-i6S transmitter\nUSB stick input'),0,1);
block('HostReference',[285 150 505 300],sprintf('MATLAB\nStick → velocity / yaw-rate commands\nSource and freshness checks'),2,1);
block('FlightController',[635 150 885 300],sprintf('Pixhawk 6C\nReference shaping and robust SE(3)\nSix-rotor allocation\nInner-loop target: 100 Hz'),2,2);
block('Dynamics',[1030 150 1260 300],sprintf('CopterSim\nM600 6-DoF dynamics\nTranslation + rotation\nSensors / 3DOutput'),1,3);
block('Visualization',[1420 155 1645 280],sprintf('RflySim3D\n3D flight visualization\nM600 position and attitude'),1,0);
block('Monitor',[1420 465 1645 560],sprintf('MATLAB live dashboard\nFlight states\nEnvironment and response'),1,0);
wire('Transmitter',1,'HostReference',1,240,'Four stick axes');
wire('HostReference',1,'FlightController',1,570,'Velocity / yaw-rate commands');
wire('FlightController',1,'Dynamics',1,955,'Six virtual inputs');
wire('Dynamics',1,'Visualization',1,1340,'3DOutput');
tag('EstimateOut','Goto','ESTIMATE',[920 345 1090 380]);
tag('EstimateIn','From','ESTIMATE',[70 345 240 380]);
wire('FlightController',2,'EstimateOut',1,905,'Estimated state');
wire('EstimateIn',1,'HostReference',2,265,'');
tag('SensorOut','Goto','HIL_SENSOR',[1320 405 1490 440]);
tag('SensorIn','From','HIL_SENSOR',[430 405 600 440]);
wire('Dynamics',3,'SensorOut',1,1280,'Simulated sensors');
wire('SensorIn',1,'FlightController',2,615,'');
tag('DisplayOut','Goto','DISPLAY_COPY',[1320 325 1490 360]);
tag('DisplayIn','From','DISPLAY_COPY',[1170 495 1340 530]);
wire('Dynamics',2,'DisplayOut',1,1300,'Live state');
wire('DisplayIn',1,'Monitor',1,1375,'');
note([30 25],'GPENMPC HIL',22);
note([30 80],'System connections · Stick input, onboard control, dynamics and live visualization',12);
note([30 500],sprintf('Manual commands → Onboard reference shaping → Robust SE(3) tracking → Six-rotor allocation\nSimulated sensor feedback closes the hardware in the loop system.'),13);
open_system(name);set_param(name,'Location',[40 60 1750 870]);drawnow;set_param(name,'ZoomFactor','FitSystem');
set_param(name,'Lock','on');save_system(name,fullfile(target,[name '.slx']));
    function block(id,pos,caption,nin,nout)
        p=[name '/' id];add_block('built-in/Subsystem',p,'Position',pos,'FontName','Microsoft YaHei', ...
         'FontSize','14','ShowName','off','BackgroundColor','white');
        for k=1:nin,add_block('built-in/Inport',sprintf('%s/In%d',p,k),'Port',num2str(k));end
        for k=1:nout,add_block('built-in/Outport',sprintf('%s/Out%d',p,k),'Port',num2str(k));end
        set_param(p,'Mask','on','MaskDisplay',sprintf('disp(sprintf(''%s''))',strrep(caption,newline,'\n')));
    end
    function tag(id,kind,txt,pos)
        add_block(['built-in/' kind],[name '/' id],'GotoTag',txt,'Position',pos,'ShowName','off','FontSize','11');
    end
    function wire(src,sp,dst,dp,bend,txt)
        a=get_param([name '/' src],'PortHandles');b=get_param([name '/' dst],'PortHandles');
        p=get_param(a.Outport(sp),'Position');q=get_param(b.Inport(dp),'Position');
        h=add_line(name,[p;bend p(2);bend q(2);q]);set_param(h,'Name',txt,'FontSize','11');
    end
    function note(pos,txt,sz)
        a=Simulink.Annotation(name,txt);a.Position=pos;a.FontName='Microsoft YaHei';a.FontSize=sz;
    end
end
