function layout_m600_demo_model()
% Prepare model presentation geometry. Open the saved layout for routine viewing.
root=gpenmpc_external_path('demo_artifact_root');
path=fullfile(root,'M600_Model.slx');
assert(isfile(fullfile(root,'M600_Model_before_layout_20260916.slx')));
name='M600_Model';load_system(path);
assert(strcmp(get_param(name,'Dirty'),'off'),'Save the current model before arranging it.');
open_system(name);drawnow;
blocks=find_system(name,'SearchDepth',1,'Type','Block');
before=connections(name);
for k=1:numel(blocks)
    set_param(blocks{k},'FontName','Arial','FontSize','18');
end
place('CurrentM600_10ms',[370 140 810 800]);
ports=get_param([name '/CurrentM600_10ms'],'PortHandles');
inputs={'inPWMs','InitialPosition','InitialEuler','TerrainIn15d','inDoubCtrls','DeliveryIoClock'};
for k=1:numel(inputs)
    p=get_param(ports.Inport(k),'Position');y=p(2);
    if ismember(k,[2 3]),pos=[90 y-17 265 y+17];
    elseif k==6,pos=[235 y-12 259 y+12];
    else,pos=[225 y-8 255 y+8];end
    place(inputs{k},pos);
end
firstOut=get_param(ports.Outport(1),'Position');
secondOut=get_param(ports.Outport(2),'Position');
dy=secondOut(2)-firstOut(2);
place('Kinematics',[1080 firstOut(2)-dy/2 1085 firstOut(2)+11.5*dy]);
place('SensorOutput',[1240 180 1465 360]);
place('3DOutput',[1240 470 1465 650]);
place('VehicleType',[1120 483 1205 517]);
place('VirtualSensorBoardLevelInverse',[1540 195 1610 255]);
place('HILSensor30d',[1740 217 1770 233]);
place('HILGPS30d',[1570 307 1600 323]);
place('VehileInfo60d',[1570 552 1600 568]);
observers={'outCopterData','RotorObserverStateN','RotorObserverSimTimeS', ...
    'RotorObserverGeneration','RotorObserverSession','RotorObserverValid','RotorObserverFailed'};
for k=1:numel(observers)
    p=get_param(ports.Outport(k+12),'Position');
    y=p(2);place(observers{k},[1045 y-8 1075 y+8]);
    set_param([name '/' observers{k}],'FontSize','14');
end
set_param([name '/Kinematics'],'ShowName','off');
set_param([name '/VirtualSensorBoardLevelInverse'],'ShowName','off');
note('Kinematics',[1030 107],16);
note(sprintf('Sensor frame\nconversion'),[1540 270],16);
% Leave room for both port captions around the converter.
place('Flat Earth to LLA',[105 875 335 940]);
place('GeoAltitude',[475 910 665 944]);
set_param([name '/GeoAltitude'],'BlockMirror','on');
lla=get_param([name '/Flat Earth to LLA'],'PortHandles');
gps1=get_param(lla.Outport(1),'Position');gps2=get_param(lla.Outport(2),'Position');
gpsGap=gps2(2)-gps1(2);
place('GPSpos',[45 gps1(2)-gpsGap/2 50 gps2(2)+gpsGap/2]);
set_param([name '/GPSpos'],'BlockMirror','on');
drawnow;
% Align the constant after the editor resolves masked port positions.
lla=get_param([name '/Flat Earth to LLA'],'PortHandles');
alt=get_param([name '/GeoAltitude'],'PortHandles');
targetPort=get_param(lla.Inport(2),'Position');actualPort=get_param(alt.Outport(1),'Position');
pos=get_param([name '/GeoAltitude'],'Position');
place('GeoAltitude',pos+[0 targetPort(2)-actualPort(2) 0 targetPort(2)-actualPort(2)]);
annotations=find_system(name,'SearchDepth',1,'FindAll','on','Type','annotation');
for k=1:numel(annotations)
    a=get_param(annotations(k),'Object');
    if startsWith(string(a.Text),'M600')
        a.Text=sprintf('M600 Model\nRotor response, aircraft motion and simulated sensors');
        a.Position=[85 25];a.FontName='Arial';a.FontSize=20;
    end
end
% Set drawing points explicitly while preserving line endpoints.
lines=find_system(name,'FindAll','on','SearchDepth',1,'Type','line');
for k=1:numel(lines)
    src=get_param(lines(k),'SrcPortHandle');
    dst=get_param(lines(k),'DstPortHandle');
    if src<0||numel(dst)~=1||dst<0,continue;end
    s=get_param(src,'Position');d=get_param(dst,'Position');
    if s(2)==d(2),points=[s;d];
    else,x=round((s(1)+d(1))/2);points=[s;x s(2);x d(2);d];end
    set_param(lines(k),'Points',points);
end
% Route coordinate-conversion lines outside the model.
route('Flat Earth to LLA',1,'GPSpos',1,80);
route('Flat Earth to LLA',2,'GPSpos',2,80);
g=get_param([name '/GPSpos'],'PortHandles');
b=get_param([name '/Kinematics'],'PortHandles');
s=get_param(g.Outport(1),'Position');d=get_param(b.Inport(11),'Position');
l=get_param(g.Outport(1),'Line');
set_param(l,'Points',[s;15 s(2);15 90;1020 90;1020 d(2);d]);
% Two existing fan-outs retain their branch objects and source identities.
routeBranch(ports.Outport(2),[850 secondOut(2)],'Flat Earth to LLA', ...
    [850 secondOut(2);850 120;350 120;350 895]);
busOut=get_param(b.Outport(1),'Position');
routeBranch(b.Outport(1),[1110 busOut(2)],'',[]);
animation=get_param(ports.Outport(12),'Position');
v=get_param([name '/3DOutput'],'PortHandles');d=get_param(v.Inport(2),'Position');
set_param(get_param(ports.Outport(12),'Line'),'Points', ...
    [animation;835 animation(2);835 d(2);d]);
assert(isequal(before,connections(name)),'Presentation layout changed signal endpoints.');
open_system(name);set_param(name,'Location',[45 55 1845 970]);drawnow;
set_param(name,'ZoomFactor','FitSystem');save_system(name,path);
print(['-s' name],'-dpng','-r130',fullfile(root,'M600_Model_layout.png'));
fprintf('M600 presentation layout saved; %d signal connections retained.\n',numel(before));
close_system(name,0);
    function place(block,pos),set_param([name '/' block],'Position',pos);drawnow;end
    function note(txt,pos,sz)
        found=find_system(name,'SearchDepth',1,'FindAll','on','Type','annotation');
        a=[];
        for n=1:numel(found)
            candidate=get_param(found(n),'Object');
            if strcmp(candidate.Text,txt),a=candidate;break;end
        end
        if isempty(a),a=Simulink.Annotation(name,txt);end
        a.Position=pos;a.FontName='Arial';a.FontSize=sz;
    end
    function route(srcBlock,srcNum,dstBlock,dstNum,x)
        a=get_param([name '/' srcBlock],'PortHandles');
        z=get_param([name '/' dstBlock],'PortHandles');
        s=get_param(a.Outport(srcNum),'Position');d=get_param(z.Inport(dstNum),'Position');
        if s(2)==d(2),pts=[s;d];else,pts=[s;x s(2);x d(2);d];end
        set_param(get_param(a.Outport(srcNum),'Line'),'Points',pts);
    end
    function routeBranch(src,junction,special,via)
        l=get_param(src,'Line');s=get_param(src,'Position');
        children=get_param(l,'LineChildren');
        assert(numel(children)==2,'Expected the existing two-way signal branch.');
        set_param(l,'Points',[s;junction]);
        for q=1:numel(children)
            dst=get_param(children(q),'DstPortHandle');assert(isscalar(dst)&&dst>0);
            d=get_param(dst,'Position');parent=get_param(dst,'Parent');
            if ~isempty(special)&&strcmp(parent,[name '/' special])
                pts=[via(1:end-1,:);via(end,1) d(2);d];
            else
                x=round((junction(1)+d(1))/2);
                if strcmp(parent,[name '/3DOutput']),x=1220;end
                pts=[junction;x junction(2);x d(2);d];
            end
            if junction(2)==d(2),pts=[junction;d];end
            set_param(children(q),'Points',pts);
        end
    end
end

function edges=connections(name)
edges=strings(0,1);blocks=find_system(name,'SearchDepth',1,'Type','Block');
for k=1:numel(blocks)
    p=get_param(blocks{k},'PortConnectivity');
    for j=1:numel(p)
        if isempty(p(j).DstBlock),continue;end
        for n=1:numel(p(j).DstBlock)
            edges(end+1,1)=string(get_param(blocks{k},'SID'))+":"+string(p(j).Type)+">"+ ...
                string(get_param(p(j).DstBlock(n),'SID'))+":"+string(p(j).DstPort(n)); %#ok<AGROW>
        end
    end
end
edges=sort(edges);
end
