function result=analyze_sensor_aligned_reboot_raw(runRoot,outputJson)
% Parse retained sensor-aligned reboot records.
assert(~isfile(outputJson),'audit:Existing','Choose an unused output path.');
a=load(fullfile(runRoot,'INITIALIZATION_MATLAB','RAW_INITIALIZATION_OBSERVATION.mat'));
b=load(fullfile(runRoot,'UDP_RECOVERY_RAW.mat'));
result=struct('schema','HOST_SENSOR_REBOOT_RAW_AUDIT_V1','run_root',runRoot, ...
    'COM_UDP_board_actions',0,'pre_reset',summarize(a.result.transport_evidence), ...
    'recovery',summarize(b.receipt.evidence));
fid=fopen(outputJson,'w','n','UTF-8');assert(fid>=0);c=onCleanup(@()fclose(fid));
fprintf(fid,'%s\n',jsonencode(result,PrettyPrint=true));clear c
disp(jsonencode(result,PrettyPrint=true));
end

function r=summarize(e)
raw=e.raw_mavlink; topics=cellfun(@(q)q.topic,raw,'UniformOutput',false);
u=unique(topics); counts=struct('topic',{},'count',{});
for k=1:numel(u),counts(end+1)=struct('topic',u{k},'count',sum(strcmp(topics,u{k})));end
r=struct('raw_count',numel(raw),'counts',counts,'streams',struct(), ...
    'statustext',{{}},'command_ack',{{}},'heartbeat',{{}},'transmit_topics',{{}}, ...
    'fatal',e.fatal,'first_fatal',e.first_fatal);
defs={'ATTITUDE',{'roll','pitch','yaw','rollspeed','pitchspeed','yawspeed'}; ...
    'HIGHRES_IMU',{'xacc','yacc','zacc','xgyro','ygyro','zgyro','xmag','ymag','zmag','id'}; ...
    'LOCAL_POSITION_NED',{'x','y','z','vx','vy','vz'}; ...
    'ESTIMATOR_STATUS',{'flags','vel_ratio','pos_horiz_ratio','pos_vert_ratio','mag_ratio','hagl_ratio'}};
for k=1:size(defs,1)
    topic=defs{k,1};fields=defs{k,2};q=raw(strcmp(topics,topic));
    data=NaN(numel(q),numel(fields));rx=NaN(numel(q),1);src=NaN(numel(q),1);
    for j=1:numel(q)
        p=q{j}.message.Payload;rx(j)=double(q{j}.rx_s);
        if isfield(p,'time_usec'),src(j)=double(p.time_usec)/1e6;
        elseif isfield(p,'time_boot_ms'),src(j)=double(p.time_boot_ms)/1e3;end
        for f=1:numel(fields),if isfield(p,fields{f}),data(j,f)=double(p.(fields{f}));end,end
    end
    v=struct('count',numel(q),'fields',{fields},'rx_first',[],'rx_last',[], ...
        'source_first',[],'source_last',[],'source_reversals',sum(diff(src)<0), ...
        'mean',mean(data,1,'omitnan'),'std',std(data,0,1,'omitnan'), ...
        'min',min(data,[],1),'max',max(data,[],1),'first_values',[],'last_values',[], ...
        'first_3s_mean',[],'last_3s_mean',[],'last_3s_std',[],'last_3s_count',0);
    if ~isempty(q)
        v.rx_first=rx(1);v.rx_last=rx(end);v.source_first=src(1);v.source_last=src(end);
        v.first_values=data(1,:);v.last_values=data(end,:);
        v.first_3s_mean=mean(data(rx<=rx(1)+3,:),1,'omitnan');
        v.last_3s_mean=mean(data(rx>=rx(end)-3,:),1,'omitnan');
        v.last_3s_std=std(data(rx>=rx(end)-3,:),0,1,'omitnan');v.last_3s_count=sum(rx>=rx(end)-3);
    end
    r.streams.(topic)=v;
end
for k=1:numel(raw)
    q=raw{k};p=q.message.Payload;
    switch q.topic
        case 'STATUSTEXT'
            p.text=char(p.text(:)');p.text=p.text(p.text~=0);
            r.statustext{end+1}=struct('rx_s',q.rx_s,'payload',p);
        case 'COMMAND_ACK'
            r.command_ack{end+1}=struct('rx_s',q.rx_s,'payload',p);
        case 'HEARTBEAT'
            r.heartbeat{end+1}=struct('rx_s',q.rx_s,'payload',p);
    end
end
if isfield(e,'raw_transmit_messages')
    tx=e.raw_transmit_messages;
    for k=1:numel(tx)
        r.transmit_topics{end+1}=tx{k};
    end
end
end
