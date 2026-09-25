function report=test_mavlink_poll_observation_latency(outputRoot)
% Compare callback delivery and latestmsgs on one mavlinkio subscription
% using localhost telemetry fixtures.
build=fileparts(fileparts(mfilename('fullpath')));addpath(fullfile(build,'host_runtime'));
out=fullfile(outputRoot,'MAVLINK_OBSERVATION_LATENCY.json');assert(~isfile(out));
d=mavlinkdialect(fullfile(build,'m600_coptersim','matlab_validation','+m600check','px4_health_events.xml'));
receiver=[];serializer=[];peer=[];sub=[];clean=onCleanup(@finish);
n=40;sendNs=zeros(n,1,'uint64');callbackNs=sendNs;pollNs=sendNs;frames=cell(n,1);callbackMessages=cell(n,1);pollMessages=cell(n,1);
receiver=mavlinkio(d,'SystemID',255,'ComponentID',190);connect(receiver,'UDP','LocalPort',62285);
client=mavlinkclient(receiver,1,1);sub=mavlinksub(receiver,client,'TUNNEL','BufferSize',64,'NewMessageFcn',@arrived);
serializer=mavlinkio(d,'SystemID',1,'ComponentID',1);peer=udpport('datagram','IPV4','LocalPort',62286);
for k=1:n
    m=createmsg(d,'TUNNEL');m.Payload.payload_type=uint16(42002);m.Payload.target_system=uint8(255);
    m.Payload.target_component=uint8(190);m.Payload.payload_length=uint8(2);m.Payload.payload(:)=0;
    m.Payload.payload(1)=uint8(250);m.Payload.payload(2)=uint8(k);frames{k}=uint8(serializemsg(serializer,m));
end
t=tic;next=1;
while toc(t)<3
    if next<=n&&toc(t)>=(next-1)*.01
        sendNs(next)=gpenmpcNative.rflyOriginalHostMonotonicNs();write(peer,frames{next},'uint8','127.0.0.1',62285);next=next+1;
    end
    msgs=latestmsgs(sub,64);observed=gpenmpcNative.rflyOriginalHostMonotonicNs();
    for j=1:numel(msgs)
        k=double(msgs(j).Payload.payload(2));assert(k>=1&&k<=n);
        if pollNs(k)==0,pollNs(k)=observed;pollMessages{k}=msgs(j);end
    end
    if next>n&&all(pollNs>0)&&all(callbackNs>0),break;end
    pause(.001);
end
complete=all(sendNs>0)&all(pollNs>=sendNs)&all(callbackNs>=sendNs);
assert(complete&&isequaln(callbackMessages,pollMessages),'test:PublicObservationMismatch');
report=struct('scope','SAME_PUBLIC_MAVLINKIO_SUBSCRIPTION_HOST_LOCALHOST_ONLY', ...
    'sample_count',n,'complete',complete,'callback_and_poll_exact_messages_equal',true, ...
    'send_ns',sendNs,'callback_first_observation_ns',callbackNs,'poll_first_observation_ns',pollNs, ...
    'callback_latency_ms',double(callbackNs-sendNs)/1e6,'poll_latency_ms',double(pollNs-sendNs)/1e6, ...
    'poll_timestamp_semantics','ACTUAL_FIRST_PUBLIC_LATESTMSGS_RETURN_NOT_CALLBACK_OR_NIC_TIME', ...
    'hardware_actions',0,'COM',0,'plant_runs',0,'control_publications',0);
% Execute cleanup once while the shared nested workspace is still alive.
clear clean
f=fopen(out,'w');assert(f>=0);c=onCleanup(@()fclose(f));fprintf(f,'%s\n',jsonencode(report,PrettyPrint=true));
disp(jsonencode(struct('samples',n,'same_messages',true,'callback_median_ms',median(report.callback_latency_ms), ...
    'poll_median_ms',median(report.poll_latency_ms),'callback_max_ms',max(report.callback_latency_ms),'poll_max_ms',max(report.poll_latency_ms))));
    function arrived(~,msgs)
        stamp=gpenmpcNative.rflyOriginalHostMonotonicNs();
        for jj=1:numel(msgs),kk=double(msgs(jj).Payload.payload(2));assert(callbackNs(kk)==0);callbackNs(kk)=stamp;callbackMessages{kk}=msgs(jj);end
    end
    function finish()
        if ~isempty(sub),delete(sub);sub=[];end
        if ~isempty(receiver),delete(receiver);receiver=[];end
        if ~isempty(serializer),delete(serializer);serializer=[];end
        if ~isempty(peer),delete(peer);peer=[];end
    end
end
