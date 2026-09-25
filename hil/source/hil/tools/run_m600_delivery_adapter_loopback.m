function report=run_m600_delivery_adapter_loopback(outputDir)
% Test the delivery adapter with localhost MAVLink and UDP fixtures.
arguments,outputDir (1,1) string,end
assert(~isfolder(outputDir));mkdir(outputDir);
root=fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(root,'matlab_validation'),fullfile(root,'m600_coptersim','matlab_validation'));
ports=48271:48276;probes=cell(size(ports));
for k=1:numel(ports),probes{k}=udpport('datagram','IPV4','LocalHost','127.0.0.1','LocalPort',ports(k));end
for k=1:numel(probes),delete(probes{k});end
cfg=struct('local_mavlink_port',48271,'remote_mavlink_port',48272,'truth_port',48273, ...
    'coptersim_time_port',48274,'target_system',231,'target_component',77, ...
    'clock_max_rtt_s',2,'clock_sync_samples',3,'clock_sync_period_s',.03,'clock_max_age_s',30, ...
    'clock_max_uncertainty_s',1.1,'clock_max_utc_drift_s',.1,'clock_max_time_heartbeat_age_s',2, ...
    'clock_max_time_heartbeat_lag_s',.25,'maximum_truth_lag_s',2,'maximum_raw_records',10000, ...
    'state_max_age_s',.5,'heartbeat_max_age_s',.3,'landed_max_age_s',.3, ...
    'live_enabled',true,'outer_preflight_pass',true,'command_timeout_s',2,'poll_period_s',.01, ...
    'require_terrain_diagnostic_extension',true);
cfg.delivery_environment_contract=struct('expected_session_token',26090501,'initial_payload_kg',2.21, ...
    'payload_by_generation_kg',[2.21;1.75;.98;.55;0], ...
    'mass_by_generation_kg',[11.71;11.25;10.48;10.05;9.5],'remote_port',48276);
parentPlan=jsondecode(fileread(fullfile(gpenmpc_external_path('native_tuned_hover_preparation'),'PLAN.json')));
cfg.temporary_allocator_geometry=parentPlan.temporary_allocator_geometry;
cfg.native_hover_tuning=make_m600_delivery_native_tuning_contract();
checks=struct('name',{},'pass',{});failure='';io=[];peer=[];sub=[];sender=[];envRx=[];pulse=[];
peerClock=[];startUtcMs=int64(0);counter=int64(0);ackBound=false;armedFixture=false;landedFixture=1;
cleanup=struct('adapter',false,'peer',false,'sender',false,'environment_receiver',false,'ports_rebound',false);
guard=onCleanup(@finish); %#ok<NASGU>
try
    dialect=mavlinkdialect('common.xml');
    peer=mavlinkio(dialect,'SystemID',cfg.target_system,'ComponentID',cfg.target_component);
    connect(peer,'UDP','LocalPort',cfg.remote_mavlink_port);
    host=mavlinkclient(peer,255,190);
    sub=mavlinksub(peer,host,'BufferSize',1000,'NewMessageFcn',@onPeer);
    sender=udpport('datagram','IPV4','LocalHost','127.0.0.1','LocalPort',48275);
    envRx=udpport('datagram','IPV4','LocalHost','127.0.0.1','LocalPort',48276);
    io=m600check.makeM600CopterSimIo(cfg);
    startUtcMs=int64(floor(posixtime(datetime('now','TimeZone','UTC'))*1000));peerClock=tic;
    pulse=timer('ExecutionMode','fixedSpacing','Period',.05,'BusyMode','drop','TimerFcn',@(~,~)emit);
    start(pulse);io.sendHeartbeat();
    s=[];t=tic;
    while toc(t)<5
        s=io.snapshot();
        if s.model_ready&&s.delivery_environment.present&& ...
                s.delivery_environment.decoded.environment_extension.initial_not_applied&& ...
                s.delivery_environment.board_ground_disarmed_fresh,break;end
        pause(.02);
    end
    check('actual_adapter_observed_unbound_tag2_and_ground',~isempty(s)&&s.model_ready&& ...
        s.delivery_environment.present&&s.delivery_environment.board_ground_disarmed_fresh);
    value=struct('schema_version',2,'generation',1,'source_io_time_s',s.model_diagnostic.last_source_time_s, ...
        'task_reference_time_s',0,'payload_kg',2.21,'wind_ned_xy_mps',[.2;-.1], ...
        'mission_phase',2,'reference_jet_ned',zeros(12,1),'payload_generation',0, ...
        'task_clock_paused',true,'session_token',26090501, ...
        'board_min_rx_io_time_s',min(s.heartbeat_rx_s,s.extended_rx_s),'board_valid_flags',15, ...
        'landed_state',1,'continuity_epoch',s.delivery_board_continuity_epoch,'service_release_generation',0);
    receipt=io.sendPlantEnvironment(value);
    t=tic;while envRx.NumDatagramsAvailable==0&&toc(t)<2,pause(.01);end
    assert(envRx.NumDatagramsAvailable>0);d=read(envRx,1,'uint8');bytes=packet(d,1);
    head=unpack(bytes(1:8),'int32');frame=unpack(bytes(9:end),'double');
    check('actual_sendPlantEnvironment_2i28d_wire_identity',numel(bytes)==232&&head(1)==1234567897&&head(2)==cfg.target_system&& ...
        numel(frame)==28&&frame(1)==2&&frame(2)==1&&frame(23)==26090501&&receipt.bytes==232);
    ackBound=true;t=tic;s=[];
    while toc(t)<3
        s=io.snapshot();
        if s.delivery_environment.can_continue_task,break;end
        pause(.02);
    end
    check('stateful_delivery_ACK_bound_and_task_visible',~isempty(s)&&s.delivery_environment.can_continue_task&& ...
        s.delivery_environment.mass_ack_valid&& ...
        s.delivery_environment.decoded.environment_extension.applied_frame_generation==1);
    epoch0=s.delivery_board_continuity_epoch;armedFixture=true;
    deadline=tic;
    while toc(deadline)<2
        pause(.02);s=io.snapshot();
        if ~s.delivery_environment.board_ground_disarmed_fresh&& ...
                s.delivery_board_continuity_epoch>epoch0,break;end
    end
    check('armed_transition_breaks_ground_and_increments_epoch',~s.delivery_environment.board_ground_disarmed_fresh&& ...
        s.delivery_board_continuity_epoch>epoch0);
    epoch1=s.delivery_board_continuity_epoch;armedFixture=false;
    deadline=tic;
    while toc(deadline)<2
        pause(.02);s=io.snapshot();
        if s.delivery_environment.board_ground_disarmed_fresh&& ...
                s.delivery_board_continuity_epoch>epoch1,break;end
    end
    check('disarmed_ground_transition_starts_new_epoch',s.delivery_environment.board_ground_disarmed_fresh&& ...
        s.delivery_board_continuity_epoch>epoch1);
    ev=io.evidence();
    check('raw_environment_TX_and_transition_ledger_retained',numel(ev.raw_environment_transmit_datagrams)==1&& ...
        numel(ev.delivery_board_transitions)>=3&&ev.delivery_environment_enabled);
catch err,failure=getReport(err,'extended','hyperlinks','off');end
finish();clear guard
report=struct('schema','HOST_M600_DELIVERY_ADAPTER_LOOPBACK_V1','pass',isempty(failure)&&all([checks.pass]), ...
    'checks_total',numel(checks),'checks_passed',sum([checks.pass]),'checks',checks,'failure',failure, ...
    'adapter_path',which('m600check.makeM600CopterSimIo'),'adapter_sha256',sha(which('m600check.makeM600CopterSimIo')), ...
    'delivery_updater_sha256',sha(which('m600check.updateCopterSimDeliveryDiagnostic')), ...
    'cleanup',cleanup,'COM_open',0,'board_actions',0,'control_actions',0,'claim','Localhost adapter test.');
fid=fopen(fullfile(outputDir,'RESULT.json'),'w','n','UTF-8');assert(fid>=0);c=onCleanup(@()fclose(fid));fprintf(fid,'%s\n',jsonencode(report,PrettyPrint=true));clear c;
disp(jsonencode(struct('pass',report.pass,'checks',sprintf('%d/%d',report.checks_passed,report.checks_total),'failure',failure,'cleanup',cleanup)));
assert(report.pass,'gpenmpc:DeliveryAdapterLoopback','See RESULT.json.');

    function onPeer(~,messages)
        for j=1:numel(messages)
            m=messages(j);
            if m.MsgID==111&&m.Payload.tc1==0
                r=createmsg(dialect,'TIMESYNC');r.Payload.tc1=int64(round((100+toc(peerClock))*1e9));
                r.Payload.ts1=int64(m.Payload.ts1);sendudpmsg(peer,r,'127.0.0.1',cfg.local_mavlink_port);
            end
        end
    end
    function emit()
        elapsed=toc(peerClock);counter=counter+1;
        m=createmsg(dialect,'HEARTBEAT');m.Payload.type=uint8(13);m.Payload.autopilot=uint8(12);
        m.Payload.base_mode=uint8(128*double(armedFixture));m.Payload.custom_mode=uint32(1*65536);
        m.Payload.system_status=uint8(3);m.Payload.mavlink_version=uint8(3);sendPeer(m);
        m=createmsg(dialect,'EXTENDED_SYS_STATE');m.Payload.landed_state=uint8(landedFixture);m.Payload.vtol_state=uint8(0);sendPeer(m);
        m=createmsg(dialect,'LOCAL_POSITION_NED');m.Payload.time_boot_ms=uint32(floor((100+elapsed)*1000));
        m.Payload.x=single(0);m.Payload.y=single(0);m.Payload.z=single(0);m.Payload.vx=single(0);m.Payload.vy=single(0);m.Payload.vz=single(0);sendPeer(m);
        m=createmsg(dialect,'ATTITUDE');m.Payload.time_boot_ms=uint32(floor((100+elapsed)*1000));
        for n={'roll','pitch','yaw','rollspeed','pitchspeed','yawspeed'},m.Payload.(n{1})=single(0);end,sendPeer(m);
        m=createmsg(dialect,'ESTIMATOR_STATUS');m.Payload.time_usec=uint64(floor((100+elapsed)*1e6));m.Payload.flags=uint16(959);
        for n={'vel_ratio','pos_horiz_ratio','pos_vert_ratio','mag_ratio','pos_horiz_accuracy','pos_vert_accuracy'},m.Payload.(n{1})=single(.1);end
        m.Payload.hagl_ratio=single(NaN);m.Payload.tas_ratio=single(NaN);sendPeer(m);
        m=createmsg(dialect,'AUTOPILOT_VERSION');m.Payload.uid=uint64(42);m.Payload.flight_sw_version=uint32(hex2dec('011000FF'));
        m.Payload.vendor_id=uint16(65500);m.Payload.product_id=uint16(65501);m.Payload.flight_custom_version=uint8(1:8);sendPeer(m);
        d=zeros(1,32);d(1:7)=[0,0,elapsed,0,1,0,1];d(26)=2;
        if ackBound,d(27:32)=[26090501,1,0,2.21,11.71,0];else,d(27:32)=[0,0,0,2.21,11.71,16];end
        diagnostic=[packLE(int32([1234567890,cfg.target_system])),packLE(d)];
        floats=zeros(1,24,'single');floats(7)=1;
        truth=[packLE(int32([123456789,cfg.target_system,5,0])),packLE(floats),packLE(double([elapsed,0,0,0,0,0,0]))];
        nowMs=int64(floor(posixtime(datetime('now','TimeZone','UTC'))*1000));
        clock=[packLE(int32([123456789,cfg.target_system])),packLE(int64([startUtcMs,nowMs,counter]))];
        write(sender,truth,'uint8','127.0.0.1',cfg.truth_port);write(sender,diagnostic,'uint8','127.0.0.1',cfg.truth_port);
        write(sender,clock,'uint8','127.0.0.1',cfg.coptersim_time_port);
    end
    function sendPeer(m),sendudpmsg(peer,m,'127.0.0.1',cfg.local_mavlink_port);end
    function check(name,ok),checks(end+1)=struct('name',name,'pass',islogical(ok)&&isscalar(ok)&&ok);end
    function finish()
        if ~isempty(pulse),try,stop(pulse);delete(pulse);pulse=[];catch,end,end
        if ~isempty(io),try,cleanup.adapter=io.close();catch,end,end
        if ~isempty(sub),try,sub.NewMessageFcn=[];delete(sub);sub=[];catch,end,end
        if ~isempty(peer),try,disconnect(peer);delete(peer);peer=[];cleanup.peer=true;catch,end,end
        if ~isempty(sender),try,delete(sender);sender=[];cleanup.sender=true;catch,end,end
        if ~isempty(envRx),try,delete(envRx);envRx=[];cleanup.environment_receiver=true;catch,end,end
        try
            for z=ports,p=udpport('datagram','IPV4','LocalHost','127.0.0.1','LocalPort',z);delete(p);end
            cleanup.ports_rebound=true;
        catch,end
    end
end
function b=packet(v,i),if istable(v),x=v.Data;if iscell(x),b=x{i};else,b=x(i,:);end,else,b=v(i).Data;end,b=reshape(uint8(b),1,[]);end
function v=unpack(b,t),v=typecast(reshape(uint8(b),1,[]),t);[~,~,e]=computer;if e=='B',v=swapbytes(v);end,end
function b=packLE(v),[~,~,e]=computer;if e=='B',v=swapbytes(v);end,b=reshape(typecast(v,'uint8'),1,[]);end
function h=sha(p),fid=fopen(p,'rb');c=onCleanup(@()fclose(fid));x=fread(fid,Inf,'*uint8');m=java.security.MessageDigest.getInstance('SHA-256');m.update(x);h=upper(reshape(dec2hex(typecast(m.digest(),'uint8'),2).',1,[]));clear c,end
