function out=analyze_native_hover_post_disarm(outputDir)
% Analyze retained post-disarm data.
arguments,outputDir (1,1) string,end
assert(~isfolder(outputDir)&&~isfile(outputDir),'analysis:OutputExists','Use a new HOST output.');
build=fileparts(fileparts(mfilename('fullpath')));oldPath=path;
addpath(fullfile(build,'m600_coptersim','matlab_validation'));
pathCleanup=onCleanup(@()path(oldPath)); %#ok<NASGU>
runRoot=gpenmpc_external_path('native_hover_post_disarm_recording');
input=fullfile(runRoot,'INNER_MATLAB','RAW_MATLAB_HIL.mat');
inventory=whos('-file',input);disp(struct2table(inventory));
assert(any(strcmp({inventory.name},'result')),'analysis:MissingResult','Expected retained result variable.');
% MAT contains one result structure: select it rather than loading workspace.
loaded=load(input,'result');r=loaded.result;clear loaded;e=r.transport_evidence;
offset=mean([e.fixed_best_timesync.offset_lower_s,e.fixed_best_timesync.offset_upper_s]);
truthOffset=e.coptersim_start_utc_s-e.utc_zero_s;
formalBegin=eventTime(r,'FORMAL_BEGIN');formalEnd=eventTime(r,'FORMAL_END');
disarmRequest=eventTime(r,'STANDARD_DISARM_REQUEST');disarmAck=eventTime(r,'STANDARD_DISARM_ACK');
att=zeros(0,9);lp=zeros(0,9);act=zeros(0,21);hb=zeros(0,4);land=zeros(0,3);imu=zeros(0,13);
texts=struct('raw_index',{},'rx_s',{},'severity',{},'text',{});
for k=1:numel(e.raw_mavlink)
    a=e.raw_mavlink{k};p=a.message.Payload;
    switch a.topic
        case 'ATTITUDE'
            att(end+1,:)=[k,a.rx_s,double(p.time_boot_ms)/1000+offset,double([p.roll,p.pitch,p.yaw,p.rollspeed,p.pitchspeed,p.yawspeed])]; %#ok<AGROW>
        case 'LOCAL_POSITION_NED'
            lp(end+1,:)=[k,a.rx_s,double(p.time_boot_ms)/1000+offset,double([p.x,p.y,p.z,p.vx,p.vy,p.vz])]; %#ok<AGROW>
        case 'HIL_ACTUATOR_CONTROLS'
            rawS=double(p.time_usec)/1e6;
            act(end+1,:)=[k,a.rx_s,rawS+offset,rawS,double(p.mode),reshape(double(p.controls),1,[])]; %#ok<AGROW>
        case 'HEARTBEAT'
            hb(end+1,:)=[k,a.rx_s,double(bitand(uint8(p.base_mode),128)~=0),double(p.base_mode)]; %#ok<AGROW>
        case 'EXTENDED_SYS_STATE',land(end+1,:)=[k,a.rx_s,double(p.landed_state)]; %#ok<AGROW>
        case 'HIGHRES_IMU'
            imu(end+1,:)=[k,a.rx_s,double(p.time_usec)/1e6+offset, ...
                double([p.xacc,p.yacc,p.zacc,p.xgyro,p.ygyro,p.zgyro,p.xmag,p.ymag,p.zmag,p.abs_pressure])]; %#ok<AGROW>
        case 'STATUSTEXT'
            texts(end+1)=struct('raw_index',k,'rx_s',a.rx_s,'severity',double(p.severity), ...
                'text',strtrim(strrep(char(p.text),char(0),''))); %#ok<AGROW>
    end
end
truth=zeros(0,29);diagRows=zeros(0,12);badTruth=0;
for k=1:numel(e.raw_truth_datagrams)
    a=e.raw_truth_datagrams{k};bytes=reshape(uint8(a.bytes),1,[]);
    if numel(bytes)==264
        h=typecast(bytes(1:8),'int32');d=typecast(bytes(9:end),'double');
        assert(h(1)==1234567890&&h(2)==1,'analysis:DiagnosticHeader','Wrong retained diagnostic header.');
        diagRows(end+1,:)=[k,a.rx_s,d(3)+truthOffset,d(3),d(1),d(2),d(4),d(5),d(6),d(8),d(9),d(26)]; %#ok<AGROW>
    elseif ismember(numel(bytes),[112,168,200])
        d=m600check.decodeTruthPacket(bytes,struct('expected_copter_id',1,'expected_vehicle_type',5));
        if ~d.valid,badTruth=badTruth+1;continue;end
        truth(end+1,:)=[k,a.rx_s,d.time_s+truthOffset,d.time_s,d.position_ned_m,d.velocity_ned_mps, ...
            d.euler_rad,d.angular_rate_body_radps,d.acceleration_body_mps2,d.motor_rpm,d.quaternion_norm,double(d.valid)]; %#ok<AGROW>
    end
end
assert(~isempty(att)&&~isempty(truth)&&~isempty(hb)&&~isempty(diagRows),'analysis:MissingStreams','Required retained streams missing.');
firstDisarmed=find(hb(:,2)>=disarmRequest&hb(:,3)==0,1);
assert(~isempty(firstDisarmed),'analysis:MissingDisarm','No fresh disarmed heartbeat after request.');
disarmObserved=hb(firstDisarmed,2);endS=min(max(truth(:,3)),max(att(:,3)));
phaseWindows=[formalBegin formalEnd;formalEnd disarmObserved;disarmObserved disarmObserved+8; ...
    disarmObserved+8 min(disarmObserved+20,endS);min(disarmObserved+20,endS) endS];
phaseNames={'FORMAL_UNCHANGED','NATIVE_LAND_TO_OBSERVED_DISARM','POST_DISARM_FIRST_8S','POST_DISARM_8_TO_20S','POST_DISARM_REMAINDER'};
phases=struct([]);
for k=1:size(phaseWindows,1)
    q=phaseSummary(phaseNames{k},phaseWindows(k,:),truth,att,lp,act,hb,land,imu,diagRows,r.config);
    if isempty(phases),phases=q;else,phases(end+1)=q;end %#ok<AGROW>
end
% Compare nearest source samples at fixed diagnostic times.
sampleOffsets=[-1,0,1,3,5,6.652,8,10,15,20,40,60];samples=struct([]);
for k=1:numel(sampleOffsets)
    t=disarmObserved+sampleOffsets(k);if t> endS,continue;end
    q=sampleAt(t,truth,att,lp,act,imu,diagRows,r.config);
    q.relative_to_first_disarmed_heartbeat_s=sampleOffsets(k);
    if isempty(samples),samples=q;else,samples(end+1)=q;end %#ok<AGROW>
end
vendorPath=fullfile(runRoot,'COPTERSIM_STDERR.log');vendor=regexp(fileread(vendorPath),'\r?\n','split');
vendorEvents=struct('vendor_s',{},'line',{});
for k=1:numel(vendor)
    if contains(vendor{k},'Disarmed by external command')||contains(vendor{k},'Preflight Fail: Attitude')||contains(vendor{k},'Landing detected')
        token=regexp(vendor{k},'^\[\s*([0-9.]+)\s','tokens','once');
        if ~isempty(token),vendorEvents(end+1)=struct('vendor_s',str2double(token{1}),'line',vendor{k});end %#ok<AGROW>
    end
end
afterTexts=texts([texts.rx_s]>=formalEnd);
statusDisarm=find(contains({texts.text},'Disarmed by external command')&[texts.rx_s]>=disarmRequest,1);
vendorDisarm=find(contains({vendorEvents.line},'Disarmed by external command'),1,'last');
vendorMapping=struct('paired_event','Disarmed by external command','host_rx_s',NaN,'vendor_s',NaN, ...
    'vendor_to_host_receive_offset_s',NaN,'claim','Paired receive-time observation.');
if ~isempty(statusDisarm)&&~isempty(vendorDisarm)
    vendorMapping.host_rx_s=texts(statusDisarm).rx_s;vendorMapping.vendor_s=vendorEvents(vendorDisarm).vendor_s;
    vendorMapping.vendor_to_host_receive_offset_s=vendorMapping.host_rx_s-vendorMapping.vendor_s;
end
crossings=struct();
for deg=[5 10 20 45 60]
    crossings.(sprintf('abs_roll_or_pitch_%d_deg',deg))=struct( ...
        'plant',firstRow(truth,truth(:,3)>=disarmObserved&max(abs(truth(:,11:12)),[],2)>=deg*pi/180), ...
        'PX4',firstRow(att,att(:,3)>=disarmObserved&max(abs(att(:,4:5)),[],2)>=deg*pi/180));
end
sourceFiles={fullfile(build,'matlab_validation','+m600check','contactKernel.m'), ...
    fullfile(build,'matlab_validation','+m600check','derivativeSoftware.m'), ...
    fullfile(build,'matlab_validation','+m600check','stepPx4Rk4.m'), ...
    fullfile(build,'matlab_validation','+m600check','generatedPlantDerivative.m')};
sourceBindings=struct([]);for k=1:numel(sourceFiles),q=identity(sourceFiles{k});if isempty(sourceBindings),sourceBindings=q;else,sourceBindings(end+1)=q;end;end
out=struct('schema','HOST_POST_DISARM_STABILITY_ANALYSIS_V1','input',identity(input), ...
    'mat_inventory',inventory,'selected_top_level_variable','result','load_limitation','One retained result struct contains needed raw streams; no separate top-level stream variables exist.', ...
    'source_time_mapping',struct('PX4_to_host_s',offset,'model_to_host_s',truthOffset, ...
        'best_timesync',e.fixed_best_timesync,'utc_pair_uncertainty_s',e.utc_pair_uncertainty_s), ...
    'formal_status_preserved',r.status,'formal_window_unchanged',[formalBegin formalEnd], ...
    'disarm',struct('request_s',disarmRequest,'ACK_s',disarmAck,'first_disarmed_HB_rx_s',disarmObserved), ...
    'row_counts',struct('truth',size(truth,1),'invalid_truth',badTruth,'ATTITUDE',size(att,1), ...
        'LP',size(lp,1),'actuator',size(act,1),'IMU',size(imu,1),'diagnostic',size(diagRows,1)), ...
    'phases',phases,'diagnostic_samples',samples,'crossings_not_new_performance_gates',crossings, ...
    'post_formal_status_text',afterTexts,'vendor_events',vendorEvents,'vendor_receive_pair',vendorMapping, ...
    'first_model_fault',firstRow(diagRows,diagRows(:,5)~=0), ...
    'first_terrain_capture',firstRow(diagRows,diagRows(:,10)~=0), ...
    'model_contact_source_bindings',sourceBindings, ...
    'source_findings',{{'derivativeSoftware applies vertical contact support to dx(6); contact torque is zero.', ...
        'stepPx4Rk4 ground_confirmed uses contact force, vertical position/velocity and 0.5 s support dwell.', ...
        'A stationary upright 8 s delivery dwell also requires retained attitude and fresh independent PX4 status.', ...
        'Actual rotor thrust states and per-leg contact moments are absent from published truth. The six animation channels are scaled model inputs, not measured rotor force or physical RPM.'}}, ...
    'limitations',{{'Nearest source pairs are diagnostic with reported time differences; no truth is fed to controller/EKF.', ...
        'The examined 8 s window represents the proposed delivery dwell.'}}, ...
    'hardware_actions',0,'COM_UDP_actions',0,'model_or_controller_runs',0,'source_changes',0,'new_flight_admission',false);
mkdir(outputDir);f=fopen(fullfile(outputDir,'SUMMARY.json'),'w','n','UTF-8');assert(f>=0,'analysis:Write','Cannot write new HOST summary.');
c=onCleanup(@()fclose(f));fprintf(f,'%s\n',jsonencode(out,PrettyPrint=true));clear c
disp(jsonencode(struct('disarm',out.disarm,'phases',phases,'vendor_mapping',vendorMapping,'first_model_fault',out.first_model_fault),PrettyPrint=true));
clear pathCleanup
end
function q=phaseSummary(name,w,tr,at,lp,ac,hb,la,im,di,cfg)
ix=tr(:,3)>=w(1)&tr(:,3)<w(2);t=tr(ix,:);a=at(at(:,3)>=w(1)&at(:,3)<w(2),:);
d=di(di(:,3)>=w(1)&di(:,3)<w(2),:);v=ac(ac(:,3)>=w(1)&ac(:,3)<w(2),:);
ages=v(:,2)-v(:,3);fresh=ages>=-cfg.clock_max_uncertainty_s&ages<=cfg.state_max_age_s;
h=hb(hb(:,2)>=w(1)&hb(:,2)<w(2),:);l=la(la(:,2)>=w(1)&la(:,2)<w(2),:);
q=struct('name',name,'window_host_s',w,'truth_rows',size(t,1),'truth_first_last_source_s',edge(t,3), ...
    'truth_max_source_gap_s',maxOrNaN(diff(unique(t(:,3)))),'truth_max_abs_euler_deg',maxCols(abs(t(:,11:13)))*180/pi, ...
    'truth_first_last_euler_deg',[edgeValues(t,11:13)]*180/pi, ...
    'truth_max_body_rate_deg_s',maxCols(abs(t(:,14:16)))*180/pi,'truth_max_speed_mps',maxOrNaN(vecnorm(t(:,8:10),2,2)), ...
    'truth_max_horizontal_speed_mps',maxOrNaN(vecnorm(t(:,8:9),2,2)), ...
    'truth_position_min',minCols(t(:,5:7)),'truth_position_max',maxCols(t(:,5:7)), ...
    'model_input6_min',minCols(t(:,20:25)/1000),'model_input6_max',maxCols(t(:,20:25)/1000), ...
    'PX4_attitude_rows',size(a,1),'PX4_max_abs_euler_deg',maxCols(abs(a(:,4:6)))*180/pi, ...
    'PX4_max_body_rate_deg_s',maxCols(abs(a(:,7:9)))*180/pi, ...
    'actuator_rows',size(v,1),'actuator_source_fresh_rows',nnz(fresh), ...
    'actuator_fresh6_min',minCols(v(fresh,6:11)),'actuator_fresh6_max',maxCols(v(fresh,6:11)), ...
    'actuator_source_age_range_s',[minOrNaN(ages),maxOrNaN(ages)], ...
    'HB_rows',size(h,1),'HB_any_armed',any(h(:,3)~=0),'HB_max_receive_gap_s',maxOrNaN(diff(h(:,2))), ...
    'landed_state_counts',struct('unknown',nnz(l(:,3)==0),'on_ground',nnz(l(:,3)==1), ...
        'in_air',nnz(l(:,3)==2),'takeoff',nnz(l(:,3)==3),'landing',nnz(l(:,3)==4)), ...
    'diagnostic_rows',size(d,1),'model_ground_true_rows',nnz(d(:,8)==1),'model_failed_rows',nnz(d(:,5)~=0), ...
    'terrain_capture_rows',nnz(d(:,10)~=0),'contact_force_range_N',[minOrNaN(d(:,7)),maxOrNaN(d(:,7))], ...
    'LP_rows',nnz(lp(:,3)>=w(1)&lp(:,3)<w(2)),'IMU_rows',nnz(im(:,3)>=w(1)&im(:,3)<w(2)), ...
    'truth_essential_nonfinite',nnz(~isfinite(t(:,5:19))));
% Compute time-aligned orientation difference.
gap=zeros(size(t,1),3);delta=zeros(size(t,1),1);
for k=1:size(t,1),[delta(k),j]=min(abs(at(:,3)-t(k,3)));gap(k,:)=atan2(sin(at(j,4:6)-t(k,11:13)),cos(at(j,4:6)-t(k,11:13)));end
q.attitude_pair_max_delta_s=maxOrNaN(delta);q.attitude_pair_max_gap_deg=maxCols(abs(gap))*180/pi;
end
function q=sampleAt(t,tr,at,lp,ac,im,di,cfg)
[td,i]=min(abs(tr(:,3)-t));a=tr(i,:);[ad,j]=min(abs(at(:,3)-a(3)));[ld,k]=min(abs(lp(:,3)-a(3)));
[dd,z]=min(abs(di(:,3)-a(3)));[md,m]=min(abs(im(:,3)-a(3)));u=find(ac(:,3)<=a(3),1,'last');
q=struct('requested_host_s',t,'truth_raw_index',a(1),'truth_source_s',a(3),'truth_rx_s',a(2), ...
    'truth_requested_time_delta_s',td,'truth_p_ned_m',a(5:7),'truth_v_ned_mps',a(8:10), ...
    'truth_euler_deg',a(11:13)*180/pi,'truth_body_rates_deg_s',a(14:16)*180/pi, ...
    'truth_acceleration_body',a(17:19),'model_inputs6',a(20:25)/1000, ...
    'PX4_attitude_source_s',at(j,3),'attitude_pair_delta_s',ad,'PX4_euler_deg',at(j,4:6)*180/pi, ...
    'PX4_rates_deg_s',at(j,7:9)*180/pi,'LP_pair_delta_s',ld,'PX4_position_ned',lp(k,4:6),'PX4_velocity_ned',lp(k,7:9), ...
    'diagnostic_pair_delta_s',dd,'ground_confirmed',di(z,8),'contact_force_N',di(z,7), ...
    'IMU_pair_delta_s',md,'IMU_acceleration',im(m,4:6),'IMU_gyro',im(m,7:9), ...
    'actuator_source_age_s',NaN,'actuator_receive_age_s',NaN,'actuator_fresh',false,'actuator_controls6',nan(1,6));
if ~isempty(u)
    q.actuator_source_age_s=a(3)-ac(u,3);q.actuator_receive_age_s=a(3)-ac(u,2);
    q.actuator_fresh=q.actuator_source_age_s<=cfg.state_max_age_s&&ac(u,2)-ac(u,3)>=-cfg.clock_max_uncertainty_s&&ac(u,2)-ac(u,3)<=cfg.state_max_age_s;
    if q.actuator_fresh,q.actuator_controls6=ac(u,6:11);end
end
end
function t=eventTime(r,kind),k=find(strcmp({r.events.kind},kind),1);assert(~isempty(k),'analysis:EventMissing','Missing %s',kind);t=r.events(k).time_s;end
function q=firstRow(x,mask),i=find(mask,1);q=[];if ~isempty(i),q=x(i,:);end;end
function x=edge(a,c),x=[NaN NaN];if ~isempty(a),x=[a(1,c),a(end,c)];end;end
function x=edgeValues(a,c),x=nan(2,numel(c));if ~isempty(a),x=a([1 end],c);end;end
function v=maxCols(x),v=nan(1,size(x,2));if ~isempty(x),v=max(x,[],1);end;end
function v=minCols(x),v=nan(1,size(x,2));if ~isempty(x),v=min(x,[],1);end;end
function v=maxOrNaN(x),v=NaN;if ~isempty(x),v=max(x);end;end
function v=minOrNaN(x),v=NaN;if ~isempty(x),v=min(x);end;end
function id=identity(p)
f=fopen(p,'rb');assert(f>=0,'analysis:MissingInput','Cannot read %s',p);c=onCleanup(@()fclose(f));
md=java.security.MessageDigest.getInstance('SHA-256');count=0;
while ~feof(f),x=fread(f,1024*1024,'*uint8');count=count+numel(x);md.update(x);end
id=struct('path',p,'bytes',count,'sha256',upper(reshape(dec2hex(typecast(md.digest(),'uint8'),2).',1,[])));clear c
end
