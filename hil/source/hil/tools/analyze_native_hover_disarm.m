function out=analyze_native_hover_disarm(outputDir)
% Analyze retained disarm records.
arguments,outputDir (1,1) string,end
assert(~isfolder(outputDir));mkdir(outputDir);
b=fileparts(fileparts(mfilename('fullpath')));addpath(fullfile(b,'m600_coptersim','matlab_validation'));
runRoot=gpenmpc_external_path('native_hover_disarm_recording');
input=fullfile(runRoot,'INNER_MATLAB','RAW_MATLAB_HIL.mat');x=load(input,'result');r=x.result;e=r.transport_evidence;
offset=mean([e.fixed_best_timesync.offset_lower_s,e.fixed_best_timesync.offset_upper_s]);
armReq=r.events(find(strcmp({r.events.kind},'LOGICAL_ARM_REQUEST'),1)).time_s;
formal=r.events(find(strcmp({r.events.kind},'FORMAL_BEGIN'),1)).time_s;
cut=r.first_failed_state_evidence.captured_at_s;
att=zeros(0,9);lp=zeros(0,9);act=zeros(0,21);hb=zeros(0,7);ext=zeros(0,4);
txt=struct('raw_index',{},'rx_s',{},'severity',{},'text',{});ack=zeros(0,5);imu=zeros(0,13);
for k=1:numel(e.raw_mavlink)
    a=e.raw_mavlink{k};p=a.message.Payload;
    switch a.topic
        case 'ATTITUDE'
            t=double(p.time_boot_ms)/1000;
            att(end+1,:)=[k,a.rx_s,t+offset,double([p.roll,p.pitch,p.yaw,p.rollspeed,p.pitchspeed,p.yawspeed])]; %#ok<AGROW>
        case 'LOCAL_POSITION_NED'
            t=double(p.time_boot_ms)/1000;lp(end+1,:)=[k,a.rx_s,t+offset,double([p.x,p.y,p.z,p.vx,p.vy,p.vz])]; %#ok<AGROW>
        case 'HIL_ACTUATOR_CONTROLS'
            t=double(p.time_usec)/1e6;v=reshape(double(p.controls),1,[]);
            act(end+1,:)=[k,a.rx_s,t+offset,t,double(p.mode),v]; %#ok<AGROW>
        case 'HEARTBEAT'
            c=uint32(p.custom_mode);hb(end+1,:)=[k,a.rx_s,double(bitand(uint8(p.base_mode),128)~=0), ...
                double(bitand(bitshift(c,-16),255)),double(bitand(bitshift(c,-24),255)),double(p.base_mode),double(p.system_status)]; %#ok<AGROW>
        case 'EXTENDED_SYS_STATE',ext(end+1,:)=[k,a.rx_s,double(p.landed_state),double(p.vtol_state)]; %#ok<AGROW>
        case 'STATUSTEXT'
            txt(end+1)=struct('raw_index',k,'rx_s',a.rx_s,'severity',double(p.severity), ...
                'text',strtrim(strrep(char(p.text),char(0),''))); %#ok<AGROW>
        case 'COMMAND_ACK',ack(end+1,:)=[k,a.rx_s,double(p.command),double(p.result),double(p.result_param2)]; %#ok<AGROW>
        case 'HIGHRES_IMU'
            imu(end+1,:)=[k,a.rx_s,double(p.time_usec)/1e6+offset, ...
                double([p.xacc,p.yacc,p.zacc,p.xgyro,p.ygyro,p.zgyro,p.xmag,p.ymag,p.zmag,p.abs_pressure])]; %#ok<AGROW>
    end
end
truth=zeros(0,29);diag=zeros(0,8);
for k=1:numel(e.raw_truth_datagrams)
    a=e.raw_truth_datagrams{k};bytes=uint8(a.bytes);
    if numel(bytes)==264
        d=typecast(bytes(9:end),'double');diag(end+1,:)=[k,a.rx_s,d(3)+e.coptersim_start_utc_s-e.utc_zero_s,d(3),d(1),d(4),d(5),d(6)]; %#ok<AGROW>
    elseif ismember(numel(bytes),[112,168,200])
        d=m600check.decodeTruthPacket(bytes,struct('expected_copter_id',1,'expected_vehicle_type',5));
        if ~d.valid,error('analysis:TruthDecode','Retained truth invalid: %s',d.reason);end
        truth(end+1,:)=[k,a.rx_s,d.time_s+e.coptersim_start_utc_s-e.utc_zero_s,d.time_s, ...
            d.position_ned_m,d.velocity_ned_mps,d.euler_rad,d.angular_rate_body_radps, ...
            d.acceleration_body_mps2,d.motor_rpm,d.quaternion_norm,double(d.valid)]; %#ok<AGROW>
    end
end
commandRows=struct('tx_index',{},'sent_s',{},'msg_id',{},'command',{},'parameters',{});
for k=1:numel(e.raw_transmit_messages)
    a=e.raw_transmit_messages{k};m=a.message;
    if double(m.MsgID)==76
        p=m.Payload;commandRows(end+1)=struct('tx_index',k,'sent_s',a.sent_s,'msg_id',double(m.MsgID), ...
            'command',double(p.command),'parameters',double([p.param1,p.param2,p.param3,p.param4,p.param5,p.param6,p.param7])); %#ok<AGROW>
    end
end
% Retain source time, receive time and raw indices.
write(att,{'raw_index','rx_s','mapped_source_s','roll_rad','pitch_rad','yaw_rad','roll_rate','pitch_rate','yaw_rate'},'ATTITUDE.csv');
write(lp,{'raw_index','rx_s','mapped_source_s','x','y','z','vx','vy','vz'},'LOCAL_POSITION.csv');
write(act,[{'raw_index','rx_s','mapped_source_s','raw_source_s','mode'},compose('control%d',1:16)],'ACTUATOR.csv');
write(hb,{'raw_index','rx_s','armed','main_mode','sub_mode','base_mode','system_status'},'HEARTBEAT.csv');
write(ext,{'raw_index','rx_s','landed_state','vtol_state'},'LANDED_STATE.csv');
write(ack,{'raw_index','rx_s','command','result','result_param2'},'COMMAND_ACK.csv');
write(imu,{'raw_index','rx_s','mapped_source_s','ax','ay','az','gx','gy','gz','mx','my','mz','abs_pressure'},'HIGHRES_IMU.csv');
write(truth,[{'raw_index','rx_s','mapped_source_s','sim_s','x','y','z','vx','vy','vz', ...
    'roll_rad','pitch_rad','yaw_rad','wx','wy','wz','ax','ay','az'},compose('rpm%d',1:8),{'quat_norm','valid'}],'TRUTH.csv');
write(diag,{'raw_index','rx_s','mapped_source_s','sim_s','model_failed','contact_force_N','ground_confirmed','airborne_observed'},'MODEL_DIAGNOSTIC.csv');
writetable(struct2table(txt),fullfile(outputDir,'STATUSTEXT.csv'));
armedChanges=hb([true;diff(hb(:,3))~=0],:);landChanges=ext([true;diff(ext(:,3))~=0],:);
angleCross=struct();
for deg=[5 10 20 30 45 60]
    key=sprintf('abs_roll_%d_deg',deg);
    angleCross.(key)=struct('PX4',first(att,att(:,3)>=armReq&abs(att(:,4))>=deg*pi/180), ...
        'plant',first(truth,truth(:,3)>=armReq&abs(truth(:,11))>=deg*pi/180));
end
ia=att(:,3)>=armReq&att(:,3)<=cut;it=truth(:,3)>=armReq&truth(:,3)<=cut;
ic=act(:,2)>=armReq&act(:,2)<=cut;id=diag(:,3)>=armReq&diag(:,3)<=cut;
matched=zeros(nnz(ia),10);ar=att(ia,:);
for k=1:size(ar,1)
    [delta,j]=min(abs(truth(:,3)-ar(k,3)));
    err=atan2(sin(ar(k,4:6)-truth(j,11:13)),cos(ar(k,4:6)-truth(j,11:13)));
    matched(k,:)=[ar(k,1),truth(j,1),ar(k,3),truth(j,3),delta,err,ar(k,4),truth(j,11)];
end
write(matched,{'att_raw_index','truth_raw_index','att_source_s','truth_source_s','pair_dt_s', ...
    'roll_gap_rad','pitch_gap_rad','yaw_gap_rad','PX4_roll_rad','plant_roll_rad'},'ATTITUDE_TRUTH_PAIRS.csv');
freshAct=act(ic,:);age=freshAct(:,2)-freshAct(:,3);actWindow=struct('row_count',size(freshAct,1), ...
    'unique_source_count',numel(unique(freshAct(:,4))),'source_reversals',nnz(diff(freshAct(:,4))<0), ...
    'source_age_min_s',min(age),'source_age_max_s',max(age), ...
    'control6_min',min(freshAct(:,6:11),[],1),'control6_max',max(freshAct(:,6:11),[],1), ...
    'first_nonzero_source_fresh_row',first(freshAct,any(abs(freshAct(:,6:11))>0,2)&age>=-r.config.clock_max_uncertainty_s&age<=r.config.state_max_age_s));
out=struct('classification','HOST_ONLY_FORMAL_UNEXPECTED_DISARM_CAUSAL_OBSERVATIONS', ...
    'input',input,'input_sha256',sha(input),'source_mavlink_offset_s',offset, ...
    'events',r.events,'command_tx',commandRows,'status_text',txt, ...
    'armed_state_changes',armedChanges,'landed_state_changes',landChanges, ...
    'arm_request_s',armReq,'formal_begin_s',formal,'failure_capture_s',cut, ...
    'angle_crossings_descriptive_not_new_gates',angleCross, ...
    'plant_window',struct('row_count',nnz(it),'min_z',min(truth(it,7)),'max_z',max(truth(it,7)), ...
        'max_abs_euler_deg',max(abs(truth(it,11:13)),[],1)*180/pi, ...
        'rpm6_min',min(truth(it,20:25),[],1),'rpm6_max',max(truth(it,20:25),[],1), ...
        'any_ground_false',any(diag(id,7)==0),'any_airborne_latched',any(diag(id,8)==1)), ...
    'PX4_window_max_abs_euler_deg',max(abs(att(ia,4:6)),[],1)*180/pi, ...
    'matched_attitude',struct('pair_count',size(matched,1),'max_pair_dt_s',max(matched(:,5)), ...
        'max_abs_euler_gap_deg',max(abs(matched(:,6:8)),[],1)*180/pi), ...
    'actuator_window',actWindow,'first_failed_state',r.first_failed_state_evidence, ...
    'host_command_counts',r.counts,'first_transport_fatal',e.first_fatal, ...
    'hardware_actions',0,'COM_UDP_or_model_launch',0, ...
    'limits','Clock mappings are approximate. Nearest-source pairs are diagnostic; control direction and ordering require the model and allocator source mapping. Stale actuator frames are excluded from output inference.');
fid=fopen(fullfile(outputDir,'DISARM_CAUSAL_SUMMARY.json'),'w');assert(fid>=0);c=onCleanup(@()fclose(fid)); %#ok<NASGU>
fwrite(fid,jsonencode(out,PrettyPrint=true),'char');
disp(jsonencode(struct('changes',armedChanges,'angle_crossings',angleCross,'plant',out.plant_window, ...
    'matched',out.matched_attitude,'actuator',actWindow,'texts',txt),PrettyPrint=true));
    function write(data,names,file),writetable(array2table(data,'VariableNames',names),fullfile(outputDir,file));end
end
function r=first(rows,mask),i=find(mask,1);r=[];if ~isempty(i),r=rows(i,:);end;end
function h=sha(path)
fid=fopen(path,'rb');assert(fid>=0);c=onCleanup(@()fclose(fid)); %#ok<NASGU>
md=java.security.MessageDigest.getInstance('SHA-256');md.update(fread(fid,Inf,'*uint8'));
h=upper(reshape(dec2hex(typecast(md.digest(),'uint8'),2).',1,[]));
end
