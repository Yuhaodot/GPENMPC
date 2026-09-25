function summary=summarize_usb_rc_short(runRoot)
% Summarize saved component data.
build=fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(build,'host_runtime'),fullfile(build,'m600_coptersim','matlab_validation'));
a=load(fullfile(runRoot,'SHORT_HIL','RAW_BOARD_LOCAL_SHORT_HIL.mat'),'methodRaw','rawIo','result');
consoleParts={};
for k=1:numel(a.rawIo.raw_mavlink)
 r=a.rawIo.raw_mavlink{k};
 if strcmp(r.topic,'SERIAL_CONTROL')
  payload=r.message.Payload;
  if payload.count>0,consoleParts{end+1}=char(payload.data(1:double(payload.count)));end %#ok<AGROW>
 end
end
consoleLines=splitlines(string([consoleParts{:}]));
failureLines=consoleLines(contains(consoleLines,["LOCAL_IO ","LOCAL_CTX ","LOCAL_SCHED ","LOCAL_AUTH ","LOCAL_GP ","RFLY_LOCAL_NESTED"]));
if ~isempty(failureLines),disp(join(failureLines,newline));end
pub=[];counter=[];controls=[];reference=[];
for k=1:numel(a.methodRaw)
 e=a.methodRaw{k};if ~isfield(e,'committed'),continue;end
 for j=1:numel(e.committed)
  r=e.committed{j}.raw;c=gpenmpcNative.RflyLocalCommittedDecoder(r.message,r.original_host_receive_ns);
  pub(end+1,1)=double(c.original_publication_us)*1e-6; %#ok<AGROW>
  counter(end+1,1)=double(c.joint_installs); %#ok<AGROW>
  controls(end+1,:)=double(c.published_control16(1:6)); %#ok<AGROW>
  reference(end+1,:)=c.reference6(:).'; %#ok<AGROW>
 end
end
[counter,keep]=unique(counter,'stable');pub=pub(keep);controls=controls(keep,:);reference=reference(keep,:);
rows=a.rawIo.raw_transmit_messages;
use=cellfun(@(r)isfield(r,'canonical_local_task_inputs')&&r.canonical_local_task_inputs&&r.send_returned,rows);
rows=rows(use);assert(mod(numel(rows),6)==0);
target=zeros(numel(rows)/6,4);source=zeros(size(target,1),1);sim=source;thrust=zeros(size(target,1),6);
for k=1:size(target,1)
 bytes=zeros(647,1,'uint8');
 for j=1:6
  p=rows{6*(k-1)+j}.message.Payload;assert(p.payload(1)==uint8(207+j));
  n=min(119,647-(j-1)*119);bytes((j-1)*119+(1:n))=p.payload(10:9+n);
 end
 u=gpenmpcNative.RflyLocalTaskCodec.decode(bytes);target(k,:)=u.outer.target4(:).';
 source(k)=double(u.source.sample_us)*1e-6;sim(k)=u.rotor.original_observation.original_sim_time_s;
 thrust(k,:)=u.rotor.original_observation.observed_thrust_n(:).';
end
t=[];p=[];angles=[];rpm=[];
for k=1:numel(a.rawIo.raw_truth_datagrams)
 r=a.rawIo.raw_truth_datagrams{k};if ~ismember(numel(r.bytes),[112 168 200]),continue;end
 d=m600check.decodeTruthPacket(r.bytes,struct('expected_copter_id',1,'expected_vehicle_type',5));assert(d.valid);
 t(end+1,1)=d.time_s;p(end+1,:)=d.position_ned_m(:).';angles(end+1,:)=d.euler_rad(:).';rpm(end+1,:)=d.motor_rpm(1:6); %#ok<AGROW>
end
active=t>=min(sim)&t<=max(sim);
observedYaw=unwrap(angles(active,3));
summary=struct('run',char(runRoot),'component_only',true,'complete_enmpc_gp_method',false, ...
 'short_completed',a.result.window_completed,'safe_ground',a.result.safe_ground, ...
 'retained_board_failure_lines',failureLines, ...
 'board_joint_installs_last_observed',counter(end),'exported_committed_observations',numel(counter), ...
 'board_publication_span_s',pub(end)-pub(1), ...
 'mean_interval_from_counter_deltas_ms',1000*(pub(end)-pub(1))/(counter(end)-counter(1)), ...
 'individual_control_period_distribution_available',false, ...
 'interval_caveat','RLC is decimated; counter-normalized means describe average intervals.', ...
 'successful_six_fragment_inputs',size(target,1),'nonzero_operator_velocity_inputs',nnz(any(abs(target(:,2:4))>0,2)), ...
 'maximum_absolute_operator_velocity',max(abs(target(:,2:4)),[],1), ...
 'nonzero_operator_yaw_inputs',nnz(target(:,1)), ...
 'positive_operator_yaw_inputs',nnz(target(:,1)>0),'negative_operator_yaw_inputs',nnz(target(:,1)<0), ...
 'maximum_absolute_operator_yaw_rate_rad_s',max(abs(target(:,1))), ...
 'truth_unwrapped_yaw_range_deg',(max(observedYaw)-min(observedYaw))*180/pi, ...
 'observed_plant_thrust_minmax_n',[min(thrust,[],1);max(thrust,[],1)], ...
 'published_virtual_controls_minmax',[min(controls,[],1);max(controls,[],1)], ...
 'reference6_minmax',[min(reference,[],1);max(reference,[],1)], ...
 'truth_rows_in_input_sim_span',nnz(active),'truth_position_minmax_ned_m',[min(p(active,:),[],1);max(p(active,:),[],1)], ...
 'truth_euler_minmax_rad',[min(angles(active,:),[],1);max(angles(active,:),[],1)], ...
 'truth_rotor_rpm_minmax',[min(rpm(active,:),[],1);max(rpm(active,:),[],1)], ...
 'hardware_actions_in_analysis',0);
path=fullfile(runRoot,'USB_RC_OBSERVED_SUMMARY.json');assert(~isfile(path));
f=fopen(path,'w','n','UTF-8');assert(f>=0);cl=onCleanup(@()fclose(f));fprintf(f,'%s\n',jsonencode(summary,PrettyPrint=true));
disp(jsonencode(summary));
end
