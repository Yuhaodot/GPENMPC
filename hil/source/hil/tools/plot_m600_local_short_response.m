function metrics=plot_m600_local_short_response(runRoot)
% Plot model motion and six rotor responses from one retained run.
% Export a 183x165 mm PDF, PNG and editable FIG.
% Use receive time for the single-run display.
runRoot=char(runRoot);build=fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(build,'m600_coptersim','matlab_validation'));
source=fullfile(runRoot,'SHORT_HIL','RAW_BOARD_LOCAL_SHORT_HIL.mat');
output=fullfile(runRoot,'OBSERVED_MODEL_RESPONSE');
assert(~isfile([output '.mat']),'gpenmpcPlot:PreserveExisting');
q=load(source,'rawIo','result');r=q.result;
assert(isfinite(r.first_commit_io_s),'gpenmpcPlot:NoControlWindow');
raw=q.rawIo.raw_truth_datagrams;
data=nan(numel(raw),19);n=0;invalid=0;
for k=1:numel(raw)
 p=raw{k};if ~ismember(numel(p.bytes),[112 168 200]),continue,end
 s=m600check.decodeTruthPacket(p.bytes,struct('expected_copter_id',1,'expected_vehicle_type',5));
 if ~s.valid,invalid=invalid+1;continue,end
 n=n+1;data(n,:)=[p.rx_s s.time_s s.position_ned_m s.velocity_ned_mps s.euler_rad s.motor_rpm];
end
data=data(1:n,:);assert(n>1&&invalid==0,'gpenmpcPlot:TruthDecode');
% Preserve all raw rows in data; evaluate only the predeclared active window.
active=data(:,1)>=r.first_commit_io_s & data(:,1)<=r.formal_end_io_s;
assert(nnz(active)>1,'gpenmpcPlot:NoActiveModelRows');
groundIndex=find(data(:,1)<r.first_commit_io_s,1,'last');
assert(~isempty(groundIndex),'gpenmpcPlot:MissingGroundReference');
groundNed=data(groundIndex,3:5);t=data(:,1)-r.first_commit_io_s;
% Invert the DLL animation scale clamp(inPWMs,0,1)*1000 to recover normalized input.
altitude=groundNed(3)-data(:,5);virtualInput=data(:,12:17)/1000;
positionChange=data(active,3:5)-groundNed;
metrics=struct('raw_source',source,'scope','COMPONENT_MODEL_RESPONSE', ...
 'component_initialization',r.method.component_initialization,'raw_truth_rows',n, ...
 'active_model_rows',nnz(active),'invalid_truth_packets',invalid, ...
 'observed_board_commit_count',r.counts.commits, ...
 'observed_commit_span_s',r.last_commit_io_s-r.first_commit_io_s, ...
 'window_completed',r.window_completed,'gp_calls',r.method.gp_calls, ...
 'outer_submissions',r.method.outer.outer_submissions, ...
 'model_time_span_s',max(data(active,2))-min(data(active,2)), ...
 'maximum_model_displacement_m',max(vecnorm(positionChange,2,2)), ...
 'maximum_height_above_start_m',max(altitude(active)), ...
 'six_virtual_inputs_nonzero_rows',sum(all(virtualInput(active,:)>0,2)), ...
 'virtual_input_min',min(virtualInput(active,:),[],1),'virtual_input_max',max(virtualInput(active,:),[],1), ...
 'model_output_scale',1000,'physical_rpm_state_exists',false, ...
 'physical_output_not_inferred_from_virtual_input',true, ...
 'per_tick_100Hz_or_jitter_proven',false,'time_axis','HOST_RX_MINUS_FIRST_OBSERVED_COMMIT_SECONDS');
if ~metrics.component_initialization,metrics.scope='ONE_FULL_METHOD_ATTEMPT_OBSERVED_MODEL_RESPONSE';end
f=figure('Visible','off','Color','w','Units','centimeters','Position',[2 2 18.3 16.5]);
cleanup=onCleanup(@()close(f));
layout=tiledlayout(f,3,1,'TileSpacing','compact','Padding','compact');
axesList=gobjects(3,1);windowEnd=r.formal_end_io_s-r.first_commit_io_s;
visible=t>=-1; % Include the recorded LAND tail.
axesList(1)=nexttile(layout);plot(t(visible),altitude(visible),'Color',[0 .35 .6],'LineWidth',1);
ylabel('Height above start (m)');title('a   CopterSim model response','FontWeight','normal');
axesList(2)=nexttile(layout);plot(t(visible),data(visible,9:11)*180/pi,'LineWidth',.8);
ylabel('Euler angle (deg)');title('b   Model attitude','FontWeight','normal');legend('Roll','Pitch','Yaw','Location','eastoutside');
axesList(3)=nexttile(layout);plot(t(visible),virtualInput(visible,:),'LineWidth',.8);
ylabel('Virtual rotor input (0-1)');title('c   Six model-received virtual inputs','FontWeight','normal');
legend('1','2','3','4','5','6','Location','eastoutside','NumColumns',2);
xlabel(layout,'Host receive time from first observed control commit (s)');
for k=1:3
 set(axesList(k),'FontName','Arial','FontSize',8,'TickDir','out','Box','off');
 xline(axesList(k),0,':','Color',[.4 .4 .4],'HandleVisibility','off');
 xline(axesList(k),windowEnd,'--','Color',[.3 .3 .3],'HandleVisibility','off');
 xlim(axesList(k),[min(t(visible)) max(t(visible))]);grid(axesList(k),'on');
end
label='Board SE(3) component with nominal initialization.';
if ~metrics.component_initialization,label='Full-method attempt; participation is reported separately';end
title(layout,{label,'Dashed line: planned control-window end; subsequent data show safety landing'},'FontSize',9,'FontWeight','normal');
save([output '.mat'],'metrics','data','groundNed','active','source');
savefig(f,[output '.fig']);
exportgraphics(f,[output '.pdf'],'ContentType','vector');
exportgraphics(f,[output '.png'],'Resolution',200);
disp(jsonencode(metrics));
end
