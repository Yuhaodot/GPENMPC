function report=analyze_delivery_observation_cadence(outputRoot)
% Extract recorded MAVLink sample and receive timing.
arguments,outputRoot (1,1) string,end
build=string(fileparts(fileparts(mfilename('fullpath'))));
parent=string(gpenmpc_external_path('delivery_observation_fixture'));
assert(startsWith(lower(outputRoot),lower(build+"\evidence\"))&&~isfolder(outputRoot));
mkdir(outputRoot);oldPath=path;pg=onCleanup(@()path(oldPath)); %#ok<NASGU>
addpath(fullfile(build,'m600_coptersim','matlab_validation'));
loaded=load(parent,'result');result=loaded.result;t=result.transport_evidence;
assert(isfield(t,'raw_mavlink')&&iscell(t.raw_mavlink));
rows=t.raw_mavlink;n=numel(rows);ids=zeros(n,1);topic=cell(n,1);
for k=1:n,ids(k)=double(rows{k}.message.MsgID);topic{k}=rows{k}.topic;end
[uniqueIds,~,whichId]=unique(ids);counts=accumarray(whichId,1);
messageCounts=table(uniqueIds,counts,'VariableNames',{'message_id','count'});
writetable(messageCounts,fullfile(outputRoot,'MESSAGE_COUNTS.csv'));
wanted=[331,93,32,30,83,230,111];groups=struct();
for id=wanted
    ix=find(ids==id);r=repmat(emptyRow(),numel(ix),1);
    for j=1:numel(ix)
        k=ix(j);m=rows{k}.message;p=m.Payload;
        r(j).raw_index=k;r(j).message_id=id;r(j).rx_s=double(rows{k}.rx_s);
        r(j).system_id=double(m.SystemID);r(j).component_id=double(m.ComponentID);r(j).sequence=double(m.Seq);
        if isfield(p,'time_usec'),r(j).source_time_us=double(p.time_usec);
        elseif isfield(p,'time_boot_ms'),r(j).source_time_us=double(p.time_boot_ms)*1000;end
        if id==331
            r(j).frame=double(p.frame_id);r(j).child_frame=double(p.child_frame_id);
            r(j).estimator_type=double(p.estimator_type);r(j).reset_counter=double(p.reset_counter);
            r(j).essential_finite=all(isfinite(double([p.x,p.y,p.z,p.vx,p.vy,p.vz,p.q(:).',p.rollspeed,p.pitchspeed,p.yawspeed])));
            r(j).quaternion_norm=norm(double(p.q));
        elseif id==93
            r(j).essential_finite=all(isfinite(double(p.controls(1:6))));
            r(j).unused_nan_count=nnz(isnan(double(p.controls(7:end))));
            r(j).mode=double(p.mode);r(j).flags=double(p.flags);
        elseif id==111
            r(j).timesync_tc1_ns=double(p.tc1);r(j).timesync_ts1_ns=double(p.ts1);
        end
    end
    name=sprintf('msg%d',id);groups.(name)=r;
    if ~isempty(r),writetable(struct2table(r),fullfile(outputRoot,sprintf('MESSAGE_%d_TIMING.csv',id)));end
end
cadence=struct();
for id=wanted(wanted~=111)
    name=sprintf('msg%d',id);cadence.(name)=summarize(groups.(name));
end
referenceRows=repmat(struct('raw_tx_index',0,'sent_s',NaN,'message_id',0,'wire_time_boot_ms',NaN, ...
    'target_system',NaN,'target_component',NaN,'first_later_93_rx_s',NaN,'first_later_93_source_us',NaN),0,1);
if isfield(t,'raw_transmit_messages')
    for k=1:numel(t.raw_transmit_messages)
        x=t.raw_transmit_messages{k};m=x.message;if double(m.MsgID)~=84,continue,end
        p=m.Payload;r=struct('raw_tx_index',k,'sent_s',double(x.sent_s),'message_id',84, ...
            'wire_time_boot_ms',double(p.time_boot_ms),'target_system',double(p.target_system), ...
            'target_component',double(p.target_component),'first_later_93_rx_s',NaN,'first_later_93_source_us',NaN);
        a=groups.msg93;
        if ~isempty(a)
            j=find([a.rx_s]>r.sent_s,1);
            if ~isempty(j),r.first_later_93_rx_s=a(j).rx_s;r.first_later_93_source_us=a(j).source_time_us;end
        end
        referenceRows(end+1)=r; %#ok<AGROW>
    end
end
if ~isempty(referenceRows),writetable(struct2table(referenceRows),fullfile(outputRoot,'REFERENCE_TX_AND_LATER_93.csv'));end
pairing=struct('odometry_count',numel(groups.msg331),'actuator_count',numel(groups.msg93), ...
    'odometry_with_later_actuator_source',0,'source_delta_s',stats([]),'receive_delta_s',stats([]), ...
    'interpretation','CHRONOLOGICAL_ORDER_ONLY_NOT_REFERENCE_OR_STATE_CONSUMPTION_BINDING');
delta=[];rxdelta=[];
for k=1:numel(groups.msg331)
    o=groups.msg331(k);a=groups.msg93;
    j=find([a.system_id]==o.system_id&[a.component_id]==o.component_id&[a.source_time_us]>o.source_time_us,1);
    if ~isempty(j)
        delta(end+1)=(a(j).source_time_us-o.source_time_us)/1e6; %#ok<AGROW>
        rxdelta(end+1)=a(j).rx_s-o.rx_s; %#ok<AGROW>
    end
end
pairing.odometry_with_later_actuator_source=numel(delta);pairing.source_delta_s=stats(delta);pairing.receive_delta_s=stats(rxdelta);
sync=struct('recorded_timesync_messages',numel(groups.msg111),'fixed_best',[],'initial_response_count',0, ...
    'periodic_validation_count',0,'time_field_semantics','tc1=board_hrt_ns; ts1=echoed_request_token; no sample-rate conversion');
if isfield(t,'fixed_best_timesync'),sync.fixed_best=t.fixed_best_timesync;end
if isfield(t,'timesync_responses'),sync.initial_response_count=numel(t.timesync_responses);end
if isfield(t,'timesync_validation_responses'),sync.periodic_validation_count=numel(t.timesync_validation_responses);end
if isempty(groups.msg331)
    conclusion='ODOMETRY_CADENCE_UNAVAILABLE_IN_SOURCE_RECORD';
elseif cadence.msg331.nonpositive_source_intervals>0||cadence.msg331.source_intervals_over_10_0001_ms>0
    conclusion='RECORDED_ODOMETRY_NOT_CONTINUOUS_FRESH_ATOMIC_10MS_UPDATES';
else
    conclusion='OBSERVED_SOURCE_INTERVALS_WITHIN_10MS';
end
sourceRoot=string(gpenmpc_external_path('px4_mavlink_source'));
sources=[sourceRoot+"\streams\ODOMETRY.hpp",sourceRoot+"\streams\HIL_ACTUATOR_CONTROLS.hpp", ...
    sourceRoot+"\mavlink_timesync.cpp",sourceRoot+"\mavlink_receiver.cpp", ...
    build+"\host_runtime\+gpenmpcNative\CurrentPhysicalCausalRuntime.m", ...
    build+"\m600_coptersim\matlab_validation\+m600check\advanceCanonicalPx4Observation.m"];
binding=struct('path',{},'sha256',{});
for k=1:numel(sources),binding(end+1)=struct('path',char(sources(k)),'sha256',m600check.fileSha256(sources(k)));end %#ok<AGROW>
parentInfo=dir(parent);
report=struct('schema','MAVLINK_OBSERVATION_CADENCE_V1','parent_path',char(parent), ...
    'parent_bytes',parentInfo.bytes,'parent_sha256',m600check.fileSha256(parent), ...
    'parent_status',result.status,'parent_formal_entered',result.formal_entered, ...
    'parent_formal_completed',result.formal_completed,'parent_failure',result.failure, ...
    'parent_action_counts',result.counts,'observed_window','FULL_RECORDED_SESSION__NO_FORMAL_WINDOW_ENTERED', ...
    'raw_message_count',n,'recorded_subscriptions',{t.subscribed_topics}, ...
    'message_counts',table2struct(messageCounts),'cadence',cadence,'timesync',sync, ...
    'odometry_actuator_chronology',pairing,'reference_tx_count',numel(referenceRows), ...
    'reference_with_later93_count',nnz(isfinite([referenceRows.first_later_93_rx_s])), ...
    'result',conclusion, ...
    'canonical_required_source_dt_s',.01,'runtime_maximum_source_dt_s',.0100001, ...
    'runtime_source_dt_match_tolerance_s',5e-7, ...
    'time_semantics',struct('ODOMETRY_331','vehicle_odometry.timestamp_sample_us', ...
      'HIL_ACTUATOR_93','actuator_outputs_sim.timestamp_us', ...
      'receive','original MATLAB callback dequeue time; not kernel network arrival', ...
      'reference_tx','HOST sent_s and outgoing time_boot_ms identify the sent reference.', ...
      'px4_reference_receipt','In OFFBOARD, the receiver timestamps trajectory_setpoint with hrt_absolute_time; board consumption requires a separate acknowledgement.', ...
      'generation','Source-advance count reconstructed during analysis.'), ...
    'limitations',{{'ODOMETRY availability requires an active subscription.', ...
      'TIMESYNC maps the recorded source and host clocks.', ...
      'A later actuator timestamp proves chronology, not which reference/state was consumed.', ...
      'Analysis uses the recorded samples and source timestamps.'}}, ...
    'source_bindings',binding,'analysis_script_sha256',m600check.fileSha256(mfilename('fullpath')+".m"), ...
    'COM_open',0,'UDP_open',0,'board_actions',0,'model_runs',0);
fid=fopen(fullfile(outputRoot,'RESULT.json'),'w','n','UTF-8');assert(fid>=0);
g=onCleanup(@()fclose(fid));fprintf(fid,'%s\n',jsonencode(report,PrettyPrint=true));clear g
fprintf('005_CADENCE raw=%d ODOM331=%d ACT93=%d refs=%d %s\n',n,numel(groups.msg331),numel(groups.msg93),numel(referenceRows),conclusion);
disp(jsonencode(cadence.msg331));disp(jsonencode(cadence.msg93));
end
function r=emptyRow()
r=struct('raw_index',0,'message_id',0,'system_id',0,'component_id',0,'sequence',0, ...
    'rx_s',NaN,'source_time_us',NaN,'frame',NaN,'child_frame',NaN,'estimator_type',NaN, ...
    'reset_counter',NaN,'essential_finite',NaN,'quaternion_norm',NaN,'unused_nan_count',NaN, ...
    'mode',NaN,'flags',NaN,'timesync_tc1_ns',NaN,'timesync_ts1_ns',NaN);
end
function s=summarize(r)
s=struct('count',numel(r),'source_identity_pairs',[],'first_source_us',NaN,'last_source_us',NaN, ...
    'first_rx_s',NaN,'last_rx_s',NaN,'unique_source_timestamp_count',0,'nonpositive_source_intervals',0, ...
    'duplicate_source_intervals',0,'reversed_source_intervals',0,'source_intervals_over_10_0001_ms',0, ...
    'source_intervals_matching_exact10ms_within_0_5us',0,'source_interval_s',stats([]),'receive_interval_s',stats([]));
if isempty(r),return,end
s.source_identity_pairs=unique([[r.system_id].',[r.component_id].'],'rows');
s.armed_flag_set_count=NaN;
if all(isfinite([r.mode])),s.armed_flag_set_count=nnz(bitand(uint8([r.mode]),uint8(128)));end
s.essential_finite_count=nnz([r.essential_finite]==1);
s.essential_nonfinite_count=nnz([r.essential_finite]==0);
s.recorded_estimator_types=unique([r.estimator_type]);s.recorded_estimator_types=s.recorded_estimator_types(isfinite(s.recorded_estimator_types));
s.recorded_frames=unique([[r.frame].',[r.child_frame].'],'rows');s.recorded_frames=s.recorded_frames(all(isfinite(s.recorded_frames),2),:);
s.first_source_us=r(1).source_time_us;s.last_source_us=r(end).source_time_us;
s.first_rx_s=r(1).rx_s;s.last_rx_s=r(end).rx_s;
s.unique_source_timestamp_count=numel(unique([r.source_time_us]));
dt=diff([r.source_time_us])/1e6;drx=diff([r.rx_s]);
same=all(diff([[r.system_id].',[r.component_id].'],1,1)==0,2).';dt=dt(same);drx=drx(same);
s.nonpositive_source_intervals=nnz(dt<=0);s.duplicate_source_intervals=nnz(dt==0);s.reversed_source_intervals=nnz(dt<0);
s.source_intervals_over_10_0001_ms=nnz(dt>.0100001);
s.source_intervals_matching_exact10ms_within_0_5us=nnz(abs(dt-.01)<=.5e-6);
s.source_interval_s=stats(dt);s.receive_interval_s=stats(drx);
s.duration_mean_source_hz=NaN;if s.last_source_us>s.first_source_us,s.duration_mean_source_hz=(numel(r)-1)*1e6/(s.last_source_us-s.first_source_us);end
end
function s=stats(x)
x=double(x(:));x=x(isfinite(x));s=struct('count',numel(x),'min',NaN,'median',NaN,'p95',NaN,'max',NaN);
if isempty(x),return,end
s.min=min(x);s.median=median(x);s.p95=prctile(x,95);s.max=max(x);
end
