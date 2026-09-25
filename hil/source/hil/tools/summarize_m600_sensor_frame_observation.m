function summary=summarize_m600_sensor_frame_observation(runRoot,outputDir,options)
% SUMMARIZE_M600_SENSOR_FRAME_OBSERVATION Audit all retained observation rows.
% API: summary = summarize_m600_sensor_frame_observation(runRoot,outputDir)
% post_initialization_start_s (default 20) partitions reporting only;
% preserve preceding rows, peaks and full-window statistics.
% Paired states are the latest independently received values, not simultaneous acquisitions.
% Report source times, receive ages, clock validity and uncertainty for each pair.
if nargin<3,options=struct();end
assert(isstruct(options)&&isscalar(options)&&all(ismember(fieldnames(options), ...
    {'post_initialization_start_s'})),'m600check:ObservationOptions');
if ~isfield(options,'post_initialization_start_s'),options.post_initialization_start_s=20;end
split=options.post_initialization_start_s;
assert(isnumeric(split)&&isscalar(split)&&isfinite(split)&&split>=0,'m600check:ObservationSplit');
buildRoot=fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(buildRoot,'matlab_validation'),fullfile(buildRoot,'m600_coptersim','matlab_validation'));
runRoot=char(runRoot);outputDir=char(outputDir);
rawPath=fullfile(runRoot,'INITIALIZATION_MATLAB','RAW_INITIALIZATION_OBSERVATION.mat');
rowsPath=fullfile(runRoot,'INITIALIZATION_MATLAB','OBSERVATION_ROWS.csv');
indexPath=fullfile(runRoot,'CONSUMED_INPUTS','SOURCE_INDEX.json');
planPath=fullfile(runRoot,'CONSUMED_INPUTS','CONSUMED_PLAN.json');
jsonPath=fullfile(outputDir,'SENSOR_FRAME_OBSERVATION_SUMMARY.json');
csvPath=fullfile(outputDir,'SENSOR_FRAME_OBSERVATION_ROWS.csv');
assert(~isfile(jsonPath)&&~isfile(csvPath),'m600check:ObservationOutputExists','Never overwrite an existing analysis.');
for path={rawPath,rowsPath,indexPath,planPath},assert(isfile(path{1}),'m600check:ObservationInputMissing','Missing %s',path{1});end
loaded=load(rawPath,'result');r=loaded.result;
assert(strcmp(r.schema,'M600_COPTER_INITIALIZATION_OBSERVATION_V1'),'m600check:ObservationSchema');
rows=struct2table(r.rows,'AsArray',true);sourceRows=readtable(rowsPath,'TextType','string','VariableNamingRule','preserve');
n=height(rows);assert(n==numel(r.snapshots)&&n==height(sourceRows)&&n>0,'m600check:ObservationAccounting');
assert(all(abs(rows.host_time_s-sourceRows.host_time_s)<1e-10)&& ...
    all(abs(rows.elapsed_s-sourceRows.elapsed_s)<1e-10),'m600check:ObservationRowsMismatch');
e=r.transport_evidence;consumed=jsondecode(fileread(indexPath));plan=jsondecode(fileread(planPath));
identity=verifyConsumed(consumed);
assert(all([identity.matches]),'m600check:ConsumedInputMismatch','A retained input differs from SOURCE_INDEX.');
% Decode raw truth once; retain counts for unsupported/invalid inputs instead
% of quietly turning absence into a zero attitude or dropping fault rows.
td=e.raw_truth_datagrams;nt=numel(td);tr=nan(nt,1);ts=tr;te=nan(nt,3);tv=false(nt,1);tl=zeros(nt,1);
reason=cell(nt,1);
for k=1:nt
    entry=td{k};tr(k)=double(entry.rx_s);tl(k)=numel(entry.bytes);
    if ismember(tl(k),[112,168,200])
        d=m600check.decodeTruthPacket(uint8(entry.bytes),struct('expected_copter_id',r.config.target_system,'expected_vehicle_type',5));
        tv(k)=d.valid;reason{k}=d.reason;
        if d.valid,ts(k)=d.time_s;te(k,:)=d.euler_rad;end
    else,reason{k}='NON_TRUTH_COEXISTING_DATAGRAM_PRESERVED_IN_RAW';end
end
% Use ATTITUDE as the estimator observation.
mv=e.raw_mavlink;ma=find(cellfun(@(v)strcmp(v.topic,'ATTITUDE'),mv));na=numel(ma);
ar=nan(na,1);at=ar;ae=nan(na,3);
for k=1:na
    a=mv{ma(k)};p=a.message.Payload;ar(k)=double(a.rx_s);
    at(k)=double(p.time_boot_ms)/1000;ae(k,:)=double([p.roll,p.pitch,p.yaw]);
end
numberNames={'snapshot_index','snapshot_time_s','snapshot_to_row_latency_s','attitude_raw_index','truth_raw_index','attitude_rx_s','truth_pair_rx_s', ...
    'attitude_raw_source_s','attitude_mapped_source_s','truth_pair_raw_source_s','truth_pair_mapped_source_s', ...
    'attitude_receive_age_s','attitude_source_age_s','truth_receive_age_s','truth_source_age_s', ...
    'estimate_receive_age_s','estimate_source_age_s','attitude_truth_source_skew_s', ...
    'px4_roll_deg','px4_pitch_deg','px4_yaw_deg','truth_roll_deg','truth_pitch_deg','truth_yaw_deg', ...
    'roll_error_deg','pitch_error_deg','yaw_error_deg','so3_error_deg','tilt_error_deg', ...
    'estimate_n_m','estimate_e_m','estimate_d_m','truth_n_m','truth_e_m','truth_d_m', ...
    'estimate_vn_mps','estimate_ve_mps','estimate_vd_mps','truth_vn_mps','truth_ve_mps','truth_vd_mps', ...
    'unregistered_position_gap_n_m','unregistered_position_gap_e_m','unregistered_position_gap_d_m', ...
    'unregistered_position_gap_norm_m','velocity_gap_n_mps','velocity_gap_e_mps','velocity_gap_d_mps','velocity_gap_norm_mps'};
for name=numberNames,rows.(name{1})=nan(n,1);end
flagNames={'attitude_raw_match','truth_raw_match','finite_attitude_pair','attitude_receive_fresh', ...
    'attitude_source_fresh','truth_receive_fresh','truth_source_fresh','estimate_receive_fresh', ...
    'estimate_source_fresh','attitude_truth_time_within_existing_bound','diagnostic_pair_timing_qualified', ...
    'finite_position_velocity_pair'};
for name=flagNames,rows.(name{1})=false(n,1);end
rows.reporting_partition=repmat("INITIALIZATION_REPORTING_PARTITION",n,1);
rows.reporting_partition(rows.elapsed_s>=split)="COMPLETE_POST_INITIALIZATION_REPORTING_PARTITION";
stateAge=r.config.state_max_age_s;
for k=1:n
    s=r.snapshots{k};rows.snapshot_index(k)=k;rows.snapshot_time_s(k)=s.now_s;
    rows.snapshot_to_row_latency_s(k)=rows.host_time_s(k)-s.now_s;
    assert(rows.snapshot_to_row_latency_s(k)>=0,'m600check:SnapshotTimeMismatch');
    p=s.attitude;ai=[];ti=[];
    if ~isempty(p)
        source=double(p.time_boot_ms)/1000;rpy=double([p.roll,p.pitch,p.yaw]);
        ai=find(at==source&ar<=s.now_s&all(ae==rpy,2),1,'last');
        rows.attitude_raw_source_s(k)=source;
        rows{k,{'px4_roll_deg','px4_pitch_deg','px4_yaw_deg'}}=rpy*180/pi;
        if ~isempty(ai)
            rows.attitude_raw_match(k)=true;rows.attitude_raw_index(k)=ma(ai);
            rows.attitude_rx_s(k)=ar(ai);rows.attitude_receive_age_s(k)=s.now_s-ar(ai);
        end
    end
    if ~isempty(s.truth)
        q=s.truth;ti=find(tv&tr==q.rx_s&ts==q.raw_source_time_s,1,'last');
        rows.truth_pair_rx_s(k)=q.rx_s;rows.truth_pair_raw_source_s(k)=q.raw_source_time_s;
        rows.truth_pair_mapped_source_s(k)=q.sample.time_s;
        rows.truth_receive_age_s(k)=s.now_s-q.rx_s;rows.truth_source_age_s(k)=s.now_s-q.sample.time_s;
        rows{k,{'truth_n_m','truth_e_m','truth_d_m'}}=q.position_ned_m;
        rows{k,{'truth_vn_mps','truth_ve_mps','truth_vd_mps'}}=q.velocity_ned_mps;
        if ~isempty(ti)
            rows.truth_raw_match(k)=true;rows.truth_raw_index(k)=ti;
            rows{k,{'truth_roll_deg','truth_pitch_deg','truth_yaw_deg'}}=te(ti,:)*180/pi;
        end
    end
    if ~isempty(s.estimate)
        q=s.estimate;offset=q.sample.time_s-q.raw_source_time_s;
        rows.attitude_mapped_source_s(k)=rows.attitude_raw_source_s(k)+offset;
        rows.attitude_source_age_s(k)=s.now_s-rows.attitude_mapped_source_s(k);
        rows.estimate_receive_age_s(k)=s.now_s-q.rx_s;rows.estimate_source_age_s(k)=s.now_s-q.sample.time_s;
        rows{k,{'estimate_n_m','estimate_e_m','estimate_d_m'}}=q.position_ned_m;
        rows{k,{'estimate_vn_mps','estimate_ve_mps','estimate_vd_mps'}}=q.velocity_ned_mps;
    end
    % Calculate geometry even on stale/fault/invalid-clock rows, while clearly
    % keeping those rows unqualified for time-aligned interpretation.
    ep=rows{k,{'px4_roll_deg','px4_pitch_deg','px4_yaw_deg'}}*pi/180;
    tp=rows{k,{'truth_roll_deg','truth_pitch_deg','truth_yaw_deg'}}*pi/180;
    times=[rows.attitude_mapped_source_s(k),rows.truth_pair_mapped_source_s(k)];
    rows.finite_attitude_pair(k)=~isempty(ai)&&~isempty(ti)&&all(isfinite([ep,tp]));
    if rows.finite_attitude_pair(k)
        % The pure helper requires finite source times. When clock unavailable,
        % use same-snapshot indices only to compute rotations, NEVER qualify.
        helperTimes=times;if any(~isfinite(times)),helperTimes=[0,0];end
        c=m600check.compareIndependentAttitude(ep.',tp.',helperTimes(1),helperTimes(2),stateAge);
        rows{k,{'roll_error_deg','pitch_error_deg','yaw_error_deg'}}=c.wrapped_euler_error_rad.'*180/pi;
        rows.so3_error_deg(k)=c.attitude_geodesic_error_rad*180/pi;rows.tilt_error_deg(k)=c.tilt_error_rad*180/pi;
    end
    if all(isfinite(times)),rows.attitude_truth_source_skew_s(k)=abs(diff(times));end
    for pair={'attitude','truth','estimate'}
        name=pair{1};receive=rows.([name '_receive_age_s'])(k);sourceAge=rows.([name '_source_age_s'])(k);
        rows.([name '_receive_fresh'])(k)=isfinite(receive)&&receive>=0&&receive<=stateAge;
        rows.([name '_source_fresh'])(k)=isfinite(sourceAge)&&isfinite(s.clock_uncertainty_s)&& ...
            sourceAge>=-s.clock_uncertainty_s&&sourceAge<=stateAge;
    end
    rows.attitude_truth_time_within_existing_bound(k)=isfinite(rows.attitude_truth_source_skew_s(k))&& ...
        rows.attitude_truth_source_skew_s(k)<=stateAge;
    rows.diagnostic_pair_timing_qualified(k)=rows.finite_attitude_pair(k)&&logical(s.clock_valid)&& ...
        rows.attitude_receive_fresh(k)&&rows.attitude_source_fresh(k)&&rows.truth_receive_fresh(k)&& ...
        rows.truth_source_fresh(k)&&rows.attitude_truth_time_within_existing_bound(k);
    a=rows{k,{'estimate_n_m','estimate_e_m','estimate_d_m'}};b=rows{k,{'truth_n_m','truth_e_m','truth_d_m'}};
    va=rows{k,{'estimate_vn_mps','estimate_ve_mps','estimate_vd_mps'}};vb=rows{k,{'truth_vn_mps','truth_ve_mps','truth_vd_mps'}};
    rows.finite_position_velocity_pair(k)=all(isfinite([a,b,va,vb]));
    if rows.finite_position_velocity_pair(k)
        rows{k,{'unregistered_position_gap_n_m','unregistered_position_gap_e_m','unregistered_position_gap_d_m'}}=a-b;
        rows.unregistered_position_gap_norm_m(k)=norm(a-b);
        rows{k,{'velocity_gap_n_mps','velocity_gap_e_mps','velocity_gap_d_mps'}}=va-vb;
        rows.velocity_gap_norm_mps(k)=norm(va-vb);
    end
end
summary=struct('schema','M600_SENSOR_FRAME_OBSERVATION_OFFLINE_V1', ...
    'status','DESCRIPTIVE_SAME_SNAPSHOT_SENSOR_FRAME_OBSERVATION__NOT_FLIGHT_ADMISSION', ...
    'run_root',runRoot,'generated_utc',char(datetime('now','TimeZone','UTC','Format','yyyy-MM-dd''T''HH:mm:ss.SSS''Z''')), ...
    'hardware_actions',0,'COM_UDP_model_Dll_execution',0,'flight_pass_claim',false, ...
    'origin_fitting_performed',false,'truth_or_estimate_modified',false,'calibration_offsets_compensated',false, ...
    'input_raw_mat',fileIdentity(rawPath),'input_rows',fileIdentity(rowsPath), ...
    'input_consumed_index',fileIdentity(indexPath),'input_consumed_plan',fileIdentity(planPath), ...
    'consumed_inputs',identity,'source',fileIdentity([mfilename('fullpath') '.m']), ...
    'attitude_helper',fileIdentity(which('m600check.compareIndependentAttitude')), ...
    'truth_decoder',fileIdentity(which('m600check.decodeTruthPacket')), ...
    'input_observation_status',r.status,'input_window_completed',r.window_completed, ...
    'input_elapsed_s',r.elapsed_observation_s,'input_stop_reason',r.stop_reason,'input_first_fatal',r.first_fatal, ...
    'input_failure',r.failure,'input_action_counts',r.counts,'input_config',r.config, ...
    'input_events',r.events,'input_transport_fatal',e.fatal,'input_transport_first_fatal',e.first_fatal, ...
    'input_raw_overflow_drops',e.raw_record_overflow_drops,'input_sockets_closed',r.matlab_sockets_closed, ...
    'input_outer_final_safety_still_required',r.outer_final_safety_still_required);
summary.timing=struct('pairing','EXACT_SNAPSHOT_PAYLOAD_AND_EXACT_RAW_SOURCE_RX_MATCH__NO_FUTURE_SELECTION_NO_INTERPOLATION', ...
    'claim',e.timing_claim,'source_mapping','PER_SNAPSHOT_LOCAL_POSITION_PX4_OFFSET_AND_EXISTING_TRUTH_SAMPLE_CLOCK', ...
    'existing_state_max_age_s',stateAge,'pair_skew_limit_source','EXISTING_CALLER_STATE_MAX_AGE_NOT_NEW_PERFORMANCE_GATE', ...
    'undefined_source_times_geometry_only',true,'utc_zero_s',e.utc_zero_s, ...
    'coptersim_start_utc_s',e.coptersim_start_utc_s,'fixed_best_timesync',e.fixed_best_timesync);
summary.partitions=struct('split_elapsed_s',split,'provenance','DECLARED_REPORTING_ONLY_ALL_ROWS_RETAINED_NOT_DATA_SELECTED', ...
    'full_window',statistics(rows,true(n,1)), ...
    'initialization_all_rows',statistics(rows,rows.elapsed_s<split), ...
    'complete_post_initialization_all_rows',statistics(rows,rows.elapsed_s>=split));
summary.accounting=struct('raw_snapshot_count',numel(r.snapshots),'raw_rows_count',numel(r.rows), ...
    'input_csv_rows',height(sourceRows),'output_csv_rows',n,'rows_removed',0, ...
    'raw_mavlink_count',numel(mv),'raw_attitude_count',na,'raw_truth_datagram_count',nt, ...
    'valid_decoded_truth_count',nnz(tv),'supported_invalid_truth_count',nnz(ismember(tl,[112,168,200])&~tv), ...
    'other_datagrams_count',nnz(~ismember(tl,[112,168,200])),'raw_truth_decode_reasons',countText(reason), ...
    'snapshot_attitude_match_count',nnz(rows.attitude_raw_match),'snapshot_truth_match_count',nnz(rows.truth_raw_match), ...
    'truth_and_attitude_packet_counts_not_independent_scientific_denominator',true);
summary.interpretation={ ...
    'Independent PX4 ATTITUDE versus M600 diagnostic truth; neither enters a controller through this analyzer.', ...
    'Native LOCAL_POSITION and model positions may use different origins: reported raw position difference is UNREGISTERED_ORIGIN_GAP, not tracking error.', ...
    'Calibration slots, nonzero offsets and SIM device IDs are reported in consumed/preflight evidence, not treated as faults or numerically undone.', ...
    'Statistics include initialization, stale, model-fault and clock-invalid rows.', ...
    'The installation-rotation inverse predicts removal of duplicated level rotation; compare the prediction with the measured post-change observation.', ...
    'The outer-run receipt records final safety.'};
summary.plan_identity=plan;
if ~isfolder(outputDir),mkdir(outputDir);end
writetable(rows,csvPath);summary.output_csv=fileIdentity(csvPath);
created=java.io.File(jsonPath).createNewFile();assert(created,'m600check:ObservationOutputCreate','Cannot exclusively create output JSON.');
fid=fopen(jsonPath,'w');if fid<0,error('m600check:ObservationOutputCreate','Cannot open newly created JSON.');end
cleanup=onCleanup(@()fclose(fid));fwrite(fid,jsonencode(summary,'PrettyPrint',true),'char');clear cleanup;
fprintf('SENSOR_FRAME_OBSERVATION_OFFLINE rows=%d attitudePairs=%d postRoll=%.9f postPitch=%.9f\n', ...
    n,nnz(rows.finite_attitude_pair),summary.partitions.complete_post_initialization_all_rows.roll_error_deg.mean, ...
    summary.partitions.complete_post_initialization_all_rows.pitch_error_deg.mean);
end

function out=statistics(t,mask)
q=t(mask,:);out=struct('row_count',height(q),'first_elapsed_s',NaN,'last_elapsed_s',NaN);
if ~isempty(q),out.first_elapsed_s=q.elapsed_s(1);out.last_elapsed_s=q.elapsed_s(end);end
for f={'roll_error_deg','pitch_error_deg','yaw_error_deg','so3_error_deg','tilt_error_deg', ...
    'unregistered_position_gap_norm_m','velocity_gap_norm_mps','attitude_truth_source_skew_s', ...
    'attitude_receive_age_s','attitude_source_age_s','truth_receive_age_s','truth_source_age_s', ...
    'estimate_receive_age_s','estimate_source_age_s','clock_uncertainty_s','clock_utc_drift_s'}
    v=q.(f{1});good=isfinite(v);s=struct('finite_count',nnz(good),'nonfinite_count',nnz(~good), ...
        'mean',NaN,'rms',NaN,'minimum',NaN,'maximum',NaN,'maximum_abs',NaN,'maximum_abs_elapsed_s',NaN);
    if any(good)
        x=v(good);s.mean=mean(x);s.rms=sqrt(mean(x.^2));s.minimum=min(x);s.maximum=max(x);
        [s.maximum_abs,k]=max(abs(x));g=find(good);s.maximum_abs_elapsed_s=q.elapsed_s(g(k));
    end
    out.(f{1})=s;
end
for f={'heartbeat_fresh','clock_valid','model_ready','model_fault','identity_present','identity_matches_expected', ...
    'identity_changed','estimate_source_reversed','truth_source_reversed','model_source_reversed', ...
    'finite_attitude_pair','diagnostic_pair_timing_qualified','finite_position_velocity_pair'}
    if ismember(f{1},q.Properties.VariableNames),out.([f{1} '_true_count'])=nnz(q.(f{1}));end
end
out.armed_true_count=nnz(q.armed==1);out.armed_unknown_count=nnz(~isfinite(q.armed));
out.fatal_reasons=countText(cellstr(string(q.fatal)));out.model_reasons=countText(cellstr(string(q.model_reason)));
% Report the final row alongside full-window distributions.
out.last_row=[];if ~isempty(q),out.last_row=table2struct(q(end,:));end
end

function list=countText(values)
values=string(values);[u,~,g]=unique(values);list=repmat(struct('value','','count',0),numel(u),1);
for k=1:numel(u),list(k)=struct('value',char(u(k)),'count',nnz(g==k));end
end

function out=verifyConsumed(index)
assert(isfield(index,'entries'),'m600check:ConsumedIndexSchema');
entries=index.entries;out=repmat(struct('label','','path','','bytes',0,'sha256','','matches',false),numel(entries),1);
for k=1:numel(entries)
    p=entries(k).retained_path;assert(isfile(p),'m600check:ConsumedFileMissing','%s',p);id=fileIdentity(p);
    out(k)=struct('label',entries(k).label,'path',p,'bytes',id.bytes,'sha256',id.sha256, ...
        'matches',id.bytes==entries(k).bytes&&strcmpi(id.sha256,entries(k).sha256));
end
end

function out=fileIdentity(path)
d=dir(path);assert(isscalar(d)&&~d.isdir,'m600check:ObservationIdentity','%s',path);
fid=fopen(path,'rb');assert(fid>=0,'m600check:ObservationRead');cleanup=onCleanup(@()fclose(fid));
md=java.security.MessageDigest.getInstance('SHA-256');
while true,b=fread(fid,1024*1024,'*uint8');if isempty(b),break;end;md.update(b);end
digest=typecast(md.digest(),'uint8');out=struct('path',path,'bytes',d.bytes,'sha256',upper(reshape(dec2hex(digest,2).',1,[])));
end
