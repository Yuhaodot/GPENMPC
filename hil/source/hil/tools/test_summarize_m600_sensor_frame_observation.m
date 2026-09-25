function receipt=test_summarize_m600_sensor_frame_observation(outputDir)
% Test observation summaries using a retained disarmed snapshot.
buildRoot=fileparts(fileparts(mfilename('fullpath')));addpath(fullfile(buildRoot,'tools'));
assert(~isfolder(outputDir),'m600check:ObservationTestExists');mkdir(outputDir);
parent=gpenmpc_external_path('initialization_observation_recording');
a=load(fullfile(parent,'INITIALIZATION_MATLAB','RAW_INITIALIZATION_OBSERVATION.mat'),'result');base=a.result;
s=base.snapshots{end};row=base.rows(end);raw=base.transport_evidence;
att=find(cellfun(@(m)strcmp(m.topic,'ATTITUDE')&&double(m.message.Payload.time_boot_ms)/1000==double(s.attitude.time_boot_ms)/1000&&m.rx_s<=s.now_s,raw.raw_mavlink),1,'last');
truth=find(cellfun(@(m)m.rx_s==s.truth.rx_s,raw.raw_truth_datagrams),1,'last');
assert(~isempty(att)&&~isempty(truth));raw.raw_mavlink=raw.raw_mavlink(att);raw.raw_truth_datagrams=raw.raw_truth_datagrams(truth);
names={};ok=[];scenarios={'positive','clock_invalid_fault_retained','stale_retained','missing_attitude_raw','missing_truth_raw','early_exit_no_late_partition'};
for k=1:numel(scenarios)
    result=base;result.snapshots={s};result.rows=row;result.transport_evidence=raw;
    switch scenarios{k}
        case 'clock_invalid_fault_retained'
            result.snapshots{1}.clock_valid=false;result.rows.clock_valid=false;
            result.snapshots{1}.fatal='FIXTURE_MODEL_OR_CLOCK_FAULT';result.rows.fatal='FIXTURE_MODEL_OR_CLOCK_FAULT';
        case 'stale_retained'
            result.snapshots{1}.now_s=s.now_s+10;result.rows.host_time_s=row.host_time_s+10;result.rows.elapsed_s=row.elapsed_s+10;
        case 'missing_attitude_raw',result.transport_evidence.raw_mavlink={};
        case 'missing_truth_raw',result.transport_evidence.raw_truth_datagrams={};
        case 'early_exit_no_late_partition',result.rows.elapsed_s=1;
    end
    root=fullfile(outputDir,scenarios{k});mkdir(root);mkdir(fullfile(root,'INITIALIZATION_MATLAB'));mkdir(fullfile(root,'CONSUMED_INPUTS'));
    save(fullfile(root,'INITIALIZATION_MATLAB','RAW_INITIALIZATION_OBSERVATION.mat'),'result','-v7.3');
    writetable(struct2table(result.rows,'AsArray',true),fullfile(root,'INITIALIZATION_MATLAB','OBSERVATION_ROWS.csv'));
    copyfile(fullfile(parent,'CONSUMED_INPUTS','SOURCE_INDEX.json'),fullfile(root,'CONSUMED_INPUTS','SOURCE_INDEX.json'));
    copyfile(fullfile(parent,'CONSUMED_INPUTS','CONSUMED_PLAN.json'),fullfile(root,'CONSUMED_INPUTS','CONSUMED_PLAN.json'));
    summary=summarize_m600_sensor_frame_observation(root,fullfile(root,'analysis'));
    q=readtable(summary.output_csv.path,'VariableNamingRule','preserve');
    check([scenarios{k} '_one_input_one_output'],height(q)==1&&summary.accounting.rows_removed==0);
    check([scenarios{k} '_no_hardware_no_flight'],summary.hardware_actions==0&&~summary.flight_pass_claim&&~summary.origin_fitting_performed);
    switch scenarios{k}
        case 'positive'
            reference=atan2(sin(double([s.attitude.roll,s.attitude.pitch,s.attitude.yaw])),cos(double([s.attitude.roll,s.attitude.pitch,s.attitude.yaw])))*180/pi;
            check('actual_old_tilt_reproduced',q.roll_error_deg< -6&&q.roll_error_deg> -7&&q.pitch_error_deg>6&&q.pitch_error_deg<7);
            check('independent_snapshot_attitude_preserved',max(abs([q.px4_roll_deg,q.px4_pitch_deg,q.px4_yaw_deg]-reference))<1e-12);
            check('mapped_source_clock_not_raw_epoch_compared',q.attitude_truth_source_skew_s<0.25&&q.diagnostic_pair_timing_qualified);
        case 'clock_invalid_fault_retained'
            check('clock_fault_does_not_erase_attitude_peak',isfinite(q.roll_error_deg)&&~q.diagnostic_pair_timing_qualified);
            check('fault_text_preserved',strcmp(string(q.fatal),'FIXTURE_MODEL_OR_CLOCK_FAULT'));
        case 'stale_retained'
            check('stale_data_no_qualified_pair',~q.diagnostic_pair_timing_qualified&&~q.attitude_receive_fresh&&~q.truth_source_fresh);
            check('stale_peak_and_row_not_dropped',isfinite(q.so3_error_deg)&&q.attitude_receive_age_s>10);
        case 'missing_attitude_raw'
            check('snapshot_not_substituted_for_raw_match',~q.attitude_raw_match&&~q.finite_attitude_pair&&isnan(q.so3_error_deg));
        case 'missing_truth_raw'
            check('missing_truth_orientation_not_filled_zero',~q.truth_raw_match&&~q.finite_attitude_pair&&isnan(q.truth_roll_deg));
        case 'early_exit_no_late_partition'
            check('early_exit_empty_late_partition_not_secondary_exception',summary.partitions.complete_post_initialization_all_rows.row_count==0&&isnan(summary.partitions.complete_post_initialization_all_rows.roll_error_deg.mean));
    end
end
angles=m600check.compareIndependentAttitude([0;0;pi-1e-6],[0;0;-pi+1e-6],1,1,0);
check('SO3_and_wrapped_yaw_not_spurious_360_degree',abs(angles.attitude_geodesic_error_rad-2e-6)<1e-12&&abs(angles.wrapped_euler_error_rad(3)+2e-6)<1e-12);
receipt=struct('status','PASS_HOST_ONLY_OBSERVATION_ANALYZER_REGRESSION','case_count',numel(ok),'passed_count',nnz(ok), ...
    'checks',struct('name',names,'passed',num2cell(ok)),'hardware_actions',0,'model_execution',0, ...
    'uses_real_snapshot_with_explicit_negative_fixture_edits',true,'source_parent',parent);
f=fopen(fullfile(outputDir,'TEST_RESULT.json'),'w');cleanup=onCleanup(@()fclose(f));fwrite(f,jsonencode(receipt,'PrettyPrint',true),'char');clear cleanup;
assert(all(ok),'m600check:ObservationRegressionFailed');fprintf('OBSERVATION_ANALYZER_TESTS %d/%d PASS\n',nnz(ok),numel(ok));
    function check(name,value)
        names{end+1}=name;ok(end+1)=logical(value);
    end
end
