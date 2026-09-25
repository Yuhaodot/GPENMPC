function report=run_native_hover_telemetry_tests(outputDir)
% Test telemetry handling with MATLAB fixtures.
arguments,outputDir (1,1) string,end
assert(~isfolder(outputDir),'gpenmpc:PreserveEvidence','Output already exists.');
mkdir(outputDir);root=fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(root,'m600_coptersim','matlab_validation'));
checks=struct('name',{},'pass',{},'actual_report',{});
cfg=struct('state_max_age_s',.25,'heartbeat_max_age_s',3);
base=fixture();check('exact_current_optional_nan_pass',base,cfg,true);
s=base;s.estimator_status_observation.payload.flags=uint16(47);
check('only_required_local_output_bits_sufficient',s,cfg,true);
s=base;s.estimator_status_observation.payload.flags=uint16(959);
check('at_rest_const_pos_not_alone_rejected',s,cfg,true);
for name={'estimate','attitude_observation','estimator_status_observation'}
    n=name{1};bound=cfg.state_max_age_s;if contains(n,'estimator'),bound=cfg.heartbeat_max_age_s;end
    s=base;s.(n).mapped_source_time_s=10-bound-.01;
    check([n '_source_stale_despite_fresh_rx'],s,cfg,false);
    s=base;s.(n).rx_s=10-bound-.01;
    check([n '_rx_stale_despite_current_source'],s,cfg,false);
    s=base;s.(n).source_progress_rx_s=10-bound-.01;s.(n).latest_rx_s=9.999;
    s.(n).duplicate_count=30;
    check([n '_duplicates_do_not_replenish_progress'],s,cfg,false);
    s=base;s.(n).source_generation_count=1;s.(n).source_advance_count=0;s.(n).duplicate_count=30;
    check([n '_single_source_generation_not_continuity'],s,cfg,false);
    s=base;s.(n).source_reversal_latched=true;
    check([n '_reverse_remains_rejected_with_current_payload'],s,cfg,false);
    s=base;s.(n).source_invalid_latched=true;
    check([n '_invalid_source_latched'],s,cfg,false);
    s=base;s.(n).mapped_source_time_s=10.02;
    check([n '_negative_age_inside_measured_uncertainty'],s,cfg,true);
    s=base;s.(n).mapped_source_time_s=10.021;
    check([n '_negative_age_exceeds_measured_uncertainty'],s,cfg,false);
end
for name={'roll','pitch','yaw','rollspeed','pitchspeed','yawspeed'}
    s=base;s.attitude_observation.payload.(name{1})=NaN;
    check(['attitude_nonfinite_' name{1}],s,cfg,false);
end
for name={'vel_ratio','pos_horiz_ratio','pos_vert_ratio','mag_ratio','pos_horiz_accuracy','pos_vert_accuracy'}
    s=base;s.estimator_status_observation.payload.(name{1})=NaN;
    check(['estimator_essential_nan_' name{1}],s,cfg,false);
end
for name={'hagl_ratio','tas_ratio'}
    s=base;s.estimator_status_observation.payload.(name{1})=Inf;
    check(['optional_inf_' name{1}],s,cfg,false);
    s=base;s.estimator_status_observation.payload=rmfield(s.estimator_status_observation.payload,name{1});
    check(['optional_missing_not_silently_invented_' name{1}],s,cfg,false);
end
for bit=[1 2 4 8 32]
    s=base;s.estimator_status_observation.payload.flags=bitand(uint16(959),bitcmp(uint16(bit)));
    check(sprintf('missing_required_flag_%d',bit),s,cfg,false);
end
for flags=[1024 2048]
    s=base;s.estimator_status_observation.payload.flags=bitor(uint16(959),uint16(flags));
    check(sprintf('error_flag_%d',flags),s,cfg,false);
end
s=base;s.estimator_status_observation.payload.flags=NaN;check('invalid_flags_nan',s,cfg,false);
s=base;s.estimator_status_observation.payload.flags=47.5;check('invalid_flags_fractional',s,cfg,false);
s=base;s.estimator_status_observation.payload.flags=65536;check('invalid_flags_range',s,cfg,false);
s=base;s.estimate.position_ned_m(2)=Inf;check('local_position_nonfinite',s,cfg,false);
s=base;s.estimate.velocity_ned_mps(2)=NaN;check('local_velocity_nonfinite',s,cfg,false);
s=base;s.clock_valid=false;check('invalid_clock',s,cfg,false);
s=base;s.clock_uncertainty_s=NaN;check('missing_measured_uncertainty',s,cfg,false);
legacy=rmfield(cfg,'heartbeat_max_age_s');check('legacy_config_report_false_without_exception',base,legacy,false);
check('missing_snapshot_report_false_without_exception',struct(),cfg,false);
s=base;s.estimator_status_observation.mapped_source_time_s=8;s.estimator_status_observation.rx_s=8;
s.estimator_status_observation.source_progress_rx_s=8;
check('actual_low_rate_est_two_second_gap_not_point25_gate',s,cfg,true);
s=base;s.estimator_status_observation.payload.vel_ratio=1.5;
check('no_new_innovation_ratio_performance_threshold',s,cfg,true);
source=which('m600check.evaluateNativeHoverTelemetry');
report=struct('schema','NATIVE_HOVER_TELEMETRY_HOST_TESTS_V1','passed',all([checks.pass]), ...
    'checks_total',numel(checks),'checks_passed',nnz([checks.pass]),'checks',checks, ...
    'source',source,'source_sha256',sha(source),'test_sha256',sha(mfilename('fullpath')+".m"), ...
    'source_provenance',{{ ...
    gpenmpc_install_path('rfly','Python38\Lib\site-packages\pymavlink\message_definitions\v1.0\common.xml'), ...
    gpenmpc_external_path('px4_ekf_helper_source'), ...
    gpenmpc_external_path('px4_estimator_status_stream_source')}}, ...
    'COM_open',0,'UDP_open',0,'hardware_actions',0,'controller_or_model_changes',0);
fid=fopen(fullfile(outputDir,'NATIVE_HOVER_TELEMETRY_TESTS.json'),'w');assert(fid>=0);
guard=onCleanup(@()fclose(fid)); %#ok<NASGU>
fwrite(fid,jsonencode(report,PrettyPrint=true),'char');
assert(report.passed,'gpenmpc:TelemetryTestsFailed','Inspect complete retained test report.');
    function check(name,s,cfg,expected)
        r=m600check.evaluateNativeHoverTelemetry(s,cfg);
        checks(end+1)=struct('name',name,'pass',isequal(r.passed,expected),'actual_report',r);
    end
end
function s=fixture()
o=struct('raw_source_time_s',100,'mapped_source_time_s',9.99,'rx_s',9.99, ...
    'source_progress_rx_s',9.99,'latest_rx_s',9.99,'source_generation_count',10, ...
    'source_advance_count',9,'duplicate_count',0,'source_reversal_latched',false,'source_invalid_latched',false);
lp=o;lp.position_ned_m=[0 0 0];lp.velocity_ned_mps=[0 0 0];
att=o;att.payload=struct('roll',0,'pitch',0,'yaw',0,'rollspeed',0,'pitchspeed',0,'yawspeed',0);
est=o;est.payload=struct('flags',uint16(959),'vel_ratio',.01,'pos_horiz_ratio',.01, ...
    'pos_vert_ratio',.01,'mag_ratio',.01,'pos_horiz_accuracy',.5,'pos_vert_accuracy',.5, ...
    'hagl_ratio',NaN,'tas_ratio',NaN);
s=struct('now_s',10,'clock_valid',true,'clock_uncertainty_s',.02, ...
    'estimate',lp,'attitude_observation',att,'estimator_status_observation',est);
end
function h=sha(path)
fid=fopen(path,'rb');assert(fid>=0);c=onCleanup(@()fclose(fid)); %#ok<NASGU>
md=java.security.MessageDigest.getInstance('SHA-256');md.update(fread(fid,Inf,'*uint8'));
h=upper(reshape(dec2hex(typecast(md.digest(),'uint8'),2).',1,[]));
end
