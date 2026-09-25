function report=summarize_m600_initialization_stream_health(runRoot,outputDir)
% SUMMARIZE_M600_INITIALIZATION_STREAM_HEALTH Analyze retained initialization telemetry.
% Write JSON and, when needed, an offline ATTITUDE_VS_PLANT comparison.
% Evaluate estimator health separately from ATT/LP age and packet progression.
runRoot=char(runRoot);outputDir=char(outputDir);
outputPath=fullfile(outputDir,'INITIALIZATION_STREAM_HEALTH.json');
assert(~isfile(outputPath),'m600check:StreamHealthOutputExists');
rawPath=fullfile(runRoot,'INITIALIZATION_MATLAB','RAW_INITIALIZATION_OBSERVATION.mat');
assert(isfile(rawPath),'m600check:StreamHealthRawMissing');a=load(rawPath,'result');r=a.result;
assert(strcmp(r.schema,'M600_COPTER_INITIALIZATION_OBSERVATION_V1'),'m600check:StreamHealthSchema');
assert(~isempty(r.rows),'m600check:StreamHealthNoRows');
start=r.rows(1).host_time_s-r.rows(1).elapsed_s;
if r.window_completed,stop=start+r.duration_contract_s;else,stop=r.rows(end).host_time_s;end
assert(isfinite(start)&&isfinite(stop)&&stop>start,'m600check:StreamHealthWindow');
rawIdentity=identity(rawPath);m=r.transport_evidence.raw_mavlink;
report=struct('schema','M600_INITIALIZATION_RAW_STREAM_HEALTH_V1', ...
    'status','DESCRIPTIVE_RAW_STREAM_CONTINUITY_AND_FLAGS__NOT_FLIGHT_PASS', ...
    'source',identity([mfilename('fullpath') '.m']),'raw_input',rawIdentity,'run_root',runRoot, ...
    'hardware_actions',0,'COM_UDP_model_Dll_execution',0,'flight_pass_claim',false,'scientific_credit',0, ...
    'original_observation_status',r.status,'window_completed',r.window_completed, ...
    'duration_contract_s',r.duration_contract_s,'observed_window_s',[start,stop], ...
    'observation_elapsed_s',r.elapsed_observation_s,'original_stop_reason',r.stop_reason, ...
    'original_first_fatal',r.first_fatal,'original_events',r.events,'original_counts',r.counts, ...
    'fault_latch_cleared',r.fault_latch_cleared,'source_reset_performed',r.source_reset_performed, ...
    'transport_fatal',r.transport_evidence.fatal,'raw_overflow_drops',r.transport_evidence.raw_record_overflow_drops);
for name={'ATTITUDE','LOCAL_POSITION_NED','ESTIMATOR_STATUS'}
    topic=name{1};ix=find(cellfun(@(x)strcmp(x.topic,topic),m));
    rx=nan(numel(ix),1);src=rx;payloads=cell(numel(ix),1);
    for k=1:numel(ix)
        x=m{ix(k)};rx(k)=double(x.rx_s);p=x.message.Payload;payloads{k}=p;
        if strcmp(topic,'ESTIMATOR_STATUS'),src(k)=double(p.time_usec)/1e6;
        else,src(k)=double(p.time_boot_ms)/1000;end
    end
    in=rx>=start&rx<=stop;
    s=struct('topic',topic,'raw_packet_count',numel(ix),'raw_indices',ix, ...
        'before_window_count',nnz(rx<start),'in_window_count',nnz(in),'after_window_count',nnz(rx>stop), ...
        'invalid_receive_time_count',nnz(~isfinite(rx)), ...
        'entire_capture',inspect(rx,src,payloads,start,stop), ...
        'declared_observation_window',inspect(rx(in),src(in),payloads(in),start,stop), ...
        'freshness_acceptance_decided',false,'independent_estimator_health_decided',false);
    if strcmp(topic,'ESTIMATOR_STATUS')
        s.source_timestamp_field='time_usec/1e6';
        s.freshness_basis='LOW_RATE_STATUS_STREAM__REPORT_ACTUAL_SOURCE_AND_RECEIVE_GAPS';
        s.optional_ratios='HAGL/TAS ratios may be NaN when unfused; record each estimator field separately.';
        s.flag_definitions=struct('ATTITUDE',1,'VELOCITY_HORIZ',2,'VELOCITY_VERT',4,'POS_HORIZ_REL',8, ...
            'POS_HORIZ_ABS',16,'POS_VERT_ABS',32,'POS_VERT_AGL',64,'CONST_POS_MODE',128, ...
            'PRED_POS_HORIZ_REL',256,'PRED_POS_HORIZ_ABS',512,'GPS_GLITCH',1024,'ACCEL_ERROR',2048);
        s.flag_interpretation='Record CONST_POS and optional-channel availability during disarmed observation.';
    else
        s.source_timestamp_field='time_boot_ms/1000';
        s.existing_config_state_max_age_s=r.config.state_max_age_s;
        s.freshness_basis='Existing ATT/LP/truth caller state-age bound retained; raw progress and maximum gaps are independently disclosed.';
    end
    report.streams.(topic)=s;
end
rows=struct2table(r.rows,'AsArray',true);
report.snapshot_accounting=struct('rows',height(rows),'snapshots',numel(r.snapshots), ...
    'heartbeat_fresh_count',nnz(rows.heartbeat_fresh),'clock_valid_count',nnz(rows.clock_valid), ...
    'model_ready_count',nnz(rows.model_ready),'model_fault_count',nnz(rows.model_fault), ...
    'armed_count',nnz(rows.armed==1),'identity_present_count',nnz(rows.identity_present), ...
    'identity_matches_expected_count',nnz(rows.identity_matches_expected), ...
    'identity_missing_is_not_invented_as_matching',true);
for f={'estimate_source_reversed','truth_source_reversed','model_source_reversed','raw_model_source_reversed','attitude_source_reversed'}
    if ismember(f{1},rows.Properties.VariableNames),report.snapshot_accounting.([f{1} '_count'])=nnz(rows.(f{1}));end
end
existing=fullfile(runRoot,'SENSOR_ANALYSIS','SENSOR_FRAME_OBSERVATION_SUMMARY.json');
if isfile(existing)
    pair=jsondecode(fileread(existing));assert(strcmpi(pair.input_raw_mat.sha256,rawIdentity.sha256), ...
        'm600check:StreamHealthWrongAttitudeReference','Existing attitude analysis uses a different raw observation.');
else
    addpath(fileparts(mfilename('fullpath')));
    pair=summarize_m600_sensor_frame_observation(runRoot,fullfile(outputDir,'ATTITUDE_VS_PLANT'));
    existing=fullfile(outputDir,'ATTITUDE_VS_PLANT','SENSOR_FRAME_OBSERVATION_SUMMARY.json');
end
report.attitude_vs_plant=struct('summary',identity(existing),'csv',pair.output_csv, ...
    'partitions',pair.partitions,'rows_removed',pair.accounting.rows_removed, ...
    'timing',pair.timing,'origin_fitting_performed',pair.origin_fitting_performed, ...
    'calibration_offsets_compensated',pair.calibration_offsets_compensated);
report.interpretation={ ...
    'A complete observation window only proves capture completion, not sustained ATT/LP/EST health.', ...
    'Each stream needs advancing source timestamps, receive coverage including window endpoints, finite essential fields, plus independently interpreted estimator solution flags.', ...
    'Packet count/unique timestamp count alone is insufficient: maximum receive gap, source-progress silence, repeated timestamps and resets remain visible.', ...
    'ESTIMATOR_STATUS is available in raw_mavlink for offline health analysis.', ...
    'PX4 estimator flags describe the estimator-reported state; plant agreement is assessed separately.', ...
    'Vendor READY marks the initialization lifecycle boundary; the subsequent observation provides measurement data.', ...
    'Any clock/model/source fault inside the observation remains latched.', ...
    'The absence of LOCAL_POSITION may prevent common-clock attitude pairing in the existing analyzer; raw receive/source continuity remains separately measurable.', ...
    'Final board safety belongs to the independent outer finally/postflight evidence, not this offline report.'};
if ~isfolder(outputDir),mkdir(outputDir);end
created=java.io.File(outputPath).createNewFile();assert(created,'m600check:StreamHealthOutputCreate');
f=fopen(outputPath,'w','n','UTF-8');assert(f>=0);cleanup=onCleanup(@()fclose(f));fprintf(f,'%s\n',jsonencode(report,'PrettyPrint',true));clear cleanup;
fprintf('RAW_STREAM_HEALTH ATT=%d LP=%d EST=%d window_complete=%d flight_pass=0 hardware=0\n', ...
    report.streams.ATTITUDE.in_window_count,report.streams.LOCAL_POSITION_NED.in_window_count, ...
    report.streams.ESTIMATOR_STATUS.in_window_count,r.window_completed);
end

function out=inspect(rx,t,p,start,stop)
n=numel(t);out=struct('packet_count',n,'source_unique_count',0,'source_nonfinite_count',nnz(~isfinite(t)), ...
    'source_first_s',NaN,'source_last_s',NaN,'source_span_s',NaN,'receive_span_s',NaN, ...
    'source_repeat_count',0,'source_reversal_count',0,'receive_reversal_count',0, ...
    'max_receive_gap_s',NaN,'max_positive_source_gap_s',NaN,'first_receive_s',NaN,'last_receive_s',NaN, ...
    'window_start_to_first_receive_s',NaN,'last_receive_to_window_end_s',NaN, ...
    'maximum_source_progress_silence_including_endpoints_s',stop-start,'source_over_receive_span_ratio',NaN, ...
    'source_reversals',struct([]),'field_nonfinite',struct(),'flags',struct([]));
if n==0,out.presence='ABSENT';return;end
out.presence='PRESENT_NOT_AUTOMATICALLY_HEALTHY';out.source_unique_count=numel(unique(t(isfinite(t))));
out.source_first_s=t(1);out.source_last_s=t(end);out.source_span_s=t(end)-t(1);
out.first_receive_s=rx(1);out.last_receive_s=rx(end);out.receive_span_s=rx(end)-rx(1);
out.window_start_to_first_receive_s=rx(1)-start;out.last_receive_to_window_end_s=stop-rx(end);
dt=diff(t);dr=diff(rx);out.source_repeat_count=nnz(dt==0);out.source_reversal_count=nnz(dt<0);out.receive_reversal_count=nnz(dr<0);
if ~isempty(dr),out.max_receive_gap_s=max(dr);end
if any(dt>0),out.max_positive_source_gap_s=max(dt(dt>0));end
if out.receive_span_s>0,out.source_over_receive_span_ratio=out.source_span_s/out.receive_span_s;end
rev=find(dt<0);events=repmat(struct('packet_index',0,'receive_s',NaN,'previous_source_s',NaN,'new_source_s',NaN),numel(rev),1);
for k=1:numel(rev),i=rev(k)+1;events(k)=struct('packet_index',i,'receive_s',rx(i),'previous_source_s',t(i-1),'new_source_s',t(i));end
out.source_reversals=events;
% Duplicates do not refresh progression. Retain resets and report the maximum gap.
progress=isfinite(t)&[true;dt>0];pr=rx(progress&rx>=start&rx<=stop);
if ~isempty(pr),out.maximum_source_progress_silence_including_endpoints_s=max([pr(1)-start;diff(pr);stop-pr(end)]);end
fields=fieldnames(p{1});
for j=1:numel(fields)
    name=fields{j};nanCount=0;infCount=0;elements=0;missing=0;rowsBad=0;minimum=Inf;maximum=-Inf;
    for k=1:n
        if ~isfield(p{k},name),missing=missing+1;continue;end
        v=double(p{k}.(name));elements=elements+numel(v);nanCount=nanCount+nnz(isnan(v));infCount=infCount+nnz(isinf(v));
        rowsBad=rowsBad+any(~isfinite(v),'all');finite=v(isfinite(v));
        if ~isempty(finite),minimum=min(minimum,min(finite(:)));maximum=max(maximum,max(finite(:)));end
    end
    if minimum==Inf,minimum=NaN;maximum=NaN;end
    out.field_nonfinite.(name)=struct('element_count',elements,'nan_count',nanCount,'inf_count',infCount, ...
        'nonfinite_message_count',rowsBad,'missing_message_count',missing,'finite_minimum',minimum,'finite_maximum',maximum);
end
if isfield(p{1},'flags')
    flags=cellfun(@(x)double(x.flags),p);u=unique(flags);list=repmat(struct('value',0,'count',0,'first_receive_s',NaN,'last_receive_s',NaN,'set_bit_values',[]),numel(u),1);
    for k=1:numel(u)
        i=find(flags==u(k));bits=2.^(0:15);list(k)=struct('value',u(k),'count',numel(i),'first_receive_s',rx(i(1)), ...
            'last_receive_s',rx(i(end)),'set_bit_values',bits(bitand(uint32(u(k)),uint32(bits))~=0));
    end
    out.flags=list;
end
end

function out=identity(path)
d=dir(path);assert(isscalar(d)&&~d.isdir,'m600check:StreamHealthIdentity');f=fopen(path,'rb');assert(f>=0);cleanup=onCleanup(@()fclose(f));
md=java.security.MessageDigest.getInstance('SHA-256');while true,b=fread(f,1024*1024,'*uint8');if isempty(b),break;end;md.update(b);end
hash=typecast(md.digest(),'uint8');out=struct('path',path,'bytes',d.bytes,'sha256',upper(reshape(dec2hex(hash,2).',1,[])));
end
