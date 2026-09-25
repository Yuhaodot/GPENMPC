function report = evaluateNativeHoverTelemetry(s,cfg)
%EVALUATENATIVEHOVERTELEMETRY Pure current-source telemetry validity report.
% ATT/LP reuse state_max_age_s. Low-rate EST reuses heartbeat_max_age_s as
% an explicit HOST_DIAGNOSTIC_LIVENESS_BOUND.
% Source ages use the adapter's fixed best TIMESYNC mapping. Only its actual
% measured uncertainty permits a negative age.
report=struct('schema','NATIVE_HOVER_TELEMETRY_V1','passed',false,'failure','', ...
    'checks',struct(),'streams',struct(),'estimator_flags',struct(), ...
    'optional_estimator_fields',struct(),'provenance',struct());
report.provenance=struct( ...
    'state_bound','EXISTING_CFG_STATE_MAX_AGE_S__SOURCE_RX_AND_SOURCE_PROGRESS', ...
    'estimator_bound','EXISTING_CFG_HEARTBEAT_MAX_AGE_S__HOST_DIAGNOSTIC_LIVENESS_BOUND', ...
    'source_mapping','SAME_FIXED_BEST_TIMESYNC_MIDPOINT__MEASURED_UNCERTAINTY', ...
    'progress','AT_LEAST_TWO_DISTINCT_SOURCE_TIMESTAMPS__NO_ADDITIONAL_DWELL', ...
    'mavlink_definition','common.xml ESTIMATOR_STATUS_FLAGS', ...
    'px4_definition','PX4 v1.16 src/modules/ekf2/EKF/ekf_helper.cpp get_ekf_soln_status', ...
    'px4_stream','src/modules/mavlink/streams/ESTIMATOR_STATUS.hpp primary selected estimator status');
% Legacy socket-only snapshots remain usable. Admission additionally
% requires the hover heartbeat field.
c=report.checks;
c.configuration=positiveScalar(field(cfg,'state_max_age_s',NaN))&& ...
    positiveScalar(field(cfg,'heartbeat_max_age_s',NaN));
c.current_time=finiteScalar(field(s,'now_s',NaN));
c.clock_valid=isTrue(field(s,'clock_valid',false));
uncertainty=field(s,'clock_uncertainty_s',NaN);
c.measured_clock_uncertainty=finiteScalar(uncertainty)&&uncertainty>=0;
report.checks=c;
if ~allChecks(c),report=failure(report);return;end
now=s.now_s;
[report.streams.local_position,okLP]=stream(field(s,'estimate',[]),now, ...
    cfg.state_max_age_s,uncertainty);
[report.streams.attitude,okATT]=stream(field(s,'attitude_observation',[]),now, ...
    cfg.state_max_age_s,uncertainty);
[report.streams.estimator_status,okEST]=stream(field(s,'estimator_status_observation',[]),now, ...
    cfg.heartbeat_max_age_s,uncertainty);
c.local_position_timing=okLP;c.attitude_timing=okATT;c.estimator_status_timing=okEST;
lp=field(s,'estimate',struct());
c.local_position_finite=finiteVector(field(lp,'position_ned_m',[]),3)&& ...
    finiteVector(field(lp,'velocity_ned_mps',[]),3);
att=field(field(s,'attitude_observation',struct()),'payload',struct());
c.attitude_six_components_finite=finiteFields(att, ...
    {'roll','pitch','yaw','rollspeed','pitchspeed','yawspeed'});
est=field(field(s,'estimator_status_observation',struct()),'payload',struct());
c.estimator_essential_fields_finite=finiteFields(est, ...
    {'vel_ratio','pos_horiz_ratio','pos_vert_ratio','mag_ratio','pos_horiz_accuracy','pos_vert_accuracy'});
optional={'hagl_ratio','tas_ratio'};optionalOK=true;
for k=1:numel(optional)
    name=optional{k};v=field(est,name,[]);
    present=isnumeric(v)&&isreal(v)&&isscalar(v);
    allowed=present&&(isfinite(v)||isnan(v));
    report.optional_estimator_fields.(name)=struct('present',present,'value',v, ...
        'nan_allowed_when_not_fused',true,'is_nan',present&&isnan(v),'allowed',allowed);
    optionalOK=optionalOK&&allowed;
end
c.estimator_optional_fields_valid=optionalOK;
flags=field(est,'flags',NaN);
c.estimator_flags_typed_range=finiteScalar(flags)&&flags==fix(flags)&&flags>=0&&flags<=65535;
required=uint16(47);rejected=uint16(3072);
flagReport=struct('observed',flags,'required_mask',double(required), ...
    'required_bits',[1 2 4 8 32],'rejected_mask',double(rejected), ...
    'rejected_bits',[1024 2048],'missing_required_bits',[], ...
    'present_rejected_bits',[],'const_pos_mode_is_not_alone_rejected',true, ...
    'const_pos_reason','PX4 flag includes vehicle_at_rest', ...
    'AGL_and_global_absolute_position_not_required_for_local_hover',true);
c.estimator_required_outputs_valid=false;c.estimator_error_flags_clear=false;
if c.estimator_flags_typed_range
    f=uint16(flags);
    c.estimator_required_outputs_valid=bitand(f,required)==required;
    c.estimator_error_flags_clear=bitand(f,rejected)==0;
    flagReport.missing_required_bits=flagReport.required_bits(arrayfun(@(b)bitand(f,uint16(b))==0,flagReport.required_bits));
    flagReport.present_rejected_bits=flagReport.rejected_bits(arrayfun(@(b)bitand(f,uint16(b))~=0,flagReport.rejected_bits));
end
report.estimator_flags=flagReport;report.checks=c;
report.passed=allChecks(c);report=failure(report);
end

function [r,passed]=stream(o,now,bound,uncertainty)
r=struct('present',isstruct(o)&&isscalar(o)&&~isempty(fieldnames(o)), ...
    'bound_s',bound,'mapped_source_time_s',NaN,'raw_source_time_s',NaN, ...
    'source_age_s',Inf,'rx_age_s',Inf,'source_progress_age_s',Inf, ...
    'latest_received_age_s',Inf,'source_generation_count',0, ...
    'source_advance_count',0,'duplicate_count',0,'source_reversal_latched',false, ...
    'source_invalid_latched',false,'minimum_allowed_source_age_s',-uncertainty,'checks',struct());
source=field(o,'mapped_source_time_s',NaN);
% Existing LP representation retains its sample and receives only additional
% observation metadata; this fallback preserves that public snapshot shape.
if ~finiteScalar(source),source=field(field(o,'sample',struct()),'time_s',NaN);end
rx=field(o,'rx_s',NaN);progress=field(o,'source_progress_rx_s',NaN);
r.mapped_source_time_s=source;r.raw_source_time_s=field(o,'raw_source_time_s',NaN);
r.source_age_s=now-source;r.rx_age_s=now-rx;r.source_progress_age_s=now-progress;
r.latest_received_age_s=now-field(o,'latest_rx_s',NaN);
for name={'source_generation_count','source_advance_count','duplicate_count', ...
        'source_reversal_latched','source_invalid_latched'}
    r.(name{1})=field(o,name{1},r.(name{1}));
end
c=struct('present',r.present, ...
    'source_finite',finiteScalar(source)&&finiteScalar(r.raw_source_time_s)&&r.raw_source_time_s>=0, ...
    'source_fresh',finiteScalar(r.source_age_s)&&r.source_age_s>=-uncertainty&&r.source_age_s<=bound, ...
    'receive_fresh',finiteScalar(r.rx_age_s)&&r.rx_age_s>=0&&r.rx_age_s<=bound, ...
    'progress_fresh',finiteScalar(r.source_progress_age_s)&&r.source_progress_age_s>=0&&r.source_progress_age_s<=bound, ...
    'distinct_source_progress_observed',finiteScalar(r.source_generation_count)&&r.source_generation_count>=2&& ...
        finiteScalar(r.source_advance_count)&&r.source_advance_count>=1, ...
    'no_source_reversal',isFalse(r.source_reversal_latched), ...
    'no_invalid_source',isFalse(r.source_invalid_latched));
r.checks=c;passed=allChecks(c);r.passed=passed;
end
function value=field(s,name,fallback)
value=fallback;if isstruct(s)&&isscalar(s)&&isfield(s,name),value=s.(name);end
end
function yes=finiteScalar(v),yes=isnumeric(v)&&isreal(v)&&isscalar(v)&&isfinite(v);end
function yes=positiveScalar(v),yes=finiteScalar(v)&&v>0;end
function yes=finiteVector(v,n),yes=isnumeric(v)&&isreal(v)&&numel(v)==n&&all(isfinite(v(:)));end
function yes=finiteFields(s,names)
yes=true;for k=1:numel(names),yes=yes&&finiteScalar(field(s,names{k},NaN));end
end
function yes=isTrue(v),yes=(islogical(v)||isnumeric(v))&&isscalar(v)&&isreal(v)&&isfinite(v)&&v==1;end
function yes=isFalse(v),yes=(islogical(v)||isnumeric(v))&&isscalar(v)&&isreal(v)&&isfinite(v)&&v==0;end
function yes=allChecks(s),v=struct2cell(s);yes=~isempty(v)&&all(cellfun(@isTrue,v));end
function r=failure(r)
names=fieldnames(r.checks);bad=names(~cellfun(@isTrue,struct2cell(r.checks)));
if isempty(bad),r.failure='';else,r.failure=strjoin(bad,';');end
end
