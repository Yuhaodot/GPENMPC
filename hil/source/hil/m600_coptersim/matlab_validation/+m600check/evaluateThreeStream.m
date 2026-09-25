function result = evaluateThreeStream(log, policy)
%EVALUATETHREESTREAM Evaluate reference, PX4 estimate and plant truth separately.
%
% Each truth row inside policy.window_s is an anchor and stays in the explicit
% denominator, including invalid rows. Reference/estimate use the most recent
% past source sample received by that truth row's receive time. No interpolation,
% future sample, clock-offset estimation, dropped-error retry or state repair.
% The raw three-stream log is returned untouched in result.raw_streams.
%
% All policy fields are mandatory, with NO default performance/freshness gates:
%   clock_id: all records must use the caller-established common clock.
%   clock_mapping_evidence: text identifying how that alignment was established.
%     The caller supplies and validates this alignment provenance.
%   window_s: [inclusive start,end] common-clock seconds.
%   minimum_anchor_samples: positive integer.
%   required_valid_fraction: [0,1]. Invalid rows are always counted/disclosed.
%   maximum_anchor_gap_s: bound covering both endpoints and inter-row gaps.
%   maximum_source_age_s: struct with reference and estimate values.
%   maximum_receive_lag_s: struct with reference, estimate and truth values.
%   maximum_alignment_skew_s: oldest/newest source-time difference.
%   limits: position_rms_m, position_p95_m, position_max_m,
%     estimator_gap_max_m, velocity_tracking_rms_mps, truth_speed_max_mps.
% Numerical bounds are caller-selected, nonnegative (Inf explicitly disables a
% performance limit). The evaluator never chooses a hover acceptance threshold.
%
% Metrics use VALID aligned rows only and report that population explicitly.
% Data admission and performance are separate; both are required for passed.
% Position error is truth-reference, estimator gap is estimate-truth. RMS uses
% Euclidean vector magnitudes. P95 uses linear rank interpolation (type 7).
% Nonzero movement or nonzero motor output is never an acceptance condition.

validatePolicy(policy);
names = {'reference','estimate','truth'};
assert(isstruct(log) && isscalar(log) && all(isfield(log,names)), ...
    'm600check:EvaluateLog', 'Expected separate reference, estimate and truth.');
result = emptyResult(log,policy);
for index = 1:numel(names)
    result.stream_integrity.(names{index}) = inspectStream(log.(names{index}),policy.clock_id);
end
integrity = all(structfun(@(x)x.valid,result.stream_integrity));
if ~integrity
    result.status = 'INVALID_STREAM_STRUCTURE_CLOCK_OR_TIME_ORDER';
    return
end

truthTimes = [log.truth.time_s];
anchorIndices = find(truthTimes >= policy.window_s(1) & truthTimes <= policy.window_s(2));
result.anchor_rows = numel(anchorIndices);
if isempty(anchorIndices)
    result.status = 'NO_TRUTH_ANCHORS_IN_DECLARED_WINDOW';
    return
end
anchors = log.truth(anchorIndices);
anchorTimes = [anchors.time_s];
gaps = [anchorTimes(1)-policy.window_s(1), diff(anchorTimes), ...
    policy.window_s(2)-anchorTimes(end)];
result.maximum_observed_anchor_gap_s = max(gaps);
result.window_coverage_pass = max(gaps) <= policy.maximum_anchor_gap_s;
result.first_anchor_s = anchorTimes(1);
result.last_anchor_s = anchorTimes(end);
rows = repmat(emptyRow(),numel(anchors),1);
referenceCursor = 0; estimateCursor = 0;

for index = 1:numel(anchors)
    anchor = anchors(index);
    [referenceCursor,ref] = pastSample(log.reference,referenceCursor,anchor);
    [estimateCursor,est] = pastSample(log.estimate,estimateCursor,anchor);
    row = emptyRow();
    row.truth_index = anchorIndices(index);
    row.truth_time_s = anchor.time_s;
    row.truth_received_at_s = anchor.received_at_s;
    row.truth_position_ned_m = anchor.position_ned_m;
    row.truth_velocity_ned_mps = anchor.velocity_ned_mps;
    reasons = strings(0,1);
    reasons = validateState(anchor,'TRUTH',policy.maximum_receive_lag_s.truth,reasons);
    if isempty(ref)
        reasons(end+1,1) = "REFERENCE_MISSING_AT_ANCHOR";
    else
        row.reference_index = referenceCursor;
        row.reference_time_s = ref.time_s;
        row.reference_received_at_s = ref.received_at_s;
        row.reference_position_ned_m = ref.position_ned_m;
        row.reference_velocity_ned_mps = ref.velocity_ned_mps;
        row.reference_age_s = anchor.time_s-ref.time_s;
        reasons = validateState(ref,'REFERENCE',policy.maximum_receive_lag_s.reference,reasons);
        if row.reference_age_s > policy.maximum_source_age_s.reference
            reasons(end+1,1) = "REFERENCE_STALE";
        end
    end
    if isempty(est)
        reasons(end+1,1) = "ESTIMATE_MISSING_AT_ANCHOR";
    else
        row.estimate_index = estimateCursor;
        row.estimate_time_s = est.time_s;
        row.estimate_received_at_s = est.received_at_s;
        row.estimate_position_ned_m = est.position_ned_m;
        row.estimate_velocity_ned_mps = est.velocity_ned_mps;
        row.estimate_age_s = anchor.time_s-est.time_s;
        reasons = validateState(est,'ESTIMATE',policy.maximum_receive_lag_s.estimate,reasons);
        if row.estimate_age_s > policy.maximum_source_age_s.estimate
            reasons(end+1,1) = "ESTIMATE_STALE";
        end
    end
    if ~isempty(ref) && ~isempty(est)
        row.alignment_skew_s = anchor.time_s-min(ref.time_s,est.time_s);
        if row.alignment_skew_s > policy.maximum_alignment_skew_s
            reasons(end+1,1) = "SOURCE_TIME_ALIGNMENT_SKEW";
        end
    end
    row.valid = isempty(reasons);
    if row.valid
        row.reason = 'VALID_ALIGNED_DIAGNOSTIC_ROW';
        row.tracking_error_ned_m = anchor.position_ned_m-ref.position_ned_m;
        row.estimator_gap_ned_m = est.position_ned_m-anchor.position_ned_m;
        row.velocity_tracking_error_ned_mps = anchor.velocity_ned_mps-ref.velocity_ned_mps;
        row.tracking_error_m = norm(row.tracking_error_ned_m);
        row.estimator_gap_m = norm(row.estimator_gap_ned_m);
        row.velocity_tracking_error_mps = norm(row.velocity_tracking_error_ned_mps);
        row.truth_speed_mps = norm(anchor.velocity_ned_mps);
    else
        row.reason = char(strjoin(reasons,';'));
    end
    rows(index) = row;
end
result.rows = rows;
valid = [rows.valid];
result.valid_rows = nnz(valid);
result.invalid_rows = nnz(~valid);
result.valid_fraction = nnz(valid)/numel(rows);
result.complete_data = all(valid) && result.window_coverage_pass;
result.data_admission_pass = numel(rows) >= policy.minimum_anchor_samples && ...
    result.valid_fraction >= policy.required_valid_fraction && result.window_coverage_pass;
if ~any(valid)
    result.status = 'NO_VALID_THREE_STREAM_ALIGNMENT'; return
end
positionError = [rows(valid).tracking_error_m];
gap = [rows(valid).estimator_gap_m];
velocityError = [rows(valid).velocity_tracking_error_mps];
result.metrics = struct('position_rms_m',sqrt(mean(positionError.^2)), ...
    'position_p95_m',quantileType7(positionError,0.95), ...
    'position_max_m',max(positionError), 'estimator_gap_max_m',max(gap), ...
    'velocity_tracking_rms_mps',sqrt(mean(velocityError.^2)), ...
    'truth_speed_max_mps',max([rows(valid).truth_speed_mps]), ...
    'metric_rows',nnz(valid),'metric_population','VALID_ALIGNED_TRUTH_ANCHORS_ONLY', ...
    'percentile_definition','LINEAR_RANK_TYPE_7');
limitNames = fieldnames(policy.limits);
for index = 1:numel(limitNames)
    name = limitNames{index};
    result.performance_checks.(name) = result.metrics.(name) <= policy.limits.(name);
end
result.performance_pass = all(structfun(@(x)x,result.performance_checks));
result.passed = result.data_admission_pass && result.performance_pass;
if ~result.data_admission_pass
    result.status = 'DATA_ADMISSION_NOT_MET';
elseif ~result.performance_pass
    result.status = 'VALID_EVALUATION_PERFORMANCE_LIMITS_NOT_MET';
else
    result.status = 'PASS_CALLER_DEFINED_THREE_STREAM_CRITERIA';
end
end

function [cursor,sample] = pastSample(stream,cursor,anchor)
while cursor < numel(stream) && stream(cursor+1).time_s <= anchor.time_s && ...
        stream(cursor+1).received_at_s <= anchor.received_at_s
    cursor = cursor+1;
end
if cursor==0, sample=[]; else, sample=stream(cursor); end
end

function reasons = validateState(sample,label,maxLag,reasons)
if any(~isfinite([sample.position_ned_m,sample.velocity_ned_mps]))
    reasons(end+1,1) = string(label)+"_NONFINITE_STATE";
end
lag = sample.received_at_s-sample.time_s;
if lag < 0
    reasons(end+1,1) = string(label)+"_RECEIVED_BEFORE_SOURCE_TIME";
elseif lag > maxLag
    reasons(end+1,1) = string(label)+"_RECEIVE_LAG_EXCEEDED";
end
end

function report = inspectStream(stream,clockId)
report = struct('valid',false,'rows',numel(stream),'reason','');
if ~isstruct(stream) || isempty(stream)
    report.reason='MISSING_STREAM'; return
end
required={'clock_id','time_s','received_at_s','position_ned_m','velocity_ned_mps'};
if ~all(isfield(stream,required))
    report.reason='MISSING_FIELDS'; return
end
for index=1:numel(stream)
    s=stream(index);
    if ~(isnumeric(s.time_s)&&isreal(s.time_s)&&isscalar(s.time_s)&& ...
            isfinite(s.time_s)&&isnumeric(s.received_at_s)&&isreal(s.received_at_s)&& ...
            isscalar(s.received_at_s)&&isfinite(s.received_at_s))
        report.reason='INVALID_TIMESTAMP'; return
    end
    if ~((ischar(s.clock_id)&&isrow(s.clock_id)) || (isstring(s.clock_id)&&isscalar(s.clock_id))) || ...
            ~strcmp(string(s.clock_id),string(clockId))
        report.reason='CLOCK_ID_MISMATCH'; return
    end
    if ~(isnumeric(s.position_ned_m)&&isreal(s.position_ned_m)&&isequal(size(s.position_ned_m),[1,3])&& ...
            isnumeric(s.velocity_ned_mps)&&isreal(s.velocity_ned_mps)&&isequal(size(s.velocity_ned_mps),[1,3]))
        report.reason='INVALID_STATE_SHAPE'; return
    end
end
if any(diff([stream.time_s])<=0)
    report.reason='SOURCE_TIME_NOT_STRICTLY_INCREASING'; return
end
if any(diff([stream.received_at_s])<0)
    report.reason='RECEIVE_TIME_REVERSED'; return
end
report.valid=true;
report.reason='STRUCTURE_AND_DECLARED_CLOCK_CONSISTENT';
end

function validatePolicy(p)
required={'clock_id','clock_mapping_evidence','window_s','minimum_anchor_samples', ...
    'required_valid_fraction','maximum_anchor_gap_s','maximum_source_age_s', ...
    'maximum_receive_lag_s','maximum_alignment_skew_s','limits'};
assert(isstruct(p)&&isscalar(p)&&all(isfield(p,required)), ...
    'm600check:EvaluationPolicy','All explicit evaluation policy fields are required.');
assert(strlength(string(p.clock_id))>0&&isscalar(string(p.clock_id))&& ...
    strlength(string(p.clock_mapping_evidence))>0&&isscalar(string(p.clock_mapping_evidence)), ...
    'm600check:EvaluationPolicy','Common clock and mapping evidence are required.');
assert(isnumeric(p.window_s)&&isreal(p.window_s)&&numel(p.window_s)==2&& ...
    all(isfinite(p.window_s))&&p.window_s(2)>p.window_s(1), ...
    'm600check:EvaluationPolicy','The explicit evaluation window must increase.');
assert(isnumeric(p.minimum_anchor_samples)&&isscalar(p.minimum_anchor_samples)&& ...
    isfinite(p.minimum_anchor_samples)&&p.minimum_anchor_samples>=1&& ...
    p.minimum_anchor_samples==fix(p.minimum_anchor_samples), ...
    'm600check:EvaluationPolicy','minimum_anchor_samples must be a positive integer.');
assert(bound(p.required_valid_fraction)&&p.required_valid_fraction<=1&& ...
    bound(p.maximum_anchor_gap_s)&&bound(p.maximum_alignment_skew_s), ...
    'm600check:EvaluationPolicy','Invalid explicit sample/freshness bounds.');
checkBounds(p.maximum_source_age_s,{'reference','estimate'});
checkBounds(p.maximum_receive_lag_s,{'reference','estimate','truth'});
checkBounds(p.limits,{'position_rms_m','position_p95_m','position_max_m', ...
    'estimator_gap_max_m','velocity_tracking_rms_mps','truth_speed_max_mps'});
end

function checkBounds(value,names)
assert(isstruct(value)&&isscalar(value)&&all(isfield(value,names))&& ...
    numel(fieldnames(value))==numel(names), ...
    'm600check:EvaluationPolicy','Expected every named bound, without unknown fields.');
for index=1:numel(names)
    assert(bound(value.(names{index})),'m600check:EvaluationPolicy', ...
        'All bounds must be nonnegative numeric scalars, optionally Inf.');
end
end

function yes=bound(value)
yes=isnumeric(value)&&isreal(value)&&isscalar(value)&&~isnan(value)&&value>=0;
end

function q=quantileType7(values,p)
values=sort(values);
rank=1+(numel(values)-1)*p;
low=floor(rank); high=ceil(rank);
q=values(low)+(rank-low)*(values(high)-values(low));
end

function r=emptyRow()
r=struct('valid',false,'reason','', ...
    'truth_index',NaN,'reference_index',NaN,'estimate_index',NaN, ...
    'truth_time_s',NaN,'reference_time_s',NaN,'estimate_time_s',NaN, ...
    'truth_received_at_s',NaN,'reference_received_at_s',NaN,'estimate_received_at_s',NaN, ...
    'reference_age_s',NaN,'estimate_age_s',NaN,'alignment_skew_s',NaN, ...
    'reference_position_ned_m',nan(1,3),'estimate_position_ned_m',nan(1,3),'truth_position_ned_m',nan(1,3), ...
    'reference_velocity_ned_mps',nan(1,3),'estimate_velocity_ned_mps',nan(1,3),'truth_velocity_ned_mps',nan(1,3), ...
    'tracking_error_ned_m',nan(1,3),'estimator_gap_ned_m',nan(1,3), ...
    'velocity_tracking_error_ned_mps',nan(1,3),'tracking_error_m',NaN, ...
    'estimator_gap_m',NaN,'velocity_tracking_error_mps',NaN,'truth_speed_mps',NaN);
end

function r=emptyResult(log,policy)
r=struct('schema','M600_THREE_STREAM_EVALUATION_V1','status','NOT_EVALUATED', ...
    'passed',false,'diagnostic_only',true,'controller_or_estimator_feed_permitted',false, ...
    'policy',policy,'clock_mapping_verified_by_this_function',false, ...
    'alignment_rule','LATEST_PAST_SOURCE_SAMPLE_RECEIVED_BY_TRUTH_ANCHOR', ...
    'raw_streams',log,'stream_integrity',struct(), ...
    'anchor_rows',0,'valid_rows',0,'invalid_rows',0,'valid_fraction',0, ...
    'maximum_observed_anchor_gap_s',NaN,'first_anchor_s',NaN,'last_anchor_s',NaN, ...
    'window_coverage_pass',false,'complete_data',false,'data_admission_pass',false, ...
    'performance_pass',false,'performance_checks',struct(), 'metrics',struct(), ...
    'rows',repmat(emptyRow(),0,1));
end
