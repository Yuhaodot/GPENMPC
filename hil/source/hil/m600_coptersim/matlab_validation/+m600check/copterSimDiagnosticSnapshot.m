function snapshot=copterSimDiagnosticSnapshot(state,nowS,maximumAgeS)
assert(isscalar(nowS)&&isfinite(nowS)&&nowS>=0);
assert(isscalar(maximumAgeS)&&isfinite(maximumAgeS)&&maximumAgeS>0);
snapshot=struct('model_ready',false,'status','MODEL_DIAGNOSTIC_MISSING', ...
    'receive_age_s',Inf,'source_progress_age_s',Inf, ...
    'maximum_age_s',maximumAgeS,'progress_observed',false, ...
    'last_source_time_s',NaN,'packet_count',0,'valid_packet_count',0, ...
    'decoded',[],'fatal_reason','','first_fault',[]);
if isempty(state),return;end
snapshot.receive_age_s=nowS-state.last_receive_s;
snapshot.source_progress_age_s=nowS-state.last_progress_receive_s;
snapshot.progress_observed=state.progress_observed;
snapshot.last_source_time_s=state.last_source_time_s;
snapshot.packet_count=state.packet_count;
snapshot.valid_packet_count=state.valid_packet_count;
snapshot.decoded=state.last_decoded;
snapshot.fatal_reason=state.fatal_reason;snapshot.first_fault=state.first_fault;
if ~isempty(state.fatal_reason)
    snapshot.status=state.fatal_reason;return
end
if isempty(state.last_decoded)||~state.last_decoded.packet_valid ...
        ||state.last_decoded.must_stop||state.last_decoded.schema_version~=1
    snapshot.status='MODEL_DIAGNOSTIC_INVALID';return
end
if ~state.progress_observed
    snapshot.status='MODEL_DIAGNOSTIC_SOURCE_NOT_YET_ADVANCING';return
end
if snapshot.receive_age_s<0||snapshot.source_progress_age_s<0
    snapshot.status='MODEL_DIAGNOSTIC_HOST_RECEIVE_TIME_INVALID';return
end
if snapshot.receive_age_s>maximumAgeS
    snapshot.status='MODEL_DIAGNOSTIC_RECEIVE_STALE';return
end
if snapshot.source_progress_age_s>maximumAgeS
    snapshot.status='MODEL_DIAGNOSTIC_SOURCE_PROGRESS_STALE';return
end
snapshot.status='MODEL_DIAGNOSTIC_FRESH_HEALTHY_ADVANCING';
snapshot.model_ready=true;
end
