function [x13,accepted,reason]=px4EstimateState(sample,expected,nowNs)
% Map legacy MAVLink or private RSP1 bytes to estimator state.
% The integration plan supplies freshness bounds; plant truth is not a state source.
x13=zeros(13,1);accepted=false;reason='MALFORMED_ESTIMATE';
required={'source','uid','system_id','component_id','boot_generation', ...
    'position_ned_m','velocity_ned_mps','quaternion_wxyz_body_to_ned', ...
    'omega_frd_rad_s','position_rx_ns','attitude_rx_ns','rates_rx_ns', ...
    'position_generation','attitude_generation','rates_generation', ...
    'position_valid','attitude_valid','rates_valid'};
if ~isstruct(sample) || ~isscalar(sample) || ~all(isfield(sample,required)),return;end
limits={'uid','system_id','component_id','boot_generation','maximum_age_ns'};
if ~isstruct(expected) || ~isscalar(expected) || ~all(isfield(expected,limits)) ...
        || ~finiteScalar(nowNs) || ~finiteScalar(expected.maximum_age_ns) ...
        || expected.maximum_age_ns<=0
    reason='INVALID_EXPLICIT_BOUNDARY';return
end
approvedSources=["PX4_EKF2_MAVLINK","PX4_EKF2_MAVLINK_ODOMETRY_331", ...
    "PX4_PRIVATE_VEHICLE_ODOMETRY_RSP1"];
if ~textScalar(sample.source) || ~any(string(sample.source)==approvedSources)
    reason='TRUTH_OR_UNAPPROVED_SOURCE_REJECTED';return
end
if string(sample.source)=="PX4_PRIVATE_VEHICLE_ODOMETRY_RSP1"
    try
        original=gpenmpcNative.RflySnapshotSample(sample.source_export_bytes, ...
            sample.source_host_receive_ns,expected);
        % Reconstruct every supplied public field, not just the numeric state.
        % A caller cannot alter q, identity, receipt time or origin after decode.
        names=fieldnames(original);
        for j=1:numel(names)
            if ~isfield(sample,names{j})||~isequaln(sample.(names{j}),original.(names{j}))
                reason='PRIVATE_SNAPSHOT_RECONSTRUCTION_MISMATCH';return
            end
        end
    catch
        reason='INVALID_PRIVATE_SNAPSHOT_EXPORT';return
    end
end
if string(sample.source)=="PX4_EKF2_MAVLINK_ODOMETRY_331"
    atomicFields={'atomic_estimate','plant_truth_used','sample_timestamp_ns', ...
        'odometry_reset_counter','odometry_frame_id', ...
        'odometry_child_frame_id','odometry_estimator_type'};
    if ~all(isfield(sample,atomicFields)) ...
            || ~(islogical(sample.atomic_estimate)&&isscalar(sample.atomic_estimate) ...
                && sample.atomic_estimate) ...
            || ~(islogical(sample.plant_truth_used) ...
                && isscalar(sample.plant_truth_used)&&~sample.plant_truth_used) ...
            || ~finiteInteger(sample.sample_timestamp_ns) ...
            || sample.sample_timestamp_ns<1 ...
            || ~finiteInteger(sample.odometry_reset_counter) ...
            || double(sample.odometry_frame_id)~=1 ...
            || double(sample.odometry_child_frame_id)~=1 ...
            || double(sample.odometry_estimator_type)~=8 ...
            || sample.position_generation~=sample.attitude_generation ...
            || sample.position_generation~=sample.rates_generation ...
            || sample.position_rx_ns~=sample.attitude_rx_ns ...
            || sample.position_rx_ns~=sample.rates_rx_ns
        reason='NONATOMIC_OR_INVALID_ODOMETRY_SOURCE';return
    end
end
if ~textScalar(sample.uid) || ~textScalar(expected.uid) ...
        || ~strcmp(sample.uid,expected.uid)
    reason='UID_MISMATCH';return
end
idFields={'system_id','component_id','boot_generation'};
for k=1:numel(idFields)
    field=idFields{k};
    if ~finiteScalar(sample.(field)) || ~finiteScalar(expected.(field)) ...
            || sample.(field)~=expected.(field)
        reason='STREAM_OR_BOOT_IDENTITY_MISMATCH';return
    end
end
streams={'position','attitude','rates'};
for k=1:3
    stream=streams{k};rx=sample.([stream '_rx_ns']);
    generation=sample.([stream '_generation']);valid=sample.([stream '_valid']);
    if ~(islogical(valid) && isscalar(valid) && valid) ...
            || ~finiteScalar(generation) || generation<1 || generation~=fix(generation)
        reason='INVALID_OR_UNOBSERVED_STREAM';return
    end
    if ~finiteScalar(rx) || rx<0 || rx>nowNs || nowNs-rx>expected.maximum_age_ns
        reason='STALE_OR_FUTURE_STREAM';return
    end
end
p=sample.position_ned_m;v=sample.velocity_ned_mps;
q=sample.quaternion_wxyz_body_to_ned;w=sample.omega_frd_rad_s;
if ~finiteVector(p,3) || ~finiteVector(v,3) || ~finiteVector(q,4) || ~finiteVector(w,3)
    reason='NONFINITE_OR_WRONG_DIMENSION';return
end
q=double(q(:));n=norm(q);
% Unit-quaternion tolerance is a numerical representation guard only.
% It does not change any attitude/tracking performance criterion.
if abs(n-1)>1e-6
    reason='NONUNIT_QUATERNION';return
end
q=q/n;
C=diag([1 1 -1]);
x13=[C*double(p(:));C*double(v(:));q.*[1;-1;-1;1];double(w(:)).*[-1;-1;1]];
accepted=true;reason='FRESH_SELECTED_PX4_ESTIMATE';
end

function y=finiteScalar(v)
y=isnumeric(v) && isreal(v) && isscalar(v) && isfinite(v);
end
function y=finiteInteger(v)
y=finiteScalar(v)&&v>=0&&v==fix(v)&&v<=double(intmax('uint64'));
end
function y=finiteVector(v,n)
y=isnumeric(v) && isreal(v) && isvector(v) && numel(v)==n && all(isfinite(v(:)));
end
function y=textScalar(v)
y=(ischar(v) && isrow(v)) || (isstring(v) && isscalar(v) && ~ismissing(v));
end

