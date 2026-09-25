function [bytes,fragments]=encode_canonical_local_task_mex(bound,rls,originalReceiveNs,outer,registered,backend)
% Encode validated local-task bindings with the caller-selected MEX.
arguments
    bound (1,1) struct
    rls
    originalReceiveNs
    outer (1,1) struct
    registered (1,1) struct
    backend (1,1) function_handle
end
% Validate the bound RLS, rotor and environment once before encoding.
u=gpenmpcNative.RflyLocalTaskCodec.originalNumericalObservation(bound,rls,originalReceiveNs,registered);
s=u.source;r=u.rotor.original_observation;
fields={s.identity.uid;s.identity.boot_generation;s.sample_us;s.publication_us;s.original_receipt_us; ...
    s.source_generation;s.generation_delta;s.sample_delta_us;r.dll_generation;r.dll_session;r.original_host_receive_ns; ...
    u.payload.original_schedule_generation;u.wind.original_estimate_generation;outer.generation;outer.source_generation; ...
    outer.original_sample_us;outer.original_host_source_receive_ns;outer.original_creation_ns;outer.original_expiry_ns};
assert(all(cellfun(@(v)isa(v,'uint64')&&isscalar(v),fields)), ...
    'gpenmpcNative:LocalTaskMexTypedFields','Original integer fields must remain uint64, never converted through double.');
assert(isa(s.identity.system,'uint8')&&isscalar(s.identity.system)&&isa(s.identity.component,'uint8') ...
    &&isscalar(s.identity.component)&&isa(s.reset_counter,'uint8')&&isscalar(s.reset_counter) ...
    &&numel(r.observed_thrust_n)==6&&numel(u.wind.estimate_xy_mps)==2&&numel(outer.target4)==4, ...
    'gpenmpcNative:LocalTaskMexTypedFields');
integers=vertcat(fields{:});route=[s.identity.system;s.identity.component;s.reset_counter];
values=[double(r.original_sim_time_s);double(r.observed_thrust_n(:));double(u.payload.payload_kg); ...
    double(u.wind.estimate_xy_mps(:));double(outer.target4(:))];
hashes=[u.execution_session_sha256,u.configuration_sha256,u.task_sha256,u.reference_asset_sha256, ...
    hashBytes(s.state_and_origin_sha256),hashBytes(r.original_observation_sha), ...
    hashBytes(u.rotor.verified_association_receipt_sha256),hashBytes(u.payload.original_schedule_evidence_sha256), ...
    hashBytes(u.wind.original_estimate_evidence_sha256)];
if nargout<2
    bytes=backend(integers,uint32(u.leg_index),route,values,hashes,u.original_sensor52);
else
    [bytes,payload,lengths]=backend(integers,uint32(u.leg_index),route,values,hashes,u.original_sensor52);
    % Return six TUNNEL payload structs for serialization by the IO owner.
    fragments=cell(6,1);
    for k=1:6
        fragments{k}=struct('payload_type',uint16(42002),'target_system',registered.identity.system, ...
            'target_component',registered.identity.component,'payload_length',lengths(k),'payload',payload(:,k));
    end
end
end
function h=hashBytes(x)
if isa(x,'uint8'),h=x(:);else,h=uint8(sscanf(char(x),'%2x'));end
assert(numel(h)==32&&any(h~=0),'gpenmpcNative:LocalTaskMexHash');
end
