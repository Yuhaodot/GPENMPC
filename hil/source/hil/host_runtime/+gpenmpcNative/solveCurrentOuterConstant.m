function result=solveCurrentOuterConstant(compact,fixedConstant)
% Keep the trajectory and fixed GP/configuration data in the outer worker.
fixed=fixedConstant.Value;
assert(string(compact.source_manifest_sha256)==string(fixed.source_manifest_sha256) ...
    && string(compact.source_binding_sha256)==string(fixed.source_binding_sha256) ...
    && string(compact.configuration_payload_sha256)==string(fixed.configuration_payload_sha256) ...
    && string(compact.gp_model_sha256)==string(fixed.gp_model_sha256), ...
    'gpenmpcNative:ConstantIdentity','Compact snapshot and constant differ.');
binding=[];
if isfield(compact,'original_observation')
    o=compact.original_observation;
    assert(isfield(fixed,'trajectory')&&isfield(fixed,'profile') ...
        &&compact.snapshot_timestamp_ns==double(o.source.original_host_receive_ns), ...
        'gpenmpcNative:ConstantIdentity','Original observation clock and bound assets required.');
    a=struct('sourceRoot',fixed.source_root,'enmpc',fixed.config,'profile',fixed.profile, ...
        'prediction_context',fixed.context,'gp_model',fixed.gp_model, ...
        'configurationBinding',struct('effective_configuration_payload_sha256',fixed.configuration_payload_sha256));
    [full,binding]=gpenmpcNative.makeLocalCommittedOuterSnapshot(a,fixed.trajectory,compact.warm_start, ...
        o.source,o.committed,o.inputs,o.registered,o.dispatch_ns,o.maximum_age_ns,true);
else
    full=compact;
    full=rmfield(full,{'work_root','source_manifest_sha256','source_binding_sha256','gp_model_sha256'});
end
full.source_root=fixed.source_root;full.config=fixed.config;
full.context=fixed.context;full.gp_model=fixed.gp_model;
if isfield(fixed,'trajectory')
    assert(~isfield(compact,'trajectory'),'gpenmpcNative:ConstantTrajectory','No per-update replacement of the bound trajectory.');
    full.trajectory=fixed.trajectory;
end
result=gpenmpcNative.solveCurrentOuter(full);
if ~isempty(binding),result.source_binding=binding;end
result.dynamic_snapshot_only=true;
result.immutable_worker_constant=true;
end
