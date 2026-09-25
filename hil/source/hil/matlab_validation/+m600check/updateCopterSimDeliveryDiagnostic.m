function [modelState,deliveryState,decoded]=updateCopterSimDeliveryDiagnostic(modelState,deliveryState,bytes,receivedAtS,expectedCopterId,policy)
% Pure tag-2 observer for the real ii32d CopterSim channel.
% Model health/time and task-environment identity are deliberately separate:
% an environment fault blocks task progress but does not reset/stop the plant.
assert(isscalar(receivedAtS)&&isfinite(receivedAtS)&&receivedAtS>=0);
[deliveryState,decoded]=m600check.observeCopterSimDeliveryDiagnostics( ...
    deliveryState,bytes,expectedCopterId,policy);
if isempty(modelState)
    modelState=struct('packet_count',0,'valid_packet_count',0, ...
        'last_receive_s',-Inf,'last_progress_receive_s',-Inf, ...
        'last_source_time_s',NaN,'progress_observed',false, ...
        'last_decoded',[],'fatal_reason','','first_fault',[], ...
        'terrain_extension_required',false,'delivery_extension_required',true);
end
assert(isfield(modelState,'delivery_extension_required')&&modelState.delivery_extension_required, ...
    'm600check:DeliveryObserverContractChanged');
modelState.packet_count=modelState.packet_count+1;
modelState.last_receive_s=receivedAtS;modelState.last_decoded=decoded;
if decoded.packet_valid,modelState.valid_packet_count=modelState.valid_packet_count+1;end
if decoded.must_stop
    if isempty(modelState.fatal_reason)
        modelState.fatal_reason=['MODEL_DIAGNOSTIC:' decoded.status];
        modelState.first_fault=struct('received_at_s',receivedAtS, ...
            'packet_index',modelState.packet_count,'decoded',decoded,'raw_bytes',bytes);
    end
    return
end
if isnan(modelState.last_source_time_s)
    modelState.last_source_time_s=decoded.sim_time_s;
    modelState.last_progress_receive_s=receivedAtS;
elseif decoded.sim_time_s>modelState.last_source_time_s
    modelState.progress_observed=true;
    modelState.last_source_time_s=decoded.sim_time_s;
    modelState.last_progress_receive_s=receivedAtS;
end
end
