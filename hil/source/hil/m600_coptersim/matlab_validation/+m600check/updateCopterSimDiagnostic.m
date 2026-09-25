function state=updateCopterSimDiagnostic(state,bytes,receivedAtS,expectedCopterId,requireTerrainExtension)
%UPDATECOPTERSIMDIAGNOSTIC Pure official-packet observer used by live adapter.
% No I/O. Duplicates refresh receive age, NOT source-progress age. A fault
% remains latched for the entire observer; only a new explicit run starts [].
if nargin<5,requireTerrainExtension=false;end
assert(islogical(requireTerrainExtension)&&isscalar(requireTerrainExtension), ...
    'm600check:TerrainObserverContract');
if isempty(state)
    state=struct('packet_count',0,'valid_packet_count',0, ...
        'last_receive_s',-Inf,'last_progress_receive_s',-Inf, ...
        'last_source_time_s',NaN,'progress_observed',false, ...
        'last_decoded',[],'fatal_reason','','first_fault',[], ...
        'terrain_extension_required',requireTerrainExtension);
end
% A session cannot downgrade its diagnostic identity or reuse legacy credit.
assert(isfield(state,'terrain_extension_required')&& ...
    isequal(state.terrain_extension_required,requireTerrainExtension), ...
    'm600check:TerrainObserverContractChanged');
assert(isscalar(receivedAtS)&&isfinite(receivedAtS)&&receivedAtS>=0);
if requireTerrainExtension
    decoded=m600check.decodeCopterSimTerrainDiagnostics(bytes,expectedCopterId,state.last_source_time_s,true);
else
    decoded=m600check.decodeCopterSimDiagnostics(bytes,expectedCopterId,state.last_source_time_s);
end
state.packet_count=state.packet_count+1;
state.last_receive_s=receivedAtS;
state.last_decoded=decoded;
if decoded.packet_valid,state.valid_packet_count=state.valid_packet_count+1;end
if decoded.must_stop
    if isempty(state.fatal_reason)
        state.fatal_reason=['MODEL_DIAGNOSTIC:' decoded.status];
        state.first_fault=struct('received_at_s',receivedAtS, ...
            'packet_index',state.packet_count,'decoded',decoded,'raw_bytes',bytes);
    end
    return
end
if isnan(state.last_source_time_s)
    state.last_source_time_s=decoded.sim_time_s;
    state.last_progress_receive_s=receivedAtS;
elseif decoded.sim_time_s>state.last_source_time_s
    state.progress_observed=true;
    state.last_source_time_s=decoded.sim_time_s;
    state.last_progress_receive_s=receivedAtS;
end
end
