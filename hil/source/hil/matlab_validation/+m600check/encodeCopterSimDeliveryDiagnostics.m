function payload=encodeCopterSimDeliveryDiagnostics(terrainPayload,ack,pending)
%#codegen
% V2 extension following plant and terrain diagnostic fields 1:25.
% No acknowledgement of a sent request: ack is produced only by the same
% core's accepted-step commit, not by the UDP callback or host task logic.
assert(isa(terrainPayload,'double')&&isreal(terrainPayload)&&isequal(size(terrainPayload),[32,1]));
assert(terrainPayload(26)==1&&all(terrainPayload(27:32)==0));
assert(islogical(pending)&&isscalar(pending));
assert(islogical(ack.valid)&&isscalar(ack.valid)&&islogical(ack.task_env_failed)&&isscalar(ack.task_env_failed));
payload=terrainPayload;payload(26)=2;
payload(27:32)=[double(ack.session_token);double(ack.applied_frame_generation); ...
 double(ack.applied_payload_generation);double(ack.actual_payload_kg);double(ack.actual_total_mass_kg);0];
if ack.task_env_failed
 assert(ack.failure_code>=1&&ack.failure_code<=15);payload(32)=double(ack.failure_code);
elseif ~ack.valid
 payload(32)=16; % Explicit initialization without an accepted core step.
elseif pending
 payload(32)=17; % Old applied identity retained; a new core commit is pending.
end
end
