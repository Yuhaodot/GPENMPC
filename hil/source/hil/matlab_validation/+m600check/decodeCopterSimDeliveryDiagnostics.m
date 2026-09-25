function result=decodeCopterSimDeliveryDiagnostics(datagram,expectedCopterId,previousSimTime,policy)
% Pure decoder for explicit delivery extension. No socket or physical action.
% Original raw bytes retained; the temporary V1 copy below validates EXACT
% existing terrain semantics, not a replacement of the recorded datagram.
% policy binds expected_session_token and exact payload/mass-by-generation.
validatePolicy(policy);
if ~isa(datagram,'uint8')||~isvector(datagram)||numel(datagram)~=264
 result=m600check.decodeCopterSimTerrainDiagnostics(datagram,expectedCopterId,previousSimTime,true);
 result.environment_extension=blank();return
end
bytes=datagram(:);p=typecast(bytes(9:264),'double');[~,~,endian]=computer;
if endian=='B',p=swapbytes(p);end
p=p(:);
if ~isfinite(p(26))||p(26)~=2
 result=m600check.decodeCopterSimTerrainDiagnostics(datagram,expectedCopterId,previousSimTime,true);
 result.environment_extension=blank();reject('REQUIRED_DELIVERY_EXTENSION_V2_ABSENT');return
end
% For a real tag-2 packet the former initial tag-1 decode was necessarily
% rejected, then completely overwritten below. Decode its unchanged terrain
% prefix only once. Invalid framing/tag retains the original failure path.
tmp=p;tmp(26)=1;tmp(27:32)=0;
tmpWire=tmp;if endian=='B',tmpWire=swapbytes(tmpWire);end
terrainBytes=bytes;terrainBytes(9:264)=typecast(tmpWire,'uint8');
result=m600check.decodeCopterSimTerrainDiagnostics(terrainBytes,expectedCopterId,previousSimTime,true);
result.raw_datagram=datagram;result.payload=p;result.environment_extension=blank();
result.environment_extension.present=true;result.environment_extension.protocol=2;
result.terrain_extension.tag=2; % Full packet identity; terrain 1:25 did not change.
if ~result.packet_valid,return;end
if any(~isfinite(p(27:32)))||~uintExact(p(27),0,flintmax)|| ...
 ~uintExact(p(28),0,2^32-1)||~uintExact(p(29),0,numel(policy.payload_by_generation_kg)-1)|| ...
 ~uintExact(p(32),0,17)||p(30)<0||p(31)<=0
 reject('DELIVERY_EXTENSION_NONFINITE_OR_RANGE');return
end
status=p(32);generation=p(29);e=blank();e.present=true;e.protocol=2;
e.session_token=p(27);e.applied_frame_generation=p(28);e.applied_payload_generation=generation;
e.actual_payload_kg=p(30);e.actual_total_mass_kg=p(31);e.status_code=status;
e.task_env_failed=status>=1&&status<=15;e.pending=status==17;e.initial_not_applied=status==16;
e.same_model_time_s=p(3);e.applied_time_not_encoded=true;
allowUnbound=isfield(policy,'allow_unbound_pre_session')&&policy.allow_unbound_pre_session&& ...
 status==16&&p(27)==0&&p(28)==0&&generation==0;
if p(27)~=policy.expected_session_token&&~allowUnbound
 result.environment_extension=e;reject('DELIVERY_SESSION_BINDING_MISMATCH');return
end
e.session_bound=p(27)==policy.expected_session_token;
index=generation+1;
if p(30)~=policy.payload_by_generation_kg(index)||p(31)~=policy.mass_by_generation_kg(index)
 result.environment_extension=e;reject('DELIVERY_ACTUAL_MASS_IDENTITY_MISMATCH');return
end
if status==0&&p(28)==0
 result.environment_extension=e;reject('DELIVERY_HEALTHY_WITHOUT_CORE_COMMIT');return
end
if status==16&&(p(28)~=0||generation~=0)
 result.environment_extension=e;reject('DELIVERY_INITIAL_STATUS_WITH_APPLIED_GENERATION');return
end
e.valid=true;
% A valid fault status keeps truthful healthy plant observation available for
% native safety landing; TASK permission is independent and remains false.
e.can_continue_task=e.session_bound&&status==0&&p(28)>0&&result.can_use_as_healthy_observation;
e.mass_ack_valid=e.can_continue_task;
e.keep_same_plant_evolving=true;e.request_plant_reset=false;
result.environment_extension=e;

 function reject(reason)
  result.packet_valid=false;result.status=reason;result.can_use_as_healthy_observation=false;
  result.must_stop=true;result.environment_extension.can_continue_task=false;
  result.environment_extension.mass_ack_valid=false;
 end
end
function e=blank()
e=struct('present',false,'protocol',0,'valid',false,'session_token',0,'session_bound',false, ...
 'applied_frame_generation',0,'applied_payload_generation',0,'actual_payload_kg',NaN, ...
 'actual_total_mass_kg',NaN,'status_code',-1,'task_env_failed',false,'pending',false, ...
 'initial_not_applied',false,'same_model_time_s',NaN,'applied_time_not_encoded',true, ...
 'can_continue_task',false,'mass_ack_valid',false,'keep_same_plant_evolving',true,'request_plant_reset',false);
end
function yes=uintExact(v,a,b),yes=isnumeric(v)&&isscalar(v)&&isreal(v)&&isfinite(v)&&v==fix(v)&&v>=a&&v<=b;end
function validatePolicy(p)
assert(isstruct(p)&&isscalar(p)&&all(isfield(p,{'expected_session_token','payload_by_generation_kg','mass_by_generation_kg'})), ...
 'm600check:DeliveryDecodePolicy','Explicit same-session mass identity required.');
assert(uintExact(p.expected_session_token,1,flintmax),'m600check:DeliveryDecodePolicy','Session token must be an exactly represented integer.');
if isfield(p,'allow_unbound_pre_session')
 assert(islogical(p.allow_unbound_pre_session)&&isscalar(p.allow_unbound_pre_session), ...
 'm600check:DeliveryDecodePolicy','Pre-session observation permission must be explicit logical, never task permission.');
end
x=p.payload_by_generation_kg;m=p.mass_by_generation_kg;
assert(isnumeric(x)&&isreal(x)&&isvector(x)&&numel(x)>=2&&numel(x)<=5&& ...
 all(isfinite(x))&&all(x>=0)&&all(diff(x)<0)&&isnumeric(m)&&isreal(m)&& ...
 isvector(m)&&numel(m)==numel(x)&&all(isfinite(m))&&all(m>0), ...
 'm600check:DeliveryDecodePolicy','Invalid prospective generation/mass identities.');
end
