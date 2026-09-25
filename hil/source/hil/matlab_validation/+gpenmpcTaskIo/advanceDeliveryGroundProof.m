function [next,proof,token]=advanceDeliveryGroundProof(previous,observation,timeS,request,policy)
%ADVANCEDELIVERYGROUNDPROOF Pure, one-shot PX4 ground/disarm service proof.
% This helper supplements the delivery lifecycle. The caller owns dual-ground
% validation, payload execution and ACK, Offboard/rearm and finalization.
%
% policy: service_dwell_s=8, max_heartbeat_age_s, max_extended_age_s,
% max_observation_gap_s, clock_id, expected_uid, require_plant_contact(logical).
% These age/gap values must come from the caller's existing observation policy;
% there are no silent defaults and no new live timing values in this helper.
%
% request: service_id(nonempty text), next_payload_generation(integer 1..4),
% service_active(logical). The caller must retain returned state across calls,
% faults, session changes and delivery phases. [] means a NEW mission proof,
% never retry/reinitialize during an existing delivery to reissue a token.
%
% observation: uid,session_id,boot_id,source_generation,clock_id,
% source_identity_valid(logical), heartbeat, extended_sys_state.
% Each message separately contains uid/session_id/boot_id/source_generation,
% clock_id/received_at_s/valid(logical)/message_name and its own native value:
% heartbeat.armed (0/1); extended_sys_state.landed_state (ON_GROUND=1).
% Optional plant_contact is logical. It NEVER substitutes for either message.
% HEARTBEAT/EXTENDED_SYS_STATE have no native timestamp: received_at_s means
% trusted monotonic receiver time.
% A source_generation is the authenticated receiver/boot epoch, NOT the MAVLink
% 8-bit sequence number. Raw decoder and session-identity verification remain
% caller responsibilities; a host service/plant flag is not independent PX4.
% Caller must feed every received armed/land-state transition (or retain an
% invalid-event latch); a last-value snapshot must not hide intervening flight.
%
% token is [] except on the SINGLE call crossing a complete continuous 8 s.
% It binds the exact next generation/service/boot/session. It records proof at
% issuance only; the caller must recheck current identity/freshness/dual ground
% atomically when applying payload. No held token permits a later unsafe write.
validatePolicy(policy);
if isempty(previous)
    next=struct('schema','GPENMPC_PX4_DELIVERY_GROUND_PROOF_V1','policy',policy, ...
        'binding',[],'service_id','','requested_generation',0,'last_eval_s',NaN, ...
        'last_heartbeat_receive_s',NaN,'last_extended_receive_s',NaN, ...
        'dwell_start_s',NaN,'dwell_s',0,'last_released_generation',0, ...
        'released_service_ids',{{}},'release_count',0,'reset_count',0,'last_reason','NEW_MISSION_PROOF');
else
    next=previous;
    assert(isstruct(next)&&isscalar(next)&&strcmp(next.schema,'GPENMPC_PX4_DELIVERY_GROUND_PROOF_V1')&& ...
        isequaln(next.policy,policy),'gpenmpcTaskIo:GroundProofState','State/policy identity changed.');
end
token=[];reason='';good=false;
if ~finiteScalar(timeS)||timeS<0
    reason='CURRENT_TIME_INVALID';
elseif isfinite(next.last_eval_s)&&timeS<next.last_eval_s
    reason='CURRENT_TIME_REVERSED';
else
    if isfinite(next.last_eval_s),gap=timeS-next.last_eval_s;else,gap=0;end
    next.last_eval_s=timeS;
    if ~validRequest(request)
        reason='SERVICE_REQUEST_UNKNOWN_OR_INVALID';
    elseif ~request.service_active
        reason='NOT_IN_ACTIVE_GROUND_SERVICE';
    else
        [good,reason,binding]=eligible(observation,timeS,policy);
        if good
            identityChanged=~isempty(next.binding)&&~isequal(next.binding,binding);
            serviceChanged=~strcmp(next.service_id,char(request.service_id))|| ...
                next.requested_generation~=double(request.next_payload_generation);
            if identityChanged
                next=resetDwell(next,'BOOT_SESSION_SOURCE_GENERATION_CHANGED');
                next.last_heartbeat_receive_s=NaN;next.last_extended_receive_s=NaN;
            end
            if serviceChanged,next=resetDwell(next,'SERVICE_OR_REQUESTED_GENERATION_CHANGED');end
            next.binding=binding;next.service_id=char(request.service_id);
            next.requested_generation=double(request.next_payload_generation);
            h=double(observation.heartbeat.received_at_s);e=double(observation.extended_sys_state.received_at_s);
            if (isfinite(next.last_heartbeat_receive_s)&&h<next.last_heartbeat_receive_s)|| ...
                    (isfinite(next.last_extended_receive_s)&&e<next.last_extended_receive_s)
                good=false;reason='NATIVE_MESSAGE_RECEIVE_TIME_REVERSED';
            elseif request.next_payload_generation<=next.last_released_generation
                reason='GENERATION_ALREADY_RELEASED';next=resetDwell(next,reason);
            elseif request.next_payload_generation~=next.last_released_generation+1
                good=false;reason='PAYLOAD_GENERATION_NOT_EXACT_NEXT';
            elseif any(strcmp(next.released_service_ids,char(request.service_id)))
                good=false;reason='SERVICE_ID_ALREADY_RELEASED';
            else
                if gap>policy.max_observation_gap_s,next=resetDwell(next,'OBSERVATION_GAP_BROKE_CONTINUITY');end
                if ~isfinite(next.dwell_start_s),next.dwell_start_s=timeS;end
                next.dwell_s=timeS-next.dwell_start_s;
                reason='COLLECTING_CURRENT_PAST_PX4_GROUND_DISARM_PROOF';
                if next.dwell_s>=8.0
                    next.last_released_generation=double(request.next_payload_generation);
                    next.release_count=next.release_count+1;
                    next.released_service_ids{end+1}=char(request.service_id);
                    token=struct('schema','ONE_SHOT_PX4_GROUND_DWELL_RELEASE_TOKEN_V1', ...
                        'service_id',char(request.service_id),'next_payload_generation',double(request.next_payload_generation), ...
                        'binding',binding,'issued_at_s',double(timeS),'ground_disarmed_since_s',next.dwell_start_s, ...
                        'continuous_dwell_s',next.dwell_s,'required_dwell_s',8.0, ...
                        'heartbeat_received_at_s',h,'extended_received_at_s',e, ...
                        'current_conditions_expire_no_later_than_s',min([h+policy.max_heartbeat_age_s,e+policy.max_extended_age_s,timeS+policy.max_observation_gap_s]), ...
                        'requires_current_identity_freshness_dual_ground_recheck_at_consumption',true, ...
                        'payload_execution_performed',false,'persistent_permission',false);
                    reason='ONE_NEXT_PAYLOAD_GENERATION_PROOF_ISSUED';
                end
            end
            if ~isfinite(next.last_heartbeat_receive_s)||h>next.last_heartbeat_receive_s,next.last_heartbeat_receive_s=h;end
            if ~isfinite(next.last_extended_receive_s)||e>next.last_extended_receive_s,next.last_extended_receive_s=e;end
        end
    end
end
if ~good,next=resetDwell(next,reason);end
next.last_reason=reason;
proof=struct('status',reason,'current_native_ground_disarmed_evidence_valid',good, ...
    'continuous_dwell_s',next.dwell_s,'required_dwell_s',8.0,'token_issued_this_call',~isempty(token), ...
    'release_count',next.release_count,'last_released_generation',next.last_released_generation, ...
    'reset_count',next.reset_count,'plant_contact_is_not_px4_landed',true, ...
    'dwell_provenance','MU_CAMBRIDGE_MA_02_A1_CANONICAL_EIGHT_SECOND_GROUND_SERVICE__NOT_DV008', ...
    'mission_contract_sha256','E4B20DE53ACC4AA442ACD8262AE5639AEE1644505E12362C62520AE9F6CB6013', ...
    'lifecycle_contract_sha256','B1CCE6C7C11CC87B23D23B7FC683039BD442C8DB5598F45F75DDFC1295295B61', ...
    'board_evidence_transport_implemented',false,'payload_execution_performed',false, ...
    'arm_disarm_mode_request_count',0,'parameter_mapping_write_count',0,'hardware_actions',0);
end

function [yes,reason,binding]=eligible(o,t,p)
yes=false;reason='NATIVE_BOARD_EVIDENCE_UNKNOWN_OR_INVALID';binding=[];
required={'uid','session_id','boot_id','source_generation','clock_id','source_identity_valid','heartbeat','extended_sys_state'};
if ~isstruct(o)||~isscalar(o)||~all(isfield(o,required))||~trueLogical(o.source_identity_valid),return;end
if ~textScalar(o.uid)||~strcmp(char(o.uid),char(p.expected_uid))||~textScalar(o.session_id)|| ...
        ~textScalar(o.boot_id)||~textScalar(o.clock_id)||~strcmp(char(o.clock_id),char(p.clock_id))|| ...
        ~integer(o.source_generation,1,flintmax),reason='BOARD_BOOT_SESSION_CLOCK_IDENTITY_INVALID';return;end
binding=struct('uid',char(o.uid),'session_id',char(o.session_id),'boot_id',char(o.boot_id), ...
    'source_generation',double(o.source_generation),'clock_id',char(o.clock_id));
for name={'heartbeat','extended_sys_state'}
    key=name{1};x=o.(key);
    if ~isstruct(x)||~isscalar(x)||~all(isfield(x,[fieldnames(binding);{'received_at_s';'valid';'message_name'}]))|| ...
            ~trueLogical(x.valid),reason='INDEPENDENT_NATIVE_MESSAGE_MISSING_OR_INVALID';return;end
    for f=fieldnames(binding).'
        field=f{1};v=x.(field);
        if strcmp(field,'source_generation'),same=integer(v,1,flintmax)&&double(v)==binding.(field);
        else,same=textScalar(v)&&strcmp(char(v),binding.(field));end
        if ~same,reason='HEARTBEAT_EXTENDED_NOT_SAME_BOOT_SESSION_SOURCE';return;end
    end
    if ~textScalar(x.message_name),reason='NATIVE_MESSAGE_TYPE_INVALID';return;end
    if strcmp(key,'heartbeat'),expected='HEARTBEAT';ageBound=p.max_heartbeat_age_s;
    else,expected='EXTENDED_SYS_STATE';ageBound=p.max_extended_age_s;end
    if ~strcmp(char(x.message_name),expected),reason='PLANT_OR_HOST_FLAG_CANNOT_REPLACE_NATIVE_PX4_MESSAGE';return;end
    if ~finiteScalar(x.received_at_s)||x.received_at_s<0||x.received_at_s>t,reason='NATIVE_MESSAGE_TIME_INVALID_OR_FUTURE';return;end
    if t-x.received_at_s>ageBound,reason='NATIVE_HEARTBEAT_OR_LANDED_MESSAGE_STALE';return;end
end
h=o.heartbeat;e=o.extended_sys_state;
if ~isfield(h,'armed')||~knownBinary(h.armed),reason='ARM_STATE_UNKNOWN';return;end
if h.armed~=0,reason='PX4_ARMED_BREAKS_SERVICE_DWELL';return;end
if ~isfield(e,'landed_state')||~finiteScalar(e.landed_state)||e.landed_state~=1
    reason='PX4_NOT_FRESHLY_ON_GROUND';return
end
if p.require_plant_contact&&(~isfield(o,'plant_contact')||~trueLogical(o.plant_contact))
    reason='ADDITIONAL_PLANT_CONTACT_NOT_CONFIRMED';return
end
yes=true;reason='CURRENT_PAST_PX4_GROUND_DISARM_EVIDENCE_VALID';
end
function s=resetDwell(s,reason)
if isfinite(s.dwell_start_s)||s.dwell_s~=0,s.reset_count=s.reset_count+1;end
s.dwell_start_s=NaN;s.dwell_s=0;s.last_reason=reason;
end
function yes=validRequest(r)
yes=isstruct(r)&&isscalar(r)&&all(isfield(r,{'service_id','next_payload_generation','service_active'}))&& ...
    textScalar(r.service_id)&&integer(r.next_payload_generation,1,4)&&islogical(r.service_active)&&isscalar(r.service_active);
end
function validatePolicy(p)
keys={'service_dwell_s','max_heartbeat_age_s','max_extended_age_s','max_observation_gap_s','clock_id','expected_uid','require_plant_contact'};
assert(isstruct(p)&&isscalar(p)&&all(isfield(p,keys))&&numel(fieldnames(p))==numel(keys),'gpenmpcTaskIo:GroundProofPolicy');
assert(finiteScalar(p.service_dwell_s)&&p.service_dwell_s==8.0,'gpenmpcTaskIo:CanonicalServiceDwell','Canonical current A1 service is 8 s, not DV008 10 s.');
for f={'max_heartbeat_age_s','max_extended_age_s','max_observation_gap_s'}
    assert(finiteScalar(p.(f{1}))&&p.(f{1})>0,'gpenmpcTaskIo:GroundProofPolicy');
end
assert(textScalar(p.clock_id)&&textScalar(p.expected_uid)&& ...
    islogical(p.require_plant_contact)&&isscalar(p.require_plant_contact),'gpenmpcTaskIo:GroundProofPolicy');
end
function yes=finiteScalar(v),yes=isnumeric(v)&&isreal(v)&&isscalar(v)&&isfinite(v);end
function yes=integer(v,a,b),yes=finiteScalar(v)&&v==fix(v)&&v>=a&&v<=b;end
function yes=trueLogical(v),yes=islogical(v)&&isscalar(v)&&v;end
function yes=knownBinary(v),yes=(finiteScalar(v)||(islogical(v)&&isscalar(v)))&&ismember(v,[0,1]);end
function yes=textScalar(v),yes=((ischar(v)&&isrow(v))||(isstring(v)&&isscalar(v)&&~ismissing(v)))&&strlength(string(v))>0;end
