function [next,actions,event]=advanceCanonicalDeliveryCycle(previous,observation,timeS,request,policy)
%ADVANCECANONICALDELIVERYCYCLE Pure same-session grounded-delivery coordinator.
% Reuses gpenmpcTaskIo.advanceDeliveryGroundProof; it never owns an endpoint,
% modifies mass/plant state, sends a command, or grants independent flight.
% LAND/normal disarm remain the existing checked outer lifecycle's job.
% request_rearm is a one-shot handoff to that lifecycle, NOT a replacement for
% its command ACK/retry policy. This function also waits for a new armed HB.
%
% request is the ground-proof request: service_id,next_payload_generation,
% service_active. policy additionally supplies the prospective exact payload
% and total-mass identities (five values: initial then four deliveries), plant
% identity, and EXISTING timing policies. No mass/calibration is guessed here.
% observation.native is the existing ground-proof native-message structure;
% observation.plant is independent current contact/mass evidence. A payload
% ACK is insufficient unless that SAME plant's fresh measured payload/mass/
% generation agree. Observation acquisition/clock mappings remain external.
%
% Before issuance, interruptions discard dwell and fresh8 must start again.
% After issuance, any unsafe/session interruption latches this cycle failed:
% do not resend or infer whether an uncertain unload happened. Keep state and
% independently reconcile mass before any new outer recovery identity.
% Reference time stays paused throughout LAND/dwell/unload/rearm. On the one
% confirmed-rearm transition the caller receives a fresh PX4 P/V/A/yaw initial
% condition. No task-index jump, truth feedback or plant teleport is performed.
% observation.admission.rearm_admission_pass must be fresh external proof of
% the existing PX4 health/finite/attitude-safety checks, not a plant-contact
% flag. Missing/false evidence never produces a rearm handoff or ascent.
validatePolicy(policy);
if isempty(previous)
    next=struct('schema','CANONICAL_DELIVERY_CYCLE_V1','policy',policy,'phase','IDLE', ...
        'ground_proof',[],'last_time_s',NaN,'service_id','','generation',0, ...
        'completed_generation',0,'payload_generation',0,'payload_request',[], ...
        'unload_request_count',0,'accepted_mass_ack_count',0,'rearm_handoff_count',0, ...
        'rejected_ack_count',0,'cycle_binding',[],'mass_ack',[],'mass_ack_time_s',NaN, ...
        'rearm_ready_since_s',NaN,'rearm_handoff_time_s',NaN,'rearm_handoff_issued',false, ...
        'failure','','failure_at_s',NaN,'history',{{}},'last_ground_proof',[]);
else
    next=previous;
    assert(isstruct(next)&&isscalar(next)&&strcmp(next.schema,'CANONICAL_DELIVERY_CYCLE_V1')&& ...
        isequaln(next.policy,policy),'m600check:DeliveryCycleState','Preserve the complete prior state and prospective policy.');
end
actions=blankActions();event=struct('code','NO_ACTION','phase',next.phase,'ground_proof',[], ...
    'failure','','hardware_actions',0,'payload_execution_performed',false,'flight_admission',false);
if strcmp(next.phase,'FAIL_CLOSED'),finishEvent('FAILED_CYCLE_REQUIRES_INDEPENDENT_MASS_RECONCILIATION');return;end
if ~scalarFinite(timeS)||timeS<0||(isfinite(next.last_time_s)&&timeS<next.last_time_s)
    resetProof();fail('EXECUTION_CLOCK_INVALID_OR_REVERSED');return
end
if ~isempty(next.payload_request)&&~strcmp(next.phase,'COMPLETE')&&isfinite(next.last_time_s)&& ...
        timeS-next.last_time_s>policy.ground_proof_policy.max_observation_gap_s
    if hasAck(observation),next.rejected_ack_count=next.rejected_ack_count+1;end
    next.last_time_s=timeS;resetProof();fail('POST_REQUEST_UNOBSERVED_GAP');return
end
next.last_time_s=timeS;
if ~validRequest(request)
    resetProof();fail('SERVICE_REQUEST_INVALID');return
end
if ~request.service_active
    if strcmp(next.phase,'COMPLETE'),actions.reference_clock_paused=false;actions.advance_reference=true;actions.hold_reference=false;
    elseif ~strcmp(next.phase,'IDLE'),resetProof();fail('ACTIVE_SERVICE_WITHDRAWN');return;end
    finishEvent('NO_ACTIVE_SERVICE');return
end
g=double(request.next_payload_generation);
if ~strcmp(char(request.service_id),char(policy.service_ids{g}))
    resetProof();fail('SERVICE_NODE_ORDER_MISMATCH');return
end
if strcmp(next.phase,'COMPLETE')&&g==next.generation&&strcmp(next.service_id,char(request.service_id))
    actions.reference_clock_paused=false;actions.advance_reference=true;actions.hold_reference=false;
    if hasAck(observation),next.rejected_ack_count=next.rejected_ack_count+1;end
    finishEvent('CYCLE_ALREADY_COMPLETE_NO_SECOND_REQUEST');return
end
if any(strcmp(next.phase,{'IDLE','COMPLETE'}))
    if g~=next.completed_generation+1,resetProof();fail('PAYLOAD_GENERATION_NOT_EXACT_NEXT');return;end
    next.phase='WAIT_GROUND';next.service_id=char(request.service_id);next.generation=g;
    next.payload_request=[];next.mass_ack=[];next.mass_ack_time_s=NaN;
    next.rearm_ready_since_s=NaN;next.rearm_handoff_time_s=NaN;next.rearm_handoff_issued=false;
elseif g~=next.generation||~strcmp(next.service_id,char(request.service_id))
    resetProof();fail('SERVICE_CHANGED_BEFORE_COMPLETION');return
end
[nativeOkay,binding,nativeReason]=nativeCommon(field(observation,'native',[]),timeS,policy.ground_proof_policy);
[plantOkay,plantReason]=plantCommon(field(observation,'plant',[]),timeS,policy,binding,nativeOkay);
n=field(observation,'native',struct());
if ~isstruct(n)||~isscalar(n),n=struct();end
n.plant_contact=false;
if nativeOkay&&plantOkay,n.plant_contact=observation.plant.contact;
else,n.source_identity_valid=false;end
[next.ground_proof,proof,token]=gpenmpcTaskIo.advanceDeliveryGroundProof( ...
    next.ground_proof,n,timeS,request,policy.ground_proof_policy);
next.last_ground_proof=proof;event.ground_proof=proof;
if ~isempty(next.payload_request)
    if ~nativeOkay||~plantOkay||~isequal(binding,next.cycle_binding)
        if hasAck(observation),next.rejected_ack_count=next.rejected_ack_count+1;end
        resetProof();fail(['POST_REQUEST_OBSERVATION_INVALID:' nativeReason ':' plantReason]);return
    end
end
if any(strcmp(next.phase,{'WAIT_GROUND','DWELL'}))
    % Unsolicited/old ACK cannot issue or accelerate an unload request.
    if hasAck(observation),next.rejected_ack_count=next.rejected_ack_count+1;end
    if ~nativeOkay||~plantOkay||~proof.current_native_ground_disarmed_evidence_valid
        next.phase='WAIT_GROUND';finishEvent('FRESH_DUAL_GROUND_AND_DISARM_REQUIRED');return
    end
    if ~massMatches(observation.plant,next.payload_generation,policy)
        resetProof();fail('PRE_UNLOAD_ACTUAL_MASS_IDENTITY_MISMATCH');return
    end
    next.phase='DWELL';
    if isempty(token),finishEvent('CONTINUOUS_EIGHT_SECOND_DWELL');return;end
    assert(token.next_payload_generation==g&&token.continuous_dwell_s>=8, ...
        'm600check:DeliveryGroundProof','Unexpected proof generation or duration.');
    next.unload_request_count=next.unload_request_count+1;next.cycle_binding=binding;
    next.payload_request=struct('service_id',next.service_id,'generation',g, ...
        'request_ordinal',next.unload_request_count,'issued_at_s',timeS,'binding',binding, ...
        'plant_instance_id',char(policy.plant_identity.instance_id), ...
        'plant_model_sha256',upper(char(policy.plant_identity.model_sha256)), ...
        'payload_before_kg',policy.payload_by_generation_kg(g), ...
        'payload_after_kg',policy.payload_by_generation_kg(g+1), ...
        'mass_before_kg',policy.mass_by_generation_kg(g),'mass_after_kg',policy.mass_by_generation_kg(g+1), ...
        'ground_token',token,'requires_fresh_dual_ground_at_executor',true, ...
        'payload_execution_performed',false);
    next.phase='WAIT_MASS_ACK';actions.request_payload_update=true;actions.payload_request=next.payload_request;
    finishEvent('ONE_GROUNDED_UNLOAD_REQUEST');return
end
if strcmp(next.phase,'WAIT_MASS_ACK')
    if ~proof.current_native_ground_disarmed_evidence_valid
        if hasAck(observation),next.rejected_ack_count=next.rejected_ack_count+1;end
        resetProof();fail('DUAL_GROUND_OR_DISARM_LOST_AFTER_UNLOAD_REQUEST');return
    end
    if timeS-next.payload_request.issued_at_s>=policy.payload_ack_timeout_s
        if hasAck(observation),next.rejected_ack_count=next.rejected_ack_count+1;end
        fail('ACTUAL_MASS_ACK_TIMEOUT');return
    end
    if ~(massMatches(observation.plant,g-1,policy)||massMatches(observation.plant,g,policy))
        fail('UNEXPECTED_POST_REQUEST_MASS_OR_GENERATION');return
    end
    if ~hasAck(observation),finishEvent('WAITING_FOR_ACTUAL_MASS_ACK_NO_REARM');return;end
    if ~validAck(observation.payload_ack,observation.plant,next.payload_request,timeS,policy)
        next.rejected_ack_count=next.rejected_ack_count+1;finishEvent('REJECTED_STALE_OR_UNCONFIRMED_MASS_ACK');return
    end
    next.mass_ack=observation.payload_ack;next.mass_ack_time_s=timeS;
    next.payload_generation=g;next.accepted_mass_ack_count=next.accepted_mass_ack_count+1;
    next.phase='WAIT_REARM';finishEvent('ACTUAL_MASS_CHANGE_CONFIRMED_REFERENCE_STILL_PAUSED');return
end
if strcmp(next.phase,'WAIT_REARM')
    if hasAck(observation),next.rejected_ack_count=next.rejected_ack_count+1;end
    if ~massMatches(observation.plant,g,policy),fail('CONFIRMED_MASS_IDENTITY_LOST');return;end
    if timeS-next.mass_ack_time_s>=policy.rearm_timeout_s,fail('REARM_CONFIRMATION_TIMEOUT');return;end
    armed=double(observation.native.heartbeat.armed);
    if armed==1
        if ~next.rearm_handoff_issued||observation.native.heartbeat.received_at_s<=next.rearm_handoff_time_s
            resetProof();fail('UNEXPECTED_OR_OLD_ARMED_STATE');return
        end
        if ~freshAdmission(observation,timeS,policy,false)
            fail('OFFBOARD_OR_FRESH_ESTIMATE_LOST_AT_REARM');return
        end
        next.completed_generation=g;next.phase='COMPLETE';
        next.history{end+1}=struct('service_id',next.service_id,'generation',g, ...
            'unload_request',next.payload_request,'mass_ack',next.mass_ack, ...
            'rearm_handoff_time_s',next.rearm_handoff_time_s,'armed_confirmed_at_s',timeS);
        actions.reference_clock_paused=false;actions.advance_reference=true;actions.hold_reference=false;
        actions.start_reference_from_estimate=observation.estimate;
        actions.initial_reference_source='FRESH_SAME_BOOT_PX4_ESTIMATE__NO_PLANT_STATE_RESET';
        finishEvent('REARM_CONFIRMED_START_STATE_MATCHED_ASCENT');return
    end
    if ~proof.current_native_ground_disarmed_evidence_valid
        resetProof();fail('GROUND_SAFETY_LOST_BEFORE_REARM');return
    end
    if freshAdmission(observation,timeS,policy,true)
        if ~isfinite(next.rearm_ready_since_s),next.rearm_ready_since_s=timeS;end
        if ~next.rearm_handoff_issued&&timeS-next.rearm_ready_since_s>=policy.rearm_dwell_s
            next.rearm_handoff_issued=true;next.rearm_handoff_time_s=timeS;
            next.rearm_handoff_count=next.rearm_handoff_count+1;
            actions.request_rearm=true;actions.rearm_initial_reference=observation.estimate;
            finishEvent('MASS_ACK_AND_GROUND_VERIFIED_REARM_HANDOFF');return
        end
    elseif next.rearm_handoff_issued
        fail('REARM_ADMISSION_LOST_AFTER_HANDOFF');return
    else
        next.rearm_ready_since_s=NaN;
    end
    finishEvent('WAITING_FOR_FRESH_REARM_ADMISSION_REFERENCE_PAUSED');return
end
fail('UNKNOWN_DELIVERY_PHASE');

    function resetProof()
        [next.ground_proof,p,~]=gpenmpcTaskIo.advanceDeliveryGroundProof( ...
            next.ground_proof,struct(),timeS,request,policy.ground_proof_policy);
        next.last_ground_proof=p;event.ground_proof=p;
    end
    function fail(reason)
        if isempty(next.failure),next.failure=reason;next.failure_at_s=timeS;end
        next.phase='FAIL_CLOSED';actions=blankActions();finishEvent('FAIL_CLOSED_NO_REARM');
    end
    function finishEvent(code)
        event.code=code;event.phase=next.phase;event.failure=next.failure;
    end
end

function a=blankActions()
a=struct('request_payload_update',false,'payload_request',[],'request_rearm',false, ...
    'rearm_initial_reference',[],'start_reference_from_estimate',[],'initial_reference_source','', ...
    'reference_clock_paused',true,'advance_reference',false,'hold_reference',true, ...
    'outer_native_land_and_standard_disarm_required',true,'plant_reset',false, ...
    'parameter_mapping_write_count',0,'hardware_actions',0,'flight_admission',false);
end
function [yes,b,reason]=nativeCommon(n,t,p)
yes=false;b=[];reason='NATIVE_EVIDENCE_INVALID';
keys={'uid','session_id','boot_id','source_generation','clock_id','source_identity_valid','heartbeat','extended_sys_state'};
if ~isstruct(n)||~isscalar(n)||~all(isfield(n,keys))||~isTrue(n.source_identity_valid),return;end
if ~text(n.uid)||~strcmp(char(n.uid),char(p.expected_uid))||~text(n.session_id)||~text(n.boot_id)|| ...
        ~text(n.clock_id)||~strcmp(char(n.clock_id),char(p.clock_id))||~integer(n.source_generation,1,flintmax),return;end
b=struct('uid',char(n.uid),'session_id',char(n.session_id),'boot_id',char(n.boot_id), ...
    'source_generation',double(n.source_generation),'clock_id',char(n.clock_id));
for k=1:2
    if k==1,m=n.heartbeat;kind='HEARTBEAT';age=p.max_heartbeat_age_s;else,m=n.extended_sys_state;kind='EXTENDED_SYS_STATE';age=p.max_extended_age_s;end
    if ~isstruct(m)||~isscalar(m)||~all(isfield(m,[fieldnames(b);{'valid';'received_at_s';'message_name'}]))|| ...
            ~isTrue(m.valid)||~text(m.message_name)||~strcmp(char(m.message_name),kind)|| ...
            ~scalarFinite(m.received_at_s)||m.received_at_s<0||m.received_at_s>t||t-m.received_at_s>age,return;end
    for f=fieldnames(b).'
        key=f{1};v=m.(key);
        if strcmp(key,'source_generation'),same=integer(v,1,flintmax)&&double(v)==b.(key);
        else,same=text(v)&&strcmp(char(v),b.(key));end
        if ~same,return;end
    end
end
if ~isfield(n.heartbeat,'armed')||~binary(n.heartbeat.armed)|| ...
        ~isfield(n.extended_sys_state,'landed_state')||~integer(n.extended_sys_state.landed_state,1,4),return;end
yes=true;reason='FRESH_SAME_BOOT_NATIVE_MESSAGES';
end
function [yes,reason]=plantCommon(x,t,p,b,nativeOkay)
yes=false;reason='PLANT_EVIDENCE_INVALID';
keys={'instance_id','model_sha256','session_id','clock_id','valid','received_at_s','mapped_source_time_s', ...
    'contact','payload_generation','payload_kg','mass_kg'};
if ~nativeOkay||~isstruct(x)||~isscalar(x)||~all(isfield(x,keys))||~isTrue(x.valid),return;end
if ~text(x.instance_id)||~strcmp(char(x.instance_id),char(p.plant_identity.instance_id))|| ...
        ~text(x.model_sha256)||~strcmpi(char(x.model_sha256),char(p.plant_identity.model_sha256))|| ...
        ~text(x.session_id)||~strcmp(char(x.session_id),b.session_id)|| ...
        ~text(x.clock_id)||~strcmp(char(x.clock_id),b.clock_id)||~islogical(x.contact)||~isscalar(x.contact)|| ...
        ~freshTime(x.received_at_s,t,p.max_plant_age_s)||~freshTime(x.mapped_source_time_s,t,p.max_plant_age_s)|| ...
        ~integer(x.payload_generation,0,4)||~scalarFinite(x.payload_kg)||x.payload_kg<0|| ...
        ~scalarFinite(x.mass_kg)||x.mass_kg<=0,return;end
yes=true;reason='SAME_PLANT_FRESH_CONTACT_MASS_EVIDENCE';
end
function yes=massMatches(x,g,p)
yes=x.payload_generation==g&&x.payload_kg==p.payload_by_generation_kg(g+1)&&x.mass_kg==p.mass_by_generation_kg(g+1);
end
function yes=validAck(a,x,r,t,p)
keys={'service_id','generation','request_ordinal','request_issued_at_s','binding', ...
    'plant_instance_id','plant_model_sha256','accepted','received_at_s','measurement_time_s','payload_kg','mass_kg'};
yes=isstruct(a)&&isscalar(a)&&all(isfield(a,keys));if ~yes,return;end
yes=text(a.service_id)&&strcmp(char(a.service_id),r.service_id)&&isequal(a.generation,r.generation)&& ...
    isequal(a.request_ordinal,r.request_ordinal)&&isequal(a.request_issued_at_s,r.issued_at_s)&& ...
    isequal(a.binding,r.binding)&&text(a.plant_instance_id)&&strcmp(char(a.plant_instance_id),r.plant_instance_id)&& ...
    text(a.plant_model_sha256)&&strcmpi(char(a.plant_model_sha256),r.plant_model_sha256)&&isTrue(a.accepted)&& ...
    freshTime(a.received_at_s,t,p.max_plant_age_s)&&freshTime(a.measurement_time_s,t,p.max_plant_age_s)&& ...
    a.received_at_s>=r.issued_at_s&&a.measurement_time_s>=r.issued_at_s&& ...
    scalarFinite(a.payload_kg)&&a.payload_kg==r.payload_after_kg&&scalarFinite(a.mass_kg)&&a.mass_kg==r.mass_after_kg&& ...
    massMatches(x,r.generation,p)&&x.mapped_source_time_s>=r.issued_at_s;
end
function yes=freshAdmission(o,t,p,requirePrearm)
yes=false;if ~isstruct(o)||~isscalar(o)||~all(isfield(o,{'admission','estimate','native'})),return;end
a=o.admission;x=o.estimate;
if ~isstruct(a)||~isscalar(a)||~all(isfield(a,{'offboard_ready','prearm_ready','rearm_admission_pass','valid','binding','received_at_s'}))|| ...
        ~isTrue(a.offboard_ready)||~isTrue(a.rearm_admission_pass)||~isTrue(a.valid)|| ...
        ~freshTime(a.received_at_s,t,p.max_estimate_age_s),return;end
if requirePrearm&&~isTrue(a.prearm_ready),return;end
keys={'binding','received_at_s','mapped_source_time_s','position_ned_m','velocity_ned_mps','acceleration_ned_mps2','yaw_ned_rad','valid'};
if ~isstruct(x)||~isscalar(x)||~all(isfield(x,keys))||~isTrue(x.valid)|| ...
        ~freshTime(x.received_at_s,t,p.max_estimate_age_s)||~freshTime(x.mapped_source_time_s,t,p.max_estimate_age_s),return;end
[n,b,~]=nativeCommon(o.native,t,p.ground_proof_policy);if ~n||~isequal(x.binding,b)||~isequal(a.binding,b),return;end
for name={'position_ned_m','velocity_ned_mps','acceleration_ned_mps2'}
    v=x.(name{1});if ~isnumeric(v)||~isreal(v)||numel(v)~=3||any(~isfinite(v(:))),return;end
end
yes=scalarFinite(x.yaw_ned_rad);
end
function validatePolicy(p)
keys={'ground_proof_policy','service_ids','payload_by_generation_kg','mass_by_generation_kg','plant_identity', ...
    'max_plant_age_s','max_estimate_age_s','payload_ack_timeout_s','rearm_timeout_s','rearm_dwell_s'};
assert(isstruct(p)&&isscalar(p)&&all(isfield(p,keys)),'m600check:DeliveryCyclePolicy','Explicit complete policy required.');
assert(iscell(p.service_ids)&&numel(p.service_ids)==4&&all(cellfun(@text,p.service_ids))&& ...
    numel(unique(string(p.service_ids)))==4,'m600check:DeliveryCyclePolicy','Four ordered unique service IDs required.');
for name={'payload_by_generation_kg','mass_by_generation_kg'}
    v=p.(name{1});assert(isnumeric(v)&&isreal(v)&&numel(v)==5&&all(isfinite(v(:)))&& ...
        all(diff(v(:))<0),'m600check:DeliveryCyclePolicy','Prospective five exact decreasing mass/payload identities required.');
end
assert(all(p.payload_by_generation_kg>=0)&&p.payload_by_generation_kg(end)==0&&all(p.mass_by_generation_kg>0), ...
    'm600check:DeliveryCyclePolicy','No negative payload or nonpositive mass.');
assert(isstruct(p.plant_identity)&&isscalar(p.plant_identity)&&all(isfield(p.plant_identity,{'instance_id','model_sha256'}))&& ...
    text(p.plant_identity.instance_id)&&text(p.plant_identity.model_sha256)&& ...
    ~isempty(regexp(char(p.plant_identity.model_sha256),'^[0-9A-Fa-f]{64}$','once')), ...
    'm600check:DeliveryCyclePolicy','Explicit exact plant identity required.');
for name={'max_plant_age_s','max_estimate_age_s','payload_ack_timeout_s','rearm_timeout_s','rearm_dwell_s'}
    assert(scalarFinite(p.(name{1}))&&p.(name{1})>0,'m600check:DeliveryCyclePolicy','Existing positive caller timing policy required.');
end
gp=p.ground_proof_policy;
assert(isstruct(gp)&&isscalar(gp)&&isfield(gp,'service_dwell_s')&&gp.service_dwell_s==8&& ...
    isfield(gp,'require_plant_contact')&&isTrue(gp.require_plant_contact), ...
    'm600check:DeliveryCyclePolicy','Canonical eight-second dual-ground proof is mandatory.');
end
function yes=validRequest(r)
yes=isstruct(r)&&isscalar(r)&&all(isfield(r,{'service_id','next_payload_generation','service_active'}))&& ...
    text(r.service_id)&&integer(r.next_payload_generation,1,4)&&islogical(r.service_active)&&isscalar(r.service_active);
end
function yes=hasAck(o),yes=isstruct(o)&&isscalar(o)&&isfield(o,'payload_ack')&&~isempty(o.payload_ack);end
function v=field(s,k,fallback),v=fallback;if isstruct(s)&&isscalar(s)&&isfield(s,k),v=s.(k);end,end
function yes=freshTime(v,t,age),yes=scalarFinite(v)&&v>=0&&v<=t&&t-v<=age;end
function yes=scalarFinite(v),yes=isnumeric(v)&&isreal(v)&&isscalar(v)&&isfinite(v);end
function yes=integer(v,a,b),yes=scalarFinite(v)&&v==fix(v)&&v>=a&&v<=b;end
function yes=binary(v),yes=(scalarFinite(v)||(islogical(v)&&isscalar(v)))&&ismember(v,[0,1]);end
function yes=isTrue(v),yes=islogical(v)&&isscalar(v)&&v;end
function yes=text(v),yes=((ischar(v)&&isrow(v))||(isstring(v)&&isscalar(v)&&~ismissing(v)))&&strlength(string(v))>0;end
