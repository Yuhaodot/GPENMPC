function [next,result]=observeCopterSimDeliveryDiagnostics(previous,datagram,expectedCopterId,policy)
% Pure stateful wrapper. Retain NEXT across all packets in one observer run.
% [] is ONLY for an explicitly new observer, never recovery from a fault.
% This function neither authenticates core commits nor grants flight authority.
% Invalid caller policy/state is an API error; caller must keep its old state.
if isempty(previous)
 result=m600check.decodeCopterSimDeliveryDiagnostics(datagram,expectedCopterId,NaN,policy);
 next=initial(expectedCopterId,policy);
else
 validateState(previous);
 next=previous;
 % Original CopterID and source-clock maximum are not reset by later inputs.
 result=m600check.decodeCopterSimDeliveryDiagnostics(datagram,next.expected_copter_id,next.last_source_time_s,policy);
end
next.packet_count=next.packet_count+1;
reasons=cell(0,1);e=result.environment_extension;
if expectedCopterId~=next.expected_copter_id
 reasons{end+1,1}='COPTER_ID_BINDING_CHANGED';
end
if ~samePolicy(policy,next.policy)
 reasons{end+1,1}='POLICY_BINDING_CHANGED';
end
if ~result.packet_valid
 reasons{end+1,1}=['PACKET_REJECTED:' result.status];
elseif result.must_stop
 reasons{end+1,1}=['MODEL_OR_CLOCK_STOP:' result.status];
end
if e.valid
 if e.task_env_failed
  reasons{end+1,1}=sprintf('ENVIRONMENT_FAILURE_%02d',e.status_code);
 end
 if ~e.session_bound
  if next.session_bound||next.task_fault_latched
   reasons{end+1,1}='UNBOUND_PRESESSION_RETURN_AFTER_BINDING_OR_FAULT';
  end
 elseif next.session_bound&&e.session_token~=next.session_token
  reasons{end+1,1}='SESSION_TOKEN_CHANGED';
 end
 if e.session_bound&&next.identity_recorded
  if e.applied_frame_generation<next.last_frame_generation
   reasons{end+1,1}='FRAME_GENERATION_REVERSED';
  end
  if e.applied_payload_generation<next.last_payload_generation
   reasons{end+1,1}='PAYLOAD_GENERATION_REVERSED';
  end
  if e.applied_frame_generation==next.last_frame_generation&& ...
    (e.applied_payload_generation~=next.last_payload_generation|| ...
     ~sameBits(e.actual_payload_kg,next.last_payload_kg)||~sameBits(e.actual_total_mass_kg,next.last_total_mass_kg))
   reasons{end+1,1}='CONFLICTING_FRAME_GENERATION_REPLAY';
  end
  if e.applied_payload_generation==next.last_payload_generation&& ...
    (~sameBits(e.actual_payload_kg,next.last_payload_kg)||~sameBits(e.actual_total_mass_kg,next.last_total_mass_kg))
   reasons{end+1,1}='MASS_CHANGED_WITHIN_PAYLOAD_GENERATION';
  end
 end
end
sequenceValid=isempty(reasons);
if ~sequenceValid
 if ~next.task_fault_latched
  next.first_fault=struct('packet_index',next.packet_count,'reason',reasons{1}, ...
   'decoder_status',result.status,'model_time_s',result.sim_time_s, ...
   'environment_status_code',e.status_code,'session_token',e.session_token, ...
   'frame_generation',e.applied_frame_generation,'payload_generation',e.applied_payload_generation, ...
   'model_failed',result.model_failed,'model_failure_code',result.failure_code);
 end
 next.task_fault_latched=true;
end
% Binding becomes sticky even when the first correctly identified packet
% reports a task-environment or model fault. Later traffic cannot unbind it.
if e.valid&&e.session_bound&&samePolicy(policy,next.policy)&&expectedCopterId==next.expected_copter_id
 next.session_bound=true;next.session_token=e.session_token;
end
if result.packet_valid&&isfinite(result.sim_time_s)
 if isnan(next.last_source_time_s),next.last_source_time_s=result.sim_time_s;
 else,next.last_source_time_s=max(next.last_source_time_s,result.sim_time_s);end
end
% Do not promote rejected/out-of-order identities into the accepted ledger.
% A later good packet may remain useful MODEL evidence, but cannot clear fault.
if sequenceValid&&~next.task_fault_latched&&e.valid&&e.session_bound
 next.identity_recorded=true;next.last_frame_generation=e.applied_frame_generation;
 next.last_payload_generation=e.applied_payload_generation;
 next.last_payload_kg=e.actual_payload_kg;next.last_total_mass_kg=e.actual_total_mass_kg;
end
if e.valid&&~e.session_bound,next.unbound_packet_count=next.unbound_packet_count+1;end
next.last_reasons=reasons;
taskAllowed=sequenceValid&&~next.task_fault_latched&&e.can_continue_task;
ackAllowed=sequenceValid&&~next.task_fault_latched&&e.mass_ack_valid;
% Preserve packet validity, terrain/raw values, model observation permission
% and decoder must_stop. An ENV fault is not a command to stop the HIL plant.
result.environment_extension.can_continue_task=taskAllowed;
result.environment_extension.mass_ack_valid=ackAllowed;
result.stateful_delivery=struct('schema','COPTERSIM_DELIVERY_OBSERVER_V1', ...
 'packet_index',next.packet_count,'sequence_valid',sequenceValid,'current_reasons',{reasons}, ...
 'task_fault_latched',next.task_fault_latched,'first_fault',next.first_fault, ...
 'session_bound',next.session_bound,'session_token',next.session_token, ...
 'accepted_frame_generation',next.last_frame_generation,'accepted_payload_generation',next.last_payload_generation, ...
 'source_time_max_s',next.last_source_time_s,'can_continue_task',taskAllowed,'mass_ack_valid',ackAllowed, ...
 'must_stop_task',next.task_fault_latched,'model_stop_required_by_decoder',result.must_stop, ...
 'keep_same_plant_evolving',true,'request_plant_reset',false,'flight_admission',false, ...
 'task_completion_proven',false,'wall_freshness_evaluated',false);
end
function s=initial(id,p)
s=struct('schema','COPTERSIM_DELIVERY_OBSERVER_STATE_V1','expected_copter_id',id,'policy',canonicalPolicy(p), ...
 'packet_count',0,'last_source_time_s',NaN,'session_bound',false,'session_token',0, ...
 'identity_recorded',false,'last_frame_generation',0,'last_payload_generation',0, ...
 'last_payload_kg',NaN,'last_total_mass_kg',NaN,'unbound_packet_count',0,'task_fault_latched',false, ...
 'first_fault',struct(),'last_reasons',{{}});
end
function p=canonicalPolicy(p)
p.payload_by_generation_kg=p.payload_by_generation_kg(:).';
p.mass_by_generation_kg=p.mass_by_generation_kg(:).';
if ~isfield(p,'allow_unbound_pre_session'),p.allow_unbound_pre_session=false;end
p=orderfields(p);
end
function yes=samePolicy(a,b)
a=canonicalPolicy(a);b=canonicalPolicy(b);yes=isequaln(a,b);
if yes
 yes=sameBits(double(a.payload_by_generation_kg),double(b.payload_by_generation_kg))&& ...
  sameBits(double(a.mass_by_generation_kg),double(b.mass_by_generation_kg));
end
end
function yes=sameBits(a,b),yes=isequal(typecast(double(a(:)),'uint64'),typecast(double(b(:)),'uint64'));end
function validateState(s)
required={'schema','expected_copter_id','policy','packet_count','last_source_time_s','session_bound', ...
 'session_token','identity_recorded','last_frame_generation','last_payload_generation','last_payload_kg', ...
 'last_total_mass_kg','unbound_packet_count','task_fault_latched','first_fault','last_reasons'};
assert(isstruct(s)&&isscalar(s)&&all(isfield(s,required))&&strcmp(s.schema,'COPTERSIM_DELIVERY_OBSERVER_STATE_V1'), ...
 'm600check:DeliveryObserverState','Retain the complete same-run observer state.');
assert(islogical(s.session_bound)&&isscalar(s.session_bound)&&islogical(s.identity_recorded)&&isscalar(s.identity_recorded)&& ...
 islogical(s.task_fault_latched)&&isscalar(s.task_fault_latched)&&isstruct(s.first_fault)&&isscalar(s.first_fault), ...
 'm600check:DeliveryObserverState','Invalid observer latch fields.');
assert(isnumeric(s.packet_count)&&isscalar(s.packet_count)&&isfinite(s.packet_count)&&s.packet_count>=0&& ...
 s.packet_count==fix(s.packet_count)&&isscalar(s.last_source_time_s)&& ...
 (isnan(s.last_source_time_s)||(isfinite(s.last_source_time_s)&&s.last_source_time_s>=0)), ...
 'm600check:DeliveryObserverState','Invalid source-clock or packet ledger.');
assert(~s.task_fault_latched||all(isfield(s.first_fault,{'packet_index','reason','decoder_status'})), ...
 'm600check:DeliveryObserverState','A latched fault must retain its first evidence.');
end
