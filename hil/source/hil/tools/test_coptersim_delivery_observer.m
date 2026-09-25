function report=test_coptersim_delivery_observer(outputDir)
% Test delivery observation across multiple packets.
arguments,outputDir (1,1) string = "",end
build=string(fileparts(fileparts(mfilename('fullpath'))));oldPath=path;
pathGuard=onCleanup(@()path(oldPath)); %#ok<NASGU>
addpath(fullfile(build,'matlab_validation'),'-begin');
addpath(fullfile(build,'m600_coptersim','matlab_validation'),'-end');
if strlength(outputDir)>0
 assert(~isfolder(outputDir)&&~isfile(outputDir),'m600check:OutputExists','Preserve earlier evidence.');
end
checks=struct('name',{},'passed',{});callCount=0;
p=struct('expected_session_token',1234567,'payload_by_generation_kg',[2.21 1.75 .98 .55 0], ...
 'mass_by_generation_kg',9.5+[2.21 1.75 .98 .55 0],'allow_unbound_pre_session',true);
init=make(1,0,0,0,16);bound=make(2,1234567,1,0,0);
[s,r]=advance([],init,p);check('unbound_initial_valid_model_not_task',r.packet_valid&&r.can_use_as_healthy_observation&& ...
 ~s.session_bound&&~s.task_fault_latched&&~task(r)&&~r.environment_extension.mass_ack_valid);
[s,r]=advance(s,make(1.1,0,0,0,16),p);
check('unbound_repeated_initialization_no_implicit_fault',~s.task_fault_latched&&s.unbound_packet_count==2&&~task(r));
[s,r]=advance(s,bound,p);
check('unbound_to_bound_committed_positive',s.session_bound&&s.session_token==1234567&&task(r)&& ...
 r.environment_extension.mass_ack_valid&&s.last_frame_generation==1);
boundState=s;
[s,r]=advance(s,make(2.1,0,0,0,16),p);
check('bound_to_unbound_rejected_task_model_still_truthful',s.task_fault_latched&&~task(r)&& ...
 ~r.environment_extension.mass_ack_valid&&r.packet_valid&&r.can_use_as_healthy_observation&&~r.must_stop&& ...
 strcmp(s.first_fault.reason,'UNBOUND_PRESESSION_RETURN_AFTER_BINDING_OR_FAULT'));
first=s.first_fault;
[s,r]=advance(s,make(2.2,1234567,2,0,0),p);
check('healthy_after_unbound_failure_cannot_restore_task',s.task_fault_latched&&isequaln(first,s.first_fault)&&~task(r)&& ...
 ~r.environment_extension.mass_ack_valid&&s.last_frame_generation==1);
% Valid repeats, pending with same applied identity, then committed next step.
[s,r]=advance([],bound,p);[s,r]=advance(s,bound,p);
check('identical_packet_repeat_valid_not_new_generation',task(r)&&~s.task_fault_latched&&s.last_frame_generation==1);
[s,r]=advance(s,make(2.1,1234567,1,0,17),p);
check('pending_same_generation_not_fault_or_mass_ack',~s.task_fault_latched&&r.environment_extension.pending&&~task(r)&& ...
 ~r.environment_extension.mass_ack_valid);
[s,r]=advance(s,make(2.2,1234567,2,0,0),p);
check('pending_to_later_commit_valid',task(r)&&s.last_frame_generation==2);
for generation=1:4
 [s,r]=advance(s,make(3+generation,1234567,2+generation,generation,0),p);
 check(sprintf('payload_generation_%d_advances_exact_mass',generation),task(r)&& ...
  s.last_payload_generation==generation&&s.last_payload_kg==p.payload_by_generation_kg(generation+1));
end
% Every explicit environment failure latches TASK failure without inventing a
% physical/HIL stop. Subsequent healthy and initialization cannot erase it.
for code=1:15
 [q,~]=advance([],bound,p);[q,r]=advance(q,make(2.1,1234567,2,0,code),p);f=q.first_fault;
 check(sprintf('env_%02d_latches_task_only',code),q.task_fault_latched&&~task(r)&&~r.must_stop&& ...
  r.can_use_as_healthy_observation&&r.environment_extension.task_env_failed&& ...
  r.stateful_delivery.keep_same_plant_evolving&&~r.stateful_delivery.request_plant_reset);
 [q,r]=advance(q,make(2.2,1234567,3,0,0),p);
 check(sprintf('env_%02d_healthy_does_not_clear_first_fault',code),~task(r)&&isequaln(q.first_fault,f)&& ...
  ~r.environment_extension.mass_ack_valid&&r.can_use_as_healthy_observation);
 [q,r]=advance(q,make(2.3,0,0,0,16),p);
 check(sprintf('env_%02d_unbound_does_not_clear_binding_or_fault',code),~task(r)&&q.session_bound&& ...
  q.session_token==1234567&&isequaln(q.first_fault,f)&&~r.stateful_delivery.sequence_valid);
end
% First malformed traffic, before binding, also prohibits later pre-session use.
bad=init;bad(26)=3;[s,r]=advance([],bad,p);f=s.first_fault;
check('prebinding_malformed_packet_fault',s.task_fault_latched&&~task(r)&&r.must_stop);
[s,r]=advance(s,make(2,0,0,0,16),p);
check('fault_before_binding_cannot_return_unbound',~task(r)&&~r.stateful_delivery.sequence_valid&&isequaln(f,s.first_fault));
[s,r]=advance(s,make(3,1234567,1,0,0),p);
check('fault_before_binding_later_bind_cannot_enable_task',s.session_bound&&s.task_fault_latched&&~task(r));
% Frame and payload order are independent: later frame cannot wash payload rollback.
sequenceCases={ ...
 'frame_regression',make(3,1234567,3,1,0),make(4,1234567,2,1,0),'FRAME_GENERATION_REVERSED'; ...
 'payload_regression',make(3,1234567,3,1,0),make(4,1234567,4,0,0),'PAYLOAD_GENERATION_REVERSED'; ...
 'same_frame_new_payload',make(3,1234567,3,0,0),make(4,1234567,3,1,0),'CONFLICTING_FRAME_GENERATION_REPLAY'};
for index=1:size(sequenceCases,1)
 [q,~]=advance([],sequenceCases{index,2},p);last=q;
 [q,r]=advance(q,sequenceCases{index,3},p);
 check(sequenceCases{index,1},q.task_fault_latched&&~task(r)&&strcmp(q.first_fault.reason,sequenceCases{index,4})&& ...
  q.last_frame_generation==last.last_frame_generation&&q.last_payload_generation==last.last_payload_generation&& ...
  r.can_use_as_healthy_observation&&~r.must_stop);
end
% The codec rejects wrong masses before sequence promotion.
for field=[30 31]
 [q,~]=advance([],bound,p);bad=make(3,1234567,1,0,0);bad(field)=bad(field)+eps(bad(field));
 [q,r]=advance(q,bad,p);
 check(sprintf('same_generation_mass_field_%d_one_ulp_conflict',field),q.task_fault_latched&&~task(r)&& ...
  contains(q.first_fault.reason,'DELIVERY_ACTUAL_MASS_IDENTITY_MISMATCH')&&q.last_frame_generation==1);
end
% +0 and -0 have equal numeric value but conflicting original mass bits.
last=make(3,1234567,5,4,0);[q,~]=advance([],last,p);
bad=make(4,1234567,6,4,0);bad(30)=typecast(bitshift(uint64(1),63),'double');[q,r]=advance(q,bad,p);
check('same_payload_generation_signed_zero_change_rejected',q.task_fault_latched&&~task(r)&& ...
 strcmp(q.first_fault.reason,'MASS_CHANGED_WITHIN_PAYLOAD_GENERATION')&&r.packet_valid);
% Session and policy may not be replaced to make an otherwise wrong packet fit.
[q,~]=advance([],bound,p);[q,r]=advance(q,make(3,1234568,2,0,0),p);
check('wrong_session_packet_rejected_without_rebinding',q.task_fault_latched&&~task(r)&&q.session_token==1234567);
changed=p;changed.expected_session_token=1234568;
[q,~]=advance([],bound,p);[q,r]=advance(q,make(3,1234568,2,0,0),changed);
check('changed_policy_cannot_rebind_session',q.task_fault_latched&&~task(r)&&q.session_token==1234567&& ...
 strcmp(q.first_fault.reason,'POLICY_BINDING_CHANGED'));
changed=p;changed.mass_by_generation_kg=changed.mass_by_generation_kg+1;bad=bound;bad(31)=bad(31)+1;
[q,~]=advance([],bound,p);[q,r]=advance(q,bad,changed);
check('changed_expected_mass_table_cannot_redefine_identity',q.task_fault_latched&&~task(r)&&strcmp(q.first_fault.reason,'POLICY_BINDING_CHANGED'));
changed=p;changed.allow_unbound_pre_session=false;[q,~]=advance([],bound,p);[q,r]=advance(q,make(3,1234567,2,0,0),changed);
check('changed_unbound_policy_is_explicit_context_failure',q.task_fault_latched&&~task(r)&&strcmp(q.first_fault.reason,'POLICY_BINDING_CHANGED'));
changed=p;changed.payload_by_generation_kg=changed.payload_by_generation_kg.';changed.mass_by_generation_kg=changed.mass_by_generation_kg.';
[q,~]=advance([],bound,p);[q,r]=advance(q,make(3,1234567,2,0,0),changed);
check('json_row_column_vector_shape_not_identity_change',task(r)&&~q.task_fault_latched);
[q,~]=advance([],bound,p);callCount=callCount+1;
[q,r]=m600check.observeCopterSimDeliveryDiagnostics(q,packet(make(3,1234567,2,0,0)),2,p);
check('caller_copter_id_change_never_redefines_saved_identity',q.task_fault_latched&&~task(r)&&q.expected_copter_id==1&& ...
 strcmp(q.first_fault.reason,'COPTER_ID_BINDING_CHANGED'));
% Reject source-clock reversal without resetting the observer origin.
[q,~]=advance([],make(10,1234567,1,0,0),p);[q,r]=advance(q,make(9,1234567,2,0,0),p);f=q.first_fault;
check('source_reversal_latched_and_max_clock_retained',q.task_fault_latched&&q.last_source_time_s==10&& ...
 r.must_stop&&~task(r)&&contains(f.reason,'SIMULATION_TIME_REVERSED'));
[q,r]=advance(q,make(11,1234567,3,0,0),p);
check('later_source_clock_does_not_wash_reversal',~task(r)&&isequaln(f,q.first_fault)&&q.last_source_time_s==11);
% Real terrain failure raw bits survive the wrapper and environment failures.
raw=zeros(15,1);raw(1)=typecast(bits('FFF800000000ABCD'),'double');raw(15)=Inf;
prefix=[1;4;4;114.8;1;0;1];st=m600check.initialCopterSimTerrainDiagnosticState();
[terrainPacket,~]=m600check.encodeCopterSimTerrainDiagnostics(prefix,raw,2,0,true,false,st);
terrainPacket(26)=2;terrainPacket(27:32)=[1234567;2;0;2.21;11.71;7];
[q,~]=advance([],bound,p);[q,r]=advance(q,terrainPacket,p);f=q.first_fault;
check('terrain_first_fault_not_masked_by_environment_fault',q.task_fault_latched&&r.must_stop&&r.model_failed&&r.failure_code==4&& ...
 r.terrain_extension.first_reason==2&&sameBits(r.terrain_extension.first_terrain15,raw)&& ...
 r.environment_extension.task_env_failed&&~task(r)&&isequal(r.raw_datagram,packet(terrainPacket)));
[q,r]=advance(q,make(5,1234567,3,0,0),p);
check('healthy_after_terrain_fault_never_restores_task',~task(r)&&isequaln(f,q.first_fault)&&~r.stateful_delivery.flight_admission);
% Preserve decoder input and previous-state values.
bytes=packet(bound).';savedBytes=bytes;old=boundState;callCount=callCount+1;
[~,r]=m600check.observeCopterSimDeliveryDiagnostics(old,bytes,1,p);
check('row_raw_datagram_and_previous_state_unmodified',isequal(bytes,savedBytes)&&isequal(r.raw_datagram,savedBytes)&&isequaln(old,boundState));
badState=boundState;badState.schema='UNKNOWN';
check('unknown_state_schema_rejected',throws(@()m600check.observeCopterSimDeliveryDiagnostics(badState,packet(bound),1,p)));
badState=boundState;badState.task_fault_latched=true;
check('latch_without_first_evidence_rejected',throws(@()m600check.observeCopterSimDeliveryDiagnostics(badState,packet(bound),1,p)));
check('all_names_unique',numel(unique({checks.name}))==numel(checks));
report=struct('schema','HOST_COPTERSIM_DELIVERY_OBSERVER_TEST_V1','passed',all([checks.passed]), ...
 'case_count',numel(checks),'cases_passed',sum([checks.passed]),'observer_calls',callCount,'cases',checks, ...
 'COM_open',0,'UDP_open',0,'board_actions',0,'model_started',false,'flight_admission',false,'task_completion_proven',false, ...
 'limitations',{{'Observer state must persist; caller may initialize [] only for an explicitly new observer run.', ...
 'Synthetic packet sequences exercise delivery-observer decoding.', ...
 'Wall freshness, model fault latches and final hardware safety remain in the existing adapter/outer.'}});
names={mfilename,'m600check.observeCopterSimDeliveryDiagnostics','m600check.decodeCopterSimDeliveryDiagnostics', ...
 'm600check.encodeCopterSimTerrainDiagnostics','m600check.decodeCopterSimTerrainDiagnostics'};
report.source_bindings=struct('path',{},'bytes',{},'sha256',{});
for sourceIndex=1:numel(names)
 src=which(names{sourceIndex});info=dir(src);report.source_bindings(end+1)=struct('path',src,'bytes',info.bytes, ...
  'sha256',m600check.fileSha256(src)); %#ok<AGROW>
end
if strlength(outputDir)>0
 mkdir(outputDir);fid=fopen(fullfile(outputDir,'RESULT.json'),'w','n','UTF-8');assert(fid>=0);
 fg=onCleanup(@()fclose(fid));fprintf(fid,'%s\n',jsonencode(report,PrettyPrint=true));clear fg
end
disp(struct('passed',report.passed,'case_count',report.case_count,'cases_passed',report.cases_passed,'observer_calls',callCount));
assert(report.passed,'m600check:DeliveryObserverTests','See compact case receipt.');
 function [s,r]=advance(previous,payload,policy)
  callCount=callCount+1;[s,r]=m600check.observeCopterSimDeliveryDiagnostics(previous,packet(payload),1,policy);
 end
 function check(name,okay),checks(end+1)=struct('name',name,'passed',isscalar(okay)&&logical(okay));end %#ok<AGROW>
 function payload=make(time,session,frame,generation,status)
  payload=zeros(32,1);payload(1:7)=[0;0;time;114.8;1;0;1];payload(26)=2;
  payload(27:32)=[session;frame;generation;p.payload_by_generation_kg(generation+1);p.mass_by_generation_kg(generation+1);status];
 end
end
function bytes=packet(p)
header=int32([1234567890;1]);p=p(:);[~,~,endian]=computer;
if endian=='B',header=swapbytes(header);p=swapbytes(p);end
bytes=[reshape(typecast(header,'uint8'),[],1);reshape(typecast(p,'uint8'),[],1)];
end
function yes=task(r),yes=r.stateful_delivery.can_continue_task&&r.environment_extension.can_continue_task;end
function yes=sameBits(a,b),yes=isequal(typecast(a(:),'uint64'),typecast(b(:),'uint64'));end
function value=bits(hex),value=bitor(bitshift(uint64(hex2dec(hex(1:8))),32),uint64(hex2dec(hex(9:16))));end
function yes=throws(f),yes=false;try,f();catch,yes=true;end,end
