function report=test_local_tunnel_matlab(outputRoot,fixtureRoot,scope)
% Same actual C snapshot/query fixtures, decoded-message transport only.
% CRC/authentication are upstream owner's responsibility, explicitly mocked.
arguments
    outputRoot (1,1) string
    fixtureRoot (1,1) string
    scope (1,1) string="ALL"
end
build=string(fileparts(fileparts(mfilename('fullpath'))));oldPath=path;
restorePath=onCleanup(@()path(oldPath)); %#ok<NASGU>
assert(~isfolder(outputRoot),'Preserve prior result');mkdir(outputRoot);
diary(fullfile(outputRoot,'MATLAB_DIARY.txt'));dg=onCleanup(@()diary('off')); %#ok<NASGU>
addpath(fullfile(build,'host_runtime'),'-begin');
if scope=="LATEST_STATE"
    report=checkLatestState(build);
    f=fopen(fullfile(outputRoot,'RESULT.json'),'w');assert(f>=0);
    fprintf(f,'%s\n',jsonencode(report,PrettyPrint=true));fclose(f);
    disp(jsonencode(report));assert(report.all_pass);return
end
if scope=="RLC_HISTORY"
    report=checkRuntimeHistory(build);
    f=fopen(fullfile(outputRoot,'RESULT.json'),'w');assert(f>=0);
    fprintf(f,'%s\n',jsonencode(report,PrettyPrint=true));fclose(f);
    disp(jsonencode(report));assert(report.all_pass);return
end
assert(scope=="ALL");
sp=fullfile(fixtureRoot,'RLS1_SNAPSHOTS.bin');gp=fullfile(fixtureRoot,'RGP1_RGR1_PAIRS.bin');
tracked=[string(mfilename('fullpath'))+'.m';sp;gp; ...
    string(which('gpenmpcNative.RflyLocalSnapshotDecoder'));string(which('gpenmpcNative.RflyLocalGpCodec')); ...
    string(which('gpenmpcNative.RflyLocalTunnelReassembler'))];before=arrayfun(@sha,tracked);
snap=reshape(readbytes(sp),382,[]);pairs=reshape(readbytes(gp),596,[]);
assert(size(snap,2)==60&&size(pairs,2)==59);
s=gpenmpcNative.RflyLocalSnapshotDecoder(snap(:,1),uint64(100));
expected=struct('uid',s.identity.uid,'boot_generation',s.identity.boot_generation, ...
    'source_system',s.identity.system,'source_component',s.identity.component, ...
    'target_system',uint8(255),'target_component',uint8(190), ...
    'link_lifecycle_generation',uint64(8),'confirmed_host_rx_ns',uint64(100), ...
    'execution_session_sha256',repmat('A',1,64));
origin=struct('link_lifecycle_generation',expected.link_lifecycle_generation, ...
    'execution_session_sha256',expected.execution_session_sha256);
limit=uint64(1000); % Fixture assembly limit.
assembler=gpenmpcNative.RflyLocalTunnelReassembler(expected,limit);
checks=struct('name',{},'pass',{});tick=uint64(1000);snapshotGenerations=zeros(60,1,'uint64');
for k=1:60
    s=gpenmpcNative.RflyLocalSnapshotDecoder(snap(:,k),tick);snapshotGenerations(k)=s.source_generation;
    check(sprintf('rls1_fields_reconstruct_actual_c_bytes_%02d',k),isequal(reencode(s),snap(:,k)));
    check(sprintf('rls1_no_authority_or_clock_%02d',k),~s.control_authority ...
        &&~s.time_mapping_established&&~s.dll_association_proven_here&&~s.full_ekf_history_proven_here);
    sm=fragments(snap(:,k),10,s.source_generation,expected);
    gm={};
    if k<=59
        q=gpenmpcNative.RflyLocalGpCodec.decodeRequest(pairs(1:310,k));
        gm=fragments(pairs(1:310,k),8,q.output_generation,expected);
    end
    % Two different channels may interleave, not two messages in one slot.
    for j=1:4
        assembler.ingest(sm{j},tick,tick+uint64(10),origin);tick=tick+uint64(1);
        if j<=numel(gm),assembler.ingest(gm{j},tick,tick+uint64(10),origin);tick=tick+uint64(1);end
    end
    out=assembler.take('snapshot',tick+uint64(10));
    check(sprintf('rls1_four_real_body_fragments_%02d',k),isequal(out.message,snap(:,k)) ...
        &&out.decoded.source_generation==s.source_generation ...
        &&numel(out.fragment_rx_ns)==4&&~out.freshness_renewed ...
        &&out.original_host_receive_ns==out.fragment_rx_ns(1));
    if k<=59
        gout=assembler.take('gp_request',tick+uint64(10));
        check(sprintf('rgp1_three_real_body_fragments_%02d',k),isequal(gout.message,pairs(1:310,k)) ...
            &&gout.decoded.output_generation==q.output_generation ...
            &&numel(gout.fragment_rx_ns)==3);
    end
    tick=tick+uint64(100);
end
for k=1:59
    q=gpenmpcNative.RflyLocalGpCodec.decodeRequest(pairs(1:310,k));
    indices=find(snapshotGenerations==q.source_generation);
    check(sprintf('gp_request_has_unique_same_actual_snapshot_%02d',k),numel(indices)==1);
    s=gpenmpcNative.RflyLocalSnapshotDecoder(snap(:,indices),tick);
    check(sprintf('gp_source_hrt_uid_exact_%02d',k),q.source_timestamp_ns==s.original_sample_us*uint64(1000) ...
        &&isequal(q.identity,s.identity));
end
state=assembler.status();check('full_denominator_417_fragments_119_messages', ...
    state.private_fragments_received==417&&isequal(state.messages_completed,uint64([59 60 0])) ...
    &&~any(state.active)&&~any(state.ready)&&~state.failed&&state.connections==0);
assembler.close();check('closed_no_restart',reject(@()assembler.poll(tick+uint64(100))));

s=gpenmpcNative.RflyLocalSnapshotDecoder(snap(:,1),uint64(100));
m=fragments(snap(:,1),10,s.source_generation,expected);
for c=1:19
    a=gpenmpcNative.RflyLocalTunnelReassembler(expected,limit);bad=m;ori=origin;
    switch c
        case 1,bad{1}.SystemID=uint8(2);
        case 2,bad{1}.Payload.target_component=uint8(3);
        case 3,bad{1}.Payload.payload(1)=uint8(48); % forbidden legacy schema3
        case 4,bad{1}.Payload.payload(1)=uint8(164); % index4 of four
        case 5,bad{1}.Payload.payload_length=uint8(127);
        case 6,bad{4}.Payload.payload(end)=uint8(1);
        case 7,bad{2}.Payload.payload(9)=bitxor(bad{2}.Payload.payload(9),uint8(1));
        case 8,bad=m([2 1 3 4]);
        case 9,bad=m([1 1 3 4]);
        case 10,bad{1}.Payload.payload(12)=bitxor(bad{1}.Payload.payload(12),uint8(1));
        case 11,ori.link_lifecycle_generation=uint64(9);
    end
    if c<=11
        check(sprintf('negative_%02d_permanent_fault',c),reject(@()feed(a,bad,ori,uint64(200),uint64(210)))&&a.Failed);
    elseif c==12
        a.ingest(m{1},uint64(200),uint64(210),ori);
        check('partial_timeout_retained',reject(@()a.poll(uint64(1201)))&&a.Failed&&a.evidence().status.active(2));
    elseif c==13
        feed(a,m,ori,uint64(200),uint64(210));
        check('unconsumed_complete_timeout',reject(@()a.take('snapshot',uint64(1201)))&&a.Failed);
    elseif c==14
        feed(a,m,ori,uint64(200),uint64(210));a.take('snapshot',uint64(220));
        check('completed_generation_replay',reject(@()feed(a,m,ori,uint64(230),uint64(240)))&&a.Failed);
    elseif c==15
        feed(a,m,ori,uint64(200),uint64(210));
        s2=gpenmpcNative.RflyLocalSnapshotDecoder(snap(:,2),uint64(220));m2=fragments(snap(:,2),10,s2.source_generation,expected);
        check('complete_queue_overflow_no_overwrite',reject(@()feed(a,m2,ori,uint64(230),uint64(240)))&&a.Failed);
    elseif c==16
        a.ingest(m{1},uint64(200),uint64(210),ori);
        check('original_receive_reversed',reject(@()a.ingest(m{2},uint64(199),uint64(211),ori))&&a.Failed);
    elseif c==17
        a.ingest(m{1},uint64(200),uint64(210),ori);
        check('processing_clock_reversed',reject(@()a.ingest(m{2},uint64(201),uint64(209),ori))&&a.Failed);
    elseif c==18
        check('queued_first_fragment_expired',reject(@()a.ingest(m{1},uint64(200),uint64(1201),ori))&&a.Failed);
    elseif c==19
        a.close();check('closed_ingress_no_reopen',reject(@()feed(a,m,ori,uint64(200),uint64(210)))&&a.Failed);
    end
    check(sprintf('negative_%02d_no_restart',c),reject(@()a.poll(uint64(2000))));
end
for k=1:5
    b=snap(:,1);
    if k==1,b(383-32)=bitxor(b(383-32),uint8(1));end
    if k==2,b(105:108)=uint8(255);b=checksum(b);end
    if k==3,b(349)=uint8(2);b=checksum(b);end
    if k==4,b(47:54)=uint8(0);b=checksum(b);end
    if k==5,b(277)=bitxor(b(277),uint8(1));end
    check(sprintf('rls1_corrupt_%d_reject',k),reject(@()gpenmpcNative.RflyLocalSnapshotDecoder(b,uint64(200))));
end
check('all_source_inputs_stable',isequal(before,arrayfun(@sha,tracked)));
report=struct('scope','ACTUAL_C_RLS1_RGP1_TO_MATLAB_EXISTING_LINK_DISPATCH_HOST_ONLY', ...
    'checks',checks,'passed',sum([checks.pass]),'total',numel(checks),'all_pass',all([checks.pass]), ...
    'actual_c_snapshot_records',60,'actual_c_gp_queries',59,'positive_fragments',417, ...
    'tracked',tracked,'sha256',before,'synthetic_host_time',true, ...
    'decoded_mavlink_envelope_fixture',true,'upstream_crc_authentication_mock',true, ...
    'actual_mavlink_crc_verified_here',false,'hardware_actions',0,'COM',0,'connections',0);
f=fopen(fullfile(outputRoot,'RESULT.json'),'w','n','UTF-8');assert(f>=0);fg=onCleanup(@()fclose(f)); %#ok<NASGU>
fprintf(f,'%s\n',jsonencode(report,PrettyPrint=true));
fprintf('Local MATLAB RLS/GP dispatcher %d/%d\n',report.passed,report.total);assert(report.all_pass);
    function check(name,pass)
        checks(end+1)=struct('name',name,'pass',logical(pass)); %#ok<AGROW>
        if ~pass,fprintf(2,'FAILED %s\n',name);end
    end
end
function report=checkLatestState(build)
% Actual sole IO latest mailbox and its consumer guards; loopback only.
addpath(fullfile(build,'tools'),fullfile(build,'matlab_validation'),fullfile(build,'m600_coptersim','matlab_validation'));
addpath(gpenmpc_external_path('native_visual_host_source'),'-end');
x=load(fullfile(gpenmpc_external_path('tunnel_source_binding'),'SHORT_HIL','RAW_BOARD_LOCAL_SHORT_HIL.mat'),'cfg','rawIo','methodRaw');
c=x.cfg;c.local_mavlink_port=62291;c.remote_mavlink_port=62292;c.truth_port=62293;c.coptersim_time_port=62294;
c.mavlink_transport.scope='HOST_ONLY_LOOPBACK';c.mavlink_transport.local_port=62291;c.mavlink_transport.remote_port=62292;
c.delivery_environment_contract.remote_port=62295;addpath(fileparts(c.mavlink_transport.source.exact_path),'-begin');
e=x.rawIo.canonical_exchange_expected;wanted={};
for k=1:numel(x.methodRaw)
 z=x.methodRaw{k};
 if isfield(z,'mode')&&strcmp(z.mode,'FLIGHT')&&isfield(z,'source')&&isstruct(z.source)&&isfield(z.source,'message')
  wanted{end+1}=z.source;
  if numel(wanted)==2,break;end
 end
end
assert(numel(wanted)==2);
d=mavlinkdialect(fullfile(build,'m600_coptersim','matlab_validation','+m600check','px4_health_events.xml'),2);
sender=mavlinkio(d,'SystemID',1,'ComponentID',1);
peer=udpport('datagram','IPV4','LocalHost','127.0.0.1','LocalPort',62292);
cleanup=onCleanup(@()closeLatestPeers(peer,sender));checks=struct('name',{},'pass',{});
for trial=0:4
 io=m600check.makeM600CopterSimIo(c);cleanupIo=onCleanup(io.close);io.bindCanonicalSession(e);
 first=wanted{1}.message;second=wanted{2}.message;
 if trial==2,second(end)=bitxor(second(end),uint8(1));end
 if trial==3,second(5:12)=be(e.uid+uint64(1));second=checksum(second);end
 if trial==4,second(132:139)=be(NaN);second=checksum(second);end
 wire={};
 for q=1:2
  body=first;if q==2,body=second;end
  parts=fragments(body,10,u64body(body(47:54)),struct('source_system',uint8(1),'source_component',uint8(1), ...
   'target_system',uint8(255),'target_component',uint8(190)));
  for j=1:4
   msg=createmsg(d,'TUNNEL');msg.Payload=parts{j}.Payload;wire{end+1}=uint8(serializemsg(sender,msg));
  end
 end
 % Warm the receiver with the first body before sending the second body.
 for q=1:2
  for j=(q-1)*4+1:q*4,write(peer,wire{j},'uint8','127.0.0.1',62291);end
  pause(.002);
  for j=1:12
   state=io.pollCanonical(true);assert(isempty(state.failure));
   if state.status.private_fragments_received==q*4,break;end
  end
 end
 if trial==0
  % Treat cold receiver initialization as preparation.
  disp(struct('cold_preparation_only',true,'status',state.status));
  if state.completed_queue_counts(2)>0
   try,io.takeCanonical('snapshot',false);catch,end
  end
  clear cleanupIo
  continue
 end
 if state.status.last_generation(2)~=wanted{2}.generation||state.completed_queue_counts(2)~=1
  disp(trial);disp(state);disp(state.status);
 end
 assert(state.status.last_generation(2)==wanted{2}.generation&&state.completed_queue_counts(2)==1, ...
  'The latest original generation must reach the existing mailbox; expired older state need not complete.');
 bad=false;value=[];
 try,value=io.takeCanonical('snapshot',false);catch problem,bad=true;disp(problem.identifier);end
 if trial==1
  expected=gpenmpcNative.RflyLocalSnapshotDecoder(second,value.original_host_receive_ns);
  if bad||~isequaln(value.decoded,expected)||~isequal(value.message,second)
   disp(which('m600check.makeM600CopterSimIo'));disp(size(value.message));disp(size(second));
   disp(struct('bad',bad,'decoded_equal',isequaln(value.decoded,expected),'bytes_equal',isequal(value.message,second)));
   disp(value.decoded);
  end
  assert(~bad&&isequaln(value.decoded,expected)&&isequal(value.message,second));
  raw=io.evidence();assert(numel(raw.raw_mavlink)==8);
  checks(end+1)=struct('name','latest_exact_decoded_before_use_all_original_frames_retained','pass',true);
 else
  assert(bad);names={'','digest_rejected_at_use','wrong_uid_rejected_at_use','nonfinite_rejected_at_use'};
  checks(end+1)=struct('name',names{trial},'pass',true);
 end
 clear cleanupIo
end
report=struct('all_pass',all([checks.pass]),'checks',checks,'hardware_actions',0,'controls',0, ...
 'source','RETAINED_TUNNEL_BYTES_HOST_FIXTURE');
clear cleanup
end
function closeLatestPeers(peer,sender),delete(peer);delete(sender);end
function v=u64body(b),v=swapbytes(typecast(reshape(b,[],1),'uint64'));end
function report=checkRuntimeHistory(build)
% Only the changed assembly route. Original retained RLC2 bytes are used;
% synthetic host times test scheduling, not actual board/transport timing.
x=load(fullfile(fileparts(fileparts(build)),'assets','runtime','receiver_warmup.mat'));
c=x.records{1};d=gpenmpcNative.RflyLocalCommittedDecoder(c.message,c.original_host_receive_ns);
e=struct('uid',d.identity.uid,'boot_generation',d.identity.boot_generation, ...
 'source_system',d.identity.system,'source_component',d.identity.component, ...
 'target_system',uint8(255),'target_component',uint8(190), ...
 'link_lifecycle_generation',uint64(8),'confirmed_host_rx_ns',uint64(100), ...
 'execution_session_sha256',repmat('A',1,64),'runtime_state_only',true,'component_initialization',false);
o=struct('link_lifecycle_generation',e.link_lifecycle_generation,'execution_session_sha256',e.execution_session_sha256);
m=fragments(c.message,14,d.output_generation,e);limit=uint64(50000000);first=uint64(1000);
a=gpenmpcNative.RflyLocalTunnelReassembler(e,limit);
a.ingest(m{1},first,first,o);
partial=a.take('committed_state',first+limit+uint64(1));
checks=struct('name','partial_history_not_a_control_record','pass',isempty(partial)&&~a.Failed);
for k=2:numel(m)
 t=first+limit+uint64(k);a.ingest(m{k},t,t,o);
end
r=a.take('committed_state',first+2*limit);
checked=gpenmpcNative.RflyLocalCommittedDecoder(r.message,r.original_host_receive_ns);
checks(end+1)=struct('name','complete_history_preserves_bytes_identity_original_time', ...
 'pass',isequal(r.message,c.message)&&checked.output_generation==d.output_generation ...
 &&r.original_host_receive_ns==first&&~r.freshness_renewed&&~r.control_authority);
bad=gpenmpcNative.RflyLocalTunnelReassembler(e,limit);bad.ingest(m{1},first,first,o);
checks(end+1)=struct('name','history_order_still_fail_closed', ...
 'pass',reject(@()bad.ingest(m{3},first+limit+uint64(1),first+limit+uint64(1),o))&&bad.Failed);
e.runtime_state_only=false;legacy=gpenmpcNative.RflyLocalTunnelReassembler(e,limit);
legacy.ingest(m{1},first,first,o);
checks(end+1)=struct('name','non_runtime_assembly_limit_unchanged', ...
 'pass',reject(@()legacy.poll(first+limit+uint64(1)))&&legacy.Failed);
e.runtime_state_only=true;
f=fullfile(build,'rfly_vendor_integration','full_inner_abi','snapshot_wire_fixture');
b=reshape(readbytes(fullfile(f,'RGP1_RGR1_PAIRS.bin')),596,[]);q=gpenmpcNative.RflyLocalGpCodec.decodeRequest(b(1:310,1));
e.uid=q.identity.uid;e.boot_generation=q.identity.boot_generation;
e.source_system=q.identity.system;e.source_component=q.identity.component;
g=fragments(b(1:310,1),8,q.output_generation,e);a=gpenmpcNative.RflyLocalTunnelReassembler(e,limit);
a.ingest(g{1},first,first,o);
late=first+limit+uint64(1);a.poll(late);
checks(end+1)=struct('name','partial_GP_never_delivered_despite_HOST_queue_delay', ...
 'pass',isempty(a.take('gp_request',late))&&~a.Failed);
for k=2:3,a.ingest(g{k},first+uint64(k-1),late+uint64(k),o);end
ev=a.evidence();
checks(end+1)=struct('name','fully_checked_late_GP_retired_without_control_session_failure', ...
 'pass',isempty(a.take('gp_request',late+uint64(4)))&&~a.Failed ...
 &&ev.status.last_generation(1)==q.output_generation&&ev.status.messages_completed(1)==1);
next=gpenmpcNative.RflyLocalGpCodec.decodeRequest(b(1:310,2));
gn=fragments(b(1:310,2),8,next.output_generation,e);t=late+uint64(10);feed(a,gn,o,t,t);
fresh=a.take('gp_request',t+uint64(4));
checks(end+1)=struct('name','next_genuine_GP_keeps_original_bytes_and_receive_time', ...
 'pass',~a.Failed&&isequal(fresh.message,b(1:310,2))&&fresh.original_host_receive_ns==t&&~fresh.freshness_renewed);
bad=gpenmpcNative.RflyLocalTunnelReassembler(e,limit);bad.ingest(g{1},first,first,o);
checks(end+1)=struct('name','genuine_GP_wire_gap_still_rejected', ...
 'pass',reject(@()bad.ingest(g{2},late,late,o))&&bad.Failed);
bad=gpenmpcNative.RflyLocalTunnelReassembler(e,limit);bad.ingest(g{1},first,late,o);
checks(end+1)=struct('name','late_GP_wrong_fragment_order_still_rejected', ...
 'pass',reject(@()bad.ingest(g{3},first+uint64(1),late+uint64(1),o))&&bad.Failed);
bad=gpenmpcNative.RflyLocalTunnelReassembler(e,limit);corrupt=g;
corrupt{3}.Payload.payload(12)=bitxor(corrupt{3}.Payload.payload(12),uint8(1));
checks(end+1)=struct('name','late_GP_body_checksum_still_rejected', ...
 'pass',reject(@()feed(bad,corrupt,o,first,late))&&bad.Failed);
b=reshape(readbytes(fullfile(f,'RLS1_SNAPSHOTS.bin')),382,[]);s=gpenmpcNative.RflyLocalSnapshotDecoder(b(:,1),first);
e.uid=s.identity.uid;e.boot_generation=s.identity.boot_generation;
e.source_system=s.identity.system;e.source_component=s.identity.component;
m=fragments(b(:,1),10,s.source_generation,e);a=gpenmpcNative.RflyLocalTunnelReassembler(e,limit);
feed(a,m,o,first,first);v=a.take('snapshot',first+limit+uint64(1));
checks(end+1)=struct('name','old_complete_snapshot_retained_without_fresh_input_authority', ...
 'pass',~isempty(v)&&~a.Failed&&isequal(v.message,b(:,1)) ...
 &&v.original_host_receive_ns==first&&~v.freshness_renewed&&~v.control_authority ...
 &&first+limit+uint64(1)-v.original_host_receive_ns>limit);
% Replay consecutive fragments with callback delay beyond fast-input age;
% preserve the bytes for numerical and historical pairing.
a=gpenmpcNative.RflyLocalTunnelReassembler(e,limit);
feed(a,m,o,first,first+limit+uint64(10));v=a.take('snapshot',first+limit+uint64(20));
checked=gpenmpcNative.RflyLocalSnapshotDecoder(v.message,v.original_host_receive_ns);
checks(end+1)=struct('name','queued_complete_RLS_keeps_original_numeric_pair', ...
 'pass',~a.Failed&&checked.source_generation==s.source_generation ...
 &&isequal(v.message,b(:,1))&&v.original_host_receive_ns==first ...
 &&isequal(v.fragment_rx_ns,first+uint64((0:3)'))&&~v.control_authority);
bad=gpenmpcNative.RflyLocalTunnelReassembler(e,limit);bad.ingest(m{1},first,first,o);
z=bad.ingest(m{2},first+limit+uint64(1),first+limit+uint64(1),o);
checks(end+1)=struct('name','actual_RLS_wire_timeout_still_not_delivered', ...
 'pass',strcmp(z.status,'EXPIRED_SNAPSHOT_NOT_USED')&&isempty(bad.take('snapshot',first+limit+uint64(2))));
next=gpenmpcNative.RflyLocalSnapshotDecoder(b(:,2),first);
mn=fragments(b(:,2),10,next.source_generation,e);t=first+limit+uint64(3);
feed(bad,mn,o,t,t);v=bad.take('snapshot',t+uint64(4));
checks(end+1)=struct('name','next_actual_RLS_after_wire_timeout_keeps_original_time', ...
 'pass',~bad.Failed&&isequal(v.message,b(:,2))&&v.original_host_receive_ns==t);
bad=gpenmpcNative.RflyLocalTunnelReassembler(e,limit);bad.ingest(m{1},first,first+limit,o);
checks(end+1)=struct('name','queued_RLS_wrong_order_still_rejected', ...
 'pass',reject(@()bad.ingest(m{3},first+uint64(1),first+limit+uint64(1),o))&&bad.Failed);
% Exercise the send consumer's age rejection.
pair=reshape(readbytes(fullfile(gpenmpc_external_path('local_task_input'),'MATCHED_RLS_RLI.bin')),1029,[]);
u=gpenmpcNative.RflyLocalTaskCodec.decode(pair(383:end,1));
e.uid=u.source.identity.uid;e.boot_generation=u.source.identity.boot_generation;
e.session_generation=e.boot_generation;e.source_system=u.source.identity.system;e.source_component=u.source.identity.component;
e.execution_session_sha256=upper(reshape(dec2hex(u.execution_session_sha256,2).',1,[]));
e.configuration_sha256=upper(reshape(dec2hex(u.configuration_sha256,2).',1,[]));
o.execution_session_sha256=e.execution_session_sha256;
m=fragments(pair(1:382,1),10,u.source.source_generation,e);
a=gpenmpcNative.RflyLocalTunnelReassembler(e,limit);feed(a,m,o,first,first);v=a.take('snapshot',first+uint64(4));
[payloads,binding]=gpenmpcNative.validateLocalTaskInputForSend(pair(383:end,1),v,e,limit,first+uint64(4));
checks(end+1)=struct('name','same_real_send_consumer_accepts_fresh_exact_RLS_RLI', ...
 'pass',numel(payloads)==6&&binding.original_host_receive_ns==first);
checks(end+1)=struct('name','history_retention_never_extends_actual_fast_input_50ms', ...
 'pass',reject(@()gpenmpcNative.validateLocalTaskInputForSend(pair(383:end,1),v,e,limit,first+limit+uint64(1))));
report=struct('checks',checks,'all_pass',all([checks.pass]),'passed',sum([checks.pass]), ...
 'total',numel(checks),'hardware_actions',0,'synthetic_host_clock',true,'original_RLC2_bytes',true);
end
function feed(a,m,o,rx,now)
for k=1:numel(m),a.ingest(m{k},rx+uint64(k-1),now+uint64(k-1),o);end
end
function m=fragments(b,schema,gen,e)
n=ceil(numel(b)/119);m=cell(n,1);
for k=0:n-1
    count=min(119,numel(b)-119*k);p=zeros(128,1,'uint8');
    p(1)=bitor(bitshift(uint8(schema),4),uint8(k));p(2:9)=be(gen);p(10:9+count)=b(119*k+1:119*k+count);
    m{k+1}=struct('MsgID',uint32(385),'SystemID',e.source_system,'ComponentID',e.source_component, ...
        'Payload',struct('target_system',e.target_system,'target_component',e.target_component, ...
        'payload_type',uint16(42002),'payload_length',uint8(9+count),'payload',p));
end
end
function b=reencode(s)
i=s.identity;
b=[uint8('RLS1').';be(i.uid);be(i.boot_generation);i.system;i.component; ...
    be(s.original_sample_us);be(s.original_publication_us);be(s.original_receipt_us); ...
    be(s.source_generation);be(s.generation_delta);be(s.sample_delta_us);s.reset_counter; ...
    s.state_and_origin_sha256;be(s.odometry_instance);be(s.task_origin_ned_m);be(s.canonical_state13); ...
    be(s.endpoint_receiver_hrt_us);be(s.endpoint_wire_time_us);be(s.endpoint_event_ordinal); ...
    be(s.fields_updated);be(s.receiver_instance);be(s.receiver_channel);s.hil_system;s.hil_component; ...
    s.hil_mavlink_sequence;s.hil_payload_length;s.hil_sensor_id;s.original_sensor52; ...
    be(s.gyro_instance);be(s.accel_instance);be(s.gyro_device_id);be(s.accel_device_id); ...
    be(s.original_endpoint_subscription_generation);s.gyro_update_called;s.accel_update_called;zeros(32,1,'uint8')];
b=checksum(b);
end
function b=be(v)
[~,~,endian]=computer;if endian=='L',v=swapbytes(v);end;b=reshape(typecast(v(:),'uint8'),[],1);
end
function b=readbytes(p)
f=fopen(p,'rb');assert(f>=0);g=onCleanup(@()fclose(f));b=fread(f,Inf,'*uint8'); %#ok<NASGU>
end
function ok=reject(f)
ok=false;try,f();catch,ok=true;end
end
function b=checksum(b)
md=java.security.MessageDigest.getInstance('SHA-256');md.update(typecast(b(1:end-32),'int8'));
b(end-31:end)=reshape(typecast(md.digest(),'uint8'),[],1);
end
function h=sha(p)
b=readbytes(p);md=java.security.MessageDigest.getInstance('SHA-256');md.update(typecast(b,'int8'));
h=string(upper(reshape(dec2hex(typecast(md.digest(),'uint8'),2).',1,[])));
end
