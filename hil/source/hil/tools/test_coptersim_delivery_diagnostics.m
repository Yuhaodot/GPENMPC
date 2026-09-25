function report=test_coptersim_delivery_diagnostics(outputDir)
% Test packet decoding with a synthetic mass table.
arguments,outputDir (1,1) string = "",end
build=string(fileparts(fileparts(mfilename('fullpath'))));
oldPath=path;pathGuard=onCleanup(@()path(oldPath)); %#ok<NASGU>
addpath(fullfile(build,'matlab_validation'),'-begin');
addpath(fullfile(build,'m600_coptersim','matlab_validation'),'-end');
if strlength(outputDir)>0
 assert(~isfolder(outputDir)&&~isfile(outputDir),'m600check:OutputExists','Preserve previous tests.');
end
checks=struct('name',{},'passed',{},'actual_status',{});
policy=struct('expected_session_token',1234567,'payload_by_generation_kg',[2.21 1.75 .98 .55 0], ...
 'mass_by_generation_kg',9.5+[2.21 1.75 .98 .55 0]);
healthy=[0;0;10;114.8;1;0;1];terrain=zeros(15,1);
state=m600check.initialCopterSimTerrainDiagnosticState();
[v1,~]=m600check.encodeCopterSimTerrainDiagnostics(healthy,terrain,0,0,false,true,state);
ack=struct('valid',true,'task_env_failed',false,'failure_code',0,'session_token',1234567, ...
 'applied_frame_generation',1,'applied_payload_generation',0,'actual_payload_kg',2.21,'actual_total_mass_kg',11.71);
v2=m600check.encodeCopterSimDeliveryDiagnostics(v1,ack,false);
baseBytes=packet(v2);base=decode(v2);
check('healthy_v2_exact_session_and_committed_mass',base.packet_valid&&base.environment_extension.valid&& ...
 base.environment_extension.session_bound&&base.environment_extension.can_continue_task&&base.environment_extension.mass_ack_valid,base.status);
check('terrain_first25_bits_unchanged',sameBits(v1(1:25),v2(1:25)),'ENCODER');
check('raw_datagram_unchanged_column',isequal(base.raw_datagram,baseBytes),'RAW');
row=m600check.decodeCopterSimDeliveryDiagnostics(baseBytes.',1,9,policy);
check('raw_datagram_unchanged_row',isequal(row.raw_datagram,baseBytes.')&&row.packet_valid,'RAW');
check('little_endian_header_is_official',isequal(baseBytes(1:8),uint8([210;2;150;73;1;0;0;0])),'WIRE');
check('decoded_source_clock_retained_not_new_task_clock',base.environment_extension.same_model_time_s==10&& ...
 base.environment_extension.applied_time_not_encoded,'CLOCK');
check('codec_no_task_completion_or_flight_claim',~isfield(base,'task_complete')&&~isfield(base,'flight_pass')&& ...
 ~isfield(base.environment_extension,'task_complete'),'SCOPE');
for gen=0:4
 a=ack;a.applied_payload_generation=gen;a.applied_frame_generation=gen+1;
 a.actual_payload_kg=policy.payload_by_generation_kg(gen+1);a.actual_total_mass_kg=policy.mass_by_generation_kg(gen+1);
 p=m600check.encodeCopterSimDeliveryDiagnostics(v1,a,false);d=decode(p);
 check(sprintf('generation_%d_exact_expected_mass',gen),d.packet_valid&&d.environment_extension.mass_ack_valid&& ...
  d.environment_extension.applied_payload_generation==gen&&isequal(d.raw_datagram,packet(p)),d.status);
end
for code=1:15
 a=ack;a.task_env_failed=true;a.failure_code=code;
 p=m600check.encodeCopterSimDeliveryDiagnostics(v1,a,false);d=decode(p);e=d.environment_extension;
 check(sprintf('environment_failure_%02d_not_task_permission_but_model_observable',code), ...
  d.packet_valid&&e.valid&&e.task_env_failed&&e.status_code==code&&~e.can_continue_task&&~e.mass_ack_valid&& ...
  d.can_use_as_healthy_observation&&~d.must_stop&&e.keep_same_plant_evolving&&~e.request_plant_reset&& ...
  isequal(d.raw_datagram,packet(p)),d.status);
 a.valid=false;p=m600check.encodeCopterSimDeliveryDiagnostics(v1,a,true);
 check(sprintf('environment_failure_%02d_precedes_init_pending',code),p(32)==code,'ENCODER');
end
a=ack;a.valid=false;a.applied_frame_generation=0;
initial=m600check.encodeCopterSimDeliveryDiagnostics(v1,a,true);d=decode(initial);
check('status16_initial_not_commit_or_task',d.packet_valid&&d.environment_extension.initial_not_applied&& ...
 d.environment_extension.valid&&~d.environment_extension.mass_ack_valid&&~d.environment_extension.can_continue_task&&initial(32)==16,d.status);
pending=m600check.encodeCopterSimDeliveryDiagnostics(v1,ack,true);d=decode(pending);
check('status17_pending_retains_old_applied_identity_without_ack',d.packet_valid&&d.environment_extension.pending&& ...
 d.environment_extension.applied_frame_generation==1&&~d.environment_extension.mass_ack_valid&& ...
 ~d.environment_extension.can_continue_task&&d.can_use_as_healthy_observation,d.status);
% Legacy/V1 path is not replaced, and the delivery decoder refuses to invent V2.
legacy=zeros(32,1);legacy(1:7)=healthy;old=m600check.decodeCopterSimDiagnostics(packet(legacy),1,9);
legacyTerrain=m600check.decodeCopterSimTerrainDiagnostics(packet(legacy),1,9,false);
same=true;names=fieldnames(old);
for fieldIndex=1:numel(names),same=same&&isequaln(old.(names{fieldIndex}),legacyTerrain.(names{fieldIndex}));end
check('legacy_all_fields_unchanged_on_explicit_legacy_path',same,'LEGACY');
before=m600check.decodeCopterSimTerrainDiagnostics(packet(v1),1,9,true);
d=decode(v1);after=m600check.decodeCopterSimTerrainDiagnostics(packet(v1),1,9,true);
check('v1_delivery_refused_and_existing_decoder_unchanged',stops(d)&&isequaln(before,after)&& ...
 isequal(d.raw_datagram,packet(v1)),d.status);
% Raw reason2 payloads: no float arithmetic, no NaN canonicalization.
rawValues={typecast(bits('7FF8000000001234'),'double'),typecast(bits('FFF800000000ABCD'),'double'),Inf,-Inf};
rawNames={'POSITIVE_NAN_PAYLOAD','NEGATIVE_NAN_PAYLOAD','POSITIVE_INF','NEGATIVE_INF'};
failed=healthy;failed(1:2)=[1;4];
for channel=1:15
 for kind=1:numel(rawValues)
  raw=zeros(15,1);raw(channel)=rawValues{kind};
  [faultV1,~]=m600check.encodeCopterSimTerrainDiagnostics(failed,raw,2,0,true,false,state);
  p=m600check.encodeCopterSimDeliveryDiagnostics(faultV1,ack,false);d=decode(p);
  check(sprintf('terrain_ch%02d_%s_bits_and_fault_retained',channel,rawNames{kind}), ...
   d.packet_valid&&d.model_failed&&d.failure_code==4&&d.must_stop&&~d.can_use_as_healthy_observation&& ...
   d.terrain_extension.valid&&d.terrain_extension.first_reason==2&& ...
   sameBits(p(1:25),faultV1(1:25))&&sameBits(d.payload(1:25),faultV1(1:25))&& ...
   sameBits(d.terrain_extension.first_terrain15,raw)&&isequal(d.raw_datagram,packet(p))&& ...
   ~d.environment_extension.can_continue_task&&~d.environment_extension.mass_ack_valid,d.status);
 end
end
raw=zeros(15,1);raw(1)=typecast(bits('8000000000000000'),'double');raw(15)=Inf;
[faultV1,~]=m600check.encodeCopterSimTerrainDiagnostics(failed,raw,2,0,true,false,state);
for code=1:15
 a=ack;a.task_env_failed=true;a.failure_code=code;p=m600check.encodeCopterSimDeliveryDiagnostics(faultV1,a,false);d=decode(p);
 check(sprintf('env_%02d_cannot_mask_terrain_failure_or_signed_zero',code),d.packet_valid&&d.must_stop&& ...
  d.model_failed&&d.failure_code==4&&d.environment_extension.task_env_failed&& ...
  d.environment_extension.valid&&~d.environment_extension.can_continue_task&& ...
  sameBits(d.terrain_extension.first_terrain15,raw)&&isequal(d.raw_datagram,packet(p)),d.status);
end
% Exact little-endian codec, not platform-default byte reversal.
wrongEndian=baseBytes;for word=1:32,ix=9+(word-1)*8:16+(word-1)*8;wrongEndian(ix)=flipud(wrongEndian(ix));end
d=m600check.decodeCopterSimDeliveryDiagnostics(wrongEndian,1,9,policy);
check('wrong_endian_double_words_rejected',stops(d)&&isequal(d.raw_datagram,wrongEndian),d.status);
% Extension malformed fields, exact mass identities, unknown protocol/status.
mutations={26,0,'tag0';26,1,'tag1';26,3,'tag3';26,NaN,'tagNaN'; ...
 27,0,'session_zero';27,1234568,'wrong_session';27,1.5,'session_fraction'; ...
 28,-1,'frame_negative';28,.5,'frame_fraction';28,2^32,'frame_overflow'; ...
 29,-1,'payload_generation_negative';29,.5,'payload_generation_fraction';29,5,'payload_generation_unknown'; ...
 30,2.21+eps(2.21),'payload_one_ulp_wrong';30,-1,'payload_negative'; ...
 31,11.71+eps(11.71),'total_mass_one_ulp_wrong';31,0,'total_mass_zero'; ...
 32,-1,'status_negative';32,.5,'status_fraction';32,18,'status_unknown';28,0,'healthy_without_core_commit'};
for mutationIndex=1:size(mutations,1)
 p=v2;p(mutations{mutationIndex,1})=mutations{mutationIndex,2};d=decode(p);
 check(['malformed_' mutations{mutationIndex,3}],stops(d)&&isequal(d.raw_datagram,packet(p)),d.status);
end
for fieldIndex=27:32
 for valueIndex=1:3
  nonfinite=[NaN Inf -Inf];p=v2;p(fieldIndex)=nonfinite(valueIndex);d=decode(p);
  check(sprintf('nonfinite_extension_field_%d_kind%d',fieldIndex,valueIndex),stops(d)&&isequal(d.raw_datagram,packet(p)),d.status);
 end
end
for fieldIndex=1:10
 p=v2;p(fieldIndex)=NaN;d=decode(p);
 check(sprintf('nonfinite_prefix_terrain_metadata_%d',fieldIndex),stops(d)&&isequal(d.raw_datagram,packet(p)),d.status);
end
p=initial;p(28)=1;d=decode(p);check('initial_cannot_claim_applied_frame',stops(d),d.status);
p=initial;p(29)=1;p(30)=policy.payload_by_generation_kg(2);p(31)=policy.mass_by_generation_kg(2);
d=decode(p);check('initial_cannot_claim_payload_generation',stops(d),d.status);
p=v2;p(8)=1;p(9)=2;d=decode(p);check('healthy_prefix_cannot_hide_fake_terrain_capture',stops(d),d.status);
p=m600check.encodeCopterSimDeliveryDiagnostics(faultV1,ack,false);p(11:25)=0;d=decode(p);
check('terrain_reason2_requires_actual_nonfinite_evidence',stops(d),d.status);
d=m600check.decodeCopterSimDeliveryDiagnostics(baseBytes,1,11,policy);
check('source_clock_reverse_retains_stop_not_task_ack',d.packet_valid&&d.must_stop&&~d.time_monotonic&& ...
 ~d.environment_extension.can_continue_task&&~d.environment_extension.mass_ack_valid&&isequal(d.raw_datagram,baseBytes),d.status);
% Pre-session observation permission does not authorize task execution.
unbound=initial;unbound(27)=0;d=decode(unbound);
check('unbound_initial_default_rejected',stops(d),d.status);
allow=policy;allow.allow_unbound_pre_session=true;
d=m600check.decodeCopterSimDeliveryDiagnostics(packet(unbound),1,9,allow);
check('explicit_unbound_initial_observable_only',d.packet_valid&&d.can_use_as_healthy_observation&& ...
 d.environment_extension.valid&&d.environment_extension.initial_not_applied&&~d.environment_extension.session_bound&& ...
 ~d.environment_extension.can_continue_task&&~d.environment_extension.mass_ack_valid&& ...
 ~d.environment_extension.request_plant_reset&&isequal(d.raw_datagram,packet(unbound)),d.status);
for code=[0:15 17 18]
 p=unbound;p(32)=code;if code==0,p(28)=1;end
 d=m600check.decodeCopterSimDeliveryDiagnostics(packet(p),1,9,allow);
 check(sprintf('allow_unbound_does_not_allow_status_%02d',code),stops(d),d.status);
end
p=unbound;p(28)=1;d=m600check.decodeCopterSimDeliveryDiagnostics(packet(p),1,9,allow);
check('allow_unbound_does_not_allow_applied_frame',stops(d),d.status);
p=unbound;p(27)=42;d=m600check.decodeCopterSimDeliveryDiagnostics(packet(p),1,9,allow);
check('allow_unbound_does_not_allow_unknown_nonzero_session',stops(d),d.status);
p=unbound;p(31)=p(31)+eps(p(31));d=m600check.decodeCopterSimDeliveryDiagnostics(packet(p),1,9,allow);
check('unbound_still_requires_exact_mass',stops(d),d.status);
badPolicy=allow;badPolicy.allow_unbound_pre_session=1;
check('numeric_allow_flag_refused',throws(@()m600check.decodeCopterSimDeliveryDiagnostics(baseBytes,1,9,badPolicy)),'POLICY');
badPolicy=allow;badPolicy.allow_unbound_pre_session=[true false];
check('nonscalar_allow_flag_refused',throws(@()m600check.decodeCopterSimDeliveryDiagnostics(baseBytes,1,9,badPolicy)),'POLICY');
% Framing errors must not authorize work or mutate source bytes.
wireCases={baseBytes(1:263),[baseBytes;uint8(0)],double(baseBytes),reshape(baseBytes,8,33)};
for wireIndex=1:numel(wireCases)
 bytes=wireCases{wireIndex};d=m600check.decodeCopterSimDeliveryDiagnostics(bytes,1,9,policy);
 check(sprintf('invalid_wire_shape_type_length_%d',wireIndex),stops(d)&&isequal(d.raw_datagram,bytes),d.status);
end
bad=baseBytes;bad(1)=bitxor(bad(1),uint8(1));d=m600check.decodeCopterSimDeliveryDiagnostics(bad,1,9,policy);
check('wrong_magic_rejected',stops(d)&&isequal(d.raw_datagram,bad),d.status);
d=m600check.decodeCopterSimDeliveryDiagnostics(baseBytes,2,9,policy);
check('wrong_copter_id_rejected',stops(d)&&isequal(d.raw_datagram,baseBytes),d.status);
% Encoder contract is type-strict; decoder independently checks numeric fields.
check('encoder_wrong_payload_shape',throws(@()m600check.encodeCopterSimDeliveryDiagnostics(v1.',ack,false)),'ENCODER');
check('encoder_single_payload',throws(@()m600check.encodeCopterSimDeliveryDiagnostics(single(v1),ack,false)),'ENCODER');
p=v1;p(26)=2;check('encoder_not_double_extend',throws(@()m600check.encodeCopterSimDeliveryDiagnostics(p,ack,false)),'ENCODER');
p=v1;p(32)=1;check('encoder_reserved_payload_nonzero',throws(@()m600check.encodeCopterSimDeliveryDiagnostics(p,ack,false)),'ENCODER');
check('encoder_numeric_pending_refused',throws(@()m600check.encodeCopterSimDeliveryDiagnostics(v1,ack,1)),'ENCODER');
a=ack;a.valid=1;check('encoder_numeric_valid_refused',throws(@()m600check.encodeCopterSimDeliveryDiagnostics(v1,a,false)),'ENCODER');
a=ack;a.task_env_failed=1;check('encoder_numeric_failed_refused',throws(@()m600check.encodeCopterSimDeliveryDiagnostics(v1,a,false)),'ENCODER');
for code=[0 16]
 a=ack;a.task_env_failed=true;a.failure_code=code;
 check(sprintf('encoder_invalid_failure_code_%d',code),throws(@()m600check.encodeCopterSimDeliveryDiagnostics(v1,a,false)),'ENCODER');
end
policyCases={rmfield(policy,'expected_session_token'),setfield(policy,'expected_session_token',0), ... %#ok<SFLD>
 setfield(policy,'payload_by_generation_kg',[2 2 1]),setfield(policy,'mass_by_generation_kg',[1 2]), ... %#ok<SFLD>
 setfield(policy,'mass_by_generation_kg',[1 2 NaN 4 5])}; %#ok<SFLD>
for policyIndex=1:numel(policyCases)
 pPolicy=policyCases{policyIndex};
 check(sprintf('invalid_policy_%d',policyIndex),throws(@()m600check.decodeCopterSimDeliveryDiagnostics(baseBytes,1,9,pPolicy)),'POLICY');
end
check('unique_case_names',numel(unique({checks.name}))==numel(checks),'DENOMINATOR');
[~,~,endian]=computer;
report=struct('schema','HOST_COPTERSIM_DELIVERY_DIAGNOSTICS_TEST_V1','passed',all([checks.passed]), ...
 'case_count',numel(checks),'cases_passed',sum([checks.passed]),'cases',checks, ...
 'host_endian',endian,'wire_endian','LITTLE','policy_fixture',policy, ...
 'fixture_is_current_task_mass_identity',false,'COM_open',0,'UDP_open',0,'board_actions',0, ...
 'model_started',false,'CopterSim_started',false,'task_completion_proven',false,'flight_admission',false, ...
 'limitations',{{'Pure packet tests do not authenticate same-core origin or prove actual mass commit.', ...
 'Stateful consumer must reject return to unbound initialization after a bound session/fault.', ...
 'Wall freshness and actual delivery/landing/rearm must be independently established.'}});
sources={'m600check.encodeCopterSimDeliveryDiagnostics','m600check.decodeCopterSimDeliveryDiagnostics', ...
 'm600check.encodeCopterSimTerrainDiagnostics','m600check.decodeCopterSimTerrainDiagnostics', ...
 'm600check.decodeCopterSimDiagnostics',mfilename};
report.source_bindings=struct('path',{},'bytes',{},'sha256',{});
for sourceIndex=1:numel(sources)
 source=which(sources{sourceIndex});info=dir(source);
 report.source_bindings(end+1)=struct('path',source,'bytes',info.bytes,'sha256',m600check.fileSha256(source)); %#ok<AGROW>
end
if strlength(outputDir)>0
 mkdir(outputDir);fid=fopen(fullfile(outputDir,'RESULT.json'),'w','n','UTF-8');assert(fid>=0);
 fileGuard=onCleanup(@()fclose(fid));fprintf(fid,'%s\n',jsonencode(report,PrettyPrint=true));clear fileGuard
end
disp(struct('passed',report.passed,'case_count',report.case_count,'cases_passed',report.cases_passed));
assert(report.passed,'m600check:DeliveryCodecTestFailed','See per-case receipt.');
 function d=decode(p),d=m600check.decodeCopterSimDeliveryDiagnostics(packet(p),1,9,policy);end
 function check(name,okay,status)
  checks(end+1)=struct('name',name,'passed',isscalar(okay)&&logical(okay),'actual_status',status); %#ok<AGROW>
 end
end
function out=packet(payload)
h=int32([1234567890;1]);d=payload(:);[~,~,endian]=computer;
if endian=='B',h=swapbytes(h);d=swapbytes(d);end
out=[reshape(typecast(h,'uint8'),[],1);reshape(typecast(d,'uint8'),[],1)];
end
function yes=sameBits(a,b),yes=isequal(typecast(a(:),'uint64'),typecast(b(:),'uint64'));end
function out=bits(hex),out=bitor(bitshift(uint64(hex2dec(hex(1:8))),32),uint64(hex2dec(hex(9:16))));end
function yes=stops(d),yes=~d.packet_valid&&d.must_stop&&~d.can_use_as_healthy_observation&& ...
 ~d.environment_extension.can_continue_task&&~d.environment_extension.mass_ack_valid;end
function yes=throws(f),yes=false;try,f();catch,yes=true;end,end
