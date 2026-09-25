function report=test_local_decoder_exact_cache(outputRoot)
% Test decoder caching with retained bytes.
build=string(fileparts(fileparts(mfilename('fullpath'))));
addpath(fullfile(build,'host_runtime'));
assert(~isfolder(outputRoot));mkdir(outputRoot);
checks=struct('name',{},'pass',{});raw=struct();
rs=fullfile(build,'rfly_vendor_integration','full_inner_abi','snapshot_wire_fixture','RLS1_SNAPSHOTS.bin');
rc=fullfile(gpenmpc_external_path('committed_state_diagnostics'),'COMMITTED_RLC2.bin');
snap=reshape(readbin(rs),382,[]);committed=reshape(readbin(rc),1494,[]);
rx=bitshift(uint64(1),53)+uint64(1);
old={@RflyLocalSnapshotDecoderReference,@RflyLocalCommittedDecoderReference};
fresh={@gpenmpcNative.RflyLocalSnapshotDecoder,@gpenmpcNative.RflyLocalCommittedDecoder};
data={snap,committed};labels={'RLS1','RLC2'};
for family=1:2
 a=data{family};reference=old{family};candidate=fresh{family};
 equal=true;repeated=true;
 for k=1:size(a,2)
  q=rx+uint64(k);expected=reference(a(:,k),q);actual=candidate(a(:,k),q);
  equal=equal&&bitsame(expected,actual);repeated=repeated&&bitsame(expected,candidate(a(:,k),q));
 end
 check([labels{family} '_all_retained_first_parse_bit_exact'],equal);
 check([labels{family} '_all_retained_repeat_bit_exact'],repeated);
 b=a(:,1);expected=reference(b,rx);candidate(b,rx);
 check([labels{family} '_type_length_rx_rejected_after_cache'], ...
  reject(@()candidate(double(b),rx))&&reject(@()candidate(b(1:end-1),rx)) ...
  &&reject(@()candidate(b,double(rx)))&&reject(@()candidate(b,uint64(0))));
 bad=b;bad(1)=bitxor(bad(1),uint8(1));
 check([labels{family} '_magic_change_rejected'],reject(@()candidate(bad,rx)));
 bad=b;bad(25)=bitxor(bad(25),uint8(1));
 check([labels{family} '_body_change_same_rx_checksum_rejected'],reject(@()candidate(bad,rx)));
 bad=b;
 if family==1,bad(349)=uint8(2);else,bad(643:650)=be(2.0);end
 bad=seal(bad);
 check([labels{family} '_valid_digest_invalid_fields_rejected'],reject(@()candidate(bad,rx)));
 check([labels{family} '_failure_did_not_poison_good_cache'],bitsame(expected,candidate(b,rx)));
 first=candidate(b,rx);second=candidate(b,rx+uint64(1));
 check([labels{family} '_uint64_rx_above_double_precision_not_aliased'], ...
  first.original_host_receive_ns==rx&&second.original_host_receive_ns==rx+uint64(1) ...
  &&bitsame(second,reference(b,rx+uint64(1))));
 changed=candidate(b,rx);changed.identity.uid=uint64(1);changed.original_bytes(:)=0;
 if family==1,changed.canonical_state13(:)=42;else,changed.state64(:)=42;changed.closed_gp_evidence.trust=99;end
 check([labels{family} '_caller_nested_value_mutation_cannot_mutate_cache'],bitsame(expected,candidate(b,rx)));
 candidate(a(:,end),rx);again=candidate(b,rx);
 check([labels{family} '_A_B_A_same_single_slot_result'],bitsame(expected,again));
 % Same call pattern: each newly observed message is consumed four times.
 % First parse is counted too, not just the three cache hits.
 calls=24;slow=zeros(calls,1);fast=slow;
 for k=1:4
  for j=1:4,reference(a(:,1),rx);candidate(a(:,1),rx);end
 end
 for k=1:calls
  idx=mod(k-1,size(a,2))+1;q=rx+uint64(k);
  t=tic;for j=1:4,unused=reference(a(:,idx),q);end;slow(k)=toc(t); %#ok<NASGU>
  t=tic;for j=1:4,unused=candidate(a(:,idx),q);end;fast(k)=toc(t); %#ok<NASGU>
 end
 raw.(labels{family})=struct('retained_count',size(a,2),'four_decode_reference_s',slow,'four_decode_candidate_s',fast);
end
% Preserve optional NaN payloads and signed zero in the format mutation.
b=committed(:,1);b(99:106)=uint8([127;248;0;0;0;0;0;7]);b(107:114)=uint8([128;0;0;0;0;0;0;0]);b=seal(b);
o=old{2}(b,rx);n=fresh{2}(b,rx);check('RLC2_NaN_payload_and_negative_zero_exact',bitsame(o,n)&&bitsame(n,fresh{2}(b,rx)));
legacy=[uint8('RLC1').';b(5:1366);zeros(32,1,'uint8')];legacy=seal(legacy);
lo=old{2}(legacy,rx);ln=fresh{2}(legacy,rx);
check('RLC1_legacy_format_is_separate_cache_key',bitsame(lo,ln)&&~ln.learning_audit_available&&numel(ln.original_bytes)==1398);
check('RLC2_after_RLC1_not_mixed',bitsame(n,fresh{2}(b,rx)));
invalid=b;invalid(1391:1398)=be(2.0);invalid=seal(invalid);
check('learning_invalid_with_valid_digest_rejected_after_cache',reject(@()fresh{2}(invalid,rx)));
report=struct('passed',all([checks.pass]),'checks',checks,'total',numel(checks),'pass_count',sum([checks.pass]), ...
 'scope','PURE_RETAINED_DECODER_SAME_BYTES_AND_EXACT_RX_CACHE_NO_ADMISSION', ...
 'snapshot_rows',size(snap,2),'committed_rows',size(committed,2),'cache_capacity_each',1, ...
 'rx_above_2pow53',char(string(rx)),'COM',0,'board',0,'IO',0,'model',0,'solver',0, ...
 'reference_snapshot',sha(rs),'reference_committed',sha(rc));
report.reference_source_sha256={sha(string(which('RflyLocalSnapshotDecoderReference'))),sha(string(which('RflyLocalCommittedDecoderReference')))};
report.current_source_sha256={sha(string(which('gpenmpcNative.RflyLocalSnapshotDecoder'))),sha(string(which('gpenmpcNative.RflyLocalCommittedDecoder')))};
for family=1:2
 v=raw.(labels{family});report.([labels{family} '_four_reference_median_s'])=median(v.four_decode_reference_s);
 report.([labels{family} '_four_candidate_median_s'])=median(v.four_decode_candidate_s);
end
save(fullfile(outputRoot,'RAW.mat'),'raw','checks','rx');
f=fopen(fullfile(outputRoot,'RESULT.json'),'w');assert(f>=0);g=onCleanup(@()fclose(f)); %#ok<NASGU>
fprintf(f,'%s\n',jsonencode(report,PrettyPrint=true));disp(jsonencode(report));assert(report.passed);
 function check(name,ok),checks(end+1)=struct('name',name,'pass',logical(ok));end
end
function b=readbin(p),f=fopen(p,'rb');assert(f>=0);g=onCleanup(@()fclose(f));b=fread(f,Inf,'*uint8');end %#ok<NASGU>
function h=sha(p),b=readbin(p);h=upper(reshape(dec2hex(digest(b),2).',1,[]));end
function h=digest(b),m=java.security.MessageDigest.getInstance('SHA-256');m.update(typecast(b(:),'int8'));h=reshape(typecast(m.digest(),'uint8'),[],1);end
function b=seal(b),b(end-31:end)=digest(b(1:end-32));end
function b=be(v),[~,~,e]=computer;if e=='L',v=swapbytes(v);end;b=reshape(typecast(v(:),'uint8'),[],1);end
function ok=reject(f),ok=false;try,unused=f();catch,ok=true;end;end %#ok<NASGU>
function ok=bitsame(a,b)
ok=strcmp(class(a),class(b))&&isequal(size(a),size(b));if ~ok,return;end
if isstruct(a)
 names=fieldnames(a);ok=isequal(names,fieldnames(b));if ~ok,return;end
 for k=1:numel(a),for j=1:numel(names),ok=ok&&bitsame(a(k).(names{j}),b(k).(names{j}));end;end
elseif isa(a,'double')||isa(a,'single')
 ok=isequal(typecast(a(:),'uint8'),typecast(b(:),'uint8'));
else,ok=isequaln(a,b);
end
end
