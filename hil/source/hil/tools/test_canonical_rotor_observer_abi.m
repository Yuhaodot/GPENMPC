function result=test_canonical_rotor_observer_abi()
% Test rotor-observer ABI helpers.
root=fileparts(fileparts(mfilename('fullpath')));oldPath=path;
guard=onCleanup(@()path(oldPath)); %#ok<NASGU>
addpath(fullfile(root,'matlab_validation'),'-begin');
names={};checks=false(0,1);
e=struct('session_token',uint64(73),'dll_sha256',uint8((1:32).'), ...
    'maximum_source_age_s',.1,'maximum_receive_age_s',.1);
rotor=[0;1.25;2.5;10;20;32.145727009134916];
enc=@(r,t,g)m600check.encodeCanonicalRotorObserver(r,t,uint64(g),e.session_token,e.dll_sha256);
decode=@(b,c,s)m600check.decodeCanonicalRotorObserver(b,e,c,s);
b=enc(rotor,.01,1);c=clock(.01,1,1);
[r,s]=decode(b,c,[]);
check('roundtrip_exact_six_lag_states',r.valid&&r.accepted_new_sample&&isequal(r.rotor_thrust_state_n,rotor));
check('source_not_position_truth',~r.position_truth_present&&~r.controller_or_estimator_position_feed_permitted);
check('binding_not_runtime_attestation',~r.runtime_origin_attested&&isequal(r.dll_sha256,e.dll_sha256));
check('explicit_new_abi_128_bytes',numel(b)==128&&isequal(b(1:8),uint8('M6ROTOR1').'));
check('known_ieee_crc32_vector',m600check.canonicalRotorObserverCrc32(uint8('123456789'))==uint32(hex2dec('CBF43926')));
check('little_endian_session_and_generation',b(17)==73&&b(25)==1&&all(b(18:24)==0));
b2=enc(rotor+.1,.02,2);[r,s2]=decode(b2,clock(.02,1.01,1.01),s);
check('next_accepted_step',r.valid&&r.generation==2&&r.accepted_new_sample);
[r,s3]=decode(b2,clock(.03,1.02,1.02),s2);
check('duplicate_keeps_original_rx',r.valid&&r.duplicate_ignored&&~r.accepted_new_sample&&s3.receive_wall_time_s==1.01);
[r,s4]=decode(uint8([]),clock(.04,1.03,1.03),s3);
check('poll_does_not_renew_generation_or_rx',r.valid&&~r.accepted_new_sample&&s4.generation==2&&s4.receive_wall_time_s==1.01);
[r,~]=decode(uint8([]),clock(.11,1.10,1.10),s);
check('age_exact_boundary_accepted',r.valid);
[r,~]=decode(uint8([]),clock(.110001,1.05,1.05),s);
check('source_stale_even_recent_receive',~r.valid&&strcmp(r.reason,'SOURCE_STALE'));
[r,~]=decode(uint8([]),clock(.02,1.100001,1.100001),s);
check('receive_stale_even_fresh_source',~r.valid&&strcmp(r.reason,'RECEIVE_STALE'));
[r,stale]=decode(b,clock(.02,1.100001,1.100001),s);
check('duplicate_cannot_refresh_stale_sample',~r.valid&&strcmp(r.reason,'RECEIVE_STALE'));
[r,~]=decode(b2,clock(.02,1.11,1.11),stale);
check('failure_latched_no_packet_recovery',~r.valid&&strcmp(r.reason,'RECEIVE_STALE'));
bad=enc(rotor+1,.01,1);reject('same_generation_conflict',bad,c,s,'GENERATION_CONFLICT');
reject('generation_reverse',b,clock(.03,1.02,1.02),s2,'GENERATION_REVERSED');
bad=enc(rotor,.03,3);[r,gap]=decode(bad,clock(.03,1.02,1.02),s);
check('generation_gap_accounted_fail_closed',~r.valid&&strcmp(r.reason,'GENERATION_GAP')&&gap.missing_generations==1);
reject('initial_sequence_must_start_one',b2,clock(.02,1.01,1.01),[],'INITIAL_GENERATION_NOT_ONE');
reject('source_time_not_advancing',enc(rotor,.01,2),clock(.02,1.01,1.01),s,'SOURCE_TIME_NOT_ADVANCING');
reject('source_from_future',b,clock(0,1,1),[],'SOURCE_FROM_FUTURE');
reject('host_clock_reverse',b2,clock(.02,.99,.99),s,'HOST_CLOCK_REVERSED');
reject('model_clock_reverse',b2,clock(0,1.01,1.01),s,'MODEL_CLOCK_REVERSED');
reject('receive_clock_reverse',b2,clock(.02,1.01,.99),s,'RECEIVE_CLOCK_REVERSED');
reject('receive_future',b,clock(.01,1,1.01),[],'RECEIVE_FROM_FUTURE');
reject('truncated_packet',b(1:end-1),c,[],'PACKET_SHAPE');
bad=b;bad(1)=0;reject('magic',bad,c,[],'MAGIC');
bad=b;bad(9)=2;reject('version',bad,c,[],'VERSION');
bad=b;bad(11)=0;reject('only_accepted_core_step',bad,c,[],'NOT_ACCEPTED_CORE_STEP');
bad=b;bad(13)=127;reject('declared_length',bad,c,[],'DECLARED_SIZE');
bad=b;bad(125)=1;reject('reserved_nonzero',bad,c,[],'RESERVED_NONZERO');
bad=b;bad(41)=bitxor(bad(41),uint8(1));reject('payload_crc',bad,c,[],'CRC');
bad=b;bad(17)=74;bad=crc(bad);reject('wrong_session',bad,c,[],'SESSION_MISMATCH');
bad=b;bad(89)=99;bad=crc(bad);reject('wrong_dll_binding',bad,c,[],'DLL_IDENTITY_MISMATCH');
bad=b;bad(25:32)=0;bad=crc(bad);reject('zero_generation',bad,c,[],'ZERO_GENERATION');
for value=[NaN,Inf,-1]
    bad=doubleField(b,33,value);reject(sprintf('bad_source_%g',value),bad,c,[],'SOURCE_TIME_INVALID');
    bad=doubleField(b,41,value);reject(sprintf('bad_rotor_%g',value),bad,c,[],'ROTOR_STATE_INVALID');
end
throws('encoder_rejects_nan',@()enc([NaN;rotor(2:end)],.01,1));
throws('encoder_rejects_negative',@()enc([-1;rotor(2:end)],.01,1));
throws('encoder_rejects_animation_shape',@()enc(zeros(8,1),.01,1));
throws('encoder_rejects_nonfinite_time',@()enc(rotor,Inf,1));
throws('encoder_rejects_zero_generation',@()enc(rotor,.01,0));
[r,~]=decode(uint8([]),c,[]);check('no_sample_not_admitted',~r.valid&&strcmp(r.reason,'NO_OBSERVER_SAMPLE'));
e2=e;e2.session_token=uint64(74);new=m600check.encodeCanonicalRotorObserver(rotor,0,uint64(1),e2.session_token,e2.dll_sha256);
[r,~]=m600check.decodeCanonicalRotorObserver(uint8([]),e2,c,s);
check('poll_cannot_relabel_session_identity',~r.valid&&strcmp(r.reason,'EXPECTED_CONTEXT_CHANGED'));
[r,~]=m600check.decodeCanonicalRotorObserver(b,e2,c,s);
check('packet_cannot_replace_bound_context',~r.valid&&strcmp(r.reason,'EXPECTED_CONTEXT_CHANGED'));
loose=e;loose.maximum_receive_age_s=1;
[r,~]=m600check.decodeCanonicalRotorObserver(uint8([]),loose,c,s);
check('cannot_relax_receive_age_in_session',~r.valid&&strcmp(r.reason,'EXPECTED_CONTEXT_CHANGED'));
[r,~]=m600check.decodeCanonicalRotorObserver(new,e2,clock(0,2,2),[]);
check('explicit_new_session_context',r.valid&&r.session_token==74);
badClock=c;badClock.now_sim_time_s=NaN;reject('nonfinite_clock',b,badClock,[],'INVALID_EXPECTED_OR_CLOCK');
large=bitshift(uint64(1),63)+uint64(31);e2.session_token=large;
big=m600check.encodeCanonicalRotorObserver(rotor,.01,uint64(1),large,e2.dll_sha256);
[r,~]=m600check.decodeCanonicalRotorObserver(big,e2,c,[]);
check('uint64_token_preserved_without_double_rounding',r.valid&&r.session_token==large);
result=struct('status','PASS_PURE_ROTOR_OBSERVER_ABI_ONLY','passed',sum(checks),'total',numel(checks), ...
    'tests',{names},'checks',checks);
disp(jsonencode(result));assert(all(checks));
    function check(name,condition)
        names{end+1}=name;checks(end+1,1)=logical(condition);
        assert(condition,'m600check:RotorObserverTest','%s',name);
    end
    function reject(name,packet,clk,previous,reason)
        [out,st]=decode(packet,clk,previous);
        check(name,~out.valid&&st.failed&&strcmp(out.reason,reason)&&all(isnan(out.rotor_thrust_state_n)));
    end
    function throws(name,fun)
        caught=false;try,fun();catch,caught=true;end;check(name,caught);
    end
end
function c=clock(sim,now,rx)
c=struct('now_sim_time_s',sim,'now_wall_time_s',now,'receive_wall_time_s',rx);
end
function b=crc(b)
v=m600check.canonicalRotorObserverCrc32(b(1:120));
for k=1:4,b(120+k)=uint8(bitand(bitshift(v,-8*(k-1)),uint32(255)));end
end
function b=doubleField(b,start,value)
v=typecast(double(value),'uint64');
for k=1:8,b(start+k-1)=uint8(bitand(bitshift(v,-8*(k-1)),uint64(255)));end
b=crc(b);
end
