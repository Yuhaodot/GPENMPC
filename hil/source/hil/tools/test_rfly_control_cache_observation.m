function report=test_rfly_control_cache_observation(outputRoot)
% Replay actual DLL packet bytes against the independently saved model trace.
% Original HOST reception/clock-map values below are explicit test fixtures.
arguments,outputRoot (1,1) string,end
build=string(fileparts(fileparts(mfilename('fullpath'))));
addpath(fullfile(build,'matlab_validation'),fullfile(build,'m600_coptersim','matlab_validation'));
assert(~isfolder(outputRoot));mkdir(outputRoot);
base=fullfile(gpenmpc_external_path('control_cache_dll'));
bytes=readbin(fullfile(base,'CACHE216.bin'));packets=reshape(bytes,216,[]);
trace=readtable(fullfile(base,'TICKS.csv'));
expected=struct('session_token',uint64(26090501), ...
    'dll_sha256',uint8(sscanf('D155F1E4A1824B4D10EAB0860FC8946AF616874421FE1F9FB1F1BE05A54285BB','%2x')), ...
    'maximum_source_age_s',.1,'maximum_receive_age_s',.1);
checks=struct('name',{},'pass',{});state=[];samples=cell(12,1);
for generation=1:12
    accepted=find(trace.tick==generation*10,1);prior=find(trace.tick==generation*10-1,1);
    clock=struct('now_ns',uint64(100000000000)+uint64(generation*10000000), ...
        'now_wall_time_s',100+generation*.01,'original_host_receive_ns',uint64(100000000000)+uint64(generation*10000000), ...
        'receive_wall_time_s',100+generation*.01,'now_sim_time_s',trace.core_time(prior));
    [held,state]=m600check.decodeCanonicalControlCache(packets(:,generation),expected,clock,state);
    check(sprintf('independent_clock_lag_holds_%d',generation),~held.valid&&~state.failed ...
        &&strcmp(held.reason,'CACHE_WAITING_FOR_INDEPENDENT_MODEL_CLOCK'));
    clock.now_sim_time_s=trace.core_time(accepted);clock.now_ns=clock.now_ns+uint64(1000);clock.now_wall_time_s=clock.now_wall_time_s+1e-6;
    [samples{generation},state]=m600check.decodeCanonicalControlCache([],expected,clock,state);
    sample=samples{generation};
    check(sprintf('actual_accepted_input_bits_%d',generation),sample.valid&&sample.generation==uint64(generation) ...
        &&sample.input_call_count==uint64(10*generation+1)&&sample.sim_time_s==trace.core_time(accepted) ...
        &&isequal(typecast(sample.input16,'uint8'),packets(49:176,generation)));
end
check('actual_four_zero_accepted_steps_only',isequal(find(cellfun(@(s)s.all16_zero,samples)).',[7 8 9 10]) ...
    &&~samples{6}.all16_zero&&~samples{11}.all16_zero);
original=samples{12}.original_host_receive_ns;ioRx=samples{12}.original_io_receive_s;
for poll=1:2000,[steady,state]=m600check.decodeCanonicalControlCache([],expected,clock,state);end
check('2000_polls_do_not_renew_or_forge_zero',steady.valid&&steady.original_host_receive_ns==original ...
    &&steady.original_io_receive_s==ioRx&&~steady.all16_zero&&steady.generation==12);
clock.now_ns=clock.now_ns+uint64(100000);clock.now_wall_time_s=clock.now_wall_time_s+.0001;
clock.original_host_receive_ns=clock.now_ns;clock.receive_wall_time_s=clock.now_wall_time_s;
[duplicate,state]=m600check.decodeCanonicalControlCache(packets(:,12),expected,clock,state);
check('duplicate_original_rx_retained',duplicate.valid&&duplicate.duplicate_ignored&&duplicate.original_host_receive_ns==original);
clock.now_ns=clock.now_ns+uint64(200000000);clock.now_wall_time_s=clock.now_wall_time_s+.2;
[expired,state]=m600check.decodeCanonicalControlCache([],expected,clock,state);
check('original_receive_expiry_permanent',~expired.valid&&state.failed&&contains(state.failure_reason,'CacheReceiveStale'));
firstClock=struct('now_ns',uint64(1000000000),'now_wall_time_s',1,'original_host_receive_ns',uint64(1000000000), ...
    'receive_wall_time_s',1,'now_sim_time_s',trace.core_time(find(trace.tick==10,1)));
[first,firstState]=m600check.decodeCanonicalControlCache(packets(:,1),expected,firstClock,[]);
check('fresh_actual_first_packet',first.valid&&~firstState.failed);
bad=packets(:,1);bad(55)=bitxor(bad(55),uint8(1));badCase(bad,expected,firstClock,[],'crc');
bad=packets(:,1);bad(1)=0;badCase(fixcrc(bad),expected,firstClock,[],'magic');
bad=packets(:,1);bad(9)=2;badCase(fixcrc(bad),expected,firstClock,[],'version');
bad=packets(:,1);bad(11)=0;badCase(fixcrc(bad),expected,firstClock,[],'not_accepted');
bad=packets(:,1);bad(213)=1;badCase(bad,expected,firstClock,[],'reserved');
wrong=expected;wrong.session_token=wrong.session_token+uint64(1);badCase(packets(:,1),wrong,firstClock,[],'wrong_session');
wrong=expected;wrong.dll_sha256(1)=bitxor(wrong.dll_sha256(1),uint8(1));badCase(packets(:,1),wrong,firstClock,[],'wrong_dll');
bad=packets(:,1);bad(41:48)=0;badCase(fixcrc(bad),expected,firstClock,[],'zero_input_call');
bad=packets(:,1);bad(49:56)=le(NaN);badCase(fixcrc(bad),expected,firstClock,[],'nonfinite_input');
bad=packets(:,1);bad(49:56)=le(1);badCase(fixcrc(bad),expected,firstClock,firstState,'conflicting_same_generation');
badCase(packets(:,3),expected,firstClock,firstState,'generation_gap');
bad=packets(:,2);bad(41:48)=le(uint64(1));badCase(fixcrc(bad),expected,firstClock,firstState,'input_call_reversed');
wrongClock=firstClock;wrongClock.now_ns=wrongClock.now_ns-uint64(1);badCase([],expected,wrongClock,firstState,'original_host_reversed');
wrongClock=firstClock;wrongClock.now_sim_time_s=0;badCase([],expected,wrongClock,firstState,'model_clock_reversed');
missingClock=firstClock;missingClock.now_sim_time_s=NaN;
[waiting,waitingState]=m600check.decodeCanonicalControlCache(packets(:,1),expected,missingClock,[]);
check('missing_independent_clock_not_packet_time',~waiting.valid&&~waitingState.failed);
[ready,waitingState]=m600check.decodeCanonicalControlCache([],expected,firstClock,waitingState);
check('actual_clock_catches_up_without_rx_renewal',ready.valid&&ready.original_host_receive_ns==firstClock.original_host_receive_ns);
report=struct('passed',all([checks.pass]),'checks',checks,'actual_dll_packets',12, ...
    'source_cache_bytes',fullfile(base,'CACHE216.bin'),'independent_model_clock_source',fullfile(base,'TICKS.csv'), ...
    'scope','ACTUAL_DLL_ORIGINAL_BYTES_REPLAY_WITH_EXPLICIT_SYNTHETIC_HOST_RX_TIMES', ...
    'model_runs',0,'board_actions',0,'com_opens',0,'live_runtime_origin_attested',false);
save(fullfile(outputRoot,'RAW.mat'),'report','packets','trace','expected','samples','state');
f=fopen(fullfile(outputRoot,'RESULT.json'),'w');assert(f>0);c=onCleanup(@()fclose(f));fprintf(f,'%s\n',jsonencode(report,PrettyPrint=true));clear c
disp(jsonencode(struct('passed',report.passed,'checks',numel(checks))));assert(report.passed);
    function check(name,yes)
        checks(end+1)=struct('name',name,'pass',logical(yes));
        if ~yes,save(fullfile(outputRoot,'FAILED_STATE.mat'),'checks','state','clock','expired');disp(state);end
        assert(yes,'m600check:CacheTest','%s',name);
    end
    function badCase(b,e,c,s,name)
        [v,failed]=m600check.decodeCanonicalControlCache(b,e,c,s);
        check(name,~v.valid&&failed.failed&&~isempty(failed.failure_reason));
        [~,again]=m600check.decodeCanonicalControlCache(packets(:,1),expected,firstClock,failed);
        check([name '_first_fault_retained'],strcmp(failed.failure_reason,again.failure_reason));
    end
end
function b=readbin(p),f=fopen(p,'rb');assert(f>0);c=onCleanup(@()fclose(f));b=fread(f,inf,'*uint8');end
function b=le(v),[~,~,e]=computer;if e=='B',v=swapbytes(v);end;b=reshape(typecast(v(:),'uint8'),[],1);end
function b=fixcrc(b),b(209:212)=le(m600check.canonicalRotorObserverCrc32(b(1:208)));end
