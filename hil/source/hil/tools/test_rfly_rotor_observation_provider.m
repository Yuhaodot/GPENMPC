function report=test_rfly_rotor_observation_provider(outputRoot)
% Replay rotor datagrams with recorded model-clock and synthetic receive-time data.
build=string(fileparts(fileparts(mfilename('fullpath'))));
addpath(fullfile(build,'host_runtime'),fullfile(build,'matlab_validation'));
assert(~isfolder(outputRoot));mkdir(outputRoot);checks=struct('name',{},'pass',{});raw=struct();
fixture=fullfile(gpenmpc_external_path('rotor_observer_loopback_fixture'));
packets=reshape(readbin(fullfile(fixture,'RAW_PACKETS.bin')),128,[]);
f=fopen(fullfile(fixture,'ORIGINAL_OUTPUTS.bin'),'rb','ieee-le');assert(f>=0);fc=onCleanup(@()fclose(f));
header=fread(f,3,'*uint32');outputs=fread(f,[152,Inf],'double');clear fc
check('actual_retained_DLL_files_12_packets_and_output_header',size(packets,2)==12 ...
    &&isequal(header,uint32([hex2dec('4d36524f');1;152]))&&size(outputs,2)==282);
task=fullfile(build,'task_packages','cambridge_canonical','MU_CAMBRIDGE_MA_02__CANONICAL_PHYSICAL_TASK.mat');
taskSha='B876546B468265F84BD028AC549B06EF8ADDCB25CBA09CF389A08F4B21290A5F';
b=readbin(fullfile(gpenmpc_external_path('board_commit_exchange_fixture'),'SOURCE_1.bin'));
rx0=uint64(9000000000);s=gpenmpcNative.RflySnapshotDecoder(b,rx0);
expectedSource=struct('uid',string(s.observed_uid),'system_id',double(s.source_system), ...
    'component_id',double(s.source_component),'boot_generation',s.observed_boot_generation, ...
    'configuration_payload_sha256',hex(s.configuration_sha256),'maximum_runtime_age_ns',uint64(100000000));
windProvider=gpenmpcNative.RflyTaskWindProvider(task,taskSha,expectedSource);
e=struct('session_token',uint64(26090501), ...
    'dll_sha256',uint8(sscanf('B817CA67429871A64F2368B7D161B91F4E2E032607B1D76E73AE12D4255D33C1','%2x')), ...
    'maximum_source_age_s',.1,'maximum_receive_age_s',.1);
make=@(n)gpenmpcNative.RflyRotorObservationProvider(e,windProvider,n,uint64(100000000));
p=make(4);raw.held=cell(12,1);raw.accepted=cell(12,1);
for k=1:12
    now=rx0+uint64(k*10000000);wall=10+k*.01;
    % Frame layout is the actual probe's 60 vehicle+30 sensor+30 GPS+32 diagnostic.
    % The second lifetime begins at frame 122; accepted steps are ticks 10*k.
    before=outputs(123,122+10*(k-1));at=outputs(123,122+10*k);
    original=row(packets(:,k),now,wall);
    io=receipt({original},now,wall,before);p.ingest(io);
    [held,h]=p.current(now,wall);raw.held{k}=h;
    check("packet_"+k+"_cannot_advance_independent_clock",h.pending_count==1 ...
        &&((k==1&&~held.valid)||(k>1&&held.valid&&held.generation==k-1)));
    now2=now+uint64(1000000);wall2=wall+.001;
    p.ingest(receipt({},now2,wall2,at));[v,r]=p.current(now2,wall2);raw.accepted{k}=r;
    [parsed,why]=m600check.inspectCanonicalRotorObserverPacket(packets(:,k),e);
    check("actual_packet_"+k+"_accepted_after_separate_output_clock",isempty(why)&&v.valid ...
        &&v.generation==k&&v.rx_ns==now&&r.pending_count==0 ...
        &&isequal(v.rotor_thrust_state_n,parsed.rotor_thrust_state_n) ...
        &&abs(parsed.sim_time_s-at)<1e-12&&r.independent_model_clock.last_source_time_s==at);
end
check('original_six_state_order_task_effectiveness_no_position_truth', ...
    strcmp(v.state_source,'SAME_M600_ACCEPTED_STEP_ROTOR_LAG_STATE') ...
    &&strcmp(v.ordering,'SOFTWARE_M600_ORDER')&&strcmp(v.packet_rotor_order,'ROTORS_1_TO_6') ...
    &&isequal(v.thrust_effectiveness,windProvider.RotorEffectiveness) ...
    &&strcmpi(r.task_sha256,taskSha)&&~r.rotor_permutation_applied ...
    &&~r.runtime_origin_attested&&~r.plant_truth_position_used&&~r.publication_authority);
[wind,~]=windProvider.observe(0,b,now2,now2);
prepared=struct('schema','GPENMPC_BOARD_OUTER_RUNTIME_V1', ...
    'virtual_actuator',struct('command_source','CANONICAL_LEG_INITIAL_ROTOR_STATE', ...
    'command_generation',0,'rotor_command_n',zeros(6,1)), ...
    'outer',struct(),'wind',struct(),'task',struct());
[bound,ok,~]=gpenmpcNative.bindCanonicalRuntimeObservations(prepared,v,wind,now2,uint64(100000000));
check('actual_existing_runtime_binder_keeps_original_rx_and_lag_initial_command',ok ...
    &&bound.virtual_actuator.rx_ns==v.rx_ns ...
    &&isequal(bound.virtual_actuator.rotor_command_n,v.rotor_thrust_state_n));
oldRx=v.rx_ns;
for k=1:2000
    [pollValue,~]=p.current(now2+uint64(k),wall2+k*1e-9);
    assert(pollValue.valid&&pollValue.rx_ns==oldRx&&pollValue.generation==12);
end
check('2000_polls_do_not_renew_receive_or_generation',pollValue.rx_ns==oldRx&&p.status().accepted_count==12);
dupNs=now2+uint64(1000000);dupWall=wall2+.001;
p.ingest(receipt({row(packets(:,12),dupNs,dupWall)},dupNs,dupWall,outputs(123,242)));
[dup,~]=p.current(dupNs,dupWall);
check('exact_duplicate_does_not_renew_original_receive',dup.rx_ns==oldRx&&p.status().duplicates_ignored==1);

now=rx0;wall=10;one=row(packets(:,1),now,wall);two=row(packets(:,2),now+uint64(1),wall+1e-9);
bad=packets(:,1);bad(41)=bitxor(bad(41),uint8(1));n=make(2);
reject(n,@()n.ingest(receipt({row(bad,now,wall)},now,wall,0)), ...
    'gpenmpcNative:RotorPacket','CRC_checked_even_while_clock_missing');
check('first_bad_raw_bytes_retained',isequal(n.evidence().first_fault.record.bytes,bad));
oldFault=n.Failure;reject(n,@()n.ingest(receipt({one},now,wall,0)), ...
    'gpenmpcNative:RotorProviderFailed','valid_packet_cannot_wash_first_fault');
check('first_fault_not_replaced',n.Failure==oldFault);
for field=[17,89]
    bad=packets(:,1);bad(field)=bitxor(bad(field),uint8(1));bad=crc(bad);n=make(2);
    reject(n,@()n.ingest(receipt({row(bad,now,wall)},now,wall,0)), ...
        'gpenmpcNative:RotorPacket',"identity_"+field+"_checked_before_hold");
end
n=make(2);reject(n,@()n.ingest(receipt({two},now+uint64(1),wall+1e-9,0)), ...
    'gpenmpcNative:RotorGeneration','initial_generation_gap_is_terminal');
n=make(3);n.ingest(receipt({one},now,wall,0));
three=row(packets(:,3),now+uint64(2),wall+2e-9);
reject(n,@()n.ingest(receipt({three},now+uint64(2),wall+2e-9,0)), ...
    'gpenmpcNative:RotorGeneration','queued_generation_gap_not_delayed_until_clock');
n=make(2);n.ingest(receipt({one},now,wall,0));
bad=packets(:,1);bad(41)=bitxor(bad(41),uint8(1));bad=crc(bad);
reject(n,@()n.ingest(receipt({row(bad,now+uint64(1),wall+1e-9)},now+uint64(1),wall+1e-9,0)), ...
    'gpenmpcNative:RotorGeneration','queued_same_generation_conflict');
n=make(2);reject(n,@()n.ingest(receipt({one,two,three},now+uint64(2),wall+2e-9,0)), ...
    'gpenmpcNative:RotorQueueFull','bounded_clock_wait_never_overwrites_queue');
ev=n.evidence();check('overflow_keeps_two_pending_and_original_third_raw',numel(ev.pending)==2 ...
    &&isequal(ev.first_fault.record.bytes,packets(:,3))&&numel(ev.last_io_receipt.records)==3);
n=make(2);n.ingest(receipt({one},now,wall,0));
reject(n,@()n.current(now+uint64(100000001),wall+.100000001), ...
    'gpenmpcNative:RotorOriginalReceiveExpired','independent_clock_wait_does_not_extend_rx_age');
n=make(2);n.ingest(receipt({one},now,wall,.01));n.current(now,wall);
io=receipt({},now+uint64(1),wall+1e-9,0);
reject(n,@()n.ingest(io),'gpenmpcNative:RotorModelClockReversed','independent_clock_regression');
n=make(2);n.ingest(receipt({one},now,wall,.01));n.current(now,wall);
reject(n,@()n.current(now+uint64(100000001),wall+.100000001), ...
    'gpenmpcNative:RotorDiagnosticStale','clock_diagnostic_does_not_renew_on_poll');
n=make(2);n.ingest(receipt({one},now,wall,.2));
reject(n,@()n.current(now,wall),'gpenmpcNative:RotorDecoder','existing_decoder_original_source_age_still_applies');
n=make(2);n.ingest(receipt({one,two},now+uint64(1),wall+1e-9,.2));
try,n.current(now+uint64(1),wall+1e-9);catch,end
check('first_failed_decode_retains_exact_first_queue_record_not_last_batch_record', ...
    n.Failed&&isequal(n.evidence().first_fault.record.bytes,packets(:,1)) ...
    &&numel(n.evidence().pending)==2);
n=make(2);n.ingest(receipt({one},now,wall,0));io=receipt({},now+uint64(1),wall+1e-9,0);
io.independent_model_clock.maximum_age_s=.2;
reject(n,@()n.ingest(io),'gpenmpcNative:RotorClockBound','clock_age_cannot_be_relaxed');
n=make(2);io=receipt({one},now,wall,0);io.failure='ROTOR_RAW_MEMORY_BOUND';
reject(n,@()n.ingest(io),'gpenmpcNative:RotorIoFailure','original_IO_failure_remains_terminal');
n=make(2);n.ingest(receipt({one},now,wall,.01));n.current(now,wall);
reject(n,@()n.current(now-uint64(1),wall),'gpenmpcNative:RotorCurrentClock','actual_HOST_clock_reverse');
n=make(2);n.ingest(receipt({one},now,wall,0));
dupRow=row(packets(:,1),now+uint64(90000000),wall+.09);
n.ingest(receipt({dupRow},now+uint64(90000000),wall+.09,0));
reject(n,@()n.current(now+uint64(100000001),wall+.100000001), ...
    'gpenmpcNative:RotorOriginalReceiveExpired','queued_duplicate_cannot_refresh_original_rx');
raw.legacy_decoder=test_canonical_rotor_observer_abi();
report=struct('status','PASS_PURE_PROVIDER_WITH_ACTUAL_ARCHIVED_DLL_BYTES_AND_SEPARATE_CLOCK', ...
    'checks',checks,'test_count',numel(checks),'pass_count',sum([checks.pass]), ...
    'actual_retained_DLL_packets',12,'polls_without_renewal',2000, ...
    'input_fixture_path',fixture,'task_sha256',taskSha, ...
    'HOST_receive_and_diagnostic_age_fields_are_replay_fixtures',true, ...
    'runtime_origin_attested',false,'CopterSim_run',false,'DLL_run',false,'plant_steps',0, ...
    'COM_open',0,'socket_count',0,'board_actions',0);
report.source_sha256=sha256(readbin(which('gpenmpcNative.RflyRotorObservationProvider')));
save(fullfile(outputRoot,'RAW.mat'),'raw','report','e','outputs','packets','-v7.3');
f=fopen(fullfile(outputRoot,'RESULT.json'),'w');fc=onCleanup(@()fclose(f));fprintf(f,'%s\n',jsonencode(report,PrettyPrint=true));clear fc
disp(jsonencode(struct('status',report.status,'passed',report.pass_count,'total',report.test_count)));
    function check(name,ok)
        checks(end+1)=struct('name',name,'pass',logical(ok));
        if ~ok,save(fullfile(outputRoot,'FAILURE.mat'),'checks','raw');end
        assert(ok,'gpenmpcNative:RotorProviderTest','%s',name);
    end
    function reject(provider,fun,id,name)
        caught=false;try,fun();catch ex,caught=strcmp(ex.identifier,id);end
        check(name,caught&&provider.Failed);
    end
end
function r=row(bytes,ns,wall)
r=struct('bytes',bytes,'original_host_receive_ns',ns,'original_io_receive_s',wall, ...
    'sender_address','127.0.0.1','sender_port',53534,'receive_semantics','MATLAB_DEQUEUE_NOT_KERNEL_ARRIVAL');
end
function r=receipt(records,ns,wall,sim)
c=struct('model_ready',sim>0,'status','REPLAY_INDEPENDENT_ORIGINAL_DLL_DIAGNOSTIC', ...
    'fatal_reason','','maximum_age_s',.1,'receive_age_s',0,'source_progress_age_s',0, ...
    'last_source_time_s',sim,'progress_observed',sim>0);
r=struct('schema','RFLY_SAME_IO_RAW_ROTOR_OBSERVATIONS_V1','records',{records}, ...
    'original_poll_ns',ns,'original_io_poll_s',wall,'failure','', ...
    'independent_model_clock',c,'clock_source','OFFICIAL_COPTERSIM_MODEL_DIAGNOSTIC_NOT_ROTOR_PACKET');
end
function b=readbin(p),f=fopen(p,'rb');assert(f>=0);c=onCleanup(@()fclose(f));b=fread(f,Inf,'*uint8');end %#ok<NASGU>
function h=hex(b),h=upper(reshape(dec2hex(b,2).',1,[]));end
function h=sha256(b)
md=java.security.MessageDigest.getInstance('SHA-256');md.update(typecast(b(:),'int8'));
h=hex(typecast(md.digest(),'uint8'));
end
function b=crc(b)
v=m600check.canonicalRotorObserverCrc32(b(1:120));
for k=1:4,b(120+k)=uint8(bitand(bitshift(v,-8*(k-1)),uint32(255)));end
end
