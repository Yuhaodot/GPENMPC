function report=test_local_window_sender(outputRoot)
% Compare C and MATLAB RWW1 bytes and send them over localhost with a registration fixture.
build=string(fileparts(fileparts(mfilename('fullpath'))));
addpath(fullfile(build,'host_runtime'),fullfile(build,'matlab_validation'),fullfile(build,'m600_coptersim','matlab_validation'));
assert(~isfile(fullfile(outputRoot,'RESULT.json')));if ~isfolder(outputRoot),mkdir(outputRoot);end
diary(fullfile(outputRoot,'MATLAB_DIARY.txt'));io=[];peer=[];checks=struct('name',{},'pass',{});
try
fixture=fullfile(gpenmpc_external_path('canonical_combined_numerics'),'CONTINUOUS_REFERENCE_FIXTURE.bin');
fragments=fullfile(outputRoot,'ACTUAL_CXX_FRAGMENTS.bin');
command=sprintf('"%s" "%s" "%s"',fullfile(build,'tools','local_window_codec_probe.exe'),fixture,fragments);
[rc,log]=system(command);assert(rc==0,'test:Probe','%s',log);disp(log);
f=fopen(fixture,'rb','ieee-le');assert(f>=0);cl=onCleanup(@()fclose(f));
assert(isequal(fread(f,4,'*uint8'),uint8('RWI1').'));counts=fread(f,2,'uint32');nw=counts(1);assert(counts(2)==3765);
windows=cell(nw,1);originals=cell(nw,1);
for k=1:nw,originals{k}=fread(f,28484,'*uint8');windows{k}=readWindow(originals{k});end
clear cl
cxx=reshape(readBytes(fragments),129,244,nw);firstBytes=[];firstPackets={};
for k=1:nw
    w=windows{k};e=registered(w);[bytes,p,id]=gpenmpcNative.RflyLocalWindowCodec.encode(w,e);
    packed=zeros(129,244,'uint8');for j=1:244,packed(:,j)=[p{j}.payload;p{j}.payload_length];end
    check(sprintf('actual_CXX_original_window_%02d',k),isequal(bytes(163:28646),originals{k}) ...
        &&isequal(packed,cxx(:,:,k))&&~id.board_receipt_proven);
    if k==1,firstBytes=bytes;firstPackets=p;end
end
w=windows{1};e=registered(w);
bad=e;bad.leg_index=2;reject('wrong_leg_before_TX',@()gpenmpcNative.RflyLocalWindowCodec.encode(w,bad));
bad=e;bad.reference_asset_sha256=repmat('1',1,64);reject('wrong_reference_before_TX',@()gpenmpcNative.RflyLocalWindowCodec.encode(w,bad));

% Use an injected test clock.
t=gpenmpcNative.RflyLocalWindowTransfer(e,uint64(1000000000));r=t.begin(w);
check('prepared_without_send_or_board_ack',r.attempted_count==0&&r.first_original_submit_ns==0&&~r.board_receipt_proven);
for j=1:244
    p=t.frame();checkPacket=isequal(p,firstPackets{j});assert(checkPacket);
    b=t.attempt(uint64(1000000+j*1000));t.sent(b.original_submit_ns+uint64(10));
end
r=t.evidence();check('244_attempts_and_returns_without_clock_renewal',r.send_complete&&r.attempted_count==244 ...
    &&r.send_returned_count==244&&r.valid_until_host_ns==r.first_original_submit_ns+uint64(1000000000)&&~r.board_receipt_proven);
reject('duplicate_window_poison',@()t.begin(w));check('no_restart_after_fault',t.Failed);
t=gpenmpcNative.RflyLocalWindowTransfer(e,uint64(100));t.begin(w);t.attempt(uint64(1000));t.sent(uint64(1005));
reject('first_send_deadline_persists_across_chunks',@()t.attempt(uint64(1101)));
r=t.evidence();check('expiry_not_falsely_counted_as_write',r.attempted_count==1&&r.send_returned_count==1&&r.failed);
t=gpenmpcNative.RflyLocalWindowTransfer(e,uint64(100));t.begin(w);t.attempt(uint64(1000));
reject('actual_late_return_poison',@()t.sent(uint64(1101)));r=t.evidence();
check('late_return_retains_actual_attempt_and_return',r.attempted_count==1&&r.send_returned_count==1&&r.returned_ns(1)==1101&&~r.send_complete);
t=gpenmpcNative.RflyLocalWindowTransfer(e,uint64(100));t.begin(w);t.attempt(uint64(1000));t.fail('INJECTED_SEND_ERROR');
r=t.evidence();check('exception_keeps_outstanding_attempt',r.attempted_count==1&&r.send_returned_count==0&&r.failed&&r.outstanding_send);
reject('error_no_reopen',@()t.begin(w));

cfg=struct('local_mavlink_port',62351,'remote_mavlink_port',62352,'truth_port',62353,'coptersim_time_port',62354, ...
    'target_system',1,'target_component',1,'clock_max_rtt_s',1,'clock_sync_samples',2,'clock_sync_period_s',1,'clock_max_age_s',5, ...
    'clock_max_uncertainty_s',1,'clock_max_utc_drift_s',1,'clock_max_time_heartbeat_age_s',5,'clock_max_time_heartbeat_lag_s',5, ...
    'maximum_truth_lag_s',5,'maximum_raw_records',2000,'state_max_age_s',1,'live_enabled',true,'outer_preflight_pass',true, ...
    'canonical_exchange',struct('runtime','BOARD_LOCAL_FULL_INNER','assembly_limit_ns',uint64(5000000000), ...
    'gp_reply_host_max_age_ns',uint64(5000000000),'completed_queue_capacity',4, ...
    'local_window_host_max_age_ns',uint64(1000000000)));
cfg.local_short=struct('environment_ledger_capacity',256);
io=m600check.makeM600CopterSimIo(cfg);
peer=udpport('datagram','IPV4','LocalHost','127.0.0.1','LocalPort',62352,'Timeout',.2);
e.target_system=uint8(255);e.target_component=uint8(190);e.link_lifecycle_generation=uint64(7);
e.confirmed_host_rx_ns=gpenmpcNative.rflyOriginalHostMonotonicNs();
io.bindCanonicalSession(e);io.beginCanonicalLocalWindow(w);
chunks={};timing=tic;serviceCalls=0;
for j=1:16
    chunks{j}=io.sendCanonicalLocalWindowChunk(16,@() io.snapshot());
    if ~chunks{j}.send_complete
        % Exercise the same bounded same-owner service used between chunks.
        io.snapshot();serviceCalls=serviceCalls+1;
    end
end
elapsed=toc(timing);ev=io.evidence();allTx=ev.raw_transmit_messages;
isWindow=cellfun(@(v)isfield(v,'canonical_local_reference_window'),allTx);tx=allTx(isWindow);
check('cooperative_service_only_adds_existing_timesync', ...
    all(cellfun(@(v)v.message.MsgID==111,allTx(~isWindow))));
check('bounded_transfer_services_existing_RX_between_16_fragment_chunks',serviceCalls==15 ...
    &&all(cellfun(@(v)v.chunk_messages_send_returned<=16,chunks)));
check('actual_MATLAB_same_IO_244_no_commands',numel(tx)==244 ...
    &&all(cellfun(@(x)x.send_attempted&&x.send_returned&&x.message.MsgID==385,tx)) ...
    &&ev.canonical_local_window.send_complete&&~ev.canonical_local_window.board_receipt_proven);
d=mavlinkdialect(fullfile(build,'m600_coptersim','matlab_validation','+m600check','px4_health_events.xml'),2);
wire=cell(244,1);joined=zeros(28678,1,'uint8');timer=tic;
for j=1:244
    while peer.NumDatagramsAvailable==0&&toc(timer)<5,pause(.001);end
    assert(peer.NumDatagramsAvailable>0);row=read(peer,1,'uint8');wire{j}=uint8(row.Data(:));m=deserializemsg(d,wire{j});
    while m.MsgID==111
        assert(peer.NumDatagramsAvailable>0);row=read(peer,1,'uint8');wire{j}=uint8(row.Data(:));m=deserializemsg(d,wire{j});
    end
    p=m.Payload;
    assert(m.MsgID==385&&m.SystemID==255&&m.ComponentID==190&&p.payload_type==42002 ...
        &&p.target_system==e.source_system&&p.target_component==e.source_component ...
        &&isequal(p.payload(:),firstPackets{j}.payload)&&p.payload_length==firstPackets{j}.payload_length);
    n=double(p.payload_length)-10;joined((j-1)*118+1:(j-1)*118+n)=p.payload(11:10+n);
end
check('actual_wire_exact_CXX_bytes_order_padding',isequal(joined,firstBytes));
check('original_actual_TX_times_and_single_link',all(cellfun(@(x)x.original_host_send_return_ns>=x.original_host_submit_ns,tx)) ...
    &&chunks{end}.valid_until_host_ns==chunks{1}.first_original_submit_ns+uint64(1000000000));
wtime=chunks{end};starts=17:16:244;
check('full_chunks_service_same_IO_without_adjacent_burst', ...
    all(wtime.submitted_ns(starts)-wtime.returned_ns(starts-1)>=uint64(10000000)));
reject('same_IO_refuses_duplicate_without_resend',@()io.beginCanonicalLocalWindow(w));
after=io.evidence();check('send_failure_latched_without_extra_TX',numel(after.raw_transmit_messages)==numel(allTx)&&~isempty(after.canonical_exchange_failure));
io.close();io=[];delete(peer);peer=[];
report=struct('passed',all([checks.pass]),'checks',checks,'test_count',numel(checks),'actual_CXX_windows',nw, ...
    'actual_localhost_MAVLink_packets',244,'actual_send_elapsed_s',elapsed, ...
    'first_to_last_actual_send_s',double(chunks{end}.returned_ns(244)-chunks{1}.first_original_submit_ns)*1e-9, ...
    'hardware_actions',0,'COM',0,'model_runs',0, ...
    'scope','HOST_CODEC_AND_MATLAB_LOCALHOST');
save(fullfile(outputRoot,'RAW.mat'),'report','wire','ev','chunks','after');
f=fopen(fullfile(outputRoot,'RESULT.json'),'w');assert(f>=0);fprintf(f,'%s\n',jsonencode(report,PrettyPrint=true));fclose(f);disp(jsonencode(report));diary off
catch ex
    if ~isempty(io),io.close();end;if ~isempty(peer),delete(peer);end
    f=fopen(fullfile(outputRoot,'FAILURE.txt'),'a');fprintf(f,'%s\n',getReport(ex,'extended','hyperlinks','off'));fclose(f);diary off;rethrow(ex)
end
    function check(n,yes),checks(end+1)=struct('name',n,'pass',logical(yes));assert(yes,'test:Check','%s',n);end
    function reject(n,fn),id='';try,fn();catch ex,id=ex.identifier;end;check(n,~isempty(id));end
end
function w=readWindow(b)
p=1;w=struct();fields={'schema','capacity','leg_index','source_first_row','source_total_rows','row_count','binding_mode'};
for k=1:7,w.(fields{k})=take(4,'uint32');end
w.reference_asset_sha256=take(32,'uint8');w.window_generation=take(8,'uint64');
fields={'nominal_duration_s','total_duration_s','prefix_duration_s','relaunch_duration_s','vertical_frame_offset_ned_m'};
for k=1:5,w.(fields{k})=take(8,'double');end
w.time_s=take(256*8,'double');w.nominal_jet=reshape(take(3072*8,'double'),256,3,4);
w.prefix_coefficients=reshape(take(192*8,'double'),3,8,4,2);w.ground_jet=reshape(take(96,'double'),3,4);
w.rest_jet=reshape(take(96,'double'),3,4);w.relaunch_offset_ned_m=take(24,'double');assert(p==28485);
    function v=take(n,kind),v=typecast(b(p:p+n-1),kind);[~,~,e]=computer;if e=='B',v=swapbytes(v);end;v=v(:);p=p+n;end
end
function e=registered(w)
e=struct('uid',bitor(bitshift(uint64(808735931),32),uint64(0)),'session_generation',uint64(42), ...
    'source_system',uint8(1),'source_component',uint8(1),'leg_index',w.leg_index);
% Parse the decimal UID with uint64 arithmetic.
e.uid=uint64(0);for c='1234605616436508552',e.uid=e.uid*uint64(10)+uint64(c-'0');end
e.execution_session_sha256=wordHex(uint32(0:7)+uint32(hex2dec('11223344')));
e.task_sha256=wordHex(uint32(1:8));e.configuration_sha256=wordHex(uint32(33:40));
e.reference_asset_sha256=upper(reshape(dec2hex(w.reference_asset_sha256,2).',1,[]));
end
function s=wordHex(x),s=upper(reshape(dec2hex(x,8).',1,[]));end
function b=readBytes(p),f=fopen(p,'rb');assert(f>=0);c=onCleanup(@()fclose(f));b=fread(f,Inf,'*uint8');end
