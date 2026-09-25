function report=test_rfly_tunnel_reassembler(outputRoot)
% Test RSP1/RFC1 assembly with retained bodies and the MAVLink codec.
arguments,outputRoot (1,1) string,end
build=string(fileparts(fileparts(mfilename('fullpath'))));addpath(fullfile(build,'host_runtime'));
assert(~isfile(fullfile(outputRoot,'RESULT.json'))&&~isfile(fullfile(outputRoot,'RAW.mat')));
if ~isfolder(outputRoot),mkdir(outputRoot);end
base=fullfile(gpenmpc_external_path('board_commit_exchange_fixture'));
sources={readbin(fullfile(base,'SOURCE_1.bin')),readbin(fullfile(base,'SOURCE_2.bin')), ...
    readbin(fullfile(base,'SOURCE_3.bin'))};feedback=readbin(fullfile(base,'FEEDBACK1112.bin'));
t0=bitshift(uint64(1),53)+uint64(127);limit=uint64(1000000);
s=gpenmpcNative.RflySnapshotDecoder(sources{1},t0);
e=struct('source_system',s.source_system,'source_component',s.source_component, ...
    'target_system',uint8(255),'target_component',uint8(190),'uid',s.observed_uid, ...
    'session_generation',s.observed_boot_generation,'link_lifecycle_generation',uint64(7), ...
    'confirmed_host_rx_ns',t0-uint64(1000), ...
    'execution_session_sha256',repmat('C',1,64),'configuration_sha256',hex(s.configuration_sha256));
origin=struct('link_lifecycle_generation',e.link_lifecycle_generation,'execution_session_sha256',e.execution_session_sha256);
d=mavlinkdialect('common.xml',2);encoder=mavlinkio(d,SystemID=e.source_system,ComponentID=e.source_component);
clean=onCleanup(@()delete(encoder)); %#ok<NASGU>
checks=struct('name',{},'pass',{});caught=struct('name',{},'identifier',{});
[snapshotFrames,snapshotRaw]=packets(sources{1},3,uint64(s.subscription_generation));
f=gpenmpcNative.RflyCommittedFeedbackDecoder(feedback,t0);
[feedbackFrames,feedbackRaw]=packets(feedback,5,f.token.output_generation);
check('official_crc_decoded_13_frames',numel(snapshotFrames)==3&&numel(feedbackFrames)==10);
a=fresh();n=uint64(0);
for k=1:10
    if k<=3,n=n+uint64(10000);a.ingest(snapshotFrames{k},t0+n,origin);end
    n=n+uint64(10000);a.ingest(feedbackFrames{k},t0+n,origin);
end
rs=a.take('snapshot',t0+n);rf=a.take('feedback',t0+n);
check('actual_cpp_bodies_byte_exact',isequal(rs.message,sources{1})&&isequal(rf.message,feedback));
check('interleaved_independent_channels',isequal(a.status().messages_completed,uint64([1,1])));
check('original_first_receipt_not_completion',rs.original_host_receive_ns==t0+uint64(10000) ...
    &&rf.original_host_receive_ns==t0+uint64(20000)&&rs.completion_host_receive_ns>rs.original_host_receive_ns);
check('above_double_precision_uint64_preserved',rs.original_host_receive_ns>bitshift(uint64(1),53) ...
    &&isequal(rs.fragment_rx_ns,uint64([10000;30000;50000])+t0));
check('fragment_provenance_retained',numel(rf.decoded_fragments)==10 ...
    &&isequal(rf.decoded_fragments{4},feedbackFrames{4}));
check('no_authentication_or_consumption_overclaim',~rs.crc_verified_here&&~rs.transport_source_authenticated ...
    &&~rf.board_consumption_proven&&~rf.freshness_renewed);
check('fixed_two_slots_no_endpoints',a.status().connections==0&&a.status().maximum_completed_slots==2);
check('draining_does_not_duplicate',isempty(a.take('snapshot',t0+n))&&isempty(a.take('feedback',t0+n)));
next=gpenmpcNative.RflySnapshotDecoder(sources{2},t0);[nextFrames,~]=packets(sources{2},3,uint64(next.subscription_generation));
for k=1:3,a.ingest(nextFrames{k},t0+n+uint64(k*10000),origin);end
check('next_original_source_generation_accepted',isequal(a.take('snapshot',t0+n+uint64(30000)).message,sources{2}));
reject('first_fragment_missing',@(x)x.ingest(snapshotFrames{2},t0,origin),'gpenmpcNative:TunnelStart');
reject('invalid_source',@(x)x.ingest(field(snapshotFrames{1},'SystemID',uint8(9)),t0,origin),'gpenmpcNative:TunnelAddress');
reject('wrong_target',@(x)x.ingest(payload(snapshotFrames{1},'target_system',uint8(254)),t0,origin),'gpenmpcNative:TunnelAddress');
reject('wrong_session',@(x)x.ingest(snapshotFrames{1},t0,field(origin,'execution_session_sha256',repmat('D',1,64))), ...
    'gpenmpcNative:TunnelOrigin');
reject('link_lifetime_replaced',@(x)x.ingest(snapshotFrames{1},t0,field(origin,'link_lifecycle_generation',uint64(8))), ...
    'gpenmpcNative:TunnelOrigin');
reject('unknown_private_schema',@(x)x.ingest(byte(snapshotFrames{1},1,uint8(96)),t0,origin),'gpenmpcNative:TunnelSchema');
reject('invalid_fragment_index',@(x)x.ingest(byte(snapshotFrames{1},1,uint8(63)),t0,origin),'gpenmpcNative:TunnelIndex');
reject('partial_fragment',@(x)x.ingest(payload(snapshotFrames{1},'payload_length',uint8(127)),t0,origin),'gpenmpcNative:TunnelLength');
reject('nonzero_unused_payload',@(x)lastPadding(x),'gpenmpcNative:TunnelLength');
reject('zero_generation',@(x)x.ingest(zeroGeneration(snapshotFrames{1}),t0,origin),'gpenmpcNative:TunnelGeneration');
reject('duplicate_fragment',@(x)sequence(x,[1,1]),'gpenmpcNative:TunnelOrder');
reject('out_of_order',@(x)sequence(x,[1,3]),'gpenmpcNative:TunnelOrder');
reject('replacement_cannot_clear_partial',@(x)replacePartial(x),'gpenmpcNative:TunnelOrder');
reject('completed_replay',@(x)completedReplay(x),'gpenmpcNative:TunnelStart');
reject('bounded_ready_slot_overflow',@(x)overflow(x),'gpenmpcNative:TunnelOverflow');
reject('assembly_expiry_no_refresh',@(x)expire(x),'gpenmpcNative:TunnelAssemblyExpired');
reject('assembly_expiry_on_poll',@(x)pollExpire(x),'gpenmpcNative:TunnelAssemblyExpired');
reject('clock_rollback',@(x)clockBack(x),'gpenmpcNative:TunnelClock');
reject('double_time_rejected',@(x)x.ingest(snapshotFrames{1},double(t0),origin),'gpenmpcNative:TunnelClock');
reject('header_generation_body_mismatch',@(x)bodyGeneration(x),'gpenmpcNative:TunnelBodyBinding');
reject('body_checksum_mismatch',@(x)checksumBad(x),'gpenmpcNative:SnapshotDigest');
x=fresh();sequence(x,1);x.close();was=false;try,x.ingest(snapshotFrames{2},t0+uint64(1),origin);catch,was=true;end
check('explicit_close_no_restart',was&&x.Closed&&x.Failed);
check('closed_partial_original_bytes_retained',x.evidence().active{1}.message(1)==uint8('R') ...
    &&~x.evidence().persistence_proven);
x=fresh();x.ingest(snapshotFrames{1},t0,origin);x.ingest(snapshotFrames{2},t0+limit-uint64(1),origin);
x.ingest(snapshotFrames{3},t0+limit,origin);boundary=x.take('snapshot',t0+limit);
check('exact_assembly_boundary_accepted_not_new_t0',boundary.original_host_receive_ns==t0);
x=fresh();non=createmsg(d,'HEARTBEAT');[nm,status]=deserializemsg(d,serializemsg(encoder,non));
check('nonprivate_crc_valid',status==0);x.ingest(nm,t0,origin);
check('unrelated_messages_not_private_faults',x.status().ignored_nonprivate==1&&~x.Failed);
report=struct('scope','HOST_ONLY_EXISTING_CXX_BODY_AND_OFFICIAL_MAVLINK_CODEC_REPLAY', ...
    'checks',checks,'passed',all([checks.pass]),'checks_total',numel(checks),'negative_cases',caught, ...
    'actual_retained_cpp_snapshot_bodies',3,'actual_retained_cpp_feedback_bodies',1, ...
    'live_uart_frames',0,'live_registration_proven',false,'hardware_actions',0,'connections_opened',0, ...
    'assembly_limit_provenance','HOST_SESSION_RESOURCE_BOUND_FIXTURE_ONLY_NOT_SCIENTIFIC_OR_LIVE_ADMISSION', ...
    'source_paths',{{char(fullfile(base,'SOURCE_1.bin')),char(fullfile(base,'FEEDBACK1112.bin'))}});
save(fullfile(outputRoot,'RAW.mat'),'report','snapshotRaw','feedbackRaw','rs','rf','e','origin');
fid=fopen(fullfile(outputRoot,'RESULT.json'),'wt');assert(fid>0);c=onCleanup(@()fclose(fid));
fprintf(fid,'%s\n',jsonencode(report,PrettyPrint=true));clear c
disp(jsonencode(struct('passed',report.passed,'checks',report.checks_total,'negative_cases',numel(caught),'hardware_actions',0)));
assert(report.passed,'gpenmpcNative:TunnelTests','A tunnel test failed.');
    function check(name,pass),checks(end+1)=struct('name',name,'pass',logical(pass));end
    function x=fresh(),x=gpenmpcNative.RflyTunnelReassembler(e,limit);end
    function reject(name,fn,expected)
        x=fresh();id="";try,fn(x);catch ex,id=string(ex.identifier);end
        caught(end+1)=struct('name',name,'identifier',char(id));check(name,id==expected&&x.Failed&&x.Closed);
        original=x.Failure;try,x.ingest(snapshotFrames{1},t0+limit+uint64(9),origin);catch,end
        check([name '_permanent_first_failure'],x.Failure==original&&x.Failed);
    end
    function sequence(x,indices)
        for z=1:numel(indices),x.ingest(snapshotFrames{indices(z)},t0+uint64(z-1),origin);end
    end
    function lastPadding(x),sequence(x,[1,2]);x.ingest(byte(snapshotFrames{3},128,uint8(1)),t0+uint64(2),origin);end
    function replacePartial(x),sequence(x,1);x.ingest(nextFrames{1},t0+uint64(1),origin);end
    function completedReplay(x),sequence(x,1:3);x.take('snapshot',t0+uint64(3));x.ingest(snapshotFrames{1},t0+uint64(4),origin);end
    function overflow(x),sequence(x,1:3);x.ingest(nextFrames{1},t0+uint64(4),origin);end
    function expire(x),x.ingest(snapshotFrames{1},t0,origin);x.ingest(snapshotFrames{2},t0+limit,origin);x.ingest(snapshotFrames{3},t0+limit+uint64(1),origin);end
    function pollExpire(x),sequence(x,1);x.poll(t0+limit+uint64(1));end
    function clockBack(x),sequence(x,1);x.poll(t0-uint64(1));end
    function bodyGeneration(x)
        for z=1:3,m=snapshotFrames{z};m.Payload.payload(9)=m.Payload.payload(9)+uint8(1);x.ingest(m,t0+uint64(z),origin);end
    end
    function checksumBad(x)
        x.ingest(byte(snapshotFrames{1},20,bitxor(snapshotFrames{1}.Payload.payload(20),uint8(1))),t0,origin);
        x.ingest(snapshotFrames{2},t0+uint64(1),origin);x.ingest(snapshotFrames{3},t0+uint64(2),origin);
    end
    function [frames,raw]=packets(body,version,generation)
        count=ceil(numel(body)/119);frames=cell(count,1);raw=cell(count,1);
        for j=0:count-1
            part=[uint8(version*16+j);be(generation);body(j*119+1:min(j*119+119,numel(body)))];
            m=createmsg(d,'TUNNEL');m.Payload.target_system=e.target_system;m.Payload.target_component=e.target_component;
            m.Payload.payload_type=uint16(42002);m.Payload.payload_length=uint8(numel(part));m.Payload.payload(:)=0;
            m.Payload.payload(1:numel(part))=part;raw{j+1}=uint8(serializemsg(encoder,m));
            [frames{j+1},ok]=deserializemsg(d,raw{j+1});assert(ok==0,'CRC rejected fixture.');
        end
    end
end
function b=readbin(p),f=fopen(p,'rb');assert(f>0);c=onCleanup(@()fclose(f));b=fread(f,inf,'*uint8');end
function h=hex(b),h=upper(reshape(dec2hex(b,2).',1,[]));end
function b=be(v),[~,~,e]=computer;if e=='L',v=swapbytes(v);end;b=reshape(typecast(v,'uint8'),[],1);end
function m=field(m,k,v),m.(k)=v;end
function m=payload(m,k,v),m.Payload.(k)=v;end
function m=byte(m,k,v),m.Payload.payload(k)=v;end
function m=zeroGeneration(m),m.Payload.payload(2:9)=uint8(0);end
