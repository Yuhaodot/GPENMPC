function receipt=test_matlab_context_wire(outputRoot,input,solverFixture)
% Test context serialization with explicit wire and solver-payload fixtures.
arguments
    outputRoot (1,1) string
    input (1,1) string
    solverFixture (1,1) string
end
root=string(fileparts(mfilename('fullpath')));build=string(fileparts(fileparts(root)));
assert(~isfolder(outputRoot),'Preserve previous test receipt.');
addpath(fullfile(build,'host_runtime'),'-begin');
assert(isfile(input)&&isfile(solverFixture),'gpenmpcNative:ContextFixture', ...
    'Supply existing context-wire and solver-payload fixture files.');
fi=fopen(input,'rb','ieee-be');assert(fi>=0);cleanup=onCleanup(@()fclose(fi));
snapshotBytes=fread(fi,246,'*uint8');contextBytes=fread(fi,316,'*uint8');
dialect=mavlinkdialect('common.xml',2);reassembled=uint8([]);downBytes=0;
for k=0:2
    n=fread(fi,1,'*uint16');frame=fread(fi,double(n),'*uint8').';
    [decoded,status]=deserializemsg(dialect,frame,OutputAllMessage=true);
    assert(numel(decoded)==1&&status==0);payload=decoded.Payload;
    assert(payload.payload_type==42002&&payload.payload(1)==48+k);
    reassembled=[reassembled;reshape(payload.payload(10:double(payload.payload_length)),[],1)]; %#ok<AGROW>
    downBytes=downBytes+double(n);
end
assert(isequal(reassembled,snapshotBytes));
s=gpenmpcNative.RflySnapshotDecoder(reassembled,uint64(9000000000));
assert(s.observed_uid==bitor(bitshift(uint64(hex2dec('11223344')),32),uint64(hex2dec('55667788')))&&s.observed_boot_generation==42 ...
    &&s.original_sample_hrt_us==1000000&&s.original_board_receipt_hrt_us==1000300);
% Explicit codec fixture source events remain clearly separate from a live
% CanonicalOuterCoordinator response/clock-association provider.
c=struct('configuration_sha256',contextBytes(5:36),'reference_generation',v(37:44,'uint64'), ...
    'outer_generation',v(45:52,'uint64'),'reference_source_ticket',contextBytes(53:84), ...
    'outer_source_ticket',contextBytes(85:116),'reference_source_receipt_ns',v(117:124,'uint64'), ...
    'reference_creation_ns',v(125:132,'uint64'),'reference_expiry_ns',v(133:140,'uint64'), ...
    'outer_source_receipt_ns',v(141:148,'uint64'),'outer_creation_ns',v(149:156,'uint64'), ...
    'outer_expiry_ns',v(157:164,'uint64'),'reference_ned',v(165:252,'double'), ...
    'outer_payload',v(253:284,'double'),'target_system',uint8(1),'target_component',uint8(1));
assert(isequal(c.reference_source_ticket,s.ticket)&&isequal(c.outer_source_ticket,s.ticket));
serializer=mavlinkio(dialect,SystemID=42,ComponentID=191,ComponentType='MAV_TYPE_GCS',AutopilotType='MAV_AUTOPILOT_INVALID');
[packets,r,encoded]=gpenmpcNative.RflyContextEncoder(c,serializer,dialect);assert(isequal(encoded,contextBytes));
archived=load(solverFixture,'raw');
[actualPayload,payloadReceipt]=gpenmpcNative.RflyOuterPayload(archived.raw.first);
assert(isequal(actualPayload,[archived.raw.first.phase_acceleration_s_inv; ...
    archived.raw.first.outer_acceleration_correction_f_mps2(:)]));
actualContext=c;actualContext.outer_payload=actualPayload;
[~,~,actualEncoded]=gpenmpcNative.RflyContextEncoder(actualContext,serializer,dialect);
assert(isequal(actualEncoded(253:284),bigEndian(actualPayload)));
mkdir(outputRoot);fo=fopen(fullfile(outputRoot,'MATLAB_CONTEXT_PACKETS.bin'),'wb','ieee-be');assert(fo>=0);
for k=1:3,fwrite(fo,uint16(numel(packets{k})),'uint16');fwrite(fo,packets{k},'uint8');end;fclose(fo);
negative=0;
for mode=1:5
    bad=c;rejected=false;
    if mode==1,bad.reference_creation_ns=double(bad.reference_creation_ns);end
    if mode==2,bad.outer_expiry_ns=bad.outer_creation_ns-uint64(1);end
    if mode==3,bad.reference_source_ticket(:)=0;end
    if mode==4,bad.outer_payload(2)=NaN;end
    try
        if mode==5,b=snapshotBytes;b(100)=bitxor(b(100),uint8(1));gpenmpcNative.RflySnapshotDecoder(b,uint64(9000000000));
        else,gpenmpcNative.RflyContextEncoder(bad,serializer,dialect);end
    catch ex,rejected=startsWith(string(ex.identifier),'gpenmpcNative:');end
    assert(rejected);negative=negative+1;
end
receipt=struct('status','PASS_MATLAB_CONTEXT_AND_PRIVATE_SNAPSHOT_FUNCTIONS', ...
    'context316_bit_exact',true,'private_snapshot246_bit_exact',true,'negative_cases',negative, ...
    'actual_snapshot_packets',3,'actual_context_packets',3,'snapshot_wire_bytes',downBytes, ...
    'context_wire_bytes',r.serialized_bytes,'live_time_domain_binding',false,'coordinator_connected',false,'board_access',0);
receipt.actual_archived_solver_payload4_verbatim=true;receipt.outer_payload_scope=payloadReceipt.scope;
fj=fopen(fullfile(outputRoot,'MATLAB_CONTEXT_RESULT.json'),'w');fwrite(fj,jsonencode(receipt,PrettyPrint=true),'char');fclose(fj);disp(receipt);
    function x=v(indices,kind)
        x=typecast(contextBytes(indices),kind);[~,~,endian]=computer;if endian=='L',x=swapbytes(x);end;x=x(:);
    end
    function bytes=bigEndian(x)
        [~,~,endian]=computer;if endian=='L',x=swapbytes(x);end;bytes=reshape(typecast(x(:),'uint8'),[],1);
    end
end
