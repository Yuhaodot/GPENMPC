function receipt=test_matlab_committed_feedback(outputRoot,input,fixtureKind)
% Decode explicit committed-feedback packets using the selected fixture layout.
arguments
    outputRoot (1,1) string
    input (1,1) string
    fixtureKind (1,1) string {mustBeMember(fixtureKind,["PRODUCTION_IO","HEADER_WIRE"])} = "PRODUCTION_IO"
end
root=string(fileparts(mfilename('fullpath')));build=string(fileparts(fileparts(root)));
assert(~isfolder(outputRoot),'Preserve previous receipt');addpath(fullfile(build,'host_runtime'),'-begin');
assert(isfile(input),'gpenmpcNative:FeedbackFixture','Supply an existing committed-feedback fixture file.');
rows=76;scope='HEADER_WIRE_TEST_EXPLICIT_MOCK_COMMIT_NOT_REAL_IO_OR_BOARD';
if fixtureKind=="PRODUCTION_IO",rows=2;scope='ACTUAL_PRODUCTION_IO_CPP_AND_PUMP_CPP_WITH_EXPLICIT_SIMULATED_UORB_HRT_AUTHORITY';end
fi=fopen(input,'rb','ieee-be');assert(fi>=0);cleanup=onCleanup(@()fclose(fi));
assert(isequal(fread(fi,4,'*uint8'),uint8('RFC5').')&&fread(fi,1,'*uint32')==rows);
dialect=mavlinkdialect('common.xml',2);wireBytes=0;frames=0;
for row=1:rows
    bytes=fread(fi,1112,'*uint8');oracle=fread(fi,61,'*double');assert(numel(oracle)==61);
    reassembled=uint8([]);
    for part=0:9
        n=fread(fi,1,'*uint16');packet=fread(fi,double(n),'*uint8').';
        [decoded,status]=deserializemsg(dialect,packet,OutputAllMessage=true);
        assert(numel(decoded)==1&&status==0&&packet(6)==1&&packet(7)==1);
        p=decoded.Payload;assert(p.payload_type==42002&&p.target_system==42&&p.target_component==191&&p.payload(1)==80+part);
        reassembled=[reassembled;reshape(p.payload(10:double(p.payload_length)),[],1)]; %#ok<AGROW>
        wireBytes=wireBytes+double(n);frames=frames+1;
    end
    assert(isequal(bytes,reassembled));
    f=gpenmpcNative.RflyCommittedFeedbackDecoder(reassembled,uint64(9000000000)+uint64(row));
    assert(isequal(typecast(f.actual61,'uint8'),typecast(oracle,'uint8'))&&strcmp(f.disposition,'Fresh'));
    assert(isequal(f.control.desired_force_projected_up_n,oracle(14:16)) ...
        &&isequal(f.control.rotor_command_n,oracle(5:10))&&f.control.force_projection_norm_mismatch_n==oracle(59) ...
        &&f.control.rotor_saturated==logical(oracle(61))&&~f.control_authority&&~f.transport_source_authenticated);
    assert(isa(f.token.sample_generation,'uint64')&&isa(f.commit_completed_us,'uint64')&&f.token.sample_generation==uint64(row));
end
assert(isempty(fread(fi,1,'*uint8')));negative=0;
for mutation=1:6
    bad=bytes;rejected=false;
    if mutation==1,bad(600)=bitxor(bad(600),uint8(1));end
    if mutation==2,bad(5)=0;end
    if mutation==3,bad=bad(1:end-1);end
    if mutation==4,bad(1)=0;end
    if mutation==5,bad(1065:1072)=0;bad=rehash(bad);end % commit before publication
    try
        if mutation==6,gpenmpcNative.RflyCommittedFeedbackDecoder(bad,9000000000);
        else,gpenmpcNative.RflyCommittedFeedbackDecoder(bad,uint64(9000000000));end
    catch ex,rejected=startsWith(string(ex.identifier),'gpenmpcNative:Feedback');end
    assert(rejected);negative=negative+1;
end
for disposition=uint8([2,3])
    historical=bytes;historical(5)=disposition;historical=rehash(historical);
    history=gpenmpcNative.RflyCommittedFeedbackDecoder(historical,uint64(9000000000));
    assert(history.historical_only&&~history.control_authority&&isequal(typecast(history.actual61,'uint8'),typecast(f.actual61,'uint8')));
end
receipt=struct('status','PASS_ACTUAL_MATLAB_RFC1_DECODER','actual61_bit_exact_rows',rows, ...
    'actual_generated_mavlink_frames',frames,'wire_bytes',wireBytes,'message_bytes',1112, ...
    'negative_cases',negative,'historical_and_revoked_preserve_actual61',true,'host_kernel_recomputed',false, ...
    'scope',scope,'input_file',input,'board_access',0);
mkdir(outputRoot);fo=fopen(fullfile(outputRoot,'MATLAB_COMMITTED_FEEDBACK_RESULT.json'),'w');fwrite(fo,jsonencode(receipt,PrettyPrint=true),'char');fclose(fo);disp(receipt);
end
function bytes=rehash(bytes)
md=java.security.MessageDigest.getInstance('SHA-256');md.update(typecast(bytes(1:1080),'int8'));
bytes(1081:1112)=reshape(typecast(md.digest(),'uint8'),[],1);
end
