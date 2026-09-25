function report=test_rfly_host_context_binding(outputRoot)
% Test MATLAB and C++ context binding with retained solver values
% and synthetic event and timestamp associations.
arguments
    outputRoot (1,1) string
end
build=string(fileparts(fileparts(mfilename('fullpath'))));
base=fullfile(gpenmpc_external_path('rfly_host_context_binding'));
assert(~isfolder(outputRoot),'Preserve each earlier test receipt.');mkdir(outputRoot);
priorPath=path;restore=onCleanup(@()path(priorPath)); %#ok<NASGU>
addpath(fullfile(build,'host_runtime'),'-begin');
checks=struct('name',{},'pass',{},'detail',{});
sourceNames=["RflyHostContextBinding.m","RflySnapshotSample.m","px4EstimateState.m", ...
    "RflyOuterPayload.m","RflyContextEncoder.m","RflySnapshotDecoder.m"];
sourcePaths=fullfile(build,'host_runtime','+gpenmpcNative',sourceNames);
before=arrayfun(@fileHash,sourcePaths);
compileProbe();runProbe("export");
bytes1=readBytes(fullfile(outputRoot,'SOURCE_1.bin'));
bytes2=readBytes(fullfile(outputRoot,'SOURCE_2.bin'));
oldFixture=readBytes(fullfile(build,'rfly_vendor_integration','px4_wire','host_context_02','PRIVATE_SNAPSHOT_AND_CONTEXT.bin'));
check('existing_wire_fixture_first246_exact',isequal(bytes1,oldFixture(1:246)));
rx1=uint64(9000000000);rx2=uint64(9300000000);age=uint64(400000000);
s1=gpenmpcNative.RflySnapshotDecoder(bytes1,rx1);s2=gpenmpcNative.RflySnapshotDecoder(bytes2,rx2);
expected=struct('uid',string(s1.observed_uid),'system_id',double(s1.source_system), ...
    'component_id',double(s1.source_component),'boot_generation',s1.observed_boot_generation, ...
    'maximum_age_ns',age,'configuration_payload_sha256',upper(reshape(dec2hex(s1.configuration_sha256,2).',1,[])));
archivePath=fullfile(gpenmpc_external_path('runtime_alignment_fixture'),'RAW.mat');
archive=load(archivePath,'raw');archivedResponse=archive.raw.first;
[archived4,archivedReceipt]=gpenmpcNative.RflyOuterPayload(archivedResponse);
reference=decodeValue(oldFixture(247:562),165:252,'double');
owner=newOwner();p1=owner.recordSnapshot(bytes1,rx1);
[~,ok]=gpenmpcNative.px4EstimateState(p1,expected,rx1+uint64(1000));
check('stored_private_source_reconstructs_at_actual_estimate_boundary',ok);
owner.submitted(uint64(1),p1.source_ticket,rx1+uint64(10000));
check('pending_outer_source_cannot_retire',~owner.retireSource(p1.source_ticket));
r1=responseFor(p1,uint64(1));owner.committed(r1,rx1+uint64(50000));
[c1,b1]=owner.reference(uint64(1),p1.source_ticket,reference,rx1+uint64(100000));
check('actual_archived_solver_four_doubles_verbatim',isequal(typecast(c1.outer_payload,'uint64'),typecast(archived4,'uint64')));
check('one_exact_stored_source_ticket_used_by_reference_outer_and_binding', ...
    isequal(c1.outer_source_ticket,p1.source_ticket)&&isequal(c1.reference_source_ticket,p1.source_ticket)&&isequal(b1.snapshot_ticket,p1.source_ticket));
check('held_outer_source_cannot_retire',~owner.retireSource(p1.source_ticket));
p2=owner.recordSnapshot(bytes2,rx2);
[c2,b2]=owner.reference(uint64(2),p2.source_ticket,reference,rx2+uint64(100000));
check('two_references_hold_same_outer_generation_payload_and_all_original_times',c2.outer_generation==c1.outer_generation ...
    &&isequal(c2.outer_source_ticket,c1.outer_source_ticket)&&isequal(c2.outer_payload,c1.outer_payload) ...
    &&c2.outer_source_receipt_ns==c1.outer_source_receipt_ns&&c2.outer_creation_ns==c1.outer_creation_ns&&c2.outer_expiry_ns==c1.outer_expiry_ns);
check('new_reference_bound_to_later_exact_source',isequal(b2.snapshot_ticket,p2.source_ticket)&&c2.reference_source_receipt_ns==rx2);
check('old_outer_still_pinned_after_new_reference',~owner.retireSource(p1.source_ticket));
owner.submitted(uint64(2),p2.source_ticket,rx2+uint64(150000));
r2=responseFor(p2,uint64(2));owner.committed(r2,rx2+uint64(200000));
[c3,b3]=owner.reference(uint64(3),p2.source_ticket,reference,rx2+uint64(300000));
check('fresh_outer_updates_generation_source_and_original_expiry',c3.outer_generation==2 ...
    &&isequal(c3.outer_source_ticket,p2.source_ticket)&&c3.outer_source_receipt_ns==rx2 ...
    &&c3.outer_creation_ns==rx2+uint64(200000)&&c3.outer_expiry_ns==rx2+age);
check('retire_only_unheld_old_source',owner.retireSource(p1.source_ticket)&&~owner.retireSource(p2.source_ticket));

dialect=mavlinkdialect('common.xml',2);
serializer=mavlinkio(dialect,SystemID=42,ComponentID=191,ComponentType='MAV_TYPE_GCS',AutopilotType='MAV_AUTOPILOT_INVALID');
contexts={c1,c2,c3};encoderReceipts=cell(3,1);packetSets=cell(3,1);
for n=1:3
    c=contexts{n};[packets,encoderReceipts{n},encoded]=gpenmpcNative.RflyContextEncoder(c,serializer,dialect);
    check("RCT1_"+n+"_actual_original_HOST_times_in_bytes",isequal(encoded(117:164), ...
        bigEndian([c.reference_source_receipt_ns;c.reference_creation_ns;c.reference_expiry_ns; ...
        c.outer_source_receipt_ns;c.outer_creation_ns;c.outer_expiry_ns])));
    check("RCT1_"+n+"_actual_solver4_binary64_bits",isequal(encoded(253:284),bigEndian(archived4)));
    check("RCT1_"+n+"_source_ticket_bytes_exact",isequal(encoded(53:84),c.reference_source_ticket)&&isequal(encoded(85:116),c.outer_source_ticket));
    assembled=uint8([]);
    for k=1:3
        [messages,status]=deserializemsg(dialect,packets{k},OutputAllMessage=true);
        check("RCT1_"+n+"_actual_MAVLink_parser_"+k,numel(messages)==1&&status==0);
        payload=messages.Payload;
        check("RCT1_"+n+"_schema4_fragment_"+k,payload.payload_type==42002&&payload.payload(1)==64+k-1 ...
            &&decodeValue(payload.payload(:),2:9,'uint64')==c.reference_generation);
        assembled=[assembled;reshape(payload.payload(10:double(payload.payload_length)),[],1)]; %#ok<AGROW>
    end
    check("RCT1_"+n+"_whole_reassembly_exact",isequal(assembled,encoded));
    writeBytes(fullfile(outputRoot,"CONTEXT_"+n+".bin"),encoded);packetSets{n}=packets;
end
runProbe("check");board=jsondecode(fileread(fullfile(outputRoot,'BOARD_CONTEXT_RESULT.json')));
check('actual_Cpp_ContextBinding_all_contexts_accepted',board.failed==0&&board.contexts_accepted==3&&board.actual_ContextBinding);
referenceDeadline=[s1.original_sample_hrt_us;s2.original_sample_hrt_us;s2.original_sample_hrt_us]+idivide(age,uint64(1000));
outerDeadline=[s1.original_sample_hrt_us;s1.original_sample_hrt_us;s2.original_sample_hrt_us]+idivide(age,uint64(1000));
check('same_conservative_original_source_deadline_not_clock_map',isequal(uint64(board.reference_valid_until_us(:)),referenceDeadline) ...
    &&isequal(uint64(board.outer_valid_until_us(:)),outerDeadline)&&~board.clock_mapping);

% Use a fresh owner for each negative case and verify its fail-closed status.
for mode=1:12
    neg=newOwner();pn=neg.recordSnapshot(bytes1,rx1);caught=false;why="";
    try
        switch mode
            case 1,neg.submitted(uint64(1),bitxor(pn.source_ticket,uint8(1)),rx1+uint64(1000));
            case 2,neg.recordSnapshot(bytes2,rx1-uint64(1));
            case 3
                bad=modifiedSource(bytes2,'sample',s1.original_sample_hrt_us-uint64(1));neg.recordSnapshot(bad,rx2);
            case 4
                bad=modifiedSource(bytes2,'generation',s1.subscription_generation);neg.recordSnapshot(bad,rx2);
            case 5
                check('unheld_source_can_retire_for_reentry_test',neg.retireSource(pn.source_ticket));neg.recordSnapshot(bytes1,rx2);
            otherwise
                neg.submitted(uint64(1),pn.source_ticket,rx1+uint64(10000));rr=responseFor(pn,uint64(1));
                if mode==6,rr.generation=2;end
                if mode==7,rr.input_boundary.estimate_generations(2)=rr.input_boundary.estimate_generations(2)+1;end
                if mode==8,rr.input_boundary.estimate_rx_ns(2)=rr.input_boundary.estimate_rx_ns(2)+1;end
                if mode==9,rr.schema='GPENMPC_BOARD_OUTER_ASYNC_POLL_RESPONSE_V1';rr.status='PENDING';end
                if mode==10
                    neg.committed(rr,rx1+age+uint64(1));
                else
                    neg.committed(rr,rx1+uint64(50000));
                    if mode==11,neg.reference(uint64(1),pn.source_ticket,reference,rx1+age+uint64(1));end
                    if mode==12
                        neg.reference(uint64(1),pn.source_ticket,reference,rx1+uint64(100000));
                        neg.reference(uint64(1),pn.source_ticket,reference,rx1+uint64(100001));
                    end
                end
        end
    catch ex,caught=startsWith(string(ex.identifier),'gpenmpcNative:');why=string(ex.message);
    end
    st=neg.status();check("negative_"+mode+"_actual_fail_closed",caught&&st.failed,why);
end
for mode=1:3
    neg=newOwner();large=uint64(flintmax)+uint64(2);pn=neg.recordSnapshot(bytes1,large);
    neg.submitted(uint64(1),pn.source_ticket,large+uint64(10000));rr=responseFor(pn,uint64(1));
    rr.input_boundary.estimate_rx_ns=repmat(large,3,1);
    if mode==1,rr.input_boundary.estimate_rx_ns(1)=large+uint64(1);end
    if mode==2,rr.input_boundary.estimate_rx_ns=double(rr.input_boundary.estimate_rx_ns);end
    caught=false;try,neg.committed(rr,large+uint64(50000));catch ex,caught=startsWith(string(ex.identifier),'gpenmpcNative:');end
    check("exact_uint64_receipt_over_flintmax_case_"+mode,((mode<3)&&caught)||((mode==3)&&~caught));
end
% Async committed and canonical soft fallback retain applied-command semantics.
soft=responseFor(p1,uint64(1));soft.schema='GPENMPC_BOARD_OUTER_ASYNC_POLL_RESPONSE_V1';
soft.status='DEADLINE_FALLBACK_COMMITTED';soft.decision_success=false;
soft.applied_command_source='CANONICAL_CONTINUITY_FALLBACK';
[payload,softReceipt]=gpenmpcNative.RflyOuterPayload(soft);
check('async_committed_soft_fallback_not_rejected_by_new_success_gate',isequal(typecast(payload,'uint64'),typecast(archived4,'uint64')));
pending=soft;pending.status='PENDING';rejected=false;
try,gpenmpcNative.RflyOuterPayload(pending);catch ex,rejected=strcmp(ex.identifier,'gpenmpcNative:ContextOuterNotCommitted');end
check('async_PENDING_produces_no_payload',rejected);
softOwner=newOwner();sp=softOwner.recordSnapshot(bytes1,rx1);softOwner.submitted(uint64(1),sp.source_ticket,rx1+uint64(10000));
softOwner.committed(soft,rx1+uint64(50000));[softContext,~]=softOwner.reference(uint64(1),sp.source_ticket,reference,rx1+uint64(100000));
check('async_soft_fallback_reaches_context_with_applied_four_values',isequal(softContext.outer_payload,archived4));
% Callback FIFO originals may predate a later processed solver completion.
% Separate processing order without relabelling source times or expiry.
queued=newOwner();qp1=queued.recordSnapshot(bytes1,rx1,rx1+uint64(10));
queued.submitted(uint64(1),qp1.source_ticket,rx1+uint64(10000));
queued.committed(responseFor(qp1,uint64(1)),rx2+uint64(20000));
qp2=queued.recordSnapshot(bytes2,rx2,rx2+uint64(30000));
[qc,~]=queued.reference(uint64(1),qp2.source_ticket,reference,rx2+uint64(40000));
check('queued_original_rx_before_processed_commit_accepted_without_retime', ...
    qp2.source_host_receive_ns==rx2&&isequal(qp2.source_export_bytes,bytes2) ...
    &&qc.reference_source_receipt_ns==rx2&&qc.reference_expiry_ns==rx2+age ...
    &&qc.outer_source_receipt_ns==rx1&&qc.outer_expiry_ns==rx1+age);
check('queued_original_rx_and_processing_clocks_separate', ...
    queued.status().last_original_source_receive_ns==rx2 ...
    &&queued.status().last_processing_event_ns==rx2+uint64(40000));
for mode=1:3
    neg=newOwner();neg.recordSnapshot(bytes1,rx1,rx1+uint64(100));caught=false;
    try
        if mode==1,neg.recordSnapshot(bytes2,rx2,rx2-uint64(1));
        elseif mode==2,neg.recordSnapshot(bytes2,rx1-uint64(1),rx2);
        else,neg.recordSnapshot(bytes2,rx1+uint64(1),rx1+uint64(99));end
    catch,caught=true;end
    check("queued_source_original_or_processing_order_negative_"+mode,caught&&neg.status().failed);
end
after=arrayfun(@fileHash,sourcePaths);check('production_sources_not_changed_during_test',isequal(before,after));
report=struct('passed',all([checks.pass]),'checks',checks,'total',numel(checks),'passed_checks',sum([checks.pass]), ...
    'scope','HOST_FIXTURE_EVENT_ASSOCIATION__ACTUAL_ARCHIVED_SOLVER4__ACTUAL_MATLAB_CODEC_AND_CPP_CONTEXT_BINDING', ...
    'association_fields_fixture_only',{{'generation','input_boundary.estimate_generations','input_boundary.estimate_rx_ns','configuration_payload_sha256'}}, ...
    'archived_solver_path',archivePath,'archived_solver_sha256',fileHash(archivePath), ...
    'production_source_paths',sourcePaths,'source_sha256_before',before,'source_sha256_after',after, ...
    'new_solver_calls',0,'kernel_steps',0,'live_session_registered',false,'live_time_domain_binding',false, ...
    'board_actions',0,'COM_or_socket_created',false,'board_context_result',board,'owner_status',owner.status(), ...
    'archived_payload_receipt',archivedReceipt,'soft_fallback_receipt',softReceipt);
save(fullfile(outputRoot,'RAW.mat'),'report','p1','p2','contexts','b1','b2','b3','r1','r2','archived4','encoderReceipts','packetSets','softContext');
fid=fopen(fullfile(outputRoot,'RESULT.json'),'w');assert(fid>=0);fwrite(fid,jsonencode(report,PrettyPrint=true),'char');fclose(fid);
disp(struct('passed',report.passed,'checks',report.total,'passed_checks',report.passed_checks,'output',outputRoot));
assert(report.passed,'gpenmpcNative:HostContextTest','See RESULT.json for actual failed negative controls.');

    function check(name,passed,detail)
        if nargin<3,detail="";end
        checks(end+1)=struct('name',string(name),'pass',logical(passed),'detail',string(detail)); %#ok<AGROW>
    end
    function value=newOwner()
        value=gpenmpcNative.RflyHostContextBinding(expected,4,age,age);
    end
    function response=responseFor(p,generation)
        response=archivedResponse;response.generation=generation;
        response.configuration_payload_sha256=expected.configuration_payload_sha256;
        response.input_boundary.estimate_generations=repmat(double(p.position_generation),3,1);
        response.input_boundary.estimate_rx_ns=repmat(p.source_host_receive_ns,3,1);
    end
    function compileProbe()
        rfly=fullfile(build,'rfly_vendor_integration');
        headers=string(gpenmpc_external_path('px4_fmuv6c_build_headers'));
        compiler=gpenmpc_install_path('llvm','bin\clang++.exe');
        arm=fullfile(build,'evidence','arm_controller','GPENMPC_Rfly_Canonical_Controller_ert_rtw');
        args=[quote(compiler),"-std=c++14 -O2 -ffp-contract=off -fno-fast-math -Wall -Wextra -Werror -pedantic -static -Wno-address-of-packed-member", ...
            "-isystem",quote(fullfile(headers,'mavlink','common')),"-isystem",quote(fullfile(headers,'mavlink')), ...
            "-I",quote(fullfile(rfly,'official_io_nuttx','integration_patch','generated_all')), ...
            "-I",quote(fullfile(build,'px4_full_inner','px4_state_adapter','host_stub')),"-I",quote(headers),"-I",quote(arm), ...
            quote(fullfile(base,'context_deadline_probe.cpp'))];
        objects=["GPENMPC_Rfly_Canonical_Controller_host.o","rt_nonfinite_host.o","rtGetInf_host.o"];
        for j=1:numel(objects),args(end+1)=quote(fullfile(rfly,'host_matlab_slim_crosscheck',objects(j)));end %#ok<AGROW>
        args=[args,"-o",quote(fullfile(outputRoot,'context_deadline_probe.exe'))];
        command=strjoin(args,' ');[status,text]=system(command);writeText(fullfile(outputRoot,'PROBE_COMPILE.txt'),command+newline+string(text));
        assert(status==0,'gpenmpcNative:ContextProbeCompile','%s',text);
    end
    function runProbe(mode)
        command=strjoin([quote(fullfile(outputRoot,'context_deadline_probe.exe')), ...
            quote(fullfile(gpenmpc_external_path('full_inner_px4_float_mapping'),'MATLAB_ARGUMENTS_AND_EXPECTED.bin')), ...
            quote(fullfile(gpenmpc_external_path('full_inner_px4_float_mapping'),'MATLAB_NED_AND_MAPPED_STATE.bin')),quote(outputRoot),mode],' ');
        [status,text]=system(command);writeText(fullfile(outputRoot,"PROBE_"+upper(mode)+".txt"),command+newline+string(text));
        assert(status==0,'gpenmpcNative:ContextProbeRun','%s',text);
    end
end
function b=readBytes(file)
f=fopen(file,'rb');assert(f>=0);cleanup=onCleanup(@()fclose(f));b=fread(f,inf,'*uint8'); %#ok<NASGU>
end
function writeBytes(file,b)
f=fopen(file,'wb');assert(f>=0);cleanup=onCleanup(@()fclose(f));fwrite(f,b,'uint8'); %#ok<NASGU>
end
function writeText(file,text)
f=fopen(file,'w');assert(f>=0);cleanup=onCleanup(@()fclose(f));fwrite(f,char(text),'char'); %#ok<NASGU>
end
function hash=fileHash(file)
b=readBytes(file);md=java.security.MessageDigest.getInstance('SHA-256');md.update(typecast(b,'int8'));
hash=string(upper(reshape(dec2hex(typecast(md.digest(),'uint8'),2).',1,[])));
end
function text=quote(value)
text='"'+string(value)+'"';
end
function bytes=bigEndian(value)
[~,~,endian]=computer;if endian=='L',value=swapbytes(value);end
bytes=reshape(typecast(value(:),'uint8'),[],1);
end
function value=decodeValue(bytes,indices,kind)
value=typecast(bytes(indices),kind);[~,~,endian]=computer;if endian=='L',value=swapbytes(value);end;value=value(:);
end
function bytes=modifiedSource(bytes,field,value)
if strcmp(field,'sample')
    bytes(63:70)=bigEndian(uint64(value));bytes(71:78)=bigEndian(uint64(value)+uint64(100));bytes(79:86)=bigEndian(uint64(value)+uint64(300));
else,bytes(59:62)=bigEndian(uint32(value));end
% Inject corruption into a host fixture.
md=java.security.MessageDigest.getInstance('SHA-256');md.update(typecast(bytes(1:214),'int8'));bytes(215:246)=reshape(typecast(md.digest(),'uint8'),[],1);
end
