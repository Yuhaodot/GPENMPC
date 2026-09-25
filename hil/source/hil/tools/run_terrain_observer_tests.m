function report=run_terrain_observer_tests(outputDir)
% Production observer checks, without sockets, DLL, model or board.
arguments,outputDir (1,1) string,end
assert(~isfolder(outputDir)&&~isfile(outputDir));
b=string(fileparts(fileparts(mfilename('fullpath'))));oldPath=path;
g=onCleanup(@()path(oldPath)); %#ok<NASGU>
addpath(fullfile(b,'matlab_validation'),fullfile(b,'m600_coptersim','matlab_validation'));
checks=struct('name',{},'passed',{});examples={};
for required=[false,true]
    s=[];d=zeros(32,1);d(1:7)=[0;0;1;0;1;0;1];if required,d(26)=1;end
    bytes=packet(d);
    s=m600check.updateCopterSimDiagnostic(s,bytes,.1,1,required);
    v=m600check.copterSimDiagnosticSnapshot(s,.1,.25);
    check(sprintf('mode%d_first_packet_not_progress',required),~v.model_ready);
    d(3)=1.01;s=m600check.updateCopterSimDiagnostic(s,packet(d),.11,1,required);
    v=m600check.copterSimDiagnosticSnapshot(s,.11,.25);
    check(sprintf('mode%d_new_progress_ready',required),v.model_ready);
    s=m600check.updateCopterSimDiagnostic(s,packet(d),.4,1,required);
    v=m600check.copterSimDiagnosticSnapshot(s,.4,.25);
    check(sprintf('mode%d_duplicates_never_replenish_source_age',required), ...
        ~v.model_ready&&strcmp(v.status,'MODEL_DIAGNOSTIC_SOURCE_PROGRESS_STALE'));
    d(3)=1.02;s=m600check.updateCopterSimDiagnostic(s,packet(d),.41,1,required);
    check(sprintf('mode%d_only_new_progress_recovers_nonfault',required), ...
        m600check.copterSimDiagnosticSnapshot(s,.41,.25).model_ready);
    check(sprintf('mode%d_session_contract_cannot_change',required), ...
        fails(@()m600check.updateCopterSimDiagnostic(s,packet(d),.42,1,~required)));
    original=m600check.decodeCopterSimDiagnostics(packet(d),1,1.01);
    if ~required
        check('default_legacy_decoder_result_bit_exact',isequaln(original,s.last_decoded));
        implicit=m600check.updateCopterSimDiagnostic([],bytes,.1,1);
        explicit=m600check.updateCopterSimDiagnostic([],bytes,.1,1,false);
        check('four_argument_legacy_call_unchanged',isequaln(implicit,explicit));
    else
        check('required_extension_semantics_valid',s.last_decoded.terrain_extension.valid);
    end
end
legacy=zeros(32,1);legacy(1:7)=[0;0;1;0;1;0;1];
s=m600check.updateCopterSimDiagnostic([],packet(legacy),.1,1,true);
check('legacy_packet_cannot_impersonate_new_DLL',~isempty(s.fatal_reason)&& ...
    contains(s.fatal_reason,'REQUIRED_TERRAIN_EXTENSION_ABSENT'));
extended=legacy;extended(26)=1;extended(3)=2;
s=m600check.updateCopterSimDiagnostic(s,packet(extended),.2,1,true);
check('later_extension_cannot_clear_wrong_identity',~m600check.copterSimDiagnosticSnapshot(s,.2,.25).model_ready);

bits=bitor(bitshift(uint64(hex2dec('7FF80000')),32),uint64(123));
for reason=[1,2,3]
    state=m600check.initialCopterSimTerrainDiagnosticState();raw=zeros(15,1);locked=0;
    if reason==2,raw(15)=typecast(bits,'double');elseif reason==3,locked=1;raw(1)=1+eps(1);end
    [bad,~]=m600check.encodeCopterSimTerrainDiagnostics([1;4;2;0;0;1;1],raw,reason,locked,true,false,state);
    s=m600check.updateCopterSimDiagnostic([],packet(bad),.1,1,true);
    first=s.first_fault;
    check(sprintf('reason%d_valid_packet_but_never_healthy',reason), ...
        first.decoded.packet_valid&&first.decoded.must_stop&& ...
        ~m600check.copterSimDiagnosticSnapshot(s,.1,.25).model_ready);
    check(sprintf('reason%d_exact_reason_and_raw_payload',reason), ...
        first.decoded.terrain_extension.first_reason==reason&& ...
        isequal(first.raw_bytes,packet(bad))&& ...
        isequal(typecast(first.decoded.terrain_extension.first_terrain15,'uint64'),typecast(raw,'uint64')));
    s=m600check.updateCopterSimDiagnostic(s,packet(extended),.2,1,true);
    check(sprintf('reason%d_first_fault_not_washed_by_healthy',reason), ...
        isequaln(first,s.first_fault)&&~m600check.copterSimDiagnosticSnapshot(s,.2,.25).model_ready);
    examples{end+1}=first; %#ok<AGROW>
end
for j=1:2
    broken=extended;if j==1,broken(26)=2;else,broken(27)=1;end
    s=m600check.updateCopterSimDiagnostic([],packet(broken),.1,1,true);
    check(sprintf('unknown_extension_%d_fail_closed',j),~isempty(s.fatal_reason));
end
check('nonlogical_contract_rejected',fails(@()m600check.updateCopterSimDiagnostic([],packet(extended),.1,1,1)));
check('legacy_state_without_explicit_contract_rejected', ...
    fails(@()m600check.updateCopterSimDiagnostic(struct('last_source_time_s',0),packet(extended),.1,1,true)));
mkdir(outputDir);
report=struct('passed',all([checks.passed]),'checks',checks,'checks_total',numel(checks), ...
    'checks_passed',sum([checks.passed]),'COM_open',0,'UDP_open',0,'board_actions',0, ...
    'examples',{examples},'observer_sha256',m600check.fileSha256(which('m600check.updateCopterSimDiagnostic')), ...
    'decoder_sha256',m600check.fileSha256(which('m600check.decodeCopterSimTerrainDiagnostics')), ...
    'claim','Host observer tests.');
save(fullfile(outputDir,'RAW.mat'),'report');
f=fopen(fullfile(outputDir,'RESULT.json'),'w','n','UTF-8');assert(f>=0);c=onCleanup(@()fclose(f)); %#ok<NASGU>
fprintf(f,'%s\n',jsonencode(report,PrettyPrint=true));
fprintf('Terrain observer: %d/%d checks passed.\n',report.checks_passed,report.checks_total);
assert(report.passed);
    function check(n,p),checks(end+1)=struct('name',n,'passed',logical(p));end
end
function b=packet(d)
head=int32([1234567890;1]);[~,~,e]=computer;
if e=='B',head=swapbytes(head);d=swapbytes(d);end
b=[typecast(head,'uint8');typecast(d(:),'uint8')];b=b(:);
end
function yes=fails(fn)
yes=false;try,fn();catch,yes=true;end
end
