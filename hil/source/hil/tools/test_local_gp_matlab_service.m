function report=test_local_gp_matlab_service(outputRoot,pairsPath)
% Compare C GP-query bytes with the MATLAB predictor.
arguments
    outputRoot (1,1) string
    pairsPath (1,1) string
end
build=string(fileparts(fileparts(mfilename('fullpath'))));oldPath=path;
cleanup=onCleanup(@()path(oldPath)); %#ok<NASGU>
assert(~isfolder(outputRoot),'Preserve prior results');mkdir(outputRoot);
diary(fullfile(outputRoot,'MATLAB_DIARY.txt'));diaryGuard=onCleanup(@()diary('off')); %#ok<NASGU>
addpath(fullfile(build,'host_runtime'),'-begin');
a=gpenmpcNative.loadCanonicalAssets();
tracked=[string(mfilename('fullpath'))+'.m';pairsPath; ...
    string(which('gpenmpcNative.RflyLocalGpCodec'));string(which('gpenmpcNative.RflyLocalGpService')); ...
    string(which('gpenmpcNative.canonicalSparseGpFixedInput'));string(which('gpenmpcSparseGpPredict'))];
before=arrayfun(@sha,tracked);
f=fopen(pairsPath,'rb');assert(f>=0);fg=onCleanup(@()fclose(f)); %#ok<NASGU>
raw=fread(f,Inf,'*uint8');assert(mod(numel(raw),596)==0&&numel(raw)==596*59);raw=reshape(raw,596,[]);
q=gpenmpcNative.RflyLocalGpCodec.decodeRequest(raw(1:310,1));
e=struct('uid',q.identity.uid,'boot_generation',q.identity.boot_generation, ...
    'board_system',q.identity.system,'board_component',q.identity.component, ...
    'host_system',uint8(255),'host_component',uint8(190),'link_lifecycle_generation',uint64(8), ...
    'confirmed_host_rx_ns',uint64(100),'execution_session_sha256',repmat('A',1,64));
origin=struct('link_lifecycle_generation',e.link_lifecycle_generation, ...
    'execution_session_sha256',e.execution_session_sha256);
checks=struct('name',{},'pass',{});rows=struct('index',{},'source_generation',{},'output_generation',{}, ...
    'maximum_gp_difference',{},'prediction_wall_s',{},'hard_invalid',{},'trust',{});
service=gpenmpcNative.RflyLocalGpService(a,e);maxDifference=0;
for k=1:59
    request=raw(1:310,k);expected=raw(311:596,k);
    q=gpenmpcNative.RflyLocalGpCodec.decodeRequest(request);
    c=gpenmpcNative.RflyLocalGpCodec.decodeReply(expected);
    check(sprintf('c_reply_exact_roundtrip_%02d',k), ...
        isequal(gpenmpcNative.RflyLocalGpCodec.encodeReply(q,c.result18),expected));
    now=uint64(1000+1000*k);timer=tic;
    result=service.process(request,now,now+uint64(1),origin);wall=toc(timer);
    got=gpenmpcNative.RflyLocalGpCodec.decodeReply(result.reply_bytes);
    difference=max(abs(got.result18-c.result18));maxDifference=max(maxDifference,difference);
    check(sprintf('original_matlab_gp18_matches_c_%02d',k),difference<=1e-10 ...
        &&isequal(got.result18(15),c.result18(15)) ...
        &&isequal(result.reply_bytes(1:110),expected(1:110)));
    joined=zeros(286,1,'uint8');
    for j=1:3
        t=result.tunnel_payloads{j};n=double(t.payload_length)-9;index=j-1;
        check(sprintf('reply_direction_schema_%02d_%d',k,j),t.payload_type==42002 ...
            &&t.target_system==e.board_system&&t.target_component==e.board_component ...
            &&result.source_system==e.host_system&&result.source_component==e.host_component ...
            &&t.payload(1)==bitshift(uint8(9),4)+uint8(index) ...
            &&all(t.payload(10+n:end)==0));
        joined(119*index+1:119*index+n)=t.payload(10:9+n);
    end
    check(sprintf('reply_payloads_no_send_%02d',k),isequal(joined,result.reply_bytes) ...
        &&result.packets_sent==0&&result.control_publications==0);
    rows(end+1)=struct('index',k,'source_generation',string(q.source_generation), ...
        'output_generation',string(q.output_generation),'maximum_gp_difference',difference, ...
        'prediction_wall_s',wall,'hard_invalid',got.result18(15),'trust',got.result18(14)); %#ok<AGROW>
end
check('59_actual_predictions_no_hold',service.Calls==59&&service.Completed==59);
check('duplicate_request_permanent_reject',reject(@()service.process(raw(1:310,59),uint64(100000),uint64(100001),origin)) ...
    &&service.Failed&&service.Calls==59);
check('no_retry_after_fault',reject(@()service.process(raw(1:310,1),uint64(100002),uint64(100003),origin)));
for k=1:8
    s=gpenmpcNative.RflyLocalGpService(a,e);b=raw(1:310,1);o=origin;rx=uint64(200);now=uint64(201);
    if k==1,b(279)=bitxor(b(279),uint8(1));end
    if k==2,b(95)=bitxor(b(95),uint8(1));b=checksum(b);end
    if k==3,b(63)=bitxor(b(63),uint8(1));b=checksum(b);end
    if k==4,b(12)=bitxor(b(12),uint8(1));b=checksum(b);end
    if k==5,o.link_lifecycle_generation=o.link_lifecycle_generation+uint64(1);end
    if k==6,rx=uint64(99);end
    if k==7,now=rx-uint64(1);end
    if k==8,s.close();end
    check(sprintf('negative_%d_no_gp_call',k),reject(@()s.process(b,rx,now,o))&&s.Calls==0);
end
q=gpenmpcNative.RflyLocalGpCodec.decodeRequest(raw(1:310,1));
c=gpenmpcNative.RflyLocalGpCodec.decodeReply(raw(311:596,1));c.result18(15)=1;
d=gpenmpcNative.RflyLocalGpCodec.decodeReply(gpenmpcNative.RflyLocalGpCodec.encodeReply(q,c.result18));
check('original_hard_invalid_field_preserved',d.result18(15)==1);
q.request19(:)=0; % Keep original bytes authoritative.
check('request_encode_redecodes_original_bytes', ...
    isequal(gpenmpcNative.RflyLocalGpCodec.encodeReply(q,c.result18),d.original_bytes));
after=arrayfun(@sha,tracked);check('all_inputs_stable',isequal(before,after));
service.close();
report=struct('scope','ACTUAL_C_PENDING_WIRE_TO_ORIGINAL_MATLAB_GP_HOST_ONLY', ...
    'checks',checks,'passed',sum([checks.pass]),'total',numel(checks),'all_pass',all([checks.pass]), ...
    'actual_c_query_records',59,'actual_matlab_predictions',59,'rows',rows, ...
    'maximum_gp_difference',maxDifference,'timing_claim','HOST_CODEC_AND_PREDICT_TIMING', ...
    'tracked',tracked,'before_sha256',before,'after_sha256',after,'COM',0,'board',0,'IO_connections',0, ...
    'authority_and_arrival_fixture',true,'actual_mavlink_serialization_here',false);
write(fullfile(outputRoot,'RESULT.json'),report);
fprintf('MATLAB actual GP/wire %d/%d, maxdiff %.17g\n',report.passed,report.total,maxDifference);
assert(report.all_pass);
    function check(name,pass)
        checks(end+1)=struct('name',name,'pass',logical(pass)); %#ok<AGROW>
        if ~pass,fprintf(2,'FAILED %s\n',name);end
    end
end
function ok=reject(f)
ok=false;try,f();catch,ok=true;end
end
function b=checksum(b)
md=java.security.MessageDigest.getInstance('SHA-256');md.update(typecast(b(1:end-32),'int8'));
b(end-31:end)=reshape(typecast(md.digest(),'uint8'),[],1);
end
function h=sha(p)
f=fopen(p,'rb');assert(f>=0);g=onCleanup(@()fclose(f)); %#ok<NASGU>
md=java.security.MessageDigest.getInstance('SHA-256');
while ~feof(f),b=fread(f,1048576,'*uint8');md.update(typecast(b,'int8'));end
h=string(upper(reshape(dec2hex(typecast(md.digest(),'uint8'),2).',1,[])));
end
function write(p,r)
f=fopen(p,'w','n','UTF-8');assert(f>=0);g=onCleanup(@()fclose(f)); %#ok<NASGU>
fprintf(f,'%s\n',jsonencode(r,PrettyPrint=true));
end
