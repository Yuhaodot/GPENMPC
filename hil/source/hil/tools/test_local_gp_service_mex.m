function report=test_local_gp_service_mex(outputRoot,runName)
% Test GP service validation and its MEX backend with retained C bytes
% and synthetic session and arrival metadata.
arguments
    outputRoot (1,1) string
    runName (1,1) string = "local_gp_service"
end
build=string(fileparts(fileparts(mfilename('fullpath'))));
assert(isfolder(outputRoot)&&~isempty(regexp(char(runName),'^[A-Z0-9_]+$','once')));
out=fullfile(outputRoot,runName);assert(~isfolder(out)&&~isfile(out),'Preserve prior evidence.');mkdir(out);
oldPath=path;services={};guard=onCleanup(@finish); %#ok<NASGU>
diary(fullfile(out,'MATLAB_DIARY.txt'));diaryGuard=onCleanup(@()diary('off')); %#ok<NASGU>
stage='load_original_assets_and_bound_binary';
try
    addpath(fullfile(build,'host_runtime'),'-begin');a=gpenmpcNative.loadCanonicalAssets();
    mexDir=fullfile(gpenmpc_external_path('canonical_gp_wire_mex'));
    mexPath=fullfile(mexDir,'canonical_gp_wire_mex.mexw64');addpath(mexDir,'-begin');
    mexSha='21017CD36C857466AE538EAE716C868173D2C54DF4D2B5CC458E3C6681CEBE6B';
    validation=jsondecode(fileread(fullfile(mexDir,'RESULT.json')));
    assert(validation.all_pass&&validation.actual_C_query_reply_rows==59 ...
        &&strcmpi(validation.binary_sha256,mexSha)&&strcmpi(sha(mexPath),mexSha));
    fixturePath=fullfile(build,'rfly_vendor_integration','full_inner_abi','snapshot_wire_fixture','RGP1_RGR1_PAIRS.bin');
    raw=readbytes(fixturePath);assert(numel(raw)==596*59);pairs=reshape(raw,596,59);
    tracked=[string(mfilename('fullpath'))+'.m';fixturePath;mexPath; ...
        string(which('gpenmpcNative.RflyLocalGpService'));string(which('gpenmpcNative.RflyLocalGpCodec')); ...
        string(which('gpenmpcNative.canonicalSparseGpFixedInput'));string(which('gpenmpcSparseGpPredict'))];
    before=arrayfun(@sha,tracked);
    q=gpenmpcNative.RflyLocalGpCodec.decodeRequest(pairs(1:310,1));
    e=struct('uid',q.identity.uid,'boot_generation',q.identity.boot_generation, ...
        'board_system',q.identity.system,'board_component',q.identity.component, ...
        'host_system',uint8(255),'host_component',uint8(190),'link_lifecycle_generation',uint64(8), ...
        'confirmed_host_rx_ns',uint64(100),'execution_session_sha256',repmat('A',1,64));
    nativeExpected=e;nativeExpected.gp_backend=struct('kind','CANONICAL_GP_WIRE_MEX', ...
        'path',mexPath,'binary_sha256',mexSha);
    origin=struct('link_lifecycle_generation',e.link_lifecycle_generation,'execution_session_sha256',e.execution_session_sha256);
    checks=struct('name',{},'pass',{});matlabService=newService(e);mexService=newService(nativeExpected);
    check('absent_backend_keeps_original_matlab_default',strcmp(matlabService.Backend,'MATLAB_ORIGINAL')&&isempty(matlabService.BackendBinding));
    binding=mexService.BackendBinding;
    check('explicit_mex_binding_hashed_once_at_construction',strcmp(mexService.Backend,'CANONICAL_GP_WIRE_MEX') ...
        &&strcmpi(binding.path,mexPath)&&strcmpi(binding.binary_sha256,mexSha) ...
        &&binding.binary_hash_checks==1&&binding.runtime_file_checks==0&&~binding.fallback_allowed ...
        &&binding.construction_load_probe&&binding.construction_gp_calls==0&&mexService.Calls==0);
    stage='59_actual_C_queries_through_both_production_service_backends';
    matlabReplies=zeros(286,59,'uint8');mexReplies=matlabReplies;matlabResults=zeros(59,18);mexResults=matlabResults;
    maxBackendError=0;maxFixtureError=0;metadataExact=true;invalidExact=true;payloadsExact=true;noIo=true;
    for k=1:59
        request=pairs(1:310,k);expected=gpenmpcNative.RflyLocalGpCodec.decodeReply(pairs(311:596,k));
        rx=uint64(1000+1000*k);processing=rx+uint64(1);
        rm=matlabService.process(request,rx,processing,origin);rn=mexService.process(request,rx,processing,origin);
        matlabReplies(:,k)=rm.reply_bytes;mexReplies(:,k)=rn.reply_bytes;
        matlabResults(k,:)=rm.result18;mexResults(k,:)=rn.result18;
        maxBackendError=max(maxBackendError,max(abs(rm.result18-rn.result18)));
        maxFixtureError=max(maxFixtureError,max(abs([rm.result18;rn.result18]-expected.result18.'),[],'all'));
        metadataExact=metadataExact&&isequal(rm.reply_bytes(1:110),pairs(311:420,k)) ...
            &&isequal(rn.reply_bytes(1:110),pairs(311:420,k));
        invalidExact=invalidExact&&rm.result18(15)==expected.result18(15)&&rn.result18(15)==expected.result18(15);
        payloadsExact=payloadsExact&&isequal(joinPayloads(rm,e),rm.reply_bytes)&&isequal(joinPayloads(rn,e),rn.reply_bytes);
        noIo=noIo&&rm.packets_sent==0&&rn.packets_sent==0&&rm.control_publications==0&&rn.control_publications==0;
    end
    check('59_calls_and_completions_per_backend_no_hold',matlabService.Calls==59&&matlabService.Completed==59&&mexService.Calls==59&&mexService.Completed==59);
    check('all_original_identity_and_entire_request_digest_bytes_exact',metadataExact);
    check('59_original_results_within_existing_1e_10',maxBackendError<=1e-10&&maxFixtureError<=1e-10);
    check('all_original_hard_invalid_fields_exact',invalidExact);
    check('all_bound_mex_reply_bytes_equal_prevalidated_C_fixture',isequal(mexReplies,pairs(311:596,:)));
    check('both_original_fragment_contracts_preserved_without_IO',payloadsExact&&noIo);
    stage='bounded_backend_and_service_negative_controls';
    missing=nativeExpected;missing.gp_backend=rmfield(missing.gp_backend,'path');
    check('explicit_mex_missing_binding_field_rejected',reject(@()gpenmpcNative.RflyLocalGpService(a,missing),'gpenmpcNative:LocalGpBackendBinding'));
    absent=nativeExpected;absent.gp_backend.path=fullfile(out,'ABSENT','canonical_gp_wire_mex.mexw64');
    check('explicit_mex_absent_binary_rejected',reject(@()gpenmpcNative.RflyLocalGpService(a,absent),'gpenmpcNative:LocalGpBackendMissing'));
    wrong=nativeExpected;wrong.gp_backend.binary_sha256=repmat('B',1,64);
    check('wrong_expected_binary_hash_rejected_without_fallback',reject(@()gpenmpcNative.RflyLocalGpService(a,wrong),'gpenmpcNative:LocalGpBackendBinding'));
    wrong=nativeExpected;wrong.gp_backend.kind='MATLAB_ORACLE';
    check('unknown_backend_kind_rejected_without_fallback',reject(@()gpenmpcNative.RflyLocalGpService(a,wrong),'gpenmpcNative:LocalGpBackendBinding'));
    check('default_replay_rejected_before_another_prediction',reject(@()matlabService.process(pairs(1:310,59),uint64(100000),uint64(100001),origin),'gpenmpcNative:LocalGpReplay') ...
        &&matlabService.Failed&&matlabService.Closed&&matlabService.Calls==59);
    check('mex_replay_rejected_before_another_prediction',reject(@()mexService.process(pairs(1:310,59),uint64(100000),uint64(100001),origin),'gpenmpcNative:LocalGpReplay') ...
        &&mexService.Failed&&mexService.Closed&&mexService.Calls==59);
    check('mex_fault_is_latched_no_retry',reject(@()mexService.process(pairs(1:310,1),uint64(100002),uint64(100003),origin),'gpenmpcNative:LocalGpClosed') ...
        &&mexService.Failure=="gpenmpcNative:LocalGpReplay"&&mexService.Calls==59);
    closedDefault=newService(e);closedDefault.close();closedMex=newService(nativeExpected);closedMex.close();
    check('both_closed_backends_reject_before_prediction',reject(@()closedDefault.process(pairs(1:310,1),uint64(200),uint64(201),origin),'gpenmpcNative:LocalGpClosed') ...
        &&reject(@()closedMex.process(pairs(1:310,1),uint64(200),uint64(201),origin),'gpenmpcNative:LocalGpClosed') ...
        &&closedDefault.Calls==0&&closedMex.Calls==0);
    session=newService(nativeExpected);badOrigin=origin;badOrigin.execution_session_sha256=repmat('B',1,64);
    check('mex_keeps_original_session_check_before_prediction',reject(@()session.process(pairs(1:310,1),uint64(200),uint64(201),badOrigin),'gpenmpcNative:LocalGpOrigin')&&session.Calls==0);
    clock=newService(nativeExpected);
    check('mex_keeps_original_clock_check_before_prediction',reject(@()clock.process(pairs(1:310,1),uint64(200),uint64(199),origin),'gpenmpcNative:LocalGpClock')&&clock.Calls==0);
    checksumService=newService(nativeExpected);bad=pairs(1:310,1);bad(279)=bitxor(bad(279),uint8(1));
    check('mex_keeps_original_request_checksum_check_before_prediction',reject(@()checksumService.process(bad,uint64(200),uint64(201),origin),'gpenmpcNative:LocalGpDigest')&&checksumService.Calls==0);
    % One finite OOD query exercises the original hard-invalid path, without
    % altering a returned flag or any model/acceptance threshold.
    ood=pairs(1:310,1);features=a.gp_model.input_mean+100*a.gp_model.input_scale;
    ood(135:270)=bigEndianBytes(features);ood=checksum(ood);
    om=newService(e);on=newService(nativeExpected);
    mr=om.process(ood,uint64(200),uint64(201),origin);nr=on.process(ood,uint64(200),uint64(201),origin);
    check('finite_ood_original_hard_invalid_preserved_without_rewrite',mr.result18(15)==1&&nr.result18(15)==1 ...
        &&max(abs(mr.result18-nr.result18))<=1e-10&&om.Calls==1&&on.Calls==1);
    after=arrayfun(@sha,tracked);check('all_original_inputs_and_binary_unchanged',isequal(before,after));
    save(fullfile(out,'RAW.mat'),'pairs','matlabReplies','mexReplies','matlabResults','mexResults','nativeExpected','origin','ood','mr','nr','binding');
    report=struct('scope','PRODUCTION_GP_SERVICE_EXPLICIT_MEX_BACKEND_HOST_UNIT_RETAINED_C_FIXTURE', ...
        'checks',checks,'total',numel(checks),'passed',sum([checks.pass]),'all_pass',all([checks.pass]), ...
        'actual_C_query_rows',59,'fixture_calls_per_backend',59,'additional_finite_ood_calls_per_backend',1, ...
        'maximum_absolute_error_between_backends',maxBackendError,'maximum_absolute_error_vs_C_fixture',maxFixtureError, ...
        'numerical_tolerance',1e-10,'hard_invalid_exact',invalidExact, ...
        'mex_reply_bytes_equal_C_fixture',isequal(mexReplies,pairs(311:596,:)), ...
        'mex_binding',binding,'source_session_arrival_fixture',true,'fresh_live_admission_proven',false, ...
        'timing_claim','HOST_REPLAY_TIMING', ...
        'tracked',tracked,'before_sha256',before,'after_sha256',after,'COM',0,'board',0, ...
        'IO_connections',0,'MATLAB_models',0,'controller_runs',0,'plant_runs',0,'solver_calls',0);
    writeJson(fullfile(out,'RESULT.json'),report);
    fprintf('Local GP service MEX %d/%d, maximum backend difference %.17g. HOST unit only.\n',report.passed,report.total,maxBackendError);
    assert(report.all_pass,'gpenmpcNative:LocalGpServiceMexTest','Production service backend unit check failed.');
catch ex
    writeJson(fullfile(out,'FAILURE.json'),struct('stage',stage,'exception',getReport(ex,'extended','hyperlinks','off'), ...
        'COM',0,'board',0,'IO_connections',0,'MATLAB_models',0,'controller_runs',0,'plant_runs',0));rethrow(ex)
end
clear guard
    function s=newService(expected)
        s=gpenmpcNative.RflyLocalGpService(a,expected);services{end+1}=s;
    end
    function check(name,pass)
        checks(end+1)=struct('name',name,'pass',logical(pass));if ~pass,fprintf(2,'FAILED %s\n',name);end
    end
    function finish()
        for index=1:numel(services),services{index}.close();end
        path(oldPath);
    end
end
function b=joinPayloads(result,e)
b=zeros(286,1,'uint8');assert(numel(result.tunnel_payloads)==3);
for j=1:3
    p=result.tunnel_payloads{j};n=double(p.payload_length)-9;
    assert(p.payload_type==42002&&p.target_system==e.board_system&&p.target_component==e.board_component ...
        &&p.payload(1)==uint8(144+j-1)&&all(p.payload(10+n:end)==0) ...
        &&result.source_system==e.host_system&&result.source_component==e.host_component);
    b((j-1)*119+1:(j-1)*119+n)=p.payload(10:9+n);
end
end
function yes=reject(fn,id)
yes=false;try,fn();catch ex,yes=strcmp(ex.identifier,id);end
end
function b=checksum(b)
md=java.security.MessageDigest.getInstance('SHA-256');md.update(typecast(b(1:end-32),'int8'));b(end-31:end)=reshape(typecast(md.digest(),'uint8'),[],1);
end
function b=bigEndianBytes(v)
[~,~,e]=computer;if e=='L',v=swapbytes(v);end,b=reshape(typecast(v(:),'uint8'),[],1);
end
function b=readbytes(p)
f=fopen(p,'rb');assert(f>=0);g=onCleanup(@()fclose(f));b=fread(f,Inf,'*uint8'); %#ok<NASGU>
end
function h=sha(p)
md=java.security.MessageDigest.getInstance('SHA-256');md.update(typecast(readbytes(p),'int8'));
h=string(upper(reshape(dec2hex(typecast(md.digest(),'uint8'),2).',1,[])));
end
function writeJson(p,r)
assert(~isfile(p),'Preserve prior evidence.');f=fopen(p,'w','n','UTF-8');assert(f>=0);g=onCleanup(@()fclose(f)); %#ok<NASGU>
fprintf(f,'%s\n',jsonencode(r,PrettyPrint=true));
end
