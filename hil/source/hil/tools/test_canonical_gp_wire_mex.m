function report=test_canonical_gp_wire_mex(outputRoot,runName)
% Compare the GP wire MEX with 59 retained C wire rows.
% Write results in a new output directory.
arguments
    outputRoot (1,1) string
    runName (1,1) string = "canonical_gp_wire"
end
build=string(fileparts(fileparts(mfilename('fullpath'))));
assert(isfolder(outputRoot)&&~isempty(regexp(char(runName),'^[A-Z0-9_]+$','once')));
out=fullfile(outputRoot,runName);
assert(~isfolder(out)&&~isfile(out),'gpenmpcNative:GpWireEvidenceExists','Preserve prior evidence.');
mkdir(out);oldPath=path;restore=onCleanup(@()restorePath(oldPath)); %#ok<NASGU>
addpath(fullfile(build,'host_runtime'),'-begin');
diary(fullfile(out,'MATLAB_DIARY.txt'));dg=onCleanup(@()diary('off')); %#ok<NASGU>
cc=gpenmpc_install_path('llvm','bin\clang.exe');
cxx=gpenmpc_install_path('llvm','bin\clang++.exe');
mr=string(matlabroot);toolRoot=fullfile(build,'tools');
standalone=fullfile(build,'evidence','gp_predictor');
generated=fullfile(standalone,'generated');library=fullfile(standalone,'libcanonical_gp256.a');
pairsPath=fullfile(build,'rfly_vendor_integration','full_inner_abi', ...
    'snapshot_wire_fixture','RGP1_RGR1_PAIRS.bin');
source=fullfile(toolRoot,'canonical_gp_wire_mex.cpp');
api=fullfile(toolRoot,'canonical_gp_standalone_api.c');
wire=fullfile(build,'rfly_vendor_integration','px4_wire','CanonicalLocalGpWire.hpp');
tracked=[string(mfilename('fullpath'))+'.m';source;api; ...
    fullfile(toolRoot,'canonical_gp_standalone_api.h');library; ...
    fullfile(generated,'gpenmpcNative_canonicalSparseGpFixedInput.c'); ...
    fullfile(generated,'gpenmpcNative_canonicalSparseGpFixedInput.h');wire; ...
    fullfile(build,'rfly_vendor_integration','px4_wire','RflySnapshotWireTypes.hpp'); ...
    fullfile(build,'px4_full_inner','portable','CanonicalPortable.hpp'); ...
    fullfile(build,'px4_full_inner','consumption','ConsumptionBinding.hpp'); ...
    fullfile(build,'px4_full_inner','consumption','CanonicalSha256.hpp');pairsPath];
a=gpenmpcNative.loadCanonicalAssets();
tracked=[tracked;string(which('gpenmpcNative.RflyLocalGpCodec')); ...
    string(which('gpenmpcNative.canonicalSparseGpFixedInput'));string(which('gpenmpcSparseGpPredict'))];
before=arrayfun(@sha,tracked);commands=strings(0,1);buildLog="";
receipt=jsondecode(fileread(fullfile(standalone,'RESULT.json')));
assert(receipt.pass,'gpenmpcNative:GpWireLibrary','Existing standalone library validation must pass.');
assert(strcmpi(sha(library),receipt.library_sha256)&&strcmpi(receipt.model_sha256,a.binding.gp_model_sha256), ...
    'gpenmpcNative:GpWireLibraryIdentity','Existing validated library/model identity must match.');
assert(strcmpi(a.binding.gp_model_sha256, ...
    '4A09E9A3D4818B5555CD3439A6D2133026EEA0FDC2774A1FB17F9E05486E5BB2') ...
    &&isequal(size(a.gp_model.inducing_standardized),[256 17]));
version=fullfile(mr,'extern','version','c_mexapi_version.c');vo=fullfile(out,'mex_version.o');
ao=fullfile(out,'canonical_gp_standalone_api.o');
run(sprintf('"%s" -std=c11 -O2 -DMATLAB_MEX_FILE -DMATLAB_DEFAULT_RELEASE=R2018a -I"%s" -c "%s" -o "%s"', ...
    cc,fullfile(mr,'extern','include'),version,vo));
run(sprintf('"%s" -std=c11 -O2 -ffp-contract=off -fno-fast-math -Wall -Wextra -Werror -I"%s" -I"%s" -c "%s" -o "%s"', ...
    cc,generated,fullfile(mr,'extern','include'),api,ao));
binary=fullfile(out,'canonical_gp_wire_mex.mexw64');
run(sprintf(['"%s" -std=c++14 -O2 -ffp-contract=off -fno-fast-math -shared -static ' ...
    '-Wall -Wextra -Werror -DMATLAB_MEX_FILE -DMATLAB_DEFAULT_RELEASE=R2018a ' ...
    '-I"%s" "%s" "%s" "%s" "%s" "%s" "%s" "%s" -Wl,--no-undefined -o "%s"'], ...
    cxx,fullfile(mr,'extern','include'),source,ao,vo,library, ...
    fullfile(mr,'extern','lib','win64','microsoft','libmex.lib'), ...
    fullfile(mr,'extern','lib','win64','microsoft','libmx.lib'), ...
    fullfile(mr,'extern','lib','win64','mingw64','exportsmexfileversion.def'),binary));
% Reject an in-use binary or conflicting test artifact.
assert(isempty(which('canonical_gp_wire_mex')), ...
    'gpenmpcNative:GpWireAlreadyResolved','An existing GP wire MEX is already on path.');
addpath(out,'-begin');assert(strcmpi(string(which('canonical_gp_wire_mex')),binary));
raw=readbytes(pairsPath);assert(numel(raw)==596*59);pairs=reshape(raw,596,59);
checks=struct('name',{},'pass',{});native=zeros(59,18);matlab=native;fixture=native;
replies=zeros(286,59,'uint8');matlabReplies=replies;
shapeOk=true;metadataOk=true;nativeCodecExact=true;hardInvalidExact=true;
nativeFixtureByteExact=true;
for k=1:59
    request=pairs(1:310,k);expected=pairs(311:596,k);
    q=gpenmpcNative.RflyLocalGpCodec.decodeRequest(request);
    c=gpenmpcNative.RflyLocalGpCodec.decodeReply(expected);
    [reply,y]=canonical_gp_wire_mex(request);
    r=gpenmpcNative.RflyLocalGpCodec.decodeReply(reply);
    ym=gpenmpcNative.canonicalSparseGpFixedInput(a.gp_model,q.request19(2:18).');
    replies(:,k)=reply;native(k,:)=y;matlab(k,:)=ym;fixture(k,:)=c.result18.';
    matlabReplies(:,k)=gpenmpcNative.RflyLocalGpCodec.encodeReply(q,ym);
    shapeOk=shapeOk&&isa(reply,'uint8')&&isequal(size(reply),[286 1]) ...
        &&isa(y,'double')&&isequal(size(y),[1 18])&&isequal(r.result18,y.');
    metadataOk=metadataOk&&isequal(reply(1:110),expected(1:110)) ...
        &&isequal(reply(1:110),matlabReplies(1:110,k)) ...
        &&isequal(r.original_request_sha256,q.original_request_sha256);
    nativeCodecExact=nativeCodecExact&&isequal(reply,gpenmpcNative.RflyLocalGpCodec.encodeReply(q,y));
    hardInvalidExact=hardInvalidExact&&y(15)==ym(15)&&y(15)==c.result18(15);
    nativeFixtureByteExact=nativeFixtureByteExact&&isequal(reply,expected);
end
maxMatlab=max(abs(native-matlab),[],'all');maxFixture=max(abs(native-fixture),[],'all');
check('59_exact_shapes_and_optional_original_output',shapeOk);
check('59_original_identity_tags_model_and_entire_request_digest_exact',metadataOk);
check('59_cpp_reply_codec_bytes_equal_matlab_codec_given_same_numerics',nativeCodecExact);
check('59_original_numerics_absolute_error_le_1e_10',maxMatlab<=1e-10&&maxFixture<=1e-10);
check('59_original_hard_invalid_classifications_exact',hardInvalidExact);
column=pairs(1:310,1);rowReply=canonical_gp_wire_mex(column.');
check('row_and_column_request_identical',isequal(rowReply,replies(:,1)));
badDigest=column;badDigest(279)=bitxor(badDigest(279),uint8(1));
badConfig=column;badConfig(63)=bitxor(badConfig(63),uint8(1));badConfig=checksum(badConfig);
badModel=column;badModel(95)=bitxor(badModel(95),uint8(1));badModel=checksum(badModel);
check('reject_nonbyte_input',reject(@()canonical_gp_wire_mex(double(column)), ...
    'gpenmpcNative:CanonicalGpWireMexShape'));
check('reject_nonvector_310_elements',reject(@()canonical_gp_wire_mex(reshape(column,10,31)), ...
    'gpenmpcNative:CanonicalGpWireMexShape'));
check('reject_wrong_length',reject(@()canonical_gp_wire_mex(column(1:309)), ...
    'gpenmpcNative:CanonicalGpWireMexShape'));
check('reject_corrupt_checksum',reject(@()canonical_gp_wire_mex(badDigest), ...
    'gpenmpcNative:CanonicalGpWireMexRequest'));
check('reject_well_checksummed_wrong_configuration',reject(@()canonical_gp_wire_mex(badConfig), ...
    'gpenmpcNative:CanonicalGpWireMexRequest'));
check('reject_well_checksummed_wrong_model',reject(@()canonical_gp_wire_mex(badModel), ...
    'gpenmpcNative:CanonicalGpWireMexRequest'));
% Test the hard-invalid policy with a finite out-of-distribution input.
ood=column;features=a.gp_model.input_mean+100*a.gp_model.input_scale;
ood(135:270)=bigEndianBytes(features);ood=checksum(ood);
oq=gpenmpcNative.RflyLocalGpCodec.decodeRequest(ood);
om=gpenmpcNative.canonicalSparseGpFixedInput(a.gp_model,oq.request19(2:18).');
[ob,oy]=canonical_gp_wire_mex(ood);od=gpenmpcNative.RflyLocalGpCodec.decodeReply(ob);
check('finite_ood_original_hard_invalid_exact_no_rewrite',oy(15)==1&&om(15)==1 ...
    &&isequal(od.result18,oy.')&&max(abs(oy-om))<=1e-10 ...
    &&isequal(ob,gpenmpcNative.RflyLocalGpCodec.encodeReply(oq,oy)));
% Same-process warmed full-call diagnostic. No per-sample correctness checks,
% disk IO, model comparison or table creation occurs inside either timer.
warmup=8;calls=32;matlabTimes=zeros(calls,1);mexTimes=matlabTimes;
for k=1:warmup
    b=pairs(1:310,k);u=matlabCall(b,a.gp_model);v=canonical_gp_wire_mex(b); %#ok<NASGU>
end
for k=1:calls
    b=pairs(1:310,mod(k-1,59)+1);
    if mod(k,2)==1
        t=tic;u=matlabCall(b,a.gp_model);matlabTimes(k)=toc(t); %#ok<NASGU>
        t=tic;v=canonical_gp_wire_mex(b);mexTimes(k)=toc(t); %#ok<NASGU>
    else
        t=tic;v=canonical_gp_wire_mex(b);mexTimes(k)=toc(t); %#ok<NASGU>
        t=tic;u=matlabCall(b,a.gp_model);matlabTimes(k)=toc(t); %#ok<NASGU>
    end
end
check('all_warmed_diagnostic_times_positive_finite', ...
    all(isfinite(matlabTimes)&matlabTimes>0&isfinite(mexTimes)&mexTimes>0));
after=arrayfun(@sha,tracked);check('production_inputs_and_generated_library_unchanged',isequal(before,after));
save(fullfile(out,'RAW.mat'),'pairs','native','matlab','fixture','replies','matlabReplies', ...
    'ood','ob','oy','om','matlabTimes','mexTimes');
writetable(table((1:calls)',matlabTimes,mexTimes,'VariableNames', ...
    {'call','matlab_codec_predict_encode_s','native_mex_fullcall_s'}),fullfile(out,'HOST_TIMING.csv'));
report=struct('scope','HOST_PURE_RGP1_CPP_CODEC_ORIGINAL_GP256_RGR1_MEX', ...
    'checks',checks,'total',numel(checks),'passed',sum([checks.pass]),'all_pass',all([checks.pass]), ...
    'actual_C_query_reply_rows',59,'additional_finite_ood_probes',1, ...
    'maximum_absolute_error_vs_matlab',maxMatlab,'maximum_absolute_error_vs_fixture',maxFixture, ...
    'numerical_tolerance',1e-10, ...
    'tolerance_provenance','EXISTING_HOST_NUMERICAL_IMPLEMENTATION_COMPARISON_NOT_SAFETY_MARGIN', ...
    'all_native_reply_bytes_equal_C_fixture',nativeFixtureByteExact, ...
    'hard_invalid_exact',hardInvalidExact,'original_identity_and_request_digest_exact',metadataOk, ...
    'model_sha256',a.binding.gp_model_sha256,'gp_inducing_count',256,'precision','double', ...
    'benchmark_calls_each',calls,'benchmark_warmup_calls_each',warmup,'alternating_timing_order',true, ...
    'matlab_fullcall_median_s',median(matlabTimes),'matlab_fullcall_p95_s',p95(matlabTimes), ...
    'matlab_fullcall_max_s',max(matlabTimes),'native_fullcall_median_s',median(mexTimes), ...
    'native_fullcall_p95_s',p95(mexTimes),'native_fullcall_max_s',max(mexTimes), ...
    'timing_claim','SAME_PROCESS_WARMED_HOST_TIMING', ...
    'binary_path',binary,'binary_sha256',sha(binary),'commands',commands, ...
    'tracked',tracked,'before_sha256',before,'after_sha256',after, ...
    'atomic_single_owner_C_API_reused',true, ...
    'socket_count',0,'COM',0,'board',0,'hardware_actions',0,'plant_runs',0,'solver_calls',0);
write(fullfile(out,'RESULT.json'),jsonencode(report,PrettyPrint=true));
fprintf('GP wire MEX: %d/%d checks; maximum error %.17g. Warmed median MATLAB %.6g s, MEX %.6g s.\n', ...
    report.passed,report.total,maxMatlab,report.matlab_fullcall_median_s,report.native_fullcall_median_s);
assert(report.all_pass,'gpenmpcNative:GpWireValidation','Pure MEX validation failed.');
    function run(cmd)
        commands(end+1)=string(cmd);[rc,txt]=system(cmd);buildLog=buildLog+string(txt); %#ok<AGROW>
        write(fullfile(out,'BUILD_LOG.txt'),buildLog);assert(rc==0,'%s',txt);
    end
    function check(n,p)
        checks(end+1)=struct('name',n,'pass',logical(p)); %#ok<AGROW>
        if ~p,fprintf(2,'FAILED %s\n',n);end
    end
end
function b=matlabCall(request,model)
q=gpenmpcNative.RflyLocalGpCodec.decodeRequest(request);
y=gpenmpcNative.canonicalSparseGpFixedInput(model,q.request19(2:18).');
b=gpenmpcNative.RflyLocalGpCodec.encodeReply(q,y);
end
function ok=reject(f,id)
ok=false;try,unused=f(); %#ok<NASGU>
catch ex,ok=strcmp(ex.identifier,id);end
end
function b=checksum(b)
md=java.security.MessageDigest.getInstance('SHA-256');md.update(typecast(b(1:end-32),'int8'));
b(end-31:end)=reshape(typecast(md.digest(),'uint8'),[],1);
end
function b=bigEndianBytes(v)
[~,~,endian]=computer;if endian=='L',v=swapbytes(v);end
b=reshape(typecast(v(:),'uint8'),[],1);
end
function b=readbytes(p)
f=fopen(p,'rb');assert(f>=0);c=onCleanup(@()fclose(f));b=fread(f,Inf,'*uint8'); %#ok<NASGU>
end
function h=sha(p)
b=readbytes(p);md=java.security.MessageDigest.getInstance('SHA-256');md.update(typecast(b,'int8'));
h=string(upper(reshape(dec2hex(typecast(md.digest(),'uint8'),2).',1,[])));
end
function v=p95(t),s=sort(t);v=s(max(1,ceil(.95*numel(s))));end
function write(p,t)
f=fopen(p,'w','n','UTF-8');assert(f>=0);c=onCleanup(@()fclose(f));fprintf(f,'%s\n',t); %#ok<NASGU>
end
function restorePath(p)
clear canonical_gp_wire_mex
path(p);
end
