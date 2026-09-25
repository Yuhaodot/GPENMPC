function report=test_canonical_local_task_wire_mex(fixturePath,outputRoot,runName)
% Compare retained RLI1 bytes with the host codec.
% Accept exporter item fields input647, source382, registered,
% original_source_receive_ns and scope, or cases containing bound, rls,
% originalReceiveNs, outer, registered, expectedBytes and scope.
% Reconstructed bindings are codec fixtures only.
arguments
    fixturePath (1,1) string
    outputRoot (1,1) string
    runName (1,1) string = "task_wire"
end
build=string(fileparts(fileparts(mfilename('fullpath'))));
assert(isfile(fixturePath)&&isfolder(outputRoot)&&~isempty(regexp(char(runName),'^[A-Z0-9_]+$','once')));
out=fullfile(outputRoot,runName);assert(~isfolder(out)&&~isfile(out),'gpenmpcNative:LocalTaskMexEvidenceExists');
mkdir(out);oldPath=path;cleanup=onCleanup(@()restore(oldPath)); %#ok<NASGU>
addpath(fullfile(build,'host_runtime'),fullfile(build,'tools'),'-begin');
diary(fullfile(out,'MATLAB_DIARY.txt'));dg=onCleanup(@()diary('off')); %#ok<NASGU>
source=fullfile(build,'tools','canonical_local_task_wire_mex.cpp');
helper=fullfile(build,'tools','encode_canonical_local_task_mex.m');
vi=fullfile(build,'rfly_vendor_integration');app=fullfile(build,'host_runtime','native_include');
tracked=[source;helper;string(mfilename('fullpath'))+'.m';fixturePath; ...
    string(which('gpenmpcNative.RflyLocalTaskCodec'));string(which('gpenmpcNative.RflyLocalSnapshotDecoder')); ...
    fullfile(vi,'px4_wire','CanonicalLocalTaskWire.hpp');fullfile(vi,'px4_wire','CanonicalLocalGpWire.hpp'); ...
    fullfile(build,'px4_full_inner','consumption','CanonicalSha256.hpp')];
before=arrayfun(@sha,tracked);commands=strings(0,1);buildLog="";
checks=struct('name',{},'pass',{});report=struct('scope','HOST_RETAINED_RLI1_COMPILED_CODEC_NO_NEW_ADMISSION', ...
    'board',0,'COM',0,'sockets',0,'solver_calls',0,'getters_called',0);
try
    [cases,fixtureConstruction]=retainedCases(fixturePath);
    cc=gpenmpc_install_path('llvm','bin\clang.exe');
    cxx=gpenmpc_install_path('llvm','bin\clang++.exe');mr=string(matlabroot);
    version=fullfile(mr,'extern','version','c_mexapi_version.c');vo=fullfile(out,'mex_version.o');
    run(sprintf('"%s" -std=c11 -O2 -DMATLAB_MEX_FILE -DMATLAB_DEFAULT_RELEASE=R2018a -I"%s" -c "%s" -o "%s"', ...
        cc,fullfile(mr,'extern','include'),version,vo));
    binary=fullfile(out,'canonical_local_task_wire_mex.mexw64');
    run(sprintf(['"%s" -std=c++14 -O2 -ffp-contract=off -fno-fast-math -shared -static ' ...
        '-Wall -Wextra -Werror -Wno-address-of-packed-member -DMATLAB_MEX_FILE -DMATLAB_DEFAULT_RELEASE=R2018a ' ...
        '-I"%s" -I"%s" -I"%s" -isystem "%s" -isystem "%s" ' ...
        '"%s" "%s" "%s" "%s" "%s" -Wl,--no-undefined -o "%s"'], ...
        cxx,fullfile(mr,'extern','include'),fullfile(vi,'px4_wire','pump_host_stub'),app, ...
        fullfile(app,'mavlink'),fullfile(app,'mavlink','common'),source,vo, ...
        fullfile(mr,'extern','lib','win64','microsoft','libmex.lib'), ...
        fullfile(mr,'extern','lib','win64','microsoft','libmx.lib'), ...
        fullfile(mr,'extern','lib','win64','mingw64','exportsmexfileversion.def'),binary));
    assert(isempty(which('canonical_local_task_wire_mex')),'gpenmpcNative:LocalTaskMexAlreadyResolved');
    addpath(out,'-begin');assert(strcmpi(string(which('canonical_local_task_wire_mex')),binary));
    backend=@canonical_local_task_wire_mex;
    n=numel(cases);native=zeros(647,n,'uint8');original=native;fragments=cell(n,1);
    for k=1:n
        c=cases(k);assert(isa(c.expectedBytes,'uint8')&&numel(c.expectedBytes)==647&&strlength(string(c.scope))>0);
        [original(:,k),p]=originalCall(c);
        [native(:,k),q]=compiledCall(c,backend);fragments{k}=q;
        check(sprintf('retained_%d_original647_and_six_payloads_byte_exact',k), ...
            isequal(original(:,k),c.expectedBytes(:))&&isequal(native(:,k),c.expectedBytes(:))&&isequal(p,q));
        m=gpenmpcNative.RflyLocalTaskCodec.decode(native(:,k));
        check(sprintf('retained_%d_original_metadata_sensor_and_timestamps_exact',k), ...
            isequal(m.source,c.bound.source)&&isequal(m.original_sensor52,reshape(c.rls(277:328),[],1)) ...
            &&sameOuter(m.outer,c.outer)&&~m.control_authority&&~m.original_timestamps_renewed);
    end
    args=packed(gpenmpcNative.RflyLocalTaskCodec.decode(cases(1).expectedBytes));
    direct=backend(args{:});check('direct_fixed_fields_match_retained_bytes',isequal(direct,native(:,1)));
    bad=args;bad{1}=double(bad{1});check('reject_integer_conversion_through_double', ...
        reject(@()backend(bad{:}),'gpenmpcNative:LocalTaskWireMexShape'));
    bad=args;bad{5}(1,2)=bitxor(bad{5}(1,2),uint8(1));check('reject_changed_canonical_configuration', ...
        reject(@()backend(bad{:}),'gpenmpcNative:LocalTaskWireMexConfiguration'));
    bad=args;bad{1}(19)=bad{1}(18)-uint64(1);check('reject_original_outer_expiry_before_creation', ...
        reject(@()backend(bad{:}),'gpenmpcNative:LocalTaskWireMexFields'));
    wrong=cases(1);wrong.bound.rotor.source.source_generation=wrong.bound.rotor.source.source_generation+uint64(1);
    check('adapter_retains_original_same_source_binding_check', ...
        reject(@()compiledCall(wrong,backend),'gpenmpcNative:LocalTaskSource'));
    % Same inputs/outputs in the same process; include original observation
    % validation, MATLAB field packing, codec and six payload structs on both
    % paths. No isolated pure-MEX timing is presented as end-to-end performance.
    warmup=4;calls=16;matlabSeconds=zeros(calls,1);compiledSeconds=matlabSeconds;
    for k=1:warmup
        c=cases(mod(k-1,n)+1);[a,p]=originalCall(c);[b,q]=compiledCall(c,backend); %#ok<ASGLU>
    end
    for k=1:calls
        c=cases(mod(k-1,n)+1);
        if mod(k,2)
            t=tic;[a,p]=originalCall(c);matlabSeconds(k)=toc(t); %#ok<ASGLU>
            t=tic;[b,q]=compiledCall(c,backend);compiledSeconds(k)=toc(t); %#ok<ASGLU>
        else
            t=tic;[b,q]=compiledCall(c,backend);compiledSeconds(k)=toc(t); %#ok<ASGLU>
            t=tic;[a,p]=originalCall(c);matlabSeconds(k)=toc(t); %#ok<ASGLU>
        end
    end
    check('finite_positive_warmed_full_adapter_diagnostics',all(isfinite(matlabSeconds)&matlabSeconds>0) ...
        &&all(isfinite(compiledSeconds)&compiledSeconds>0));
    after=arrayfun(@sha,tracked);check('original_sources_and_fixture_unchanged',isequal(before,after));
    save(fullfile(out,'RAW.mat'),'cases','native','original','fragments','matlabSeconds','compiledSeconds');
    writetable(table((1:calls)',matlabSeconds,compiledSeconds,'VariableNames', ...
        {'call','matlab_encode_and_fragments_s','compiled_adapter_encode_and_fragments_s'}),fullfile(out,'TIMING.csv'));
    report.cases=n;report.fixture_scope={cases.scope};report.checks=checks;report.passed=all([checks.pass]);
    report.fixture_construction=fixtureConstruction;
    report.total=numel(checks);report.pass_count=sum([checks.pass]);report.binary_path=binary;report.binary_sha256=sha(binary);
    report.matlab_fullcall_median_s=median(matlabSeconds);report.matlab_fullcall_max_s=max(matlabSeconds);
    report.compiled_fullcall_median_s=median(compiledSeconds);report.compiled_fullcall_max_s=max(compiledSeconds);
    report.warmup_each=warmup;report.calls_each=calls;
    report.timing_claim='SAME_PROCESS_WARMED_ADAPTER_CODEC_TIMING';
    report.original_bound_source_checks_retained=true;report.getter_environment_causality_revalidated=false;
    report.tracked=tracked;report.before_sha256=before;report.after_sha256=after;report.commands=commands;
    write(fullfile(out,'RESULT.json'),jsonencode(report,PrettyPrint=true));
    fprintf('RLI codec %d/%d; MATLAB encode+fragments %.6g s, compiled full adapter %.6g s (diagnostic only).\n', ...
        report.pass_count,report.total,report.matlab_fullcall_median_s,report.compiled_fullcall_median_s);
    assert(report.passed,'gpenmpcNative:LocalTaskMexValidation');
catch problem
    report.passed=false;report.failure=getReport(problem,'extended','hyperlinks','off');report.checks=checks;report.commands=commands;
    write(fullfile(out,'FAILURE.json'),jsonencode(report,PrettyPrint=true));rethrow(problem);
end
    function run(command)
        commands(end+1)=string(command);[code,txt]=system(command);buildLog=buildLog+string(txt); %#ok<AGROW>
        write(fullfile(out,'BUILD_LOG.txt'),buildLog);assert(code==0,'%s',txt);
    end
    function check(name,passed)
        checks(end+1)=struct('name',name,'pass',logical(passed)); %#ok<AGROW>
        if ~passed,fprintf(2,'FAILED %s\n',name);end
    end
end
function [cases,construction]=retainedCases(p)
x=load(p);
if isfield(x,'item')
    item=x.item;required={'input647','source382','registered','original_source_receive_ns','scope'};
    assert(isstruct(item)&&isscalar(item)&&all(isfield(item,required)));
    assert(isa(item.input647,'uint8')&&numel(item.input647)==647 ...
        &&isa(item.source382,'uint8')&&numel(item.source382)==382 ...
        &&isa(item.original_source_receive_ns,'uint64')&&isscalar(item.original_source_receive_ns));
    m=gpenmpcNative.RflyLocalTaskCodec.decode(item.input647);
    % Reconstruct codec fixtures from the retained RLI1 fields.
    bound=struct('schema','RFLY_LOCAL_ORIGINAL_INPUT_BINDING_V1', ...
        'numerical_inputs_complete',true,'control_authority',false,'source',m.source, ...
        'leg_index',m.leg_index,'rotor',m.rotor,'payload',m.payload,'wind',m.wind);
    cases=struct('bound',bound,'rls',item.source382(:), ...
        'originalReceiveNs',item.original_source_receive_ns,'outer',m.outer, ...
        'registered',item.registered,'expectedBytes',item.input647(:),'scope',item.scope);
    construction='DECODED_ACTUAL_RETAINED_RLI1_WITH_ORIGINAL_RLS_AND_RECEIVE_NS_CODEC_FIXTURE_ONLY';
else
    assert(isfield(x,'cases')&&~isempty(x.cases));cases=x.cases;
    if iscell(cases),cases=[cases{:}];end
    construction='RETAINED_CALL_FIELDS_SUPPLIED_BY_FIXTURE';
end
required={'bound','rls','originalReceiveNs','outer','registered','expectedBytes','scope'};
assert(isstruct(cases)&&all(isfield(cases,required)));
end
function [b,p]=originalCall(c)
b=gpenmpcNative.RflyLocalTaskCodec.encode(c.bound,c.rls,c.originalReceiveNs,c.outer,c.registered);
p=gpenmpcNative.RflyLocalTaskCodec.fragments(b,c.registered.identity.system,c.registered.identity.component);
end
function [b,p]=compiledCall(c,backend)
[b,p]=encode_canonical_local_task_mex(c.bound,c.rls,c.originalReceiveNs,c.outer,c.registered,backend);
end
function ok=sameOuter(decoded,original)
names=fieldnames(decoded);ok=true;
for k=1:numel(names)
    key=names{k};value=original.(key);if strcmp(key,'target4'),value=value(:);end
    ok=ok&&isequal(decoded.(key),value);
end
end
function args=packed(m)
s=m.source;r=m.rotor.original_observation;o=m.outer;
u=[s.identity.uid;s.identity.boot_generation;s.sample_us;s.publication_us;s.original_receipt_us; ...
    s.source_generation;s.generation_delta;s.sample_delta_us;r.dll_generation;r.dll_session;r.original_host_receive_ns; ...
    m.payload.original_schedule_generation;m.wind.original_estimate_generation;o.generation;o.source_generation; ...
    o.original_sample_us;o.original_host_source_receive_ns;o.original_creation_ns;o.original_expiry_ns];
d=[r.original_sim_time_s;r.observed_thrust_n;m.payload.payload_kg;m.wind.estimate_xy_mps;o.target4];
h=[m.execution_session_sha256,m.configuration_sha256,m.task_sha256,m.reference_asset_sha256,s.state_and_origin_sha256, ...
    r.original_observation_sha,m.rotor.verified_association_receipt_sha256,m.payload.original_schedule_evidence_sha256,m.wind.original_estimate_evidence_sha256];
args={u,m.leg_index,[s.identity.system;s.identity.component;s.reset_counter],d,h,m.original_sensor52};
end
function ok=reject(f,id)
ok=false;try,unused=f(); %#ok<NASGU>
catch problem,ok=strcmp(problem.identifier,id);end
end
function h=sha(p)
f=fopen(p,'rb');assert(f>=0);cl=onCleanup(@()fclose(f));b=fread(f,Inf,'*uint8'); %#ok<NASGU>
m=java.security.MessageDigest.getInstance('SHA-256');m.update(typecast(b,'int8'));
h=string(upper(reshape(dec2hex(typecast(m.digest(),'uint8'),2).',1,[])));
end
function write(p,text)
f=fopen(p,'w','n','UTF-8');assert(f>=0);cl=onCleanup(@()fclose(f));fprintf(f,'%s\n',text); %#ok<NASGU>
end
function restore(p)
clear canonical_local_task_wire_mex
path(p);
end
