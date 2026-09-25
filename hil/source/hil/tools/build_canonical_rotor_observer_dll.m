function report=build_canonical_rotor_observer_dll(reuseGenerated)
% Generate a rotor-observer DLL from the typed-output SLX.
% C++ tests use a stub socket; the environment parser test opens no sockets.
arguments
    reuseGenerated (1,1) logical = false
end
build=string(fileparts(fileparts(mfilename('fullpath'))));
modelRoot=fullfile(build,'m600_coptersim');
modelDir=fullfile(modelRoot,'model_source');
outputDir=fullfile(modelRoot,'rotor_observer_dll');
if ~isfolder(outputDir),mkdir(outputDir);end
compiled=fullfile(outputDir,'compiled');if ~isfolder(compiled),mkdir(compiled);end
candidate=fullfile(compiled,'GPENMPC_M600_Canonical.dll');assert(~isfile(candidate),'m600check:CandidateExists');
source=fullfile(modelDir,'GPENMPC_M600_Canonical.slx');params=fullfile(modelDir,'M600_CORE_PARAMETERS.mat');
sourceSha='741C8C1BB08DD8125C2721F15F505698BC59DA8211F78905BCB25F5F754C9763';
parameterSha='D65290884733F07C19C54553BF8D83C703EB88AF5D0AAF110DE56C76FA767AA0';
assert(strcmpi(fileSha(source),sourceSha)&&strcmpi(fileSha(params),parameterSha));
prepared=jsondecode(fileread(fullfile(modelDir,'RESULT.json')));
assert(prepared.passed&&prepared.update_diagram_pass&&prepared.passed_checks==55 ...
    &&prepared.parent_unchanged&&prepared.source_unchanged,'m600check:ObserverPreparation');
for k=1:numel(prepared.source_sha256)
    assert(strcmpi(fileSha(prepared.source_sha256(k).path),prepared.source_sha256(k).sha256), ...
        'm600check:ObserverCoreSourceChanged');
end
history={};previous=struct();
if isfile(fullfile(outputDir,'RESULT.json'))
    previous=jsondecode(fileread(fullfile(outputDir,'RESULT.json')));
    assert(~previous.pass&&strcmp(previous.source_model_sha256,sourceSha),'m600check:FailedBuildIdentity');
    if isfield(previous,'history'),history=previous.history;previous=rmfield(previous,'history');end
    if isstruct(history),history=num2cell(history);end
    history{end+1}=previous;
end
if reuseGenerated
    assert(isfield(previous,'code_generation')&&previous.code_generation,'m600check:NoPriorCodegen');
end
oldPath=path;oldDir=pwd;pg=onCleanup(@()path(oldPath));dg=onCleanup(@()cd(oldDir)); %#ok<NASGU>
addpath(fullfile(build,'matlab_validation'),fullfile(modelRoot,'matlab_validation'),outputDir,'-begin');
m600check.loadFixture();cd(outputDir);
modelFile=fullfile(outputDir,'GPENMPC_M600_Canonical.slx');copyfile(source,modelFile,'f');copyfile(params,outputDir,'f');
evalin('base',"run('"+fullfile(modelRoot,'GPENMPC_M600_CopterSim_init.m')+"')");cd(outputDir);
mdl='GPENMPC_M600_Canonical';assert(~bdIsLoaded(mdl));load_system(modelFile);
mg=onCleanup(@()closeOwned(mdl)); %#ok<NASGU>
report=struct('status','IN_PROGRESS','pass',false,'source_model',source, ...
    'source_model_sha256',sourceSha,'parameters_sha256',parameterSha,'dll',candidate,'dll_sha256','', ...
    'code_generation',false,'dll_compiled',false,'dll_loaded',false,'CopterSim_started',0, ...
    'COM_open',0,'UDP_open',0,'board_actions',0,'original_output_exports_retained',false, ...
    'unique_model_instance',false,'same_owner_post_step_hook',false,'default_disabled',true, ...
    'fixed_localhost_only','127.0.0.1','runtime_signature_or_attestation',false,'failure','', ...
    'history',{history},'code_generation_reused',reuseGenerated);
try
    observerRoot=fullfile(build,'host_runtime','copter_observer');
    compiler=gpenmpc_install_path('llvm','bin\clang++.exe');
    testFiles={'test_rotor_observer','test_rotor_observer_environment'};
    for k=1:2
        exe=fullfile(outputDir,[testFiles{k} '.exe']);
        command=q(compiler)+" -std=c++17 -O2 -Wall -Wextra -Werror -static "+ ...
            q(fullfile(observerRoot,[testFiles{k} '.cpp']))+" -o "+q(exe);
        if k==2,command=command+" -lws2_32";end
        [rc,log]=system(command);writeText(fullfile(outputDir,[testFiles{k} '_COMPILE.log']),log);assert(rc==0,'%s',log);
        [rc,log]=system(q(exe));writeText(fullfile(outputDir,[testFiles{k} '_RESULT.json']),log);assert(rc==0,'%s',log);
        test=jsondecode(log);assert(test.passed==test.total&&test.real_socket_opens==0);report.(testFiles{k})=test;
    end
    encoded=m600check.encodeCanonicalRotorObserver([0;1.25;2.5;10;20;32.145727009134916], ...
        .01,uint64(1),uint64(73),uint8((1:32).'));
    cppBytes=uint8(sscanf(report.test_rotor_observer.packet_hex,'%2x'));
    report.cpp_matlab_packet_byte_identical=isequal(encoded,cppBytes);assert(report.cpp_matlab_packet_byte_identical);
    if ~reuseGenerated
        set_param(mdl,'GenCodeOnly','on','GenerateReport','off');slbuild(mdl);
    end
    report.code_generation=true;
    generated=fullfile(outputDir,'GPENMPC_M600_Canonical_ert_rtw');assert(isfolder(generated));
    generatedHeader=fileread(fullfile(generated,'GPENMPC_M600_Canonical.h'));
    assert(contains(generatedHeader,'outCopterData[32]'));
    typedNames={'RotorObserverStateN','RotorObserverSimTimeS','RotorObserverGeneration', ...
        'RotorObserverSession','RotorObserverValid','RotorObserverFailed'};
    for k=1:numel(typedNames),assert(contains(generatedHeader,typedNames{k}));end
    parentWrapperDir=fullfile(gpenmpc_external_path('canonical_delivery_dll_build'),'compiled');
    oldCpp=fileread(fullfile(parentWrapperDir,'modeldllgen.cpp'));
    oldH=fileread(fullfile(parentWrapperDir,'modeldllgen.h'));
    cpp="#include ""rotor_observer_win32.hpp"""+newline+string(oldCpp);
    globals=join([ ...
        "gpenmpc_rotor_observer::LocalhostSink rotor_sink;"; ...
        "gpenmpc_rotor_observer::Observer<gpenmpc_rotor_observer::LocalhostSink> rotor_observer(rotor_sink);"; ...
        "bool rotor_configuration_read=false;"; ...
        "void rotor_after_step() noexcept {"; ...
        " if(!rotor_configuration_read){ rotor_configuration_read=true; gpenmpc_rotor_observer::Config c;"; ...
        "  if(gpenmpc_rotor_observer::environment_config(c)) rotor_observer.configure(c); else rotor_observer.invalid_environment(); }"; ...
        " const auto &y=mmc.GPENMPC_M600_Canonical_Y; gpenmpc_rotor_observer::Sample s;"; ...
        " for(unsigned k=0;k<6;++k)s.rotor_n[k]=y.RotorObserverStateN[k];"; ...
        " s.sim_time_s=y.RotorObserverSimTimeS;"; ...
        " s.generation=gpenmpc_rotor_observer::from_generated_words(y.RotorObserverGeneration.chunks[0],y.RotorObserverGeneration.chunks[1]);"; ...
        " s.session=gpenmpc_rotor_observer::from_generated_words(y.RotorObserverSession.chunks[0],y.RotorObserverSession.chunks[1]);"; ...
        " s.valid=y.RotorObserverValid;"; ...
        " s.failed=y.RotorObserverFailed || mmc.getRTM()->getErrorStatus()!=nullptr;"; ...
        " rotor_observer.after_step(s);"; ...
        "}"],newline);
    anchor="GPENMPC_M600_Canonical mmc;";assert(count(cpp,anchor)==1);
    cpp=replace(cpp,anchor,anchor+newline+globals);
    assert(count(cpp,'mmc.step();')==1);cpp=replace(cpp,'mmc.step();',"mmc.step();"+newline+"  rotor_after_step();");
    anchor='mmc.terminate(); mmc.~GPENMPC_M600_Canonical(); new (&mmc) GPENMPC_M600_Canonical(); mmc.initialize();';
    assert(count(cpp,anchor)==1);cpp=replace(cpp,anchor,"rotor_observer.before_model_reset();"+newline+string(anchor));
    destroy=sprintf('DLLGEN_EXPORT void DllDestroyModel()\n{\n\tmmc.terminate();');
    assert(count(cpp,destroy)==1,'m600check:ObserverDestroyAnchor');
    cpp=replace(cpp,destroy,sprintf('DLLGEN_EXPORT void DllDestroyModel()\n{\n\trotor_observer.destroy();\n\tmmc.terminate();'));
    statusExport=join([ ...
        "extern ""C"" DLLGEN_EXPORT void DllRotorObserverStatus(uint64_t out[11]) {"; ...
        " if(!out)return;const auto&s=rotor_observer.stats();"; ...
        " const uint64_t v[11]={s.enabled,s.failure,s.open_attempts,s.send_attempts,s.sent_packets,"; ...
        "  s.duplicates,s.invalid_steps_skipped,s.last_observed_generation,s.last_sent_generation,s.close_calls,s.missing_generations};"; ...
        " memcpy(out,v,sizeof(v));"; ...
        "}"],newline);
    cpp=cpp+newline+statusExport;
    h=string(oldH)+newline+"extern ""C"" DLLGEN_EXPORT void DllRotorObserverStatus(uint64_t out[11]);"+newline;
    inverse=replace(cpp,statusExport,'');inverse=replace(inverse,"#include ""rotor_observer_win32.hpp"""+newline,'');
    inverse=replace(inverse,newline+globals,'');inverse=replace(inverse,newline+"  rotor_after_step();",'');
    inverse=replace(inverse,"rotor_observer.before_model_reset();"+newline,'');
    inverse=replace(inverse,sprintf('\trotor_observer.destroy();\n'),'');
    report.original_wrapper_inverse_patch_equal=strcmp(strtrim(inverse),strtrim(oldCpp));assert(report.original_wrapper_inverse_patch_equal);
    report.unique_model_instance=count(cpp,'GPENMPC_M600_Canonical mmc;')==1;
    report.same_owner_post_step_hook=count(cpp,'mmc.step();')==1&&count(cpp,'rotor_after_step();')==1;
    exports={'DlloutHILSensor30d','DlloutHILGPS30d','DlloutVehileInfo60d','DllOutCopterData'};
    report.original_output_exports_retained=all(cellfun(@(n)contains(cpp,n),exports));
    assert(report.unique_model_instance&&report.same_owner_post_step_hook&&report.original_output_exports_retained);
    writeText(fullfile(compiled,'modeldllgen.cpp'),cpp);writeText(fullfile(compiled,'modeldllgen.h'),h);
    copyfile(fullfile(observerRoot,'rotor_observer.hpp'),compiled,'f');
    copyfile(fullfile(observerRoot,'rotor_observer_win32.hpp'),compiled,'f');
    def=fullfile(modelRoot,'matlab_validation','rfly_export_compat.def');copyfile(def,compiled,'f');
    files=dir(fullfile(generated,'**','*.cpp'));paths=string(fullfile({files.folder},{files.name})).';
    paths=paths(~endsWith(paths,'ert_main.cpp')&~endsWith(paths,'modeldllgen.cpp'));
    shared=fullfile(outputDir,'slprj','ert','_sharedutils');
    if isfolder(shared),extra=dir(fullfile(shared,'*.cpp'));paths=[paths;string(fullfile({extra.folder},{extra.name})).'];end
    includes=unique([string(compiled);string(generated);string({files.folder}).';string(shared); ...
        string(fullfile(matlabroot,'simulink','include'));string(fullfile(matlabroot,'extern','include')); ...
        string(fullfile(matlabroot,'rtw','c','src'))]);
    command=q(compiler)+" -std=c++17 -O2 -shared -static -fms-extensions -fdeclspec -Wl,--no-undefined";
    for k=1:numel(includes),if isfolder(includes(k)),command=command+" -I"+q(includes(k));end;end
    for k=1:numel(paths),command=command+" "+q(paths(k));end
    command=command+" "+q(fullfile(compiled,'modeldllgen.cpp'))+" "+q(fullfile(compiled,'rfly_export_compat.def'))+" -lws2_32 -o "+q(candidate);
    writeText(fullfile(outputDir,'COMPILE_COMMAND.txt'),command);
    [rc,log]=system(command);writeText(fullfile(outputDir,'DLL_COMPILE.log'),log);assert(rc==0,'%s',log);
    report.dll_compiled=true;report.dll_sha256=fileSha(candidate);
    report.source_model_unchanged=strcmpi(fileSha(source),sourceSha)&&strcmpi(fileSha(params),parameterSha);
    for k=1:numel(prepared.source_sha256)
        assert(strcmpi(fileSha(prepared.source_sha256(k).path),prepared.source_sha256(k).sha256), ...
            'm600check:ObserverCoreSourceChanged');
    end
    report.wrapper_sha256=fileSha(fullfile(compiled,'modeldllgen.cpp'));
    report.report_test_checks=report.test_rotor_observer.total+report.test_rotor_observer_environment.total+1;
    assert(report.source_model_unchanged);
    report.pass=true;report.status='PASS_HOST_SAME_OWNER_OBSERVER_DLL_CANDIDATE__NOT_LIVE_LOADED';
catch problem
    report.status='HOST_OBSERVER_BUILD_IMPLEMENTATION_FAILURE__NO_HARDWARE';
    report.failure=getReport(problem,'extended','hyperlinks','off');
end
writeText(fullfile(outputDir,'RESULT.json'),jsonencode(report,PrettyPrint=true));disp(jsonencode(report));
assert(report.pass,'m600check:ObserverDllBuild','%s',report.failure);
end
function closeOwned(mdl),if bdIsLoaded(mdl),close_system(mdl,0);end,end
function v=q(v),assert(~contains(v,'"'));v='"'+string(v)+'"';end
function h=fileSha(p)
f=fopen(p,'rb');assert(f>=0);c=onCleanup(@()fclose(f)); %#ok<NASGU>
md=java.security.MessageDigest.getInstance('SHA-256');md.update(uint8(fread(f,Inf,'*uint8')));
h=upper(reshape(dec2hex(typecast(md.digest(),'uint8'),2).',1,[]));
end
function writeText(p,t)
f=fopen(p,'w','n','UTF-8');assert(f>=0);c=onCleanup(@()fclose(f)); %#ok<NASGU>
fprintf(f,'%s\n',t);
end
