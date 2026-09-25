function report=build_canonical_generated_closure_mex(outputRoot)
% Compile the Cortex-M generated C as a host numerical MEX.
arguments
    outputRoot (1,1) string
end
build=string(fileparts(fileparts(mfilename('fullpath'))));
generated=fullfile(build,'evidence','arm_controller', ...
    'GPENMPC_Rfly_Canonical_Controller_ert_rtw');
assert(~isfolder(outputRoot),'Preserve existing evidence.');mkdir(outputRoot);
old=path;pg=onCleanup(@()path(old));addpath(fullfile(build,'host_runtime'),'-begin'); %#ok<NASGU>
oldDir=pwd;dg=onCleanup(@()cd(oldDir)); %#ok<NASGU>
cCompiler=gpenmpc_install_path('llvm','bin\clang.exe');
cppCompiler=gpenmpc_install_path('llvm','bin\clang++.exe');
matlabRoot=gpenmpc_install_path('matlab','');cd(outputRoot);
files=["GPENMPC_Rfly_Canonical_Controller.c","rt_nonfinite.c","rtGetInf.c"];
files=[fullfile(generated,files),fullfile(build,'tools','canonical_generated_closure_bridge.c')];
inputs=struct('path',{},'sha256',{});objects=cell(1,4);commands=strings(0,1);log='';
for k=1:numel(files)
    source=files(k);
    inputs(end+1)=identity(source); %#ok<AGROW>
    if k==1,assert(strcmp(inputs(end).sha256,'A47F1C255BDAC1DAE712494BE8D9D66FC4F83138DBA9B0C2E3A31E114D4ABD0F'));end
    objects{k}=char(fullfile(outputRoot,"generated_part_"+k+".o"));
    command=sprintf('"%s" -std=c11 -O2 -ffp-contract=off -fno-fast-math -Wall -Wextra -Werror -I"%s" -c "%s" -o "%s"', ...
        cCompiler,generated,source,objects{k});
    commands(end+1)=string(command);[rc,out]=system(command);log=[log out]; %#ok<AGROW>
    assert(rc==0,'gpenmpc:GeneratedClosureCBuild','%s',out);
end
gateway=fullfile(build,'tools','canonical_generated_closure_mex.cpp');
headers=dir(fullfile(generated,'*.h'));
for k=1:numel(headers),inputs(end+1)=identity(fullfile(headers(k).folder,headers(k).name));end %#ok<AGROW>
inputs(end+1)=identity(gateway);inputs(end+1)=identity(cCompiler);inputs(end+1)=identity(cppCompiler);
versionSource=fullfile(matlabRoot,'extern','version','c_mexapi_version.c');
inputs(end+1)=identity(versionSource);
versionObject=fullfile(outputRoot,'c_mexapi_version.o');
command=sprintf('"%s" -std=c11 -O2 -DMATLAB_MEX_FILE -DMATLAB_DEFAULT_RELEASE=R2018a -I"%s" -c "%s" -o "%s"', ...
    cCompiler,fullfile(matlabRoot,'extern','include'),versionSource,versionObject);
commands(end+1)=string(command);[rc,out]=system(command);log=[log out];assert(rc==0,'%s',out);
objects{end+1}=char(versionObject);
libraries=[string(fullfile(matlabRoot,'extern','lib','win64','microsoft','libmex.lib')), ...
    string(fullfile(matlabRoot,'extern','lib','win64','microsoft','libmx.lib'))];
for k=1:numel(libraries),inputs(end+1)=identity(libraries(k));end %#ok<AGROW>
definition=fullfile(matlabRoot,'extern','lib','win64','mingw64','exportsmexfileversion.def');inputs(end+1)=identity(definition);
args=strjoin('"'+string([objects cellstr(libraries)])+'"',' ');
command=sprintf('"%s" -std=c++14 -O2 -ffp-contract=off -fno-fast-math -shared -static -Wall -Wextra -Werror -DMATLAB_MEX_FILE -DMATLAB_DEFAULT_RELEASE=R2018a -I"%s" -I"%s" "%s" %s "%s" -Wl,--no-undefined -o "%s"', ...
    cppCompiler,fullfile(matlabRoot,'extern','include'),generated,gateway,args,definition, ...
    fullfile(outputRoot,'canonical_generated_closure_mex.mexw64'));
commands(end+1)=string(command);[rc,out]=system(command);log=[log out];assert(rc==0,'%s',out);
for k=1:numel(inputs),assert(isequal(inputs(k),identity(inputs(k).path)));end
report=struct('schema','UNCHANGED_ARM_GENERATED_C_HOST_MEX_V1','pass',true, ...
    'inputs',inputs,'compile_and_link_commands',commands, ...
    'binary',identity(fullfile(outputRoot,'canonical_generated_closure_mex.mexw64')), ...
    'generated_C_translation_units',3, ...
    'toolchain_repairs',{{'Initial build helper resolved missing standalone SHA utility', ...
        'Existing MinGW C subprocess exited without diagnostics; existing project clang C compiler selected', ...
        'MATLAB char-vector library concatenation corrected to string array', ...
        'Official mex.h and generated rtwtypes separated by a pure copy C ABI bridge'}}, ...
    'model_execution',0,'board_actions',0,'COM_UDP_actions',0);
write(fullfile(outputRoot,'BUILD_LOG.txt'),log);
write(fullfile(outputRoot,'BUILD_RESULT.json'),jsonencode(report,PrettyPrint=true));
disp(jsonencode(report));
end
function id=identity(file)
f=fopen(file,'rb');assert(f>=0);g=onCleanup(@()fclose(f)); %#ok<NASGU>
b=fread(f,Inf,'*uint8');d=java.security.MessageDigest.getInstance('SHA-256');d.update(b);
id=struct('path',char(file),'sha256',upper(reshape(dec2hex(typecast(d.digest(),'uint8'),2).',1,[])));
end
function write(file,value)
f=fopen(file,'w','n','UTF-8');assert(f>=0);g=onCleanup(@()fclose(f)); %#ok<NASGU>
fprintf(f,'%s\n',value);
end
