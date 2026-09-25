function report = build_native_lower_loop_mex(outputDir)
% Build five PX4 numerical sources and the MATLAB gateway.
arguments
    outputDir (1,1) string
end
assert(~isfolder(outputDir),'gpenmpc:ExistingOutput','Use a new output directory.');
here=string(fileparts(mfilename('fullpath')));
sourceDir=fullfile(here,'native_lower_loop_mex');
px4=gpenmpc_external_path('px4_source_root');
relative=["src/modules/mc_att_control/AttitudeControl/AttitudeControl.cpp";...
    "src/lib/rate_control/rate_control.cpp";...
    "src/lib/control_allocation/control_allocation/ControlAllocation.cpp";...
    "src/lib/control_allocation/control_allocation/ControlAllocationPseudoInverse.cpp";...
    "src/lib/control_allocation/control_allocation/ControlAllocationSequentialDesaturation.cpp"];
expected=["25733BEF293C2670E3D327D8830C91F9665F226606CF33F13DD42FB91095E03C";...
    "446012432C15E5CA49038E73796BA847EB611ABBE02F003BD8B198275F8DC960";...
    "E16EABD67D9C6ECB1D171017A662BA16196251E225E188BD6EBDCC83969F939F";...
    "3200E76A8A150C0DD514C8E4008D7392729BCA53BF42E71151A74D364C803499";...
    "07E60BC11A9948D41C99950FD99FFF06368BB6203F05B79D64386A0A04351D7C"];
sources=fullfile(string(px4),relative); bindings=struct('path',{},'bytes',{},'sha256',{});
for k=1:numel(sources)
    bindings(k)=identity(sources(k)); %#ok<AGROW>
    assert(string(bindings(k).sha256)==expected(k),'gpenmpc:SourceChanged','Original PX4 source changed: %s',sources(k));
end
% Bind all local compatibility/gateway inputs; none shadows controller math.
local=dir(fullfile(sourceDir,'**','*'));local=local(~[local.isdir]);
for k=1:numel(local)
    bindings(end+1)=identity(fullfile(local(k).folder,local(k).name)); %#ok<AGROW>
end
generated=fullfile(px4,'build','px4_sitl_default');
required=[string(fullfile(generated,'uORB','topics','rate_ctrl_status.h'));...
    string(fullfile(generated,'uORB','topics','control_allocator_status.h'))];
for k=1:numel(required),bindings(end+1)=identity(required(k));end %#ok<AGROW>
% R2026a mex uses GNU ld --format=binary/default for its bundle resource.
% Use GNU UCRT rather than LLVM lld and rebuild all translation units together.
compilerRoot=gpenmpc_install_path('gcc','');
compiler=fullfile(compilerRoot,'bin','g++.exe');assert(isfile(compiler));
vendorOptions=fullfile(matlabroot,'bin','win64','mexopts','mingw64_g++.xml');assert(isfile(vendorOptions));
options=fullfile(sourceDir,'mingw64_gpp_spaced_root.xml');assert(isfile(options));
bindings(end+1)=identity(vendorOptions);bindings(end+1)=identity(compiler);
bindings(end+1)=identity(fullfile(compilerRoot,'bin','ld.bfd.exe'));
bindings(end+1)=identity([mfilename('fullpath') '.m']);
mkdir(outputDir);old=pwd;cdGuard=onCleanup(@()cd(old)); %#ok<NASGU>
oldCompiler=getenv('MW_MINGW64_LOC');envGuard=onCleanup(@()setenv('MW_MINGW64_LOC',oldCompiler)); %#ok<NASGU>
setenv('MW_MINGW64_LOC',compilerRoot);cd(outputDir);
includes=[fullfile(sourceDir,'host_compat');fullfile(string(px4),'src');...
    fullfile(string(px4),'src','lib');fullfile(string(px4),'src','lib','matrix');...
    fullfile(string(px4),'platforms','common','include');string(generated)];
for k=1:numel(sources),includes(end+1)=string(fileparts(sources(k)));end %#ok<AGROW>
includes=unique(includes,'stable');
args={'-R2018a','-v','-f',char(options),...
    'CXXFLAGS=$CXXFLAGS -std=c++17','-outdir',char(outputDir),...
    '-output','gpenmpc_px4_native_lower_loop_mex'};
for k=1:numel(includes),args{end+1}=char("-I"+includes(k));end %#ok<AGROW>
args{end+1}=char(fullfile(sourceDir,'native_lower_loop_mex.cpp'));
for k=1:numel(sources),args{end+1}=char(sources(k));end %#ok<AGROW>
failure='';buildLog='';
try
    buildProblem=[];
    buildLog=evalc('try, mex(args{:}); catch buildException, buildProblem=buildException; end');
    if ~isempty(buildProblem),rethrow(buildProblem);end
    binary=fullfile(outputDir,'gpenmpc_px4_native_lower_loop_mex.mexw64');assert(isfile(binary));
    report=struct('status','PASS_HOST_CPP_MEX_BUILD_ONLY','binary',identity(binary));
catch err
    failure=getReport(err,'extended','hyperlinks','off');
    report=struct('status','HOST_MEX_BUILD_FAILED','binary',struct());
end
report.bindings=bindings;report.mex_arguments=string(args);
report.parameter_policy='MISSING_PARAMETERS_REJECTED';
report.original_cpp_translation_units=5;
report.exact_classes={'AttitudeControl','RateControl','ControlAllocationSequentialDesaturation'};
report.host_adapters={'OS header declarations only','generated uORB structs without bus',...
    'immutable MC_AIRMODE=0 parameter adapter'};
report.hardware_actions=0;report.com_open=0;report.simulation_runs=0;
report.closed_loop_performance_pass=false;report.failure=failure;
writeText(fullfile(outputDir,'BUILD_LOG.txt'),buildLog+string(failure));
writeText(fullfile(outputDir,'BUILD_RESULT.json'),jsonencode(report,PrettyPrint=true));
disp(struct('status',report.status,'original_cpp_translation_units',5, ...
    'hardware_actions',0,'failure',failure));
assert(isempty(failure),'gpenmpc:NativeMexBuildFailed','%s',failure);
end

function result=identity(path)
path=char(path);assert(isfile(path),'gpenmpc:MissingSource','%s',path);
f=fopen(path,'rb');assert(f>=0);c=onCleanup(@()fclose(f)); %#ok<NASGU>
bytes=fread(f,Inf,'*uint8');md=java.security.MessageDigest.getInstance('SHA-256');
md.update(bytes);digest=typecast(md.digest(),'uint8');
result=struct('path',path,'bytes',numel(bytes),'sha256',upper(reshape(dec2hex(digest,2).',1,[])));
end
function writeText(path,text)
f=fopen(path,'w','n','UTF-8');assert(f>=0);c=onCleanup(@()fclose(f)); %#ok<NASGU>
fprintf(f,'%s\n',text);
end
