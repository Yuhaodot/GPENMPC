function report=build_native_sensor_filter_mex(outputDir)
% Build the PX4 sensor filters and host adapter.
arguments,outputDir (1,1) string,end
assert(~isfolder(outputDir)&&~isfile(outputDir),'gpenmpc:ExistingOutput','No overwrite.');
assert(all(double(char(outputDir))<128),'gpenmpc:GnuOutputEncoding','Existing GNU linker requires an ASCII build output path.');
here=string(fileparts(mfilename('fullpath')));gateway=fullfile(here,'native_sensor_filter_mex','native_sensor_filter_mex.cpp');
px4=gpenmpc_external_path('px4_source_root');
headers=["src/lib/mathlib/math/filter/LowPassFilter2p.hpp"; ...
    "src/lib/mathlib/math/filter/AlphaFilter.hpp";"src/lib/mathlib/math/filter/NotchFilter.hpp"];
expected=["AA15173DA259CED0BCB799C1BFA87D11419453EA765DC06850CD31AD968EE296"; ...
    "F5397A18862E2E7FA21AA8DCB589C0737005D6B2D34F53C96D3059692C0BF2D8"; ...
    "B7E2D2143BFDFE371546B5393BD125F25A4141C30D37D92D1F8FAFD06DFA8464"];
bindings=struct('path',{},'bytes',{},'sha256',{});
for k=1:numel(headers)
    bindings(k)=identity(fullfile(px4,headers(k)));assert(strcmpi(bindings(k).sha256,expected(k)));
end
bindings(end+1)=identity(gateway);bindings(end+1)=identity([mfilename('fullpath') '.m']);
options=fullfile(here,'native_lower_loop_mex','mingw64_gpp_spaced_root.xml');bindings(end+1)=identity(options);
compilerRoot=gpenmpc_install_path('gcc','');
bindings(end+1)=identity(fullfile(compilerRoot,'bin','g++.exe'));bindings(end+1)=identity(fullfile(compilerRoot,'bin','ld.bfd.exe'));
mkdir(outputDir);old=pwd;oldCompiler=getenv('MW_MINGW64_LOC');
guard=onCleanup(@()restore(old,oldCompiler)); %#ok<NASGU>
cd(outputDir);setenv('MW_MINGW64_LOC',compilerRoot);
includes=[fullfile(here,'native_lower_loop_mex','host_compat'); ...
    fullfile(string(px4),'src');fullfile(string(px4),'src','lib');fullfile(string(px4),'src','lib','matrix'); ...
    fullfile(string(px4),'platforms','common','include');fullfile(string(px4),'build','px4_sitl_default')];
args={'-R2018a','-v','-f',char(options),'CXXFLAGS=$CXXFLAGS -std=c++17', ...
    '-outdir',char(outputDir),'-output','gpenmpc_px4_sensor_filter_mex'};
for k=1:numel(includes),args{end+1}=char("-I"+includes(k));end %#ok<AGROW>
args{end+1}=char(gateway);failure='';buildLog='';
try
    buildProblem=[];buildLog=evalc('try,mex(args{:});catch buildException,buildProblem=buildException;end');
    if ~isempty(buildProblem),rethrow(buildProblem);end
    binary=identity(fullfile(outputDir,'gpenmpc_px4_sensor_filter_mex.mexw64'));
catch err
    failure=getReport(err,'extended','hyperlinks','off');binary=struct();
end
report=struct('passed',isempty(failure),'failure',failure,'bindings',bindings,'binary',binary, ...
    'classification','HOST_PX4_FILTER_KERNEL_BUILD', ...
    'mex_arguments',string(args),'hardware_actions',0,'COM_actions',0,'runtime_tests_completed',false);
writeText(fullfile(outputDir,'BUILD_LOG.txt'),buildLog+string(failure));
writeText(fullfile(outputDir,'BUILD_RESULT.json'),jsonencode(report,PrettyPrint=true));
disp(struct('passed',report.passed,'failure',failure,'hardware_actions',0));
assert(isempty(failure),'gpenmpc:SensorMexBuild','%s',failure);
end
function restore(folder,compiler),cd(folder);setenv('MW_MINGW64_LOC',compiler);end
function result=identity(path)
path=char(path);assert(isfile(path));f=fopen(path,'rb');assert(f>=0);c=onCleanup(@()fclose(f)); %#ok<NASGU>
b=fread(f,Inf,'*uint8');md=java.security.MessageDigest.getInstance('SHA-256');md.update(b);
result=struct('path',path,'bytes',numel(b),'sha256',upper(reshape(dec2hex(typecast(md.digest(),'uint8'),2).',1,[])));
end
function writeText(path,value)
f=fopen(path,'w','n','UTF-8');assert(f>=0);c=onCleanup(@()fclose(f)); %#ok<NASGU>
fprintf(f,'%s\n',value);
end
