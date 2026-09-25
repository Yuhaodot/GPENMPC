function report=buildM600IoCoreCodegen(outputDir)
%BUILDM600IOCORECODEGEN Compile and test the complete persistent I/O core.
% HOST-ONLY numerical equivalence, not CopterSim/HIL/mission performance.
arguments
    outputDir (1,1) string
end
addpath(fullfile(fileparts(fileparts(mfilename('fullpath'))),'tools'));
if isfolder(outputDir)
    existing=dir(outputDir);names=string({existing.name});
    assert(all(ismember(names,[".",".."])), ...
        'm600check:ExistingBuild','Only an empty pre-codegen directory can be resumed.');
else
    mkdir(outputDir);
end
previous=pwd;restore=onCleanup(@()cd(previous)); %#ok<NASGU>
here=string(fileparts(mfilename('fullpath')));addpath(here);
fixture=m600check.loadFixture(); %#ok<NASGU> Exact dependency paths/hash checks.
parameterFile=fullfile(gpenmpc_external_path('m600_core_model'),'M600_CORE_PARAMETERS.mat');
loaded=load(parameterFile,'parameters','environment');
assert(all(isfield(loaded,{'parameters','environment'})),'m600check:ParameterSchema','Expected canonical builder fields.');
p=loaded.parameters;environment=loaded.environment;
u=zeros(16,1);position=zeros(3,1);euler=zeros(3,1);
entry=fullfile(here,'gpenmpcM600IoCoreCodegen.m');
core=fullfile(here,'+m600check','copterSimIoCore.m');
coreShaBefore=gpenmpcNative.fileSha256(core);
oldCompiler=getenv('MW_MINGW64_LOC');
compilerRestore=onCleanup(@()setenv('MW_MINGW64_LOC',oldCompiler)); %#ok<NASGU>
setenv('MW_MINGW64_LOC',gpenmpc_install_path('llvm',''));
cfg=coder.config('mex');cfg.TargetLang='C++';cfg.GenerateReport=false;
cd(outputDir);
compatibility=struct('used',false,'removed_gcc_only_flags',strings(0,1));
try
    codegen('-config',cfg,entry,'-args',{u,false,position,euler,environment,p}, ...
        '-d',fullfile(outputDir,'codegen'),'-o','gpenmpc_m600_io_core_mex');
catch failure
    writeText(fullfile(outputDir,'INITIAL_CODEGEN_FAILURE.txt'), ...
        getReport(failure,'extended','hyperlinks','off'));
    buildFile=fullfile(outputDir,'codegen','build.ninja');
    if ~isfile(buildFile),rethrow(failure);end
    original=fileread(buildFile);
    assert(contains(strrep(original,'\','/'),strrep(gpenmpc_install_path('llvm'),'\','/'))&& ...
        contains(original,'-fno-predictive-commoning'),'m600check:UnknownBuildFailure', ...
        'Not the known compiler-flag compatibility failure; preserve exact diagnostic.');
    copyfile(buildFile,fullfile(outputDir,'codegen','build.gcc_original.ninja'));
    writeText(buildFile,strrep(original,' -fno-predictive-commoning',''));
    cd(fullfile(outputDir,'codegen'));
    [rc,buildLog]=system(sprintf('"%s" -j 2',gpenmpc_install_path('matlab','toolbox\shared\coder\ninja\win64\ninja.exe')));
    writeText(fullfile(outputDir,'COMPATIBLE_BUILD.log'),buildLog);
    assert(rc==0,'m600check:CompatibleBuildFailed','%s',buildLog);
    copyfile(fullfile(outputDir,'codegen','gpenmpc_m600_io_core_mex.mexw64'),outputDir);
    cd(outputDir);
    compatibility=struct('used',true,'removed_gcc_only_flags',"-fno-predictive-commoning");
end
addpath(outputDir);
rows=struct('name',{},'maximum_absolute_difference',{},'failed',{},'step_count',{});
maximumDifference=0;
angles=[0,0,0;.2,0,0;0,-.3,0;0,0,.7;-.2,.3,-1.1];
for k=1:size(angles,1)
    for altitude=[0,1]
        euler=angles(k,:).';position=[3;4;-altitude];
        pair(sprintf('reset_angle_%d_altitude_%g',k,altitude),u,true,position,euler,environment);
    end
end
position=[0;0;0];euler=zeros(3,1);
pair('ground_sequence_reset',u,true,position,euler,environment);
for k=1:32
    pair(sprintf('ground_step_%02d',k),u,false,position,euler,environment);
end
position=[0;0;-1];pair('airborne_sequence_reset',u,true,position,euler,environment);
for k=1:32
    u(1:6)=.40+.03*sin(k*.2+(1:6).');
    environment.wind_xy_mps=loaded.environment.wind_xy_mps+[.1*sin(k);.1*cos(k)];
    pair(sprintf('airborne_step_%02d',k),u,false,position,euler,environment);
end
u(1)=NaN;[~,bad]=pair('nonfinite_input_fail_closed',u,false,position,euler,environment);
assert(bad.failed&&~bad.observation_valid&&bad.failure_code==1);
u(1)=0.4;[~,latched]=pair('fault_latched_no_hidden_advance',u,false,position,euler,environment);
assert(latched.failed&&latched.failure_code==1&&latched.sim_time_s==bad.sim_time_s);
near(latched.state_up,bad.state_up,'latched_state');
u=zeros(16,1);[~,reset]=pair('explicit_new_run_reset',u,true,zeros(3,1),euler,environment);
assert(~reset.failed&&reset.sim_time_s==0&&reset.plant_step_count==0);
assert(strcmpi(coreShaBefore,gpenmpcNative.fileSha256(core)),'m600check:ConcurrentCoreEdit','Core changed while compiling.');
report=struct('status','PASS_COMPLETE_IO_CORE_CPP_MEX_EQUIVALENCE', ...
    'cases',numel(rows),'passed',numel(rows),'maximum_absolute_difference',maximumDifference, ...
    'tests',rows,'parameter_path',parameterFile, ...
    'parameter_sha256',gpenmpcNative.fileSha256(parameterFile),'core_sha256',coreShaBefore, ...
    'entry_sha256',gpenmpcNative.fileSha256(entry),'mex_path',string(which('gpenmpc_m600_io_core_mex')), ...
    'mex_sha256',gpenmpcNative.fileSha256(which('gpenmpc_m600_io_core_mex')), ...
    'parameter_environment_role','CANONICAL_NUMERICAL_FIXTURE', ...
    'loaded_environment',loaded.environment,'compiler_compatibility',compatibility, ...
    'hardware_actions',0,'socket_open',0,'com_open',0,'simulator_runs',0);
writeText(fullfile(outputDir,'IO_CORE_CODEGEN_RESULT.json'),jsonencode(report,PrettyPrint=true));
disp(jsonencode(report,PrettyPrint=true));

    function [a,ad]=pair(name,input,isReset,pos,anglesNed,env)
        [a,ad]=gpenmpcM600IoCoreCodegen(input,isReset,pos,anglesNed,env,p);
        [b,bd]=gpenmpc_m600_io_core_mex(input,isReset,pos,anglesNed,env,p);
        delta=max(compareStruct(a,b,'outputs'),compareStruct(ad,bd,'diagnostic'));
        maximumDifference=max(maximumDifference,delta);
        rows(end+1)=struct('name',name,'maximum_absolute_difference',delta, ...
            'failed',ad.failed,'step_count',ad.plant_step_count); %#ok<AGROW>
    end
end

function delta=compareStruct(a,b,label)
assert(isequal(fieldnames(a),fieldnames(b)),'m600check:CoreFieldMismatch','%s field layout differs.',label);
delta=0;names=fieldnames(a);
for k=1:numel(names)
    name=names{k};delta=max(delta,near(a.(name),b.(name),[label '.' name]));
end
end
function delta=near(a,b,label)
assert(strcmp(class(a),class(b))&&isequal(size(a),size(b)), ...
    'm600check:CoreTypeMismatch','%s type/size differs.',label);
if islogical(a)||isinteger(a)
    assert(isequal(a,b),'m600check:CoreDiscreteMismatch','%s discrete value differs.',label);delta=0;return
end
assert(all(isfinite(a(:)))&&all(isfinite(b(:))),'m600check:CoreNonfinite','%s nonfinite.',label);
delta=max(abs(a(:)-b(:)));
assert(delta<=1e-10*max(1,max(abs(a(:)))),'m600check:CoreNumericMismatch','%s differs %.17g.',label,delta);
end
function writeText(path,value)
f=fopen(path,'w','n','UTF-8');assert(f>=0);closer=onCleanup(@()fclose(f)); %#ok<NASGU>
fprintf(f,'%s\n',value);
end
