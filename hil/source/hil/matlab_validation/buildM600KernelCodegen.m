function report = buildM600KernelCodegen(outputDir)
% Compile actual M600 MATLAB arithmetic to MEX and compare, without hardware.
arguments
    outputDir (1,1) string
end
addpath(fullfile(fileparts(fileparts(mfilename('fullpath'))),'tools'));
assert(~isfolder(outputDir),'m600check:ExistingBuild','No result overwrite.');
mkdir(outputDir); previous=pwd; restore=onCleanup(@()cd(previous)); %#ok<NASGU>
here=string(fileparts(mfilename('fullpath'))); addpath(here);
f=m600check.loadFixture(); p=f.parameters;
x=zeros(19,1);x(7)=1;u=zeros(16,1);jet=zeros(12,1);wind=zeros(2,1);
setenv('MW_MINGW64_LOC',gpenmpc_install_path('llvm',''));
cfg=coder.config('mex');cfg.TargetLang='C++';cfg.GenerateReport=false;
cd(outputDir);
compatibility=struct('used',false,'removed_gcc_only_flags',strings(0,1));
try
codegen('-config',cfg,fullfile(here,'gpenmpcM600CodegenDerivative.m'),...
    '-args',{x,u,jet,0.0,wind,0.0,p},'-d',fullfile(outputDir,'codegen'),...
    '-o','gpenmpc_m600_derivative_mex');
catch failure
    buildFile=fullfile(outputDir,'codegen','build.ninja');
    if ~isfile(buildFile), rethrow(failure); end
    original=fileread(buildFile);
    assert(contains(strrep(original,'\','/'),strrep(gpenmpc_install_path('llvm'),'\','/')) && ...
        contains(original,'-fno-predictive-commoning'),'m600check:UnknownBuildFailure', ...
        'Only the known GCC-only flag incompatibility can be repaired here.');
    % Generated build configuration only; generated arithmetic is untouched.
    copyfile(buildFile,fullfile(outputDir,'codegen','build.gcc_original.ninja'));
    compatible=strrep(original,' -fno-predictive-commoning','');
    fid=fopen(buildFile,'w','n','UTF-8');assert(fid>=0);
    fprintf(fid,'%s',compatible);fclose(fid);
    cd(fullfile(outputDir,'codegen'));
    command=sprintf('"%s" -j 2',gpenmpc_install_path('matlab','toolbox\shared\coder\ninja\win64\ninja.exe'));
    [rc,log]=system(command);
    fid=fopen(fullfile(outputDir,'COMPATIBLE_BUILD.log'),'w','n','UTF-8');assert(fid>=0);
    fprintf(fid,'%s\n%s\n',getReport(failure,'extended','hyperlinks','off'),log);fclose(fid);
    assert(rc==0,'m600check:CompatibleBuildFailed','%s',log);
    copyfile(fullfile(outputDir,'codegen','gpenmpc_m600_derivative_mex.mexw64'),outputDir);
    cd(outputDir);
    compatibility=struct('used',true,'removed_gcc_only_flags',"-fno-predictive-commoning");
end
addpath(outputDir);
maximumDifference=0;
for k=1:96
    x(1:3)=[sin(k);cos(k);-0.25*k];
    x(4:6)=[sin(k);cos(k);sin(.1*k)];
    q=[1;.13*sin(k);.17*cos(k);.23*sin(.7*k)];x(7:10)=q/norm(q);
    x(11:13)=[.1*sin(k);.2*cos(k);.3*sin(.8*k)];
    x(14:19)=p.calibration.rotor_allocation.per_rotor_thrust_upper_n *...
        (.3+.1*sin(k+(1:6).'));
    u(1:6)=.4+.2*cos(.3*k+(1:6).');
    jet(4:9)=[sin(k);cos(k);.1;.2*cos(k);.3*sin(k);.1];
    wind=[.3*sin(k);.4*cos(k)];payload=f.initial_payload_kg*mod(k,5)/4;
    pp=p;pp.mission.structured_residual.enabled=mod(k,2)==0;
    [a,ad,ac,ar]=gpenmpcM600CodegenDerivative(x,u,jet,payload,wind,k*.01,pp);
    [b,bd,bc,br]=gpenmpc_m600_derivative_mex(x,u,jet,payload,wind,k*.01,pp);
    maximumDifference=max(maximumDifference,compare(a,b));
    maximumDifference=max(maximumDifference,compare(ar,br));
    for name=string(fieldnames(ad)).'
        maximumDifference=max(maximumDifference,compare(ad.(name),bd.(name)));
    end
    for name=string(fieldnames(ac)).'
        maximumDifference=max(maximumDifference,compare(ac.(name),bc.(name)));
    end
end
report=struct('status','PASS_GENERATED_CPP_MEX_POINTWISE_EQUIVALENCE',...
    'cases',96,'passed',96,'maximum_absolute_difference',maximumDifference,...
    'source_manifest_sha256',f.source_manifest_sha256,...
    'mex',string(which('gpenmpc_m600_derivative_mex')),...
    'compiler_compatibility',compatibility,...
    'hardware_actions',0,'com_open',0,'simulation_runs',0);
file=fopen(fullfile(outputDir,'CODEGEN_RESULT.json'),'w','n','UTF-8');assert(file>=0);
c=onCleanup(@()fclose(file)); %#ok<NASGU>
fprintf(file,'%s\n',jsonencode(report,PrettyPrint=true));
disp(jsonencode(report,PrettyPrint=true));
end
function e=compare(a,b)
assert(isequal(size(a),size(b)) && all(isfinite(double(a(:)))) && all(isfinite(double(b(:)))));
e=max(abs(double(a(:))-double(b(:))));
assert(e<=1e-10*max(1,max(abs(double(a(:))))),'m600check:MexMismatch','Compiled arithmetic mismatch');
end
