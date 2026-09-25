function report=build_native_position_control_mex(outputDir)
% Build the bound PositionControl kernel as a host MEX.
arguments
    outputDir (1,1) string
end
assert(~isfolder(outputDir)&&~isfile(outputDir),'gpenmpc:ExistingOutput','Use a new output directory.');
here=string(fileparts(mfilename('fullpath')));
sourceDir=fullfile(here,'native_position_control_mex');
shared=fullfile(here,'native_lower_loop_mex');
px4=string(gpenmpc_external_path('px4_source_root'));
generated=fullfile(px4,'build','px4_sitl_default');
relative=["src/modules/mc_pos_control/PositionControl/PositionControl.cpp"; ...
    "src/modules/mc_pos_control/PositionControl/PositionControl.hpp"; ...
    "src/modules/mc_pos_control/PositionControl/ControlMath.cpp"; ...
    "src/modules/mc_pos_control/PositionControl/ControlMath.hpp"; ...
    "src/lib/geo/geo.h"; ...
    "build/px4_sitl_default/uORB/topics/trajectory_setpoint.h"; ...
    "build/px4_sitl_default/uORB/topics/vehicle_attitude_setpoint.h"; ...
    "build/px4_sitl_default/uORB/topics/vehicle_local_position_setpoint.h"];
expected=["7AAC7280D989CE584BB2E7FE179D91C319638CB312C2B2E7F7934492A5EF8A82"; ...
    "0B01D9662F6E54BA959529C620393FDD6A101C402B519C6934C242CDFD4E0076"; ...
    "815DF26DA18EF1FF0E54276B649701924EF5005E5AC9ACB3EDFAF30EC9DE76F2"; ...
    "3E0A30E51DA796A7C04F8DBE13BACE0496752EC11426B2D9736AC1AB1502360D"; ...
    "4DAD0FEEE24043C09E87D7A03A684A1D48D2A575304156BBE640C14E129738D3"; ...
    "C0B78EBE790BCC01F6C78DE728E965A0857758EF8A0AC242B5E208034E967B79"; ...
    "59F04020FE39F0DE1A8BD414DED5CE0DAB53FF7E1FE4CB39DD883AAC87E3946D"; ...
    "B7835796B252DE4765E3A36506A84431EA5282CA7272C9BCD6FFD33129A1FD4B"];
bindings=struct('path',{},'bytes',{},'sha256',{});args={};failure='';buildLog='';
report=struct('schema','HOST_GPENMPC_PX4_POSITIONCONTROL_MEX_BUILD_V1', ...
    'status','HOST_POSITION_MEX_BUILD_NOT_STARTED','binary',struct(), ...
    'compiler_invocation_attempted',false,'original_cpp_translation_units',2, ...
    'kernel_implementation','GPENMPC_POSITION_CONTROL', ...
    'hardware_actions',0,'COM_UDP_actions',0,'simulation_runs',0,'flight_admission',false);
mkdir(outputDir);
oldDirectory=pwd;directoryCleanup=onCleanup(@()cd(oldDirectory)); %#ok<NASGU>
oldCompiler=getenv('MW_MINGW64_LOC');compilerCleanup=onCleanup(@()setenv('MW_MINGW64_LOC',oldCompiler)); %#ok<NASGU>
try
    for k=1:numel(relative)
        id=identity(fullfile(px4,relative(k)));
        assert(string(id.sha256)==expected(k),'gpenmpc:PositionSourceChanged', ...
            'Bound original numerical source/header changed: %s',id.path);
        append(id);
    end
    % Record the numerical headers and shared host compatibility shims.
    for folder=[fullfile(px4,'src','lib','matrix','matrix');fullfile(px4,'src','lib','mathlib'); ...
            fullfile(shared,'host_compat');sourceDir].'
        items=dir(fullfile(folder,'**','*'));items=items(~[items.isdir]);
        for k=1:numel(items)
            path=fullfile(items(k).folder,items(k).name);
            [~,~,ext]=fileparts(path);
            if folder==sourceDir||startsWith(string(path),fullfile(shared,'host_compat')) ...
                    ||any(string(ext)==[".h",".hpp"])
                append(identity(path));
            end
        end
    end
    append(identity(fullfile(px4,'platforms','common','include','px4_platform_common','defines.h')));
    options=fullfile(shared,'mingw64_gpp_spaced_root.xml');
    optionIdentity=identity(options);
    assert(strcmp(optionIdentity.sha256,'BF1733B4F7A38881A1B7B522AB73C423A31164AD1B7DB68D81F0251B37345603'), ...
        'gpenmpc:PositionCompilerOptions','Existing GNU spaced-root options changed.');
    append(optionIdentity);
    compilerRoot=gpenmpc_install_path('gcc','');
    append(identity(fullfile(compilerRoot,'bin','g++.exe')));
    append(identity(fullfile(compilerRoot,'bin','ld.bfd.exe')));
    append(identity(fullfile(matlabroot,'bin','win64','mexopts','mingw64_g++.xml')));
    append(identity([mfilename('fullpath') '.m']));
    setenv('MW_MINGW64_LOC',compilerRoot);cd(outputDir);
    includes=[fullfile(shared,'host_compat');fullfile(px4,'src');fullfile(px4,'src','lib'); ...
        fullfile(px4,'src','lib','matrix');fullfile(px4,'platforms','common','include'); ...
        generated;fullfile(px4,'src','modules','mc_pos_control','PositionControl')];
    args={'-R2018a','-v','-f',char(options),'CXXFLAGS=$CXXFLAGS -std=c++17', ...
        '-outdir',char(outputDir),'-output','gpenmpc_px4_native_position_control_mex'};
    for k=1:numel(includes),args{end+1}=char("-I"+includes(k));end %#ok<AGROW>
    args{end+1}=char(fullfile(sourceDir,'native_position_control_mex.cpp'));
    args{end+1}=char(fullfile(px4,relative(1)));
    args{end+1}=char(fullfile(px4,relative(3)));
    report.compiler_invocation_attempted=true;buildProblem=[];buildException=[]; %#ok<NASGU>
    buildLog=evalc('try, mex(args{:}); catch buildException, buildProblem=buildException; end');
    if ~isempty(buildProblem),rethrow(buildProblem);end
    % Detect concurrent source changes rather than reporting a mixed build.
    for k=1:numel(bindings)
        after=identity(bindings(k).path);
        assert(isequal(after,bindings(k)),'gpenmpc:PositionBuildInputChanged','Input changed during compilation: %s',after.path);
    end
    binary=fullfile(outputDir,'gpenmpc_px4_native_position_control_mex.mexw64');
    report.binary=identity(binary);report.status='PASS_HOST_POSITIONCONTROL_CPP_MEX_BUILD_ONLY';
catch problem
    failure=getReport(problem,'extended','hyperlinks','off');
    report.status='HOST_POSITIONCONTROL_MEX_BUILD_FAILED';
end
report.bindings=bindings;report.mex_arguments=string(args);report.failure=failure;
report.exact_classes={'PositionControl','ControlMath free functions'};
report.shared_host_compatibility_path=char(fullfile(shared,'host_compat'));
report.parameter_policy='ALL_PARAMETERS_EXPLICIT';
report.exclusions={'The gateway executes the PositionControl kernel.', ...
    'EKF, sensor filters, hover-thrust estimation, takeoff/landing and Commander are outside this kernel.', ...
    'Per-step vertical I and terminal-inhibit inputs are caller-supplied phase values, not automatic phase inference', ...
    'Hover thrust uses explicit setHoverThrust/updateHoverThrust calls; module-level integrator slew, trajectories and transport are handled separately.', ...
    'Uses the GPENMPC PositionControl implementation.'};
writeText(fullfile(outputDir,'BUILD_LOG.txt'),string(buildLog)+newline+string(failure));
writeText(fullfile(outputDir,'BUILD_RESULT.json'),jsonencode(report,PrettyPrint=true));
disp(struct('status',report.status,'original_cpp_translation_units',2,'hardware_actions',0,'failure',failure));
assert(isempty(failure),'gpenmpc:NativePositionBuildFailed','%s',failure);
    function append(id)
        if isempty(bindings)||~any(strcmp({bindings.path},id.path)),bindings(end+1)=id;end
    end
end
function r=identity(file)
file=char(file);assert(isfile(file),'gpenmpc:PositionMissingSource','Missing %s',file);
fid=fopen(file,'rb');assert(fid>=0);cleanup=onCleanup(@()fclose(fid)); %#ok<NASGU>
bytes=fread(fid,Inf,'*uint8');md=java.security.MessageDigest.getInstance('SHA-256');md.update(bytes);
r=struct('path',file,'bytes',numel(bytes), ...
    'sha256',upper(reshape(dec2hex(typecast(md.digest(),'uint8'),2).',1,[])));
end
function writeText(file,value)
fid=fopen(file,'w','n','UTF-8');assert(fid>=0);cleanup=onCleanup(@()fclose(fid)); %#ok<NASGU>
fprintf(fid,'%s\n',value);
end
