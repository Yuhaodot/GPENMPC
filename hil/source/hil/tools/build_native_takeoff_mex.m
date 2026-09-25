function report=build_native_takeoff_mex(outputDir)
% Build the two PX4 translation units as a host MEX.
arguments
    outputDir (1,1) string
end
assert(~isfolder(outputDir)&&~isfile(outputDir)&&all(double(char(outputDir))<128), ...
    'gpenmpc:TakeoffBuildOutput','Use a new ASCII output directory.');
here=string(fileparts(mfilename('fullpath')));sourceDir=fullfile(here,'native_takeoff_mex');
shared=fullfile(here,'native_lower_loop_mex');
px4=string(gpenmpc_external_path('px4_source_root'));
relative=["src/modules/mc_pos_control/Takeoff/Takeoff.cpp";"src/modules/mc_pos_control/Takeoff/Takeoff.hpp"; ...
    "src/lib/hysteresis/hysteresis.cpp";"src/lib/hysteresis/hysteresis.h";"src/drivers/drv_hrt.h"; ...
    "build/px4_sitl_default/uORB/topics/takeoff_status.h"];
hashes=["1444F131871F31A14E5D568BD71F5F791EB8B7A78E926AA8F04905858F49434B"; ...
    "4D59F66E8B9344F00A6093EA2699E6607D1B55F84A45104DB9D744B9220D525E"; ...
    "3007EE36D755E9DEB77EB44C41DE151F28D3E34C6EC42B5E3C5819A31A05C9E0"; ...
    "01624A6A249408A18B9ABF23B74E9FD224271EF24B6ED6EC230FACDEC983192D"; ...
    "A77091B563B50E049E25671323185CE0F82AB7B28D665FC2D2B1074253CB83C2"; ...
    "C6E829215D76B8E25FFD919FF38A9A7E1973A4A2F433C22EBDD265915A082358"];
bindings=struct('path',{},'bytes',{},'sha256',{});args={};failure='';log='';
report=struct('schema','HOST_ORIGINAL_PX4_TAKEOFF_MEX_BUILD_V1','passed',false, ...
    'compiler_invocation_attempted',false,'original_cpp_translation_units',2, ...
    'source_classes',{{'TakeoffHandling','systemlib::Hysteresis'}},'hardware_actions',0, ...
    'COM_UDP_actions',0,'MEX_runs',0,'flight_admission',false,'binary',struct());
mkdir(outputDir);oldDir=pwd;dg=onCleanup(@()cd(oldDir)); %#ok<NASGU>
oldCompiler=getenv('MW_MINGW64_LOC');cg=onCleanup(@()setenv('MW_MINGW64_LOC',oldCompiler)); %#ok<NASGU>
try
    for k=1:numel(relative)
        id=identity(fullfile(px4,relative(k)));assert(string(id.sha256)==hashes(k),'gpenmpc:TakeoffSourceChanged');append(id);
    end
    support=[fullfile(px4,'src','lib','geo','geo.h'); ...
        fullfile(px4,'platforms','common','include','px4_platform_common','time.h'); ...
        fullfile(px4,'platforms','common','include','px4_platform_common','defines.h'); ...
        fullfile(px4,'platforms','posix','include','queue.h')];
    for file=support.',append(identity(file));end
    for folder=[fullfile(shared,'host_compat');sourceDir;fullfile(px4,'src','lib','mathlib');fullfile(px4,'src','lib','matrix','matrix')].'
        items=dir(fullfile(folder,'**','*'));items=items(~[items.isdir]);
        for k=1:numel(items)
            file=fullfile(items(k).folder,items(k).name);[~,~,ext]=fileparts(file);
            if folder==sourceDir||startsWith(string(file),fullfile(shared,'host_compat'))||any(string(ext)==[".h",".hpp"]),append(identity(file));end
        end
    end
    options=fullfile(shared,'mingw64_gpp_spaced_root.xml');id=identity(options);
    assert(strcmp(id.sha256,'BF1733B4F7A38881A1B7B522AB73C423A31164AD1B7DB68D81F0251B37345603'));append(id);
    compiler=gpenmpc_install_path('gcc','');
    append(identity(fullfile(compiler,'bin','g++.exe')));append(identity(fullfile(compiler,'bin','ld.bfd.exe')));
    append(identity([mfilename('fullpath') '.m']));
    setenv('MW_MINGW64_LOC',compiler);cd(outputDir);
    inc=[sourceDir;fullfile(shared,'host_compat');fullfile(px4,'src');fullfile(px4,'src','lib'); ...
        fullfile(px4,'src','lib','matrix');fullfile(px4,'platforms','common','include'); ...
        fullfile(px4,'platforms','posix','include');fullfile(px4,'build','px4_sitl_default')];
    % Original drv_hrt.h uses GNU typeof; enable the language extension
    % instead of patching that parent source or adding a numeric shim.
    args={'-R2018a','-v','-f',char(options),'CXXFLAGS=$CXXFLAGS -std=gnu++17 -include native_takeoff_host_prelude.hpp','-outdir',char(outputDir),'-output','gpenmpc_px4_native_takeoff_mex'};
    for k=1:numel(inc),args{end+1}=char("-I"+inc(k));end %#ok<AGROW>
    args=[args,{char(fullfile(sourceDir,'native_takeoff_mex.cpp')),char(fullfile(px4,relative(1))),char(fullfile(px4,relative(3)))}];
    report.compiler_invocation_attempted=true;buildProblem=[];buildException=[]; %#ok<NASGU>
    log=evalc('try,mex(args{:});catch buildException,buildProblem=buildException;end');
    if ~isempty(buildProblem),rethrow(buildProblem);end
    for k=1:numel(bindings),assert(isequal(identity(bindings(k).path),bindings(k)),'gpenmpc:TakeoffBuildInputChanged');end
    report.binary=identity(fullfile(outputDir,'gpenmpc_px4_native_takeoff_mex.mexw64'));report.passed=true;
catch e,failure=getReport(e,'extended','hyperlinks','off');end
report.bindings=bindings;report.mex_arguments=string(args);report.failure=failure;
report.clock_policy='Explicit uint64 source drives native hysteresis; per-loop float dt clip[.002,.04] is caller-side mc_pos_control behavior, initial dt is explicit HOST fixture.';
report.exclusions={'No land detector/Commander/HTE/EKF','No private-state instrumentation','Reset reconstructs the HOST kernel instance.'};
write(fullfile(outputDir,'BUILD_LOG.txt'),string(log)+newline+string(failure));write(fullfile(outputDir,'BUILD_RESULT.json'),jsonencode(report,PrettyPrint=true));
assert(report.passed,'gpenmpc:NativeTakeoffBuildFailed','%s',failure);
    function append(id)
        if isempty(bindings)||~any(strcmp({bindings.path},id.path)),bindings(end+1)=id;end
    end
end
function id=identity(file)
file=char(file);f=fopen(file,'rb');assert(f>=0,'gpenmpc:TakeoffMissingFile','Missing %s',file);g=onCleanup(@()fclose(f)); %#ok<NASGU>
b=fread(f,Inf,'*uint8');d=java.security.MessageDigest.getInstance('SHA-256');d.update(b);
id=struct('path',file,'bytes',numel(b),'sha256',upper(reshape(dec2hex(typecast(d.digest(),'uint8'),2).',1,[])));
end
function write(file,value)
f=fopen(file,'w','n','UTF-8');assert(f>=0);g=onCleanup(@()fclose(f)); %#ok<NASGU>
fprintf(f,'%s\n',value);
end
