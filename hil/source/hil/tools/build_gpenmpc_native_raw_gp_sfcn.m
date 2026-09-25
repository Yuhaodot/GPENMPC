function receipt=build_gpenmpc_native_raw_gp_sfcn(out)
% Build the retained-GP fixture S-function.
arguments
    out (1,1) string
end
assert(isfolder(out),'Existing isolated test output directory required.');
build=string(fileparts(fileparts(mfilename('fullpath'))));
source=fullfile(build,'tools','gpenmpc_native_raw_gp_sfcn.cpp');
api=fullfile(build,'tools','canonical_gp_standalone_api.c');
standalone=fullfile(build,'evidence','gp_predictor');
library=fullfile(standalone,'libcanonical_gp256.a');generated=fullfile(standalone,'generated');
prior=jsondecode(fileread(fullfile(standalone,'RESULT.json')));
assert(prior.pass&&strcmpi(sha(library),prior.library_sha256),'Validated original GP archive mismatch.');
assert(strcmpi(prior.model_sha256,'4A09E9A3D4818B5555CD3439A6D2133026EEA0FDC2774A1FB17F9E05486E5BB2'));
cc=gpenmpc_install_path('llvm','bin\clang.exe');
cxx=gpenmpc_install_path('llvm','bin\clang++.exe');
ao=fullfile(out,'raw_gp_original_api.o');vo=fullfile(out,'raw_gp_mex_version.o');
binary=fullfile(out,'gpenmpc_native_raw_gp_sfcn.mexw64');
assert(~isfile(ao)&&~isfile(vo)&&~isfile(binary),'Preserve existing objects and binaries.');
commands=strings(0,1);logs=strings(0,1);
run(sprintf('"%s" -std=c11 -O2 -DMATLAB_MEX_FILE -DMATLAB_DEFAULT_RELEASE=R2018a -I"%s" -c "%s" -o "%s"', ...
    cc,fullfile(matlabroot,'extern','include'),fullfile(matlabroot,'extern','version','c_mexapi_version.c'),vo));
run(sprintf('"%s" -std=c11 -O2 -ffp-contract=off -fno-fast-math -Wall -Wextra -Werror -I"%s" -I"%s" -c "%s" -o "%s"',cc,generated,fullfile(matlabroot,'extern','include'),api,ao));
run(sprintf(['"%s" -std=c++14 -O2 -ffp-contract=off -fno-fast-math -shared -static ' ...
    '-Wall -Wextra -Werror -Wno-address-of-packed-member -Wno-unused-parameter -Wno-missing-field-initializers ' ...
    '-DMATLAB_MEX_FILE -DMATLAB_DEFAULT_RELEASE=R2018a -I"%s" -I"%s" ' ...
    '"%s" "%s" "%s" "%s" "%s" "%s" "%s" -Wl,--no-undefined -o "%s"'], ...
    cxx,fullfile(matlabroot,'extern','include'),fullfile(matlabroot,'simulink','include'), ...
    source,ao,vo,library,fullfile(matlabroot,'extern','lib','win64','microsoft','libmex.lib'), ...
    fullfile(matlabroot,'extern','lib','win64','microsoft','libmx.lib'), ...
    fullfile(matlabroot,'extern','lib','win64','mingw64','exportsmexfileversion.def'),binary));
receipt=struct('scope','COMPILED_RETAINED_GP_ONLY_BUILD_NO_EXECUTION','source',source,'source_sha256',sha(source), ...
    'binary',binary,'binary_sha256',sha(binary),'original_gp_library_sha256',sha(library), ...
    'commands',commands,'logs',logs,'COM',0,'board',0,'model_runs',0,'sockets',0,'plant',0,'controller',0);
    function run(command)
        commands(end+1,1)=command;[rc,t]=system(command);logs(end+1,1)=string(t);
        assert(rc==0,'%s',t);
    end
end
function h=sha(p)
f=fopen(p,'rb');assert(f>=0);g=onCleanup(@()fclose(f));b=fread(f,Inf,'*uint8'); %#ok<NASGU>
md=java.security.MessageDigest.getInstance('SHA-256');md.update(typecast(b,'int8'));
h=upper(reshape(dec2hex(typecast(md.digest(),'uint8'),2).',1,[]));
end
