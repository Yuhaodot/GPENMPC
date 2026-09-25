function binary=build_gpenmpc_usb_joystick()
% Build the desktop USB input adapter.
here=fileparts(mfilename('fullpath'));
out=fullfile(here,'usb_rc_runtime');if ~isfolder(out),mkdir(out);end
cc=gpenmpc_install_path('llvm','bin\clang.exe');
cxx=gpenmpc_install_path('llvm','bin\clang++.exe');
mr=matlabroot;vo=fullfile(out,'mex_version.o');
binary=fullfile(out,'gpenmpc_usb_joystick_mex.mexw64');
run(sprintf('"%s" -std=c11 -O2 -DMATLAB_MEX_FILE -DMATLAB_DEFAULT_RELEASE=R2018a -I"%s" -c "%s" -o "%s"', ...
    cc,fullfile(mr,'extern','include'),fullfile(mr,'extern','version','c_mexapi_version.c'),vo));
run(sprintf(['"%s" -std=c++14 -O2 -shared -static -Wall -Wextra -Werror ' ...
    '-DMATLAB_MEX_FILE -DMATLAB_DEFAULT_RELEASE=R2018a -I"%s" "%s" "%s" "%s" "%s" "%s" -o "%s"'], ...
    cxx,fullfile(mr,'extern','include'),fullfile(here,'gpenmpc_usb_joystick_mex.cpp'),vo, ...
    fullfile(mr,'extern','lib','win64','microsoft','libmex.lib'), ...
    fullfile(mr,'extern','lib','win64','microsoft','libmx.lib'), ...
    fullfile(mr,'extern','lib','win64','mingw64','exportsmexfileversion.def'),binary));
addpath(out,'-begin');
end
function run(command)
[status,output]=system(command);assert(status==0,'gpenmpcRc:Compile','%s',output);
end
