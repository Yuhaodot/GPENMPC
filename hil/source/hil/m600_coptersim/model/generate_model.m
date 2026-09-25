function generate_model()
% Generate C++ sources for the M600 CopterSim model.
here=string(fileparts(mfilename('fullpath')));
root=string(fileparts(fileparts(here)));
parent=fullfile(root,'m600_coptersim','model_source');
out=fullfile(here,'generated');
assert(~isfile(fullfile(out,'GPENMPC_M600_Canonical_ert_rtw','GPENMPC_M600_Canonical.cpp')));
if ~isfolder(out),mkdir(out);end
addpath(fullfile(root,'host_runtime'),fullfile(root,'matlab_validation'), ...
    fullfile(root,'m600_coptersim','matlab_validation'));
m600check.loadFixture();
copyfile(fullfile(parent,'GPENMPC_M600_Canonical.slx'),out);
copyfile(fullfile(parent,'M600_CORE_PARAMETERS.mat'),out);
evalin('base',"run('"+fullfile(root,'m600_coptersim','GPENMPC_M600_CopterSim_init.m')+"')");
cd(out);addpath(out);
mdl='GPENMPC_M600_Canonical';load_system(fullfile(out,[mdl '.slx']));
cleanup=onCleanup(@()close_system(mdl,0)); %#ok<NASGU>
set_param(mdl,'GenCodeOnly','on','GenerateReport','off');
slbuild(mdl);
fprintf('M600 model source generated.\n');
end
