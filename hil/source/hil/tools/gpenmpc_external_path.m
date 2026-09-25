function value=gpenmpc_external_path(key)
% Resolve an optional external data or tool dependency.
root=fileparts(fileparts(fileparts(fileparts(mfilename('fullpath')))));
addpath(fullfile(root,'assets','canonical','method_source','matlab','utilities'));
value=gpenmpcExternalPath(key);
end
