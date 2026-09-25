function value=gpenmpc_hil_path(relative)
% Resolve a delivery resource relative to the host source root.
root=fileparts(fileparts(mfilename('fullpath')));
value=char(java.io.File(fullfile(root,char(relative))).getCanonicalPath());
assert(startsWith(lower(value),lower([root filesep])), ...
    'gpenmpc:ResourcePath','The resource must remain within the host source tree.');
end
