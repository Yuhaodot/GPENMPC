function value=gpenmpc_install_path(kind,relative)
% Resolve an installed tool or SDK from its configured root.
arguments
    kind (1,1) string
    relative (1,1) string = ""
end
switch kind
    case "matlab"
        root=string(matlabroot);variable="MATLAB installation";
    case "llvm"
        variable="GPENMPC_LLVM_ROOT";root=string(getenv(variable));
        if strlength(root)==0,root=string(getenv('MW_MINGW64_LOC'));end
    case "gcc"
        variable="GPENMPC_GCC_ROOT";root=string(getenv(variable));
        if strlength(root)==0,root=string(getenv('MW_MINGW64_LOC'));end
    case "rfly"
        variable="GPENMPC_RFLY_ROOT";root=string(getenv(variable));
    otherwise
        error('gpenmpc:InstallationKind','Unknown installation: %s',kind);
end
assert(strlength(root)>0&&isfolder(root),'gpenmpc:InstallationRoot', ...
    'Configure %s with an existing installation directory.',variable);
root=string(java.io.File(char(root)).getCanonicalPath());
value=string(java.io.File(char(fullfile(root,relative))).getCanonicalPath());
assert(strcmpi(value,root)||startsWith(lower(value),lower(root+filesep)), ...
    'gpenmpc:InstallationPath','Tool path must remain within the configured installation.');
assert(isfile(value)||isfolder(value),'gpenmpc:InstallationMissing', ...
    'Required tool or SDK path was not found: %s',value);
value=char(value);
end
