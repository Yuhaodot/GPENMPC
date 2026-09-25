function value=gpenmpcExternalPath(key)
% Resolve an optional external data or tool dependency.
path=getenv('GPENMPC_EXTERNAL_PATHS');
assert(~isempty(path)&&isfile(path),'gpenmpc:ExternalPaths', ...
 'Set GPENMPC_EXTERNAL_PATHS to the dependency configuration JSON.');
configuration=jsondecode(fileread(path));key=char(string(key));
assert(isstruct(configuration)&&isscalar(configuration)&&isfield(configuration,key), ...
 'gpenmpc:ExternalPathKey','Configure dependency %s.',key);
value=string(configuration.(key));
assert(isscalar(value)&&strlength(value)>0,'gpenmpc:ExternalPathValue', ...
 'Dependency %s must be a nonempty path.',key);
value=char(value);
end
