function root=gpenmpc_log_root()
% Return the configured session-recording directory.
root=getenv('GPENMPC_HIL_LOG_ROOT');
if isempty(root)
    distribution=fileparts(fileparts(fileparts(fileparts(mfilename('fullpath')))));
    root=fullfile(distribution,'logs');
end
root=char(java.io.File(root).getCanonicalPath());
end
