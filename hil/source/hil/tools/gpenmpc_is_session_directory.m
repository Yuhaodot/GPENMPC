function allowed=gpenmpc_is_session_directory(value,build)
% Match an exact session name under an approved recording directory.
value=char(java.io.File(char(value)).getCanonicalPath());
[parent,name,extension]=fileparts(value);
live=char(java.io.File(fullfile(char(build),'live')).getCanonicalPath());
allowed=isempty(extension)&&~isempty(regexp(name,'^manual_session_[0-9]{3}$','once')) ...
    &&(strcmpi(parent,gpenmpc_log_root())||strcmpi(parent,live));
end
