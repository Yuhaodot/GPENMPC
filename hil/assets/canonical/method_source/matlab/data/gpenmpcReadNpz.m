function [arrays, audit] = gpenmpcReadNpz(path)
%GPENMPCREADNPZ Read a numeric NPZ archive in MATLAB.

arguments
    path (1,1) string
end
if ~isfile(path)
    error("gpenmpcReadNpz:Input", "NPZ archive does not exist: %s", path);
end
temporaryRoot = string(tempname);
mkdir(temporaryRoot);
cleanup = onCleanup(@() cleanupTemporary(temporaryRoot));
unzip(path, temporaryRoot);
files = dir(fullfile(temporaryRoot, "**", "*.npy"));
if isempty(files)
    error("gpenmpcReadNpz:Empty", "No NPY members found in %s", path);
end

arrays = struct;
members = repmat(struct("name", "", "shape", [], "descr", ""), numel(files), 1);
for index = 1:numel(files)
    memberPath = string(fullfile(files(index).folder, files(index).name));
    originalName = erase(string(files(index).name), ".npy");
    fieldName = matlab.lang.makeValidName(originalName);
    if isfield(arrays, fieldName)
        error("gpenmpcReadNpz:Duplicate", "Duplicate NPZ member: %s", originalName);
    end
    [arrays.(fieldName), metadata] = gpenmpcReadNpy(memberPath);
    members(index).name = originalName;
    members(index).shape = metadata.shape;
    members(index).descr = metadata.descr;
end
audit = struct;
audit.schema = "GPENMPC_MATLAB_NATIVE_NPZ_READER_AUDIT_V1";
audit.path = path;
audit.member_count = numel(files);
audit.members = members;
clear cleanup
cleanupTemporary(temporaryRoot);
end


function cleanupTemporary(path)
if isfolder(path)
    rmdir(path, "s");
end
end
