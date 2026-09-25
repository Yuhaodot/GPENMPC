function digest = gpenmpcSha256File(path)
%GPENMPCSHA256FILE Return uppercase SHA-256 for one file.

arguments
    path (1,1) string
end
if ~isfile(path)
    error("gpenmpcSha256File:Missing", "Cannot hash missing file %s.", path);
end
file = fopen(path, "rb");
if file < 0
    error("gpenmpcSha256File:Open", "Cannot open file %s.", path);
end
cleanup = onCleanup(@() fclose(file));
engine = java.security.MessageDigest.getInstance("SHA-256");
while ~feof(file)
    bytes = fread(file, 1024 * 1024, "*uint8");
    if ~isempty(bytes)
        engine.update(typecast(bytes, "int8"));
    end
end
bytes = typecast(engine.digest(), "uint8");
digest = upper(string(reshape(dec2hex(bytes, 2).', 1, [])));
end
