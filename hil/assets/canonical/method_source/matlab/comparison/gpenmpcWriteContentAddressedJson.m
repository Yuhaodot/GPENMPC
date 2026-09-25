function receipt = gpenmpcWriteContentAddressedJson(root, prefix, payload)
%GPENMPCWRITECONTENTADDRESSEDJSON Append one SHA-addressed JSON artifact.

arguments
    root (1,1) string
    prefix (1,1) string
    payload (1,1) struct
end
if ~isfolder(root), mkdir(root); end
temporary = fullfile(root, ".json_" + string(char(java.util.UUID.randomUUID())) + ".tmp");
writelines(jsonencode(payload, PrettyPrint=true), temporary, Encoding="UTF-8");
sha = gpenmpcSha256File(temporary);
path = fullfile(root, prefix + "_" + extractBefore(sha, 17) + ".json");
if isfile(path)
    if gpenmpcSha256File(path) ~= sha
        error("gpenmpcWriteContentAddressedJson:Collision", ...
            "Content-address prefix collision at %s.", path);
    end
    delete(temporary);
else
    [moved, message] = movefile(temporary, path);
    if ~moved
        if isfile(path) && gpenmpcSha256File(path) == sha
            delete(temporary);
        else
            error("gpenmpcWriteContentAddressedJson:Commit", ...
                "Immutable JSON commit failed at %s: %s", path, message);
        end
    end
end
receipt = struct("path", string(path), "bytes", dir(path).bytes, "sha256", sha);
end
