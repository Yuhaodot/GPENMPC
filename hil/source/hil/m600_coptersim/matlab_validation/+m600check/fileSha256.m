function h=fileSha256(path)
%FILESHA256 Byte-level HOST-ONLY hash; no project-global helper dependency.
fid=fopen(path,'rb');assert(fid>=0,'m600check:HashOpenFailed','Cannot read %s.',path);
guard=onCleanup(@()fclose(fid));md=java.security.MessageDigest.getInstance('SHA-256');
while ~feof(fid)
    bytes=fread(fid,65536,'*uint8');
    if ~isempty(bytes),md.update(bytes);end
end
raw=typecast(md.digest(),'uint8');h=upper(reshape(dec2hex(raw,2).',1,[]));clear guard
end
