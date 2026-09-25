function crc = canonicalRotorObserverCrc32(bytes)
%#codegen
% IEEE CRC-32; corruption check only, not authentication or source attestation.
assert(isa(bytes,'uint8') && isvector(bytes));
crc=uint32(4294967295);
for k=1:numel(bytes)
    crc=bitxor(crc,uint32(bytes(k)));
    for j=1:8
        if bitand(crc,uint32(1))~=0
            crc=bitxor(bitshift(crc,-1),uint32(hex2dec('EDB88320')));
        else
            crc=bitshift(crc,-1);
        end
    end
end
crc=bitcmp(crc);
end
