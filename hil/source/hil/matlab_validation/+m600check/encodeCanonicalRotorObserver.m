function bytes=encodeCanonicalRotorObserver(rotorStateN,simTimeS,generation,sessionToken,dllSha256)
%#codegen
% Read-only ABI for the SAME accepted CopterSim core step, not a second plant.
% Connect rotorStateN to y.rotor_thrust_software_order_n = stateUp(14:19).
% Never connect animation/RPM or normalized command. Software rotor order is
% preserved; these six lag states (N) precede effectiveness multiplication.
% generation increments once per accepted core commit; session changes only
% at a genuine new model lifecycle. The caller must reject any failed step.
% dllSha256 is 32 raw digest bytes supplied by an independently verified DLL
% loader context. This field binds the receipt to that context; producer
% process and loaded-module identity require independent verification.
%
% LITTLE-ENDIAN observer ABI, 128 bytes, separate from outCopterData32:
% 0:7 M6ROTOR1; 8:u16 version1; 10:u16 accepted-step flag1; 12:u32 size128;
% 16:u64 session; 24:u64 generation; 32:f64 sim seconds; 40:f64[6] thrust N;
% 88:u8[32] DLL SHA256; 120:u32 CRC32(bytes0:119); 124:u32 reserved0.
% No transport, file/serial access, control output, position or attitude.
assert(isa(rotorStateN,'double')&&isreal(rotorStateN)&&isequal(size(rotorStateN),[6,1]) ...
    &&all(isfinite(rotorStateN))&&all(rotorStateN>=0),'m600check:RotorObserverState');
assert(isa(simTimeS,'double')&&isreal(simTimeS)&&isscalar(simTimeS) ...
    &&isfinite(simTimeS)&&simTimeS>=0,'m600check:RotorObserverTime');
assert(isa(generation,'uint64')&&isscalar(generation)&&generation>0, ...
    'm600check:RotorObserverGeneration');
assert(isa(sessionToken,'uint64')&&isscalar(sessionToken)&&sessionToken>0, ...
    'm600check:RotorObserverSession');
assert(isa(dllSha256,'uint8')&&isequal(size(dllSha256),[32,1])&&any(dllSha256~=0), ...
    'm600check:RotorObserverDllIdentity');
bytes=zeros(128,1,'uint8');bytes(1:8)=uint8('M6ROTOR1').';
bytes(9:10)=pack(uint64(1),2);bytes(11:12)=pack(uint64(1),2);
bytes(13:16)=pack(uint64(128),4);bytes(17:24)=pack(sessionToken,8);
bytes(25:32)=pack(generation,8);bytes(33:40)=pack(typecast(simTimeS,'uint64'),8);
for k=1:6,bytes(41+(k-1)*8:48+(k-1)*8)=pack(typecast(rotorStateN(k),'uint64'),8);end
bytes(89:120)=dllSha256;
bytes(121:124)=pack(uint64(m600check.canonicalRotorObserverCrc32(bytes(1:120))),4);
end
function b=pack(v,n)
b=zeros(n,1,'uint8');
for k=1:n,b(k)=uint8(bitand(bitshift(v,-8*(k-1)),uint64(255)));end
end
