function [p,reason]=inspectCanonicalControlCachePacket(bytes,expected)
% Inspect M6CACHE1 v1 packets containing wrapper accepted-step input values.
p=struct('generation',uint64(0),'sim_time_s',NaN,'input_call_count',uint64(0), ...
    'input16',nan(16,1));reason='';
if ~isa(bytes,'uint8')||~isvector(bytes)||numel(bytes)~=216,reason='CACHE_PACKET_SHAPE';return,end
bytes=bytes(:);
if ~isequal(bytes(1:8),uint8('M6CACHE1').'),reason='CACHE_MAGIC';
elseif readU(bytes,9,2)~=1,reason='CACHE_VERSION';
elseif readU(bytes,11,2)~=1,reason='CACHE_NOT_ACCEPTED_CORE_STEP';
elseif readU(bytes,13,4)~=216,reason='CACHE_DECLARED_SIZE';
elseif any(bytes(213:216)~=0),reason='CACHE_RESERVED_NONZERO';
elseif readU(bytes,209,4)~=uint64(m600check.canonicalRotorObserverCrc32(bytes(1:208))),reason='CACHE_CRC';
elseif readU(bytes,17,8)~=expected.session_token,reason='CACHE_SESSION_MISMATCH';
elseif ~isequal(bytes(177:208),expected.dll_sha256),reason='CACHE_DLL_IDENTITY_MISMATCH';
end
if ~isempty(reason),return,end
p.generation=readU(bytes,25,8);p.sim_time_s=typecast(readU(bytes,33,8),'double');
p.input_call_count=readU(bytes,41,8);
for k=1:16,p.input16(k)=typecast(readU(bytes,49+(k-1)*8,8),'double');end
if p.generation==0,reason='CACHE_ZERO_GENERATION';
elseif p.input_call_count==0,reason='CACHE_NO_ACTUAL_INPUT_CALL';
elseif ~isfinite(p.sim_time_s)||p.sim_time_s<0,reason='CACHE_SOURCE_TIME_INVALID';
elseif any(~isfinite(p.input16)),reason='CACHE_INPUT_NONFINITE';
end
end
function v=readU(b,start,n)
v=uint64(0);for k=1:n,v=bitor(v,bitshift(uint64(b(start+k-1)),8*(k-1)));end
end
