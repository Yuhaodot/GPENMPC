function [p,reason]=inspectCanonicalRotorObserverPacket(bytes,expected)
% Pure shared packet parsing. No freshness, generation, source-clock or
% runtime-origin credit is granted by a syntactically valid datagram.
p=struct('generation',uint64(0),'sim_time_s',NaN,'rotor_thrust_state_n',nan(6,1));reason='';
if ~isa(bytes,'uint8')||~isvector(bytes)||numel(bytes)~=128,reason='PACKET_SHAPE';return,end
bytes=bytes(:);
if ~isequal(bytes(1:8),uint8('M6ROTOR1').'),reason='MAGIC';
elseif readU(bytes,9,2)~=1,reason='VERSION';
elseif readU(bytes,11,2)~=1,reason='NOT_ACCEPTED_CORE_STEP';
elseif readU(bytes,13,4)~=128,reason='DECLARED_SIZE';
elseif any(bytes(125:128)~=0),reason='RESERVED_NONZERO';
elseif readU(bytes,121,4)~=uint64(m600check.canonicalRotorObserverCrc32(bytes(1:120))),reason='CRC';
elseif readU(bytes,17,8)~=expected.session_token,reason='SESSION_MISMATCH';
elseif ~isequal(bytes(89:120),expected.dll_sha256),reason='DLL_IDENTITY_MISMATCH';
end
if ~isempty(reason),return,end
p.generation=readU(bytes,25,8);p.sim_time_s=typecast(readU(bytes,33,8),'double');
for k=1:6,p.rotor_thrust_state_n(k)=typecast(readU(bytes,41+(k-1)*8,8),'double');end
if p.generation==0,reason='ZERO_GENERATION';
elseif ~isfinite(p.sim_time_s)||p.sim_time_s<0,reason='SOURCE_TIME_INVALID';
elseif any(~isfinite(p.rotor_thrust_state_n))||any(p.rotor_thrust_state_n<0),reason='ROTOR_STATE_INVALID';
end
end
function v=readU(b,start,n)
v=uint64(0);for k=1:n,v=bitor(v,bitshift(uint64(b(start+k-1)),8*(k-1)));end
end
