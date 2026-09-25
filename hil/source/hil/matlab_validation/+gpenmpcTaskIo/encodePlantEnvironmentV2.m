function [bytes,f]=encodePlantEnvironmentV2(v,copterId,initialPayloadKg)
%#codegen
% Pure V2 encoder: 232 little-endian bytes = int32 magic/ID + 28 doubles.
% Official inDoubCtrls ONLY. Do not route through truncating inSILInts/Floats.
% 1 schema=2; 2 generation; 3 mapped source I/O time; 4 task time;
% 5 payload; 6:7 NED wind; 8 phase; 9:20 NED p/v/a/jerk;
% 21 payload generation; 22 paused; 23 session binding; 24 min HB/EXT rx;
% 25 flags (bits 0:3 HB/EXT/identity/clock valid; bit4 armed);
% 26 PX4 landed_state; 27 invalid-event continuity epoch; 28 release gen.
% Both raw board messages must independently pass identity/future checks
% before min(rx) is encoded. max_board_age=min(the two original bounds).
% 23/27/28 are precise integer references into the owner's evidence, not
% cryptographic proof. Owner MUST expose every invalid transition via 27.
assert(integerIn(copterId,1,255)&&finiteScalar(initialPayloadKg)&&initialPayloadKg>=0,'gpenmpcTaskIo:V2Encode','Invalid copter/payload bound.');
assert(isstruct(v)&&isscalar(v)&&v.schema_version==2,'gpenmpcTaskIo:V2Encode','Explicit numeric V2 schema required.');
assert(integerIn(v.generation,1,2^32-1)&&integerIn(v.payload_generation,0,4),'gpenmpcTaskIo:V2Encode','Invalid generation.');
assert(finiteScalar(v.source_io_time_s)&&v.source_io_time_s>=0&&finiteScalar(v.task_reference_time_s)&&v.task_reference_time_s>=0,'gpenmpcTaskIo:V2Encode','Invalid mapped time.');
assert(finiteScalar(v.payload_kg)&&v.payload_kg>=0&&v.payload_kg<=initialPayloadKg,'gpenmpcTaskIo:V2Encode','Invalid payload.');
assert(isa(v.wind_ned_xy_mps,'double')&&isequal(size(v.wind_ned_xy_mps),[2,1])&&all(isfinite(v.wind_ned_xy_mps)),'gpenmpcTaskIo:V2Encode','Expected finite 2x1 wind.');
assert(isa(v.reference_jet_ned,'double')&&isequal(size(v.reference_jet_ned),[12,1])&&all(isfinite(v.reference_jet_ned)),'gpenmpcTaskIo:V2Encode','Expected finite 12x1 reference.');
assert(integerIn(v.mission_phase,0,6)&&islogical(v.task_clock_paused)&&isscalar(v.task_clock_paused),'gpenmpcTaskIo:V2Encode','Invalid phase/paused type.');
assert(integerIn(v.session_token,1,flintmax)&&integerIn(v.continuity_epoch,1,2^32-1),'gpenmpcTaskIo:V2Encode','Invalid session/continuity.');
assert(finiteScalar(v.board_min_rx_io_time_s)&&v.board_min_rx_io_time_s>=0&&integerIn(v.board_valid_flags,0,31)&&integerIn(v.landed_state,0,3),'gpenmpcTaskIo:V2Encode','Invalid independent board fields.');
assert(integerIn(v.service_release_generation,0,4),'gpenmpcTaskIo:V2Encode','Invalid release generation.');
f=zeros(28,1);
f(1:8)=[2;double(v.generation);double(v.source_io_time_s);double(v.task_reference_time_s);double(v.payload_kg);v.wind_ned_xy_mps;double(v.mission_phase)];
f(9:20)=v.reference_jet_ned;f(21:28)=[double(v.payload_generation);double(v.task_clock_paused);double(v.session_token);double(v.board_min_rx_io_time_s);double(v.board_valid_flags);double(v.landed_state);double(v.continuity_epoch);double(v.service_release_generation)];
% Explicit little-endian numeric bit packing, independent of host byte order.
bytes=zeros(1,232,'uint8');head=uint32([1234567897,copterId]);
for k=1:2,for j=1:4,bytes((k-1)*4+j)=uint8(bitand(bitshift(head(k),-8*(j-1)),uint32(255)));end,end
for k=1:28
    bits=typecast(f(k),'uint64');
    for j=1:8,bytes(8+(k-1)*8+j)=uint8(bitand(bitshift(bits,-8*(j-1)),uint64(255)));end
end
end
function tf=finiteScalar(v),tf=isnumeric(v)&&isreal(v)&&isscalar(v)&&isfinite(v);end
function tf=integerIn(v,a,b),tf=finiteScalar(v)&&v==fix(v)&&v>=a&&v<=b;end
