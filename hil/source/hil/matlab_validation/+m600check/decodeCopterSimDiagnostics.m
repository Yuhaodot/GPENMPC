function result = decodeCopterSimDiagnostics(datagram,expectedCopterId,previousSimTime)
%DECODECOPTERSIMDIAGNOSTICS Pure decoder of official ii32d outCopterData.
% The official Windows packet
% is 264 bytes: int32 checksum1234567890, int32 CopterID, double[32], LE.
% Our payload v1: failed/code/sim_time/contact_force/ground/airborne/version1,
% then 25 zeros.
% previousSimTime=NaN only before the first packet of an explicit new run.
% Caller owns wall-clock freshness, packet routing and mandatory abort.
if nargin<3
    previousSimTime=NaN;
end
assert(isscalar(expectedCopterId) &&isfinite(expectedCopterId) ...
    &&expectedCopterId>=1 &&expectedCopterId<=double(intmax('int32')) ...
    &&expectedCopterId==floor(expectedCopterId));
assert(isscalar(previousSimTime) &&(isnan(previousSimTime) ...
    ||(isfinite(previousSimTime) &&previousSimTime>=0)));
result=struct('packet_valid',false,'status','UNDECODED', ...
    'checksum',int32(0),'copter_id',int32(0),'payload',zeros(32,1), ...
    'model_failed',false,'failure_code',0,'sim_time_s',0, ...
    'contact_force_n',0,'ground_confirmed',false,'airborne_observed',false, ...
    'schema_version',0,'time_monotonic',false, ...
    'can_use_as_healthy_observation',false,'must_stop',true);
if ~isa(datagram,'uint8') ||~isvector(datagram)
    result.status='INVALID_BYTE_TYPE_OR_SHAPE';return
end
if numel(datagram)~=264
    result.status='INVALID_PACKET_LENGTH';return
end
bytes=datagram(:);
header=typecast(bytes(1:8),'int32');
payload=typecast(bytes(9:264),'double');
[~,~,endian]=computer;
if endian=='B'
    header=swapbytes(header);payload=swapbytes(payload);
end
result.checksum=header(1);result.copter_id=header(2);result.payload=payload(:);
if header(1)~=int32(1234567890)
    result.status='INVALID_OFFICIAL_CHECKSUM';return
end
if header(2)~=int32(expectedCopterId)
    result.status='WRONG_COPTER_ID';return
end
if ~all(isfinite(payload))
    result.status='NONFINITE_DIAGNOSTIC_PAYLOAD';return
end
result.failure_code=payload(2);result.sim_time_s=payload(3);
result.contact_force_n=payload(4);result.schema_version=payload(7);
if payload(7)~=1 ||any(payload(8:32)~=0)
    result.status='UNKNOWN_DIAGNOSTIC_VERSION_OR_NONZERO_RESERVED';return
end
binaryFlags=payload([1,5,6]);
if ~all(binaryFlags==0 |binaryFlags==1) ||payload(2)<0 ...
        ||payload(2)~=floor(payload(2)) ||payload(3)<0 ||payload(4)<0
    result.status='INVALID_DIAGNOSTIC_FIELD_SEMANTICS';return
end
result.model_failed=payload(1)==1;
result.ground_confirmed=payload(5)==1;
result.airborne_observed=payload(6)==1;
if (~result.model_failed &&payload(2)~=0) ||(result.model_failed &&payload(2)==0)
    result.status='INCONSISTENT_FAILURE_FLAG_AND_CODE';return
end
result.packet_valid=true;
result.time_monotonic=isnan(previousSimTime) ||payload(3)>=previousSimTime;
if result.model_failed
    result.status='VALID_PACKET_MODEL_FAILURE_LATCHED';return
end
if ~result.time_monotonic
    result.status='VALID_PACKET_SIMULATION_TIME_REVERSED';return
end
result.status='VALID_HEALTHY_DIAGNOSTIC_PACKET';
result.can_use_as_healthy_observation=true;
result.must_stop=false;
end
