function result=decodeCopterSimTerrainDiagnostics(datagram,expectedCopterId,previousSimTime,requireExtension)
% Pure HOST decoder, not a replacement of the consumed legacy decoder.
% Actual byte payload is always retained. Only explicitly declared terrain
% first-fault raw values may be nonfinite; they NEVER create healthy credit.
% requireExtension=true prevents an uninstrumented DLL masquerading as the
% future instrumented model. false explicitly permits unchanged legacy v1.
if nargin<3,previousSimTime=NaN;end
if nargin<4,requireExtension=true;end
assert(islogical(requireExtension)&&isscalar(requireExtension));
result=m600check.decodeCopterSimDiagnostics(datagram,expectedCopterId,previousSimTime);
result.raw_datagram=datagram;
result.terrain_extension=struct('present',false,'valid',false,'required',requireExtension, ...
    'tag',0,'capture_valid',false,'first_reason',0,'first_locked_height',0, ...
    'first_terrain15',zeros(15,1),'first_terrain_float64_hex',{{}}, ...
    'meaning','OBSERVATION_ONLY_NOT_A_NEW_HEALTH_OR_CONTROL_GATE');
if ~isa(datagram,'uint8')||~isvector(datagram)||numel(datagram)~=264
    return
end
bytes=datagram(:);payload=typecast(bytes(9:264),'double');
[~,~,endian]=computer;if endian=='B',payload=swapbytes(payload);end
payload=payload(:);
if all(payload(8:32)==0)
    if requireExtension,reject('REQUIRED_TERRAIN_EXTENSION_ABSENT');end
    return
end
% Independently preserve and verify the original prefix using the EXACT
% existing semantics. The copied prefix packet is not the stored raw packet.
prefixBytes=bytes;prefixBytes(65:264)=uint8(0);
prefix=m600check.decodeCopterSimDiagnostics(prefixBytes,expectedCopterId,previousSimTime);
extension=result.terrain_extension;
result=prefix;result.payload=payload;result.raw_datagram=datagram;
result.terrain_extension=extension;
result.terrain_extension.present=true;result.terrain_extension.tag=payload(26);
result.terrain_extension.first_terrain15=payload(11:25);
rawBits=typecast(bytes(89:208),'uint64');
if endian=='B',rawBits=swapbytes(rawBits);end
if all(rawBits==0)
    % Normal flight has no first-fault capture. Reuse the exact presentation
    % strings, not a stale numerical/model observation; nonzero raw bits,
    % including signed zero and NaNs, still use the original formatter.
    persistent emptyCaptureHex
    if isempty(emptyCaptureHex),emptyCaptureHex=repmat({'0000000000000000'},1,15);end
    result.terrain_extension.first_terrain_float64_hex=emptyCaptureHex;
else
    result.terrain_extension.first_terrain_float64_hex=cellstr(upper(dec2hex(rawBits,16))).';
end
if ~prefix.packet_valid,return;end
if ~isfinite(payload(26))||payload(26)~=1||any(payload(27:32)~=0)
    reject('UNKNOWN_TERRAIN_EXTENSION_TAG_OR_RESERVED');return
end
if ~all(isfinite(payload(8:10)))||~ismember(payload(8),[0,1])|| ...
    payload(9)~=fix(payload(9))||~ismember(payload(9),0:3)
    reject('INVALID_TERRAIN_EXTENSION_FIELD_SEMANTICS');return
end
captured=payload(8)==1;reason=payload(9);terrain=payload(11:25);locked=payload(10);
result.terrain_extension.capture_valid=captured;
result.terrain_extension.first_reason=reason;
result.terrain_extension.first_locked_height=locked;
if ~captured
    if reason~=0||locked~=0||any(terrain~=0)||prefix.failure_code==4
        reject('INCONSISTENT_TERRAIN_EMPTY_CAPTURE');return
    end
else
    if ~prefix.model_failed||prefix.failure_code~=4||reason==0
        reject('INCONSISTENT_TERRAIN_CAPTURE_AND_MODEL_FAILURE');return
    end
    if reason==2
        if all(isfinite(terrain))
            reject('TERRAIN_NONFINITE_REASON_WITH_FINITE_RAW');return
        end
    elseif ~all(isfinite(terrain))
        reject('TERRAIN_FINITE_REASON_WITH_NONFINITE_RAW');return
    elseif reason==3&&terrain(1)==locked
        reject('TERRAIN_CHANGED_REASON_WITH_UNCHANGED_HEIGHT');return
    end
end
result.terrain_extension.valid=true;
% Existing prefix outcome is unchanged: any genuine model fault or time
% reversal still must_stop; extension metadata never overrules it.

    function reject(status)
        result.packet_valid=false;result.status=status;
        result.can_use_as_healthy_observation=false;result.must_stop=true;
    end
end
