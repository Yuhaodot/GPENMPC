function receipt=build_local_full_inner_dialect()
% BUILD_LOCAL_FULL_INNER_DIALECT Build the offline XML derivative.
% Read source XML without modifying it; require existing artifacts to match.
build=fileparts(fileparts(mfilename('fullpath')));
parentDir=fullfile(build,'m600_coptersim','matlab_validation','+m600check');
source=fullfile(gpenmpc_external_path('px4_integration_project'), ...
    'px4_worktree','src','modules','mavlink','mavlink', ...
    'message_definitions','v1.0','common.xml');
oldCommonPath=fullfile(parentDir,'common_matlab_parent.xml');
oldHealthPath=fullfile(parentDir,'px4_health_events.xml');
rawPath=fullfile(gpenmpc_external_path('startup_observation'), ...
    'POST_FAILURE_UDP_OBSERVATION_OFFICIAL_STREAM_PARSER.mat');
sourceSha='C06DFECC71A0AB945244C22CB3D578D1DDC022078C957044B8EB28826A4A9B65';
commonSha='4CBC302D55C356D9E1CAD5A9521EDE4086CE6ECD07CE490AA0CC5DB81EB44D78';
healthSha='1D59BE0A1F25ECAB92BD3C375BB865D659E3E12A359B58A7B2058110D7EEB6FE';
rawSha='F1510947AF614FE840471C50BA027389390C38FC0CFD7D3A6713D9E9931A469F';
assertHash(source,sourceSha);assertHash(oldCommonPath,commonSha);
assertHash(oldHealthPath,healthSha);assertHash(rawPath,rawSha);
% Copy source bytes without XML reserialization or encoding conversion.
px4=char(readBytes(source));oldCommon=char(readBytes(oldCommonPath));
oldHealth=char(readBytes(oldHealthPath));
old12904=xmlBlock(oldCommon,'message','id="12904"');
new12904=xmlBlock(px4,'message','id="12904"');
new436=xmlBlock(px4,'message','id="436"');
newEnum=xmlBlock(px4,'enum','name="MAV_STANDARD_MODE"');
assert(contains(new12904,'name="operator_altitude_geo"') ...
    &&contains(new12904,'name="timestamp"'), ...
    'gpenmpcDialect:Source','The exact PX4 12904 source lacks its required fields.');
assert(~contains(oldCommon,'<message id="436"') ...
    &&~contains(oldCommon,'<enum name="MAV_STANDARD_MODE"'), ...
    'gpenmpcDialect:Parent','The immutable parent already contains an added definition.');
newCommon=replaceOnce(oldCommon,old12904,new12904);
enumInsertion=[newEnum char(10)];messageInsertion=[new436 char(10)];
newCommon=replaceOnce(newCommon,'  </enums>',[enumInsertion '  </enums>']);
newCommon=replaceOnce(newCommon,'  </messages>',[messageInsertion '  </messages>']);
% Reverse the only three permitted changes and require byte-for-byte identity.
reversed=replaceOnce(newCommon,enumInsertion,'');
reversed=replaceOnce(reversed,messageInsertion,'');
reversed=replaceOnce(reversed,new12904,old12904);
assert(isequal(uint8(reversed),uint8(oldCommon)), ...
    'gpenmpcDialect:ParentBytes','An unrelated parent byte was changed.');
oldInclude='<include>common_matlab_parent.xml</include>';
newInclude='<include>common_local_full_inner.xml</include>';
newHealth=replaceOnce(oldHealth,oldInclude,newInclude);
assert(isequal(uint8(replaceOnce(newHealth,newInclude,oldInclude)),uint8(oldHealth)), ...
    'gpenmpcDialect:HealthBytes','An original health/Event definition was changed.');
outDir=fullfile(parentDir,'local_full_inner_dialect');
if ~isfolder(outDir),mkdir(outDir);end
commonPath=fullfile(outDir,'common_local_full_inner.xml');
dialectPath=fullfile(outDir,'px4_local_full_inner.xml');
writeNewOrIdentical(commonPath,uint8(newCommon));
writeNewOrIdentical(dialectPath,uint8(newHealth));
oldDefs=uav.internal.mavlink.parser.Import(oldHealthPath,2);
newDefs=uav.internal.mavlink.parser.Import(dialectPath,2);
assert(numel(newDefs.Messages.ID)==numel(oldDefs.Messages.ID)+1, ...
    'gpenmpcDialect:DefinitionCount','Expected exactly one additional message ID.');
unchanged=oldDefs.Messages.ID(oldDefs.Messages.ID~=12904);
for id=reshape(unchanged,1,[])
    oi=find(oldDefs.Messages.ID==id);ni=find(newDefs.Messages.ID==id);
    assert(isscalar(ni)&&isequaln(oldDefs.Messages.Fields(oi),newDefs.Messages.Fields(ni)) ...
        &&oldDefs.Messages.CRCExtra(oi)==newDefs.Messages.CRCExtra(ni) ...
        &&oldDefs.Messages.MinLength(oi)==newDefs.Messages.MinLength(ni) ...
        &&oldDefs.Messages.MaxLength(oi)==newDefs.Messages.MaxLength(ni), ...
        'gpenmpcDialect:UnrelatedMessage','Unrelated message %u was changed.',id);
end
assert(newDefs.Messages.CRCExtra(newDefs.Messages.ID==12904)==77 ...
    &&newDefs.Messages.CRCExtra(newDefs.Messages.ID==436)==193, ...
    'gpenmpcDialect:CRCExtra','Derived CRC extras differ from actual PX4 generated headers.');
validation=validateRetained(rawPath,dialectPath);
assertHash(oldCommonPath,commonSha);assertHash(oldHealthPath,healthSha);
receipt=struct('schema','GPENMPC_LOCAL_FULL_INNER_DIALECT_BUILD_V1', ...
    'scope','BOARD_LOCAL_FULL_INNER_XML_COMPATIBILITY', ...
    'px4_common_source',source,'px4_common_sha256',sourceSha, ...
    'old_common_source',oldCommonPath,'old_common_sha256',commonSha, ...
    'old_health_source',oldHealthPath,'old_health_sha256',healthSha, ...
    'replaced_message_id',12904,'added_message_id',436, ...
    'added_enum','MAV_STANDARD_MODE', ...
    'unchanged_existing_message_count',numel(unchanged), ...
    'common_path',commonPath,'common_sha256',fileSha(commonPath), ...
    'dialect_path',dialectPath,'dialect_sha256',fileSha(dialectPath), ...
    'retained_raw_source',rawPath,'retained_raw_sha256',rawSha, ...
    'validation',validation,'passed',validation.passed);
writeNewOrIdentical(fullfile(outDir,'BUILD_RECEIPT.json'), ...
    unicode2native(jsonencode(receipt),'UTF-8'));
assert(receipt.passed,'gpenmpcDialect:RetainedRaw', ...
    'Not all 4325 original complete datagrams passed official CRC decoding.');
end

function result=validateRetained(rawPath,dialectPath)
saved=load(rawPath,'raw');rows=saved.raw;
template=mavlinkdialect(dialectPath,2);
rx=0;complete=0;valid=0;crcFailure=0;exceptions=0;ids=[];failures={};
for rawIndex=1:numel(rows)
    row=rows{rawIndex};if ~strcmp(row.direction,'MAVLINK_RX'),continue,end
    rx=rx+1;b=uint8(row.bytes(:).');
    if numel(b)<3||b(1)~=253||bitand(b(3),uint8(254))~=0,continue,end
    n=12+double(b(2))+13*double(bitand(b(3),uint8(1))~=0);
    if numel(b)~=n,continue,end
    complete=complete+1;decoder=copy(template);
    try
        [message,status]=deserializemsg(decoder,b,OutputAllMessages=true);
        id=double(b(8))+256*double(b(9))+65536*double(b(10));
        okay=isscalar(message)&&isscalar(status)&&status==0 ...
            &&message.MsgID==id&&message.SystemID==b(6)&&message.ComponentID==b(7);
        if okay,valid=valid+1;ids(end+1)=id; %#ok<AGROW>
        else,crcFailure=crcFailure+1;failures{end+1}= ...
                struct('raw_row',rawIndex,'rx_row',rx,'status',status);end %#ok<AGROW>
    catch ex
        exceptions=exceptions+1;failures{end+1}= ...
            struct('raw_row',rawIndex,'rx_row',rx,'error',ex.identifier); %#ok<AGROW>
    end
    delete(decoder);
end
result=struct('rx_datagrams',rx,'complete_datagrams',complete, ...
    'noncomplete_datagrams_retained_not_admitted',rx-complete, ...
    'official_crc_zero',valid,'crc_or_message_failures',crcFailure, ...
    'exceptions',exceptions,'failures',{failures}, ...
    'heartbeat_count',sum(ids==0),'autopilot_version_count',sum(ids==148), ...
    'extended_sys_state_count',sum(ids==245),'tunnel_count',sum(ids==385), ...
    'open_drone_id_system_count',sum(ids==12904),'current_mode_count',sum(ids==436), ...
    'passed',rx==4996&&complete==4325&&valid==4325&&crcFailure==0&&exceptions==0);
end
function block=xmlBlock(text,kind,attribute)
pattern=['(?m)^[ \t]*<' kind ' ' attribute '[^>]*>[\s\S]*?^[ \t]*</' kind '>'];
matches=regexp(text,pattern,'match');
assert(numel(matches)==1,'gpenmpcDialect:Block','Expected one exact source block: %s.',attribute);
block=matches{1};
end
function text=replaceOnce(text,old,new)
at=strfind(text,old);
assert(isscalar(at),'gpenmpcDialect:Replacement','Expected one exact replacement target.');
text=[text(1:at-1) new text(at+numel(old):end)];
end
function bytes=readBytes(path)
f=fopen(path,'rb');assert(f>=0,'gpenmpcDialect:Read','Cannot read %s.',path);
cleanup=onCleanup(@()fclose(f));bytes=fread(f,Inf,'*uint8').'; %#ok<NASGU>
end
function value=fileSha(path)
digest=java.security.MessageDigest.getInstance('SHA-256');digest.update(readBytes(path));
value=upper(reshape(dec2hex(typecast(digest.digest(),'uint8'),2).',1,[]));
end
function assertHash(path,expected)
assert(strcmp(fileSha(path),expected),'gpenmpcDialect:SourceHash','Source changed: %s.',path);
end
function writeNewOrIdentical(path,bytes)
bytes=uint8(bytes(:).');
if isfile(path)
    assert(isequal(readBytes(path),bytes),'gpenmpcDialect:ExistingArtifact', ...
        'Existing artifact differs; it will not be overwritten: %s.',path);
    return
end
stream=System.IO.FileStream(path,System.IO.FileMode.CreateNew,System.IO.FileAccess.Write);
cleanup=onCleanup(@()stream.Dispose()); %#ok<NASGU>
stream.Write(bytes,0,numel(bytes));stream.Flush(true);
end
