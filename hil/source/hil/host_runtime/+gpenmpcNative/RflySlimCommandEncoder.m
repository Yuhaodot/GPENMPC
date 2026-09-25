function [packets,receipt,message,slim]=RflySlimCommandEncoder(command,assets,binding,serializer,dialect)
% Encode schema2 using the caller's MAVLink serializer and dialect.
% Serialize the verified board-issued ticket supplied by the owner.
[full,fullReceipt]=gpenmpcNative.encodeCanonicalFullInnerArguments(command,assets);
% Select canonical binary64 column-major slices: omit state19 and parameters52;
% retain dynamics30, bool and two uint64 tags.
slim=[uint8('RKS1').';full(157:397);full(814:829)];
assert(numel(slim)==261,'gpenmpcNative:SlimAbiLength','Schema2 ABI drift.');
for name={'command_generation','reference_generation','outer_generation'}
    value=binding.(name{1});
    assert(isa(value,'uint64')&&isscalar(value)&&value>0, ...
        'gpenmpcNative:SlimBindingGeneration','Exact positive uint64 binding required.');
end
for name={'snapshot_ticket','configuration_sha256'}
    value=binding.(name{1});
    assert(isa(value,'uint8')&&numel(value)==32&&any(value(:)), ...
        'gpenmpcNative:SlimBindingBytes','Exact nonzero32-byte binding required.');
end
for name={'target_system','target_component'}
    value=binding.(name{1});
    assert(isa(value,'uint8')&&isscalar(value)&&value>0, ...
        'gpenmpcNative:SlimTarget','Exact nonzero unicast target required.');
end
expectedConfig=uint8(sscanf(char(assets.binding.effective_configuration_payload_sha256),'%2x'));
assert(isequal(binding.configuration_sha256(:),expectedConfig(:)), ...
    'gpenmpcNative:SlimConfiguration','Binding must name the existing approved configuration.');
if string(command.evidence_scope)=="BOARD_COMMIT_RFC1"
    assert(isequal(binding.command_generation,command.generation) ...
        &&isequal(binding.snapshot_ticket(:),command.source_ticket(:)), ...
        'gpenmpcNative:SlimBoardPending','BOARD_COMMIT must retain its original private ticket/command generation.');
end
message=[bigEndian(binding.command_generation);binding.snapshot_ticket(:); ...
    bigEndian(binding.reference_generation);bigEndian(binding.outer_generation); ...
    binding.configuration_sha256(:);slim];
assert(numel(message)==349,'gpenmpcNative:SlimMessageLength','Schema2 binding drift.');
hash=java.security.MessageDigest.getInstance('SHA-256');hash.update(typecast(message,'int8'));
message=[message;reshape(typecast(hash.digest(),'uint8'),[],1)];
assert(numel(message)==381,'gpenmpcNative:SlimMessageLength','Schema2 must be381 bytes.');
packets=cell(4,1);wireBytes=0;
for index=0:3
    first=index*119+1;last=min(first+118,381);
    fragment=[uint8(32+index);bigEndian(binding.command_generation);message(first:last)];
    msg=createmsg(dialect,'TUNNEL');
    msg.Payload.target_system=binding.target_system;
    msg.Payload.target_component=binding.target_component;
    msg.Payload.payload_type=uint16(42002);
    msg.Payload.payload_length=uint8(numel(fragment));
    payload=zeros(size(msg.Payload.payload),'uint8');payload(1:numel(fragment))=fragment;
    msg.Payload.payload=payload;
    % Authoritative existing UAV Toolbox MAVLink2 framing/CRC/trailing-zero trim.
    packets{index+1}=reshape(uint8(serializemsg(serializer,msg)),1,[]);
    wireBytes=wireBytes+numel(packets{index+1});
end
receipt=struct('schema','GPENMPC_RFLY_SLIM_TUNNEL_SCHEMA2', ...
    'numerical_bytes',261,'binary64_count',30,'message_bytes',381,'fragments',4, ...
    'serialized_bytes',wireBytes,'parent_full_abi_sha256',fullReceipt.kernel_argument_sha256, ...
    'source_scope','HOST_SERIALIZATION_FUNCTION', ...
    'live_wire_protocol',false,'board_access',0,'publication_authority',false);
if string(command.evidence_scope)=="BOARD_COMMIT_RFC1"
    receipt.source_scope='BOARD_COMMIT_RFC1_INPUT_SERIALIZATION_NO_PUBLICATION_AUTHORITY';
    receipt.execution_session_sha256=command.execution_session_sha256;
end
end

function bytes=bigEndian(value)
[~,~,endian]=computer;
if endian=='L',value=swapbytes(value);end
bytes=reshape(typecast(value(:),'uint8'),[],1);
end
