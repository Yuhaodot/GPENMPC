function receipt=test_matlab_slim_encoder(outputRoot,abiFile,bindingFile,sourceFile)
% Cross-check serialization using 76 source-bound fixture records.
% Supply the ABI, transport metadata and raw-command fixture files explicitly.
arguments
    outputRoot (1,1) string
    abiFile (1,1) string
    bindingFile (1,1) string
    sourceFile (1,1) string
end
root=string(fileparts(mfilename('fullpath')));build=string(fileparts(root));
assert(~isfolder(outputRoot),'Preserve previous function-test output.');
old=path;cleanup=onCleanup(@()path(old)); %#ok<NASGU>
addpath(fullfile(build,'host_runtime'),'-begin');
assert(isfile(abiFile)&&isfile(bindingFile)&&isfile(sourceFile), ...
    'gpenmpcNative:SlimFixtureInputs','All three fixture files must exist.');
assets=gpenmpcNative.loadCanonicalAssets();original=load(sourceFile,'raw');
assert(numel(original.raw.samples)==60);
assert(strcmpi(gpenmpcNative.fileSha256(abiFile),'05130566CDD1599BABA745FA7BF9F1DCC9D4DD49D81FD5A81006CB6589D8351E'));
fk=fopen(abiFile,'rb','ieee-be');assert(fk>=0);kg=onCleanup(@()fclose(fk));
fb=fopen(bindingFile,'rb','ieee-be');assert(fb>=0);bg=onCleanup(@()fclose(fb));
assert(fread(fk,1,'*uint32')==76&&fread(fb,1,'*uint32')==76);
dialect=mavlinkdialect('common.xml',2);
serializer=mavlinkio(dialect,SystemID=42,ComponentID=191, ...
    ComponentType='MAV_TYPE_GCS',AutopilotType='MAV_AUTOPILOT_INVALID');
mkdir(outputRoot);packetFile=fullfile(outputRoot,'MATLAB_SCHEMA2_PACKETS.bin');
fo=fopen(packetFile,'wb','ieee-be');assert(fo>=0);og=onCleanup(@()fclose(fo));
fwrite(fo,uint8('RSM2'),'uint8');fwrite(fo,uint32(76),'uint32');
wireBytes=0;fullExact=0;negativePass=0;
for row=1:76
    full=fread(fk,829,'*uint8');assert(numel(full)==829);
    fread(fk,32,'*uint8');fread(fk,61,'*double');assert(fread(fk,1,'*uint8')==1);
    meta=fread(fb,88,'*uint8');assert(numel(meta)==88);
    if row<=60,index=row;else,index=mod(row-61,60)+1;end
    command=original.raw.samples{index}.begin.full_inner_command;
    % Reconstruct the command fields from the bound full ABI record.
    offset=5;
    command.state_up=realField(19);
    command.reference_up.position_m=realField(3);command.reference_up.velocity_mps=realField(3);
    command.reference_up.acceleration_mps2=realField(3);command.payload_kg=realField(1);
    command.wind_estimate_xy_mps=realField(2);command.augmentation_up_mps2=realField(3);
    command.attitude_command.enabled=logical(full(offset));offset=offset+1;
    command.attitude_command.desired_rotation=reshape(realField(9),3,3);
    command.attitude_command.desired_angular_velocity_body_rad_s=realField(3);
    command.attitude_command.desired_angular_acceleration_body_rad_s2=realField(3);
    command.generation=fromBE(full(814:821),'uint64');
    assert(command.generation==fromBE(full(822:829),'uint64'));
    [repacked,~]=gpenmpcNative.encodeCanonicalFullInnerArguments(command,assets);
    assert(isequal(repacked,full),'Encoded command differs from the bound fixture.');fullExact=fullExact+1;
    binding=struct('command_generation',fromBE(meta(1:8),'uint64'), ...
        'snapshot_ticket',meta(9:40),'reference_generation',fromBE(meta(41:48),'uint64'), ...
        'outer_generation',fromBE(meta(49:56),'uint64'),'configuration_sha256',meta(57:88), ...
        'target_system',uint8(1),'target_component',uint8(1));
    [packets,r,message,slim]=gpenmpcNative.RflySlimCommandEncoder(command,assets,binding,serializer,dialect);
    assert(isequal(slim,[uint8('RKS1').';full(157:397);full(814:829)]));
    assert(isequal(message(1:88),meta)&&numel(packets)==4);
    fwrite(fo,message,'uint8');
    for j=1:4,fwrite(fo,uint16(numel(packets{j})),'uint16');fwrite(fo,packets{j},'uint8');end
    wireBytes=wireBytes+r.serialized_bytes;
end
assert(isempty(fread(fk,1,'*uint8'))&&isempty(fread(fb,1,'*uint8')));
clear og kg bg
for mode=1:3
    bad=binding;
    if mode==1,bad.command_generation=double(bad.command_generation);end
    if mode==2,bad.configuration_sha256(1)=bitxor(bad.configuration_sha256(1),uint8(1));end
    if mode==3,bad.target_system=uint8(0);end
    rejected=false;
    try,gpenmpcNative.RflySlimCommandEncoder(command,assets,bad,serializer,dialect);catch ex
        rejected=startsWith(string(ex.identifier),'gpenmpcNative:Slim');
    end
    assert(rejected);negativePass=negativePass+1;
end
receipt=struct('status','PASS_ACTUAL_MATLAB_SERIALIZER_FUNCTION_ONLY', ...
    'fixture_rows',76,'unchanged_full829_exact_rows',fullExact,'schema2_packets',304, ...
    'actual_matlab_serialized_bytes',wireBytes,'negative_cases_passed',negativePass, ...
    'fixture_sha256',gpenmpcNative.fileSha256(abiFile), ...
    'cpp_private_ticket_binding_file_sha256',gpenmpcNative.fileSha256(bindingFile), ...
    'packet_file_sha256',gpenmpcNative.fileSha256(packetFile), ...
    'function_source_sha256',gpenmpcNative.fileSha256(which('gpenmpcNative.RflySlimCommandEncoder')), ...
    'canonical_full_encoder_sha256',gpenmpcNative.fileSha256(which('gpenmpcNative.encodeCanonicalFullInnerArguments')), ...
    'cpp_crosscheck_pending',true,'live_runtime_connected',false,'com_open',0,'board_actions',0);
fj=fopen(fullfile(outputRoot,'MATLAB_RESULT.json'),'w','n','UTF-8');assert(fj>=0);
jg=onCleanup(@()fclose(fj));fprintf(fj,'%s\n',jsonencode(receipt,PrettyPrint=true));disp(jsonencode(receipt));
    function value=realField(count)
        value=reshape(fromBE(full(offset:offset+8*count-1),'double'),[],1);offset=offset+8*count;
    end
end

function value=fromBE(bytes,type)
value=typecast(bytes(:),type);[~,~,endian]=computer;
if endian=='L',value=swapbytes(value);end
end
