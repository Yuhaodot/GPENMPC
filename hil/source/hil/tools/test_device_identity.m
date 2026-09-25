function result=test_device_identity()
% Test exact device identity parsing and reject invalid configurations.
cleanup=gpenmpc_test_device_config(); %#ok<NASGU>
file=getenv('GPENMPC_DEVICE_CONFIG');fixture=jsondecode(fileread(file));
actual=gpenmpc_device_identity();
expected=bitor(bitshift(uint64(hex2dec('11223344')),32),uint64(hex2dec('55667788')));
assert(strcmp(actual.uid,fixture.uid)&&actual.uid_uint64==expected);
bad={struct(),setfield(fixture,'uid',123),setfield(fixture,'uid','0'), ...
    setfield(fixture,'uid','18446744073709551616'),setfield(fixture,'uid','*'), ...
    setfield(fixture,'px4_guid',repmat('A',1,36)), ...
    setfield(fixture,'bootloader_sn_display',repmat('0',1,24))}; %#ok<SFLD>
for k=1:numel(bad)
    f=fopen(file,'w');assert(f>=0);fprintf(f,'%s',jsonencode(bad{k}));fclose(f);
    rejected=false;
    try,gpenmpc_device_identity();catch ex,rejected=strcmp(ex.identifier,'GPENMPC:DeviceConfiguration');end
    assert(rejected,'Invalid device configuration was accepted.');
end
setenv('GPENMPC_DEVICE_CONFIG',fullfile(fileparts(file),'missing.json'));
rejected=false;
try,gpenmpc_device_identity();catch ex,rejected=strcmp(ex.identifier,'GPENMPC:DeviceConfiguration');end
assert(rejected,'Missing device configuration was accepted.');
result=struct('passed',true,'exact_uint64',true,'invalid_configurations',numel(bad)+1);
disp(result);
end
