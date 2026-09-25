function value=gpenmpc_device_identity(field)
% Read the explicitly configured flight-controller identity.
arguments
    field (1,1) string = ""
end
configPath=getenv('GPENMPC_DEVICE_CONFIG');
if isempty(configPath)
    root=fileparts(fileparts(fileparts(fileparts(mfilename('fullpath')))));
    configPath=fullfile(root,'local','device.json');
end
assert(isfile(configPath),'GPENMPC:DeviceConfiguration', ...
    'Configure GPENMPC_DEVICE_CONFIG or local/device.json with the verified device identity.');
device=jsondecode(fileread(configPath));
names={'uid','px4_guid','bootloader_sn_display'};
assert(isstruct(device)&&isscalar(device)&&all(isfield(device,names)), ...
    'GPENMPC:DeviceConfiguration','Device identity fields are missing.');
for k=1:numel(names)
    v=device.(names{k});
    assert(ischar(v)&&isrow(v)&&~isempty(v),'GPENMPC:DeviceConfiguration', ...
        'Device identity fields must be nonempty strings.');
end
assert(~isempty(regexp(device.uid,'^[1-9][0-9]{0,19}$','once'))&& ...
    ~isempty(regexp(device.px4_guid,'^[0-9A-F]{36}$','once'))&& ...
    ~isempty(regexp(device.bootloader_sn_display,'^[0-9A-F]{24}$','once')), ...
    'GPENMPC:DeviceConfiguration','Device identity format is invalid.');
digits=uint64(0);limit=intmax('uint64');
for c=device.uid
    d=uint64(c-'0');
    assert(digits<=idivide(limit-d,uint64(10),'floor'), ...
        'GPENMPC:DeviceConfiguration','Device UID exceeds uint64.');
    digits=digits*uint64(10)+d;
end
guidSerial=device.px4_guid(end-23:end);
assert(strcmp(device.bootloader_sn_display,[guidSerial(17:24) guidSerial(9:16) guidSerial(1:8)]), ...
    'GPENMPC:DeviceConfiguration','GUID and bootloader serial do not identify the same device.');
device.uid_uint64=digits;
if strlength(field)==0
    value=device;
else
    assert(isfield(device,field),'GPENMPC:DeviceConfiguration','Unknown device identity field.');
    value=device.(field);
end
end
