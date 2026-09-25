function config=configure_hil(recoveryRecords,setupRecord,deviceConfig)
% Configure the local parameter records for a MATLAB HIL session.
arguments
    recoveryRecords (1,1) string
    setupRecord (1,1) string = ""
    deviceConfig (1,1) string = ""
end
assert(isfolder(recoveryRecords),'GPENMPC:RecoveryDirectory','Recovery directory does not exist.');
root=fileparts(mfilename('fullpath'));
addpath(fullfile(root,'source','hil','tools'));
oldRecords=getenv('HIL_RECOVERY_RECORDS');
oldDevice=getenv('GPENMPC_DEVICE_CONFIG');
if strlength(deviceConfig)>0,setenv('GPENMPC_DEVICE_CONFIG',char(deviceConfig));end
setenv('HIL_RECOVERY_RECORDS',char(recoveryRecords));
try
    gpenmpc_device_identity();
    [contracts,proof]=load_m600_recovery_contracts();
    snapshot=fullfile(recoveryRecords,'SERIAL_PREFLIGHT.json');
    expected='073E7CE165D8D158BC329CFE96AEC4F968A6847A2A0AB9D90C1C127D3CB65EBB';
    f=fopen(snapshot,'rb');
    assert(f>=0,'GPENMPC:ParameterSnapshot','Cannot read parameter snapshot.');
    closer=onCleanup(@()fclose(f));raw=fread(f,Inf,'*uint8');clear closer
    md=java.security.MessageDigest.getInstance('SHA-256');md.update(typecast(raw,'int8'));
    sha=upper(reshape(dec2hex(typecast(md.digest(),'uint8'),2).',1,[]));
    assert(strcmp(sha,expected),'GPENMPC:ParameterSnapshot','Parameter snapshot checksum differs.');
    if strlength(setupRecord)>0
        assert(isfile(setupRecord),'GPENMPC:SetupRecord','Hardware setup record does not exist.');
    end
catch ex
    setenv('HIL_RECOVERY_RECORDS',oldRecords);
    setenv('GPENMPC_DEVICE_CONFIG',oldDevice);
    rethrow(ex)
end
if strlength(setupRecord)>0,setenv('HIL_SETUP_RECORD',char(setupRecord));end
config=struct('project','GPENMPC','recovery_directory',char(recoveryRecords), ...
    'device_configuration',getenv('GPENMPC_DEVICE_CONFIG'), ...
    'setup_record',getenv('HIL_SETUP_RECORD'),'recovery_parameters', ...
    numel(contracts.temporary_allocator_geometry.entries)+numel(contracts.native_hover_tuning.entries), ...
    'plan_sha256',proof.plan.sha256,'snapshot_sha256',sha);
end
