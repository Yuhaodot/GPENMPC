function cleanup=gpenmpc_test_device_config()
% Isolate an offline test with a synthetic flight-controller identity.
prior=getenv('GPENMPC_DEVICE_CONFIG');
folder=tempname;mkdir(folder);
file=fullfile(folder,'device.json');
fixture=struct('uid','1234605616436508552','px4_guid','000600000000111111112222222233333333', ...
    'bootloader_sn_display','333333332222222211111111');
f=fopen(file,'w');assert(f>=0);fprintf(f,'%s',jsonencode(fixture));fclose(f);
setenv('GPENMPC_DEVICE_CONFIG',file);
cleanup=onCleanup(@restore);
    function restore()
        setenv('GPENMPC_DEVICE_CONFIG',prior);
        delete(file);rmdir(folder);
    end
end
