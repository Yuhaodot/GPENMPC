function [cfg,evidence] = native_sensor_filter_explicit_fixture(fs)
% Combine typed filter settings with synthetic sensor input.
% The caller selects sample frequency.
arguments
    fs (1,1) double
end
assert(isfinite(fs)&&fs>10&&fs<10000,'gpenmpc:SensorFixtureFs');
path=gpenmpc_external_path('native_lower_loop_readback_receipt');
source=identity(path);
assert(strcmp(source.sha256,'EC504AEB12444D41B88DD0E5411DDAA3B59B513AB9410433736A5B59C47E1D9C'),...
    'gpenmpc:SensorFixtureReceipt','Current typed receipt differs.');
receipt=jsondecode(fileread(path));
assert(receipt.passed&&receipt.original_79_semantics_match&&receipt.lower_loop_details_ready&&receipt.COM_closed);
names=["IMU_GYRO_CUTOFF","IMU_DGYRO_CUTOFF","IMU_GYRO_NF0_FRQ","IMU_GYRO_NF0_BW",...
    "IMU_GYRO_NF1_FRQ","IMU_GYRO_NF1_BW","IMU_GYRO_DNF_EN"];
types=[9,9,9,9,9,9,6];values=zeros(1,7);typed=cell(1,7);
rows=receipt.lower_loop_validation.rows;
for k=1:7
    at=find(string({rows.name})==names(k));assert(numel(at)==1&&rows(at).passed);
    row=rows(at).observation;v=row.typed_value;
    assert(isempty(row.read_error)&&row.write_count==0&&v.mav_type==types(k)&&strcmp(v.name,names(k)));
    bits=char(v.raw_bits_hex);assert(~isempty(regexp(bits,'^[A-Fa-f0-9]{8}$','once')));
    raw=uint32(hex2dec(bits));if types(k)==9,x=double(typecast(raw,'single'));else,x=double(typecast(raw,'int32'));end
    assert(isfinite(x)&&x==v.decoded);values(k)=x;typed{k}=v;
end
assert(isequal(values,[40,30,0,20,0,20,0]),'gpenmpc:SensorFixtureReceipt','Observed static-filter identity changed.');
cfg=struct('sample_rate_hz',fs,'gyro_cutoff_hz',values(1),'dgyro_cutoff_hz',values(2),...
    'notch0_frequency_hz',values(3),'notch0_bandwidth_hz',values(4),...
    'notch1_frequency_hz',values(5),'notch1_bandwidth_hz',values(6),...
    'dynamic_notch_enable',values(7),'offset_sensor',zeros(3,1),'bias_body',zeros(3,1),...
    'scale',ones(3,1),'mount_rotation',eye(3),'initial_gyro_uncalibrated',zeros(3,1),...
    'initial_acceleration_uncalibrated',zeros(3,1),'parameter_provenance','EXPLICIT_HOST_FIXTURE');
evidence=struct('typed_source',source,'typed_values',{typed},'sample_rate_hz',fs,...
    'actual_selected_gyro_raw_rate_known',false,'ratemax_used_as_fs',false,...
    'host_fixture_fields',{{'sample_rate_hz','offset_sensor=zeros','bias_body=zeros',...
    'mount_rotation=identity','initial_gyro_uncalibrated=zeros','initial_acceleration_uncalibrated=zeros'}},...
    'scale_role','Explicit unity: consumed Gyroscope::Correct has no scale term',...
    'claim','Static filter parameters with synthetic sensor inputs.',...
    'COM_actions',0,'board_actions',0);
end
function result=identity(path)
f=fopen(path,'rb');assert(f>=0);c=onCleanup(@()fclose(f)); %#ok<NASGU>
b=fread(f,Inf,'*uint8');d=java.security.MessageDigest.getInstance('SHA-256');d.update(b);
result=struct('path',path,'bytes',numel(b),'sha256',upper(reshape(dec2hex(typecast(d.digest(),'uint8'),2).',1,[])));
end
