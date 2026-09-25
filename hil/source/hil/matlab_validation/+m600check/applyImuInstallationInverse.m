function output=applyImuInstallationInverse(bodySpecificForce,bodyAngularRate,installation)
%APPLYIMUINSTALLATIONINVERSE Pure L' transformation before PX4 rotation L.
% Transform specific force and angular rate from body to sensor axes.
% Gravity has ALREADY been included in bodySpecificForce. Applying this to
% kinematic acceleration before gravity subtraction is invalid.
required={'schema','parameters','sensor_to_body','body_to_sensor'};
assert(isstruct(installation)&&isscalar(installation)&&all(isfield(installation,required)), ...
    'm600check:InstallationIdentityMissing','Installation identity is missing.');
assert(strcmp(installation.schema,'PX4_IMU_ROTATION_NONE_LEVEL_ADJUSTMENT_V1'), ...
    'm600check:InstallationSchema','Unsupported installation identity.');
expected=m600check.buildBoardImuInstallationRotation(installation.parameters);
for key={'sensor_to_body','body_to_sensor'}
    R=installation.(key{1});
    assert(isnumeric(R)&&isreal(R)&&isequal(size(R),[3,3])&&all(isfinite(R),'all'), ...
        'm600check:InstallationMatrixNonfinite','Expected a finite real 3x3 rotation.');
    assert(norm(R*R.'-eye(3),'fro')<=1e-12&&abs(det(R)-1)<=1e-12, ...
        'm600check:InstallationMatrixNotRotation','Rotation must be proper orthogonal.');
    assert(norm(R-expected.(key{1}),'fro')<=1e-12,'m600check:InstallationMatrixIdentityMismatch', ...
        'Matrix differs from the declared parameter identity.');
end
for value={bodySpecificForce,bodyAngularRate}
    v=value{1};assert(isnumeric(v)&&isreal(v)&&ismatrix(v)&&size(v,1)==3&&size(v,2)>0&&all(isfinite(v),'all'), ...
        'm600check:ImuVectorInvalid','Inputs must be finite real 3-by-N vectors.');
end
assert(isequal(size(bodySpecificForce),size(bodyAngularRate)),'m600check:ImuVectorShape','IMU sample shapes differ.');
Linv=installation.body_to_sensor;
output=struct('raw_specific_force',Linv*double(bodySpecificForce), ...
    'raw_angular_rate',Linv*double(bodyAngularRate), ...
    'magnetometer_handled',false,'plant_state_modified',false);
end
