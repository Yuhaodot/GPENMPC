function installation=buildBoardImuInstallationRotation(parameters)
%BUILDBOARDIMUINSTALLATIONROTATION Pure PX4 ROTATION_NONE level adjustment.
% PX4 Utilities.cpp:GetSensorLevelAdjustment; Accelerometer/Gyroscope.cpp:
% _rotation = Dcmf(Eulerf(x,y,z)) * get_rot_matrix(rotation).
% This helper deliberately supports only the observed SENS_BOARD_ROT=0.
% It models the ROTATION component of calibration, not offset/scale/thermal
% corrections. It reads no parameters or board, and changes no plant state.
names={'SENS_BOARD_ROT','SENS_BOARD_X_OFF','SENS_BOARD_Y_OFF','SENS_BOARD_Z_OFF'};
assert(isstruct(parameters)&&isscalar(parameters)&&all(isfield(parameters,names)), ...
    'm600check:InstallationIdentityMissing','All four source parameter values are required.');
for k=1:numel(names)
    v=parameters.(names{k});
    assert(isnumeric(v)&&isreal(v)&&isscalar(v)&&isfinite(v), ...
        'm600check:InstallationIdentityNonfinite','Installation parameter must be finite real scalar.');
end
assert(parameters.SENS_BOARD_ROT==0,'m600check:InstallationRotationUnsupported', ...
    'Only the verified ROTATION_NONE identity is implemented; do not guess rotation enums.');
angles=double([parameters.SENS_BOARD_X_OFF,parameters.SENS_BOARD_Y_OFF,parameters.SENS_BOARD_Z_OFF]);
xyz=angles*pi/180;x=xyz(1);y=xyz(2);z=xyz(3);
Rx=[1,0,0;0,cos(x),-sin(x);0,sin(x),cos(x)];
Ry=[cos(y),0,sin(y);0,1,0;-sin(y),0,cos(y)];
Rz=[cos(z),-sin(z),0;sin(z),cos(z),0;0,0,1];
L=Rz*Ry*Rx;
installation=struct('schema','PX4_IMU_ROTATION_NONE_LEVEL_ADJUSTMENT_V1', ...
    'parameters',parameters,'angles_degrees',angles,'sensor_to_body',L, ...
    'body_to_sensor',L.','rotation_order','Rz(z)*Ry(y)*Rx(x)', ...
    'calibration_scope','ROTATION_COMPONENT_ONLY__OFFSET_SCALE_THERMAL_NOT_MODELLED', ...
    'application_boundary','FINAL_SPECIFIC_FORCE_AND_GYRO_SENSOR_FIELDS__NOT_KINEMATIC_ACCELERATION_OR_PLANT_POSE', ...
    'magnetometer_handled',false,'plant_state_modified',false,'board_actions',0);
end
