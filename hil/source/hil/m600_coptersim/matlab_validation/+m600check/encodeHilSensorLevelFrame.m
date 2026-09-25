function [wire,L]=encodeHilSensorLevelFrame(body)
%#codegen
%ENCODEHILSENSORLEVELFRAME Fixed, single body-to-virtual-sensor transform.
% The official SensorOutput has already produced specific force (one gravity
% subtraction), gyro and body magnetic field with its original sensor model.
% PX4 v1.16 calibrates all three SIMULATION sensors using L * raw. Encode
% raw=L' * body only here, never in the plant, Kinematics, DCM or truth path.
% Exact required REAL32 values represented as doubles: X/Y/Z deg and ROT=0.
% This function is bound to those board parameters; it cannot be selected
% from observed attitude/error or used with unknown/different board leveling.
assert(isa(body,'double')&&isreal(body)&&isequal(size(body),[30,1]));
phi=6.445373058319092*pi/180;
theta=-6.442468166351318*pi/180;
psi=0;
cp=cos(phi);sp=sin(phi);ct=cos(theta);st=sin(theta);cy=cos(psi);sy=sin(psi);
L=[ct*cy, -cp*sy+sp*st*cy, sp*sy+cp*st*cy; ...
   ct*sy, cp*cy+sp*st*sy, -sp*cy+cp*st*sy; ...
   -st, sp*ct, cp*ct];
wire=body;
wire(2:4)=L.'*body(2:4);
wire(5:7)=L.'*body(5:7);
wire(8:10)=L.'*body(8:10);
% Channels 1 and 11:30, including time/baro/temperature/flags/reserved, are
% bitwise pass-through. HILGPS30d and VehileInfo60d do not enter this function.
end
