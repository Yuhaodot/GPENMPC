%% GPENMPC M600 CopterSim model initialization
% This file is executed by the official RflySim Simulink DLL template.
% Initializes vehicle parameters for the effective M600 model. Use with the
% current MATLAB plant kernel for rotor-speed lag, force mapping, drag-frame
% transforms and ground-contact dynamics.
%
% Official RflySim source model SHA-256:
%   1BD06222AFD5E662D433DC50B81EF9804CD3084763A5EF5D99E2B95764505873
% GPENMPC platform profile SHA-256:
%   406786CE01DB0B1F66B1AE305D73C9421AC03FCB5B5F5E2664D708106DD9D09B
% GPENMPC dynamics calibration SHA-256:
%   ACF09DFA521897B55D5EC473E44DCC4F546356B36465F56D2C3F1D628409583E
%
% The effective six-degree-of-freedom research model uses published platform
% values and model priors for inertia and actuator force/torque coefficients.

load MavLinkStruct;

%% Vehicle and visualization identities
ModelParam_3DType = int16(5);   % RflySim generic hexarotor visualization
ModelParam_uavType = int16(5);  % RflySim hexarotor X force/moment geometry

ModelInit_PosE = [0, 0, 0];
ModelInit_AngEuler = [0, 0, 0];
ModelInit_VelB = [0, 0, 0];
ModelInit_RateB = [0, 0, 0];

% This model initialization uses a fixed 2.27 kg payload.
GPENMPC_M600_BaseMassKg = 9.5;
GPENMPC_M600_PayloadKg = 2.27;
ModelParam_uavMass = GPENMPC_M600_BaseMassKg + GPENMPC_M600_PayloadKg;

% The allocation model uses fixed inertia without a payload parallel-axis term.
GPENMPC_M600_BaseInertiaKgM2 = [1.6, 1.6, 3.0];
ModelParam_uavJ = diag(GPENMPC_M600_BaseInertiaKgM2);

%% World/GPS origin
ModelParam_GPSLatLong = [40.1540302, 116.2593683];
ModelParam_envAltitude = -50;

%% RflySim parameter APIs
FaultParamAPI.FaultInParams = zeros(32, 1);
FaultParamAPI.FaultInParams(3) = 1;
FaultParamAPI.InitInParams = zeros(32, 1);
FaultParamAPI.InitInParams(1:3) = [1, 2, 3];

%% Six-rotor effective actuator model
ModelParam_uavMotNumbs = int8(6);
ModelParam_motorMinThr = 0.05;
ModelParam_motorCr = 842.1;
ModelParam_motorWb = 22.83;
ModelParam_motorT = 0.12;
ModelParam_motorJm = 0.0; % GPENMPC 19-state plant excludes rotor gyro torque

% Scale the effective force coefficient to the per-rotor thrust limit at
% the template's 0.95 input saturation.
GPENMPC_M600_PerRotorUpperN = 32.145727009134916;
GPENMPC_M600_MotorInputUpper = 0.95;
GPENMPC_M600_MaxInternalRotorRadS = ModelParam_motorWb + ...
    ModelParam_motorCr .* GPENMPC_M600_MotorInputUpper;
ModelParam_rotorCt = GPENMPC_M600_PerRotorUpperN ./ ...
    GPENMPC_M600_MaxInternalRotorRadS.^2;

% Effective yaw moment arm: 0.025 m.
GPENMPC_M600_YawMomentArmM = 0.025;
ModelParam_rotorCm = ModelParam_rotorCt .* GPENMPC_M600_YawMomentArmM;
ModelInit_RPM = 0;
ModelInit_Inputs = zeros(1, 16);

%% Effective force/moment and aerodynamic parameters
ModelParam_uavR = 0.5665; % six-rotor allocation arm radius, m
ModelParam_uavCd = 0.0634905529323215; % effective N/(m/s)^2 per axis
ModelParam_uavCCm = [0, 0, 0]; % matches current GPENMPC rigid-body plant
ModelParam_uavDearo = 0.0;     % drag applied at centre of mass
