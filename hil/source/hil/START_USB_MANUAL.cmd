@echo off
setlocal
title GPENMPC HIL
set "GPENMPC_MATLAB_EXE="
if defined GPENMPC_MATLAB_ROOT set "GPENMPC_MATLAB_EXE=%GPENMPC_MATLAB_ROOT%\bin\matlab.exe"
if not defined GPENMPC_MATLAB_EXE for /f "delims=" %%M in ('where matlab.exe 2^>nul') do if not defined GPENMPC_MATLAB_EXE set "GPENMPC_MATLAB_EXE=%%M"
if not exist "%GPENMPC_MATLAB_EXE%" (
  echo Set GPENMPC_MATLAB_ROOT or add matlab.exe to PATH.
  pause
  exit /b 1
)
cd /d "%~dp0"
echo Opening the MATLAB HIL session.
echo Wait for USB_RC_ACTIVE before moving the sticks.
echo Ctrl+Shift+L requests landing and recovery. Keep MATLAB open during flight.
"%GPENMPC_MATLAB_EXE%" -nosplash -sd "%~dp0." -r "addpath(fullfile(pwd,'tools')); start_gpenmpc_usb_manual;"
endlocal
