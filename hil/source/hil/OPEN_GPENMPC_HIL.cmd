@echo off
setlocal
cd /d "%~dp0"
start "" powershell.exe -NoProfile -WindowStyle Hidden -File "%~dp0tools\launch_gpenmpc_demo.ps1"
endlocal
