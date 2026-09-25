[CmdletBinding()]
param()

$ErrorActionPreference = 'Stop'

$buildRoot = (& (Join-Path $PSScriptRoot 'gpenmpc_external_path.ps1') 'host_build_root')
$matlabTools = Join-Path $buildRoot 'tools'
$modelBuild = Join-Path $buildRoot 'm600_coptersim'
$buildReceiptPath = Join-Path $modelBuild 'evidence\M600_COPTERSIM_DLL_BUILD_AND_LOAD_RESULT.json'
$runtime = (& (Join-Path $PSScriptRoot 'gpenmpc_external_path.ps1') 'm600_substitution_runtime')
$runtimeDll = Join-Path $runtime 'external\model\GPENMPC_M600_CopterSim.dll'
$phaseRoot = (& (Join-Path $PSScriptRoot 'gpenmpc_external_path.ps1') 'm600_model_validation_root')
$runId = 'M600_MODEL_HIL_' + (Get-Date -Format 'yyyyMMdd_HHmmss_fff')
$outputDir = Join-Path (Join-Path $phaseRoot 'runs') $runId
$outerReceiptPath = Join-Path $outputDir 'M600_HIL_EXECUTION.json'
$stdoutPath = Join-Path $outputDir 'MATLAB_STDOUT.log'
$startupProbePath = Join-Path $outputDir 'MATLAB_STARTUP_PREFLIGHT.log'
$windowCapturePath = Join-Path $outputDir 'COPTERSIM_M600_HIL_WINDOW.png'
$captureScript = Join-Path $modelBuild 'capture_coptersim_window.ps1'
$matlabExe = (& (Join-Path $PSScriptRoot 'gpenmpc_install_path.ps1') 'matlab' 'bin\matlab.exe')

foreach ($path in @($buildReceiptPath, $runtimeDll, $captureScript,
        (Join-Path $matlabTools 'run_m600_model_hil_smoke.m'),
        (Join-Path $runtime 'CopterSim.exe'),
        $matlabExe)) {
    if (-not (Test-Path -LiteralPath $path -PathType Leaf)) {
        throw "Required HIL input is missing: $path"
    }
}

$existingCopterSim = @(Get-Process -Name CopterSim -ErrorAction SilentlyContinue)
if ($existingCopterSim.Count -ne 0) {
    throw 'A CopterSim process is already using the serial connection.'
}

$buildReceipt = Get-Content -LiteralPath $buildReceiptPath -Raw | ConvertFrom-Json
$expectedDllHash = [string]$buildReceipt.dll.sha256
$runtimeDllHash = (Get-FileHash -Algorithm SHA256 -LiteralPath $runtimeDll).Hash
if ($runtimeDllHash -ne $expectedDllHash) {
    throw 'Runtime M600 DLL hash does not match the frozen host build receipt.'
}

New-Item -ItemType Directory -Path $outputDir -Force | Out-Null
$copterProcess = $null
$moduleBefore = $null
$moduleAfter = $null
$matlabExit = $null
$matlabResult = $null
$matlabStartupExit = $null
$matlabStartupPreflightPassed = $false
$failure = $null

try {
    $startupOutput = (& $matlabExe -batch "disp('GPENMPC_MATLAB_STARTUP_OK'); exit(0);" 2>&1) -join "`n"
    $matlabStartupExit = $LASTEXITCODE
    Set-Content -LiteralPath $startupProbePath -Value $startupOutput -Encoding utf8
    if ($matlabStartupExit -ne 0 -or $startupOutput -notmatch 'GPENMPC_MATLAB_STARTUP_OK') {
        throw "MATLAB startup preflight failed before CopterSim or COM3 ownership. Exit code: $matlabStartupExit"
    }
    $matlabStartupPreflightPassed = $true

    $arguments = @(
        '1', '1', '-1', 'GPENMPC_M600_CopterSim', '0', 'Grasslands',
        '0', '0', '0', '0', '3:921600', '2'
    )
    $copterProcess = Start-Process `
        -FilePath (Join-Path $runtime 'CopterSim.exe') `
        -ArgumentList $arguments `
        -WorkingDirectory $runtime `
        -WindowStyle Hidden `
        -PassThru
    Start-Sleep -Seconds 12
    $copterProcess = Get-Process -Id $copterProcess.Id -ErrorAction Stop
    if (-not $copterProcess.Responding) {
        throw 'CopterSim is not responding after startup.'
    }

    $moduleBefore = $copterProcess.Modules |
        Where-Object { $_.ModuleName -eq 'GPENMPC_M600_CopterSim.dll' } |
        Select-Object -First 1
    if ($null -eq $moduleBefore) {
        throw 'CopterSim did not load GPENMPC_M600_CopterSim.dll.'
    }
    if ((Get-FileHash -Algorithm SHA256 -LiteralPath $moduleBefore.FileName).Hash -ne $expectedDllHash) {
        throw 'The DLL loaded by CopterSim does not match the frozen hash.'
    }

    $handle = [int64]$copterProcess.MainWindowHandle
    if ($handle -gt 0) {
        & $captureScript -WindowHandle $handle -OutputPath $windowCapturePath | Out-Null
    }

    $matlabExpression = @"
try
  addpath('$($matlabTools.Replace("'", "''"))');
  r = run_m600_model_hil_smoke('$($outputDir.Replace("'", "''"))');
  disp(jsonencode(r, PrettyPrint=true));
  if ~startsWith(string(r.status), "PASS_")
    exit(2);
  end
catch ME
  disp(getReport(ME, 'extended', 'hyperlinks', 'off'));
  exit(1);
end
"@
    $matlabOutput = (& $matlabExe -batch $matlabExpression 2>&1) -join "`n"
    $matlabExit = $LASTEXITCODE
    Set-Content -LiteralPath $stdoutPath -Value $matlabOutput -Encoding utf8

    $resultPath = Join-Path $outputDir 'RESULT.json'
    if (-not (Test-Path -LiteralPath $resultPath -PathType Leaf)) {
        throw "MATLAB did not create RESULT.json. Exit code: $matlabExit"
    }
    $matlabResult = Get-Content -LiteralPath $resultPath -Raw | ConvertFrom-Json
    if ($matlabExit -ne 0 -or
            $matlabResult.status -ne 'PASS_EFFECTIVE_M600_COPTERSIM_PIXHAWK_HIL') {
        throw "M600 model HIL did not pass. MATLAB exit code: $matlabExit"
    }

    $copterProcess = Get-Process -Id $copterProcess.Id -ErrorAction Stop
    $moduleAfter = $copterProcess.Modules |
        Where-Object { $_.ModuleName -eq 'GPENMPC_M600_CopterSim.dll' } |
        Select-Object -First 1
    if ($null -eq $moduleAfter -or
            (Get-FileHash -Algorithm SHA256 -LiteralPath $moduleAfter.FileName).Hash -ne $expectedDllHash) {
        throw 'CopterSim M600 DLL identity changed or disappeared during HIL.'
    }
} catch {
    $failure = $_.Exception.Message
} finally {
    if ($null -ne $copterProcess) {
        $live = Get-Process -Id $copterProcess.Id -ErrorAction SilentlyContinue
        if ($live) {
            Stop-Process -Id $live.Id -Force
            Wait-Process -Id $live.Id -Timeout 8 -ErrorAction SilentlyContinue
        }
    }
}

$passed = $null -eq $failure -and
    $null -ne $matlabResult -and
    $matlabResult.status -eq 'PASS_EFFECTIVE_M600_COPTERSIM_PIXHAWK_HIL' -and
    $null -ne $moduleBefore -and
    $null -ne $moduleAfter

$outerReceipt = [ordered]@{
    schema = 'GPENMPC_M600_COPTERSIM_PIXHAWK_HIL_EXECUTION_V1'
    status = if ($passed) {
        'PASS_M600_COPTERSIM_PIXHAWK_HIL'
    } else {
        'FAIL_M600_COPTERSIM_PIXHAWK_HIL'
    }
    run_id = $runId
    evidence_layer = if ($passed) {
        'PIXHAWK_HIL_HARDWARE_CLOSED_LOOP'
    } else {
        'FAILED_HIL_SESSION'
    }
    claim_limit = 'EFFECTIVE_M600_RESEARCH_MODEL_HIL'
    topology = [ordered]@{
        plant = 'CopterSim_GPENMPC_M600_CopterSim_DLL'
        controller = 'Pixhawk6C_PX4_OFFBOARD_POSITION_CONTROL'
        high_level_and_logging = 'MATLAB'
        display = 'RflySim3D'
        python_hil_driver = $false
    }
    model_dll = [ordered]@{
        path = $runtimeDll
        sha256 = $expectedDllHash
        process_load_before_hil = $null -ne $moduleBefore
        process_load_after_hil = $null -ne $moduleAfter
    }
    matlab_exit_code = $matlabExit
    matlab_startup_preflight_exit_code = $matlabStartupExit
    matlab_startup_preflight_passed = $matlabStartupPreflightPassed
    matlab_startup_preflight_path = $startupProbePath
    matlab_hil_runner_entered = Test-Path -LiteralPath (Join-Path $outputDir 'RESULT.json')
    control_commands_claimable = $passed -or $null -ne $matlabResult
    matlab_result_path = Join-Path $outputDir 'RESULT.json'
    telemetry_path = Join-Path $outputDir 'TELEMETRY.csv'
    matlab_stdout_path = $stdoutPath
    coptersim_window_capture_path = if (Test-Path -LiteralPath $windowCapturePath) {
        $windowCapturePath
    } else {
        $null
    }
    failure = $failure
    generated_utc = [DateTime]::UtcNow.ToString('o')
}
$outerReceipt | ConvertTo-Json -Depth 8 |
    Set-Content -LiteralPath $outerReceiptPath -Encoding utf8
$outerReceipt | ConvertTo-Json -Depth 8

if (-not $passed) {
    exit 1
}
