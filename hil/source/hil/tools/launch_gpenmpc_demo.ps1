# Launch the viewer without starting a hardware session.
$ErrorActionPreference = 'Stop'
$buildPath = Split-Path $PSScriptRoot -Parent
$logRoot = $env:GPENMPC_HIL_LOG_ROOT
if ([string]::IsNullOrWhiteSpace($logRoot)) { $logRoot = Join-Path (Split-Path (Split-Path $buildPath -Parent) -Parent) 'logs' }
$logPath = Join-Path $logRoot 'Demo_UI_ARTIFACTS'
$mutex = [Threading.Mutex]::new($false, 'Local\GPENMPCHILDemoViewer')
$owns = $false
try {
    try { $owns = $mutex.WaitOne(0) } catch [Threading.AbandonedMutexException] { $owns = $true }
    if (-not $owns) {
        try { $focusEvent = [Threading.EventWaitHandle]::OpenExisting('Local\GPENMPCHILDemoFocus'); $null = $focusEvent.Set(); $focusEvent.Dispose() } catch { }
        exit 0
    }
    $matlabRoot = $env:GPENMPC_MATLAB_ROOT
    if ([string]::IsNullOrWhiteSpace($matlabRoot)) {
        $matlabCommand = Get-Command matlab.exe -CommandType Application -ErrorAction SilentlyContinue | Select-Object -First 1
        if (-not $matlabCommand) { throw 'Set GPENMPC_MATLAB_ROOT to the MATLAB installation directory or add matlab.exe to PATH.' }
        $matlabPath = $matlabCommand.Source
    } else {
        $matlabPath = Join-Path $matlabRoot 'bin\matlab.exe'
    }
    if (-not (Test-Path -LiteralPath $matlabPath -PathType Leaf)) { throw "MATLAB executable was not found: $matlabPath" }
    $matlabPath = [IO.Path]::GetFullPath($matlabPath)
    $focusEvent = [Threading.EventWaitHandle]::new($false, [Threading.EventResetMode]::AutoReset, 'Local\GPENMPCHILDemoFocus')
    $readyEvent = [Threading.EventWaitHandle]::new($false, [Threading.EventResetMode]::ManualReset, 'Local\GPENMPCHILDemoReady')
    $null = $readyEvent.Reset()
    Add-Type -AssemblyName System.Windows.Forms
    Add-Type -AssemblyName System.Drawing
    $form = [Windows.Forms.Form]::new()
    $form.Text = 'GPENMPC HIL · Opening'
    $form.Size = [Drawing.Size]::new(540,175)
    $form.StartPosition = 'CenterScreen'
    $form.FormBorderStyle = 'FixedDialog'
    $form.MaximizeBox = $false
    $form.ControlBox = $false
    $label = [Windows.Forms.Label]::new()
    $label.Dock = 'Fill'; $label.Padding = [Windows.Forms.Padding]::new(24)
    $label.Font = [Drawing.Font]::new('Microsoft YaHei',11)
    $form.Controls.Add($label)
    $form.Show()
    New-Item -ItemType Directory -Path $logPath -Force | Out-Null
    $logFile = Join-Path $logPath ('PANEL_' + (Get-Date -Format 'yyyyMMdd_HHmmss_fff') + '.log')
    $argsLine = '-nosplash -sd "' + $buildPath + '" -logfile "' + $logFile + '" -batch "addpath(fullfile(pwd,''tools'')); gpenmpc_demo_entry;"'
    $clock = [Diagnostics.Stopwatch]::StartNew()
    $label.Text = "Opening the flight-control dashboard...`nLoading session controls and live plots."
    [Windows.Forms.Application]::DoEvents()
    $child = Start-Process -FilePath $matlabPath -ArgumentList $argsLine -WindowStyle Hidden -PassThru
    $shown = $false
    while (-not $child.HasExited) {
        if (-not $shown -and $readyEvent.WaitOne(0)) { $shown = $true; $form.Hide() }
        if (-not $shown) {
            $label.Text = "Opening the dashboard: $([int]$clock.Elapsed.TotalSeconds) s`nLoading session controls and live plots."
            if ($clock.Elapsed.TotalSeconds -gt 90) { $label.Text += "`nStartup is taking longer than expected. Log: $logFile" }
        }
        [Windows.Forms.Application]::DoEvents()
        Start-Sleep -Milliseconds 100
        $child.Refresh()
    }
    $form.Dispose()
    if ($child.ExitCode -ne 0) {
        [Windows.Forms.MessageBox]::Show("The dashboard closed unexpectedly.`nLog: $logFile", 'Dashboard startup error') | Out-Null
    }
} catch {
    Add-Type -AssemblyName System.Windows.Forms
    [Windows.Forms.MessageBox]::Show($_.Exception.Message, 'Dashboard startup error') | Out-Null
} finally {
    if ($form) { $form.Dispose() }
    if ($readyEvent) { $readyEvent.Dispose() }
    if ($focusEvent) { $focusEvent.Dispose() }
    if ($owns) { $mutex.ReleaseMutex() }
    $mutex.Dispose()
}
