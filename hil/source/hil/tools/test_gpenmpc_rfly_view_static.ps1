# Check source and assembly without invoking Windows UI APIs.
param([string]$BackupRoot = (& (Join-Path $PSScriptRoot 'gpenmpc_external_path.ps1') 'display_reference_source'))
$ErrorActionPreference = 'Stop'
$passed = [Collections.Generic.List[string]]::new()
function Check([string]$name, [bool]$condition) {
    if (-not $condition) { throw "Check failed: $name" }
    $passed.Add($name)
}
function Section([string]$source, [string]$begin, [string]$end) {
    $start = $source.IndexOf($begin, [StringComparison]::Ordinal)
    if ($start -lt 0) { throw "Missing section: $begin" }
    $finish = $source.IndexOf($end, $start + $begin.Length, [StringComparison]::Ordinal)
    if ($finish -lt 0) { throw "Missing section end: $end" }
    return $source.Substring($start, $finish - $start).Replace("`r`n", "`n")
}
$console = [IO.File]::ReadAllText((Join-Path $PSScriptRoot 'gpenmpc_demo_console.m'))
$original = [IO.File]::ReadAllText((Join-Path $BackupRoot 'gpenmpc_demo_console.m'))
$entry = [IO.File]::ReadAllText((Join-Path $PSScriptRoot 'gpenmpc_demo_entry.m'))
$ensure = [IO.File]::ReadAllText((Join-Path $PSScriptRoot 'ensure_gpenmpc_rfly_view.m'))
$runner = [IO.File]::ReadAllText((Join-Path $PSScriptRoot 'start_gpenmpc_usb_manual.m'))
$helper = [IO.File]::ReadAllText((Join-Path $PSScriptRoot 'RflyViewWindow.cs'))
Check 'explicit_open_including_busy_session' ($console.Contains('if string(initialMode)=="LIVE",showViewExplicitly();end'))
Check 'explicit_start_requests_visible_window' ($console.Contains('receipt=ensure_gpenmpc_rfly_view([],[],true);'))
Check 'reopen_event_rechecks_view' ($entry.Contains("isappdata(f(1),'GPENMPCShow3DView')") -and $entry.Contains('showView();'))
Check 'background_default_does_not_present' ($ensure.Contains('if nargin<3||isempty(presentWindow),presentWindow=false;end'))
Check 'runtime_keeps_readiness_only_call' ($runner.Contains('info.visualization=ensure_gpenmpc_rfly_view();'))
Check 'owner_lock_unchanged' ((Section $console '    function yes=ownerBusy()' '    function startManual(') -ceq (Section $original '    function yes=ownerBusy()' '    function startManual('))
Check 'start_gates_and_launch_unchanged' ((Section $console '    function startManual(' '    function ready=prepareView()') -ceq (Section $original '    function startManual(' '    function ready=prepareView()'))
Check 'reset_operation_unchanged' ((Section $console '    function resetManual()' '    function clearCurves()') -ceq (Section $original '    function resetManual()' '    function clearCurves()'))
Check 'periodic_monitor_unchanged' ((Section $console '    function refresh()' '    function refreshRunStatus(') -ceq (Section $original '    function refresh()' '    function refreshRunStatus('))
Check 'state_gates_unchanged' ((Section $console '    function refreshRunStatus(' '    function txt=progressText(') -ceq (Section $original '    function refreshRunStatus(' '    function txt=progressText('))
Check 'configured_official_renderer_path' ($helper.Contains('Environment.GetEnvironmentVariable("GPENMPC_RFLY_ROOT")') -and $helper.Contains('@"RflySim3D\RflySim3D\Binaries\Win64\RflySim3D.exe"'))
Check 'window_owner_revalidated' ($helper.Contains('owner != (uint)pid') -and $helper.Contains('process.MainWindowHandle != window'))
Check 'no_process_or_input_synthesis' (-not ($helper -match 'Process\.Start|\.Kill\(|SendInput\(|SendKeys|AttachThreadInput\('))
Check 'single_foreground_attempt' ([regex]::Matches($helper, 'SetForegroundWindow\(window\);').Count -eq 1)
Add-Type -Path (Join-Path $PSScriptRoot 'RflyViewWindow.dll')
$at = [GPENMPC.Display.RflyViewWindow]::CenterInWorkArea(800, 600, 0, 0, 1920, 1080)
Check 'center_primary_monitor' ($at[0] -eq 560 -and $at[1] -eq 240)
$at = [GPENMPC.Display.RflyViewWindow]::CenterInWorkArea(800, 600, -1920, 0, 0, 1080)
Check 'center_negative_coordinate_monitor' ($at[0] -eq -1360 -and $at[1] -eq 240)
$at = [GPENMPC.Display.RflyViewWindow]::CenterInWorkArea(2500, 1200, 0, 0, 1920, 1080)
Check 'oversize_window_keeps_title_visible' ($at[0] -eq 0 -and $at[1] -eq 0)
$rejected = $false
try { [void][GPENMPC.Display.RflyViewWindow]::CenterInWorkArea(0, 600, 0, 0, 1920, 1080) } catch { $rejected = $true }
Check 'invalid_geometry_rejected' $rejected
[pscustomobject]@{
    passed = $true
    checks = $passed.ToArray()
    count = $passed.Count
    source_sha256 = (Get-FileHash -LiteralPath (Join-Path $PSScriptRoot 'RflyViewWindow.cs')).Hash
    assembly_sha256 = (Get-FileHash -LiteralPath (Join-Path $PSScriptRoot 'RflyViewWindow.dll')).Hash
    window_operations = 0
    simulator_starts = 0
    board_actions = 0
} | ConvertTo-Json -Depth 4
