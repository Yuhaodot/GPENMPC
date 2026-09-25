param([Parameter(Mandatory=$true)][string]$OutputRoot)
$ErrorActionPreference='Stop'
$runtime=(& (Join-Path $PSScriptRoot 'gpenmpc_external_path.ps1') 'sensor_aligned_runtime')
$exe=Join-Path $runtime 'CopterSim.exe'
$dll=Join-Path $runtime 'external\model\GPENMPC_M600_Canonical.dll'
$expectedExe='BD3139D6D4BC665C254AD7C3C17618CF91D18138205D671FD88B428C0C017917'
$expectedDll='80C95FA673EC462EB3108016A0F53FE4A7A50A7FD9F016DFB57EE9345A09EAE4'
$fixedArgs='1 1 -1 GPENMPC_M600_Canonical 3 LowGPU 0 0 0 0 1 2'
if(Test-Path -LiteralPath $OutputRoot){throw 'Existing evidence may not be overwritten.'}
if((Get-FileHash -LiteralPath $exe -Algorithm SHA256).Hash -ne $expectedExe){throw 'Executable identity mismatch.'}
if((Get-FileHash -LiteralPath $dll -Algorithm SHA256).Hash -ne $expectedDll){throw 'DLL identity mismatch.'}
if(@(Get-Process -Name CopterSim,CopterSimNoUI -ErrorAction SilentlyContinue).Count -ne 0){throw 'Existing CopterSim: no process interference allowed.'}
$null=New-Item -ItemType Directory -Path $OutputRoot
$rows=[Collections.Generic.List[object]]::new()
$cleanup=[Collections.Generic.List[string]]::new()
$owned=$null;$ownedId=0;$ownedTicks=0L;$stdout=$null;$stderr=$null;$copyOut=$null;$copyErr=$null
$failure=$null;$forced=$false;$closed=$false;$watch=[Diagnostics.Stopwatch]::StartNew()
function ProbeFreshModules {
    param([int]$OwnedId,[long]$OwnedTicks,[string]$ExpectedExePath)
    $snapshotProcess=Get-Process -Id $OwnedId -ErrorAction Stop
    if($snapshotProcess.StartTime.ToUniversalTime().Ticks -ne $OwnedTicks){throw 'PID identity changed.'}
    if([IO.Path]::GetFullPath($snapshotProcess.Path) -ne [IO.Path]::GetFullPath($ExpectedExePath)){throw 'Executable path changed.'}
    $snapshotModules=@($snapshotProcess.Modules)
    $snapshotMatches=@($snapshotModules | Where-Object ModuleName -eq 'GPENMPC_M600_Canonical.dll')
    $snapshotEvidence=@($snapshotMatches | ForEach-Object {
        [pscustomobject]@{name=$_.ModuleName;path=$_.FileName;sha256=(Get-FileHash -LiteralPath $_.FileName -Algorithm SHA256).Hash}
    })
    return [pscustomobject]@{process_type=$snapshotProcess.GetType().FullName;module_count=$snapshotModules.Count;matches=$snapshotEvidence;match_count=$snapshotMatches.Count}
}
try {
    $info=[Diagnostics.ProcessStartInfo]::new($exe,$fixedArgs)
    $info.WorkingDirectory=$runtime;$info.UseShellExecute=$false;$info.CreateNoWindow=$true;$info.WindowStyle=[Diagnostics.ProcessWindowStyle]::Hidden
    $info.RedirectStandardOutput=$true;$info.RedirectStandardError=$true
    $info.EnvironmentVariables['QT_FORCE_STDERR_LOGGING']='1'
    $info.EnvironmentVariables['QT_MESSAGE_PATTERN']='[%{time process} %{type} pid=%{pid} tid=%{threadid}] %{message}'
    $info.EnvironmentVariables['OMP_NUM_THREADS']='1';$info.EnvironmentVariables['MKL_NUM_THREADS']='1';$info.EnvironmentVariables['OPENBLAS_NUM_THREADS']='1'
    $stdout=[IO.File]::Open((Join-Path $OutputRoot 'COPTER_STDOUT.log'),[IO.FileMode]::CreateNew,[IO.FileAccess]::Write,[IO.FileShare]::Read)
    $stderr=[IO.File]::Open((Join-Path $OutputRoot 'COPTER_STDERR.log'),[IO.FileMode]::CreateNew,[IO.FileAccess]::Write,[IO.FileShare]::Read)
    $owned=[Diagnostics.Process]::new();$owned.StartInfo=$info
    if(-not $owned.Start()){throw 'Failed to start owned no-board process.'}
    $ownedId=$owned.Id;$ownedTicks=$owned.StartTime.ToUniversalTime().Ticks
    $copyOut=$owned.StandardOutput.BaseStream.CopyToAsync($stdout);$copyErr=$owned.StandardError.BaseStream.CopyToAsync($stderr)
    try{$owned.PriorityClass=[Diagnostics.ProcessPriorityClass]::BelowNormal}catch{$cleanup.Add('Priority diagnostic: '+$_.Exception.Message)}
    $sampleWatch=[Diagnostics.Stopwatch]::StartNew();$nextSample=0.0
    while($sampleWatch.Elapsed.TotalSeconds -le 5.2 -and $watch.Elapsed.TotalSeconds -lt 7.0){
        if($owned.HasExited){throw 'Owned no-board process exited early.'}
        $sample=[ordered]@{t_s=$sampleWatch.Elapsed.TotalSeconds;utc=[DateTime]::UtcNow.ToString('o');first=$null;immediate_second=$null;error=$null}
        try {
            $sample.first=ProbeFreshModules -OwnedId $ownedId -OwnedTicks $ownedTicks -ExpectedExePath $exe
            $sample.immediate_second=ProbeFreshModules -OwnedId $ownedId -OwnedTicks $ownedTicks -ExpectedExePath $exe
        } catch {$sample.error=$_.Exception.ToString()}
        $rows.Add([pscustomobject]$sample)
        $nextSample+=0.1
        $remainingMs=[int](1000*($nextSample-$sampleWatch.Elapsed.TotalSeconds))
        if($remainingMs -gt 0){Start-Sleep -Milliseconds $remainingMs}
    }
} catch {$failure=$_.Exception.ToString()} finally {
    if($null -ne $owned -and $ownedId -gt 0){
        try {
            if(-not $owned.HasExited){
                $current=Get-Process -Id $ownedId -ErrorAction Stop
                if($current.StartTime.ToUniversalTime().Ticks -ne $ownedTicks -or [IO.Path]::GetFullPath($current.Path) -ne [IO.Path]::GetFullPath($exe)){throw 'Cleanup ownership mismatch; no process action.'}
                $closed=$owned.CloseMainWindow()
                if(-not $owned.WaitForExit(2000)){$owned.Kill();$forced=$true;if(-not $owned.WaitForExit(4000)){throw 'Owned process remained alive.'}}
            }
        } catch {$cleanup.Add($_.Exception.ToString())}
    }
    foreach($task in @($copyOut,$copyErr)){if($null -ne $task){try{if(-not $task.Wait(1000)){throw 'Log drain timed out.'}}catch{$cleanup.Add($_.Exception.ToString())}}}
    foreach($stream in @($stdout,$stderr)){if($null -ne $stream){try{$stream.Flush($true);$stream.Dispose()}catch{$cleanup.Add($_.Exception.ToString())}}}
}
$released=$ownedId -eq 0 -or $owned.HasExited
$moduleRows=@($rows | Where-Object {$null -ne $_.first -and $_.first.match_count -gt 0})
$firstPresent=$null;if($moduleRows.Count -gt 0){$firstPresent=$moduleRows[0].t_s}
$disappeared=@($rows | Where-Object {$null -ne $firstPresent -and $_.t_s -gt $firstPresent -and $null -ne $_.first -and $_.first.match_count -eq 0})
$pairDisagree=@($rows | Where-Object {$null -ne $_.first -and $null -ne $_.immediate_second -and $_.first.match_count -ne $_.immediate_second.match_count})
$log=Get-Content -LiteralPath (Join-Path $OutputRoot 'COPTER_STDERR.log') -Raw
$logShowsMode3=$log -match 'm_simModeIndex 3'
$serialConnected=$log -match 'SerialPort.*Connected|SerialPort successful|Open serial|COM3'
$result=[ordered]@{
    schema='OWNED_NO_BOARD_MODE3_MODULE_SNAPSHOTS_V1';status='HOST_ONLY_MODULE_LOAD_DIAGNOSTIC_NOT_HIL'
    hypothesis='Distinguish real startup module unload from wrapper variable/snapshot defects using repeated fresh Process.Modules pairs.'
    runtime=$runtime;arguments=$fixedArgs;exe_sha256=$expectedExe;dll_sha256=$expectedDll
    pid=$ownedId;process_start_ticks=$ownedTicks;wall_s=$watch.Elapsed.TotalSeconds;rows=$rows
    first_present_s=$firstPresent;observations=$rows.Count;present_count=$moduleRows.Count;disappeared_after_first_count=$disappeared.Count;pair_disagreement_count=$pairDisagree.Count
    failure=$failure;cleanup_errors=$cleanup;close_main_window_requested=$closed;owned_no_board_force_kill=$forced;owned_process_released=$released
    vendor_mode3_observed=$logShowsMode3;serial_connection_log_observed=$serialConnected;COM_open_by_script=0;board_commands=0;UDP_socket_created_by_script=0;live_mode_allowed=$false
    limitations='This probe captures mode-3 module lifecycle events.'
}
[IO.File]::WriteAllText((Join-Path $OutputRoot 'RESULT.json'),($result|ConvertTo-Json -Depth 12))
$result|Select-Object status,pid,wall_s,observations,first_present_s,present_count,disappeared_after_first_count,pair_disagreement_count,failure,owned_process_released,vendor_mode3_observed,serial_connection_log_observed|ConvertTo-Json
if($null -ne $owned){$owned.Dispose()}
if($null -ne $failure -or -not $released -or $serialConnected -or -not $logShowsMode3){exit 2}
