param([Parameter(Mandatory=$true)][string]$OutputPath)
$ErrorActionPreference='Stop'
if(Test-Path -LiteralPath $OutputPath) { throw 'Use a new output directory.' }
$OutputPath=[IO.Path]::GetFullPath($OutputPath)
if(-not (Test-Path -LiteralPath ([IO.Path]::GetDirectoryName($OutputPath)))) { throw 'Output parent must exist.' }
$source=Join-Path $PSScriptRoot 'HostLoadSnapshot.cs'
Add-Type -Path $source
$sampler=[GPENMPC.HostDiagnostics.HostLoadSampler]::new()
$rows=[Collections.Generic.List[object]]::new()
$checks=[Collections.Generic.List[object]]::new()
$failure=$null
function Check([string]$name,[bool]$pass) {
    $checks.Add([pscustomobject]@{name=$name;pass=$pass})
    if(-not $pass) { throw "HOST_LOAD_ASSERTION: $name" }
}
try {
    # The only real PID queried is THIS test process. Invalid IDs make no API
    # call; its duplicate must be skipped. No MATLAB/CopterSim or other work.
    $owned=[int[]]@($PID,$PID,-1,0)
    $rows.Add($sampler.Sample($owned))
    Start-Sleep -Milliseconds 1050
    $rows.Add($sampler.Sample($owned))
    Check 'exactly_two_actual_samples' ($rows.Count -eq 2)
    Check 'first_cpu_delta_explicitly_unavailable' ($null -eq $rows[0].SystemCpuBusyPercent)
    Check 'second_cpu_delta_available_and_bounded' ($null -ne $rows[1].SystemCpuBusyPercent -and $rows[1].SystemCpuBusyPercent -ge 0 -and $rows[1].SystemCpuBusyPercent -le 100)
    Check 'monotonic_actual_elapsed_time' ($rows[1].StopwatchTicks -gt $rows[0].StopwatchTicks -and $rows[1].SystemCpuIntervalSeconds -ge 1)
    foreach($row in $rows) {
        $tag="sample_$($row.Sequence)"
        Check ($tag+'_system_times_and_ram') ($row.SystemTimesValid -and $row.MemoryValid -and $row.PhysicalTotalBytes -gt 0 -and $row.PhysicalAvailableBytes -le $row.PhysicalTotalBytes)
        Check ($tag+'_exact_one_owned_query_no_duplicate') ($row.QueryHandlesOpened -eq 1 -and $row.Processes.Count -eq 3)
        Check ($tag+'_query_handle_closed') ($row.QueryHandlesCloseAttempted -eq 1 -and $row.QueryHandlesClosed -eq 1 -and $row.QueryHandlesUnconfirmedClosed -eq 0)
        $own=$row.Processes | Where-Object ProcessId -eq $PID
        Check ($tag+'_owned_io_cpu_identity') ($own.IoCountersValid -and $own.ProcessTimesValid -and $own.CreationFileTime100ns -gt 0 -and $own.QueryHandleClosed -and $own.ExitObserved -eq $false)
        $invalid=@($row.Processes | Where-Object ProcessId -le 0)
        Check ($tag+'_invalid_pid_diagnostic_no_open') ($invalid.Count -eq 2 -and @($invalid | Where-Object QueryHandleOpened).Count -eq 0 -and @($invalid | Where-Object { $_.Diagnostics -contains 'INVALID_PID_NO_OPEN' }).Count -eq 2)
        Check ($tag+'_diagnostic_only_io_label') ($own.IoScope -eq 'PROCESS_FILE_NETWORK_DEVICE_OTHER_IO_NOT_PHYSICAL_DISK' -and $row.Use -eq 'DIAGNOSTIC_ONLY_NO_REJECTION_OR_RESOURCE_HOLD')
    }
    $first=$rows[0].Processes | Where-Object ProcessId -eq $PID
    $second=$rows[1].Processes | Where-Object ProcessId -eq $PID
    Check 'first_process_delta_explicitly_unavailable' (-not $first.DeltaValid -and $null -eq $first.ReadBytesPerSecond)
    Check 'second_same_process_delta_available' ($second.DeltaValid -and $second.CreationFileTime100ns -eq $first.CreationFileTime100ns -and $second.IntervalSeconds -ge 1)
    Check 'cpu_and_io_rates_nonnegative' ($second.CpuCoreEquivalent -ge 0 -and $second.ReadBytesPerSecond -ge 0 -and $second.WriteBytesPerSecond -ge 0 -and $second.OtherBytesPerSecond -ge 0)
    Check 'all_query_handles_closed_no_retained_handle' ((($rows | Measure-Object QueryHandlesOpened -Sum).Sum -eq 2) -and (($rows | Measure-Object QueryHandlesClosed -Sum).Sum -eq 2))
} catch { $failure=$_.Exception.ToString() }
finally {
    # Runtime result serialization only here, outside both sampling calls.
    $result=[ordered]@{
        status='HOST_ONLY_READONLY_LOAD_SAMPLER_TWO_REAL_SAMPLES';pass=($null -eq $failure)
        checks_total=$checks.Count;checks_passed=@($checks | Where-Object pass).Count;checks=@($checks)
        failure=$failure;source=$source;source_sha256=(Get-FileHash -LiteralPath $source -Algorithm SHA256).Hash
        fixture_sha256=(Get-FileHash -LiteralPath $PSCommandPath -Algorithm SHA256).Hash
        only_real_queried_pid=$PID;sample_count=$rows.Count;samples=@($rows)
        limitations='System CPU/RAM and selected-process IO diagnostics. The first rate sample is null; query failures are reported.'
        MATLAB_launch=0;CopterSim_launch=0;COM_open=0;socket_open=0;board_actions=0;unrelated_process_control=0
    }
    $json=$result | ConvertTo-Json -Depth 18
    $stream=[IO.FileStream]::new($OutputPath,[IO.FileMode]::CreateNew,[IO.FileAccess]::Write,[IO.FileShare]::Read)
    try {
        $bytes=[Text.UTF8Encoding]::new($false).GetBytes($json+[Environment]::NewLine)
        $stream.Write($bytes,0,$bytes.Length);$stream.Flush($true)
    } finally { $stream.Dispose() }
}
[pscustomobject]@{pass=$result.pass;checks="$($result.checks_passed)/$($result.checks_total)";cpu_percent=$rows[1].SystemCpuBusyPercent;ram_available_bytes=$rows[1].PhysicalAvailableBytes;sample_ms=@($rows.SampleDurationMs);output=$OutputPath} | ConvertTo-Json -Depth 4
if(-not $result.pass) { exit 2 }
