param(
    [ValidateSet('Audit','Natural','Parent','Leaf','Sentinel')][string]$Role='Audit',
    [string]$OutputRoot
)
$ErrorActionPreference='Stop'
$pwsh=(Get-Command pwsh.exe -ErrorAction Stop).Source
if($Role -eq 'Natural') {
    [Console]::Out.WriteLine('OWNED_STDOUT_OK')
    [Console]::Error.WriteLine('OWNED_STDERR_OK')
    exit 7
}
if($Role -eq 'Leaf' -or $Role -eq 'Sentinel') {
    [Console]::Out.WriteLine("OWNED_SLEEP_ROLE=$Role PID=$PID")
    Start-Sleep -Seconds 45
    exit 0
}
if($Role -eq 'Parent') {
    $child=Start-Process -FilePath $pwsh -ArgumentList @('-NoProfile','-File',('"'+$PSCommandPath+'"'),'-Role','Leaf') -PassThru -WindowStyle Hidden
    [Console]::Out.WriteLine("OWNED_DESCENDANT_PID=$($child.Id)")
    # Deliberately exit before the descendant, like a launcher process.
    exit 0
}

if([string]::IsNullOrWhiteSpace($OutputRoot)) { throw 'A fresh OutputRoot is required.' }
if(Test-Path -LiteralPath $OutputRoot) { throw 'Use a new OutputRoot.' }
$null=New-Item -ItemType Directory -Path $OutputRoot
$OutputRoot=(Resolve-Path -LiteralPath $OutputRoot).Path
$source=Join-Path $PSScriptRoot 'OwnedMatlabJob.cs'
Add-Type -Path $source
$checks=[Collections.Generic.List[object]]::new()
$jobs=[Collections.Generic.List[object]]::new()
$failure=$null
$ownedPids=[Collections.Generic.List[int]]::new()
function Check([string]$name,[bool]$value) {
    $checks.Add([pscustomobject]@{name=$name;pass=$value})
    if(-not $value) { throw "HOST_JOB_ASSERTION: $name" }
}
function StartCase([string]$name,[string]$caseRole) {
    $args='-NoProfile -File "'+$PSCommandPath+'" -Role '+$caseRole
    $j=[OwnedMatlabJob]::Start($pwsh,$args,$PSScriptRoot,(Join-Path $OutputRoot ($name+'.stdout.log')),(Join-Path $OutputRoot ($name+'.stderr.log')))
    $jobs.Add($j);$ownedPids.Add($j.ProcessId)
    return $j
}
function WaitDescendant([string]$path) {
    $deadline=[Diagnostics.Stopwatch]::StartNew()
    do {
        # The owned child still holds a writable handle. Readers must share
        # WRITE as well as READ; File.ReadAllText uses an incompatible share.
        $stream=[IO.FileStream]::new($path,[IO.FileMode]::Open,[IO.FileAccess]::Read,[IO.FileShare]::ReadWrite)
        $reader=[IO.StreamReader]::new($stream)
        try { $text=$reader.ReadToEnd() } finally { $reader.Dispose(); $stream.Dispose() }
        if($text -match 'OWNED_DESCENDANT_PID=(\d+)') { return [int]$Matches[1] }
        Start-Sleep -Milliseconds 25
    } while($deadline.Elapsed.TotalSeconds -lt 15)
    throw 'Owned child PID was not recorded.'
}
try {
    $natural=StartCase 'natural' 'Natural'
    Check 'assigned_before_resume' $natural.AssignedBeforeResume
    Check 'natural_tree_exit' ($natural.WaitForTreeExit(15000))
    Check 'natural_active_processes_zero' ($natural.ActiveProcesses -eq 0)
    Check 'natural_root_exit_code_preserved' ($natural.ExitCode -eq 7)
    Check 'stdout_preserved' ([IO.File]::ReadAllText((Join-Path $OutputRoot 'natural.stdout.log')).Contains('OWNED_STDOUT_OK'))
    Check 'stderr_preserved' ([IO.File]::ReadAllText((Join-Path $OutputRoot 'natural.stderr.log')).Contains('OWNED_STDERR_OK'))

    $sentinel=StartCase 'unrelated_owned_sentinel' 'Sentinel'
    $tree=StartCase 'launcher_tree' 'Parent'
    $descendant=WaitDescendant (Join-Path $OutputRoot 'launcher_tree.stdout.log')
    $ownedPids.Add($descendant)
    $rootDeadline=[Diagnostics.Stopwatch]::StartNew();$rootExited=$false
    do {
        try { $rootExited=($tree.ExitCode -eq 0) } catch { Start-Sleep -Milliseconds 25 }
    } while(-not $rootExited -and $rootDeadline.Elapsed.TotalSeconds -lt 10)
    Check 'root_exits_before_descendant' $rootExited
    Check 'root_exit_not_tree_exit' (-not $tree.WaitForTreeExit(100))
    Check 'descendant_accounted_after_root_exit' ($tree.ActiveProcesses -ge 1)
    Check 'job_pid_list_contains_owned_descendant' ($tree.ProcessIds -contains $descendant)
    Check 'job_pid_list_excludes_unrelated_sentinel' ($tree.ProcessIds -notcontains $sentinel.ProcessId)
    Check 'root_exit_code_available_while_child_alive' ($tree.ExitCode -eq 0)
    $tree.Terminate(254)
    Check 'timeout_terminates_entire_owned_tree' ($tree.WaitForTreeExit(5000))
    Check 'terminated_tree_active_processes_zero' ($tree.ActiveProcesses -eq 0)
    Check 'terminated_tree_pid_list_empty' ($tree.ProcessIds.Count -eq 0)
    Check 'descendant_no_longer_running' ($null -eq (Get-Process -Id $descendant -ErrorAction SilentlyContinue))
    Check 'unrelated_job_survives_other_job_termination' ($sentinel.ActiveProcesses -gt 0)

    $disposeTree=StartCase 'dispose_tree' 'Parent'
    $disposeDescendant=WaitDescendant (Join-Path $OutputRoot 'dispose_tree.stdout.log')
    $ownedPids.Add($disposeDescendant)
    $disposeTree.Dispose()
    $clock=[Diagnostics.Stopwatch]::StartNew()
    while($null -ne (Get-Process -Id $disposeDescendant -ErrorAction SilentlyContinue) -and $clock.Elapsed.TotalSeconds -lt 5) {
        Start-Sleep -Milliseconds 25
    }
    Check 'kill_on_job_close_cleans_descendant' ($null -eq (Get-Process -Id $disposeDescendant -ErrorAction SilentlyContinue))
    Check 'unrelated_job_survives_job_close' ($sentinel.ActiveProcesses -gt 0)

    $overwriteRejected=$false
    try {
        $unexpected=[OwnedMatlabJob]::Start($pwsh,'-NoProfile -Command exit 0',$PSScriptRoot,(Join-Path $OutputRoot 'natural.stdout.log'),(Join-Path $OutputRoot 'unused.stderr.log'))
        $unexpected.Dispose()
    } catch { $overwriteRejected=$_.Exception.Message.Contains('fresh stdout/stderr') }
    Check 'existing_logs_rejected_without_overwrite' $overwriteRejected
    Check 'no_extra_log_created_on_rejected_launch' (-not (Test-Path -LiteralPath (Join-Path $OutputRoot 'unused.stderr.log')))
    $sentinel.Terminate(254)
    Check 'sentinel_explicit_owned_cleanup' ($sentinel.WaitForTreeExit(5000))
} catch {
    $failure=$_.Exception.ToString()
} finally {
    foreach($j in $jobs) {
        try { $j.Terminate(254); $null=$j.WaitForTreeExit(5000) } catch { }
        $j.Dispose()
    }
}
$remaining=@($ownedPids | Select-Object -Unique | ForEach-Object { Get-Process -Id $_ -ErrorAction SilentlyContinue } | Select-Object -ExpandProperty Id)
$checks.Add([pscustomobject]@{name='all_fixture_owned_processes_released';pass=($remaining.Count -eq 0)})
$passed=@($checks | Where-Object pass).Count
$result=[ordered]@{
    classification='HOST_WINDOWS_JOB_PROCESS_TREE_FIXTURE'
    pass=($null -eq $failure -and $passed -eq $checks.Count)
    checks_total=$checks.Count;checks_passed=$passed;checks=@($checks)
    failure=$failure;owned_process_ids=@($ownedPids);remaining_owned_process_ids=$remaining
    source_sha256=(Get-FileHash -LiteralPath $source -Algorithm SHA256).Hash
    source_path=$source;fixture_path=$PSCommandPath
    MATLAB_launch=0;CopterSim_launch=0;COM_open=0;network_endpoint_open=0;board_actions=0
    unrelated_existing_processes_terminated=0
}
$json=$result|ConvertTo-Json -Depth 15
[IO.File]::WriteAllText((Join-Path $OutputRoot 'HOST_JOB_RESULT.json'),$json+[Environment]::NewLine,[Text.UTF8Encoding]::new($false))
$json
if(-not $result.pass) { exit 2 }
