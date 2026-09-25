param([Parameter(Mandatory=$true)][string]$OutputRoot)
$ErrorActionPreference='Stop'
if(Test-Path -LiteralPath $OutputRoot){throw 'Choose a new output directory.'}
$runtime=(& (Join-Path $PSScriptRoot 'gpenmpc_external_path.ps1') 'native_model_runtime')
$exe=Join-Path $runtime 'CopterSim.exe'
$dll=Join-Path $runtime 'external\model\GPENMPC_M600_Canonical.dll'
$expectedExe='BD3139D6D4BC665C254AD7C3C17618CF91D18138205D671FD88B428C0C017917'
$expectedDll='F906F54D1FDB928FDFDA2E8DB81C3FD92968619E430886F9EFFFAD8891288B27'
if((Get-FileHash -LiteralPath $exe -Algorithm SHA256).Hash -ne $expectedExe){throw 'CopterSim executable mismatch.'}
if((Get-FileHash -LiteralPath $dll -Algorithm SHA256).Hash -ne $expectedDll){throw 'Canonical F906 DLL mismatch.'}
$existing=@(Get-Process -Name CopterSim,CopterSimNoUI -ErrorAction SilentlyContinue)
if($existing.Count -ne 0){throw 'Existing CopterSim found; do not interfere.'}
$null=New-Item -ItemType Directory -Path $OutputRoot
$source=Join-Path $PSScriptRoot 'OwnedCopterInitObserver.cs'
Add-Type -Path $source
$argsFixed='1 1 -1 GPENMPC_M600_Canonical 3 LowGPU 0 0 0 0 1 2'
$observer=$null;$owned=$null;$stdout=$null;$stderr=$null;$copyOut=$null;$copyErr=$null
$failure=$null;$cleanup=[Collections.Generic.List[string]]::new();$ownedId=0;$ownedStart=$null;$killed=$false;$elapsed=0.0;$evidence=$null
try {
  $observer=[GPENMPC.HostDiagnostics.OwnedCopterInitObserver]::Start(20009,1,4096)
  $info=[Diagnostics.ProcessStartInfo]::new($exe,$argsFixed)
  $info.WorkingDirectory=$runtime;$info.UseShellExecute=$false;$info.CreateNoWindow=$true;$info.WindowStyle=[Diagnostics.ProcessWindowStyle]::Hidden
  $info.RedirectStandardOutput=$true;$info.RedirectStandardError=$true
  $info.EnvironmentVariables['QT_FORCE_STDERR_LOGGING']='1'
  $info.EnvironmentVariables['QT_MESSAGE_PATTERN']='[%{time process} %{type} pid=%{pid} tid=%{threadid}] %{message}'
  $info.EnvironmentVariables['OMP_NUM_THREADS']='1';$info.EnvironmentVariables['MKL_NUM_THREADS']='1';$info.EnvironmentVariables['OPENBLAS_NUM_THREADS']='1'
  $stdout=[IO.File]::Open((Join-Path $OutputRoot 'COPTER_STDOUT.raw.log'),[IO.FileMode]::CreateNew,[IO.FileAccess]::Write,[IO.FileShare]::Read)
  $stderr=[IO.File]::Open((Join-Path $OutputRoot 'COPTER_STDERR.raw.log'),[IO.FileMode]::CreateNew,[IO.FileAccess]::Write,[IO.FileShare]::Read)
  $owned=[Diagnostics.Process]::new();$owned.StartInfo=$info
  if(-not $owned.Start()){throw 'Process start returned false.'}
  $ownedId=$owned.Id;$ownedStart=$owned.StartTime.ToUniversalTime().ToString('o')
  $copyOut=$owned.StandardOutput.BaseStream.CopyToAsync($stdout);$copyErr=$owned.StandardError.BaseStream.CopyToAsync($stderr)
  $watch=[Diagnostics.Stopwatch]::StartNew()
  while($watch.Elapsed.TotalSeconds -lt 15.0){if($owned.HasExited){throw 'Owned no-board CopterSim exited before 15 s.'};Start-Sleep -Milliseconds 50}
  $elapsed=$watch.Elapsed.TotalSeconds
} catch {$failure=$_.Exception.ToString()} finally {
  if($null -ne $observer){$observer.Dispose();$evidence=$observer.Snapshot()}
  if($null -ne $owned -and $ownedId -gt 0){
    try {
      if(-not $owned.HasExited){$null=$owned.CloseMainWindow();if(-not $owned.WaitForExit(3000)){$owned.Kill();$killed=$true;if(-not $owned.WaitForExit(5000)){throw 'Owned no-board process did not exit.'}}}
    } catch {$cleanup.Add($_.Exception.ToString())}
  }
  foreach($task in @($copyOut,$copyErr)){if($null -ne $task){try{if(-not $task.Wait(5000)){throw 'Redirect pipe drain timed out.'}}catch{$cleanup.Add($_.Exception.ToString())}}}
  foreach($stream in @($stdout,$stderr)){if($null -ne $stream){try{$stream.Flush($true);$stream.Dispose()}catch{$cleanup.Add($_.Exception.ToString())}}}
}
$released=$ownedId -eq 0 -or $owned.HasExited
$portReleased=$false
try{$p=[Net.Sockets.UdpClient]::new(20009);$p.Dispose();$portReleased=$true}catch{$cleanup.Add('Observer port 20009 did not rebind: '+$_.Exception.Message)}
$logs=@('COPTER_STDOUT.raw.log','COPTER_STDERR.raw.log')|ForEach-Object{$p=Join-Path $OutputRoot $_;if(Test-Path -LiteralPath $p){[pscustomobject]@{path=$p;bytes=(Get-Item -LiteralPath $p).Length;sha256=(Get-FileHash -LiteralPath $p -Algorithm SHA256).Hash}}}
$result=[ordered]@{
  status='HOST_ONLY_OWNED_COPTERSIM_LOG_OBSERVATION_NOT_HIL';observation_completed=($null -eq $failure -and $elapsed -ge 15)
  hypothesis='Can owned Qt stdout/stderr and official receive-only initialization channel be recorded without PX4/COM access?'
  arguments=$argsFixed;runtime=$runtime;exe_sha256=$expectedExe;dll_sha256=$expectedDll;source_sha256=(Get-FileHash -LiteralPath $source -Algorithm SHA256).Hash
  pid=$ownedId;process_start_utc=$ownedStart;observed_duration_s=$elapsed;failure=$failure;cleanup_errors=$cleanup
  owned_process_released=$released;owned_no_board_process_force_kill_used=$killed;observer_port_released=$portReleased
  qt_environment=@{QT_FORCE_STDERR_LOGGING='1';QT_MESSAGE_PATTERN='[%{time process} %{type} pid=%{pid} tid=%{threadid}] %{message}'}
  observer=$evidence;logs=$logs;COM_open=0;board_actions=0;control_requests=0;UDP_send_calls=0;HIL=$false
  limits='This probe records mode-3 vendor initialization. Mode-0 packet behavior and unlogged vendor state require separate observation.'
}
$result|ConvertTo-Json -Depth 12|Set-Content -LiteralPath (Join-Path $OutputRoot 'RESULT.json') -Encoding utf8
[pscustomobject]$result|Select-Object status,observation_completed,pid,observed_duration_s,failure,owned_process_released,observer_port_released,logs|ConvertTo-Json -Depth 5
if($null -ne $owned){$owned.Dispose()}
if($null -ne $failure -or -not $released -or -not $portReleased -or $cleanup.Count -gt 0){exit 2}
