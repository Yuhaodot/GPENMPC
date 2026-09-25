[CmdletBinding()]
param(
 [Parameter(Mandatory=$true)][string]$PlanPath,
 [Parameter(Mandatory=$true)][string]$RunRoot,
 [Parameter(Mandatory=$true)][int]$OwnedCopterPid,
 [Parameter(Mandatory=$true)][string]$ExpectedStartUtc,
 [switch]$ExecuteSafetyContinuation
)
# Resume the interrupted readback and release the owned processes.
$ErrorActionPreference='Stop'
$plan=Get-Content -LiteralPath $PlanPath -Raw|ConvertFrom-Json
$planHash=(Get-FileHash -LiteralPath $PlanPath -Algorithm SHA256).Hash
$refs=@{}
foreach($r in $plan.references){
 $item=Get-Item -LiteralPath $r.path
 if($item.Length -ne [long]$r.bytes -or (Get-FileHash -LiteralPath $r.path -Algorithm SHA256).Hash -ne $r.sha256){throw "Changed consumed input: $($r.label)"}
 $refs[[string]$r.label]=$r
}
$previous=Get-Content -LiteralPath (Join-Path $RunRoot 'OUTER_RESULT.json') -Raw|ConvertFrom-Json
$oldRecovery=Get-Content -LiteralPath (Join-Path $RunRoot 'UDP_RECOVERY.json') -Raw|ConvertFrom-Json
if($previous.plan_sha256 -ne $planHash -or $oldRecovery.safe_bridge_shutdown_allowed -or -not $oldRecovery.udp_closed){throw 'Not the failed, released readback continuation.'}
$expectedTicks=[DateTimeOffset]::Parse($ExpectedStartUtc).UtcDateTime.Ticks
function Owner {
 $p=Get-Process -Id $OwnedCopterPid -ErrorAction Stop
 if($p.StartTime.ToUniversalTime().Ticks -ne $expectedTicks -or [IO.Path]::GetFullPath($p.Path) -ne [IO.Path]::GetFullPath($refs.copter_exe.path)){throw 'Owner PID/start/path mismatch.'}
 $module=$p.Modules|Where-Object ModuleName -eq 'GPENMPC_M600_Canonical.dll'|Select-Object -First 1
 if($null -eq $module -or (Get-FileHash -LiteralPath $module.FileName -Algorithm SHA256).Hash -ne $refs.model_dll.sha256){throw 'Loaded model identity mismatch.'}
 return $p
}
$null=Owner
if(-not $ExecuteSafetyContinuation){[ordered]@{status='RECOVERY_BINDINGS_VALIDATED';references=$refs.Count;owner_pid=$OwnedCopterPid}|ConvertTo-Json;exit 0}
$out=Join-Path $RunRoot 'SAFETY_READBACK_CONTINUATION'
if(Test-Path -LiteralPath $out){throw 'Continuation evidence already exists; do not overwrite or automatically repeat.'}
$busy=@(Get-NetUDPEndpoint -ErrorAction Stop|Where-Object {$_.LocalPort -in @(14550,30101,20005,20009)})
if($busy.Count){throw 'Prior controller/observer sockets have not released.'}
New-Item -ItemType Directory -Path $out|Out-Null
Copy-Item -LiteralPath (Join-Path $RunRoot 'SERIAL_PREFLIGHT.json') -Destination (Join-Path $out 'SERIAL_PREFLIGHT.json')
if((Get-FileHash -LiteralPath (Join-Path $out 'SERIAL_PREFLIGHT.json')).Hash -ne (Get-FileHash -LiteralPath (Join-Path $RunRoot 'SERIAL_PREFLIGHT.json')).Hash){throw 'Preflight copy mismatch.'}
$journal=Join-Path $out 'JOURNAL.jsonl'
function Record([string]$kind,$details){[IO.File]::AppendAllText($journal,([ordered]@{utc=[DateTime]::UtcNow.ToString('o');kind=$kind;details=$details}|ConvertTo-Json -Depth 10 -Compress)+[Environment]::NewLine)}
function MatQuote([string]$s){return $s.Replace("'","''")}
Add-Type -Path $refs.owned_job_source.path
$script:stopped=$false;$script:treeReleased=$true;$safe=$false;$failure='';$recovery=$null;$post=$null
function Child([string]$operation,[double]$timeout){
 if($operation -notin @('UDP_RECOVERY','SERIAL_POSTFLIGHT')){throw 'Only safety operations allowed.'}
 if(-not $script:treeReleased){throw 'Previous MATLAB tree still exists.'}
 $remaining=@(Get-NetUDPEndpoint -ErrorAction Stop|Where-Object {$_.LocalPort -in @(14550,30101,20005)})
 if($remaining.Count){throw 'Competing recovery endpoint.'}
 $pidToken=if($script:stopped){0}else{$OwnedCopterPid}
 if($operation -eq 'SERIAL_POSTFLIGHT' -and -not $script:stopped){throw 'Serial owner is still alive.'}
 [IO.File]::WriteAllText((Join-Path $out 'OUTER_STAGE_TOKEN.json'),([ordered]@{operation=$operation;plan_sha256=$planHash;coptersim_owned_pid=$pidToken}|ConvertTo-Json))
 $expression="try; addpath('$(MatQuote $PSScriptRoot)'); r=launch_m600_canonical_hil('$(MatQuote $PlanPath)','$operation','$(MatQuote $out)'); disp(struct('operation','$operation','result_saved',true)); catch e; disp(getReport(e,'extended','hyperlinks','off')); exit(1); end;"
 $job=$null
 try {
  $job=[OwnedMatlabJob]::Start($refs.matlab_exe.path,('-singleCompThread -batch "'+$expression+'"'),(Split-Path -Parent $PSScriptRoot),(Join-Path $out ($operation+'_STDOUT.log')),(Join-Path $out ($operation+'_STDERR.log')))
  $script:treeReleased=$false
  Record 'OWNED_SAFETY_CHILD_STARTED' @{operation=$operation;pid=$job.ProcessId;timeout_s=$timeout;coptersim_outside_job=$true}
  $watch=[Diagnostics.Stopwatch]::StartNew();$done=$false
  while($watch.Elapsed.TotalSeconds -lt $timeout){if($job.WaitForTreeExit(1000)){$done=$true;break}}
  if(-not $done){Record 'RECOVERY_CHILD_TIMEOUT' @{operation=$operation};$job.Terminate([uint32]124);$done=$job.WaitForTreeExit([int]($plan.outer_bounds.process_exit_timeout_s*1000))}
  $script:treeReleased=$done -and $job.ActiveProcesses -eq 0
  if(-not $script:treeReleased){throw 'Safety child tree did not close.'}
  $result=@{operation=$operation;exit_code=$job.ExitCode;active_processes=$job.ActiveProcesses;elapsed_s=$watch.Elapsed.TotalSeconds}
  Record 'OWNED_SAFETY_CHILD_EXITED' $result
  return $result
 } finally {if($null -ne $job){$job.Dispose()}}
}
try {
 Record 'SAME_RUN_RECOVERY_CONTINUATION' @{plan_sha256=$planHash;source_sha256=(Get-FileHash -LiteralPath $PSCommandPath).Hash;owner_pid=$OwnedCopterPid;owner_start_utc=$ExpectedStartUtc}
 $r=Child 'UDP_RECOVERY' $plan.outer_bounds.recovery_child_timeout_s
 $recovery=Get-Content -LiteralPath (Join-Path $out 'UDP_RECOVERY.json') -Raw|ConvertFrom-Json
 if($r.exit_code -ne 0 -or -not $recovery.safe_bridge_shutdown_allowed -or -not $recovery.udp_closed){throw 'Fresh safety readback incomplete; retain the owned sensor/serial process.'}
 $p=Owner
 Stop-Process -Id $p.Id -Force
 if(-not $p.WaitForExit([int]($plan.outer_bounds.process_exit_timeout_s*1000))){throw 'Safe owner shutdown did not finish.'}
 $script:stopped=$true
 Record 'OWNED_COPTER_STOPPED' @{pid=$OwnedCopterPid;fresh_disarmed_landed=$recovery.fresh_disarmed_landed;typed_zero_parameters=$recovery.final_parameters.Count}
 $r=Child 'SERIAL_POSTFLIGHT' $plan.outer_bounds.serial_child_timeout_s
 $post=Get-Content -LiteralPath (Join-Path $out 'SERIAL_POSTFLIGHT.json') -Raw|ConvertFrom-Json
 $remaining=@(Get-NetUDPEndpoint -ErrorAction Stop|Where-Object {$_.LocalPort -in @(14550,30101,20005,20009,18570)})
 $safe=$r.exit_code -eq 0 -and $post.passed -and $post.COM_closed -and $remaining.Count -eq 0
 Record 'FINAL_RELEASE' @{udp=$remaining;owned_matlab_tree_released=$script:treeReleased;coptersim_stopped=$script:stopped;COM_closed=$post.COM_closed}
} catch {$failure=$_.Exception.Message;Record 'SAFETY_CONTINUATION_EXCEPTION' @{error=$failure}}
finally {
 $result=[ordered]@{status=$(if($safe){'SAFETY_RECOVERY_COMPLETE'}else{'SAFETY_RECOVERY_INCOMPLETE'});safe=$safe;failure=$failure;plan_sha256=$planHash;original_run=$RunRoot;coptersim_stopped=$script:stopped;owned_matlab_tree_released=$script:treeReleased;recovery_counts=$(if($recovery){$recovery.counts}else{$null});serial_postflight_passed=$(if($post){$post.passed}else{$false})}
 [IO.File]::WriteAllText((Join-Path $out 'SAFETY_CONTINUATION_RESULT.json'),($result|ConvertTo-Json -Depth 12))
 $result|ConvertTo-Json -Depth 12
}
if(-not $safe){exit 1}
