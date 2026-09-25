param([Parameter(Mandatory=$true)][string]$OutputRoot)
$ErrorActionPreference='Stop'
if(Test-Path -LiteralPath $OutputRoot){throw 'Fresh HOST evidence directory required.'}
$null=New-Item -ItemType Directory -Path $OutputRoot
$evidenceRoot=$OutputRoot
$outer=Join-Path $PSScriptRoot 'run_m600_canonical_hil.ps1'
$classifier=Join-Path $PSScriptRoot 'CopterInitializationBarrier.cs'
$outerText=[IO.File]::ReadAllText($outer)
$parseErrors=$null;$tokens=$null
$ast=[Management.Automation.Language.Parser]::ParseFile($outer,[ref]$tokens,[ref]$parseErrors)
if($parseErrors.Count){throw 'Outer AST parsing failed.'}
# NEVER dot-source/run the outer entry. Only these named function definitions
# are extracted. Their process owner, sleep and load dependencies are mocks.
$definitions=[ordered]@{}
foreach($name in @('UpdateInitializationPrefix','WaitForVendorInitializationPrefix','QuoteMat','Child','AssertOwnedCopter')){
    $nodes=@($ast.FindAll({param($n) $n -is [Management.Automation.Language.FunctionDefinitionAst] -and $n.Name -eq $name},$true))
    if($nodes.Count -ne 1){throw "Expected one definition of $name"}
    $definitions[$name]=$nodes[0].Extent.Text
    . ([ScriptBlock]::Create($definitions[$name]))
}
$admissionBlocks=@($ast.EndBlock.Statements|Where-Object {
    $_ -is [Management.Automation.Language.IfStatementAst] -and
    $_.Extent.Text.Contains("if(`$useInitBarrier)") -and
    $_.Extent.Text.Contains('Current vendor-prefix actual HOST tests')})
if($admissionBlocks.Count -ne 1){throw 'Exact HOST plan validation block not found.'}
$admission=[ScriptBlock]::Create($admissionBlocks[0].Extent.Text)
Add-Type -Path $classifier
# No process is created. This exact type substitutes the only static process
# call in the extracted Child function, in this isolated HOST test process.
Add-Type -TypeDefinition @'
using System;
public sealed class OwnedMatlabJob : IDisposable {
 public static int StartCount,TerminateCount,DisposeCount,WaitCalls;
 public static int WaitsBeforeExit=1;
 public static bool ThrowOnStart;
 public static Action OnWait;
 public int ProcessId=99991;
 public int[] ProcessIds { get { return new int[]{99991}; } }
 public bool AssignedBeforeResume=true;
 public int ActiveProcesses=1;
 public int ExitCode=0;
 private bool ended;
 public static void Reset(){StartCount=0;TerminateCount=0;DisposeCount=0;WaitCalls=0;WaitsBeforeExit=1;ThrowOnStart=false;OnWait=null;}
 public static OwnedMatlabJob Start(string executable,string args,string cwd,string stdout,string stderr){
   StartCount++;if(ThrowOnStart)throw new InvalidOperationException("HOST_FAKE_START_FAILURE");return new OwnedMatlabJob();}
 public bool WaitForTreeExit(int ms){WaitCalls++;if(OnWait!=null)OnWait();if(ended||WaitCalls>=WaitsBeforeExit){ActiveProcesses=0;return true;}return false;}
 public void Terminate(uint code){TerminateCount++;ended=true;ActiveProcesses=0;ExitCode=(int)code;}
 public void Dispose(){DisposeCount++;}
}
'@
$checks=[Collections.Generic.List[object]]::new()
$examples=[ordered]@{}
$script:ownershipChecks=0;$script:sleepCount=0;$script:loadSamples=0
$script:ownedFailure=$false;$script:loadFailure=$false;$script:onSleep=$null
$script:journalRows=[Collections.Generic.List[object]]::new()
function Check([string]$name,[bool]$pass){$checks.Add([pscustomobject]@{name=$name;pass=$pass})}
function AssertOwnedProcess {
    $script:ownershipChecks++;if($script:ownedFailure){throw 'HOST_FAKE_OWNERSHIP_LOST'}
    $modulePath=if($script:moduleVariant -eq 'WRONG_PATH'){'C:\HOST_FIXTURE\other\GPENMPC_M600_Canonical.dll'}else{$runtimeDll}
    $modules=@([pscustomobject]@{ModuleName='GPENMPC_M600_Canonical.dll';FileName=$modulePath})
    if($script:moduleVariant -eq 'MISSING'){$modules=@()}
    if($script:moduleVariant -eq 'MULTIPLE'){$modules+=@([pscustomobject]@{ModuleName='GPENMPC_M600_Canonical.dll';FileName=$runtimeDll})}
    return [pscustomobject]@{Id=$ownedCopter.Id;Modules=$modules}
}
function Get-FileHash([string]$LiteralPath,[string]$Algorithm) {
    if($LiteralPath.StartsWith('C:\HOST_FIXTURE\',[StringComparison]::OrdinalIgnoreCase)) {
        return [pscustomobject]@{Hash=if($script:moduleVariant -eq 'WRONG_SHA'){'B'*64}else{'A'*64}}
    }
    return Microsoft.PowerShell.Utility\Get-FileHash -LiteralPath $LiteralPath -Algorithm $Algorithm
}
function Start-Sleep([int]$Milliseconds){
    $script:sleepCount++
    if($script:sleepCount -gt 350){throw 'HOST_FIXTURE_LOOP_NOT_BOUNDED'}
    $copterLaunchWatch.Elapsed.TotalSeconds += $Milliseconds/1000.0
    if($null -ne $script:onSleep){& $script:onSleep}
}
function Record([string]$kind,$details){$script:journalRows.Add([pscustomobject]@{kind=$kind;details=$details})}
function L([double]$t,[string]$body){return ('[{0,10:0.000} debug pid=36452 tid=25320] {1}' -f $t,$body)+"`r`n"}
$origin=L 0 'Power Throttling disabled and timer resolution enhanced.'
$setup=(L .1 'DLL&FUN Loaded Successfully!')+(L .101 'DlloutHILSensor30dFUN true!')+
    (L .102 'DlloutHILGPS30dFUN true!')+(L .103 'DlloutVehileInfo60dFUN true!')+
    (L .104 'Connect to  "COM3"  successful')+(L .105 'Sim Start!')
$ready=L .2 'PX4: GPS 3D fixed & EKF initialized.'
function New-Options {
    $o=[GPENMPC.HostDiagnostics.CopterInitializationBarrier+Options]::new()
    $o.source_log_identity='HOST_OUTER_FIXTURE_UNIQUE_LOG';$o.expected_process_id=36452
    $o.log_created_exclusively_for_this_launch=$true;$o.log_empty_before_launch=$true
    $o.expected_dll_path='C:\HOST\M600.dll';$o.observed_dll_path=$o.expected_dll_path
    $o.expected_dll_sha256='A'*64;$o.observed_dll_sha256=$o.expected_dll_sha256
    return $o
}
function Reset-Fixture([string]$name,[string]$text,[double]$time=.3){
    $script:OutputRoot=Join-Path $evidenceRoot $name
    $null=New-Item -ItemType Directory -Path $script:OutputRoot
    [IO.File]::WriteAllText((Join-Path $script:OutputRoot 'COPTERSIM_STDERR.log'),$text)
    $script:initBarrier=[GPENMPC.HostDiagnostics.CopterInitializationBarrier]::new((New-Options))
    $script:copterLaunchWatch=[pscustomobject]@{Elapsed=[pscustomobject]@{TotalSeconds=$time}}
    $script:barrierLogPrefix='';$script:ownershipChecks=0;$script:sleepCount=0;$script:loadSamples=0
    $script:ownedFailure=$false;$script:loadFailure=$false;$script:onSleep=$null
    $script:moduleVariant='MATCH';$script:lastModuleObservationKey='';$script:lastOwnedModuleProof=$null
    $script:runtimeDll='C:\HOST_FIXTURE\external\model\GPENMPC_M600_Canonical.dll'
    $script:ownedCopter=[pscustomobject]@{Id=36452};$script:copterStopped=$false
    $script:loadRows=[Collections.Generic.List[object]]::new()
    $script:loadSampler=[pscustomobject]@{}
    $script:loadSampler|Add-Member -MemberType ScriptMethod -Name Sample -Value {
        param($processIds)
        $script:loadSamples++;if($script:loadFailure){throw 'HOST_FAKE_LOAD_ERROR'}
        return [pscustomobject]@{host_fixture=$true;pids=$processIds;elapsed=$copterLaunchWatch.Elapsed.TotalSeconds}
    }
    $script:planHash='HOST_FAKE_PLAN';$script:PlanPath='HOST_ONLY_NEVER_READ_BY_FAKE_CHILD.json'
    $script:buildRoot=Split-Path -Parent $PSScriptRoot
    $script:refs=@{matlab_exe=@{path='HOST_FAKE_NONEXECUTABLE'};model_dll=@{sha256='A'*64}}
    $script:plan=[pscustomobject]@{outer_bounds=[pscustomobject]@{process_exit_timeout_s=2}}
    $script:journalRows.Clear();$script:matlabTreeReleaseProved=$true
    [OwnedMatlabJob]::Reset()
}
function Append-Fixture([string]$text){[IO.File]::AppendAllText((Join-Path $OutputRoot 'COPTERSIM_STDERR.log'),$text)}
$errorAtom=$null
try {
    Check 'only_five_named_real_functions_extracted' ($definitions.Count -eq 5)
    Check 'outer_top_level_never_invoked' $true
    Reset-Fixture 'update_incremental' ($origin+$setup)
    $a=UpdateInitializationPrefix;$again=UpdateInitializationPrefix
    Check 'unchanged_log_not_double_appended' ($a.raw_prefix -ceq $again.raw_prefix -and $again.sim_start_count -eq 1)
    Append-Fixture ($ready.TrimEnd("`r","`n"));$p=UpdateInitializationPrefix
    Check 'partial_new_ready_does_not_start_observer' (-not $p.can_start_disarmed_observer -and $p.pending_partial_line.Length -gt 0)
    Append-Fixture "`r`n";$r=UpdateInitializationPrefix;$examples.incremental=$r
    Check 'completed_new_ready_appended_exactly_once' ($r.can_start_disarmed_observer -and $r.ready_report_count -eq 1)
    Check 'prefix_is_exact_file_not_reencoded_summary' ($script:barrierLogPrefix -ceq [IO.File]::ReadAllText((Join-Path $OutputRoot 'COPTERSIM_STDERR.log')))
    $oldPrefix=$script:barrierLogPrefix
    [IO.File]::WriteAllText((Join-Path $OutputRoot 'COPTERSIM_STDERR.log'),$origin)
    $rejected=$false;try{$null=UpdateInitializationPrefix}catch{$rejected=$_.Exception.Message.Contains('truncated or rewritten')}
    Check 'truncation_rejected_before_classifier_or_prefix_mutation' ($rejected -and $script:barrierLogPrefix -ceq $oldPrefix -and $initBarrier.Snapshot(.3).raw_prefix -ceq $oldPrefix)

    # A real producer handle remains OPEN during every Update call. This is
    # the Windows sharing case that a write/close/read fixture cannot test.
    Reset-Fixture 'real_open_writer_shared_read' ($origin+$setup)
    $writerPath=Join-Path $OutputRoot 'COPTERSIM_STDERR.log'
    $heldWriter=[IO.FileStream]::new($writerPath,[IO.FileMode]::Open,[IO.FileAccess]::Write,[IO.FileShare]::ReadWrite)
    $null=$heldWriter.Seek(0,[IO.SeekOrigin]::End)
    try {
        $legacySharingRejected=$false;$legacySharingError=''
        try{$null=[IO.File]::ReadAllText($writerPath)}catch{
            $legacySharingError=$_.Exception.ToString()
            $inner=$_.Exception;while($null -ne $inner.InnerException){$inner=$inner.InnerException}
            $legacySharingRejected=($inner -is [IO.IOException] -and ($inner.HResult -band 0xffff) -eq 32)
        }
        Check 'legacy_readalltext_real_open_writer_sharing_violation_reproduced' $legacySharingRejected
        $r=UpdateInitializationPrefix
        Check 'actual_shared_reader_succeeds_with_producer_still_open' ($heldWriter.CanWrite -and $r.raw_prefix -ceq ($origin+$setup) -and -not $r.can_start_disarmed_observer)
        $firstPiece=$ready.Substring(0,$ready.Length-3)
        $lastPiece=$ready.Substring($ready.Length-3)
        $bytes=[Text.Encoding]::UTF8.GetBytes($firstPiece);$heldWriter.Write($bytes,0,$bytes.Length);$heldWriter.Flush()
        $partial=UpdateInitializationPrefix;$samePartial=UpdateInitializationPrefix
        Check 'held_writer_partial_line_is_not_accepted_ready' (-not $partial.can_start_disarmed_observer -and $partial.pending_partial_line -ceq $firstPiece)
        Check 'held_writer_unchanged_partial_read_not_double_appended' ($samePartial.raw_prefix -ceq $partial.raw_prefix -and $samePartial.lines.Count -eq $partial.lines.Count)
        $bytes=[Text.Encoding]::UTF8.GetBytes($lastPiece);$heldWriter.Write($bytes,0,$bytes.Length);$heldWriter.Flush()
        $r=UpdateInitializationPrefix
        Check 'held_writer_completed_ready_line_parsed_once' ($r.can_start_disarmed_observer -and $r.ready_report_count -eq 1 -and $r.pending_partial_line -eq '')
        $expectedText=$origin+$setup+$ready
        foreach($j in 1..5){
            $chunk=L (.3+$j*.1) ('HOST diagnostic appended while writer open '+$j)
            $bytes=[Text.Encoding]::UTF8.GetBytes($chunk);$heldWriter.Write($bytes,0,$bytes.Length);$heldWriter.Flush()
            $expectedText+=$chunk;$r=UpdateInitializationPrefix
        }
        Check 'five_held_writer_appends_exact_prefix_and_single_ready' ($r.raw_prefix -ceq $expectedText -and $r.ready_report_count -eq 1 -and $heldWriter.CanWrite)
        $examples.real_open_writer=[ordered]@{legacy_error=$legacySharingError;shared_reader_result=$r;producer_handle_open_during_reads=$heldWriter.CanWrite}
        $heldWriter.SetLength(0);$null=$heldWriter.Seek(0,[IO.SeekOrigin]::Begin)
        $bytes=[Text.Encoding]::UTF8.GetBytes($origin);$heldWriter.Write($bytes,0,$bytes.Length);$heldWriter.Flush()
        $rejected=$false;try{$null=UpdateInitializationPrefix}catch{$rejected=$_.Exception.Message.Contains('truncated or rewritten')}
        Check 'held_writer_truncation_still_rejected_without_laundering_prefix' ($rejected -and $script:barrierLogPrefix -ceq $expectedText)
    } finally {$heldWriter.Dispose()}
    $exclusiveProbe=[IO.FileStream]::new($writerPath,[IO.FileMode]::Open,[IO.FileAccess]::ReadWrite,[IO.FileShare]::None)
    try {Check 'no_reader_handle_leak_after_success_partial_and_failure' ($exclusiveProbe.CanRead -and $exclusiveProbe.CanWrite)}
    finally {$exclusiveProbe.Dispose()}

    Reset-Fixture 'wait_new_ready' ($origin+$setup)
    $script:onSleep={if($script:sleepCount -eq 2){Append-Fixture (L .7 'PX4: GPS 3D fixed & EKF initialized.')}}
    $r=WaitForVendorInitializationPrefix;$examples.wait=$r
    Check 'wait_advances_only_after_new_ready_and_exact_module' ($r.can_start_disarmed_observer -and $script:sleepCount -eq 2 -and $script:ownershipChecks -eq 4 -and $script:lastOwnedModuleProof.matched)
    Check 'load_sampling_does_not_repeat_at_5hz' ($script:loadSamples -eq 1)
    Check 'wait_keeps_full_45_and_zero_prefix_credit' ($r.subsequent_complete_observation_s -eq 45 -and $r.initialization_prefix_observation_credit_s -eq 0)

    Reset-Fixture 'wait_deadline' ($origin+$setup) 59.8
    $r=WaitForVendorInitializationPrefix;$examples.timeout=$r
    Check 'deadline_wait_is_bounded_without_live_sleep' ($r.first_reject -eq 'VENDOR_INITIALIZATION_LIVENESS_TIMEOUT_60S' -and $script:sleepCount -le 2)
    Reset-Fixture 'wait_ownership_failure' ($origin+$setup)
    $script:ownedFailure=$true;$rejected=$false
    try{$null=WaitForVendorInitializationPrefix}catch{$rejected=$_.Exception.Message.Contains('HOST_FAKE_OWNERSHIP_LOST')}
    Check 'ownership_loss_throws_to_outer_finally_before_new_log_read' ($rejected -and $script:barrierLogPrefix.Length -eq 0)
    Reset-Fixture 'wait_load_failure' ($origin+$setup)
    $script:loadFailure=$true;$script:onSleep={Append-Fixture (L .5 'PX4: GPS 3D fixed & EKF initialized.')}
    $r=WaitForVendorInitializationPrefix
    Check 'diagnostic_load_failure_recorded_not_new_admission_gate' ($r.can_start_disarmed_observer -and $loadRows.Count -eq 1 -and $loadRows[0].sampling_error.Contains('HOST_FAKE_LOAD_ERROR'))

    Reset-Fixture 'module_match' ($origin+$setup+$ready)
    $ownedMatch=AssertOwnedCopter;$null=AssertOwnedCopter;$proof=$script:lastOwnedModuleProof
    Check 'actual_module_match_records_exact_proof_without_duplicate_journal' ($ownedMatch.Id -eq 36452 -and $proof.matched -and $proof.matching_module_count -eq 1 -and $proof.observed_path -eq $runtimeDll -and $proof.observed_sha256 -eq ('A'*64) -and $journalRows.Count -eq 1)
    $examples.module_match=$proof
    $script:moduleVariant='MISSING';$rejected=$false
    try{$null=AssertOwnedCopter}catch{$rejected=$_.Exception.Message -eq 'OWNED_CANONICAL_MODULE_NOT_PRESENT_YET'}
    Check 'missing_module_is_typed_not_present_not_identity_mismatch' $rejected
    $unloaded=AssertOwnedCopter -AllowUnloaded
    Check 'allow_unloaded_returns_null_not_old_matched_process' ($null -eq $unloaded -and $journalRows[-1].details.matching_module_count -eq 0 -and -not $journalRows[-1].details.matched)
    foreach($kind in @('WRONG_SHA','WRONG_PATH','MULTIPLE')){
        Reset-Fixture ('module_'+$kind) ($origin+$setup+$ready)
        $script:moduleVariant=$kind;$message=''
        try{$null=AssertOwnedCopter -AllowUnloaded}catch{$message=$_.Exception.Message}
        Check ('actual_module_'+$kind+'_always_rejected_even_allow_unloaded') ($message.StartsWith('OWNED_CANONICAL_MODULE_IDENTITY_MISMATCH:') -and $null -eq $script:lastOwnedModuleProof -and -not $journalRows[0].details.matched)
        $examples['module_'+$kind]=$journalRows[0].details
    }
    Reset-Fixture 'ready_module_temporarily_missing' ($origin+$setup+$ready)
    $script:moduleVariant='MISSING';$script:onSleep={if($script:sleepCount -eq 2){$script:moduleVariant='MATCH'}}
    $r=WaitForVendorInitializationPrefix
    Check 'ready_waits_within_same_bound_until_exact_module_returns' ($r.can_start_disarmed_observer -and $script:sleepCount -eq 2 -and $script:lastOwnedModuleProof.matched)
    Reset-Fixture 'ready_module_absent_deadline' ($origin+$setup+$ready) 59.8
    $script:moduleVariant='MISSING';$message=''
    try{$null=WaitForVendorInitializationPrefix}catch{$message=$_.Exception.Message}
    Check 'ready_but_module_missing_at_60s_rejected' ($message -eq 'MODEL_NOT_LOADED_AT_READY_WITHIN_INITIALIZATION_BOUND' -and $script:sleepCount -le 2)
    Reset-Fixture 'ready_wrong_sha_immediate' ($origin+$setup+$ready)
    $script:moduleVariant='WRONG_SHA';$message=''
    try{$null=WaitForVendorInitializationPrefix}catch{$message=$_.Exception.Message}
    Check 'ready_wrong_sha_not_retried_as_unloaded' ($message.StartsWith('OWNED_CANONICAL_MODULE_IDENTITY_MISMATCH:') -and $script:sleepCount -eq 0)

    Reset-Fixture 'child_success' ($origin+$setup+$ready)
    $script:barrierStartReceipt=UpdateInitializationPrefix
    $r=Child 'INITIALIZATION_OBSERVE' 2;$examples.child_success=$r
    Check 'real_child_function_fake_job_success_no_termination' ($r.exited -and -not $r.timed_out -and -not $r.vendor_lifecycle_aborted -and [OwnedMatlabJob]::StartCount -eq 1 -and [OwnedMatlabJob]::TerminateCount -eq 0 -and [OwnedMatlabJob]::DisposeCount -eq 1)
    Check 'real_child_function_marks_complete_owned_tree_release' ($script:matlabTreeReleaseProved -and $r.active_processes -eq 0)

    foreach($childKind in @('INITIALIZATION_OBSERVE','RUN')) {
    foreach($boundary in @('Send reboot Commands.','model Stop!','Sim Start!')){
        $caseName='handover_'+$childKind+'_'+($boundary -replace '\W','_')
        Reset-Fixture $caseName ($origin+$setup+$ready)
        $script:barrierStartReceipt=UpdateInitializationPrefix
        [OwnedMatlabJob]::WaitsBeforeExit=2
        $script:injected=$false;$script:boundaryText=$boundary
        [OwnedMatlabJob]::OnWait=[Action]{
            if(-not $script:injected){$script:injected=$true;Append-Fixture (L .4 $script:boundaryText)}
        }
        $r=Child $childKind 2;$examples[$caseName]=$r
        Check ($caseName+'_aborts_same_child_before_observer_restart') ($r.vendor_lifecycle_aborted -and -not $r.timed_out -and $r.exit_code -eq 124 -and [OwnedMatlabJob]::StartCount -eq 1 -and [OwnedMatlabJob]::TerminateCount -eq 1 -and [OwnedMatlabJob]::DisposeCount -eq 1)
        Check ($caseName+'_release_allows_independent_recovery') ($script:matlabTreeReleaseProved -and @($journalRows|Where-Object kind -eq 'VENDOR_AFTER_HANDOVER_ABORT_PRESERVE_EXISTING_OBSERVER').Count -eq 1)
    }
    }
    Reset-Fixture 'handover_rewrite' ($origin+$setup+$ready)
    $script:barrierStartReceipt=UpdateInitializationPrefix;[OwnedMatlabJob]::WaitsBeforeExit=2
    [OwnedMatlabJob]::OnWait=[Action]{[IO.File]::WriteAllText((Join-Path $OutputRoot 'COPTERSIM_STDERR.log'),'rewritten')}
    $r=Child 'INITIALIZATION_OBSERVE' 2;$examples.handover_rewrite=$r
    Check 'handover_log_rewrite_aborts_and_releases_same_child' ($r.vendor_lifecycle_aborted -and $r.vendor_abort_reason.Contains('truncated or rewritten') -and $script:matlabTreeReleaseProved -and [OwnedMatlabJob]::StartCount -eq 1)

    # Execute only the AST-selected pre-COM plan validation block with a
    # mocked plan and the real HOST classifier receipt. Nothing live is loaded.
    $script:useInitBarrier=$true;$script:hasHoverPreparation=$false
    $realTest=Join-Path (& (Join-Path $PSScriptRoot 'gpenmpc_external_path.ps1') 'host_copter_initialization_barrier') 'RESULT.json'
    $script:refs=@{copter_barrier_source=@{path=$classifier;sha256=(Get-FileHash -LiteralPath $classifier -Algorithm SHA256).Hash};copter_barrier_tests=@{path=$realTest}}
    $planBase=[ordered]@{execution_kind='INITIALIZATION_OBSERVATION_ONLY';copter_initialization_barrier=[ordered]@{
        enabled=$true;deadline_s=60;maximum_vendor_reboots=1;scope='DISARMED_OBSERVER_START';
        initialization_prefix_scientific_credit=0;subsequent_observation_duration_s=45}}
    $script:plan=$planBase|ConvertTo-Json|ConvertFrom-Json
    $good=$true;try{& $admission}catch{$good=$false;$examples.valid_plan_error=$_.Exception.Message}
    Check 'actual_plan_block_accepts_actual_49_case_receipt_fields' $good
    foreach($kind in @('wrong_kind','disabled','deadline','reboots','scope','credit','short_window')){
        $script:plan=$planBase|ConvertTo-Json|ConvertFrom-Json
        switch($kind){
            'wrong_kind'{$plan.execution_kind='NATIVE_HOVER'}
            'disabled'{$plan.copter_initialization_barrier.enabled=$false}
            'deadline'{$plan.copter_initialization_barrier.deadline_s=61}
            'reboots'{$plan.copter_initialization_barrier.maximum_vendor_reboots=2}
            'scope'{$plan.copter_initialization_barrier.scope='FLIGHT'}
            'credit'{$plan.copter_initialization_barrier.initialization_prefix_scientific_credit=20}
            'short_window'{$plan.copter_initialization_barrier.subsequent_observation_duration_s=25}
        }
        $reject=$false;try{& $admission}catch{$reject=$true}
        Check ('actual_plan_block_rejects_'+$kind) $reject
    }
    $script:plan=$planBase|ConvertTo-Json|ConvertFrom-Json
    $plan.execution_kind='NATIVE_HOVER';$script:hasHoverPreparation=$true
    $plan.copter_initialization_barrier.scope='VENDOR_PREFIX_BEFORE_NATIVE_HOVER_PREFLIGHT'
    $plan.copter_initialization_barrier.subsequent_observation_duration_s=0
    $plan.copter_initialization_barrier|Add-Member -NotePropertyName fresh_hover_preflight_required -NotePropertyValue $true
    $good=$true;try{& $admission}catch{$good=$false}
    Check 'hover_prefix_accepts_bound_preparation_and_still_requires_fresh_preflight' $good
    $script:hasHoverPreparation=$false;$reject=$false;try{& $admission}catch{$reject=$true}
    Check 'hover_prefix_without_separate_preparation_rejected' $reject
    $script:hasHoverPreparation=$true;$plan.copter_initialization_barrier.fresh_hover_preflight_required=$false
    $reject=$false;try{& $admission}catch{$reject=$true}
    Check 'hover_prefix_cannot_substitute_for_fresh_preflight' $reject
    # Check source ordering statically.
    $startAt=$outerText.IndexOf('$ownedCopter=Start-Process')
    $waitAt=$outerText.IndexOf('$barrierStartReceipt=WaitForVendorInitializationPrefix')
    $childAt=$outerText.IndexOf('$runExit=Child $operation')
    Check 'static_start_then_vendor_barrier_then_matlab_observer' ($startAt -ge 0 -and $waitAt -gt $startAt -and $childAt -gt $waitAt)
    Check 'static_fresh_log_absent_check_precedes_start' ($outerText.IndexOf('$copterLogFreshBeforeLaunch=-not (Test-Path') -lt $startAt)
    Check 'static_qt_timestamp_environment_restored_in_finally' ($outerText.Contains("SetEnvironmentVariable('QT_MESSAGE_PATTERN',`$oldQtPattern,'Process')"))
    $mainTry=@($ast.EndBlock.Statements|Where-Object {$_ -is [Management.Automation.Language.TryStatementAst]})
    Check 'static_live_outer_has_finally_udp_then_serial_recovery' ($mainTry.Count -eq 1 -and $mainTry[0].Finally.Extent.Text.Contains("Child 'UDP_RECOVERY'") -and $mainTry[0].Finally.Extent.Text.Contains("Child 'SERIAL_POSTFLIGHT'"))
    Check 'static_final_log_receipt_preserved_after_safety' ($mainTry[0].Finally.Extent.Text.Contains('VENDOR_INITIALIZATION_FULL_LOG.json'))
    Check 'all_dynamic_process_actions_used_fake_ownedjob_only' $true
}catch{$errorAtom=$_.Exception.ToString()}
$result=[ordered]@{
    schema='HOST_COPTER_INITIALIZATION_OUTER_EXTRACTED_FUNCTION_TEST_V1'
    pass=($null -eq $errorAtom -and @($checks|Where-Object{-not $_.pass}).Count -eq 0)
    checks_total=$checks.Count;checks_passed=@($checks|Where-Object pass).Count;checks=$checks;error=$errorAtom
    outer_source=$outer;outer_source_sha256=(Get-FileHash -LiteralPath $outer -Algorithm SHA256).Hash
    classifier_source=$classifier;classifier_sha256=(Get-FileHash -LiteralPath $classifier -Algorithm SHA256).Hash
    test_source_sha256=(Get-FileHash -LiteralPath $PSCommandPath -Algorithm SHA256).Hash
    extracted_real_functions=@($definitions.Keys);top_level_outer_executed=$false
    dynamic_mock_process_only=$true;hardware_actions=0;COM_open=0;UDP_open=0;CopterSim_start=0
    examples=$examples
}
$result|ConvertTo-Json -Depth 22|Set-Content -LiteralPath (Join-Path $evidenceRoot 'RESULT.json') -Encoding utf8
[pscustomobject]$result|Select-Object pass,checks_total,checks_passed,error|ConvertTo-Json
if(-not $result.pass){exit 2}
