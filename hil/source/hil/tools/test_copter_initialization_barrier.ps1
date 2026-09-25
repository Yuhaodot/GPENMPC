param([Parameter(Mandatory=$true)][string]$OutputRoot)
$ErrorActionPreference='Stop'
if(Test-Path -LiteralPath $OutputRoot){throw 'Use a new OutputRoot.'}
$null=New-Item -ItemType Directory -Path $OutputRoot
$source=Join-Path $PSScriptRoot 'CopterInitializationBarrier.cs'
Add-Type -Path $source
$checks=[Collections.Generic.List[object]]::new()
$examples=[ordered]@{}
function Check([string]$name,[bool]$pass){$checks.Add([pscustomobject]@{name=$name;pass=$pass})}
function New-Options {
    $o=[GPENMPC.HostDiagnostics.CopterInitializationBarrier+Options]::new()
    $o.source_log_identity='HOST_FIXTURE_EXCLUSIVE_STDERR'
    $o.expected_process_id=36452
    $o.log_created_exclusively_for_this_launch=$true;$o.log_empty_before_launch=$true
    $o.expected_dll_path='C:\HOST_FIXTURE\audited_sensor_only.dll'
    $o.observed_dll_path=$o.expected_dll_path
    $o.expected_dll_sha256='A'*64;$o.observed_dll_sha256=$o.expected_dll_sha256
    return $o
}
function New-Barrier {return [GPENMPC.HostDiagnostics.CopterInitializationBarrier]::new((New-Options))}
function L([double]$t,[string]$body,[int]$taskPid=36452,[string]$level='debug'){
    return ('[{0,10:0.000} {1} pid={2} tid=25320] {3}' -f $t,$level,$taskPid,$body)+"`r`n"
}
function Setup([double]$t=0.1){
    return ((L $t 'DLL&FUN Loaded Successfully!')+
        (L ($t+.001) 'DlloutHILSensor30dFUN true!')+
        (L ($t+.002) 'DlloutHILGPS30dFUN true!')+
        (L ($t+.003) 'DlloutVehileInfo60dFUN true!')+
        (L ($t+.004) 'SerialPort connection is successful')+
        (L ($t+.005) 'Connect to  "COM3"  successful')+
        (L ($t+.006) 'Sim Start!'))
}
$origin=L 0 'Power Throttling disabled and timer resolution enhanced.'
$ready=L 2 'PX4: GPS 3D fixed & EKF initialized.'
$normal=$origin+(Setup)+$ready
$errorAtom=$null
try {
    $b=New-Barrier;$b.Append($normal,2);$e=$b.Snapshot(2);$examples.normal=$e
    Check 'complete_fresh_prefix_ready_for_disarmed_observer' $e.can_start_disarmed_observer
    Check 'not_flight_or_health_admission' (-not $e.flight_admission -and $e.requires_independent_health)
    Check 'all_original_prefix_characters_retained' ($e.raw_prefix -ceq $normal)
    Check 'no_init_credit_full_45s_observation_retained' ($e.initialization_prefix_observation_credit_s -eq 0 -and $e.subsequent_complete_observation_s -eq 45)
    Check 'fixed_60s_is_engineering_liveness_not_sensor_gate' ($e.initialization_liveness_bound_s -eq 60 -and $e.liveness_bound_provenance.StartsWith('HOST_SESSION_ENGINEERING'))
    Check 'requires_new_observer_never_resets_existing_adapter' ($e.requires_new_disarmed_observer -and $e.must_not_reset_existing_adapter)

    $stop=L 20 'model Stop!';$reboot=L 20 'Send reboot Commands.'
    $b.Append($stop,20);$stopped=$b.Snapshot(20);$examples.old_ready_stopped=$stopped
    Check 'model_stop_invalidates_old_ready' (-not $stopped.can_start_disarmed_observer -and $stopped.invalidated_ready_reports -eq 1)
    $b.Append($reboot,20);$waiting=$b.Snapshot(20)
    Check 'first_vendor_reboot_allowed_only_as_waiting_initialization' ($waiting.vendor_reboot_count -eq 1 -and $waiting.first_reject -eq '' -and -not $waiting.can_start_disarmed_observer)
    $b.Append((Setup 35),35.1);$newStart=$b.Snapshot(35.1)
    Check 'new_sim_start_does_not_reuse_old_ekf_ready' (-not $newStart.can_start_disarmed_observer -and $newStart.last_ready_line -eq 0)
    $b.Append((L 39.437 'PX4: GPS 3D fixed & EKF initialized.'),39.5);$newReady=$b.Snapshot(39.5);$examples.new_ready=$newReady
    Check 'new_ready_after_latest_boundary_and_start_admits' ($newReady.can_start_disarmed_observer -and $newReady.last_ready_line -gt $newReady.last_sim_start_line -and $newReady.last_sim_start_line -gt $newReady.last_boundary_line)
    $b.Append((L 41 'Send reboot Commands.'),41);$second=$b.Snapshot(41);$examples.second_reboot=$second
    Check 'second_vendor_reboot_permanently_rejected' ($second.first_reject -eq 'SECOND_VENDOR_INITIALIZATION_REBOOT' -and -not $second.can_start_disarmed_observer -and $second.vendor_reboot_count -eq 2)
    $b.Append(((Setup 42)+(L 44 'PX4: GPS 3D fixed & EKF initialized.')),44);$stillRejected=$b.Snapshot(44)
    Check 'later_ready_cannot_clear_first_reject' ($stillRejected.first_reject -eq $second.first_reject -and $stillRejected.first_reject_line -eq $second.first_reject_line -and -not $stillRejected.can_start_disarmed_observer)
    Check 'rejected_tail_still_retains_every_raw_line' ($stillRejected.raw_prefix.EndsWith((L 44 'PX4: GPS 3D fixed & EKF initialized.')))

    $b=New-Barrier;$b.Append(($origin+(Setup)),1)
    $partial=(L 2 'PX4: GPS 3D fixed & EKF initialized.').TrimEnd("`r","`n")
    $b.Append($partial,2);$pe=$b.Snapshot(2);$examples.partial=$pe
    Check 'partial_ready_line_not_admitted' (-not $pe.can_start_disarmed_observer -and $pe.pending_partial_line -ceq $partial)
    $b.Append("`r`n",2.1);$pe=$b.Snapshot(2.1)
    Check 'completed_partial_line_parsed_once' ($pe.can_start_disarmed_observer -and $pe.ready_report_count -eq 1 -and $pe.pending_partial_line -eq '')
    $b.Append((L 3 'model Stop!').TrimEnd("`r","`n"),3);$partialStop=$b.Snapshot(3)
    Check 'partial_post_ready_boundary_suspends_admission' (-not $partialStop.can_start_disarmed_observer -and $partialStop.first_reject -eq '')
    $b.Append("`r`n",3.1);$completeStop=$b.Snapshot(3.1)
    Check 'completed_post_ready_boundary_invalidates_ready' (-not $completeStop.can_start_disarmed_observer -and $completeStop.model_stop_count -eq 1 -and $completeStop.last_ready_line -eq 0)
    $split=New-Barrier
    for($i=0;$i -lt $normal.Length;$i++){ $split.Append($normal.Substring($i,1),2) }
    $splitEvidence=$split.Snapshot(2)
    Check 'characterwise_chunks_same_result_and_raw' ($splitEvidence.can_start_disarmed_observer -and $splitEvidence.raw_prefix -ceq $normal -and $splitEvidence.lines.Count -eq $e.lines.Count)

    foreach($fault in @(
        @('explicit_crash','PX4: hard fault detected','EXPLICIT_FATAL_OR_CRASH'),
        @('explicit_fatal','Fatal: model plugin access violation','EXPLICIT_FATAL_OR_CRASH'),
        @('com_open_error','SerialPort connection error: access denied','EXPLICIT_SERIAL_OR_COM_ERROR'),
        @('com_write_error','COM3 write failed Win1460','EXPLICIT_SERIAL_OR_COM_ERROR'),
        @('unknown_ready_state','PX4: GPS 3D fixed & EKF unknown.','UNKNOWN_VENDOR_LIFECYCLE_STATE'),
        @('unknown_reboot_state','Send reboot ???','UNKNOWN_VENDOR_LIFECYCLE_STATE'))){
        $b=New-Barrier;$b.Append(($normal+(L 3 $fault[1])),3);$fe=$b.Snapshot(3)
        Check ($fault[0]+'_fail_closed') ($fe.first_reject -eq $fault[2] -and -not $fe.can_start_disarmed_observer)
        $examples[$fault[0]]=$fe
    }
    $b=New-Barrier;$b.Append(($normal+(L 3 '# 1 : Preflight Fail: ekf2 missing data')+(L 3.1 '# 1 : MAG #0 failed: TIMEOUT!')),3.2);$he=$b.Snapshot(3.2);$examples.health_notes=$he
    Check 'health_notes_retained_not_guessed_as_vendor_latch_failure' ($he.can_start_disarmed_observer -and $he.health_notes.Count -eq 2 -and $he.first_reject -eq '' -and $he.requires_independent_health)
    $b=New-Barrier;$b.Append(($origin+(L .01 'DllFaultParamAPI false!')+(L .02 'New optional UI diagnostic')+(Setup)+$ready),2);$ue=$b.Snapshot(2)
    Check 'optional_metadata_and_unrelated_text_not_false_crash' ($ue.can_start_disarmed_observer -and $ue.uninterpreted_line_count -ge 2)
    $b=New-Barrier;$b.Append(($origin+(L .1 'PX4: GPS 3D fixed & EKF initialized.')),.2);$fe=$b.Snapshot(.2)
    Check 'ready_without_current_simstart_rejected' ($fe.first_reject -eq 'READY_WITHOUT_CURRENT_SIM_START')
    $b=New-Barrier;$b.Append(($origin+(Setup).Replace('Connect to  "COM3"  successful','Connect to  "COM4"  successful')+$ready),2);$fe=$b.Snapshot(2)
    Check 'com4_not_accepted_as_exact_com3_connection' ($fe.first_reject -eq 'SIM_START_WITHOUT_FRESH_DLL_EXPORT_AND_COM3_EVIDENCE')
    $b=New-Barrier;$b.Append(($origin+(Setup).Replace('DlloutHILGPS30dFUN true!','DlloutHILGPS30dFUN false!')+$ready),2);$fe=$b.Snapshot(2)
    Check 'missing_required_dll_export_rejected' ($fe.first_reject -eq 'REQUIRED_DLL_EXPORT_ABSENT')

    $b=New-Barrier;$b.Append($normal,2);$b.Append((L 1.5 'ordinary later line'),3);$fe=$b.Snapshot(3);$examples.reverse_clock=$fe
    Check 'vendor_timestamp_regression_permanently_rejected' ($fe.first_reject -eq 'VENDOR_LOG_CLOCK_REVERSED')
    $b=New-Barrier;$b.Append($normal,2);$fe=$b.Snapshot(1)
    Check 'host_elapsed_regression_rejected' ($fe.first_reject -eq 'HOST_ELAPSED_CLOCK_REVERSED')
    $b=New-Barrier;$b.Append($origin,[double]::NaN);$fe=$b.Snapshot(0)
    Check 'nonfinite_host_elapsed_rejected' ($fe.first_reject -eq 'HOST_ELAPSED_CLOCK_NONFINITE_OR_NEGATIVE')
    $b=New-Barrier;$b.Append(($normal+(L 3 'late other PID' 1234)),3);$fe=$b.Snapshot(3)
    Check 'mixed_or_restarted_pid_rejected' ($fe.first_reject -eq 'PROCESS_IDENTITY_CHANGED_OR_STALE_LOG')
    $b=New-Barrier;$b.Append((Setup),1);$fe=$b.Snapshot(1)
    Check 'old_tail_without_zero_origin_rejected' ($fe.first_reject -eq 'FRESH_LOG_ZERO_ORIGIN_MISSING')
    $b=New-Barrier;$b.Append(($origin+"not a timestamped line`n"),1);$fe=$b.Snapshot(1)
    Check 'malformed_complete_line_rejected' ($fe.first_reject -eq 'MALFORMED_TIMESTAMPED_LOG_LINE')
    $b=New-Barrier;$b.Append(($origin+(Setup)),1);$before=$b.Snapshot(59.999);$after=$b.Snapshot(60)
    Check 'liveness_just_before_bound_waits' ($before.first_reject -eq '' -and -not $before.can_start_disarmed_observer)
    Check 'liveness_at_60_rejects_without_ready' ($after.first_reject -eq 'VENDOR_INITIALIZATION_LIVENESS_TIMEOUT_60S')
    $b=New-Barrier;$b.Append($normal,60);$fe=$b.Snapshot(60)
    Check 'delayed_ready_observation_at_bound_not_laundered' ($fe.first_reject -eq 'READY_OBSERVED_AFTER_INITIALIZATION_DEADLINE')

    foreach($kind in @('not_fresh','not_empty','wrong_sha','wrong_dll_path','bad_sha','missing_pid')){
        $o=New-Options
        switch($kind){
            'not_fresh'{$o.log_created_exclusively_for_this_launch=$false}
            'not_empty'{$o.log_empty_before_launch=$false}
            'wrong_sha'{$o.observed_dll_sha256='B'*64}
            'wrong_dll_path'{$o.observed_dll_path='C:\OTHER.dll'}
            'bad_sha'{$o.expected_dll_sha256='unknown'}
            'missing_pid'{$o.expected_process_id=0}
        }
        $b=[GPENMPC.HostDiagnostics.CopterInitializationBarrier]::new($o);$b.Append($normal,2);$fe=$b.Snapshot(2)
        Check ('external_proof_'+$kind+'_rejected') (-not $fe.can_start_disarmed_observer -and $fe.first_reject -ne '')
    }
    $o=New-Options;$b=[GPENMPC.HostDiagnostics.CopterInitializationBarrier]::new($o);$o.expected_process_id=999;$b.Append($normal,2);$fe=$b.Snapshot(2)
    Check 'constructor_copies_external_proof' ($fe.can_start_disarmed_observer -and $fe.external_proof.expected_process_id -eq 36452)

    $rawPath=(& (Join-Path $PSScriptRoot 'gpenmpc_external_path.ps1') 'copter_initialization_stderr_fixture')
    $realRaw=[IO.File]::ReadAllText($rawPath)
    $b=New-Barrier;$b.Append($realRaw,41);$real=$b.Snapshot(41);$examples.real_readonly_log_replay=$real
    Check 'real_log_replay_one_reboot_two_simstarts' ($real.vendor_reboot_count -eq 1 -and $real.sim_start_count -eq 2)
    Check 'real_log_new_39s_ready_not_pre_reboot_ready' ($real.can_start_disarmed_observer -and $real.last_ready_process_s -eq 39.437 -and $real.last_sim_start_line -gt $real.last_boundary_line)
    Check 'real_log_health_notes_and_prefix_retained' ($real.health_notes.Count -ge 4 -and $real.raw_prefix -ceq $realRaw)
    Check 'text_classifier_never_opens_or_writes_transports' ($real.COM_open -eq 0 -and $real.UDP_open -eq 0 -and $real.board_actions -eq 0 -and $real.parameter_writes -eq 0 -and $real.model_actions -eq 0 -and $real.arm_mode_task_commands -eq 0)
}catch{$errorAtom=$_.Exception.ToString()}
$result=[ordered]@{
    classification='HOST_VENDOR_INITIALIZATION_TEXT_CLASSIFIER'
    pass=($null -eq $errorAtom -and @($checks|Where-Object{-not $_.pass}).Count -eq 0)
    checks_total=$checks.Count;checks_passed=@($checks|Where-Object pass).Count;checks=$checks;error=$errorAtom
    source=$source;source_sha256=(Get-FileHash -LiteralPath $source -Algorithm SHA256).Hash
    test_source=$PSCommandPath;test_source_sha256=(Get-FileHash -LiteralPath $PSCommandPath -Algorithm SHA256).Hash
    parent_log=$rawPath;parent_log_sha256=if($rawPath){(Get-FileHash -LiteralPath $rawPath -Algorithm SHA256).Hash}else{''}
    evidence=$examples;hardware_actions=0;COM_open=0;UDP_open=0;CopterSim_start=0
}
$result|ConvertTo-Json -Depth 20|Set-Content -LiteralPath (Join-Path $OutputRoot 'RESULT.json') -Encoding utf8
[pscustomobject]$result|Select-Object pass,checks_total,checks_passed,error|ConvertTo-Json
if(-not $result.pass){exit 2}
