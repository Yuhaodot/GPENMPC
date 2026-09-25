param([Parameter(Mandatory=$true)][string]$ActualRoot)
$ErrorActionPreference='Stop'
$build=Split-Path $PSScriptRoot -Parent
$detail=Join-Path $ActualRoot 'OFFLINE_SENSOR_CONTENT_MATCH.json'
$output=Join-Path $ActualRoot 'OFFLINE_STEP_SNAPSHOT_RECOMPUTE.json'
if((Test-Path -LiteralPath $detail)-or(Test-Path -LiteralPath $output)){throw 'Preserve prior offline results'}
$names=@('RAW_STREAMS.bin','RESULT.json','NUMERICAL_RESULT.json','STEP_SNAPSHOT_STATUS_FINAL.bin',
    'STEP_SNAPSHOT_OBSERVATION.json','GETTER_RING_SECTION_FINAL.bin','GETTER_RING_RECORDS.bin',
    'GETTER_RING_READ_QPC.bin','GETTER_RING_OBSERVATION.json','COPTER_STDIO.raw.log')
$watch=@($names|ForEach-Object {Join-Path $ActualRoot $_})
$before=@(Get-FileHash -LiteralPath $watch -Algorithm SHA256|Select-Object Path,Hash)
$original=Get-Content -LiteralPath (Join-Path $ActualRoot 'RESULT.json') -Raw|ConvertFrom-Json
$numeric=Get-Content -LiteralPath (Join-Path $ActualRoot 'NUMERICAL_RESULT.json') -Raw|ConvertFrom-Json
$reported=Get-Content -LiteralPath (Join-Path $ActualRoot 'STEP_SNAPSHOT_OBSERVATION.json') -Raw|ConvertFrom-Json
$reportedRing=Get-Content -LiteralPath (Join-Path $ActualRoot 'GETTER_RING_OBSERVATION.json') -Raw|ConvertFrom-Json
$sss=[IO.File]::ReadAllBytes((Join-Path $ActualRoot 'STEP_SNAPSHOT_STATUS_FINAL.bin'))
$rdr=[IO.File]::ReadAllBytes((Join-Path $ActualRoot 'GETTER_RING_SECTION_FINAL.bin'))
$records=[IO.File]::ReadAllBytes((Join-Path $ActualRoot 'GETTER_RING_RECORDS.bin'))
if($sss.Length -ne 144 -or $rdr.Length -ne 86176 -or $records.Length%336 -ne 0){throw 'Original raw ABI size mismatch'}
function U32([byte[]]$buffer,[int]$at){[BitConverter]::ToUInt32($buffer,$at)}
function U64([byte[]]$buffer,[int]$at){[BitConverter]::ToUInt64($buffer,$at)}
$checks=[ordered]@{}
$checks.sss_shape=(U32 $sss 0)-eq 0x31535353 -and (U32 $sss 4)-eq 1 -and (U32 $sss 8)-eq 144 -and (U32 $sss 12)-eq 10 -and (U32 $sss 60)-eq 0
$checks.rdr_shape=(U32 $rdr 0)-eq 0x31524452 -and (U32 $rdr 4)-eq 1 -and (U32 $rdr 8)-eq 86176 -and (U32 $rdr 12)-eq 336 -and (U32 $rdr 16)-eq 256
$c=@(0..9|ForEach-Object {U64 $sss (64+8*$_)})
$mirrors=U64 $sss 32;$mirrorConflicts=U64 $sss 40;$mirrorCompleted=U64 $sss 48
$events=U64 $rdr 48;$published=U64 $rdr 32;$consumed=U64 $rdr 40;$dropped=U64 $rdr 56
$retained=[uint64]($records.Length/336)
$checks.owned_exit_and_pid_nonce=$numeric.owned_process_released -and $numeric.udp_ports_released -and $reported.owned_process_exit_established -and
    (U32 $sss 24)-eq [uint32]$numeric.owned_pid -and (U32 $rdr 20)-eq [uint32]$numeric.owned_pid -and
    (U64 $sss 16)-ne 0 -and (U64 $sss 16)-eq (U64 $rdr 64)
$checks.original_report_matches_raw=($reported.counters -join ',')-ceq($c -join ',') -and $reported.mirror_calls-eq $mirrors -and
    $reported.mirror_completions-eq $mirrorCompleted -and $reported.mirror_contentions-eq $mirrorConflicts -and
    $reportedRing.events-eq $events -and $reportedRing.published-eq $published -and $reportedRing.consumed-eq $consumed -and $reportedRing.retained-eq $retained
$checks.total_getter_accounting=$c[5]-eq($c[6]+$c[7]+$c[8]) -and $c[6]-eq $events
$checks.all_original_mirrors_complete=$mirrors-eq $mirrorCompleted -and $mirrorConflicts-eq 0 -and (U32 $sss 56)-eq 0
$checks.ring_complete_with_only_post_exit_retirement=$events-eq $published -and $published-eq $consumed -and $consumed-eq $retained -and
    $dropped-eq 0 -and (U32 $rdr 28)-eq 0 -and (U32 $rdr 80)-eq 4 -and (U64 $rdr 128)-eq 1 -and
    $reportedRing.errors_before_consumer_retirement-eq 0 -and $reportedRing.errors_after_normal_consumer_retirement-eq 4
$checks.raw_event_ordinals_contiguous=$true
for($row=0;$row -lt $retained;$row++){if((U64 $records ($row*336))-ne [uint64]($row+1)){$checks.raw_event_ordinals_contiguous=$false;break}}
# Count every initialized instance with init>=1 in the startup summary.
$otherClauses=$checks.owned_exit_and_pid_nonce -and $checks.total_getter_accounting -and $checks.all_original_mirrors_complete -and
    $c[0]-eq 1 -and $c[3]-gt 0 -and $c[3]-eq $c[4] -and $c[5]-gt 0 -and $c[8]-eq 0 -and $c[9]-eq 0 -and
    $reported.runtime_first_snapshot_fault-eq 0 -and $reported.runtime_status_errors-eq 0 -and $c[1]-eq 0
$legacyClause=$c[2]-eq 1
$preparationClause=$c[2]-ge 1
$checks.corrected_observation_algebra=$otherClauses -and $preparationClause
$checks.exact_old_failure_explained_by_single_init_clause=$otherClauses -and (-not $legacyClause) -and
    $original.native_exit_code-eq 8 -and $numeric.failure-ceq 'STEP_SNAPSHOT_OBSERVATION_INCOMPLETE' -and (-not $reported.observation_complete)
$dllPath=Join-Path $ActualRoot 'runtime\external\model\GPENMPC_M600_Diagnostic.dll'
$exePath=Join-Path $ActualRoot 'runtime\CopterSimNoUI.exe'
$dllSha=(Get-FileHash -LiteralPath $dllPath).Hash
$exeSha=(Get-FileHash -LiteralPath $exePath).Hash
$checks.actual_instrumented_DLL_and_official_NoUI_identity=$dllSha-ceq '990850A2F40F3FCC2A6C47E63A4065B60FF49AA39CC4749FF443963B06F2EF7E' -and
    $exeSha-ceq '94B81EFB44058176DD5353669D9C28FC5331CC8411AB9EA3F2D27C1E8C343241'
$analyzer=Join-Path $PSScriptRoot 'analyze_coptersim_getter_content_ring.exe'
$analyzerSha=(Get-FileHash -LiteralPath $analyzer).Hash
if($analyzerSha -cne '4A91F3160B232D960EC10CA322C66F548669AF80EEBB768EBC765393BEB50A3B'){throw 'Inspected retained analyzer identity changed'}
$analysisOutput=@(& $analyzer $ActualRoot $detail RDR1 2>&1|ForEach-Object {"$_"})
$analysisRc=$LASTEXITCODE
if($analysisRc -ne 0){throw 'Offline retained content analyzer failed'}
$content=Get-Content -LiteralPath $detail -Raw|ConvertFrom-Json
$checks.exact_full_wire_content_matching=$content.complete_retained_getter_denominator -and $content.getter_records-eq $retained -and
    $content.tcp_hil_sensor_frames-eq $numeric.hil_sensor_messages -and $content.mutually_unique_content_pairs-eq $content.tcp_hil_sensor_frames -and
    $content.wires_classified_by_matching_getter_count.zero-eq 0 -and $content.wires_classified_by_matching_getter_count.multiple-eq 0 -and
    $content.getters_classified_by_matching_wire_count.multiple-eq 0 -and $content.invalid_length-eq 0 -and $content.nonfinite_conversion-eq 0 -and
    $content.bad_crc-eq 0 -and $content.bad_signature-eq 0 -and $content.byte_comparison_controls_pass
$checks.all_matched_original_records_valid=$true
foreach($pair in $content.mutually_unique_retained_pairs){$at=[int]$pair.retained_getter_index*336
    if($records[$at+328]-ne 1 -or $records[$at+329]-ne 0 -or $records[$at+330]-ne 0){$checks.all_matched_original_records_valid=$false;break}}
$after=@(Get-FileHash -LiteralPath $watch -Algorithm SHA256|Select-Object Path,Hash)
$checks.all_original_artifacts_SHA_unchanged=($before|ConvertTo-Json -Compress)-ceq($after|ConvertTo-Json -Compress)
$pass=@($checks.Values|Where-Object {-not $_}).Count-eq 0
$result=[ordered]@{scope='OFFLINE_NOUI_SNAPSHOT_RECOMPUTATION';pass=$pass;checks=$checks;
    original_native_exit_code=$original.native_exit_code;original_failure=$numeric.failure;original_failure_files_preserved=$true;
    summary_change=[ordered]@{old='initializations == 1';correct='initializations >= 1 AND all initialization guards';old_clause=$legacyClause;corrected_clause=$preparationClause;
        reason='Repeated initialization is permitted before the first step/getter/completion; faults remain latched.'};
    original_counters=$c;initializations_satisfy_pre_step_guard_inference=$checks.corrected_observation_algebra;
    inference_boundary='Derived from the instrumented DLL identity, fault status and sticky-guard implementation.';
    pid=(U32 $sss 24);same_ring_nonce=(U64 $sss 16);final_status_state=(U32 $sss 28);status_state_is_not_exit_proof=$true;
    mirror_calls=$mirrors;mirror_completions=$mirrorCompleted;ring_retained=$retained;ring_dropped=$dropped;
    sensor_content=[ordered]@{mapping='Original getter binary64[1..13] converted to binary32; exact52bytes. No time/ordinal/sequence/receive-order filters or tolerance.';
        wire_frames=$content.tcp_hil_sensor_frames;mutually_unique_pairs=$content.mutually_unique_content_pairs;wire_distribution=$content.wires_classified_by_matching_getter_count;
        getter_distribution=$content.getters_classified_by_matching_wire_count;all_matched_original_records_valid=$checks.all_matched_original_records_valid;
        distinct_original_getter_sensor13=$content.getter_original_13_double_sensor_population_excluding_time.distinct_values;
        distinct_wire_sensor13=$content.wire_13_float_sensor_population.distinct_values;detail_path=$detail};
    actual_DLL_SHA256=$dllSha;official_NoUI_SHA256=$exeSha;analyzer_SHA256=$analyzerSha;analyzer_exit_code=$analysisRc;analyzer_output=$analysisOutput;
    original_input_SHA256=$before;new_NoUI_runs=0;COM=0;PX4_processes=0;real_board_receiver_HRT_association_proven=$false;
    live_HITL_or_control_authority_proven=$false;source_script_SHA256=(Get-FileHash -LiteralPath $PSCommandPath).Hash}
$result|ConvertTo-Json -Depth 10|Set-Content -LiteralPath $output
$analysisOutput
[pscustomobject]@{pass=$pass;checks=$checks.Count;failed=@($checks.Keys|Where-Object {-not $checks[$_]});output=$output}|ConvertTo-Json -Depth 4
if(-not $pass){exit 1}
