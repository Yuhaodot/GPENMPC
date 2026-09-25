param([Parameter(Mandatory=$true)][string]$RunRoot)
$ErrorActionPreference='Stop'
# Seal completed run files.
$resolved=(Resolve-Path -LiteralPath $RunRoot).Path
$allowed=(& (Join-Path $PSScriptRoot 'gpenmpc_external_path.ps1') 'm600_recording_output_root')
if(-not $resolved.StartsWith($allowed,[StringComparison]::OrdinalIgnoreCase)){throw 'Run root is outside the intended phase.'}
$manifestPath=Join-Path $resolved 'FINAL_CONTENT_MANIFEST.json'
if(Test-Path -LiteralPath $manifestPath){throw 'A prior manifest must remain untouched.'}
$outer=Get-Content -LiteralPath (Join-Path $resolved 'OUTER_RESULT.json') -Raw | ConvertFrom-Json
$summary=Get-Content -LiteralPath (Join-Path $resolved 'MILESTONE_SUMMARY.json') -Raw | ConvertFrom-Json
if($outer.status -ne 'LIVE_OUTCOME_RECORDED_AND_BOARD_SAFE' -or -not $outer.final_serial_safety_passed -or $outer.safety_recovery_remaining){throw 'Complete safety evidence is required before sealing.'}
if(-not $summary.formal_completed -or -not $summary.final_safety.COM_closed -or -not $summary.final_safety.UDP_closed){throw 'Incomplete milestone.'}
$items=@(Get-ChildItem -LiteralPath $resolved -Recurse -Force -File)
$rows=@(foreach($item in $items){
    if($item.Attributes -band [IO.FileAttributes]::ReparsePoint){throw 'Reparse point cannot enter the ordinary-file manifest.'}
    [ordered]@{relative_path=$item.FullName.Substring($resolved.Length+1);bytes=$item.Length;sha256=(Get-FileHash -LiteralPath $item.FullName -Algorithm SHA256).Hash}
})
[long]$totalBytes=0
foreach($entry in $rows){$totalBytes += [long]$entry['bytes']}
$manifest=[ordered]@{
    schema='M600_COMPLETED_RUN_CONTENT_MANIFEST_V1'
    classification=$summary.status
    scope='All existing ordinary files in the completed run, including raw data and exact consumed sources'
    entries=$rows
    entry_count=$rows.Count
    total_referenced_bytes=$totalBytes
    only_excluded_path='FINAL_CONTENT_MANIFEST.json'
    exclusion_reason='Manifest self-reference only; its SHA is reported separately'
    sealing_hardware_actions=0
}
[IO.File]::WriteAllText($manifestPath,($manifest | ConvertTo-Json -Depth 7),[Text.UTF8Encoding]::new($false))
foreach($row in $rows){
    $p=Join-Path $resolved $row.relative_path;$item=Get-Item -LiteralPath $p
    if($item.Length -ne $row.bytes -or (Get-FileHash -LiteralPath $p -Algorithm SHA256).Hash -ne $row.sha256){throw "Final independent rehash mismatch: $($row.relative_path)"}
}
$all=@(Get-ChildItem -LiteralPath $resolved -Recurse -Force -File)
if($all.Count -ne $rows.Count+1){throw 'Unaddressed file appeared during the seal.'}
foreach($item in $all){$item.IsReadOnly=$true}
$writable=@(Get-ChildItem -LiteralPath $resolved -Recurse -Force -File | Where-Object {-not $_.IsReadOnly})
if($writable.Count -ne 0){throw 'ReadOnly seal incomplete.'}
[ordered]@{status='CONTENT_REHASHED_AND_READONLY';entries=$rows.Count;bytes=$manifest.total_referenced_bytes;ordinary_files=$all.Count;readonly=$all.Count;writable=0;sha256=(Get-FileHash -LiteralPath $manifestPath).Hash;hardware_actions=0}|ConvertTo-Json
