param([Parameter(Mandatory=$true)][string]$PlanPath,[Parameter(Mandatory=$true)][string]$RunRoot)
$ErrorActionPreference='Stop'
# Copy consumed inputs before working-source edits and write a compact index.
$plan=Get-Content -LiteralPath $PlanPath -Raw | ConvertFrom-Json
$target=Join-Path ([IO.Path]::GetFullPath($RunRoot)) 'CONSUMED_INPUTS'
if(Test-Path -LiteralPath $target){throw 'Existing consumed inputs must be preserved.'}
foreach($ref in $plan.references){
    $f=Get-Item -LiteralPath $ref.path
    if($f.Length -ne $ref.bytes -or (Get-FileHash -LiteralPath $f.FullName).Hash -ne $ref.sha256){throw "Changed consumed input: $($ref.label)"}
}
New-Item -ItemType Directory -Path $target | Out-Null
$rows=@()
foreach($ref in $plan.references){
    $fileName=[string]$ref.label + [IO.Path]::GetExtension([string]$ref.path)
    $dest=Join-Path $target $fileName
    Copy-Item -LiteralPath $ref.path -Destination $dest
    $f=Get-Item -LiteralPath $dest
    if($f.Length -ne $ref.bytes -or (Get-FileHash -LiteralPath $dest).Hash -ne $ref.sha256){throw 'Copied input mismatch.'}
    $f.IsReadOnly=$true
    $rows+=@{label=$ref.label;original_path=$ref.path;retained_path=$dest;bytes=$ref.bytes;sha256=$ref.sha256}
}
$planDest=Join-Path $target 'CONSUMED_PLAN.json'
Copy-Item -LiteralPath $PlanPath -Destination $planDest
(Get-Item -LiteralPath $planDest).IsReadOnly=$true
$receipt=[ordered]@{status='CONSUMED_INPUTS_RETAINED';plan_sha256=(Get-FileHash -LiteralPath $planDest).Hash;entries=$rows;hardware_actions=0}
$receiptPath=Join-Path $target 'SOURCE_INDEX.json'
[IO.File]::WriteAllText($receiptPath,($receipt|ConvertTo-Json -Depth 8),[Text.UTF8Encoding]::new($false))
(Get-Item -LiteralPath $receiptPath).IsReadOnly=$true
Write-Output ("Preserved {0} input files." -f $rows.Count)
