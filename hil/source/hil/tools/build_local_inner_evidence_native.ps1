param([Parameter(Mandatory=$true)][string]$GenerationRoot,[Parameter(Mandatory=$true)][string]$OutputRoot)
$ErrorActionPreference='Stop'
if(Test-Path -LiteralPath $OutputRoot){throw 'Preserve previous execution evidence.'}
$generation=Get-Content -LiteralPath (Join-Path $GenerationRoot 'RESULT.json') -Raw | ConvertFrom-Json
if(-not $generation.pass -or -not $generation.generated){throw 'Actual MATLAB parity and generation required.'}
New-Item -ItemType Directory -Path $OutputRoot | Out-Null
$bin=(& (Join-Path $PSScriptRoot 'gpenmpc_install_path.ps1') 'llvm' 'bin')
$generated=Join-Path $GenerationRoot 'generated'
$records=@(Get-ChildItem -LiteralPath $generated -File | Where-Object Extension -in '.c','.h' | ForEach-Object {
    [ordered]@{path=$_.FullName;bytes=$_.Length;sha256=(Get-FileHash -LiteralPath $_.FullName).Hash}
})
$objects=[Collections.Generic.List[string]]::new()
$commands=[Collections.Generic.List[object]]::new()
$log=[Collections.Generic.List[string]]::new()
foreach($c in (Get-ChildItem -LiteralPath $generated -Filter '*.c' -File)){
    $object=Join-Path $OutputRoot ($c.BaseName+'.o')
    $args=@('-std=c11','-O2','-ffp-contract=off','-fno-fast-math','-Wall','-Wextra','-Werror',
        '-I',$generated,'-I',(& (Join-Path $PSScriptRoot 'gpenmpc_install_path.ps1') 'matlab' 'extern\include'),'-c',$c.FullName,'-o',$object)
    $text=& (Join-Path $bin 'clang.exe') @args 2>&1;$rc=$LASTEXITCODE
    $commands.Add([ordered]@{program='clang.exe';arguments=$args;exit_code=$rc})
    foreach($line in $text){$log.Add([string]$line)}
    if($rc -ne 0){throw "Generated C compilation failed: $($c.Name)"}
    $objects.Add($object)
}
$library=Join-Path $OutputRoot 'liblocal_inner_with_evidence.a'
& (Join-Path $bin 'llvm-ar.exe') rcs $library @($objects.ToArray())
if($LASTEXITCODE -ne 0){throw 'Archive failure.'}
$exe=Join-Path $OutputRoot 'local_inner_evidence.exe'
$harness=Join-Path $PSScriptRoot 'local_inner_evidence_native.c'
$args=@('-std=c11','-O2','-ffp-contract=off','-fno-fast-math','-Wall','-Wextra','-Werror','-static',
    '-I',$generated,'-I',(& (Join-Path $PSScriptRoot 'gpenmpc_install_path.ps1') 'matlab' 'extern\include'),$harness,$library,'-o',$exe)
if($generation.learning_audit_included){$args=@('-DGPENMPC_CANONICAL_LEARNING_AUDIT=1')+$args}
$text=& (Join-Path $bin 'clang.exe') @args 2>&1;$rc=$LASTEXITCODE
$commands.Add([ordered]@{program='clang.exe';arguments=$args;exit_code=$rc})
foreach($line in $text){$log.Add([string]$line)}
$log | Set-Content -LiteralPath (Join-Path $OutputRoot 'BUILD_LOG.txt')
if($rc -ne 0){throw 'HOST evidence harness compile failed.'}
$resultPath=Join-Path $OutputRoot 'NUMERICAL_RESULT.json'
$text=& $exe (Join-Path $GenerationRoot 'EVIDENCE_INPUTS.bin') (Join-Path $OutputRoot 'ACTUAL_OUTPUTS.bin') $resultPath
$rc=$LASTEXITCODE
$text | Set-Content -LiteralPath (Join-Path $OutputRoot 'EXECUTION_LOG.txt')
$numerical=Get-Content -LiteralPath $resultPath -Raw | ConvertFrom-Json
$unchanged=$true
foreach($r in $records){if((Get-FileHash -LiteralPath $r.path).Hash -ne $r.sha256){$unchanged=$false}}
$report=[ordered]@{pass=($rc -eq 0 -and $numerical.pass -and $unchanged);numerical=$numerical;
    generated_sources=$records;compiled_c_count=$objects.Count;commands=$commands.ToArray();
    sources_unchanged=$unchanged;library_sha256=(Get-FileHash -LiteralPath $library).Hash;
    binary_sha256=(Get-FileHash -LiteralPath $exe).Hash;fixture_sha256=(Get-FileHash -LiteralPath (Join-Path $GenerationRoot 'EVIDENCE_INPUTS.bin')).Hash;
    harness_sha256=(Get-FileHash -LiteralPath $harness).Hash;
    comparison='Scaled numerical error <= 1e-10; discrete flags match exactly.'}
$report | ConvertTo-Json -Depth 12 | Set-Content -LiteralPath (Join-Path $OutputRoot 'RESULT.json')
$text
if(-not $report.pass){throw 'Actual generated C numerical comparison failed.'}
