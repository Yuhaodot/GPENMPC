param([Parameter(Mandatory=$true)][string]$OutputRoot)
$ErrorActionPreference='Stop'
$build=Split-Path -Parent $PSScriptRoot
$generated=Join-Path $OutputRoot 'generated'
$target=Join-Path $OutputRoot 'native'
if(Test-Path -LiteralPath $target){throw 'Use a new combined-output directory.'}
New-Item -ItemType Directory -Path $target | Out-Null
$report=Get-Content -LiteralPath (Join-Path $OutputRoot 'RESULT.json') -Raw | ConvertFrom-Json
if(-not $report.pass -or -not $report.sources_unchanged -or $report.stack_usage_max -ne 2048 -or -not $report.multi_instance_code){throw 'Actual combined codegen identity not valid.'}
$compilerRoot=(& (Join-Path $PSScriptRoot 'gpenmpc_install_path.ps1') 'llvm' 'bin')
$clang=Join-Path $compilerRoot 'clang.exe';$ar=Join-Path $compilerRoot 'llvm-ar.exe'
$readobj=Join-Path $compilerRoot 'llvm-readobj.exe'
$harness=Join-Path $PSScriptRoot 'combined_numerics_harness'
$flags=@('-std=c11','-O2','-ffp-contract=off','-fno-fast-math','-Wall','-Wextra','-Werror','-fstack-usage',
    '-I',$generated,'-I',(& (Join-Path $PSScriptRoot 'gpenmpc_install_path.ps1') 'matlab' 'extern\include'),'-I',$PSScriptRoot,'-I',$harness)
$sources=@(Get-ChildItem -LiteralPath $generated -Filter '*.c' -File | Sort-Object Name)
$sourceRecords=@(Get-ChildItem -LiteralPath $generated -File | Where-Object {$_.Extension -in '.c','.h'} | ForEach-Object {
    @{path=$_.FullName;bytes=$_.Length;sha256=(Get-FileHash -LiteralPath $_.FullName -Algorithm SHA256).Hash}})
$objects=@();$commands=@();$log=Join-Path $target 'COMPILE.log'
foreach($source in $sources){
    $object=Join-Path $target ($source.BaseName+'.o')
    $args=$flags+@('-c',$source.FullName,'-o',$object)
    $messages=& $clang @args 2>&1;$code=$LASTEXITCODE
    $messages | Out-File -LiteralPath $log -Append -Encoding utf8
    $commands+=@{program=$clang;arguments=$args;exit_code=$code}
    if($code -ne 0){throw "Actual combined C compile failed: $($source.Name)"}
    $objects+=$object
}
$library=Join-Path $target 'libcanonical_combined_numerics.a'
& $ar 'rcs' $library @objects
if($LASTEXITCODE -ne 0){throw 'Combined archive failed.'}
foreach($name in @('inner_test','reference_test','gp_chain_test','resource_probe')){
    $exe=Join-Path $target ($name+'.exe');$source=Join-Path $harness ($name+'.c')
    $args=$flags+@($source,$library,'-o',$exe)
    if($name -eq 'gp_chain_test'){$args+='-municode'}
    $messages=& $clang @args 2>&1;$code=$LASTEXITCODE
    $messages | Out-File -LiteralPath $log -Append -Encoding utf8
    $commands+=@{program=$clang;arguments=$args;exit_code=$code}
    if($code -ne 0){throw "Combined harness link failed: $name"}
}
$innerFixture=Join-Path (& (Join-Path $PSScriptRoot 'gpenmpc_external_path.ps1') 'canonical_local_inner_codegen') 'FIXED_INPUTS.bin'
$referenceFixture=Join-Path (& (Join-Path $PSScriptRoot 'gpenmpc_external_path.ps1') 'canonical_reference_window_codegen') 'NATIVE_REFERENCE_FIXTURE.bin'
$gp=Join-Path $build 'evidence\gp_predictor\canonical_gp256.dll'
if((Get-FileHash -LiteralPath $gp -Algorithm SHA256).Hash -ne '3F62DBD05D125BD087513A85A2D61F70EDD62F699009225C9300A2B07EDD5FBC'){throw 'Original GP DLL identity mismatch.'}
$runs=@(
    @{name='inner_test';args=@($innerFixture,(Join-Path $target 'INNER_ACTUAL.bin'),(Join-Path $target 'INNER_RESULT.json'))},
    @{name='reference_test';args=@($referenceFixture,(Join-Path $target 'REFERENCE_RESULT.json'))},
    @{name='gp_chain_test';args=@($innerFixture,$gp,(Join-Path $target 'GP_CHAIN_ACTUAL.bin'),(Join-Path $target 'GP_CHAIN_ROWS.csv'),(Join-Path $target 'GP_CHAIN_RESULT.json'))},
    @{name='resource_probe';args=@((Join-Path $target 'RESOURCE_RESULT.json'))})
$executions=@()
foreach($run in $runs){
    $exe=Join-Path $target ($run.name+'.exe');$args=$run.args
    $messages=& $exe @args 2>&1;$code=$LASTEXITCODE
    $messages | Out-File -LiteralPath (Join-Path $target 'EXECUTION.log') -Append -Encoding utf8
    $messages | Write-Output
    $executions+=@{name=$run.name;exit_code=$code;arguments=$args}
    if($code -ne 0){throw "Actual combined C replay failed: $($run.name) rc=$code"}
}
$imports=& $readobj '--coff-imports' (Join-Path $target 'gp_chain_test.exe') 2>&1
$imports | Out-File -LiteralPath (Join-Path $target 'PE_IMPORTS.txt') -Encoding utf8
$stack=@(Get-ChildItem -LiteralPath $target -Filter '*.su' | ForEach-Object {
    $sourcePath=$_.FullName
    Get-Content -LiteralPath $sourcePath | ForEach-Object {
        $parts=$_ -split "`t";if($parts.Count -ge 3){@{source_record=$parts[0];static_bytes=[int]$parts[1];kind=$parts[2];su_path=$sourcePath}}
    }})
$unchanged=$true
foreach($source in $sourceRecords){if((Get-FileHash -LiteralPath $source.path -Algorithm SHA256).Hash -ne $source.sha256){$unchanged=$false}}
$final=@{schema='CANONICAL_COMBINED_NUMERICS_NATIVE_V1';pass=$unchanged;actual_generated_translation_units=$sources.Count;
    compiler=$clang;flags=$flags;commands=$commands;executions=$executions;source_records=$sourceRecords;sources_unchanged=$unchanged;
    stack_usage_max_codegen=2048;multi_instance_code=$true;compiler_stack_records=$stack;
    resource=(Get-Content -LiteralPath (Join-Path $target 'RESOURCE_RESULT.json') -Raw | ConvertFrom-Json);
    inner=(Get-Content -LiteralPath (Join-Path $target 'INNER_RESULT.json') -Raw | ConvertFrom-Json);
    reference=(Get-Content -LiteralPath (Join-Path $target 'REFERENCE_RESULT.json') -Raw | ConvertFrom-Json);
    gp_chain=(Get-Content -LiteralPath (Join-Path $target 'GP_CHAIN_RESULT.json') -Raw | ConvertFrom-Json);
    library_sha256=(Get-FileHash -LiteralPath $library -Algorithm SHA256).Hash;
    scope='HOST numerical C with an explicit serial-owner workspace.';board_actions=0;
    gp_location='Separate non-reentrant GP DLL with a fail-closed try-lock.';
    gp_dll_sha256=(Get-FileHash -LiteralPath $gp -Algorithm SHA256).Hash}
$final | ConvertTo-Json -Depth 12 | Out-File -LiteralPath (Join-Path $target 'RESULT.json') -Encoding utf8
if(-not $unchanged){throw 'Generated sources changed during build.'}
