param([Parameter(Mandatory=$true)][string]$EvidenceRoot)
$ErrorActionPreference='Stop'
$toolRoot=Split-Path -Parent $MyInvocation.MyCommand.Path
$clang=(& (Join-Path $PSScriptRoot 'gpenmpc_install_path.ps1') 'llvm' 'bin\clang.exe')
$readobj=(& (Join-Path $PSScriptRoot 'gpenmpc_install_path.ps1') 'llvm' 'bin\llvm-readobj.exe')
$generated=Join-Path $EvidenceRoot 'generated_c'
$native=Join-Path $EvidenceRoot 'native'
if(Test-Path -LiteralPath $native){throw 'Choose an unused output path.'}
New-Item -ItemType Directory -Path $native | Out-Null
$log=Join-Path $native 'COMPILE.log'
$objects=@()
$common=@('-std=c11','-O2','-ffp-contract=off','-fno-fast-math','-Wall','-Wextra','-Werror','-fstack-usage',('-I'+$generated),('-I'+(& (Join-Path $PSScriptRoot 'gpenmpc_install_path.ps1') 'matlab' 'extern\include')))
$sources=@(Get-ChildItem -LiteralPath $generated -Filter '*.c' | Sort-Object Name)
$hashes=@($sources | ForEach-Object { @{path=$_.FullName;sha256=(Get-FileHash -LiteralPath $_.FullName -Algorithm SHA256).Hash} })
foreach($source in $sources){
    $object=Join-Path $native ($source.BaseName+'.o')
    $output=& $clang @common '-c' $source.FullName '-o' $object 2>&1
    $code=$LASTEXITCODE
    $output | Out-File -LiteralPath $log -Append -Encoding utf8
    if($code -ne 0){throw "Actual C compilation failed: $($source.Name), rc=$code"}
    $objects+=$object
}
$harness=Join-Path $toolRoot 'canonical_reference_native_test.c'
$exe=Join-Path $native 'canonical_reference_native_test.exe'
$output=& $clang @common $harness @objects '-o' $exe 2>&1
$code=$LASTEXITCODE;$output | Out-File -LiteralPath $log -Append -Encoding utf8
if($code -ne 0){throw "Native test link failed rc=$code"}
$fixture=Join-Path $EvidenceRoot 'NATIVE_REFERENCE_FIXTURE.bin'
$resultPath=Join-Path $native 'NUMERICAL_RESULT.json'
$output=& $exe $fixture $resultPath 2>&1
$code=$LASTEXITCODE;$output | Out-File -LiteralPath $log -Append -Encoding utf8
$output | Write-Output
$imports=& $readobj '--coff-imports' $exe 2>&1
$imports | Out-File -LiteralPath (Join-Path $native 'PE_IMPORTS.txt') -Encoding utf8
$unchanged=$true
foreach($source in $hashes){if((Get-FileHash -LiteralPath $source.path -Algorithm SHA256).Hash -ne $source.sha256){$unchanged=$false}}
$result=@{schema='CANONICAL_REFERENCE_NATIVE_BUILD_V1';pass=($code -eq 0 -and $unchanged);exit_code=$code;
    actual_c_translation_units=$sources.Count;compiler=$clang;flags=$common;generated_sources_unchanged=$unchanged;
    generated_sources=$hashes;exe_sha256=(Get-FileHash -LiteralPath $exe -Algorithm SHA256).Hash;
    fixture_sha256=(Get-FileHash -LiteralPath $fixture -Algorithm SHA256).Hash;
    scope='Host native-C numerical validation'}
$result | ConvertTo-Json -Depth 6 | Out-File -LiteralPath (Join-Path $native 'RESULT.json') -Encoding utf8
if($code -ne 0){throw "Actual numeric C replay failed rc=$code"}
