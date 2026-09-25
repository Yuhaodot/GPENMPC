param([Parameter(Mandatory=$true)][string]$OutputRoot,[string]$GeneratedRoot='',[string]$LibraryPath='',[string]$FixturePath='',[switch]$ExplicitWorkspace)
$ErrorActionPreference='Stop'
$build=Split-Path -Parent $PSScriptRoot
if(-not $GeneratedRoot){$GeneratedRoot=(& (Join-Path $PSScriptRoot 'gpenmpc_external_path.ps1') 'canonical_local_inner_codegen')}
$gpRoot=Join-Path $build 'evidence\gp_predictor'
$dll=Join-Path $gpRoot 'canonical_gp256.dll'
$gpReceipt=Get-Content -LiteralPath (Join-Path $gpRoot 'DLL_RESULT.json') -Raw|ConvertFrom-Json
if(-not $gpReceipt.native.pass -or (Get-FileHash -LiteralPath $dll -Algorithm SHA256).Hash -ne $gpReceipt.dll_sha256){throw 'GP identity mismatch.'}
if(Test-Path -LiteralPath $OutputRoot){throw 'Preserve previous test.'}
New-Item -ItemType Directory -Path $OutputRoot|Out-Null
$clang=(& (Join-Path $PSScriptRoot 'gpenmpc_install_path.ps1') 'llvm' 'bin\clang++.exe')
$exe=Join-Path $OutputRoot 'state_store.exe'
$fixture=Join-Path $generatedRoot 'FIXED_INPUTS.bin'
$library=Join-Path $generatedRoot 'native\libcanonical_local_inner.a'
if($LibraryPath){$library=$LibraryPath}
if($FixturePath){$fixture=$FixturePath}
$source=Join-Path $PSScriptRoot 'test_canonical_local_state_store.cpp'
$header=Join-Path $build 'rfly_vendor_integration\CanonicalLocalInnerStateStore.hpp'
$args=@('-std=c++14','-O2','-ffp-contract=off','-fno-fast-math','-Wall','-Wextra','-Werror','-static','-municode',
 '-I',(Join-Path $generatedRoot 'generated'),'-I',(& (Join-Path $PSScriptRoot 'gpenmpc_install_path.ps1') 'matlab' 'extern\include'),'-I',$PSScriptRoot,$source,$library,'-o',$exe)
if($ExplicitWorkspace){$args=@('-DGPENMPC_CANONICAL_EXPLICIT_WORKSPACE=1')+$args}
$log=& $clang @args 2>&1;$rc=$LASTEXITCODE
$log|Set-Content -LiteralPath (Join-Path $OutputRoot 'COMPILE_LOG.txt')
if($rc -ne 0){throw "Actual compiler failed $rc"}
$run=@($fixture,$dll,(Join-Path $OutputRoot 'CHECKS.csv'),(Join-Path $OutputRoot 'NUMERICAL_RESULT.json'))
$log=& $exe @run 2>&1;$rc=$LASTEXITCODE
([string]::Join([Environment]::NewLine,@($log)))|Set-Content -LiteralPath (Join-Path $OutputRoot 'EXECUTION_LOG.txt')
$numericPath=Join-Path $OutputRoot 'NUMERICAL_RESULT.json'
$numeric=if(Test-Path -LiteralPath $numericPath){Get-Content -LiteralPath $numericPath -Raw|ConvertFrom-Json}else{[pscustomobject]@{pass=$false;first_atom='NATIVE_EARLY_EXIT_BEFORE_RESULT';exit_code=$rc;partial_checks=(Join-Path $OutputRoot 'CHECKS.csv')}}
$result=[ordered]@{status=if($rc -eq 0 -and $numeric.pass){'PASS_HOST_GENERATED_MATH_CANDIDATE_INSTALL_ORDER'}else{'FAIL_HOST_GENERATED_MATH_CANDIDATE_INSTALL_ORDER'};native=$numeric;exit_code=$rc;compile_arguments=$args;execution_arguments=$run;identities=@(@($source,$header,$library,$fixture,$dll)|ForEach-Object {[ordered]@{path=$_;sha256=(Get-FileHash -LiteralPath $_ -Algorithm SHA256).Hash;bytes=(Get-Item -LiteralPath $_).Length}});publisher='Explicit mock receipts.';actual_control_authority=$false;board_installed=$false;hardware_actions=0;clock_and_source_admission='Owned by scheduler and actual publisher, not inferred by this pure numerical state store';method_change=$false}
$result|ConvertTo-Json -Depth 15|Set-Content -LiteralPath (Join-Path $OutputRoot 'RESULT.json')
$log
if($rc -ne 0 -or -not $numeric.pass){throw 'Actual state-store test failed.'}
