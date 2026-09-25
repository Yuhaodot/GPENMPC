param([Parameter(Mandatory=$true)][string]$GeneratedRoot,[Parameter(Mandatory=$true)][string]$OutputRoot,[string]$LibraryAttempt='native',[string]$LibraryPath='',[string]$FixturePath='',[switch]$ExplicitWorkspace)
$ErrorActionPreference='Stop'
$build=Split-Path -Parent $PSScriptRoot
$gpRoot=Join-Path $build 'evidence\gp_predictor'
$dll=Join-Path $gpRoot 'canonical_gp256.dll'
$gpReceipt=Get-Content -LiteralPath (Join-Path $gpRoot 'DLL_RESULT.json') -Raw | ConvertFrom-Json
$generated=Join-Path $GeneratedRoot 'generated'
$library=Join-Path (Join-Path $GeneratedRoot $LibraryAttempt) 'libcanonical_local_inner.a'
$fixture=Join-Path $GeneratedRoot 'FIXED_INPUTS.bin'
if($LibraryPath){$library=$LibraryPath}
if($FixturePath){$fixture=$FixturePath}
$harness=Join-Path $PSScriptRoot 'canonical_local_inner_gp_chain_test.c'
$clang=(& (Join-Path $PSScriptRoot 'gpenmpc_install_path.ps1') 'llvm' 'bin\clang.exe')
$objdump=(& (Join-Path $PSScriptRoot 'gpenmpc_install_path.ps1') 'llvm' 'bin\llvm-objdump.exe')
$includes=(& (Join-Path $PSScriptRoot 'gpenmpc_install_path.ps1') 'matlab' 'extern\include')
if(Test-Path -LiteralPath $OutputRoot){throw 'Preserve previous numerical replay.'}
$inputs=@($dll,$library,$fixture,$harness,(Join-Path $gpRoot 'DLL_RESULT.json'))
$identities=@($inputs|ForEach-Object {$item=Get-Item -LiteralPath $_;[ordered]@{path=$item.FullName;bytes=$item.Length;sha256=(Get-FileHash -LiteralPath $_ -Algorithm SHA256).Hash}})
if(-not $gpReceipt.native.pass -or $gpReceipt.native_exit_code -ne 0 -or
   (Get-FileHash -LiteralPath $dll -Algorithm SHA256).Hash -ne $gpReceipt.dll_sha256){throw 'Original GP DLL receipt/hash is not PASS.'}
New-Item -ItemType Directory -Path $OutputRoot | Out-Null
$exe=Join-Path $OutputRoot 'canonical_inner_gp_chain.exe'
$arguments=@('-std=c11','-O2','-ffp-contract=off','-fno-fast-math','-Wall','-Wextra','-Werror','-static','-municode','-I',$generated,'-I',$includes,'-I',$PSScriptRoot,$harness,$library,'-o',$exe)
if($ExplicitWorkspace){$arguments=@('-DGPENMPC_CANONICAL_EXPLICIT_WORKSPACE=1')+$arguments}
$log=& $clang @arguments 2>&1;$rc=$LASTEXITCODE
$log|Set-Content -LiteralPath (Join-Path $OutputRoot 'COMPILE_LOG.txt')
if($rc -ne 0){throw "Actual C harness compile failed: $rc"}
$imports=& $objdump '-p' $exe 2>&1
$imports|Set-Content -LiteralPath (Join-Path $OutputRoot 'IMPORTS.txt')
$deps=@($imports|Select-String -Pattern 'DLL Name:\s*(.+)'|ForEach-Object {$_.Matches[0].Groups[1].Value.Trim()})
if(@($deps|Where-Object {$_ -match '(?i)(matlab|libmx|libmex|mkl|blas|lapack|emlrt)'}).Count){throw 'Unexpected MATLAB runtime dependency.'}
$runArgs=@($fixture,$dll,(Join-Path $OutputRoot 'ACTUAL_OUTPUTS.bin'),(Join-Path $OutputRoot 'ROWS.csv'),(Join-Path $OutputRoot 'NUMERICAL_RESULT.json'))
$runLog=& $exe @runArgs 2>&1;$rc=$LASTEXITCODE
$runLog|Set-Content -LiteralPath (Join-Path $OutputRoot 'EXECUTION_LOG.txt')
$numeric=Get-Content -LiteralPath (Join-Path $OutputRoot 'NUMERICAL_RESULT.json') -Raw | ConvertFrom-Json
$unchanged=$true
foreach($item in $identities){if((Get-FileHash -LiteralPath $item.path -Algorithm SHA256).Hash -ne $item.sha256){$unchanged=$false}}
$pass=($rc -eq 0 -and $numeric.pass -and $unchanged)
$result=[ordered]@{
 status=if($pass){'PASS_HOST_CONTINUOUS_GENERATED_INNER_WITH_ORIGINAL_NATIVE_GP'}else{'FAIL_HOST_CONTINUOUS_GENERATED_INNER_WITH_ORIGINAL_NATIVE_GP'}
 pass=$pass;native_exit_code=$rc;numeric=$numeric;input_identities=$identities;inputs_unchanged=$unchanged
 compiled_command=[ordered]@{program=$clang;arguments=$arguments};execution_arguments=$runArgs
 exe_sha256=(Get-FileHash -LiteralPath $exe -Algorithm SHA256).Hash;binary_imports=$deps
 comparison_tolerance_before_run='Numerical comparison: abs(actual-expected)/max(1,abs(expected)) <= 1e-10.'
 explicit_owner_workspace=[bool]$ExplicitWorkspace
 state_chain='Actual generated inner C output state and actually evaluated original GP pending used on next sample; archived pending used only for comparison'
 gp_model='256-inducing-point double-precision GP model in a HOST native DLL.'
 gp_completion='Pending fields preserve assignment and multiplication order; the next source invokes innovation closure.'
 timings='Windows replay timings for all 60 rows.'
 fixed_input_domain='60-row numerical comparison fixture.'
 official16_scope='Numeric mapping only, not official Simulink block or transport publication'
 callable_preconditions='The production owner validates source/reference/rotor/GP identity, freshness and successful publication before committing state.'
 COM_open=0;board_access=0;plant_runs=0;solver_calls=0;MAVLink_or_UDP=0;publication_authority=$false
}
$result|ConvertTo-Json -Depth 16|Set-Content -LiteralPath (Join-Path $OutputRoot 'RESULT.json')
$runLog
if(-not $pass){throw 'Actual continuous numerical replay failed.'}
