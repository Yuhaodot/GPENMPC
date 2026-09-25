param([Parameter(Mandatory=$true)][string]$OutputRoot)
$ErrorActionPreference='Stop'
$build=Split-Path -Parent $PSScriptRoot
$combined=(& (Join-Path $PSScriptRoot 'gpenmpc_external_path.ps1') 'canonical_combined_numerics')
$generated=Join-Path $combined 'generated'
$library=Join-Path $combined 'native\libcanonical_combined_numerics.a'
$fixture=Join-Path (& (Join-Path $PSScriptRoot 'gpenmpc_external_path.ps1') 'canonical_local_inner_codegen') 'FIXED_INPUTS.bin'
$dll=Join-Path $build 'evidence\gp_predictor\canonical_gp256.dll'
$dllReceipt=Get-Content -LiteralPath (Join-Path $build 'evidence\gp_predictor\DLL_RESULT.json') -Raw|ConvertFrom-Json
if(-not $dllReceipt.native.pass -or (Get-FileHash -LiteralPath $dll).Hash -ne $dllReceipt.dll_sha256){throw 'Original GP DLL identity mismatch'}
$source=Join-Path $PSScriptRoot 'test_board_schedule_math.cpp'
$sources=@($source,(Join-Path $build 'rfly_vendor_integration\BoardLocalInnerSchedule.hpp'),
 (Join-Path $build 'rfly_vendor_integration\CanonicalLocalInnerStateStore.hpp'),$library,$fixture,$dll)
$before=@($sources|ForEach-Object{[ordered]@{path=$_;sha256=(Get-FileHash -LiteralPath $_ -Algorithm SHA256).Hash;bytes=(Get-Item -LiteralPath $_).Length}})
if(Test-Path -LiteralPath $OutputRoot){throw 'Prior result must be preserved'}
New-Item -ItemType Directory -Path $OutputRoot|Out-Null
$clang=(& (Join-Path $PSScriptRoot 'gpenmpc_install_path.ps1') 'llvm' 'bin\clang++.exe')
$exe=Join-Path $OutputRoot 'schedule_math.exe'
$compileArgs=@('-DGPENMPC_CANONICAL_EXPLICIT_WORKSPACE=1','-std=c++14','-O2','-ffp-contract=off','-fno-fast-math',
 '-Wall','-Wextra','-Werror','-static','-municode','-I',$generated,'-I',(& (Join-Path $PSScriptRoot 'gpenmpc_install_path.ps1') 'matlab' 'extern\include'),
 '-I',$PSScriptRoot,$source,$library,'-o',$exe)
$compileOutput=& $clang @compileArgs 2>&1;$compileRc=$LASTEXITCODE
$compileOutput|Set-Content -LiteralPath (Join-Path $OutputRoot 'COMPILE_LOG.txt')
$runArgs=@($fixture,$dll,(Join-Path $OutputRoot 'CHECKS.csv'),(Join-Path $OutputRoot 'TIMING.csv'),
 (Join-Path $OutputRoot 'ACTUAL_MATH.bin'),(Join-Path $OutputRoot 'NUMERICAL_RESULT.json'))
$runRc=$null;$numeric=$null
if($compileRc -eq 0){$runOutput=& $exe @runArgs 2>&1;$runRc=$LASTEXITCODE
 $runOutput|Set-Content -LiteralPath (Join-Path $OutputRoot 'EXECUTION_LOG.txt')
 if(Test-Path -LiteralPath $runArgs[5]){$numeric=Get-Content -LiteralPath $runArgs[5] -Raw|ConvertFrom-Json}
}
$stable=$true;foreach($item in $before){if((Get-FileHash -LiteralPath $item.path -Algorithm SHA256).Hash -ne $item.sha256){$stable=$false}}
$result=[ordered]@{status=if($compileRc -eq 0 -and $runRc -eq 0 -and $numeric.pass -and $stable){'PASS_HOST_SCHEDULE_ACTUAL_C_GP_EVENT_CHAIN'}else{'FAIL_HOST_SCHEDULE_ACTUAL_C_GP_EVENT_CHAIN'};
 compiler=$clang;compile_arguments=$compileArgs;compile_exit_code=$compileRc;run_arguments=$runArgs;run_exit_code=$runRc;result=$numeric;
 inputs=$before;input_sha_stable=$stable;method_changed=$false;simulated_event_limits=[ordered]@{source_max_age_us=1000;gp_reply_max_age_us=20000;production_profile=$false};
 limitations=@('The fixture provides 60 numerical reference jets.',
 'Source, rotor, authority, HRT and publication are explicit mocks.',
 'Fixture nanoseconds are exactly divisible by 1000 for the test unit conversion.',
 'QPC measures HOST computation time independently of simulated event time.',
 'First row skips GP: 59 GP fills; 58 required replies passed into later source calls and 59 prior slots including the initial unavailable slot.',
 'Outer and reference decisions are fixed fixture inputs.');COM=0;UDP=0;plant_runs=0;board_actions=0}
$result|ConvertTo-Json -Depth 15|Set-Content -LiteralPath (Join-Path $OutputRoot 'RESULT.json')
if($runOutput){$runOutput}
if($result.status -notlike 'PASS*'){throw ('Schedule math integration failed: compile={0}, run={1}' -f $compileRc,$runRc)}
