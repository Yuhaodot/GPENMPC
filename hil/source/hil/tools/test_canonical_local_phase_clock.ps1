$ErrorActionPreference='Stop'
$build=Split-Path -Parent $PSScriptRoot
$phase=Join-Path $build 'evidence\phase_controller'
$out=Join-Path $phase 'phase_clock_native'
$result=Join-Path $out 'RESULT.json'
if(Test-Path -LiteralPath $result){throw 'Preserve phase-clock result.'}
$integration=Join-Path $build 'rfly_vendor_integration'
$combined=(& (Join-Path $PSScriptRoot 'gpenmpc_external_path.ps1') 'canonical_combined_numerics')
$source=Join-Path $integration 'local_phase\test_phase_clock.cpp'
$header=Join-Path $integration 'local_phase\CanonicalLocalPhaseClock.hpp'
$phaseLib=Join-Path $phase 'native\libcanonical_local_phase.a'
$archive=Join-Path $combined 'symbol_isolation\native\libcanonical_local74_private.a'
$facade=(& (Join-Path $PSScriptRoot 'gpenmpc_external_path.ps1') 'full_inner_facade_object')
$rwi=Join-Path $combined 'CONTINUOUS_REFERENCE_FIXTURE.bin'
$sjc=(& (Join-Path $PSScriptRoot 'gpenmpc_external_path.ps1') 'source_gp_joint_chain_fixture')
$gp=Join-Path $build 'evidence\gp_predictor\canonical_gp256.dll'
$passport=(Join-Path (Join-Path (Split-Path (Split-Path (Split-Path $PSScriptRoot -Parent) -Parent) -Parent) 'assets\canonical') 'binding\execution.json')
$task=Join-Path $build 'task_packages\cambridge_canonical\MU_CAMBRIDGE_MA_02__CANONICAL_PHYSICAL_TASK.mat'
$config=(Get-Content -LiteralPath $passport -Raw | ConvertFrom-Json).expected_sha256.effective_configuration_payload
$taskSha=(Get-FileHash -LiteralPath $task -Algorithm SHA256).Hash
$tracked=@($source,$header,$PSCommandPath,$phaseLib,$archive,$facade,$rwi,$sjc,$gp,$passport,$task,
 (Join-Path $integration 'full_inner_abi\CanonicalFullInnerAbi.h'),(Join-Path $integration 'full_inner_abi\test_full_inner_abi.cpp'),
 (Join-Path $integration 'CanonicalLocalInnerInputBuilder.hpp'),(Join-Path $integration 'CanonicalSnapshotStateMapping.hpp'),
 (Join-Path $build 'px4_full_inner\px4_state_adapter\AtomicOdometryAdapter.hpp'))
$before=@($tracked|ForEach-Object{@{path=$_;sha256=(Get-FileHash -LiteralPath $_ -Algorithm SHA256).Hash}})
$tool=(& (Join-Path $PSScriptRoot 'gpenmpc_install_path.ps1') 'llvm' 'bin\clang++.exe')
$exe=Join-Path $out 'test_phase_clock.exe'
$raw=Join-Path $out 'ACTUAL_PHASE_CHAIN.bin'
$numeric=Join-Path $out 'NUMERICAL_RESULT.json'
$arguments=@('-std=c++14','-O2','-Wall','-Wextra','-Werror','-ffp-contract=off','-fno-fast-math','-fstack-usage','-static','-municode',
 '-I',(Join-Path $build 'px4_full_inner\px4_state_adapter\host_stub'),
 '-I',(& (Join-Path $PSScriptRoot 'gpenmpc_external_path.ps1') 'px4_fmuv6c_build_headers'),
 $source,$facade,$archive,$phaseLib,'-o',$exe)
$messages=& $tool @arguments 2>&1;$compile=$LASTEXITCODE
$messages|Out-File -LiteralPath (Join-Path $out 'COMPILE.log') -Encoding utf8
$messages|Write-Output
$run=-1
if($compile -eq 0){$messages=& $exe $rwi $sjc $gp $numeric $raw $config $taskSha 2>&1;$run=$LASTEXITCODE
 $messages|Out-File -LiteralPath (Join-Path $out 'EXECUTION.log') -Encoding utf8;$messages|Write-Output}
$unchanged=$true;foreach($r in $before){if((Get-FileHash -LiteralPath $r.path -Algorithm SHA256).Hash -ne $r.sha256){$unchanged=$false}}
$n=$null;if(Test-Path -LiteralPath $numeric){$n=Get-Content -LiteralPath $numeric -Raw|ConvertFrom-Json}
$r=@{passed=($compile -eq 0 -and $run -eq 0 -and $unchanged);compile_exit_code=$compile;run_exit_code=$run;command=$arguments;
 numerical=$n;source_records=$before;sources_unchanged=$unchanged;
 scope='Snapshot factory and precompiled POD ABI/private74/GP DLL with explicit source-clock, external state/lag and publication fixtures; local-phase and Inner60 references are assessed separately.';
 atomicity_scope='ABI publication/joint install precedes phase validation. Rejected phase retains any actual prior ABI installs/publications and requires owner fail-stop; this helper is not an atomic rollback mechanism.';
 original_phase_math_evidence=(Join-Path $phase 'NATIVE_AND_M7_RESULT.json');board_actions=0;COM=0;UDP=0;model=0;solver=0;old_generated_C_recompiled=0;GP_recompiled=0}
$r|ConvertTo-Json -Depth 10|Out-File -LiteralPath $result -Encoding utf8
if(-not $r.passed){throw 'Phase clock actual chain failed; preserve exact failure.'}
