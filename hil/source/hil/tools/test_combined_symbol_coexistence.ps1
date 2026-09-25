param([string]$RunName='symbol_coexistence')
$ErrorActionPreference='Stop'
if($RunName -notmatch '^[A-Za-z0-9_]+$'){throw 'Local run name only'}
$build=Split-Path -Parent $PSScriptRoot
$combined=(& (Join-Path $PSScriptRoot 'gpenmpc_external_path.ps1') 'canonical_combined_numerics')
$out=Join-Path $combined 'symbol_isolation\native'
$old=Join-Path $build 'evidence\arm_controller\GPENMPC_Rfly_Canonical_Controller_ert_rtw'
$new=Join-Path $combined 'generated'
$legacy=(& (Join-Path $PSScriptRoot 'gpenmpc_external_path.ps1') 'trajectory_executor_generated_source')
$harness=Join-Path $PSScriptRoot 'combined_numerics_harness'
$namespace=Join-Path $build 'rfly_vendor_integration\CanonicalCombinedSymbolNamespace.h'
$library=Join-Path $out 'libcanonical_local74_private.a'
$reference=Join-Path $combined 'CONTINUOUS_REFERENCE_FIXTURE.bin'
$inner=Join-Path (& (Join-Path $PSScriptRoot 'gpenmpc_external_path.ps1') 'canonical_local_inner_codegen') 'FIXED_INPUTS.bin'
$gp=Join-Path $build 'evidence\gp_predictor\canonical_gp256.dll'
$baseline=(& (Join-Path $PSScriptRoot 'gpenmpc_external_path.ps1') 'source_gp_joint_chain_fixture')
$passport=(Join-Path (Join-Path (Split-Path (Split-Path (Split-Path $PSScriptRoot -Parent) -Parent) -Parent) 'assets\canonical') 'binding\execution.json')
$config=(Get-Content -LiteralPath $passport -Raw|ConvertFrom-Json).expected_sha256.effective_configuration_payload
$task=Join-Path $build 'task_packages\cambridge_canonical\MU_CAMBRIDGE_MA_02__CANONICAL_PHYSICAL_TASK.mat'
$taskSha=(Get-FileHash -LiteralPath $task -Algorithm SHA256).Hash
$innerSha=(Get-FileHash -LiteralPath $inner -Algorithm SHA256).Hash
$refSha=(Get-FileHash -LiteralPath $reference -Algorithm SHA256).Hash
$headers=(& (Join-Path $PSScriptRoot 'gpenmpc_external_path.ps1') 'px4_fmuv6c_build_headers')
$source=Join-Path $harness 'source_to_gp_joint_chain_test.cpp'
$tool=(& (Join-Path $PSScriptRoot 'gpenmpc_install_path.ps1') 'llvm' 'bin')
$final=Join-Path $out ($RunName+'_RESULT.json')
if(Test-Path -LiteralPath $final){throw 'Preserve prior result'}
$oldFiles=@((Join-Path $old 'GPENMPC_Rfly_Canonical_Controller.c'),(Join-Path $old 'rt_nonfinite.c'),(Join-Path $old 'rtGetInf.c'))
$legacyFiles=@((Join-Path $legacy 'rtrpdc_cg.c'),(Join-Path $legacy 'rtGetNaN.c'))
$probeFiles=@((Join-Path $harness 'symbol_coexistence_old_probe.c'),(Join-Path $harness 'symbol_coexistence_new_probe.c'),
    (Join-Path $harness 'symbol_coexistence_main.cpp'),(Join-Path $harness 'symbol_coexistence_probe.h'))
$sources=@($PSCommandPath,$namespace,$library,$reference,$inner,$gp,$baseline,$passport,$task,$source)+$oldFiles+$legacyFiles+$probeFiles+@(
    (Join-Path $old 'rtwtypes.h'),(Join-Path $new 'rtwtypes.h'),(Join-Path $legacy 'rtwtypes.h'),
    (Join-Path $build 'rfly_vendor_integration\CanonicalLocalInnerInputBuilder.hpp'),(Join-Path $build 'rfly_vendor_integration\CanonicalSnapshotStateMapping.hpp'),
    (Join-Path $build 'rfly_vendor_integration\CanonicalLocalInnerStateStore.hpp'),(Join-Path $build 'rfly_vendor_integration\CanonicalReferenceStateStore.hpp'),
    (Join-Path $build 'rfly_vendor_integration\CanonicalJointStateInstaller.hpp'),(Join-Path $build 'px4_full_inner\px4_state_adapter\AtomicOdometryAdapter.hpp'))
$before=@($sources|ForEach-Object{@{path=$_;sha256=(Get-FileHash -LiteralPath $_ -Algorithm SHA256).Hash}})
$flags=@('-O2','-ffp-contract=off','-fno-fast-math','-Wall','-Wextra','-Werror','-fstack-usage')
$objects=@();$commands=@();$index=0
foreach($file in ($oldFiles+$legacyFiles+@($probeFiles[0],$probeFiles[1]))){
    $object=Join-Path $out ($RunName+'_'+$index+'.o')
    $args=@('-std=c11')+$flags+@('-I',$harness)
    if($file -eq $probeFiles[1]){$args+=@('-include',$namespace,'-I',$new,'-I',(& (Join-Path $PSScriptRoot 'gpenmpc_install_path.ps1') 'matlab' 'extern\include'))}
    else{$args+=@('-I',$old)}
    $args+=@('-c',$file,'-o',$object)
    $messages=& (Join-Path $tool 'clang.exe') @args 2>&1;$rc=$LASTEXITCODE
    $messages|Out-File -LiteralPath (Join-Path $out ($RunName+'_COMPILE_'+$index+'.log')) -Encoding utf8
    $commands+=@{compiler='clang';arguments=$args;exit_code=$rc}
    if($rc -ne 0){$messages|Write-Output;throw 'Actual independent C TU compilation failed'}
    $objects+=$object;++$index
}
$chainObject=Join-Path $out ($RunName+'_chain.o')
$args=@('-std=c++14')+$flags+@('-DGPENMPC_CANONICAL_EXPLICIT_WORKSPACE=1','-Dwmain=source_gp_chain_original_main','-include',$namespace,
    '-I',$new,'-I',(Join-Path $build 'rfly_vendor_integration'),'-I',$PSScriptRoot,
    '-I',(Join-Path $build 'px4_full_inner\px4_state_adapter\host_stub'),'-I',$headers,'-I',(& (Join-Path $PSScriptRoot 'gpenmpc_install_path.ps1') 'matlab' 'extern\include'),
    '-c',$source,'-o',$chainObject)
$messages=& (Join-Path $tool 'clang++.exe') @args 2>&1;$rc=$LASTEXITCODE
$messages|Out-File -LiteralPath (Join-Path $out ($RunName+'_COMPILE_CHAIN.log')) -Encoding utf8
$commands+=@{compiler='clang++';arguments=$args;exit_code=$rc}
if($rc -ne 0){$messages|Write-Output;throw 'SJC chain translation-unit compilation failed.'}
$exe=Join-Path $out ($RunName+'.exe')
$args=@('-std=c++14')+$flags+@('-static','-municode',$probeFiles[2],$chainObject)+$objects+@($library,'-o',$exe)
$messages=& (Join-Path $tool 'clang++.exe') @args 2>&1;$rc=$LASTEXITCODE
$messages|Out-File -LiteralPath (Join-Path $out ($RunName+'_LINK.log')) -Encoding utf8
$commands+=@{compiler='clang++';arguments=$args;exit_code=$rc}
if($rc -ne 0){$messages|Write-Output;throw 'Actual old/new strong-symbol coexistence link failed'}
$rawResult=Join-Path $out ($RunName+'_CHAIN_RESULT.json');$raw=Join-Path $out ($RunName+'_RAW.bin');$probe=Join-Path $out ($RunName+'_PROBE.json')
$messages=& $exe $reference $inner $gp $rawResult $raw $config $taskSha $innerSha $refSha $probe 2>&1;$run=$LASTEXITCODE
$messages|Out-File -LiteralPath (Join-Path $out ($RunName+'_EXECUTION.log')) -Encoding utf8
$messages|Write-Output
$chain=Get-Content -LiteralPath $rawResult -Raw|ConvertFrom-Json
$p=Get-Content -LiteralPath $probe -Raw|ConvertFrom-Json
$oldRaw=[IO.File]::ReadAllBytes($baseline);$newRaw=[IO.File]::ReadAllBytes($raw)
$firstMismatch=-1
if($oldRaw.Length -ne $newRaw.Length){$firstMismatch=[Math]::Min($oldRaw.Length,$newRaw.Length)}
else {for($j=0;$j -lt $oldRaw.Length;++$j){if($oldRaw[$j] -ne $newRaw[$j]){$firstMismatch=$j;break}}}
$unchanged=$true;foreach($s in $before){if((Get-FileHash -LiteralPath $s.path -Algorithm SHA256).Hash -ne $s.sha256){$unchanged=$false}}
$symbols=@(& (Join-Path $tool 'llvm-nm.exe') --defined-only --extern-only --just-symbol-name $exe)
$symbolCheck=@('rtNaN','gpenmpc_local74_rtNaN','rt_powd_snf','gpenmpc_local74_rt_powd_snf','GPENMPC_Rfly_Canonical_Controller_step','gpenmpcNative_canonicalLocalInnerFixedFirst','gpenmpcNative_canonicalLocalInnerFixedStep')
$present=@($symbolCheck|ForEach-Object{@{symbol=$_;present=($_ -in $symbols)}})
$report=@{scope='HOST_ACTUAL_OLD_3C_AND_PRIVATE_NEW_74C_COEXISTENCE';pass=($run -eq 0 -and $chain.pass -and $p.pass -and $firstMismatch -eq -1 -and $unchanged -and @($present|Where-Object{-not $_.present}).Count -eq 0);
    native_exit_code=$run;chain=$chain;nonfinite_and_pow_probes=$p;sources_unchanged=$unchanged;source_records=$before;commands=$commands;
    actual_original_old_3C=$oldFiles;actual_parent_legacy_source=$legacyFiles;symbols=$present;
    baseline_raw_path=$baseline;baseline_raw_sha256=(Get-FileHash -LiteralPath $baseline -Algorithm SHA256).Hash;
    coexistence_raw_path=$raw;coexistence_raw_sha256=(Get-FileHash -LiteralPath $raw -Algorithm SHA256).Hash;raw_bytes=$newRaw.Length;first_mismatching_byte=$firstMismatch;
    every_raw_byte_including_nonfinite_bits_equal=($firstMismatch -eq -1);old_SJC_main_source_logic_changed=$false;
    old_SJC_entry_symbol_only_renamed_at_compile=$true;no_combined_types_in_old_or_driver_TU=$true;
    COM=0;PX4_builds=0;model_runs=0;board_actions=0}
$report|ConvertTo-Json -Depth 9|Out-File -LiteralPath $final -Encoding utf8
if(-not $report.pass){throw 'Actual coexistence regression failed; original and new raw retained'}
