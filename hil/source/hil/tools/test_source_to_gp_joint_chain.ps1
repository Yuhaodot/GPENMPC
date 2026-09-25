param([string]$RunName='source_gp_joint_chain')
$ErrorActionPreference='Stop'
if($RunName -notmatch '^[A-Za-z0-9_]+$'){throw 'Local run name only'}
$build=Split-Path -Parent $PSScriptRoot
$combined=(& (Join-Path $PSScriptRoot 'gpenmpc_external_path.ps1') 'canonical_combined_numerics')
$out=Join-Path $combined 'native'
$reference=Join-Path $combined 'CONTINUOUS_REFERENCE_FIXTURE.bin'
$inner=Join-Path (& (Join-Path $PSScriptRoot 'gpenmpc_external_path.ps1') 'canonical_local_inner_codegen') 'FIXED_INPUTS.bin'
$gp=Join-Path $build 'evidence\gp_predictor\canonical_gp256.dll'
$library=Join-Path $out 'libcanonical_combined_numerics.a'
$passport=(Join-Path (Join-Path (Split-Path (Split-Path (Split-Path $PSScriptRoot -Parent) -Parent) -Parent) 'assets\canonical') 'binding\execution.json')
$config=(Get-Content -LiteralPath $passport -Raw|ConvertFrom-Json).expected_sha256.effective_configuration_payload
$task=Join-Path $build 'task_packages\cambridge_canonical\MU_CAMBRIDGE_MA_02__CANONICAL_PHYSICAL_TASK.mat'
$taskSha=(Get-FileHash -LiteralPath $task -Algorithm SHA256).Hash
$innerSha=(Get-FileHash -LiteralPath $inner -Algorithm SHA256).Hash
$refSha=(Get-FileHash -LiteralPath $reference -Algorithm SHA256).Hash
$px4Headers=(& (Join-Path $PSScriptRoot 'gpenmpc_external_path.ps1') 'px4_fmuv6c_build_headers')
$source=Join-Path $PSScriptRoot 'combined_numerics_harness\source_to_gp_joint_chain_test.cpp'
$sources=@($passport,$reference,$inner,$gp,$library,$task,$source,$PSCommandPath,
    (Join-Path $build 'rfly_vendor_integration\CanonicalLocalInnerInputBuilder.hpp'),
    (Join-Path $build 'rfly_vendor_integration\CanonicalSnapshotStateMapping.hpp'),
    (Join-Path $build 'rfly_vendor_integration\CanonicalLocalInnerStateStore.hpp'),
    (Join-Path $build 'rfly_vendor_integration\CanonicalReferenceStateStore.hpp'),
    (Join-Path $build 'rfly_vendor_integration\CanonicalJointStateInstaller.hpp'),
    (Join-Path $build 'px4_full_inner\px4_state_adapter\AtomicOdometryAdapter.hpp'),
    (Join-Path $px4Headers 'uORB\topics\vehicle_odometry.h'))
$before=@($sources|ForEach-Object{@{path=$_;sha256=(Get-FileHash -LiteralPath $_ -Algorithm SHA256).Hash}})
$final=Join-Path $out ($RunName+'_RESULT.json')
if(Test-Path -LiteralPath $final){throw 'Preserve previous result'}
$rawResult=Join-Path $out ($RunName+'_RAW_RESULT.json')
$raw=Join-Path $out ($RunName+'_RAW.bin')
$exe=Join-Path $out ($RunName+'.exe')
$args=@('-std=c++14','-O2','-ffp-contract=off','-fno-fast-math','-Wall','-Wextra','-Werror','-fstack-usage','-static','-municode',
    '-DGPENMPC_CANONICAL_EXPLICIT_WORKSPACE=1','-I',(Join-Path $combined 'generated'),
    '-I',(Join-Path $build 'rfly_vendor_integration'),'-I',$PSScriptRoot,
    '-I',(Join-Path $build 'px4_full_inner\px4_state_adapter\host_stub'),'-I',$px4Headers,
    '-I',(& (Join-Path $PSScriptRoot 'gpenmpc_install_path.ps1') 'matlab' 'extern\include'),$source,$library,'-o',$exe)
$messages=& (& (Join-Path $PSScriptRoot 'gpenmpc_install_path.ps1') 'llvm' 'bin\clang++.exe') @args 2>&1
$compile=$LASTEXITCODE
$messages|Out-File -LiteralPath (Join-Path $out ($RunName+'_COMPILE.log')) -Encoding utf8
if($compile -ne 0){$messages|Write-Output;throw 'Actual joint source chain compile failed'}
$messages=& $exe $reference $inner $gp $rawResult $raw $config $taskSha $innerSha $refSha 2>&1
$run=$LASTEXITCODE
$messages|Out-File -LiteralPath (Join-Path $out ($RunName+'_EXECUTION.log')) -Encoding utf8
$messages|Write-Output
$r=Get-Content -LiteralPath $rawResult -Raw|ConvertFrom-Json
$unchanged=$true
foreach($s in $before){if((Get-FileHash -LiteralPath $s.path -Algorithm SHA256).Hash -ne $s.sha256){$unchanged=$false}}
$report=@{schema='ACTUAL_SNAPSHOT_REFERENCE_NUMERIC_JOINT_GP_HOST_CHAIN_V1';pass=($run -eq 0 -and $r.pass -and $unchanged);
    native=$r;source_records=$before;sources_unchanged=$unchanged;compile_arguments=$args;compile_exit_code=$compile;native_exit_code=$run;
    configuration_sha256=$config;
    fixture_scope='First 60 task queries and MATLAB reference values paired with synthetic state, lag, payload and wind; float32 PX4 topic conversion and 9 ms source clocks are explicit fixtures.';
    oracle_scope='Independent generated C evaluates each builder input36 with separate resident state; each path calls the GP DLL serially.';
    association_scope='Private Snapshot factory is actual production code. Rotor-clock association, publication, UID/boot runtime observation and source clock are explicit HOST fixtures, not providers or hardware authority.';
    raw_layout=@{endian='little';header='ASCII SJC1, uint32 declared_rows, uint32 sizeof_vehicle_odometry';
        row='vehicle_odometry_s original bytes; uint64 tags2; double input36; double transition41; double store_state64; double store_kernel61; float store_controls16; double request19; double gp18; double direct_state64; double direct_kernel61; float direct_controls16; double direct_gp18; double store_pending70; double direct_pending70'};
    raw_path=$raw;raw_bytes=(Get-Item -LiteralPath $raw).Length;raw_sha256=(Get-FileHash -LiteralPath $raw -Algorithm SHA256).Hash;
    hardware_actions=0}
$report|ConvertTo-Json -Depth 8|Out-File -LiteralPath $final -Encoding utf8
if(-not $report.pass){throw 'Actual joint source chain failed; preserve negative receipt'}
