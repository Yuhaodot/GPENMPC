param([string]$RunName='inner_input_builder')
$ErrorActionPreference='Stop'
if($RunName -notmatch '^[A-Za-z0-9_]+$'){throw 'Local run name only'}
$build=Split-Path -Parent $PSScriptRoot
$root=Join-Path (& (Join-Path $PSScriptRoot 'gpenmpc_external_path.ps1') 'canonical_combined_numerics') 'native'
$mapping=Join-Path (& (Join-Path $PSScriptRoot 'gpenmpc_external_path.ps1') 'full_inner_px4_float_mapping') 'MATLAB_NED_AND_MAPPED_STATE.bin'
$inner=Join-Path (& (Join-Path $PSScriptRoot 'gpenmpc_external_path.ps1') 'canonical_local_inner_codegen') 'FIXED_INPUTS.bin'
$passport=(Join-Path (Join-Path (Split-Path (Split-Path (Split-Path $PSScriptRoot -Parent) -Parent) -Parent) 'assets\canonical') 'binding\execution.json')
$config=(Get-Content -LiteralPath $passport -Raw | ConvertFrom-Json).expected_sha256.effective_configuration_payload
$task=Join-Path $build 'task_packages\cambridge_canonical\MU_CAMBRIDGE_MA_02__CANONICAL_PHYSICAL_TASK.mat'
$taskSha=(Get-FileHash -LiteralPath $task -Algorithm SHA256).Hash
$innerSha=(Get-FileHash -LiteralPath $inner -Algorithm SHA256).Hash
$headers=(& (Join-Path $PSScriptRoot 'gpenmpc_external_path.ps1') 'px4_fmuv6c_build_headers')
$sources=@($passport,$mapping,$inner,$task,(Join-Path $PSScriptRoot 'test_local_inner_input_builder.cpp'),
    (Join-Path $build 'rfly_vendor_integration\CanonicalLocalInnerInputBuilder.hpp'),(Join-Path $build 'rfly_vendor_integration\CanonicalSnapshotStateMapping.hpp'),
    (Join-Path $build 'rfly_vendor_integration\SlimKernelCodec.hpp'),(Join-Path $build 'rfly_vendor_integration\BoardLocalInnerSchedule.hpp'),
    (Join-Path $build 'px4_full_inner\px4_state_adapter\AtomicOdometryAdapter.hpp'),(Join-Path $headers 'uORB\topics\vehicle_odometry.h'))
$before=@($sources|ForEach-Object {@{path=$_;sha256=(Get-FileHash -LiteralPath $_ -Algorithm SHA256).Hash}})
$final=Join-Path $root ($RunName+'_RESULT.json');if(Test-Path -LiteralPath $final){throw 'Keep previous result'}
$raw=Join-Path $root ($RunName+'_RAW_RESULT.json');$exe=Join-Path $root ($RunName+'.exe')
$args=@('-std=c++14','-O2','-ffp-contract=off','-fno-fast-math','-Wall','-Wextra','-Werror','-fstack-usage','-static','-municode',
    '-I',(Join-Path $build 'px4_full_inner\px4_state_adapter\host_stub'),'-I',$headers,'-I',(Join-Path $build 'rfly_vendor_integration'),
    (Join-Path $PSScriptRoot 'test_local_inner_input_builder.cpp'),'-o',$exe)
$message=& (& (Join-Path $PSScriptRoot 'gpenmpc_install_path.ps1') 'llvm' 'bin\clang++.exe') @args 2>&1;$compile=$LASTEXITCODE
$message|Out-File -LiteralPath (Join-Path $root ($RunName+'_COMPILE.log')) -Encoding utf8
if($compile -ne 0){throw 'Actual builder compilation failed'}
$message=& $exe $mapping $inner $raw $config $taskSha $innerSha 2>&1;$run=$LASTEXITCODE
$message|Out-File -LiteralPath (Join-Path $root ($RunName+'_EXECUTION.log')) -Encoding utf8
$message|Write-Output
$r=Get-Content -LiteralPath $raw -Raw|ConvertFrom-Json
$unchanged=$true;foreach($record in $before){if((Get-FileHash -LiteralPath $record.path -Algorithm SHA256).Hash -ne $record.sha256){$unchanged=$false}}
$report=@{schema='CANONICAL_INPUT_BUILDER_PRIVATE_SNAPSHOT_V1';pass=($run -eq 0 -and $r.pass -and $unchanged);native=$r;
    source_records=$before;sources_unchanged=$unchanged;compile_arguments=$args;compile_exit_code=$compile;native_exit_code=$run;
    external_binding_scope='HOST fixture descriptors with exact original-input-file provenance and mocked clock association; no actual rotor/clock provider implemented';
    source_dt_scope='Private snapshot keeps first actual_delta=0; explicit first interval read from immutable LCI fixture; later original actual_delta_us*1e-6, no time snapping';
    configuration_sha256=$config;hardware_actions=0}
$report|ConvertTo-Json -Depth 7|Out-File -LiteralPath $final -Encoding utf8
if(-not $report.pass){throw 'Actual builder tests failed; retain result'}
