param([Parameter(Mandatory=$true)][string]$OutputRoot,[string]$ExecutionName='JOINT_STATE_INSTALLER')
$ErrorActionPreference='Stop'
$build=Split-Path -Parent $PSScriptRoot
$generated=Join-Path $OutputRoot 'generated';$native=Join-Path $OutputRoot 'native'
$fixture=Join-Path $OutputRoot 'CONTINUOUS_REFERENCE_FIXTURE.bin'
$innerFixture=Join-Path (& (Join-Path $PSScriptRoot 'gpenmpc_external_path.ps1') 'canonical_local_inner_codegen') 'FIXED_INPUTS.bin'
$sources=@('CanonicalJointStateInstaller.hpp','CanonicalLocalInnerStateStore.hpp','CanonicalReferenceStateStore.hpp' | ForEach-Object {Join-Path $build ('rfly_vendor_integration\'+$_)})
$test=Join-Path $PSScriptRoot 'combined_numerics_harness\joint_state_installer_test.cpp'
$library=Join-Path $native 'libcanonical_combined_numerics.a'
$final=Join-Path $native ($ExecutionName+'_RESULT.json');$raw=Join-Path $native ($ExecutionName+'_RAW_RESULT.json')
$exe=Join-Path $native ($ExecutionName+'_test.exe')
if(Test-Path -LiteralPath $final){throw 'Keep prior completed result.'}
$before=@(($sources+@($test,$fixture,$innerFixture,$library)) | ForEach-Object {@{path=$_;sha256=(Get-FileHash -LiteralPath $_ -Algorithm SHA256).Hash}})
$args=@('-std=c++14','-O2','-ffp-contract=off','-fno-fast-math','-Wall','-Wextra','-Werror','-fstack-usage','-static','-municode',
    '-DGPENMPC_CANONICAL_EXPLICIT_WORKSPACE=1','-I',$generated,'-I',(Join-Path $build 'rfly_vendor_integration'),'-I',(& (Join-Path $PSScriptRoot 'gpenmpc_install_path.ps1') 'matlab' 'extern\include'),$test,$library,'-o',$exe)
$message=& (& (Join-Path $PSScriptRoot 'gpenmpc_install_path.ps1') 'llvm' 'bin\clang++.exe') @args 2>&1;$compile=$LASTEXITCODE
$message | Out-File -LiteralPath (Join-Path $native ($ExecutionName+'_COMPILE.log')) -Encoding utf8
if($compile -ne 0){throw 'Actual joint installer compile failed.'}
$message=& $exe $fixture $innerFixture $raw 2>&1;$run=$LASTEXITCODE
$message | Out-File -LiteralPath (Join-Path $native ($ExecutionName+'_EXECUTION.log')) -Encoding utf8
$message | Write-Output
$r=Get-Content -LiteralPath $raw -Raw | ConvertFrom-Json
$unchanged=$true;foreach($record in $before){if((Get-FileHash -LiteralPath $record.path -Algorithm SHA256).Hash -ne $record.sha256){$unchanged=$false}}
$report=@{schema='JOINT_NUMERIC_REFERENCE_INSTALL_ACTUAL_C_V1';pass=($run -eq 0 -and $r.pass -and $unchanged);native=$r;
    source_records=$before;source_unchanged=$unchanged;compile_arguments=$args;compile_exit_code=$compile;native_exit_code=$run;
    fixture_scope='The first two task-reference query outputs feed combined First/Step with synthetic state inputs to test joint installation.';
    source_tags_scope='Explicit HOST fixture ns/generation at 9ms, not observed hardware';
    boundary='Two validators run before single-owner copies; the copy sequence contains no callbacks and assumes exclusive ownership.';hardware_actions=0}
$report | ConvertTo-Json -Depth 7 | Out-File -LiteralPath $final -Encoding utf8
if(-not $report.pass){throw 'Joint installer regression failed; retain result.'}
