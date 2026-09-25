param([Parameter(Mandatory=$true)][string]$OutputRoot,[string]$ExecutionName='REFERENCE_STATE_STORE')
$ErrorActionPreference='Stop'
$build=Split-Path -Parent $PSScriptRoot
$generated=Join-Path $OutputRoot 'generated'
$native=Join-Path $OutputRoot 'native'
$fixture=Join-Path $OutputRoot 'CONTINUOUS_REFERENCE_FIXTURE.bin'
$source=Join-Path $build 'rfly_vendor_integration\CanonicalReferenceStateStore.hpp'
$test=Join-Path $PSScriptRoot 'combined_numerics_harness\reference_state_store_test.cpp'
$library=Join-Path $native 'libcanonical_combined_numerics.a'
$final=Join-Path $native ($ExecutionName+'_RESULT.json')
$exe=Join-Path $native ($ExecutionName+'_test.exe')
$raw=Join-Path $native ($ExecutionName+'_RAW_RESULT.json')
if(Test-Path -LiteralPath $final){throw 'Keep prior completed result.'}
$before=@($source,$test,$fixture,$library | ForEach-Object {@{path=$_;sha256=(Get-FileHash -LiteralPath $_ -Algorithm SHA256).Hash}})
$args=@('-std=c++14','-O2','-ffp-contract=off','-fno-fast-math','-Wall','-Wextra','-Werror','-fstack-usage','-static','-municode',
    '-I',$generated,'-I',(Join-Path $build 'rfly_vendor_integration'),'-I',(& (Join-Path $PSScriptRoot 'gpenmpc_install_path.ps1') 'matlab' 'extern\include'),$test,$library,'-o',$exe)
$message=& (& (Join-Path $PSScriptRoot 'gpenmpc_install_path.ps1') 'llvm' 'bin\clang++.exe') @args 2>&1
$compile=$LASTEXITCODE
$message | Out-File -LiteralPath (Join-Path $native ($ExecutionName+'_COMPILE.log')) -Encoding utf8
if($compile -ne 0){throw 'Reference store compilation failed.'}
$message=& $exe $fixture $raw 2>&1
$run=$LASTEXITCODE
$message | Out-File -LiteralPath (Join-Path $native ($ExecutionName+'_EXECUTION.log')) -Encoding utf8
$message | Write-Output
$r=Get-Content -LiteralPath $raw -Raw | ConvertFrom-Json
$unchanged=$true
foreach($record in $before){if((Get-FileHash -LiteralPath $record.path -Algorithm SHA256).Hash -ne $record.sha256){$unchanged=$false}}
$report=@{schema='PURE_REFERENCE_STORE_ACTUAL_C_V1';pass=($run -eq 0 -and $r.pass -and $unchanged);native=$r;
    source_records=$before;source_unchanged=$unchanged;compile_arguments=$args;compile_exit_code=$compile;native_exit_code=$run;
    original_fixture_result=(Join-Path $native 'CONTINUOUS_REFERENCE_RESULT.json');
    boundary='Reference-state storage with mocked publication and source clocks.';
    compile_log=(Join-Path $native ($ExecutionName+'_COMPILE.log'));
    hardware_actions=0}
$report | ConvertTo-Json -Depth 7 | Out-File -LiteralPath $final -Encoding utf8
if(-not $report.pass){throw 'Reference store regression failed.'}
