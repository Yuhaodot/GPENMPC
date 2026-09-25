param([Parameter(Mandatory=$true)][string]$OutputRoot)
$ErrorActionPreference='Stop'
$build=Split-Path -Parent $PSScriptRoot
$native=Join-Path $OutputRoot 'native'
$generated=Join-Path $OutputRoot 'generated'
$fixture=Join-Path $OutputRoot 'CONTINUOUS_REFERENCE_FIXTURE.bin'
$harness=Join-Path $PSScriptRoot 'combined_numerics_harness\continuous_reference_test.c'
$library=Join-Path $native 'libcanonical_combined_numerics.a'
$exe=Join-Path $native 'continuous_reference_test.exe'
$raw=Join-Path $native 'CONTINUOUS_REFERENCE_RAW_RESULT.json'
$final=Join-Path $native 'CONTINUOUS_REFERENCE_RESULT.json'
if((Test-Path -LiteralPath $exe) -or (Test-Path -LiteralPath $final)){throw 'Preserve previous test execution.'}
$inputs=@($fixture,$harness,$library,(Join-Path (& (Join-Path $PSScriptRoot 'gpenmpc_external_path.ps1') 'canonical_reference_window_transition') 'RAW.mat'))
$inputs+=@(Get-ChildItem -LiteralPath $generated -File | Where-Object {$_.Extension -in '.h','.c'} | ForEach-Object FullName)
$before=@($inputs | ForEach-Object {@{path=$_;sha256=(Get-FileHash -LiteralPath $_ -Algorithm SHA256).Hash}})
$clang=(& (Join-Path $PSScriptRoot 'gpenmpc_install_path.ps1') 'llvm' 'bin\clang.exe')
$args=@('-std=c11','-O2','-ffp-contract=off','-fno-fast-math','-Wall','-Wextra','-Werror','-fstack-usage',
    '-I',$generated,'-I',(& (Join-Path $PSScriptRoot 'gpenmpc_install_path.ps1') 'matlab' 'extern\include'),$harness,$library,'-o',$exe)
$message=& $clang @args 2>&1;$compileCode=$LASTEXITCODE
$message | Out-File -LiteralPath (Join-Path $native 'CONTINUOUS_REFERENCE_COMPILE.log') -Encoding utf8
if($compileCode -ne 0){throw 'Actual continuous reference test compilation failed.'}
$message=& $exe $fixture $raw 2>&1;$runCode=$LASTEXITCODE
$message | Out-File -LiteralPath (Join-Path $native 'CONTINUOUS_REFERENCE_EXECUTION.log') -Encoding utf8
$message | Write-Output
$result=Get-Content -LiteralPath $raw -Raw | ConvertFrom-Json
$unchanged=$true
foreach($input in $before){if((Get-FileHash -LiteralPath $input.path -Algorithm SHA256).Hash -ne $input.sha256){$unchanged=$false}}
$report=@{schema='CONTINUOUS_REFERENCE_COMBINED_C_REGRESSION_V1';pass=($result.pass -and $unchanged -and $runCode -eq 0);
    native=$result;old_check_coverage_total=($result.checks+1);old_check_coverage_passed=($result.passed+[int]$unchanged);
    external_original_source_identity_check=$unchanged;source_records=$before;compile_arguments=$args;
    compile_exit_code=$compileCode;native_exit_code=$runCode;library=$library;
    exe_sha256=(Get-FileHash -LiteralPath $exe -Algorithm SHA256).Hash;
    separation='3765 saved-task reference/transition queries; Inner60 uses a separate synthetic reference fixture.';
    inner_reference_provenance=@{source=(Join-Path $PSScriptRoot 'test_canonical_full_inner_runtime.m');
        position='[0;0;2]';velocity='zeros(3,1)';jerk='zeros(3,1)';acceleration='[.2*sin(k*.05);.1*cos(k*.04);0]';task_phase_mapping_proven=$false};
    actual_phase_advanced=$false;timestamps_or_plant_causality_proven=$false;hardware_actions=0;model_calls=0;solver_calls=0}
$report | ConvertTo-Json -Depth 8 | Out-File -LiteralPath $final -Encoding utf8
if(-not $report.pass){throw 'Continuous reference native regression failed.'}
