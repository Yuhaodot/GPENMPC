param([Parameter(Mandatory=$true)][string]$OutputRoot,[string]$Attempt='native')
$ErrorActionPreference='Stop'
$toolRoot=(& (Join-Path $PSScriptRoot 'gpenmpc_install_path.ps1') 'llvm' 'bin')
$clang=Join-Path $toolRoot 'clang.exe'
$ar=Join-Path $toolRoot 'llvm-ar.exe'
$objdump=Join-Path $toolRoot 'llvm-objdump.exe'
$generated=Join-Path $OutputRoot 'generated'
$includes=(& (Join-Path $PSScriptRoot 'gpenmpc_install_path.ps1') 'matlab' 'extern\include')
$generation=Get-Content -LiteralPath (Join-Path $OutputRoot 'RESULT.json') -Raw | ConvertFrom-Json
if($generation.status -ne 'PASS_FIXED_ABI_MATLAB_AND_STANDALONE_C_GENERATION'){throw 'Actual C generation has not passed.'}
if($Attempt -notmatch '^native[a-z0-9_]*$'){throw 'Invalid local native attempt name.'}
$target=Join-Path $OutputRoot $Attempt
if(Test-Path -LiteralPath $target){throw 'Preserve previous native artifacts.'}
New-Item -ItemType Directory -Path $target | Out-Null
$sources=@(Get-ChildItem -LiteralPath $generated -Filter '*.c' -File)
$records=@(Get-ChildItem -LiteralPath $generated -File | Where-Object {$_.Extension -in '.c','.h'} | ForEach-Object {
    [ordered]@{path=$_.FullName;sha256=(Get-FileHash -LiteralPath $_.FullName -Algorithm SHA256).Hash;bytes=$_.Length}
})
$suspect=@($sources | Select-String -Pattern '\b(emlrt|mexCallMATLAB|mxArray|cblas_|dgemm|dtrsm|mkl_|malloc|calloc|omp_init_nest_lock|omp_get_max_threads|__m128d)\b')
if($suspect.Count){throw ('Review unexpected runtime/allocation dependency: '+($suspect|Out-String))}
$commands=[Collections.Generic.List[object]]::new()
$objects=[Collections.Generic.List[string]]::new()
$messages=[Collections.Generic.List[string]]::new()
foreach($source in $sources){
    $obj=Join-Path $target ($source.BaseName+'.o')
    $args=@('-std=c11','-O2','-ffp-contract=off','-fno-fast-math','-Wall','-Wextra','-Werror','-I',$generated,'-I',$includes,'-c',$source.FullName,'-o',$obj)
    $text=& $clang @args 2>&1;$rc=$LASTEXITCODE
    $commands.Add([ordered]@{program=$clang;arguments=$args;exit_code=$rc})
    foreach($line in $text){$messages.Add([string]$line)}
    if($rc -ne 0){$messages | Set-Content -LiteralPath (Join-Path $target 'COMPILE_LOG.txt');throw "Actual C compile failed: $($source.Name)"}
    $objects.Add($obj)
}
$library=Join-Path $target 'libcanonical_local_inner.a'
$args=@('rcs',$library)+$objects.ToArray();& $ar @args
if($LASTEXITCODE -ne 0){throw 'Actual archive failed.'}
$commands.Add([ordered]@{program=$ar;arguments=$args;exit_code=$LASTEXITCODE})
$exe=Join-Path $target 'canonical_local_inner_test.exe'
$harness=Join-Path $PSScriptRoot 'canonical_local_inner_standalone_test.c'
$args=@('-std=c11','-O2','-ffp-contract=off','-fno-fast-math','-Wall','-Wextra','-Werror','-static','-I',$generated,'-I',$includes,$harness,$library,'-o',$exe)
$text=& $clang @args 2>&1;$rc=$LASTEXITCODE
$commands.Add([ordered]@{program=$clang;arguments=$args;exit_code=$rc})
foreach($line in $text){$messages.Add([string]$line)}
$messages | Set-Content -LiteralPath (Join-Path $target 'COMPILE_LOG.txt')
if($rc -ne 0){throw 'Actual harness link failed.'}
$imports=& $objdump '-p' $exe 2>&1
if($LASTEXITCODE -ne 0){throw 'Import inspection failed.'}
$imports | Set-Content -LiteralPath (Join-Path $target 'IMPORTS.txt')
$deps=@($imports|Select-String -Pattern 'DLL Name:\s*(.+)'|ForEach-Object {$_.Matches[0].Groups[1].Value.Trim()})
if(@($deps|Where-Object {$_ -match '(?i)(matlab|libmx|libmex|mkl|blas|lapack|emlrt)'}).Count){throw 'Unexpected MATLAB/BLAS runtime dependency.'}
$args=@((Join-Path $OutputRoot 'FIXED_INPUTS.bin'),(Join-Path $target 'ACTUAL_OUTPUTS.bin'),(Join-Path $target 'NUMERICAL_RESULT.json'))
$result=& $exe @args 2>&1;$rc=$LASTEXITCODE
$result | Set-Content -LiteralPath (Join-Path $target 'EXECUTION_LOG.txt')
$native=Get-Content -LiteralPath (Join-Path $target 'NUMERICAL_RESULT.json') -Raw | ConvertFrom-Json
$unchanged=$true
foreach($r in $records){if((Get-FileHash -LiteralPath $r.path -Algorithm SHA256).Hash -ne $r.sha256){$unchanged=$false}}
$pass=$rc -eq 0 -and $native.pass -and $native.rows -eq 60 -and $unchanged
$report=[ordered]@{status=if($pass){'PASS_ACTUAL_HOST_LOCAL_INNER_GENERATED_C'}else{'FAIL_ACTUAL_HOST_LOCAL_INNER_GENERATED_C'};pass=$pass;native=$native;native_exit_code=$rc;commands=$commands.ToArray();sources=$records;generated_sources_unchanged=$unchanged;binary_imports=$deps;matlab_runtime_dependency=$false;dynamic_allocation_dependency=$false;library=$library;library_sha256=(Get-FileHash -LiteralPath $library -Algorithm SHA256).Hash;exe_sha256=(Get-FileHash -LiteralPath $exe -Algorithm SHA256).Hash;harness_sha256=(Get-FileHash -LiteralPath $harness -Algorithm SHA256).Hash;fixture_sha256=(Get-FileHash -LiteralPath (Join-Path $OutputRoot 'FIXED_INPUTS.bin') -Algorithm SHA256).Hash;precision='double';state_chain='actual generated C candidate post-state';gp_prediction_input='Archived HOST-computed GP predictions provide the pending inputs.';source_tags='Recorded uint64 source timestamp and generation.';publication_authority=$false;COM_UDP_calls=0;plant_runs=0;solver_calls=0;C_source_license='MATLAB Coder Academic License; preserve original generated notices'}
$report['comparison_tolerance_before_run']='Numerical comparison: abs(actual-expected)/max(1,abs(expected)) <= 1e-10.'
$report['official16_scope']='HIL16CtrlsNorm numerical mapping of actual61 and archived61.'
$report['callable_abi_preconditions']='trusted typed finite original mathematical inputs only; real owner must enforce all source/identity/expiry/commit guards separately'
$report['excluded_generated_subdirectories']=@('interface','examples')
$report['tmwtypes_is_compile_time_header_not_runtime']=(Join-Path $includes 'tmwtypes.h')
$build=Split-Path -Parent $PSScriptRoot
$overlayRecords=@()
foreach($pair in @(@('dynamics','gpenmpcM600Allocation.m'),@('enmpc','gpenmpcUpdateGpAgreementWeight.m'))){
    $original=Join-Path (Join-Path (Join-Path (Join-Path (Split-Path (Split-Path (Split-Path $PSScriptRoot -Parent) -Parent) -Parent) 'assets\canonical') 'method_source\matlab') $pair[0]) $pair[1]
    $overlay=Join-Path (Join-Path $PSScriptRoot 'codegen_compat') $pair[1]
    $overlayRecords+=[ordered]@{canonical_original=$original;canonical_sha256=(Get-FileHash -LiteralPath $original -Algorithm SHA256).Hash;derived_overlay=$overlay;derived_sha256=(Get-FileHash -LiteralPath $overlay -Algorithm SHA256).Hash;purpose='Fixed-shape struct declarations for C generation'}
}
$report['codegen_shape_overlays']=$overlayRecords
$report['numeric_mapping_source']=[ordered]@{path=(Join-Path $build 'host_runtime\+gpenmpcNative\canonicalRotorToHIL16CtrlsNorm.m');sha256=(Get-FileHash -LiteralPath (Join-Path $build 'host_runtime\+gpenmpcNative\canonicalRotorToHIL16CtrlsNorm.m') -Algorithm SHA256).Hash}
$report | ConvertTo-Json -Depth 20 | Set-Content -LiteralPath (Join-Path $target 'RESULT.json')
$result
if(-not $pass){throw 'Actual C numerical comparison failed.'}
