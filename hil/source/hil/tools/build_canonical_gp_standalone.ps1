param([Parameter(Mandatory=$true)][string]$OutputRoot)
$ErrorActionPreference='Stop'
$toolsRoot=(& (Join-Path $PSScriptRoot 'gpenmpc_install_path.ps1') 'llvm' 'bin')
$clang=Join-Path $toolsRoot 'clang.exe'
$ar=Join-Path $toolsRoot 'llvm-ar.exe'
$objdump=Join-Path $toolsRoot 'llvm-objdump.exe'
$matlabIncludes=(& (Join-Path $PSScriptRoot 'gpenmpc_install_path.ps1') 'matlab' 'extern\include')
$generation=Get-Content -LiteralPath (Join-Path $OutputRoot 'GENERATION_RESULT.json') -Raw | ConvertFrom-Json
if ($generation.status -ne 'PASS_HOST_ONLY_CANONICAL_GP256_STANDALONE_C_GENERATED') {throw 'Generation did not pass.'}
$generated=Join-Path $OutputRoot 'generated'
$artifactNames=@('RESULT.json','NATIVE_RESULT.json','NATIVE_TIMING.csv','NATIVE_ACTUAL_LE.bin','canonical_gp256_test.exe','libcanonical_gp256.a')
foreach($name in $artifactNames) {if(Test-Path -LiteralPath (Join-Path $OutputRoot $name)){throw "Do not overwrite existing $name"}}
$cSources=@(Get-ChildItem -LiteralPath $generated -Filter '*.c' -File)
if($cSources.Count -eq 0){throw 'No generated C found.'}
$headers=@(Get-ChildItem -LiteralPath $generated -Filter '*.h' -File)
$sourceRecords=@($cSources+$headers | ForEach-Object { [ordered]@{path=$_.FullName;bytes=$_.Length;sha256=(Get-FileHash -LiteralPath $_.FullName -Algorithm SHA256).Hash} })
# Check the recorded BLAS and MEX dependencies.
$suspect=@($cSources | Select-String -Pattern '\b(emlrt|mexCallMATLAB|mxArray|cblas_|dgemm|dtrsm|mkl_)')
if($suspect.Count -gt 0){throw ('Generated runtime/BLAS dependency needs review: '+($suspect | Out-String))}
$objectRoot=Join-Path $OutputRoot 'objects'
New-Item -Path $objectRoot -ItemType Directory -ErrorAction Stop | Out-Null
$commands=[Collections.Generic.List[object]]::new()
$compileLog=[Collections.Generic.List[string]]::new()
$objects=[Collections.Generic.List[string]]::new()
foreach($source in $cSources){
    $obj=Join-Path $objectRoot ($source.BaseName+'.o')
    $args=@('-std=c11','-O2','-ffp-contract=off','-fno-fast-math','-Wall','-Wextra','-Werror','-I',$generated,'-I',$matlabIncludes,'-c',$source.FullName,'-o',$obj)
    $messages=& $clang @args 2>&1
    $rc=$LASTEXITCODE
    $commands.Add([ordered]@{program=$clang;arguments=$args;exit_code=$rc})
    foreach($message in $messages){$compileLog.Add([string]$message)}
    if($rc -ne 0){$compileLog | Set-Content -LiteralPath (Join-Path $OutputRoot 'COMPILE_LOG.txt');throw "Generated C compile failed: $($source.Name)"}
    $objects.Add($obj)
}
$library=Join-Path $OutputRoot 'libcanonical_gp256.a'
$args=@('rcs',$library)+$objects.ToArray()
$messages=& $ar @args 2>&1;$rc=$LASTEXITCODE
$commands.Add([ordered]@{program=$ar;arguments=$args;exit_code=$rc})
foreach($message in $messages){$compileLog.Add([string]$message)}
if($rc -ne 0){throw 'Archive creation failed.'}
$binary=Join-Path $OutputRoot 'canonical_gp256_test.exe'
$harness=Join-Path $PSScriptRoot 'canonical_gp_standalone_test.c'
$args=@('-std=c11','-O2','-ffp-contract=off','-fno-fast-math','-Wall','-Wextra','-Werror','-static','-I',$generated,'-I',$matlabIncludes,$harness,$library,'-o',$binary)
$messages=& $clang @args 2>&1;$rc=$LASTEXITCODE
$commands.Add([ordered]@{program=$clang;arguments=$args;exit_code=$rc})
foreach($message in $messages){$compileLog.Add([string]$message)}
$compileLog | Set-Content -LiteralPath (Join-Path $OutputRoot 'COMPILE_LOG.txt')
if($rc -ne 0){throw 'Native harness link failed.'}
$imports=& $objdump '-p' $binary 2>&1
if($LASTEXITCODE -ne 0){throw 'Binary dependency inspection failed.'}
$imports | Set-Content -LiteralPath (Join-Path $OutputRoot 'BINARY_IMPORTS.txt')
$dependencies=@($imports | Select-String -Pattern 'DLL Name:\s*(.+)' | ForEach-Object {$_.Matches[0].Groups[1].Value.Trim()})
if(@($dependencies | Where-Object {$_ -match '(?i)(matlab|libmx|libmex|mkl|blas|lapack|emlrt)'}).Count){throw 'Unexpected MATLAB/BLAS runtime dependency.'}
$args=@((Join-Path $OutputRoot 'GP256_INPUT_EXPECTED_LE.bin'),(Join-Path $OutputRoot 'NATIVE_TIMING.csv'),(Join-Path $OutputRoot 'NATIVE_ACTUAL_LE.bin'),(Join-Path $OutputRoot 'NATIVE_RESULT.json'))
$execution=& $binary @args 2>&1;$nativeRc=$LASTEXITCODE
$execution | Set-Content -LiteralPath (Join-Path $OutputRoot 'NATIVE_LOG.txt')
$native=Get-Content -LiteralPath (Join-Path $OutputRoot 'NATIVE_RESULT.json') -Raw | ConvertFrom-Json
$timing=@(Import-Csv -LiteralPath (Join-Path $OutputRoot 'NATIVE_TIMING.csv'))
$orderedTimes=@($timing | ForEach-Object {[double]$_.native_predictor_s} | Sort-Object)
$unchanged=$true
foreach($id in $sourceRecords){if((Get-FileHash -LiteralPath $id.path -Algorithm SHA256).Hash -ne $id.sha256){$unchanged=$false}}
$pass=$nativeRc -eq 0 -and $native.pass -and $native.row_count -eq 293 -and $timing.Count -eq 293 -and $unchanged
$report=[ordered]@{status=if($pass){'PASS_HOST_STANDALONE_CANONICAL_GP256_LIBRARY'}else{'FAIL_HOST_STANDALONE_CANONICAL_GP256_LIBRARY'};pass=$pass;native_exit_code=$nativeRc;native=$native;median_s=$orderedTimes[146];p95_s=$orderedTimes[[math]::Ceiling(.95*293)-1];generated_c_translation_units=$cSources.Count;commands=$commands.ToArray();sources=$sourceRecords;source_hashes_unchanged=$unchanged;model_sha256=$generation.model_sha256;matlab_runtime_dependency=$false;binary_imported_dlls=$dependencies;library_path=$library;library_sha256=(Get-FileHash -LiteralPath $library -Algorithm SHA256).Hash;binary_path=$binary;binary_sha256=(Get-FileHash -LiteralPath $binary -Algorithm SHA256).Hash;harness_sha256=(Get-FileHash -LiteralPath $harness -Algorithm SHA256).Hash;generation_receipt_sha256=(Get-FileHash -LiteralPath (Join-Path $OutputRoot 'GENERATION_RESULT.json') -Algorithm SHA256).Hash;precision='double';gp_inducing_count=256;input_count=17;output_count=18;timing_is_native_predictor_only=$true;timing_is_board_wcet=$false;COM_UDP_actions=0;board_actions=0;solver_calls=0;plant_runs=0}
$divisionSource=Get-Content -LiteralPath (Join-Path $generated 'mldivide.c') -Raw
if($divisionSource -notmatch 'static double b_A\[65536\]'){throw 'Recheck generated reentrancy; scratch declaration changed.'}
$report['reentrant']=$false
$report['single_owner_serial_calls_required']=$true
$report['writable_static_scratch_bytes']=524288
$report['reentrancy_evidence']='generated/mldivide.c:26 static double b_A[65536]; concurrent inner/outer callers must not share this instance'
$report['matlab_compile_time_type_header']=[ordered]@{path=(Join-Path $matlabIncludes 'tmwtypes.h');sha256=(Get-FileHash -LiteralPath (Join-Path $matlabIncludes 'tmwtypes.h') -Algorithm SHA256).Hash;runtime_dependency=$false}
$report['generated_interface_examples_excluded_from_library']=$true
$report['generated_source_license']='MATLAB Coder Academic License; retain generated notices and academic-use restrictions'
$report['native_harness_serial_owner']='main thread only: initialize -> 16 serial warmups -> 293 serial timed calls -> terminate; no worker threads in this harness'
$report | ConvertTo-Json -Depth 20 | Set-Content -LiteralPath (Join-Path $OutputRoot 'RESULT.json')
$execution
if(-not $pass){throw 'Native result did not pass.'}
