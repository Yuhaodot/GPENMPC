param([Parameter(Mandatory=$true)][string]$OutputRoot)
$ErrorActionPreference='Stop'
$toolchain=(& (Join-Path $PSScriptRoot 'gpenmpc_install_path.ps1') 'llvm' 'bin')
$cc=Join-Path $toolchain 'clang.exe'
$objdump=Join-Path $toolchain 'llvm-objdump.exe'
$generated=Join-Path $OutputRoot 'generated'
$library=Join-Path $OutputRoot 'libcanonical_gp256.a'
$parent=Get-Content -LiteralPath (Join-Path $OutputRoot 'RESULT.json') -Raw | ConvertFrom-Json
if(-not $parent.pass -or (Get-FileHash -LiteralPath $library -Algorithm SHA256).Hash -ne $parent.library_sha256){throw 'Actual tested library differs.'}
$dll=Join-Path $OutputRoot 'canonical_gp256.dll'
$exe=Join-Path $OutputRoot 'canonical_gp256_dll_test.exe'
foreach($file in @($dll,$exe,(Join-Path $OutputRoot 'DLL_RESULT.json'))){if(Test-Path -LiteralPath $file){throw "Preserve existing $file"}}
$api=Join-Path $PSScriptRoot 'canonical_gp_standalone_api.c'
$apiHeader=Join-Path $PSScriptRoot 'canonical_gp_standalone_api.h'
$test=Join-Path $PSScriptRoot 'canonical_gp_standalone_dll_test.c'
$args=@('-std=c11','-O2','-ffp-contract=off','-fno-fast-math','-Wall','-Wextra','-Werror','-static','-shared','-Wl,--exclude-all-symbols','-I',$generated,'-I',(& (Join-Path $PSScriptRoot 'gpenmpc_install_path.ps1') 'matlab' 'extern\include'),$api,$library,'-o',$dll)
$output=& $cc @args 2>&1;$rc=$LASTEXITCODE
$output | Set-Content -LiteralPath (Join-Path $OutputRoot 'DLL_COMPILE_LOG.txt')
if($rc -ne 0){throw 'DLL compile failed.'}
$commands=@([ordered]@{program=$cc;arguments=$args;exit_code=$rc})
$args=@('-std=c11','-O2','-ffp-contract=off','-fno-fast-math','-Wall','-Wextra','-Werror','-static',$test,'-o',$exe)
$output=& $cc @args 2>&1;$rc=$LASTEXITCODE
$output | Add-Content -LiteralPath (Join-Path $OutputRoot 'DLL_COMPILE_LOG.txt')
if($rc -ne 0){throw 'DLL harness compile failed.'}
$commands+=([ordered]@{program=$cc;arguments=$args;exit_code=$rc})
$inspection=& $objdump '-p' $dll 2>&1
if($LASTEXITCODE -ne 0){throw 'DLL inspection failed.'}
$inspection | Set-Content -LiteralPath (Join-Path $OutputRoot 'DLL_IMPORT_EXPORT.txt')
$imports=@($inspection | Select-String -CaseSensitive 'DLL Name:\s*(.+)' | ForEach-Object {$_.Matches[0].Groups[1].Value.Trim()})
if(@($imports|Where-Object{$_ -match '(?i)(matlab|libmx|libmex|mkl|blas|lapack)'}).Count){throw 'Unexpected runtime dependency.'}
if(($inspection | Out-String) -match 'gpenmpcNative_canonicalSparseGpFixedInput'){throw 'Unguarded core was exported.'}
$output=& $exe (Join-Path $OutputRoot 'GP256_INPUT_EXPECTED_LE.bin') $dll (Join-Path $OutputRoot 'DLL_CONTENTION.csv') (Join-Path $OutputRoot 'DLL_NATIVE_RESULT.json') 2>&1
$rc=$LASTEXITCODE;$output | Set-Content -LiteralPath (Join-Path $OutputRoot 'DLL_NATIVE_LOG.txt')
$native=Get-Content -LiteralPath (Join-Path $OutputRoot 'DLL_NATIVE_RESULT.json') -Raw | ConvertFrom-Json
$libraryUnchanged=(Get-FileHash -LiteralPath $library -Algorithm SHA256).Hash -eq $parent.library_sha256
$report=[ordered]@{status=if($rc -eq 0 -and $native.pass -and $libraryUnchanged){'PASS_FIXED_GP256_NONBLOCKING_SINGLE_OWNER_DLL'}else{'FAIL_FIXED_GP256_NONBLOCKING_SINGLE_OWNER_DLL'};native_exit_code=$rc;native=$native;commands=$commands;imported_dlls=$imports;library_unchanged=$libraryUnchanged;library_sha256=$parent.library_sha256;dll_sha256=(Get-FileHash -LiteralPath $dll -Algorithm SHA256).Hash;api_source_sha256=(Get-FileHash -LiteralPath $api -Algorithm SHA256).Hash;api_header_sha256=(Get-FileHash -LiteralPath $apiHeader -Algorithm SHA256).Hash;test_source_sha256=(Get-FileHash -LiteralPath $test -Algorithm SHA256).Hash;model_is_compile_constant=$true;argument_shape='input17 double -> output18 double; explicit return status';concurrent_calls_rejected_not_queued=$true;realtime_schedule_or_causal_association_proved=$false;board_actions=0;MATLAB_runtime=$false}
$report | ConvertTo-Json -Depth 12 | Set-Content -LiteralPath (Join-Path $OutputRoot 'DLL_RESULT.json')
$output
if($report.status -notlike 'PASS_*'){throw 'DLL regression failed.'}
