param([Parameter(Mandatory=$true)][string]$OutputDirectory)
# Run the native GP Core test binary and retain build and fixture identities.
$ErrorActionPreference = 'Stop'
$taskBuild = Split-Path -Parent $PSScriptRoot
$taskOutput = [IO.Path]::GetFullPath($OutputDirectory)
if (!(Test-Path -LiteralPath $taskOutput -PathType Container)) { throw 'Existing isolated output directory required.' }
$taskResult = Join-Path $taskOutput 'CORE_TEST_RESULT.json'
if (Test-Path -LiteralPath $taskResult) { throw 'Preserve previous test receipt.' }
$taskClang = (& (Join-Path $PSScriptRoot 'gpenmpc_install_path.ps1') 'llvm' 'bin\clang++.exe')
$taskSource = Join-Path $PSScriptRoot 'gpenmpc_native_raw_gp_sfcn.cpp'
$taskHeader = Join-Path $PSScriptRoot 'gpenmpc_native_raw_gp_sfcn_standalone_test.hpp'
$taskLibrary = Join-Path $taskBuild 'evidence\gp_predictor\libcanonical_gp256.a'
$taskPairs = Join-Path $taskBuild 'rfly_vendor_integration\full_inner_abi\snapshot_wire_fixture\RGP1_RGR1_PAIRS.bin'
$taskObject = Join-Path $taskOutput 'raw_gp_original_api.o'
$taskExe = Join-Path $taskOutput 'raw_gp_core_test.exe'
$taskBuildText = & $taskClang -std=c++14 -O2 -ffp-contract=off -fno-fast-math -static -municode -Wall -Wextra -Werror -Wno-address-of-packed-member -DGPENMPC_NATIVE_RAW_GP_STANDALONE_TEST $taskSource $taskObject $taskLibrary -o $taskExe 2>&1
$taskCompileExit = $LASTEXITCODE
$taskRunText = @();$taskRunExit = $null
if ($taskCompileExit -eq 0) { $taskRunText = & $taskExe $taskPairs 2>&1;$taskRunExit = $LASTEXITCODE }
$taskChecks = @($taskRunText | Where-Object { "$_" -match '^(PASS|FAIL) ' } | ForEach-Object { [ordered]@{name="$_".Substring(5);pass="$_".StartsWith('PASS ')} })
$taskEntries = @($taskSource,$taskHeader,$taskLibrary,$taskPairs,$taskObject,$taskExe) | ForEach-Object { $file=Get-Item -LiteralPath $_;[ordered]@{path=$file.FullName;bytes=$file.Length;sha256=(Get-FileHash -LiteralPath $_ -Algorithm SHA256).Hash} }
$taskReceipt = [ordered]@{
 scope='SAME_COMPILED_CORE_NO_MATLAB_NO_SOCKET_NO_BOARD';compile_exit=$taskCompileExit;run_exit=$taskRunExit
 checks=$taskChecks;total=$taskChecks.Count;passed=@($taskChecks|Where-Object {$_.pass}).Count
 all_pass=($taskCompileExit -eq 0 -and $taskRunExit -eq 0 -and $taskChecks.Count -gt 0 -and @($taskChecks|Where-Object {!$_.pass}).Count -eq 0)
 compiler_log=($taskBuildText -join "`n");raw_output=($taskRunText -join "`n");entries=$taskEntries
 MATLAB=0;COM=0;board=0;sockets=0;plant=0;controller=0;retained_fixture_queries=59;live_admission_proven=$false
}
[IO.File]::WriteAllText($taskResult,($taskReceipt|ConvertTo-Json -Depth 8),[Text.UTF8Encoding]::new($false))
$taskReceipt|ConvertTo-Json -Depth 8
if (!$taskReceipt.all_pass) { throw 'Same-Core HOST tests did not pass.' }
