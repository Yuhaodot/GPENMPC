param([Parameter(Mandatory=$true)][string]$OutputRoot)
$ErrorActionPreference='Stop'
$reviewedSource=(& (Join-Path $PSScriptRoot 'gpenmpc_external_path.ps1') 'px4_mavlink_main_source')
$build=Split-Path -Parent $PSScriptRoot
$source=Join-Path $build 'rfly_vendor_integration\application_integration\overlay\src\modules\mavlink\mavlink_main.cpp'
$harness=Join-Path $PSScriptRoot 'test_local_stream_command.cpp'
$compiler=(& (Join-Path $PSScriptRoot 'gpenmpc_install_path.ps1') 'llvm' 'bin\clang++.exe')
$expected='F47D8A6D41CE8188D84D03C322FB90821038EFADB3415674DD3316EB36B86AF3'
$before=(Get-FileHash -LiteralPath $source -Algorithm SHA256).Hash
$reviewedBefore=(Get-FileHash -LiteralPath $reviewedSource -Algorithm SHA256).Hash
if($reviewedBefore -ne $expected){throw 'Command source differs from its expected identity'}
if(Test-Path -LiteralPath $OutputRoot){throw 'Choose an unused output path.'}
$text=[IO.File]::ReadAllText($source)
$pattern='(?s)\bint\r?\nMavlink::stream_command\(int argc, char \*argv\[\]\)\r?\n\{.*?\r?\n\}(?=\r?\n\r?\nvoid\r?\nMavlink::set_boot_complete)'
$matches=[regex]::Matches($text,$pattern)
if($matches.Count -ne 1){throw 'Unique exact actual function boundaries not found'}
$reviewedMatches=[regex]::Matches([IO.File]::ReadAllText($reviewedSource),$pattern)
if($reviewedMatches.Count -ne 1 -or $matches[0].Value -cne $reviewedMatches[0].Value){throw 'The build-overlay function must match the reviewed application source.'}
New-Item -ItemType Directory -Path $OutputRoot | Out-Null
$extracted=Join-Path $OutputRoot 'actual_stream_command.inc'
# Mechanical source extraction only: no function-body rewriting.
[IO.File]::WriteAllText($extracted,$matches[0].Value,(New-Object Text.UTF8Encoding($false)))
$exe=Join-Path $OutputRoot 'stream_command.exe'
$arguments=@('-std=c++14','-O2','-Wall','-Wextra','-Werror','-static','-DMAVLINK_UDP=1','-I',$OutputRoot,$harness,'-o',$exe)
$compile=& $compiler @arguments 2>&1;$compileRc=$LASTEXITCODE
$compile|Set-Content -LiteralPath (Join-Path $OutputRoot 'COMPILE_LOG.txt')
$runRc=$null;$run=$null;$checks=$null
if($compileRc -eq 0){
    $run=& $exe 2>&1;$runRc=$LASTEXITCODE
    $run|Set-Content -LiteralPath (Join-Path $OutputRoot 'EXECUTION_LOG.txt')
    if($runRc -eq 0){$checks=($run -join "`n")|ConvertFrom-Json}
}
$after=(Get-FileHash -LiteralPath $source -Algorithm SHA256).Hash
$reviewedAfter=(Get-FileHash -LiteralPath $reviewedSource -Algorithm SHA256).Hash
$result=[ordered]@{scope='HOST_ACTUAL_STREAM_COMMAND_EXTRACTION_ONLY';pass=($compileRc -eq 0 -and $runRc -eq 0 -and $checks.pass -and $before -eq $after -and $reviewedBefore -eq $reviewedAfter);
 source=$source;source_sha256=$before;source_unchanged=($before -eq $after);extracted_sha256=(Get-FileHash -LiteralPath $extracted).Hash;
 reviewed_source=$reviewedSource;reviewed_source_sha256=$reviewedBefore;reviewed_source_unchanged=($reviewedBefore -eq $reviewedAfter);overlay_function_exact_equal=$true;
 harness_sha256=(Get-FileHash -LiteralPath $harness).Hash;compiler=$compiler;compile_arguments=$arguments;compile_exit_code=$compileRc;run_exit_code=$runRc;result=$checks;
 boundary='ACTUAL_CLI_FUNCTION_WITH_STUB_INSTANCE_LOOKUP_AND_CONFIGURE_RECORDING_NOT_FIRMWARE_OR_STREAM_RUNTIME';
 stream_selection='Explicit /dev/ttyACM0 GPENMPC_LOCAL_WIRE -1 drain request';
 COM=0;board=0;firmware_build=0;network=0}
$result|ConvertTo-Json -Depth 12|Set-Content -LiteralPath (Join-Path $OutputRoot 'RESULT.json')
$result|ConvertTo-Json -Depth 12
if(-not $result.pass){exit 1}
