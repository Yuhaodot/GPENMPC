param(
 [Parameter(Mandatory=$true)][string]$CompilerPath,
 [Parameter(Mandatory=$true)][string]$OutputDirectory,
 [Parameter(Mandatory=$true)][string]$Px4BuildDirectory,
 [string]$SourceRoot=$env:GPENMPC_SOURCE_ROOT
)
$ErrorActionPreference='Stop'
. (Join-Path (Split-Path $PSScriptRoot -Parent) 'host_test_paths.ps1')
if (!$SourceRoot) { $SourceRoot=Split-Path (Split-Path $PSScriptRoot -Parent) -Parent }
$build=Resolve-GpenmpcTestInput $SourceRoot 'SourceRoot' -Directory
$vi=Join-Path $build 'rfly_vendor_integration';$here=Join-Path $vi 'local_source_ingress'
$compiler=Resolve-GpenmpcTestInput $CompilerPath 'CompilerPath'
$generated=Resolve-GpenmpcTestInput $Px4BuildDirectory 'Px4BuildDirectory' -Directory
$mavlink=Join-Path $generated 'mavlink';$topic=$generated
$out=New-GpenmpcTestOutput $OutputDirectory @($build,$generated,(Split-Path $compiler -Parent))
$files=@('Px4OriginalHilReceiptReader.hpp','Px4OriginalHilReceiptReader.cpp','Px4OriginalHilReceipt.hpp','GPENMPCOriginalHilReceipt.msg','test_original_hil_reader.cpp','test_original_hil_reader.ps1')|ForEach-Object {Join-Path $here $_}
$files+=@((Join-Path $generated 'uORB\topics\gpenmpc_original_hil_receipt.h'),(Join-Path $vi 'clock_tap_overlay\ExactSourceReceiptLookup.hpp'),(Join-Path $vi 'clock_tap_overlay\ClockObservationTap.hpp'),(Join-Path $topic 'uORB\topics\vehicle_odometry.h'))
$before=@(Get-FileHash -LiteralPath $files -Algorithm SHA256|Select-Object Path,Hash)
$exe=Join-Path $out 'test.exe'
$args=@('-std=c++14','-O2','-Wall','-Wextra','-Werror','-Wno-address-of-packed-member','-static',
 '-I',(Join-Path $vi 'px4_wire\pump_host_stub'),'-I',$generated,'-I',$topic,'-isystem',$mavlink,
 (Join-Path $here 'Px4OriginalHilReceiptReader.cpp'),(Join-Path $here 'test_original_hil_reader.cpp'),'-o',$exe)
$compile=@(& $compiler @args 2>&1|ForEach-Object {"$_"});$ce=$LASTEXITCODE;$run=@();$rc=-1;$result=$null
if($ce -eq 0){$run=@(& $exe 2>&1|ForEach-Object {"$_"});$rc=$LASTEXITCODE;$json=@($run|Where-Object {$_ -like '{*'});if($json.Count -eq 1){$result=$json[0]|ConvertFrom-Json}}
$after=@(Get-FileHash -LiteralPath $files -Algorithm SHA256|Select-Object Path,Hash);$stable=($before|ConvertTo-Json -Compress) -ceq ($after|ConvertTo-Json -Compress)
$receipt=[ordered]@{scope='ACTUAL_READER_CPP_REAL_MAVLINK_PRODUCER_PRIVATE_SNAPSHOT_MOCK_UORB';command=$args;compile_exit=$ce;compile_output=$compile;run_exit=$rc;run_output=$run;result=$result;input_sha256=$before;source_stable=$stable;
 boundary='Host uORB, source and HRT fixtures exercise the reader and MAVLink producer.';COM=0;application_modified=$false}
[IO.File]::WriteAllText((Join-Path $out 'RESULT.json'),($receipt|ConvertTo-Json -Depth 12)+"`n")
$compile;$run
if($ce -ne 0 -or $rc -ne 0 -or !$stable){exit 1}
