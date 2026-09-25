param(
 [Parameter(Mandatory=$true)][string]$CompilerPath,
 [Parameter(Mandatory=$true)][string]$OutputDirectory,
 [string]$SourceRoot=$env:GPENMPC_SOURCE_ROOT,
 [Parameter(Mandatory=$true)][string]$Px4BuildDirectory,
 [string]$ReferenceFixtures=$env:GPENMPC_REFERENCE_FIXTURES,
 [string]$GpChainFixture=$env:GPENMPC_GP_CHAIN_FIXTURE,
 [Parameter(Mandatory=$true)][string]$GpLibrary,
 [Parameter(Mandatory=$true)][string]$FacadeObject,
 [string]$KernelArchive=$env:GPENMPC_KERNEL_ARCHIVE
)
$ErrorActionPreference='Stop'
. (Join-Path $PSScriptRoot 'host_test_paths.ps1')
if (!$SourceRoot) { $SourceRoot=Split-Path $PSScriptRoot -Parent }
$build=Resolve-GpenmpcTestInput $SourceRoot 'SourceRoot' -Directory
$vi=Join-Path $build 'rfly_vendor_integration'
$cc=Resolve-GpenmpcTestInput $CompilerPath 'CompilerPath'
$actual=Resolve-GpenmpcTestInput $Px4BuildDirectory 'Px4BuildDirectory' -Directory
$rwi=Resolve-GpenmpcTestInput (Join-Path (Resolve-GpenmpcTestInput $ReferenceFixtures 'ReferenceFixtures' -Directory) 'CONTINUOUS_REFERENCE_FIXTURE.bin') 'reference fixture'
$sjc=Resolve-GpenmpcTestInput $GpChainFixture 'GpChainFixture'
$gp=Resolve-GpenmpcTestInput $GpLibrary 'GpLibrary'
$facade=Resolve-GpenmpcTestInput $FacadeObject 'FacadeObject'
$archive=Resolve-GpenmpcTestInput $KernelArchive 'KernelArchive'
$out=New-GpenmpcTestOutput $OutputDirectory @($build,$actual,(Split-Path $cc -Parent))
$source=Join-Path $vi 'test_full_inner_consumption.cpp';$header=Join-Path $vi 'CanonicalFullInnerConsumption.hpp';$exe=Join-Path $out 'test.exe'
$tracked=@($source,$header,$PSCommandPath,$rwi,$sjc,$gp,$facade,$archive,(Join-Path $vi 'full_inner_abi\CanonicalFullInnerAbi.h'),(Join-Path $vi 'CanonicalLocalInnerInputBuilder.hpp'))
$before=@(Get-FileHash -LiteralPath $tracked -Algorithm SHA256|Select-Object Path,Hash)
$args=@('-std=c++14','-O2','-Wall','-Wextra','-Werror','-ffp-contract=off','-fno-fast-math','-static','-municode',
 '-I',(Join-Path $vi 'px4_wire/pump_host_stub'),'-I',$actual,
 $source,$facade,$archive,'-o',$exe)
$compile=@(& $cc @args 2>&1|ForEach-Object {"$_"});$ce=$LASTEXITCODE;$run=@();$rc=-1;$result=$null
if($ce -eq 0){$run=@(& $exe $rwi $sjc $gp (Get-FileHash -LiteralPath $sjc -Algorithm SHA256).Hash 2>&1|ForEach-Object {"$_"});$rc=$LASTEXITCODE
 $json=@($run|Where-Object {$_ -like '{*'});if($json.Count -eq 1){$result=$json[0]|ConvertFrom-Json}}
$after=@(Get-FileHash -LiteralPath $tracked -Algorithm SHA256|Select-Object Path,Hash)
$stable=($before|ConvertTo-Json -Compress) -ceq ($after|ConvertTo-Json -Compress)
$receipt=[ordered]@{scope='HOST_ACTUAL_PRIVATE74_FULL_INNER_RFL2_CONSUMPTION_AND_ORIGINAL_GP';command=$args;compile_exit=$ce;compile_output=$compile;run_exit=$rc;run_output=$run;result=$result;
 input_sha256=$before;source_stable=$stable;actual_topic_header=(Join-Path $actual 'uORB/topics/vehicle_odometry.h');
 boundary='Offline fixtures for clocks, uORB, output permissions and publication.';COM=0}
[IO.File]::WriteAllText((Join-Path $out 'RESULT.json'),($receipt|ConvertTo-Json -Depth 12)+"`n")
$compile;$run
if($ce -ne 0 -or $rc -ne 0 -or !$stable){exit 1}
