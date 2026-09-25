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
$r=Join-Path $build 'rfly_vendor_integration'
$cpp=Resolve-GpenmpcTestInput $CompilerPath 'CompilerPath'
$actual=Resolve-GpenmpcTestInput $Px4BuildDirectory 'Px4BuildDirectory' -Directory
$fixture=Resolve-GpenmpcTestInput (Join-Path (Resolve-GpenmpcTestInput $ReferenceFixtures 'ReferenceFixtures' -Directory) 'CONTINUOUS_REFERENCE_FIXTURE.bin') 'reference fixture'
$raw=Resolve-GpenmpcTestInput $GpChainFixture 'GpChainFixture'
$gp=Resolve-GpenmpcTestInput $GpLibrary 'GpLibrary'
$facade=Resolve-GpenmpcTestInput $FacadeObject 'FacadeObject'
$archive=Resolve-GpenmpcTestInput $KernelArchive 'KernelArchive'
$out=New-GpenmpcTestOutput $OutputDirectory @($build,$actual,(Split-Path $cpp -Parent))
$source=Join-Path $r 'test_local_gp_wire.cpp'
$tracked=@($source,$PSCommandPath,(Join-Path $r 'px4_wire\CanonicalLocalGpWire.hpp'),
 (Join-Path $r 'px4_wire\RflySnapshotWireTypes.hpp'),$archive,$facade,$fixture,$raw,$gp)
$before=@(Get-FileHash -LiteralPath $tracked -Algorithm SHA256 | Select-Object Path,Hash)
$exe=Join-Path $out 'test.exe'
$args=@('-std=c++14','-O2','-Wall','-Wextra','-Werror','-ffp-contract=off','-fno-fast-math',
 '-Wno-address-of-packed-member','-isystem',(Join-Path $actual 'mavlink'),
 '-static','-municode',$source,$facade,$archive,'-o',$exe)
$co=@(& $cpp @args 2>&1 | ForEach-Object {"$_"});$cc=$LASTEXITCODE
$ro=@();$rc=-1;$parsed=$null
if($cc -eq 0){$ro=@(& $exe $fixture $raw $gp 2>&1 | ForEach-Object {"$_"});$rc=$LASTEXITCODE
 if($rc -eq 0){$parsed=($ro -join "`n")|ConvertFrom-Json}}
$after=@(Get-FileHash -LiteralPath $tracked -Algorithm SHA256 | Select-Object Path,Hash)
$stable=($before|ConvertTo-Json -Compress) -ceq ($after|ConvertTo-Json -Compress)
$result=[ordered]@{scope='ORIGINAL_FULL_C_AND_GP256_OVER_REAL_MAVLINK_CODEC_HOST_ONLY';
 command=$args;compile_exit=$cc;compile_output=$co;run_exit=$rc;run_output=$ro;result=$parsed;
 before=$before;after=$after;source_stable=$stable;COM=0;board=0;application_selected=$false}
[IO.File]::WriteAllText((Join-Path $out 'RESULT.json'),($result|ConvertTo-Json -Depth 10)+"`n")
$co;$ro
if($cc -ne 0 -or $rc -ne 0 -or !$stable){exit 1}
