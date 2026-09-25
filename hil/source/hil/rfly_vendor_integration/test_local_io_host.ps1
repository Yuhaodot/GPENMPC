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
$source=Join-Path $vi 'px4_runtime\Px4CanonicalLocalIo.cpp';$test=Join-Path $vi 'test_local_io_host.cpp'
$tracked=@($source,(Join-Path $vi 'px4_runtime\Px4CanonicalLocalIo.hpp'),(Join-Path $vi 'CanonicalFullInnerConsumption.hpp'),(Join-Path $vi 'CanonicalLocalInnerInputBuilder.hpp'),
 $test,$PSCommandPath,$facade,$archive,$rwi,$sjc,$gp)+@(Get-ChildItem -LiteralPath (Join-Path $vi 'px4_wire\pump_host_stub') -Recurse -File|ForEach-Object FullName)
$before=@(Get-FileHash -LiteralPath $tracked -Algorithm SHA256|Select-Object Path,Hash)
$flags=@('-std=c++14','-O2','-Wall','-Wextra','-Werror','-ffp-contract=off','-fno-fast-math','-I',(Join-Path $vi 'px4_wire\pump_host_stub'),
 '-I',$actual)
$obj=Join-Path $out 'actual_local_io.o';$exe=Join-Path $out 'test.exe'
$compileArgs=$flags+@('-Dgpenmpc_full_inner_commit=gpenmpc_test_call_full_inner_commit','-Dgpenmpc_full_inner_copy_state=gpenmpc_test_call_full_inner_copy_state','-c',$source,'-o',$obj)
$compile=@(& $cc @compileArgs 2>&1|ForEach-Object {"$_"});$ce=$LASTEXITCODE
$linkArgs=$flags+@('-static','-municode',$test,$obj,$facade,$archive,'-o',$exe)
$link=@();$le=-1;$run=@();$rc=-1;$result=$null
if($ce -eq 0){$link=@(& $cc @linkArgs 2>&1|ForEach-Object {"$_"});$le=$LASTEXITCODE
 if($le -eq 0){$run=@(& $exe $rwi $sjc $gp (Get-FileHash -LiteralPath $sjc -Algorithm SHA256).Hash 2>&1|ForEach-Object {"$_"});$rc=$LASTEXITCODE
  $json=@($run|Where-Object {$_ -like '{*'});if($json.Count -eq 1){$result=$json[0]|ConvertFrom-Json}}}
$after=@(Get-FileHash -LiteralPath $tracked -Algorithm SHA256|Select-Object Path,Hash);$stable=($before|ConvertTo-Json -Compress) -ceq ($after|ConvertTo-Json -Compress)
$receipt=[ordered]@{scope='ACTUAL_LOCAL_IO_CPP_REAL_PRIVATE74_GP_EXPLICIT_MOCK_BROKER_HRT_AUTHORITY';compile_command=$compileArgs;compile_exit=$ce;compile_output=$compile;
 link_command=$linkArgs;link_exit=$le;link_output=$link;run_exit=$rc;run_output=$run;result=$result;source_before=$before;source_stable=$stable;
 boundary='Offline fixtures for clocks, uORB, output permissions and publication.';
 COM=0}
[IO.File]::WriteAllText((Join-Path $out 'RESULT.json'),($receipt|ConvertTo-Json -Depth 12)+"`n")
$compile;$link;$run
if($ce -ne 0 -or $le -ne 0 -or $rc -ne 0 -or !$stable){exit 1}
