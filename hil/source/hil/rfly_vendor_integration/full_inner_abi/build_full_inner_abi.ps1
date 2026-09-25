param(
 [Parameter(Mandatory=$true)][string]$CompilerPath,
 [Parameter(Mandatory=$true)][string]$OutputDirectory,
 [Parameter(Mandatory=$true)][string]$GeneratedDirectory,
 [Parameter(Mandatory=$true)][string]$MatlabIncludeDirectory,
 [string]$SourceRoot=$env:GPENMPC_SOURCE_ROOT,
 [string]$NativeObjectDirectory,
 [string]$KernelArchive=$env:GPENMPC_KERNEL_ARCHIVE,
 [string]$ReferenceFixtures=$env:GPENMPC_REFERENCE_FIXTURES,
 [string]$GpChainFixture=$env:GPENMPC_GP_CHAIN_FIXTURE,
 [string]$GpLibrary,
 [switch]$WithClosedEvidence,[switch]$WithLearningAudit,[switch]$FacadeOnly
)
$ErrorActionPreference='Stop'
. (Join-Path (Split-Path $PSScriptRoot -Parent) 'host_test_paths.ps1')
if (!$SourceRoot) { $SourceRoot=Split-Path (Split-Path $PSScriptRoot -Parent) -Parent }
$build=Resolve-GpenmpcTestInput $SourceRoot 'SourceRoot' -Directory
$integration=Join-Path $build 'rfly_vendor_integration';$abiRoot=Join-Path $integration 'full_inner_abi'
$cpp=Resolve-GpenmpcTestInput $CompilerPath 'CompilerPath'
$toolBin=Split-Path $cpp -Parent
$cc=Resolve-GpenmpcTestInput (Join-Path $toolBin 'clang.exe') 'C compiler'
$generated=Resolve-GpenmpcTestInput $GeneratedDirectory 'GeneratedDirectory' -Directory
$matlabInclude=Resolve-GpenmpcTestInput $MatlabIncludeDirectory 'MatlabIncludeDirectory' -Directory
if($WithLearningAudit){$WithClosedEvidence=$true}
if($FacadeOnly -or !$WithClosedEvidence){$archive=Resolve-GpenmpcTestInput $KernelArchive 'KernelArchive'}
else {
 $native=Resolve-GpenmpcTestInput $NativeObjectDirectory 'NativeObjectDirectory' -Directory
 $null=Resolve-GpenmpcTestInput (Join-Path $native 'RESULT.json') 'native compilation report'
 $null=Resolve-GpenmpcTestInput (Join-Path $toolBin 'llvm-objcopy.exe') 'llvm-objcopy'
 $null=Resolve-GpenmpcTestInput (Join-Path $toolBin 'llvm-ar.exe') 'llvm-ar'
}
if(!$FacadeOnly){
 $rwi=Resolve-GpenmpcTestInput (Join-Path (Resolve-GpenmpcTestInput $ReferenceFixtures 'ReferenceFixtures' -Directory) 'CONTINUOUS_REFERENCE_FIXTURE.bin') 'reference fixture'
 $sjc=Resolve-GpenmpcTestInput $GpChainFixture 'GpChainFixture'
 $gp=Resolve-GpenmpcTestInput $GpLibrary 'GpLibrary'
}
$out=New-GpenmpcTestOutput $OutputDirectory @($build,$generated,$native,$toolBin,$matlabInclude)
function HashSet([string]$Domain,[object[]]$Files){
 $data=[Collections.Generic.List[byte]]::new();$data.AddRange([Text.Encoding]::UTF8.GetBytes($Domain+[char]0))
 foreach($f in $Files){$data.AddRange([Text.Encoding]::UTF8.GetBytes($f.Name+[char]0));$data.AddRange([Convert]::FromHexString((Get-FileHash -LiteralPath $f.FullName -Algorithm SHA256).Hash))}
 return [Convert]::ToHexString([Security.Cryptography.SHA256]::HashData($data.ToArray()))
}
$generatedFiles=@(Get-ChildItem -LiteralPath $generated -File|Where-Object {$_.Extension -in @('.c','.h')}|Sort-Object Name)
$facadeFiles=@((Join-Path $abiRoot 'CanonicalFullInnerAbi.cpp'),(Join-Path $abiRoot 'CanonicalFullInnerAbi.h'),
 (Join-Path $integration 'CanonicalCombinedSymbolNamespace.h'),(Join-Path $integration 'CanonicalLocalInnerStateStore.hpp'),
 (Join-Path $integration 'CanonicalReferenceStateStore.hpp'),(Join-Path $integration 'CanonicalJointStateInstaller.hpp'),
 (Join-Path $build 'px4_full_inner\consumption\CanonicalSha256.hpp'),(Join-Path $build 'px4_full_inner\portable\CanonicalPortable.hpp'))|ForEach-Object{Get-Item -LiteralPath $_}|Sort-Object Name
if($FacadeOnly){
 if(-not $WithLearningAudit){throw 'Facade-only reuses the existing selected learning archive.'}
 if(-not (Test-Path -LiteralPath $archive)){throw 'Selected numerical archive missing.'}
}elseif($WithClosedEvidence){
 $receipt=Get-Content -LiteralPath (Join-Path $native 'RESULT.json') -Raw|ConvertFrom-Json
 if(-not $receipt.pass -or $receipt.compiled_c_count -ne 74){throw 'Actual closed-evidence C parity required.'}
 $private=Join-Path $out 'private';New-Item -ItemType Directory -Path $private|Out-Null
 $namespace=Get-Content -LiteralPath (Join-Path $integration 'CanonicalCombinedSymbolNamespace.h')
 $map=@($namespace|Where-Object {$_ -match '^#define (\w+) (gpenmpc_local74_\w+)$'}|ForEach-Object {$null=$_ -match '^#define (\w+) (gpenmpc_local74_\w+)$';$Matches[1]+' '+$Matches[2]})
 if($map.Count -ne 17){throw 'Exact private runtime symbol map required.'}
 $mapPath=Join-Path $private 'symbols.txt';[IO.File]::WriteAllLines($mapPath,$map)
 $objects=@(Get-ChildItem -LiteralPath $native -File -Filter '*.o');if($objects.Count -ne 74){throw 'Expected actual 74 C objects.'}
 $outputs=@()
 foreach($o in $objects){$dest=Join-Path $private $o.Name
  & (Join-Path $toolBin 'llvm-objcopy.exe') ('--redefine-syms='+$mapPath) $o.FullName $dest
  if($LASTEXITCODE -ne 0){throw 'Private symbol mapping failed.'};$outputs+=$dest}
 $archive=Join-Path $private 'libcanonical_local74_private.a'
 & (Join-Path $toolBin 'llvm-ar.exe') rcs $archive @outputs
 if($LASTEXITCODE -ne 0){throw 'Private archive failed.'}
}
$gsha=HashSet 'GPENMPC_GENERATED_SOURCE_SET_V1' $generatedFiles;$fsha=HashSet 'GPENMPC_FULL_INNER_FACADE_SOURCE_SET_V1' $facadeFiles
$asha=(Get-FileHash -LiteralPath $archive -Algorithm SHA256).Hash
$identity=@{generated_source_set_sha256=$gsha;facade_source_set_sha256=$fsha;private_archive_sha256=$asha}
$header='#pragma once'+"`n"
foreach($entry in @(@('rfi_generated_source_sha',$gsha),@('rfi_facade_source_sha',$fsha),@('rfi_private_archive_sha',$asha))){
 $values=@([Convert]::FromHexString($entry[1])|ForEach-Object{'0x'+$_.ToString('X2')}) -join ','
 $header+='static const unsigned char '+$entry[0]+'[32]={'+$values+'};'+"`n"
}
# Mechanical build identity output, never authored control/threshold data.
[IO.File]::WriteAllText((Join-Path $out 'FullInnerBuildIdentity.h'),$header)
$source=Join-Path $abiRoot 'CanonicalFullInnerAbi.cpp';$obj=Join-Path $out 'facade.o';$smoke=Join-Path $out 'c_client.o';$exe=Join-Path $out 'test_full_inner.exe'
$common=@('-std=c++14','-O2','-Wall','-Wextra','-Werror','-ffp-contract=off','-fno-fast-math','-fno-exceptions','-fno-rtti','-DGPENMPC_CANONICAL_EXPLICIT_WORKSPACE=1',
 '-I'+$out,'-I'+$generated,'-isystem',$matlabInclude)
if($WithClosedEvidence){$common+=@('-DGPENMPC_CANONICAL_CLOSED_EVIDENCE=1')}
if($WithLearningAudit){$common+=@('-DGPENMPC_CANONICAL_LEARNING_AUDIT=1')}
$args=@($common)+@('-fstack-usage','-c',$source,'-o',$obj)
$compile=@(& $cpp @args 2>&1|ForEach-Object {"$_"});$compileExit=$LASTEXITCODE
if($FacadeOnly){
 $result=[ordered]@{scope='HOST_FACADE_WITH_SELECTED_GENERATED_ARCHIVE';identity=$identity;command=$args;compile_exit=$compileExit;compiler_output=$compile;generated_C_recompiled=0;private_archive_path=$archive;facade_path=$obj;COM=0;board=0}
 [IO.File]::WriteAllText((Join-Path $out 'RESULT.json'),($result|ConvertTo-Json -Depth 6)+"`n")
 $compile
 if($compileExit -ne 0){exit 1};exit 0
}
$cArgs=@('-std=c11','-O2','-Wall','-Wextra','-Werror','-c',(Join-Path $abiRoot 'pod_header_c_smoke.c'),'-o',$smoke)
$cOutput=@(& $cc @cArgs 2>&1|ForEach-Object {"$_"});$cExit=$LASTEXITCODE
$linkArgs=@('-std=c++14','-O2','-Wall','-Wextra','-Werror','-ffp-contract=off','-fno-fast-math','-static','-municode',
 (Join-Path $abiRoot 'test_full_inner_abi.cpp'),$obj,$smoke,$archive,'-o',$exe)
if($WithClosedEvidence){
 $oracle=Join-Path $out 'closed_oracle.o';$oracleArgs=@($common)+@('-c',(Join-Path $abiRoot 'closed_evidence_oracle.cpp'),'-o',$oracle)
 & $cpp @oracleArgs;if($LASTEXITCODE -ne 0){throw 'Independent direct closed oracle compile failed.'}
 $linkArgs+=@('-DGPENMPC_CANONICAL_CLOSED_EVIDENCE=1',$oracle)
}
if($WithLearningAudit){$linkArgs+=@('-DGPENMPC_CANONICAL_LEARNING_AUDIT=1')}
$link=@();$linkExit=-1;$run=@();$runExit=-1;$test=$null
if($compileExit -eq 0 -and $cExit -eq 0){$link=@(& $cpp @linkArgs 2>&1|ForEach-Object {"$_"});$linkExit=$LASTEXITCODE
 if($linkExit -eq 0){$run=@(& $exe $rwi $sjc $gp (Join-Path $out 'HASHES.bin') 2>&1|ForEach-Object {"$_"});$runExit=$LASTEXITCODE
  if($runExit -eq 0){$test=($run -join "`n")|ConvertFrom-Json}}}
$tracked=@($facadeFiles.FullName)+@($archive,$rwi,$sjc,$gp)
$result=[ordered]@{scope='PURE_FULL_INNER_C_ABI_ACTUAL_PRIVATE74_PLUS_GP_MOCK_SOURCE_PUBLICATION';identity=$identity;
 closed_evidence_extension=[bool]$WithClosedEvidence;learning_audit_extension=[bool]$WithLearningAudit;generated_root=$generated;private_archive_path=$archive;
 command=$args;compile_exit=$compileExit;compiler_output=$compile;c_header_command=$cArgs;c_header_compile_exit=$cExit;c_header_output=$cOutput;
 link_command=$linkArgs;link_exit=$linkExit;link_output=$link;run_exit=$runExit;run_output=$run;result=$test;
 generated_source_files=$generatedFiles.Count;generated_C_recompiled=0;input_sha256=@(Get-FileHash -LiteralPath $tracked -Algorithm SHA256|Select-Object Path,Hash);
 source_set_stable=($fsha -ceq (HashSet 'GPENMPC_FULL_INNER_FACADE_SOURCE_SET_V1' $facadeFiles));COM=0;PX4_runtime_changed=$false;application_linked=$false}
[IO.File]::WriteAllText((Join-Path $out 'RESULT.json'),($result|ConvertTo-Json -Depth 15)+"`n")
$compile;$cOutput;$link;$run
if($compileExit -ne 0 -or $cExit -ne 0 -or $linkExit -ne 0 -or $runExit -ne 0){exit 1}
