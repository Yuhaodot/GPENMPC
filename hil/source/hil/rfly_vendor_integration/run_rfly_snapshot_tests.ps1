param(
 [Parameter(Mandatory=$true)][string]$CompilerPath,
 [Parameter(Mandatory=$true)][string]$OutputDirectory,
 [string]$SourceRoot=$env:GPENMPC_SOURCE_ROOT,
 [Parameter(Mandatory=$true)][string]$Px4BuildDirectory,
 [string]$FloatMappingFixtures=$env:GPENMPC_FLOAT_MAPPING_FIXTURES,
 [Parameter(Mandatory=$true)][string]$ScienceSource,
 [Parameter(Mandatory=$true)][string]$ExecutorMappingFixtures
)
$ErrorActionPreference='Stop'
. (Join-Path $PSScriptRoot 'host_test_paths.ps1')
if (!$SourceRoot) { $SourceRoot=Split-Path $PSScriptRoot -Parent }
$build=Resolve-GpenmpcTestInput $SourceRoot 'SourceRoot' -Directory
$vi=Join-Path $build 'rfly_vendor_integration'
$cc=Resolve-GpenmpcTestInput $CompilerPath 'CompilerPath'
$actual=Resolve-GpenmpcTestInput $Px4BuildDirectory 'Px4BuildDirectory' -Directory
$rflyBuild=$build
$rflyArm=Join-Path $build 'evidence/arm_controller/GPENMPC_Rfly_Canonical_Controller_ert_rtw'
$rflyTools=Split-Path $cc -Parent
$null=Resolve-GpenmpcTestInput (Join-Path $rflyTools 'clang.exe') 'C compiler'
$rflyHeaders=$actual;$rflyMav=Join-Path $actual 'mavlink';$rflyMeta=$actual
$fixtureRoot=Resolve-GpenmpcTestInput $FloatMappingFixtures 'FloatMappingFixtures' -Directory
$rflyFixtures=@('MATLAB_ARGUMENTS_AND_EXPECTED.bin','MATLAB_NED_AND_MAPPED_STATE.bin')|ForEach-Object{Join-Path $fixtureRoot $_}
$packageRoot=Split-Path (Split-Path $build -Parent) -Parent
$rflyPassport=Resolve-GpenmpcTestInput (Join-Path $packageRoot 'assets/canonical/binding/execution.json') 'canonical execution binding'
$rflySciencePath=Resolve-GpenmpcTestInput $ScienceSource 'ScienceSource'
$executorFixtures=Resolve-GpenmpcTestInput $ExecutorMappingFixtures 'ExecutorMappingFixtures' -Directory
$rflyFixtures+=@('MATLAB_ARGUMENTS_AND_EXPECTED.bin','MATLAB_NED_AND_MAPPED_STATE.bin')|ForEach-Object{Join-Path $executorFixtures $_}
$rflyFixtureHashes=@('05130566CDD1599BABA745FA7BF9F1DCC9D4DD49D81FD5A81006CB6589D8351E','D1F28B00375F7D77CFA5585F87ED111D13702A7AAD08F101A735FEB7E69148F9','C94D0559DE3C815A6EFA61321CDC00BB2E82C73E8B109EBA8367A8657E75E0DE','281632FE0AB71D65BB1C5B41F8B9A0A282251C8DAB91FDFFBD6A9A2BE3D2ED1D')
for($i=0;$i -lt 4;$i++){if((Get-FileHash -LiteralPath $rflyFixtures[$i] -Algorithm SHA256).Hash -ne $rflyFixtureHashes[$i]){throw 'Immutable fixture identity mismatch'}}
$rflyCodePath=Join-Path $rflyArm 'GPENMPC_Rfly_Canonical_Controller.c'
$rflyWrapperPath=Join-Path $rflyBuild 'host_runtime/+gpenmpcNative/rflyCanonicalKernelBlock.m'
$rflyConfig=(Get-Content -LiteralPath $rflyPassport -Raw|ConvertFrom-Json).expected_sha256.effective_configuration_payload
$rflyScience=(Get-FileHash -LiteralPath $rflySciencePath -Algorithm SHA256).Hash
$rflyCode=(Get-FileHash -LiteralPath $rflyCodePath -Algorithm SHA256).Hash
$rflyWrapper=(Get-FileHash -LiteralPath $rflyWrapperPath -Algorithm SHA256).Hash
$rflyProtected=@($rflyPassport,$rflySciencePath,$rflyWrapperPath)+$rflyFixtures
foreach($rflyFile in @('execution/CanonicalFullInnerExecutor.hpp','state_execution/SnapshotBoundExecutor.hpp','consumption/ConsumptionBinding.hpp','px4_state_adapter/AtomicOdometryAdapter.hpp','argument_abi/CanonicalKernelArgumentCodec.hpp','actuator_interface/CanonicalRotorInterface.hpp','portable/CanonicalPortable.hpp')){$rflyProtected+=Join-Path $rflyBuild ('px4_full_inner/'+$rflyFile)}
foreach($rflyUnit in @('GPENMPC_Rfly_Canonical_Controller','rt_nonfinite','rtGetInf')){$rflyProtected+=Join-Path $rflyArm ($rflyUnit+'.c')}
foreach($rflyFile in @('CanonicalRflyExecutor.hpp','RflySnapshotBoundExecutor.hpp','SimulinkCanonicalPolicy.hpp','test_rfly_snapshot_executor.cpp')){$rflyProtected+=Join-Path $PSScriptRoot $rflyFile}
$rflyBefore=@{};foreach($rflyFile in $rflyProtected){$rflyBefore[$rflyFile]=(Get-FileHash -LiteralPath $rflyFile -Algorithm SHA256).Hash}
$rflyRun=New-GpenmpcTestOutput $OutputDirectory @($build,$actual,$rflyTools,$fixtureRoot)
Push-Location $rflyRun
try{
    $rflyObjects=@()
    foreach($rflyUnit in @('GPENMPC_Rfly_Canonical_Controller','rt_nonfinite','rtGetInf')){
        $rflyObject=$rflyUnit+'_host.o'
        & (Join-Path $rflyTools 'clang.exe') -std=c11 -O2 -ffp-contract=off -fno-fast-math -Wall -Wextra -Werror -I $rflyArm -c (Join-Path $rflyArm ($rflyUnit+'.c')) -o $rflyObject
        if($LASTEXITCODE -ne 0){throw 'ARM-target generated C HOST compile failed'}
        $rflyObjects+=$rflyObject
    }
    & (Join-Path $rflyTools 'clang++.exe') -std=c++14 -O2 -ffp-contract=off -fno-fast-math -Wall -Wextra -Werror -pedantic -static -Wno-address-of-packed-member -isystem (Join-Path $rflyMav 'common') -isystem $rflyMav -I (Join-Path $rflyBuild 'px4_full_inner/px4_ingress/generated') -I (Join-Path $vi 'px4_wire/pump_host_stub') -I $rflyHeaders -I $rflyArm (Join-Path $vi 'test_rfly_snapshot_executor.cpp') @rflyObjects -o test_rfly_snapshot_executor.exe
    if($LASTEXITCODE -ne 0){throw 'Snapshot test compile failed'}
    & '.\test_rfly_snapshot_executor.exe' @rflyFixtures $rflyConfig $rflyScience $rflyCode $rflyWrapper
    if($LASTEXITCODE -ne 0){throw 'HOST regression failed'}
}finally{
    Pop-Location
    foreach($rflyFile in $rflyProtected){if((Get-FileHash -LiteralPath $rflyFile -Algorithm SHA256).Hash -ne $rflyBefore[$rflyFile]){throw ('Source/fixture changed: '+$rflyFile)}}
}
Write-Output 'Offline component tests passed; source and fixture hashes are unchanged; no device access.'
