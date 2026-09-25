param(
 [Parameter(Mandatory=$true)][string]$CompilerPath,
 [Parameter(Mandatory=$true)][string]$OutputDirectory,
 [string]$SourceRoot=$env:GPENMPC_SOURCE_ROOT,
 [Parameter(Mandatory=$true)][string]$Px4BuildDirectory,
 [string]$FloatMappingFixtures=$env:GPENMPC_FLOAT_MAPPING_FIXTURES,
 [Parameter(Mandatory=$true)][string]$ScienceSource,
 [string]$MatlabPackets=''
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
$rflyExpected=@('05130566CDD1599BABA745FA7BF9F1DCC9D4DD49D81FD5A81006CB6589D8351E','D1F28B00375F7D77CFA5585F87ED111D13702A7AAD08F101A735FEB7E69148F9')
for($i=0;$i -lt 2;$i++){if((Get-FileHash -LiteralPath $rflyFixtures[$i]).Hash -ne $rflyExpected[$i]){throw 'Immutable float32 fixture identity mismatch'}}
$rflyWrapperPath=Join-Path $rflyBuild 'host_runtime/+gpenmpcNative/rflyCanonicalKernelBlock.m'
$rflyConfig=(Get-Content -LiteralPath $rflyPassport -Raw|ConvertFrom-Json).expected_sha256.effective_configuration_payload
$rflyScience=(Get-FileHash -LiteralPath $rflySciencePath).Hash
$rflyCode=(Get-FileHash -LiteralPath (Join-Path $rflyArm 'GPENMPC_Rfly_Canonical_Controller.c')).Hash
$rflyWrapper=(Get-FileHash -LiteralPath $rflyWrapperPath).Hash
$rflyProtected=@($rflyPassport,$rflySciencePath,$rflyWrapperPath)+$rflyFixtures
foreach($rflyName in @('CanonicalRflyExecutor.hpp','RflySnapshotBoundExecutor.hpp','SimulinkCanonicalPolicy.hpp','SlimKernelCodec.hpp','SlimSnapshotExecutor.hpp','SlimArgumentTransport.hpp','test_slim_argument_transport.cpp')){$rflyProtected+=Join-Path $PSScriptRoot $rflyName}
foreach($rflyName in @('execution/CanonicalFullInnerExecutor.hpp','state_execution/SnapshotBoundExecutor.hpp','consumption/ConsumptionBinding.hpp','argument_abi/CanonicalKernelArgumentCodec.hpp','argument_transport/CanonicalArgumentTransport.hpp','px4_state_adapter/AtomicOdometryAdapter.hpp')){$rflyProtected+=Join-Path $rflyBuild ('px4_full_inner/'+$rflyName)}
foreach($rflyName in @('GPENMPC_Rfly_Canonical_Controller.c','rt_nonfinite.c','rtGetInf.c')){$rflyProtected+=Join-Path $rflyArm $rflyName}
$rflyBefore=@{};foreach($rflyFile in $rflyProtected){$rflyBefore[$rflyFile]=(Get-FileHash -LiteralPath $rflyFile).Hash}
$rflyRun=New-GpenmpcTestOutput $OutputDirectory @($build,$actual,$rflyTools,$fixtureRoot)
Push-Location $rflyRun
try{
    $rflyObjects=@()
    foreach($rflyUnit in @('GPENMPC_Rfly_Canonical_Controller','rt_nonfinite','rtGetInf')){
        $rflyObject=$rflyUnit+'_host.o'
        & (Join-Path $rflyTools 'clang.exe') -std=c11 -O2 -ffp-contract=off -fno-fast-math -Wall -Wextra -Werror -I $rflyArm -c (Join-Path $rflyArm ($rflyUnit+'.c')) -o $rflyObject
        if($LASTEXITCODE -ne 0){throw 'Generated C compile failed'}
        $rflyObjects+=$rflyObject
    }
    & (Join-Path $rflyTools 'clang++.exe') -std=c++14 -O2 -ffp-contract=off -fno-fast-math -Wall -Wextra -Werror -pedantic -static -Wno-address-of-packed-member -isystem (Join-Path $rflyMav 'common') -isystem $rflyMav -I $rflyMeta -I (Join-Path $rflyBuild 'px4_full_inner/px4_ingress/generated') -I (Join-Path $vi 'px4_wire/pump_host_stub') -I $rflyHeaders -I $rflyArm (Join-Path $vi 'test_slim_argument_transport.cpp') @rflyObjects -o test_slim_argument_transport.exe
    if($LASTEXITCODE -ne 0){throw 'Slim transport HOST compile failed'}
    $rflyArgs=@($rflyFixtures)+@($rflyConfig,$rflyScience,$rflyCode,$rflyWrapper)
    if($MatlabPackets){$rflyArgs+=$MatlabPackets}
    & '.\test_slim_argument_transport.exe' @rflyArgs
    if($LASTEXITCODE -ne 0){throw 'Slim codec HOST regression failed'}
}finally{
    Pop-Location
    foreach($rflyFile in $rflyProtected){if((Get-FileHash -LiteralPath $rflyFile).Hash -ne $rflyBefore[$rflyFile]){throw ('Source/fixture changed: '+$rflyFile)}}
}
Write-Output 'Offline component tests passed; source and fixture hashes are unchanged; no device access.'
