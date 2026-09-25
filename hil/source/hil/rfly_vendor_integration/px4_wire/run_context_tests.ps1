param(
 [Parameter(Mandatory=$true)][string]$CompilerPath,
 [Parameter(Mandatory=$true)][string]$OutputDirectory,
 [string]$SourceRoot=$env:GPENMPC_SOURCE_ROOT,
 [Parameter(Mandatory=$true)][string]$Px4BuildDirectory,
 [string]$FloatMappingFixtures=$env:GPENMPC_FLOAT_MAPPING_FIXTURES,
 [string]$MatlabPackets='',[switch]$Dispatch,[switch]$Feedback
)
$ErrorActionPreference='Stop'
. (Join-Path (Split-Path $PSScriptRoot -Parent) 'host_test_paths.ps1')
if (!$SourceRoot) { $SourceRoot=Split-Path (Split-Path $PSScriptRoot -Parent) -Parent }
$build=Resolve-GpenmpcTestInput $SourceRoot 'SourceRoot' -Directory
$vi=Join-Path $build 'rfly_vendor_integration'
$cc=Resolve-GpenmpcTestInput $CompilerPath 'CompilerPath'
$actual=Resolve-GpenmpcTestInput $Px4BuildDirectory 'Px4BuildDirectory' -Directory
$rflyDir=$vi;$rflyBuild=$build;$rflyHeaders=$actual
$rflyArm=Join-Path $build 'evidence/arm_controller/GPENMPC_Rfly_Canonical_Controller_ert_rtw'
$fixtureRoot=Resolve-GpenmpcTestInput $FloatMappingFixtures 'FloatMappingFixtures' -Directory
$rflyFixtures=@('MATLAB_ARGUMENTS_AND_EXPECTED.bin','MATLAB_NED_AND_MAPPED_STATE.bin')|ForEach-Object{Resolve-GpenmpcTestInput (Join-Path $fixtureRoot $_) $_}
$rflyTools=Split-Path $cc -Parent
$null=Resolve-GpenmpcTestInput (Join-Path $rflyTools 'clang.exe') 'C compiler'
$rflyRun=New-GpenmpcTestOutput $OutputDirectory @($build,$actual,$rflyTools,$fixtureRoot)
$rflySource=if($Feedback){'test_committed_feedback.cpp'}elseif($Dispatch){'test_ingress_dispatch.cpp'}else{'test_context_wire.cpp'}
Push-Location $rflyRun
try{
$rflyObjects=@()
foreach($unit in @('GPENMPC_Rfly_Canonical_Controller','rt_nonfinite','rtGetInf')) {
    $object=Join-Path $rflyRun ($unit+'_host.o')
    & (Join-Path $rflyTools 'clang.exe') -std=c11 -O2 -ffp-contract=off -fno-fast-math -Wall -Wextra -Werror -I $rflyArm -c (Join-Path $rflyArm ($unit+'.c')) -o $object
    if($LASTEXITCODE -ne 0){throw 'Generated controller host compilation failed'}
    $rflyObjects+=$object
}

& $cc -std=c++14 -O2 -ffp-contract=off -fno-fast-math -Wall -Wextra -Werror -pedantic -static -Wno-address-of-packed-member -isystem (Join-Path $rflyHeaders 'mavlink/common') -isystem (Join-Path $rflyHeaders 'mavlink') -I $actual -I (Join-Path $vi 'px4_wire/pump_host_stub') -I $rflyHeaders -I $rflyArm (Join-Path (Join-Path $vi 'px4_wire') $rflySource) @rflyObjects -o test_context_wire.exe
if($LASTEXITCODE -ne 0){throw 'compile failed'}
$rflyArgs=@($rflyFixtures[0],$rflyFixtures[1])
if($MatlabPackets){$rflyArgs+=$MatlabPackets}
& '.\test_context_wire.exe' @rflyArgs
if($LASTEXITCODE -ne 0){throw 'test failed'}
}finally{Pop-Location}
