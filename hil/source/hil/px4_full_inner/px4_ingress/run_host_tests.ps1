param(
 [Parameter(Mandatory=$true)][string]$CompilerPath,
 [Parameter(Mandatory=$true)][string]$MavlinkDirectory,
 [Parameter(Mandatory=$true)][string]$OutputDirectory,
 [string]$Px4Root=$env:GPENMPC_PX4_ROOT
)
$ErrorActionPreference='Stop'
. (Join-Path $PSScriptRoot '../../rfly_vendor_integration/host_test_paths.ps1')
$ingressCompiler=Resolve-GpenmpcTestInput $CompilerPath 'CompilerPath'
$ingressGenerated=Resolve-GpenmpcTestInput $MavlinkDirectory 'MavlinkDirectory' -Directory
$ingressPx4=Resolve-GpenmpcTestInput $Px4Root 'Px4Root' -Directory
$ingressUorb=Join-Path $PSScriptRoot 'generated'
$ingressInputs=@($ingressCompiler,$PSCommandPath,
 (Join-Path $ingressGenerated 'common/mavlink.h'),(Join-Path $ingressGenerated 'common/mavlink_msg_tunnel.h'),
 (Join-Path $ingressGenerated 'mavlink_helpers.h'),(Join-Path $PSScriptRoot 'GPENMPCFullInnerIngress.hpp'),
 (Join-Path $PSScriptRoot 'test_ingress.cpp'),(Join-Path $ingressUorb 'uORB/topics/gpenmpc_full_inner_ingress.h'),
 (Join-Path $ingressUorb 'uORB/topics/uORBTopics.hpp'),(Join-Path $ingressPx4 'platforms/common/uORB/uORB.h'),
 (Join-Path $ingressPx4 'src/include/visibility.h'))
foreach($inputFile in $ingressInputs){$null=Resolve-GpenmpcTestInput $inputFile 'ingress test input'}
$ingressOutput=New-GpenmpcTestOutput $OutputDirectory @((Split-Path (Split-Path $PSScriptRoot -Parent) -Parent),$ingressPx4,$ingressGenerated)
foreach($ingressMode in @('aligned','bytewise')){
 $ingressExe=Join-Path $ingressOutput ('test_ingress_'+$ingressMode+'.exe')
 $ingressArgs=@('-std=c++17','-O2','-Wall','-Wextra','-Werror','-pedantic','-static','-fno-exceptions','-fno-rtti',
  '-Wno-address-of-packed-member','-isystem',(Join-Path $ingressGenerated 'common'),'-isystem',$ingressGenerated,
  '-isystem',$ingressUorb,'-isystem',(Join-Path $ingressPx4 'platforms/common'),'-include',(Join-Path $ingressPx4 'src/include/visibility.h'))
 if($ingressMode -eq 'bytewise'){$ingressArgs+=@('-DMAVLINK_ALIGNED_FIELDS=0')}
 & $ingressCompiler @ingressArgs (Join-Path $PSScriptRoot 'test_ingress.cpp') '-o' $ingressExe
 if($LASTEXITCODE -ne 0){throw "Ingress compile failed ($ingressMode): $LASTEXITCODE"}
 & $ingressExe (Join-Path $ingressOutput ('result_'+$ingressMode+'.json'))
 if($LASTEXITCODE -ne 0){throw "Ingress test failed ($ingressMode): $LASTEXITCODE"}
}
Get-FileHash -Algorithm SHA256 -LiteralPath $ingressInputs | Select-Object Path,Hash | ConvertTo-Json -Compress
