param(
    [Parameter(Mandatory=$true)][string]$CompilerPath,
    [Parameter(Mandatory=$true)][string]$MavlinkDirectory,
    [Parameter(Mandatory=$true)][string]$OutputDirectory
)
$ErrorActionPreference = 'Stop'
$ingressCompiler = (Resolve-Path -LiteralPath $CompilerPath).ProviderPath
$ingressGenerated = (Resolve-Path -LiteralPath $MavlinkDirectory).ProviderPath
if (![IO.Path]::IsPathRooted($OutputDirectory)) { throw 'OutputDirectory must be an absolute path.' }
$ingressOutput = [IO.Path]::GetFullPath($OutputDirectory)
$ingressSourceRoot = [IO.Path]::GetFullPath($PSScriptRoot)
if ($ingressOutput.Equals($ingressSourceRoot, [StringComparison]::OrdinalIgnoreCase) -or
    $ingressOutput.StartsWith($ingressSourceRoot + [IO.Path]::DirectorySeparatorChar, [StringComparison]::OrdinalIgnoreCase)) {
    throw 'Use a separate output directory, outside the source directory.'
}
if (Test-Path -LiteralPath $ingressOutput) { throw 'OutputDirectory must not already exist.' }
$ingressInputs = @(
    $ingressCompiler,
    (Join-Path $ingressGenerated 'common\mavlink.h'),
    (Join-Path $ingressGenerated 'common\mavlink_msg_tunnel.h'),
    (Join-Path $ingressGenerated 'mavlink_helpers.h'),
    (Join-Path $PSScriptRoot 'GPENMPCFullInnerIngress_stdarray.hpp'),
    (Join-Path $PSScriptRoot 'test_ingress_shape_fixture.cpp'),
    $PSCommandPath
)
foreach ($ingressRequired in $ingressInputs) {
    if (!(Test-Path -LiteralPath $ingressRequired -PathType Leaf)) { throw "Missing input: $ingressRequired" }
}
New-Item -ItemType Directory -Path $ingressOutput | Out-Null
foreach ($ingressMode in @('aligned','bytewise')) {
    $ingressExe = Join-Path $ingressOutput "test_ingress_shape_$ingressMode.exe"
    $ingressArgs = @('-std=c++17','-O2','-Wall','-Wextra','-Werror','-pedantic','-static','-fno-exceptions','-fno-rtti',
        '-Wno-address-of-packed-member','-isystem',(Join-Path $ingressGenerated 'common'),'-isystem',$ingressGenerated)
    if ($ingressMode -eq 'bytewise') { $ingressArgs += @('-DMAVLINK_ALIGNED_FIELDS=0') }
    & $ingressCompiler @ingressArgs (Join-Path $PSScriptRoot 'test_ingress_shape_fixture.cpp') '-o' $ingressExe
    if ($LASTEXITCODE -ne 0) { throw "Ingress shape compile failed ($ingressMode): $LASTEXITCODE" }
    & $ingressExe (Join-Path $ingressOutput "result_$ingressMode.json")
    if ($LASTEXITCODE -ne 0) { throw "Ingress shape tests failed ($ingressMode): $LASTEXITCODE" }
}
Get-FileHash -Algorithm SHA256 -LiteralPath $ingressInputs |
    Select-Object Path,Hash | ConvertTo-Json -Compress
