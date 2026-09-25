<#
.SYNOPSIS
Build the continuous-GP UDP transport MEX with LLVM MinGW.
.DESCRIPTION
Uses the project SDK bridge and headers/libraries from a local MATLAB installation.
Writes the compiled MEX to a new OutputDirectory.

Bridge provenance: RflySimAPIs/RflySimSDK/html/rfly__udp_8h_source.html,
SHA256 BD96B77CA8F771BA28E1B37967D36BCE02C8F350822B5D930260E322B7E5352D,
lines 264-275 (ipToString), 400-444 (SendTo), 704-726 (RecvNoblock), with
project-specific socket storage, macros and compiler guards.
#>
[CmdletBinding()]
param(
    [Parameter(Mandatory = $true)]
    [ValidateNotNullOrEmpty()]
    [string]$CompilerBin,

    [Parameter(Mandatory = $true)]
    [ValidateNotNullOrEmpty()]
    [string]$MatlabRoot,

    [Parameter(Mandatory = $true)]
    [ValidateNotNullOrEmpty()]
    [string]$OutputDirectory
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

$hilRoot = Split-Path -Parent $PSScriptRoot
$includeDirectory = Join-Path $MatlabRoot 'extern/include'
$nativeDirectory = Join-Path $hilRoot 'host_runtime/native_include'
$generatedDirectory = Join-Path $hilRoot 'evidence/gp_predictor/generated'
$bridge = Join-Path $PSScriptRoot 'gpenmpc_rfly_udp_sdk_bound.hpp'
$cCompiler = Join-Path $CompilerBin 'clang.exe'
$cxxCompiler = Join-Path $CompilerBin 'clang++.exe'
$archiver = Join-Path $CompilerBin 'llvm-ar.exe'
$versionSource = Join-Path $MatlabRoot 'extern/version/c_mexapi_version.c'
$mexLibrary = Join-Path $MatlabRoot 'extern/lib/win64/microsoft/libmex.lib'
$mxLibrary = Join-Path $MatlabRoot 'extern/lib/win64/microsoft/libmx.lib'
$exportDefinition = Join-Path $MatlabRoot 'extern/lib/win64/mingw64/exportsmexfileversion.def'
$apiSource = Join-Path $PSScriptRoot 'canonical_gp_standalone_api.c'
$transportSource = Join-Path $PSScriptRoot 'gpenmpc_rfly_udp_transport_mex.cpp'

if (Test-Path -LiteralPath $OutputDirectory) {
    throw "OutputDirectory must be a new directory: $OutputDirectory"
}
foreach ($required in @($cCompiler, $cxxCompiler, $archiver, $includeDirectory,
        $nativeDirectory, $generatedDirectory, $bridge, $versionSource,
        $mexLibrary, $mxLibrary, $exportDefinition, $apiSource, $transportSource)) {
    if (-not (Test-Path -LiteralPath $required)) {
        throw "Missing build input: $required"
    }
}
$expectedBridgeHash = '5AF8E6EDD6AD943D3CCD8F1F2734542508369F9146260D5E73950B0E56B8EB47'
if ((Get-FileHash -LiteralPath $bridge -Algorithm SHA256).Hash -ne $expectedBridgeHash) {
    throw "SDK bridge SHA256 does not match the project input: $bridge"
}
$generatedSources = @(Get-ChildItem -LiteralPath $generatedDirectory -File -Filter '*.c')
if ($generatedSources.Count -eq 0) {
    throw "No generated C sources found: $generatedDirectory"
}
New-Item -ItemType Directory -Path $OutputDirectory -ErrorAction Stop | Out-Null

function Invoke-Compiler {
    param([string]$Executable, [string[]]$CompilerArguments)
    & $Executable @CompilerArguments
    if ($LASTEXITCODE -ne 0) {
        throw "Compiler exit code: $LASTEXITCODE ($Executable)"
    }
}

$versionObject = Join-Path $OutputDirectory 'mex_version.o'
Invoke-Compiler -Executable $cCompiler -CompilerArguments @(
    '-std=c11', '-O2', '-DMATLAB_MEX_FILE', '-DMATLAB_DEFAULT_RELEASE=R2018a',
    "-I$includeDirectory", '-c', $versionSource, '-o', $versionObject)
$objects = @()
foreach ($file in $generatedSources) {
    $object = Join-Path $OutputDirectory ($file.BaseName + '.o')
    $objects += $object
    Invoke-Compiler -Executable $cCompiler -CompilerArguments @(
        '-std=c11', '-O2', '-ffp-contract=off', '-fno-fast-math',
        "-I$generatedDirectory", "-I$includeDirectory", '-c', $file.FullName, '-o', $object)
}
$library = Join-Path $OutputDirectory 'libcanonical_gp256.a'
Invoke-Compiler -Executable $archiver -CompilerArguments (@('rcs', $library) + $objects)
$apiObject = Join-Path $OutputDirectory 'canonical_gp_standalone_api.o'
Invoke-Compiler -Executable $cCompiler -CompilerArguments @(
    '-std=c11', '-O2', '-ffp-contract=off', '-fno-fast-math', '-Wall', '-Wextra', '-Werror',
    "-I$generatedDirectory", "-I$includeDirectory", '-c', $apiSource, '-o', $apiObject)
$options = @(
    '-std=c++14', '-O2', '-ffp-contract=off', '-fno-fast-math', '-shared', '-static',
    '-Wall', '-Wextra', '-Werror', '-Wno-address-of-packed-member', '-DMATLAB_MEX_FILE',
    '-DMATLAB_DEFAULT_RELEASE=R2018a', "-I$includeDirectory", "-I$nativeDirectory",
    "-I$hilRoot/rfly_vendor_integration/px4_wire/pump_host_stub", '-isystem',
    "$nativeDirectory/mavlink", '-isystem', "$nativeDirectory/mavlink/common")
$links = @($versionObject, $mexLibrary, $mxLibrary, $exportDefinition, '-Wl,--no-undefined')
$binary = Join-Path $OutputDirectory 'gpenmpc_rfly_udp_transport_mex.mexw64'
Invoke-Compiler -Executable $cxxCompiler -CompilerArguments (
    $options + @($transportSource) + $links + @(
        '-DGPENMPC_UDP_CONTINUOUS_GP=1', $apiObject, $library, '-lws2_32', '-o', $binary))
Write-Output $binary
