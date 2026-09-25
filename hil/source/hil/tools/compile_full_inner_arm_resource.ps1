param([Parameter(Mandatory=$true)][string]$OutputDirectory)
$ErrorActionPreference='Stop'
$build=Split-Path -Parent $PSScriptRoot
$gen=(& (Join-Path $PSScriptRoot 'gpenmpc_external_path.ps1') 'full_inner_generated_source')
$wslArguments=@()
if(-not [string]::IsNullOrWhiteSpace($env:GPENMPC_WSL_DISTRO)){$wslArguments=@('-d',$env:GPENMPC_WSL_DISTRO)}
function LinuxPath([string]$p) {
    if([string]::IsNullOrWhiteSpace($p)){throw 'An explicit installation or source path is required.'}
    if($p.StartsWith('/')){return $p}
    $converted=@(& wsl.exe @wslArguments --exec wslpath -u ([IO.Path]::GetFullPath($p)))
    if($LASTEXITCODE -ne 0 -or $converted.Count -ne 1){throw 'Could not convert the configured path to WSL.'}
    [string]$converted[0]
}
if([string]::IsNullOrWhiteSpace($env:GPENMPC_ARM_TOOLCHAIN)){throw 'Set GPENMPC_ARM_TOOLCHAIN to the ARM toolchain installation directory.'}
$toolchain=(LinuxPath $env:GPENMPC_ARM_TOOLCHAIN).TrimEnd('/')
& wsl.exe @wslArguments --exec test -x ($toolchain+'/bin/arm-none-eabi-gcc')
if($LASTEXITCODE -ne 0){throw 'The configured ARM compiler is missing or not executable in WSL.'}
$compiler=$toolchain+'/bin/arm-none-eabi-'
if([string]::IsNullOrWhiteSpace($env:GPENMPC_MATLAB_ROOT)){throw 'Set GPENMPC_MATLAB_ROOT to the MATLAB installation directory.'}
$matlabInclude=(LinuxPath $env:GPENMPC_MATLAB_ROOT).TrimEnd('/')+'/extern/include'
& wsl.exe @wslArguments --exec test -d $matlabInclude
if($LASTEXITCODE -ne 0){throw 'The configured MATLAB external headers are unavailable in WSL.'}
if(Test-Path -LiteralPath $OutputDirectory){throw 'Choose an unused output path.'}
$outFull=[IO.Path]::GetFullPath($OutputDirectory)
if(-not $outFull.StartsWith(($build+'\'),[StringComparison]::OrdinalIgnoreCase)){throw 'Output must be inside the current BUILD root'}
New-Item -ItemType Directory -Path $outFull | Out-Null
$core=Join-Path $build 'px4_full_inner\execution\CanonicalFullInnerExecutor.hpp'
$probe=Join-Path $build 'px4_full_inner\execution\arm_resource_probe.cpp'
$hashBefore=(Get-FileHash -Algorithm SHA256 -LiteralPath $core).Hash
$log=Join-Path $outFull 'COMPILE_LOG.txt'
function InvokeCompiler([string]$program,[string[]]$parameters) {
    $text=@(& wsl.exe @wslArguments --exec ($compiler+$program) @parameters 2>&1)
    $code=$LASTEXITCODE
    $text | Tee-Object -FilePath $log -Append | Write-Output
    if($code -ne 0){throw ('ARM '+$program+' failed, rc='+$code)}
}
InvokeCompiler 'g++' @('--version')
$common=@('-mcpu=cortex-m7','-mthumb','-mfpu=fpv5-d16','-mfloat-abi=hard','-Os',
    '-fno-fast-math','-ffp-contract=off','-fstack-usage','-fdata-sections','-ffunction-sections',
    '-Wall','-Wextra','-Werror','-I',(LinuxPath $gen),'-I',$matlabInclude)
$names=@('allOrAny','det','dot','eye','norm','gpenmpcNative_se3WrenchKernel_initialize',
    'gpenmpcNative_se3WrenchKernel_terminate','gpenmpcNative_se3WrenchKernel','rt_nonfinite','rtGetInf','rtGetNaN')
$objects=@()
foreach($name in $names){
    $obj=Join-Path $outFull ($name+'.o');$objects+=(LinuxPath $obj)
    InvokeCompiler 'gcc' ($common+@('-std=c11','-c',(LinuxPath (Join-Path $gen ($name+'.c'))),'-o',(LinuxPath $obj)))
}
$probeObject=Join-Path $outFull 'arm_resource_probe.o';$objects+=(LinuxPath $probeObject)
InvokeCompiler 'g++' ($common+@('-std=gnu++14','-fno-exceptions','-fno-rtti','-fno-threadsafe-statics',
    '-c',(LinuxPath $probe),'-o',(LinuxPath $probeObject)))
InvokeCompiler 'size' (@('-A')+$objects)
InvokeCompiler 'objdump' @('-s','-j','.rodata.gpenmpc_arm_object_sizes',(LinuxPath $probeObject))
$hashAfter=(Get-FileHash -Algorithm SHA256 -LiteralPath $core).Hash
if($hashBefore -ne $hashAfter){throw 'Core source changed during compile'}
$record=[ordered]@{status='PASS_COMPILE_ONLY_CORTEX_M7_OBJECTS';
    target='Cortex-M7 fpv5-d16 hard-float';object_count=$objects.Count;
    firmware_build=0;link_or_execution=0;com_open=0;board_actions=0;model_count=0;
    executor_sha256=$hashAfter;probe_sha256=(Get-FileHash -Algorithm SHA256 -LiteralPath $probe).Hash;
    limitations=@('Compile-only object assessment; linking with NuttX/libcxx/PX4 is a separate step.',
      'Static stack reports are per-function, not total call-chain stack or measured worst-case execution time.',
      'Actual board execution, timing, uORB ownership and safe output retirement remain unverified.');
    stack_files=@(Get-ChildItem -LiteralPath $outFull -Filter '*.su' | ForEach-Object {
      [ordered]@{name=$_.Name;content=[IO.File]::ReadAllText($_.FullName)}})}
$record | ConvertTo-Json -Depth 7 | Set-Content -LiteralPath (Join-Path $outFull 'RESULT.json') -Encoding utf8
$record | ConvertTo-Json -Depth 7
