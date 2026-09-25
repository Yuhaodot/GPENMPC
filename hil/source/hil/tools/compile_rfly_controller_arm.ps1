param([Parameter(Mandatory=$true)][string]$GeneratedDirectory)
$ErrorActionPreference='Stop'
$build=Split-Path -Parent $PSScriptRoot
$gen=[IO.Path]::GetFullPath($GeneratedDirectory)
if(-not $gen.StartsWith(($build+'\evidence\'),[StringComparison]::OrdinalIgnoreCase)){throw 'Project generated source only'}
$out=Join-Path (Split-Path -Parent $gen) 'cortex_m7_objects'
if(Test-Path -LiteralPath $out){throw 'Choose an unused output path.'}
New-Item -ItemType Directory -Path $out | Out-Null
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
$prefix=$toolchain+'/bin/arm-none-eabi-'
function L([string]$p){LinuxPath $p}
$names=@('GPENMPC_Rfly_Canonical_Controller','rt_nonfinite','rtGetInf')
$records=@();$log=@()
foreach($name in $names){
    $source=Join-Path $gen ($name+'.c');$object=Join-Path $out ($name+'.o')
    $parameters=@('-mcpu=cortex-m7','-mthumb','-mfpu=fpv5-d16','-mfloat-abi=hard','-Os',
        '-std=c11','-fno-fast-math','-ffp-contract=off','-fstack-usage','-fdata-sections','-ffunction-sections',
        '-Wall','-Wextra','-Werror','-I',(L $gen),'-c',(L $source),'-o',(L $object))
    $message=@(& wsl.exe @wslArguments --exec ($prefix+'gcc') @parameters 2>&1);$rc=$LASTEXITCODE
    $log+=$message;$message | Write-Output
    if($rc -ne 0){throw ('ARM compile failed: '+$name+' rc='+$rc)}
    $records += [ordered]@{source=$source;source_sha256=(Get-FileHash -LiteralPath $source -Algorithm SHA256).Hash;
        object=$object;object_sha256=(Get-FileHash -LiteralPath $object -Algorithm SHA256).Hash}
}
$objects=@($records | ForEach-Object {L $_.object})
$size=@(& wsl.exe @wslArguments --exec ($prefix+'size') @objects 2>&1)
if($LASTEXITCODE -ne 0){throw 'ARM size failed'}
$record=[ordered]@{status='PASS_CORTEX_M7_GENERATED_SIMULINK_OBJECTS_ONLY';objects=$records;size=$size;
    stack=@(Get-ChildItem -LiteralPath $out -Filter '*.su' | ForEach-Object {[ordered]@{file=$_.Name;text=[IO.File]::ReadAllText($_.FullName)}});
    board_actions=0;COM_open=0;firmware_application_linked=$false;board_runtime_verified=$false;
    limitations='Per-function static stack measurements; full call-chain stack, WCET and board validation require separate assessment.'}
$record | ConvertTo-Json -Depth 8 | Set-Content -LiteralPath (Join-Path $out 'RESULT.json') -Encoding utf8
$record | ConvertTo-Json -Depth 8
