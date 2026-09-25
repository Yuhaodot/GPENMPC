param([Parameter(Mandatory=$true)][ValidateSet('llvm','gcc','matlab','rfly')][string]$Kind,[string]$Relative='')
$ErrorActionPreference='Stop'
switch($Kind) {
    'llvm' { $setting='GPENMPC_LLVM_ROOT'; $root=$env:GPENMPC_LLVM_ROOT; if([string]::IsNullOrWhiteSpace($root)){$root=$env:MW_MINGW64_LOC} }
    'gcc' { $setting='GPENMPC_GCC_ROOT'; $root=$env:GPENMPC_GCC_ROOT; if([string]::IsNullOrWhiteSpace($root)){$root=$env:MW_MINGW64_LOC} }
    'rfly' { $setting='GPENMPC_RFLY_ROOT'; $root=$env:GPENMPC_RFLY_ROOT }
    'matlab' {
        $setting='GPENMPC_MATLAB_ROOT'; $root=$env:GPENMPC_MATLAB_ROOT
        if([string]::IsNullOrWhiteSpace($root)) {
            $command=Get-Command matlab.exe -CommandType Application -ErrorAction SilentlyContinue | Select-Object -First 1
            if($command) { $root=Split-Path (Split-Path $command.Source -Parent) -Parent }
        }
    }
}
if([string]::IsNullOrWhiteSpace($root) -or -not (Test-Path -LiteralPath $root -PathType Container)) {
    throw "Configure $setting with an existing installation directory."
}
$root=[IO.Path]::GetFullPath($root).TrimEnd([char[]]'\/')
$value=[IO.Path]::GetFullPath((Join-Path $root $Relative))
if(-not ($value.Equals($root,[StringComparison]::OrdinalIgnoreCase) -or $value.StartsWith($root+[IO.Path]::DirectorySeparatorChar,[StringComparison]::OrdinalIgnoreCase))) {
    throw 'Tool path must remain within the configured installation.'
}
if(-not (Test-Path -LiteralPath $value)) { throw "Required tool or SDK path was not found: $value" }
$value
