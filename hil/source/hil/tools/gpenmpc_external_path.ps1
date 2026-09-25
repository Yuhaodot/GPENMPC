param([Parameter(Mandatory=$true)][string]$Key)
$ErrorActionPreference='Stop'
$configPath=$env:GPENMPC_EXTERNAL_PATHS
if([string]::IsNullOrWhiteSpace($configPath) -or -not(Test-Path -LiteralPath $configPath -PathType Leaf)){
    throw 'Set GPENMPC_EXTERNAL_PATHS to the dependency configuration JSON.'
}
$configuration=Get-Content -LiteralPath $configPath -Raw|ConvertFrom-Json
$property=$configuration.PSObject.Properties[$Key]
if($null -eq $property -or [string]::IsNullOrWhiteSpace([string]$property.Value)){
    throw "Configure dependency $Key."
}
[string]$property.Value
