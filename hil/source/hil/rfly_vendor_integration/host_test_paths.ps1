# Shared path validation for offline component tests.
function Resolve-GpenmpcTestInput {
    param([string]$Value,[string]$Name,[switch]$Directory)
    if ([string]::IsNullOrWhiteSpace($Value) -or ![IO.Path]::IsPathRooted($Value)) {
        throw "$Name must be an explicit absolute path."
    }
    $kind = if ($Directory) {'Container'} else {'Leaf'}
    if (!(Test-Path -LiteralPath $Value -PathType $kind)) { throw "Missing $Name input: $Value" }
    return (Resolve-Path -LiteralPath $Value).ProviderPath
}
function New-GpenmpcTestOutput {
    param([string]$Value,[string[]]$InputRoots)
    if ([string]::IsNullOrWhiteSpace($Value) -or ![IO.Path]::IsPathRooted($Value)) {
        throw 'OutputDirectory must be an explicit absolute path.'
    }
    $resolved = [IO.Path]::GetFullPath($Value).TrimEnd('\','/')
    foreach ($inputRoot in $InputRoots) {
        if (!$inputRoot) { continue }
        $inputPath = [IO.Path]::GetFullPath($inputRoot).TrimEnd('\','/')
        if ($resolved.Equals($inputPath,[StringComparison]::OrdinalIgnoreCase) -or
            $resolved.StartsWith($inputPath+[IO.Path]::DirectorySeparatorChar,[StringComparison]::OrdinalIgnoreCase)) {
            throw 'OutputDirectory must be outside all input/source directories.'
        }
    }
    if (Test-Path -LiteralPath $resolved) { throw 'OutputDirectory already exists; choose a new directory.' }
    New-Item -ItemType Directory -Path $resolved | Out-Null
    return $resolved
}
