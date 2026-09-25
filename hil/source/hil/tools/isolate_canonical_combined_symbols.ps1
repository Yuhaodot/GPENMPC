param([string]$OutputRoot='')
$ErrorActionPreference='Stop'
$build=Split-Path -Parent $PSScriptRoot
$combined=(& (Join-Path $PSScriptRoot 'gpenmpc_external_path.ps1') 'canonical_combined_numerics')
if(-not $OutputRoot){$OutputRoot=Join-Path $combined 'symbol_isolation'}
if(Test-Path -LiteralPath $OutputRoot){throw 'Output exists; preserve prior diagnostic'}
[void](New-Item -ItemType Directory -Path $OutputRoot)
$tool=(& (Join-Path $PSScriptRoot 'gpenmpc_install_path.ps1') 'llvm' 'bin')
$nm=Join-Path $tool 'llvm-nm.exe'
$objcopy=Join-Path $tool 'llvm-objcopy.exe'
$ar=Join-Path $tool 'llvm-ar.exe'
$header=Join-Path $build 'rfly_vendor_integration\CanonicalCombinedSymbolNamespace.h'
$definitions=@(Get-Content -LiteralPath $header|Where-Object{$_ -match '^#define rt'}|ForEach-Object{
    $p=$_ -split '\s+';[pscustomobject]@{old=$p[1];new=$p[2]}})
if($definitions.Count -ne 17){throw 'Explicit runtime-only mapping changed'}
$records=@();$checks=@();$inputs=@()
foreach($kind in @('native','arm_build')){
    $inputRoot=Join-Path $combined $kind
    $output=Join-Path $OutputRoot $kind
    [void](New-Item -ItemType Directory -Path $output)
    $objects=@(Get-ChildItem -LiteralPath $inputRoot -File -Filter '*.o'|Where-Object{
        Test-Path -LiteralPath (Join-Path $combined ('generated\'+$_.BaseName+'.c'))}|Sort-Object Name)
    if($objects.Count -ne 74){throw "Expected exact 74 actual $kind generated objects"}
    $mapped=@();$symbolMap=[Collections.Generic.List[string]]::new()
    foreach($d in $definitions){$symbolMap.Add($d.old+' '+$d.new)}
    # Rename COFF reference-pointer COMDAT keys with their symbols;
    # leave libc and libm names unchanged.
    if($kind -eq 'native'){
        foreach($d in $definitions){$symbolMap.Add('.refptr.'+$d.old+' .refptr.'+$d.new)}
    }
    $mapPath=Join-Path $output 'REDEFINE_SYMBOLS.txt'
    [IO.File]::WriteAllLines($mapPath,$symbolMap,[Text.UTF8Encoding]::new($false))
    foreach($o in $objects){
        $before=(Get-FileHash -LiteralPath $o.FullName -Algorithm SHA256).Hash
        $destination=Join-Path $output $o.Name
        $messages=& $objcopy ('--redefine-syms='+$mapPath) $o.FullName $destination 2>&1
        if($LASTEXITCODE -ne 0){$messages|Write-Output;throw "Object isolation failed: $($o.Name)"}
        if((Get-FileHash -LiteralPath $o.FullName -Algorithm SHA256).Hash -ne $before){throw 'Original object changed'}
        $inputs+=@{path=$o.FullName;sha256=$before}
        $mapped+=$destination
        $records+=@{kind=$kind;name=$o.Name;original_sha256=$before;derived_sha256=(Get-FileHash -LiteralPath $destination -Algorithm SHA256).Hash}
    }
    $archive=Join-Path $output 'libcanonical_local74_private.a'
    & $ar rcs $archive @mapped
    if($LASTEXITCODE -ne 0){throw 'Archive creation failed'}
    $symbols=@(& $nm --extern-only $archive 2>&1)
    if($LASTEXITCODE -ne 0){throw 'Archive symbols unavailable'}
    [IO.File]::WriteAllLines((Join-Path $output 'SYMBOLS.txt'),[string[]]$symbols,[Text.UTF8Encoding]::new($false))
    $names=@($symbols|ForEach-Object{if($_ -match '\s[UTDRBCVW]\s+(\S+)$'){$Matches[1]}})
    foreach($d in $definitions){
        $ok=($d.old -notin $names -and ('.refptr.'+$d.old) -notin $names -and $d.new -in $names)
        $checks+=@{kind=$kind;old=$d.old;new=$d.new;isolated=$ok}
        if(-not $ok){throw "Incomplete symbol isolation: $kind/$($d.old)"}
    }
    $records+=@{kind=$kind;archive=$archive;bytes=(Get-Item -LiteralPath $archive).Length;sha256=(Get-FileHash -LiteralPath $archive -Algorithm SHA256).Hash}
}
$receipt=@{scope='OBJECT_SYMBOL_ISOLATION';
    pass=$true;generated_c_recompiled=0;native_objects=74;m7_objects=74;runtime_symbols_per_architecture=17;
    inputs=$inputs;records=$records;checks=$checks;namespace_header=$header;
    namespace_header_sha256=(Get-FileHash -LiteralPath $header -Algorithm SHA256).Hash;
    note='Exact original object contents only re-symbolized in private copies. Numerical coexistence replay and whole-application fit remain separate checks. Coder type headers must still be isolated by translation unit.';
    COM=0;board=0;application_flash=0;model_runs=0}
$receipt|ConvertTo-Json -Depth 7|Out-File -LiteralPath (Join-Path $OutputRoot 'RESULT.json') -Encoding utf8
Write-Output ('148 actual objects isolated; '+$checks.Count+' symbol checks; no original changed')
