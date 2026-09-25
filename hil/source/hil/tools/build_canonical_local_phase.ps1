$ErrorActionPreference='Stop'
$build=Split-Path -Parent $PSScriptRoot
$root=Join-Path $build 'evidence\phase_controller'
$generated=Join-Path $root 'generated'
$out=Join-Path $root 'native'
if(Test-Path -LiteralPath $out){throw 'Preserve existing phase native evidence.'}
New-Item -ItemType Directory -Path $out | Out-Null
$parent=Get-Content -LiteralPath (Join-Path $root 'RESULT.json') -Raw | ConvertFrom-Json
$fixture=Join-Path $root 'ORIGINAL_ADAPTER_PHASE.bin'
if(-not $parent.all_exact -or $parent.case_rows -ne 2400 -or -not $parent.original_sources_unchanged){throw 'Parent phase evidence invalid.'}
foreach($identity in @(@($parent.source,$parent.source_sha256),@($parent.parent_source,$parent.parent_source_sha256),@($fixture,$parent.fixture_sha256))){
    if((Get-FileHash -LiteralPath $identity[0] -Algorithm SHA256).Hash -ne $identity[1]){throw "Phase input identity mismatch $($identity[0])"}
}
$api=Join-Path $build 'rfly_vendor_integration\local_phase\CanonicalLocalPhase.h'
$harness=Join-Path $build 'rfly_vendor_integration\local_phase\phase_replay.c'
$compilerRoot=(& (Join-Path $PSScriptRoot 'gpenmpc_install_path.ps1') 'llvm' 'bin')
$clang=Join-Path $compilerRoot 'clang.exe'
$ar=Join-Path $compilerRoot 'llvm-ar.exe'
$nm=Join-Path $compilerRoot 'llvm-nm.exe'
$flags=@('-std=c11','-O2','-ffp-contract=off','-fno-fast-math','-Wall','-Wextra','-Werror','-fstack-usage',
    '-I',$generated,'-I',(& (Join-Path $PSScriptRoot 'gpenmpc_install_path.ps1') 'matlab' 'extern\include'),'-I',(Split-Path -Parent $api))
$sources=@(Get-ChildItem -LiteralPath $generated -File -Filter '*.c' | Sort-Object Name)
if($sources.Count -ne 3){throw 'Expected exactly 3 generated phase translation units.'}
$inputs=@(Get-ChildItem -LiteralPath $generated -File | Where-Object {$_.Extension -in '.c','.h'})
$inputs+=@(Get-Item -LiteralPath $api,$harness,$fixture,$parent.source,$parent.parent_source,$PSCommandPath)
$before=@($inputs | ForEach-Object {@{path=$_.FullName;bytes=$_.Length;sha256=(Get-FileHash -LiteralPath $_.FullName -Algorithm SHA256).Hash}})
$commands=@();$objects=@()
foreach($source in $sources){
    $object=Join-Path $out ($source.BaseName+'.o')
    $arguments=$flags+@('-c',$source.FullName,'-o',$object)
    $messages=& $clang @arguments 2>&1;$rc=$LASTEXITCODE
    $messages | Out-File -LiteralPath (Join-Path $out 'COMPILE.log') -Encoding utf8 -Append
    $commands+=@{program=$clang;arguments=$arguments;exit_code=$rc}
    if($rc -ne 0){throw "Phase C compile failed $($source.Name)"}
    $objects+=$object
}
$library=Join-Path $out 'libcanonical_local_phase.a'
& $ar rcs $library @objects
if($LASTEXITCODE -ne 0){throw 'Phase archive failed.'}
$exe=Join-Path $out 'phase_replay.exe'
$arguments=$flags+@($harness,$library,'-municode','-o',$exe)
$messages=& $clang @arguments 2>&1;$rc=$LASTEXITCODE
$messages | Out-File -LiteralPath (Join-Path $out 'COMPILE.log') -Encoding utf8 -Append
$commands+=@{program=$clang;arguments=$arguments;exit_code=$rc}
if($rc -ne 0){throw 'Phase native replay compile failed.'}
$actual=Join-Path $out 'ACTUAL_PHASE.bin'
$numeric=Join-Path $out 'NUMERICAL_RESULT.json'
$messages=& $exe $fixture $actual $numeric 2>&1;$rc=$LASTEXITCODE
$messages | Out-File -LiteralPath (Join-Path $out 'EXECUTION.log') -Encoding utf8
$messages | Write-Output
$commands+=@{program=$exe;arguments=@($fixture,$actual,$numeric);exit_code=$rc}
$symbols=& $nm '--defined-only' '--extern-only' $library 2>&1
$symbols | Out-File -LiteralPath (Join-Path $out 'DEFINED_SYMBOLS.txt') -Encoding utf8
if($LASTEXITCODE -ne 0){throw 'Actual symbol inspection failed.'}
$defined=@($symbols | ForEach-Object {if($_ -match '^\S+\s+[A-Z]\s+(\S+)$'){$matches[1]}})
$expected=@('gpenmpcNative_canonicalLocalPhaseAdvance','gpenmpcNative_canonicalLocalPhaseAdvance_initialize','gpenmpcNative_canonicalLocalPhaseAdvance_terminate')
$unique=(@(Compare-Object ($defined | Sort-Object) ($expected | Sort-Object)).Count -eq 0)
$unchanged=$true
foreach($record in $before){if((Get-FileHash -LiteralPath $record.path -Algorithm SHA256).Hash -ne $record.sha256){$unchanged=$false}}
$equal=((Get-FileHash -LiteralPath $actual -Algorithm SHA256).Hash -eq $parent.fixture_sha256)
$report=@{pass=($rc -eq 0 -and $equal -and $unique -and $unchanged);compiler=$clang;commands=$commands;
    numerical=(Get-Content -LiteralPath $numeric -Raw | ConvertFrom-Json);actual_binary_equals_original_oracle=$equal;
    actual_binary_sha256=(Get-FileHash -LiteralPath $actual -Algorithm SHA256).Hash;
    defined_symbols=$defined;unique_phase_symbols_no_rt_helpers=$unique;source_records=$before;sources_unchanged=$unchanged;
    public_header=$api;library=$library;library_sha256=(Get-FileHash -LiteralPath $library -Algorithm SHA256).Hash;
    scope=$parent.sample_and_outer_scope;board_actions=0;COM=0;solver=0;plant=0;runtime_integration=$false}
$report | ConvertTo-Json -Depth 8 | Out-File -LiteralPath (Join-Path $out 'RESULT.json') -Encoding utf8
if(-not $report.pass){throw 'Phase native evidence failed.'}
