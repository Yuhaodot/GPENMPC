param([Parameter(Mandatory=$true)][string]$OutputDirectory)
$ErrorActionPreference='Stop'
$project=Split-Path $PSScriptRoot -Parent
$out=[IO.Path]::GetFullPath($OutputDirectory)
$allowed=(& (Join-Path $PSScriptRoot 'gpenmpc_external_path.ps1') 'control_host_test_output_root')
if(-not $out.StartsWith($allowed,[StringComparison]::OrdinalIgnoreCase)){throw 'Use the configured host-test output root.'}
if(Test-Path -LiteralPath $out){throw 'Preserve previous evidence.'}
$source=Join-Path $PSScriptRoot 'test_usb_operator_reference.cpp'
$header=Join-Path $project 'rfly_vendor_integration\CanonicalOperatorReference.hpp'
$archive=Join-Path $project 'rfly_vendor_integration\full_inner_abi\controller_identity\private\libcanonical_local74_private.a'
$tracked=@($PSCommandPath,$source,$header,$archive)
$before=@(Get-FileHash -LiteralPath $tracked -Algorithm SHA256|Select-Object Path,Hash)
$null=New-Item -ItemType Directory -Path $out
$compiler=(& (Join-Path $PSScriptRoot 'gpenmpc_install_path.ps1') 'llvm' 'bin\clang++.exe')
$exe=Join-Path $out 'test_manual_reference_interval.exe'
$compileArgs=@('-std=c++14','-O2','-Wall','-Wextra','-Werror','-ffp-contract=off','-fno-fast-math','-static',
 '-isystem',(& (Join-Path $PSScriptRoot 'gpenmpc_install_path.ps1') 'matlab' 'extern\include'),$source,$archive,'-o',$exe)
$buildLog=@(& $compiler @compileArgs 2>&1|ForEach-Object{"$_"});$buildExit=$LASTEXITCODE
$runLog=@();$runExit=-1
if($buildExit -eq 0){$runLog=@(& $exe 2>&1|ForEach-Object{"$_"});$runExit=$LASTEXITCODE}
$after=@(Get-FileHash -LiteralPath $tracked -Algorithm SHA256|Select-Object Path,Hash)
$stable=($before|ConvertTo-Json -Compress) -ceq ($after|ConvertTo-Json -Compress)
$result=[ordered]@{scope='HOST_MANUAL_REFERENCE_INTERVAL_NUMERICS';passed=($buildExit -eq 0 -and $runExit -eq 0 -and $stable);
 compiler=$compiler;command=$compileArgs;compile_exit=$buildExit;compile_output=$buildLog;run_exit=$runExit;run_output=$runLog;
 source_before=$before;source_stable=$stable;manual_interval_s=@(.4,.401,.408236,1.,20.);numerical_minimum_s=.002;
 autonomous_50001_us_rejected=$true;hardware_actions=0;COM=0;claim='HOST reference, filter, yaw and allocator unit tests.'}
[IO.File]::WriteAllText((Join-Path $out 'RESULT.json'),($result|ConvertTo-Json -Depth 8)+"`n")
$buildLog;$runLog
if(-not $result.passed){exit 1}
