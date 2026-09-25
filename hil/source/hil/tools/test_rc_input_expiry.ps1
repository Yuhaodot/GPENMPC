param([Parameter(Mandatory=$true)][string]$OutputDirectory)
$ErrorActionPreference='Stop'
$project=Split-Path $PSScriptRoot -Parent
$vi=Join-Path $project 'rfly_vendor_integration'
$out=[IO.Path]::GetFullPath($OutputDirectory)
$allowed=([IO.Path]::GetFullPath((& (Join-Path $PSScriptRoot 'gpenmpc_external_path.ps1') 'control_host_test_output_root'))).TrimEnd('\')+'\'
if(-not $out.StartsWith($allowed,[StringComparison]::OrdinalIgnoreCase)){throw 'Use a new directory under the configured host-test output root.'}
if(Test-Path -LiteralPath $out){throw 'Preserve prior test result; select a new output directory.'}
$cc=(& (Join-Path $PSScriptRoot 'gpenmpc_install_path.ps1') 'llvm' 'bin\clang++.exe')
if(-not (Test-Path -LiteralPath $cc)){throw 'Existing host compiler is required.'}
$null=New-Item -ItemType Directory -Path $out
$actual=Join-Path $vi 'application_integration\build_fmuv6c'
$flags=@('-std=c++14','-O2','-Wall','-Wextra','-Werror','-ffp-contract=off','-fno-fast-math',
 '-include',(Join-Path $vi 'px4_wire\local_context_host_decl.hpp'),
 '-I',(Join-Path $vi 'local_source_ingress\selected_host_stub'),'-I',(Join-Path $vi 'px4_wire\pump_host_stub'),
 '-I',(& (Join-Path $PSScriptRoot 'gpenmpc_external_path.ps1') 'px4_library_source'),'-I',$actual,
 '-isystem',(Join-Path $actual 'mavlink'),'-isystem',(Join-Path $actual 'mavlink\common'),
 '-Wno-address-of-packed-member','-DGPENMPC_CANONICAL_CLOSED_EVIDENCE=1','-DGPENMPC_CANONICAL_LEARNING_AUDIT=1')
$units=@(
 @('io','px4_runtime\Px4CanonicalLocalIo.cpp'),@('cycle','px4_runtime\CanonicalLocalExecutionCycle.cpp'),
 @('reader','local_source_ingress\Px4OriginalHilReceiptReader.cpp'),@('selected','local_source_ingress\Px4SelectedSourceReader.cpp'),
 @('core','px4_runtime\CanonicalLocalExchangeCore.cpp'),@('outbox','px4_runtime\CanonicalLocalWireOutbox.cpp'),
 @('window_assembler','px4_wire\CanonicalLocalWindowAssembler.cpp'),@('window_wire','px4_wire\CanonicalLocalWindowWire.cpp'),
 @('task_input','px4_runtime\CanonicalLocalTaskInputOwner.cpp'))
$test=Join-Path $PSScriptRoot 'test_rc_input_expiry.cpp'
$facade=Join-Path $out 'facade.o'
$facadeSource=Join-Path $vi 'full_inner_abi\CanonicalFullInnerAbi.cpp'
$generated=Join-Path $project 'evidence\tracking_controller\generated'
$archive=Join-Path $vi 'full_inner_abi\controller_identity\private\libcanonical_local74_private.a'
$oracle=Join-Path $vi 'full_inner_abi\controller_identity\closed_oracle.o'
$phase=Join-Path $project 'evidence\phase_controller\native\libcanonical_local_phase.a'
$rwi=Join-Path (& (Join-Path $PSScriptRoot 'gpenmpc_external_path.ps1') 'canonical_combined_numerics') 'CONTINUOUS_REFERENCE_FIXTURE.bin'
$sjc=(& (Join-Path $PSScriptRoot 'gpenmpc_external_path.ps1') 'source_gp_joint_chain_fixture')
$tracked=@($PSCommandPath,$test,$facadeSource,(Join-Path $vi 'full_inner_abi\CanonicalFullInnerAbi.h'),$archive,$oracle,$phase,$rwi,$sjc,
 (Join-Path $vi 'test_local_task_input_owner.cpp'),(Join-Path $vi 'test_local_exchange_core.cpp'),
 (Join-Path $vi 'test_local_gp_pending.cpp'),(Join-Path $vi 'test_local_execution_cycle.cpp'),
 (Join-Path $vi 'full_inner_abi\test_full_inner_abi.cpp'),
 (Join-Path $actual 'uORB\topics\gpenmpc_full_inner_ingress.h'))
foreach($dir in @('px4_runtime','px4_wire','local_source_ingress','clock_tap_overlay')){
 $tracked+=@(Get-ChildItem -LiteralPath (Join-Path $vi $dir) -File | Where-Object {$_.Extension -in @('.hpp','.cpp')} | ForEach-Object {$_.FullName})
}
$tracked+=@(Get-ChildItem -LiteralPath $vi -Filter '*.hpp' -File|ForEach-Object {$_.FullName})
$generatedFiles=@(Get-ChildItem -LiteralPath $generated -File|Where-Object {$_.Extension -in @('.c','.h')}|Sort-Object Name)
$facadeFiles=@($facadeSource,(Join-Path $vi 'full_inner_abi\CanonicalFullInnerAbi.h'),
 (Join-Path $vi 'CanonicalCombinedSymbolNamespace.h'),(Join-Path $vi 'CanonicalLocalInnerStateStore.hpp'),
 (Join-Path $vi 'CanonicalReferenceStateStore.hpp'),(Join-Path $vi 'CanonicalJointStateInstaller.hpp'),
 (Join-Path $vi 'CanonicalOperatorReference.hpp'),(Join-Path $project 'px4_full_inner\consumption\CanonicalSha256.hpp'),
 (Join-Path $project 'px4_full_inner\portable\CanonicalPortable.hpp'))|ForEach-Object{Get-Item -LiteralPath $_}|Sort-Object Name
$tracked+=@($facadeFiles.FullName)+@($generatedFiles.FullName)
$tracked=@($tracked|Sort-Object -Unique)
$before=@(Get-FileHash -LiteralPath $tracked -Algorithm SHA256|Select-Object Path,Hash)
$records=@();$objects=@();$ok=$true
# Recompile the current facade with the retained numerical library.
function SourceSetHash([string]$Domain,[object[]]$Files){
 $data=[Collections.Generic.List[byte]]::new();$data.AddRange([Text.Encoding]::UTF8.GetBytes($Domain+[char]0))
 foreach($f in $Files){$data.AddRange([Text.Encoding]::UTF8.GetBytes($f.Name+[char]0));$data.AddRange([Convert]::FromHexString((Get-FileHash -LiteralPath $f.FullName).Hash))}
 return [Convert]::ToHexString([Security.Cryptography.SHA256]::HashData($data.ToArray()))
}
$identity=[ordered]@{generated_source_set_sha256=(SourceSetHash 'GPENMPC_GENERATED_SOURCE_SET_V1' $generatedFiles);
 facade_source_set_sha256=(SourceSetHash 'GPENMPC_FULL_INNER_FACADE_SOURCE_SET_V1' $facadeFiles);
 private_archive_sha256=(Get-FileHash -LiteralPath $archive).Hash}
$header="#pragma once`n"
foreach($entry in @(@('rfi_generated_source_sha',$identity.generated_source_set_sha256),@('rfi_facade_source_sha',$identity.facade_source_set_sha256),@('rfi_private_archive_sha',$identity.private_archive_sha256))){
 $values=@([Convert]::FromHexString($entry[1])|ForEach-Object{'0x'+$_.ToString('X2')}) -join ','
 $header+='static const unsigned char '+$entry[0]+'[32]={'+$values+'};'+"`n"
}
[IO.File]::WriteAllText((Join-Path $out 'FullInnerBuildIdentity.h'),$header)
$facadeArgs=@('-std=c++14','-O2','-Wall','-Wextra','-Werror','-ffp-contract=off','-fno-fast-math','-fno-exceptions','-fno-rtti',
 '-DGPENMPC_CANONICAL_EXPLICIT_WORKSPACE=1','-DGPENMPC_CANONICAL_CLOSED_EVIDENCE=1','-DGPENMPC_CANONICAL_LEARNING_AUDIT=1',
 '-DGPENMPC_OPERATOR_YAW_REFERENCE=1','-I',$out,'-I',$generated,'-isystem',(& (Join-Path $PSScriptRoot 'gpenmpc_install_path.ps1') 'matlab' 'extern\include'),
 '-fstack-usage','-c',$facadeSource,'-o',$facade)
$facadeLog=@(& $cc @facadeArgs 2>&1|ForEach-Object {"$_"});$facadeExit=$LASTEXITCODE
$records+=@([ordered]@{name='current_facade';command=$facadeArgs;exit=$facadeExit;output=$facadeLog})
if($facadeExit -ne 0){$ok=$false}
foreach($unit in $units){
 if(-not $ok){break}
 $source=Join-Path $vi $unit[1];$obj=Join-Path $out ($unit[0]+'.o');$compileArgs=$flags+@('-fstack-usage')
 if($unit[0] -eq 'io'){$compileArgs+=@('-Dgpenmpc_full_inner_commit=gpenmpc_test_call_full_inner_commit','-Dgpenmpc_full_inner_copy_state=gpenmpc_test_call_full_inner_copy_state')}
 $compileArgs+=@('-c',$source,'-o',$obj)
 $log=@(& $cc @compileArgs 2>&1|ForEach-Object {"$_"});$exitCode=$LASTEXITCODE
 $records+=@([ordered]@{name=$unit[0];command=$compileArgs;exit=$exitCode;output=$log})
 if($exitCode -ne 0){$ok=$false;break};$objects+=@($obj)
}
$exe=Join-Path $out 'test_rc_input_expiry.exe';$linkExit=-1;$runExit=-1;$link=@();$run=@();$result=$null
$linkArgs=$flags+@('-static','-municode',$test)+$objects+@($facade,$archive,$phase,$oracle,
 '-Wl,--wrap=gpenmpcDesiredSe3Command','-Wl,--wrap=se3WrenchKernel','-o',$exe)
if($ok){
 $link=@(& $cc @linkArgs 2>&1|ForEach-Object {"$_"});$linkExit=$LASTEXITCODE
 if($linkExit -eq 0){
  $run=@(& $exe $rwi $sjc (Get-FileHash -LiteralPath $sjc -Algorithm SHA256).Hash 2>&1|ForEach-Object {"$_"});$runExit=$LASTEXITCODE
  $json=@($run|Where-Object {$_ -like '{*'});if($json.Count -eq 1){$result=$json[0]|ConvertFrom-Json}
 }
}
$after=@(Get-FileHash -LiteralPath $tracked -Algorithm SHA256|Select-Object Path,Hash)
$stable=($before|ConvertTo-Json -Compress) -ceq ($after|ConvertTo-Json -Compress)
$report=[ordered]@{scope='RC_INPUT_EXPIRY_OFFLINE';passed=($ok -and $linkExit -eq 0 -and $runExit -eq 0 -and $stable);
 records=$records;link_command=$linkArgs;link_exit=$linkExit;link_output=$link;run_exit=$runExit;run_output=$run;result=$result;
 source_before=$before;source_stable=$stable;facade_identity=$identity;generated_C_recompiled=0;assembly_limit_us=50000;hardware_actions=0;COM=0;
 boundary='Task owner and shared ingress with a recorded source fixture and mocked clock, uORB, session and publication.'}
[IO.File]::WriteAllText((Join-Path $out 'RESULT.json'),($report|ConvertTo-Json -Depth 12)+"`n")
foreach($record in $records){if($record.exit -ne 0){$record.output}}
$link;$run
if(-not $report.passed){exit 1}
