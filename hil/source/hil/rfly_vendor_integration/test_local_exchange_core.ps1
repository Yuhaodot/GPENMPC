param(
 [Parameter(Mandatory=$true)][string]$CompilerPath,
 [Parameter(Mandatory=$true)][string]$OutputDirectory,
 [string]$SourceRoot=$env:GPENMPC_SOURCE_ROOT,
 [Parameter(Mandatory=$true)][string]$Px4BuildDirectory,
 [string]$ReferenceFixtures=$env:GPENMPC_REFERENCE_FIXTURES,
 [string]$GpChainFixture=$env:GPENMPC_GP_CHAIN_FIXTURE,
 [Parameter(Mandatory=$true)][string]$GpLibrary,
 [Parameter(Mandatory=$true)][string]$FacadeObject,
 [string]$KernelArchive=$env:GPENMPC_KERNEL_ARCHIVE,
 [switch]$WithClosedEvidence,[switch]$WithLearningAudit,[switch]$ComponentCadenceOnly,
 [Parameter(Mandatory=$true)][string]$PhaseArchive,
 [string]$ClosedOracleObject='',
 [string]$Px4Root=$env:GPENMPC_PX4_ROOT
)
$ErrorActionPreference='Stop'
. (Join-Path $PSScriptRoot 'host_test_paths.ps1')
if (!$SourceRoot) { $SourceRoot=Split-Path $PSScriptRoot -Parent }
$build=Resolve-GpenmpcTestInput $SourceRoot 'SourceRoot' -Directory
$vi=Join-Path $build 'rfly_vendor_integration'
$cc=Resolve-GpenmpcTestInput $CompilerPath 'CompilerPath'
$actual=Resolve-GpenmpcTestInput $Px4BuildDirectory 'Px4BuildDirectory' -Directory
if($WithLearningAudit){$WithClosedEvidence=$true}
$rwi=Resolve-GpenmpcTestInput (Join-Path (Resolve-GpenmpcTestInput $ReferenceFixtures 'ReferenceFixtures' -Directory) 'CONTINUOUS_REFERENCE_FIXTURE.bin') 'reference fixture'
$sjc=Resolve-GpenmpcTestInput $GpChainFixture 'GpChainFixture'
$gp=Resolve-GpenmpcTestInput $GpLibrary 'GpLibrary'
$facade=Resolve-GpenmpcTestInput $FacadeObject 'FacadeObject'
$archive=Resolve-GpenmpcTestInput $KernelArchive 'KernelArchive'
$out=New-GpenmpcTestOutput $OutputDirectory @($build,$actual,(Split-Path $cc -Parent))
$phase=Resolve-GpenmpcTestInput $PhaseArchive 'PhaseArchive'
$px4=Resolve-GpenmpcTestInput $Px4Root 'Px4Root' -Directory
$oracle='';if($WithClosedEvidence){$oracle=Resolve-GpenmpcTestInput $ClosedOracleObject 'ClosedOracleObject'}
$io=Join-Path $vi 'px4_runtime\Px4CanonicalLocalIo.cpp';$cycle=Join-Path $vi 'px4_runtime\CanonicalLocalExecutionCycle.cpp'
$reader=Join-Path $vi 'local_source_ingress\Px4OriginalHilReceiptReader.cpp';$test=Join-Path $vi 'test_local_exchange_core.cpp'
$core=Join-Path $vi 'px4_runtime\CanonicalLocalExchangeCore.cpp';$outbox=Join-Path $vi 'px4_runtime\CanonicalLocalWireOutbox.cpp';$selected=Join-Path $vi 'local_source_ingress\Px4SelectedSourceReader.cpp'
$windowAssembler=Join-Path $vi 'px4_wire\CanonicalLocalWindowAssembler.cpp';$windowWire=Join-Path $vi 'px4_wire\CanonicalLocalWindowWire.cpp'
$tracked=@($core,$outbox,$selected,$windowAssembler,$windowWire,(Join-Path $vi 'px4_wire\CanonicalLocalWindowAssembler.hpp'),(Join-Path $vi 'px4_wire\CanonicalLocalWindowWire.hpp'),(Join-Path $vi 'px4_runtime\CanonicalLocalExchangeCore.hpp'),$io,$cycle,$reader,$test,$PSCommandPath,$facade,$archive,$phase,$rwi,$sjc,$gp,
 (Join-Path $vi 'px4_wire\CanonicalLocalIngress.hpp'),(Join-Path $vi 'px4_wire\CanonicalLocalGpWire.hpp'),
 (Join-Path $vi 'px4_runtime\CanonicalLocalGpPending.hpp'),(Join-Path $vi 'test_local_gp_pending.cpp'),(Join-Path $vi 'test_local_execution_cycle.cpp'),
 (Join-Path $vi 'px4_runtime\CanonicalLocalExecutionCycle.hpp'),(Join-Path $vi 'px4_runtime\Px4CanonicalLocalIo.hpp'),(Join-Path $vi 'px4_runtime\LocalCommittedState.hpp'),
 (Join-Path $build 'px4_full_inner\px4_ingress\GPENMPCFullInnerIngress.hpp'),(Join-Path $build 'px4_full_inner\argument_transport\CanonicalArgumentTransport.hpp'),
 (Join-Path $build 'px4_full_inner\px4_state_adapter\AtomicOdometryAdapter.hpp'),(Join-Path $actual 'uORB\topics\gpenmpc_full_inner_ingress.h'))
$tracked+=@((Join-Path $vi 'px4_wire\CanonicalLocalCommittedWire.hpp'),(Join-Path $vi 'px4_runtime\CanonicalLocalWireOutbox.hpp'))
if($WithClosedEvidence){$tracked+=@($oracle)}
$before=@(Get-FileHash -LiteralPath $tracked -Algorithm SHA256|Select-Object Path,Hash)
$flags=@('-std=c++14','-O2','-Wall','-Wextra','-Werror','-ffp-contract=off','-fno-fast-math','-include',(Join-Path $vi 'px4_wire\local_context_host_decl.hpp'),
 '-I',(Join-Path $vi 'local_source_ingress\selected_host_stub'),'-I',(Join-Path $vi 'px4_wire\pump_host_stub'),'-I',(Join-Path $px4 'src/lib'),'-I',$actual,'-isystem',(Join-Path $actual 'mavlink'),'-isystem',(Join-Path $actual 'mavlink\common'),'-Wno-address-of-packed-member')
if($WithClosedEvidence){$flags+=@('-DGPENMPC_CANONICAL_CLOSED_EVIDENCE=1')}
if($WithLearningAudit){$flags+=@('-DGPENMPC_CANONICAL_LEARNING_AUDIT=1')}
$records=@();$objects=@();$ok=$true
foreach($item in @(@('io',$io),@('cycle',$cycle),@('reader',$reader),@('selected',$selected),@('core',$core),@('outbox',$outbox),@('window_assembler',$windowAssembler),@('window_wire',$windowWire))){
 $obj=Join-Path $out ($item[0]+'.o');$args=$flags+@('-fstack-usage')
 if($item[0] -eq 'io'){$args+=@('-Dgpenmpc_full_inner_commit=gpenmpc_test_call_full_inner_commit','-Dgpenmpc_full_inner_copy_state=gpenmpc_test_call_full_inner_copy_state')}
 $args+=@('-c',$item[1],'-o',$obj)
 $log=@(& $cc @args 2>&1|ForEach-Object {"$_"});$rc=$LASTEXITCODE
 $records+=@([ordered]@{name=$item[0];command=$args;exit=$rc;output=$log})
 if($rc -ne 0){$ok=$false;break};$objects+=@($obj)
}
$exe=Join-Path $out 'test.exe';$link=@();$run=@();$le=-1;$rc=-1;$result=$null;$linkArgs=@()
if($ok){
 $linkArgs=$flags+@('-static','-municode',$test)+$objects+@($facade,$archive,$phase,'-o',$exe)
 if($WithClosedEvidence){$linkArgs+=@($oracle)}
 $link=@(& $cc @linkArgs 2>&1|ForEach-Object {"$_"});$le=$LASTEXITCODE
 if($le -eq 0){$exeArgs=@($rwi,$sjc,$gp,(Get-FileHash -LiteralPath $sjc -Algorithm SHA256).Hash)
  if($WithClosedEvidence){$wireName='COMMITTED_RLC1.bin';if($WithLearningAudit){$wireName='COMMITTED_RLC2.bin'};$exeArgs+=@(Join-Path $out $wireName)}
  $previousCadence=$env:GPENMPC_COMPONENT_CADENCE_ONLY
  try{if($ComponentCadenceOnly){$env:GPENMPC_COMPONENT_CADENCE_ONLY='1'}
   $run=@(& $exe @exeArgs 2>&1|ForEach-Object {"$_"});$rc=$LASTEXITCODE
  }finally{$env:GPENMPC_COMPONENT_CADENCE_ONLY=$previousCadence}
  $json=@($run|Where-Object {$_ -like '{*'});if($json.Count -eq 1){$result=$json[0]|ConvertFrom-Json}}
}
$after=@(Get-FileHash -LiteralPath $tracked -Algorithm SHA256|Select-Object Path,Hash)
$stable=($before|ConvertTo-Json -Compress) -ceq ($after|ConvertTo-Json -Compress)
$receipt=[ordered]@{scope='ACTUAL_LOCAL_CORE_SELECTED_SOURCE_CYCLE_GP_OUTBOX_INGRESS_HOST';records=$records;link_command=$linkArgs;
 link_exit=$le;link_output=$link;run_exit=$rc;run_output=$run;result=$result;source_before=$before;source_stable=$stable;
 boundary='Offline fixtures for clocks, uORB, output permissions and publication.';
 COM=0;closed_evidence_variant=[bool]$WithClosedEvidence;learning_audit_variant=[bool]$WithLearningAudit;component_cadence_only=[bool]$ComponentCadenceOnly}
[IO.File]::WriteAllText((Join-Path $out 'RESULT.json'),($receipt|ConvertTo-Json -Depth 12)+"`n")
foreach($r in $records){if($r.exit -ne 0){$r.output}}
$link;$run
if(!$ok -or $le -ne 0 -or $rc -ne 0 -or !$stable){exit 1}
