param([Parameter(Mandatory=$true)][string]$OutputRoot,[string]$ModelDll='',[string]$ModelSha='',[switch]$GetterRing,[switch]$StepSnapshot)
$ErrorActionPreference='Stop'
if(Test-Path -LiteralPath $OutputRoot){throw 'Choose an unused output path.'}
if($GetterRing -and -not $ModelDll){throw 'Ring requires exact custom diagnostic DLL.'}
if($StepSnapshot -and -not $GetterRing){throw 'Immutable step snapshot status requires same RDR1 owner.'}
$build=Split-Path -Parent $PSScriptRoot
$vendor=(& (Join-Path $PSScriptRoot 'gpenmpc_install_path.ps1') 'rfly' 'CopterSim')
$source=Join-Path $PSScriptRoot 'probe_coptersim_software_carrier.cpp'
$expected='94B81EFB44058176DD5353669D9C28FC5331CC8411AB9EA3F2D27C1E8C343241'
if((Get-FileHash -LiteralPath (Join-Path $vendor 'CopterSimNoUI.exe') -Algorithm SHA256).Hash -ne $expected){throw 'Vendor executable differs from inspected TCP-mode binary.'}
if(@(Get-Process -Name CopterSim,CopterSimNoUI -ErrorAction SilentlyContinue).Count){throw 'Existing CopterSim process: leave untouched.'}
New-Item -ItemType Directory -Path $OutputRoot|Out-Null
$runtime=Join-Path $OutputRoot 'runtime'
New-Item -ItemType Directory -Path $runtime|Out-Null
New-Item -ItemType Directory -Path (Join-Path $runtime 'external\model') -Force|Out-Null
$files=@('CopterSimNoUI.exe','Qt5Core.dll','Qt5Gui.dll','Qt5Network.dll','Qt5SerialPort.dll','ModelData.db','Wgs84.dll')
$inputs=@()
$modelName='0'
if($ModelDll){
 if(-not $ModelSha -or (Get-FileHash -LiteralPath $ModelDll -Algorithm SHA256).Hash -ne $ModelSha){throw 'Exact diagnostic model identity required.'}
 $modelName='GPENMPC_M600_Diagnostic'
 $target=Join-Path $runtime 'external\model\GPENMPC_M600_Diagnostic.dll'
 Copy-Item -LiteralPath $ModelDll -Destination $target
 if((Get-FileHash -LiteralPath $target -Algorithm SHA256).Hash -ne $ModelSha){throw 'Isolated model copy mismatch.'}
 $inputs+=[ordered]@{path=$ModelDll;copy=$target;bytes=(Get-Item -LiteralPath $ModelDll).Length;sha256=$ModelSha}
}
foreach($name in $files){
 $parent=Join-Path $vendor $name;$target=Join-Path $runtime $name
 Copy-Item -LiteralPath $parent -Destination $target
 $sha=(Get-FileHash -LiteralPath $parent -Algorithm SHA256).Hash
 if((Get-FileHash -LiteralPath $target -Algorithm SHA256).Hash -ne $sha){throw 'Isolated copy mismatch.'}
 $inputs+=[ordered]@{path=$parent;copy=$target;bytes=(Get-Item -LiteralPath $parent).Length;sha256=$sha}
}
Copy-Item -LiteralPath (Join-Path $vendor 'platforms') -Destination (Join-Path $runtime 'platforms') -Recurse
$native=Join-Path $OutputRoot 'software_carrier.exe'
$clang=(& (Join-Path $PSScriptRoot 'gpenmpc_install_path.ps1') 'llvm' 'bin\clang++.exe')
$args=@('-std=c++14','-O2','-Wall','-Wextra','-Werror','-Wno-address-of-packed-member','-static','-municode','-I',(Join-Path $build 'rfly_vendor_integration\application_integration\build_fmuv6c'),$source,'-lws2_32','-liphlpapi','-o',$native)
$log=& $clang @args 2>&1;$rc=$LASTEXITCODE
$log|Set-Content -LiteralPath (Join-Path $OutputRoot 'COMPILE_LOG.txt')
if($rc -ne 0){throw "Software peer compile failed $rc; no CopterSim started."}
$env:OMP_NUM_THREADS='1';$env:MKL_NUM_THREADS='1';$env:OPENBLAS_NUM_THREADS='1'
$env:QT_FORCE_STDERR_LOGGING='1'
$env:QT_MESSAGE_PATTERN='[%{time process} %{type} pid=%{pid} tid=%{threadid}] %{message}'
$peerArgs=@((Join-Path $runtime 'CopterSimNoUI.exe'),$runtime,$OutputRoot,$modelName)
if($StepSnapshot){$peerArgs+='RDR1_SSS1'}elseif($GetterRing){$peerArgs+='RDR1'}
$log=& $native @peerArgs 2>&1;$rc=$LASTEXITCODE
$log|Set-Content -LiteralPath (Join-Path $OutputRoot 'PEER_LOG.txt')
$rp=Join-Path $OutputRoot 'NUMERICAL_RESULT.json'
$observed=if(Test-Path -LiteralPath $rp){Get-Content -LiteralPath $rp -Raw|ConvertFrom-Json}else{[pscustomobject]@{failure='NATIVE_EARLY_EXIT';exit_code=$rc}}
$tapPath=Join-Path $OutputRoot 'GETTER_WIRE_OBSERVATION.json'
$tap=if(Test-Path -LiteralPath $tapPath){Get-Content -LiteralPath $tapPath -Raw|ConvertFrom-Json}else{$null}
$vendorActionsPath=Join-Path $OutputRoot 'VENDOR_FAKE_PEER_ACTIONS.json'
$vendorActions=if(Test-Path -LiteralPath $vendorActionsPath){Get-Content -LiteralPath $vendorActionsPath -Raw|ConvertFrom-Json}else{$null}
$ringPath=Join-Path $OutputRoot 'GETTER_RING_OBSERVATION.json'
$ring=if(Test-Path -LiteralPath $ringPath){Get-Content -LiteralPath $ringPath -Raw|ConvertFrom-Json}else{$null}
$snapshotPath=Join-Path $OutputRoot 'STEP_SNAPSHOT_OBSERVATION.json'
$snapshot=if(Test-Path -LiteralPath $snapshotPath){Get-Content -LiteralPath $snapshotPath -Raw|ConvertFrom-Json}else{$null}
$result=[ordered]@{
 status='HOST_ONLY_OFFICIAL_COPTERSIM_SOFTWARE_CARRIER_OBSERVATION';native_exit_code=$rc;observed=$observed;actual_getter_observation=$tap
 hypothesis=if($StepSnapshot){'Does actual official NoUI preserve immutable step-complete sensor/rotor association, with independent all-getter fault accounting, and exact sensor-content comparison to every transmitted HIL_SENSOR?'}elseif($GetterRing){'Does continuous bounded RDR1 capture retain every actual NoUI getter without drops, enabling complete sensor-content comparison with actual transmitted HIL_SENSOR frames?'}else{'Does official software mode send HOST TUNNEL/PING to its separate PX4 MAVLink UDP peer instead of the TCP simulation-sensor peer?'}
 exact_child_arguments="1 48 -1 $modelName 1 Grasslands 127.0.0.1 0 0 0 1 2"
 child_mode='PX4_SITL TCP mode1; fake peer only, not PX4 or Pixhawk';exact_model=$modelName
 only_peer_messages=@('disarmed HEARTBEAT','all-zero HIL_ACTUATOR_CONTROLS','opaque diagnostic TUNNEL payload','diagnostic PING')
 vendor_automatic_commands_to_fake_peer=$vendorActions
 getter_ring_observation=$ring
 step_snapshot_observation=$snapshot
 peer_command_count_scope='NUMERICAL_RESULT counts commands from the test peer. Official NoUI commands are counted separately in the raw capture.'
 source_sha256=(Get-FileHash -LiteralPath $source -Algorithm SHA256).Hash;compiler_arguments=$args;inputs=$inputs
 simulator_processes_started=if($observed.owned_pid){1}else{0};COM_requested=0;board_commands=0;nonzero_actuator_inputs=0;PX4_processes=0;firmware_uploads=0
 raw_format='RAW_STREAMS.bin: repeating little-endian uint64 HOST QPC, uint32 channel, uint32 length, raw bytes. channels 0 TCP receive,1 HOST-API UDP receive,2 fake-peer TCP send,3 HOST-API UDP send,4 fake-PX4 MAVLink UDP receive.'
 parent_observation_erratum='frame_char_buffer reports CRC failures; corrupt-frame and valid-frame controls verify the parser.'
 limits='Carrier and optional getter-only software observation. Neither ring completeness nor exact sensor-content matches prove modelStep/getter atomicity, HITL serial forwarding, receiver-HRT association or control/flight success.'
}
$result|ConvertTo-Json -Depth 12|Set-Content -LiteralPath (Join-Path $OutputRoot 'RESULT.json')
@{observed=$observed;getter=$tap;ring=$ring;step_snapshot=$snapshot;vendor_to_fake_peer=$vendorActions}|ConvertTo-Json -Depth 5
if($rc -ne 0){exit $rc}
