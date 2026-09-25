param([Parameter(Mandatory=$true)][string]$OutputRoot)
$ErrorActionPreference='Stop'
if(Test-Path -LiteralPath $OutputRoot){throw 'OutputRoot must be new.'}
$null=New-Item -ItemType Directory -Path $OutputRoot
$source=Join-Path $PSScriptRoot 'OwnedCopterInitObserver.cs'
Add-Type -Path $source
$checks=[Collections.Generic.List[object]]::new()
function Check([string]$name,[bool]$pass){$checks.Add([pscustomobject]@{name=$name;pass=$pass})}
function Packet([int]$magic,[int]$id,[int]$code){return [byte[]]([BitConverter]::GetBytes($magic)+[BitConverter]::GetBytes($id)+[BitConverter]::GetBytes($code))}
function AwaitCount($o,[int]$n){$sw=[Diagnostics.Stopwatch]::StartNew();while($o.Snapshot().received_datagrams -lt $n -and $sw.Elapsed.TotalSeconds -lt 3){Start-Sleep -Milliseconds 10};return $o.Snapshot()}
$o=$null;$overflow=$null;$sender=$null;$errorAtom=$null;$idle=$null
try {
  foreach($bad in @(@(-1,1,16),@(65536,1,16),@(0,0,16),@(0,1,0),@(0,1,65537))){
    $rejected=$false;try{$invalid=[GPENMPC.HostDiagnostics.OwnedCopterInitObserver]::Start($bad[0],$bad[1],$bad[2]);$invalid.Dispose()}catch{$rejected=$true}
    Check ('invalid_constructor_'+($bad -join '_')) $rejected
  }
  $o=[GPENMPC.HostDiagnostics.OwnedCopterInitObserver]::Start(0,1,16)
  $sender=[Net.Sockets.UdpClient]::new()
  # Synthetic arbitrary checksum is preserved, not asserted as the vendor magic.
  foreach($b in @((Packet 123 1 0),(Packet 123 1 1),(Packet 123 1 7),(Packet 999 2 1),([byte[]](1,2,3)))){
    $null=$sender.Send([byte[]]$b,$b.Length,'127.0.0.1',$o.Port)
  }
  $e=AwaitCount $o 5
  Check 'five_actual_localhost_datagrams_received' ($e.received_datagrams -eq 5)
  Check 'zero_decoded_not_finished' ($e.packets[0].interpretation -eq 'VENDOR_INITIALIZATION_NOT_FINISHED')
  Check 'one_decoded_GPS3D_report_only' ($e.packets[1].interpretation -eq 'VENDOR_GPS3D_FIXED_REPORTED')
  Check 'unknown_code_not_coerced_ready' ($e.packets[2].interpretation -eq 'UNKNOWN_INIT_CODE_RAW_ONLY')
  Check 'other_vehicle_not_coerced_ready' ($e.packets[3].interpretation -eq 'OTHER_VEHICLE_RAW_ONLY')
  Check 'bad_length_preserved_not_decoded' (-not $e.packets[4].parsed_3i -and $e.packets[4].length -eq 3)
  Check 'raw_bytes_roundtrip' ([Convert]::ToBase64String((Packet 123 1 0)) -eq $e.packets[0].raw_base64)
  Check 'checksum_not_invented' ($e.packets[3].checksum_raw -eq 999 -and $e.checksum_policy -like 'RAW_ONLY*')
  Check 'timestamps_retained' ($e.packets[0].received_qpc -gt 0 -and $e.packets[4].received_qpc -ge $e.packets[0].received_qpc -and $e.packets[0].received_utc.EndsWith('Z'))
  Check 'remote_endpoint_retained' ($e.packets[0].remote.StartsWith('127.0.0.1:'))
  Check 'fixture_not_multicast_member' ($e.multicast_group -eq '')
  Check 'bounded_raw_memory_disclosed' ($e.retained_raw_bytes -eq 51 -and $e.max_retained_raw_bytes -eq 16777216)
  $blocked=$false;$competitor=[Net.Sockets.Socket]::new([Net.Sockets.AddressFamily]::InterNetwork,[Net.Sockets.SocketType]::Dgram,[Net.Sockets.ProtocolType]::Udp)
  try{$competitor.Bind([Net.IPEndPoint]::new([Net.IPAddress]::Loopback,$o.Port))}catch{$blocked=$true}finally{$competitor.Dispose()}
  Check 'exclusive_same_endpoint_collision_rejected' $blocked
  $constructorBlocked=$false;try{$invalid=[GPENMPC.HostDiagnostics.OwnedCopterInitObserver]::Start($o.Port,1,16);$invalid.Dispose()}catch{$constructorBlocked=$true}
  Check 'constructor_collision_fails_and_existing_owner_survives' ($constructorBlocked -and -not $o.Snapshot().socket_closed)
  $port=$o.Port;$o.Dispose();$closed=$o.Snapshot()
  Check 'socket_and_thread_closed' ($closed.socket_closed -and $closed.thread_exited -and $closed.errors.Count -eq 0)
  $probe=[Net.Sockets.UdpClient]::new($port);$probe.Dispose()
  Check 'closed_port_can_rebind' $true
  $o.Dispose();Check 'repeated_dispose_is_idempotent' ($o.Snapshot().socket_closed -and $o.Snapshot().errors.Count -eq 0)
  $idle=[GPENMPC.HostDiagnostics.OwnedCopterInitObserver]::Start(0,1,16);$idlePort=$idle.Port
  $idleWatch=[Diagnostics.Stopwatch]::StartNew();$idle.Dispose();$idleWatch.Stop();$idleEvidence=$idle.Snapshot()
  Check 'idle_receive_thread_bounded_close' ($idleEvidence.socket_closed -and $idleEvidence.thread_exited -and $idleWatch.Elapsed.TotalMilliseconds -lt 2200)
  $probe=[Net.Sockets.UdpClient]::new($idlePort);$probe.Dispose();Check 'idle_closed_port_rebinds' $true
  $overflow=[GPENMPC.HostDiagnostics.OwnedCopterInitObserver]::Start(0,1,2)
  foreach($i in 0..4){$b=Packet 123 1 $i;$null=$sender.Send($b,$b.Length,'127.0.0.1',$overflow.Port)}
  $oe=AwaitCount $overflow 5
  Check 'bounded_memory_capacity_respected' ($oe.stored_datagrams -eq 2)
  Check 'overflow_exactly_disclosed' ($oe.received_datagrams -eq 5 -and $oe.overflow_dropped -eq 3 -and -not $oe.evidence_complete)
  $overflow.Dispose();$oe=$overflow.Snapshot()
  Check 'overflow_path_closes' ($oe.socket_closed -and $oe.thread_exited)
  Check 'no_serial_or_transmit_in_observer' ($e.COM_open -eq 0 -and $e.send_count -eq 0 -and $e.receive_only)
}catch{$errorAtom=$_.Exception.ToString()}finally{if($null -ne $o){$o.Dispose()};if($null -ne $overflow){$overflow.Dispose()};if($null -ne $idle){$idle.Dispose()};if($null -ne $sender){$sender.Dispose()}}
$official=(& (Join-Path $PSScriptRoot 'gpenmpc_install_path.ps1') 'rfly' 'RflySimAPIs\RflySimSDK\ctrl\ReqCopterSim.py')
$result=[ordered]@{classification='HOST_SYNTHETIC_VENDOR_INIT_OBSERVER';pass=($null -eq $errorAtom -and @($checks|Where-Object{-not $_.pass}).Count -eq 0);source=$source;source_sha256=(Get-FileHash -LiteralPath $source -Algorithm SHA256).Hash;official_protocol_source=$official;official_source_sha256=(Get-FileHash -LiteralPath $official -Algorithm SHA256).Hash;official_source_lines='39-133';checks_total=$checks.Count;checks_passed=@($checks|Where-Object pass).Count;checks=$checks;error=$errorAtom;normal_evidence=$closed;idle_evidence=$idleEvidence;overflow_evidence=$oe;hardware_actions=0;COM_open=0;CopterSim_started=0;PX4_access=0;synthetic_localhost_sender_only=$true}
$result|ConvertTo-Json -Depth 12|Set-Content -LiteralPath (Join-Path $OutputRoot 'RESULT.json') -Encoding utf8
[pscustomobject]$result|Select-Object pass,checks_total,checks_passed,error|ConvertTo-Json
if(-not $result.pass){exit 2}
