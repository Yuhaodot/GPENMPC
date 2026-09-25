[CmdletBinding()]
param(
    [string]$PortName = 'COM3',
    [int]$BaudRate = 921600,
    [int]$DurationSeconds = 12,
    [Parameter(Mandatory = $true)]
    [string]$OutputPath
)

$ErrorActionPreference = 'Stop'

function Get-LandedStateName([int]$Value) {
    switch ($Value) {
        1 { 'UNDEFINED' }
        2 { 'ON_GROUND' }
        3 { 'IN_AIR' }
        4 { 'TAKEOFF' }
        5 { 'LANDING' }
        default { 'UNKNOWN' }
    }
}

function Convert-Frame {
    param([byte[]]$Frame)

    if ($Frame[0] -eq 0xFE) {
        $payloadLength = [int]$Frame[1]
        return [pscustomobject]@{
            Version = 1
            SystemId = [int]$Frame[3]
            ComponentId = [int]$Frame[4]
            MessageId = [int]$Frame[5]
            Payload = [byte[]]$Frame[6..(5 + $payloadLength)]
        }
    }

    $payloadLength = [int]$Frame[1]
    return [pscustomobject]@{
        Version = 2
        SystemId = [int]$Frame[5]
        ComponentId = [int]$Frame[6]
        MessageId = [int]$Frame[7] -bor ([int]$Frame[8] -shl 8) -bor ([int]$Frame[9] -shl 16)
        Payload = [byte[]]$Frame[10..(9 + $payloadLength)]
    }
}

$outputDirectory = Split-Path -Parent $OutputPath
if ($outputDirectory) {
    New-Item -ItemType Directory -Path $outputDirectory -Force | Out-Null
}

$serial = [System.IO.Ports.SerialPort]::new($PortName, $BaudRate)
$serial.ReadTimeout = 200
$serial.WriteTimeout = 200
$serial.DtrEnable = $false
$serial.RtsEnable = $false
$serial.Handshake = [System.IO.Ports.Handshake]::None

$heartbeatSamples = [System.Collections.Generic.List[object]]::new()
$landedSamples = [System.Collections.Generic.List[object]]::new()
$buffer = [System.Collections.Generic.List[byte]]::new()
$openedUtc = [DateTime]::UtcNow

try {
    $serial.Open()
    $deadline = [DateTime]::UtcNow.AddSeconds($DurationSeconds)
    $chunk = [byte[]]::new(4096)

    while ([DateTime]::UtcNow -lt $deadline) {
        try {
            $available = $serial.BytesToRead
            if ($available -le 0) {
                Start-Sleep -Milliseconds 20
                continue
            }
            $count = $serial.Read($chunk, 0, [Math]::Min($chunk.Length, $available))
            for ($i = 0; $i -lt $count; $i++) {
                $buffer.Add($chunk[$i])
            }
        } catch [System.TimeoutException] {
            continue
        }

        while ($buffer.Count -ge 8) {
            while ($buffer.Count -gt 0 -and $buffer[0] -ne 0xFE -and $buffer[0] -ne 0xFD) {
                $buffer.RemoveAt(0)
            }
            if ($buffer.Count -lt 8) {
                break
            }

            $payloadLength = [int]$buffer[1]
            if ($buffer[0] -eq 0xFE) {
                $frameLength = 6 + $payloadLength + 2
            } else {
                $signatureLength = if (($buffer[2] -band 0x01) -ne 0) { 13 } else { 0 }
                $frameLength = 10 + $payloadLength + 2 + $signatureLength
            }
            if ($buffer.Count -lt $frameLength) {
                break
            }

            $frame = [byte[]]$buffer.GetRange(0, $frameLength).ToArray()
            $buffer.RemoveRange(0, $frameLength)
            $decoded = Convert-Frame -Frame $frame

            if ($decoded.MessageId -eq 0 -and $decoded.Payload.Length -ge 9) {
                $customMode = [BitConverter]::ToUInt32($decoded.Payload, 0)
                $baseMode = [int]$decoded.Payload[6]
                $heartbeatSamples.Add([pscustomobject]@{
                    utc = [DateTime]::UtcNow.ToString('o')
                    mavlink_version = $decoded.Version
                    system_id = $decoded.SystemId
                    component_id = $decoded.ComponentId
                    custom_mode = $customMode
                    vehicle_type = [int]$decoded.Payload[4]
                    autopilot = [int]$decoded.Payload[5]
                    base_mode = $baseMode
                    armed = ($baseMode -band 128) -ne 0
                    system_status = [int]$decoded.Payload[7]
                })
            }

            if ($decoded.MessageId -eq 245 -and $decoded.Payload.Length -ge 2) {
                $landed = [int]$decoded.Payload[1]
                $landedSamples.Add([pscustomobject]@{
                    utc = [DateTime]::UtcNow.ToString('o')
                    system_id = $decoded.SystemId
                    component_id = $decoded.ComponentId
                    vtol_state = [int]$decoded.Payload[0]
                    landed_state = $landed
                    landed_state_name = Get-LandedStateName $landed
                })
            }
        }
    }
} finally {
    if ($serial.IsOpen) {
        $serial.Close()
    }
    $serial.Dispose()
}

$lastHeartbeat = if ($heartbeatSamples.Count) { $heartbeatSamples[$heartbeatSamples.Count - 1] } else { $null }
$lastLanded = if ($landedSamples.Count) { $landedSamples[$landedSamples.Count - 1] } else { $null }
$disarmedObserved = $null -ne $lastHeartbeat -and -not $lastHeartbeat.armed
$onGroundObserved = $null -ne $lastLanded -and $lastLanded.landed_state_name -eq 'ON_GROUND'
$status = if ($disarmedObserved -and $onGroundObserved) {
    'PASS_DISARMED_AND_ON_GROUND'
} elseif ($disarmedObserved) {
    'PASS_DISARMED_LANDED_STATE_UNAVAILABLE'
} else {
    'INCOMPLETE_OR_ARMED_READ_ONLY_OBSERVATION'
}

$receipt = [ordered]@{
    schema = 'GPENMPC_READ_ONLY_MAVLINK_BOARD_STATE_V1'
    status = $status
    observation_only = $true
    bytes_written_to_serial = 0
    disarmed_observed = $disarmedObserved
    on_ground_observed = $onGroundObserved
    claim_limit = 'READ_ONLY_DISARMED_OBSERVATION__LANDED_STATE_REQUIRES_VALID_ESTIMATOR_OR_HIL_SENSOR_STREAM'
    port = $PortName
    baud_rate = $BaudRate
    opened_utc = $openedUtc.ToString('o')
    closed_utc = [DateTime]::UtcNow.ToString('o')
    heartbeat_count = $heartbeatSamples.Count
    extended_system_state_count = $landedSamples.Count
    final_heartbeat = $lastHeartbeat
    final_extended_system_state = $lastLanded
    heartbeat_samples = $heartbeatSamples
    extended_system_state_samples = $landedSamples
}

$receipt | ConvertTo-Json -Depth 8 | Set-Content -LiteralPath $OutputPath -Encoding utf8
$receipt | ConvertTo-Json -Depth 8

if (-not $disarmedObserved) {
    exit 1
}
