param([string]$ApplicationRecordRoot = (& (Join-Path $PSScriptRoot 'gpenmpc_external_path.ps1') 'application_build_record'))
$ErrorActionPreference='Stop'
$taskBuild=Split-Path -Parent $PSScriptRoot
$taskApp=Join-Path $taskBuild 'rfly_vendor_integration\application_integration'
$taskCompiled=Join-Path $taskApp 'build_fmuv6c'
$taskRun=$ApplicationRecordRoot
$taskResult=Get-Content -LiteralPath (Join-Path $taskRun 'RESULT.json') -Raw | ConvertFrom-Json
if(-not $taskResult.passed -or -not $taskResult.sources_stable){throw 'Actual current application link is not valid.'}
$taskArtifacts=@()
foreach($taskName in @('px4_fmu-v6c_default.elf','px4_fmu-v6c_default.map','px4_fmu-v6c_default.bin','px4_fmu-v6c_default.px4','parameters.json')){
 $taskPath=Join-Path $taskCompiled $taskName
 $taskFile=Get-Item -LiteralPath $taskPath
 $taskArtifacts+= [ordered]@{path=$taskPath;bytes=$taskFile.Length;sha256=(Get-FileHash -LiteralPath $taskPath -Algorithm SHA256).Hash}
}
foreach($taskOld in $taskResult.artifacts){
 $taskName=($taskOld.path -split '/')[-1]
 $taskNew=@($taskArtifacts | Where-Object { [IO.Path]::GetFileName($_.path) -eq $taskName })
 if($taskNew.Count -ne 1 -or $taskNew[0].sha256 -ne $taskOld.sha256){throw ('Linked artifact changed: '+$taskName)}
}
$taskPackage=Get-Content -LiteralPath (Join-Path $taskCompiled 'px4_fmu-v6c_default.px4') -Raw | ConvertFrom-Json
$taskCompressed=[Convert]::FromBase64String($taskPackage.image)
$taskInput=[IO.MemoryStream]::new($taskCompressed,$false)
$taskOutput=[IO.MemoryStream]::new()
$taskZ=[IO.Compression.ZLibStream]::new($taskInput,[IO.Compression.CompressionMode]::Decompress)
try { $taskZ.CopyTo($taskOutput); $taskUnpacked=$taskOutput.ToArray() } finally { $taskZ.Dispose();$taskInput.Dispose();$taskOutput.Dispose() }
$taskRaw=[IO.File]::ReadAllBytes((Join-Path $taskCompiled 'px4_fmu-v6c_default.bin'))
$taskHasher=[Security.Cryptography.SHA256]::Create()
try {
 $taskImageSha=[Convert]::ToHexString($taskHasher.ComputeHash($taskUnpacked))
 $taskRawSha=[Convert]::ToHexString($taskHasher.ComputeHash($taskRaw))
} finally {$taskHasher.Dispose()}
if($taskImageSha -ne $taskRawSha -or $taskUnpacked.Length -ne $taskPackage.image_size -or $taskPackage.board_id -ne 56){throw 'Package/bin/target mismatch.'}
$taskOldParams=Get-Content -LiteralPath ((& (Join-Path $PSScriptRoot 'gpenmpc_external_path.ps1') 'application_parameter_reference')) -Raw | ConvertFrom-Json
$taskNewParams=Get-Content -LiteralPath (Join-Path $taskCompiled 'parameters.json') -Raw | ConvertFrom-Json
$taskOldMap=@{};$taskNewMap=@{}
foreach($taskParam in $taskOldParams.parameters){$taskOldMap[$taskParam.name]=($taskParam | ConvertTo-Json -Depth 20 -Compress)}
foreach($taskParam in $taskNewParams.parameters){$taskNewMap[$taskParam.name]=($taskParam | ConvertTo-Json -Depth 20 -Compress)}
$taskMissing=@($taskOldMap.Keys | Where-Object {-not $taskNewMap.ContainsKey($_)})
$taskExtra=@($taskNewMap.Keys | Where-Object {-not $taskOldMap.ContainsKey($_)})
$taskChanged=@($taskOldMap.Keys | Where-Object {$taskNewMap.ContainsKey($_) -and $taskOldMap[$_] -cne $taskNewMap[$_]})
if($taskMissing.Count -or $taskExtra.Count -or $taskChanged.Count){throw 'Parameter declarations/defaults/metadata changed.'}
$taskSymbols=Get-Content -LiteralPath (Join-Path $taskRun 'SYMBOLS.log') -Raw
$taskChecks=[ordered]@{}
foreach($taskName in @('gpenmpc_rfly_canonical_local_main','gpenmpc_rfly_session_main','gpenmpc_full_inner_prepare_numeric','gpenmpc_full_inner_commit','gpenmpc_full_inner_copy_closed_evidence')){
 $taskChecks[$taskName]=$taskSymbols.Contains($taskName)
}
foreach($taskName in @('gpenmpc_rfly_canonical_main','gpenmpc_trajectory_exec_main','gpenmpc_se3_control_main','gpenmpc_tunnel_bridge_main','rtrpdc_cg_step','CanonicalTickPump')){
 $taskChecks['absent_'+$taskName]= -not $taskSymbols.Contains($taskName)
}
if(@($taskChecks.Values | Where-Object {-not $_}).Count){throw 'Actual linked symbol disagreement.'}
$taskReceipt=[ordered]@{
 scope='FMUV6C_LOCAL_FULL_INNER_APPLICATION_PACKAGE'
 passed=$true;artifacts=$taskArtifacts;package_image_sha256=$taskImageSha;board_id=$taskPackage.board_id
 package_git_identity=$taskPackage.git_identity
 image_bytes=$taskUnpacked.Length;flash_limit_bytes=1966080;flash_remaining_bytes=(1966080-$taskUnpacked.Length)
 static_axi_sram_bytes=69376
 parameter_declarations=$taskNewMap.Count;parameter_missing=$taskMissing;parameter_extra=$taskExtra;parameter_changed=$taskChanged
 linked_checks=$taskChecks
 dynamic_publisher_exclusivity_measured=$false;runtime_heap_measured=$false;WCET_measured=$false;stack_high_water_measured=$false
 hardware_actions=0;COM=0;flash=0;application_executed=$false
 limitations='Application link and package identity checks; heap, stack and board timing require separate measurements.'
}
$taskOut=Join-Path $taskRun 'APPLICATION_PACKAGE.json'
if(Test-Path -LiteralPath $taskOut){throw 'Existing package receipt must not be overwritten.'}
$taskReceipt | ConvertTo-Json -Depth 20 | Set-Content -LiteralPath $taskOut -Encoding utf8
[pscustomobject]$taskReceipt | Select-Object scope,passed,image_bytes,flash_remaining_bytes,parameter_declarations,board_id,COM,flash | ConvertTo-Json
