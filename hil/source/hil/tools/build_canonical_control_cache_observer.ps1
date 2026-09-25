param([Parameter(Mandatory=$true)][string]$OutputDirectory)
$ErrorActionPreference='Stop'
$taskBuild=Split-Path -Parent $PSScriptRoot
$parent=(& (Join-Path $PSScriptRoot 'gpenmpc_external_path.ps1') 'rotor_observer_dll')
$generated=Join-Path $parent 'GPENMPC_M600_Canonical_ert_rtw'
$oldCompiled=Join-Path $parent 'compiled'
$observer=Join-Path $taskBuild 'host_runtime\copter_observer'
$output=[IO.Path]::GetFullPath($OutputDirectory)
if(-not $output.StartsWith(($taskBuild+[IO.Path]::DirectorySeparatorChar),[StringComparison]::OrdinalIgnoreCase)){throw 'Output must remain inside this existing build workspace'}
if(Test-Path -LiteralPath $output){throw 'Choose a new output directory.'}
if((Get-FileHash -LiteralPath (Join-Path $oldCompiled 'GPENMPC_M600_Canonical.dll')).Hash -ne 'B817CA67429871A64F2368B7D161B91F4E2E032607B1D76E73AE12D4255D33C1'){throw 'Exact prior DLL differs'}
$core=Join-Path $generated 'GPENMPC_M600_Canonical.cpp'
$coreHash=(Get-FileHash -LiteralPath $core).Hash
$parentWrapper=Join-Path $oldCompiled 'modeldllgen.cpp'
$parentHash=(Get-FileHash -LiteralPath $parentWrapper).Hash
$cpp=[IO.File]::ReadAllText($parentWrapper)
$original=$cpp
function ExactReplace([string]$Text,[string]$Old,[string]$New){
    if(([regex]::Matches($Text,[regex]::Escape($Old))).Count -ne 1){throw "Expected one exact source anchor: $Old"}
    return $Text.Replace($Old,$New)
}
$header="#include `"control_cache_observer.hpp`"`n"
$cpp=$header+$cpp
$globals=@'
gpenmpc_control_cache::Observer<gpenmpc_rotor_observer::LocalhostSink> control_cache_observer(rotor_sink);
gpenmpc_rotor_observer::Config cache_config;
gpenmpc_control_cache::Input step_input_snapshot;
uint64_t original_input_calls=0;
DWORD original_input_thread=0;
'@
$cpp=ExactReplace $cpp 'bool rotor_configuration_read=false;' ($globals+"`n"+'bool rotor_configuration_read=false;')
$oldConfigure='if(gpenmpc_rotor_observer::environment_config(c)) rotor_observer.configure(c); else rotor_observer.invalid_environment();'
$newConfigure='if(gpenmpc_rotor_observer::environment_config(c)){ cache_config=c; rotor_observer.configure(c); } else rotor_observer.invalid_environment();'
$cpp=ExactReplace $cpp $oldConfigure $newConfigure
$oldAfter='rotor_observer.after_step(s);'
$newAfter=@'
rotor_observer.after_step(s);
 control_cache_observer.after_step(step_input_snapshot,
   original_input_thread==GetCurrentThreadId()?original_input_calls:0,s,cache_config);
'@
$cpp=ExactReplace $cpp $oldAfter $newAfter
$oldStep='mmc.step();'
$newStep=@'
step_input_snapshot=gpenmpc_control_cache::capture(mmc.GPENMPC_M600_Canonical_U.inPWMs,
    original_input_thread==GetCurrentThreadId()?original_input_calls:0);
  mmc.step();
'@
$cpp=ExactReplace $cpp $oldStep $newStep
$oldCopy='memcpy(mmc.GPENMPC_M600_Canonical_U.inPWMs, inPWMs, sizeof(double)*16);'
$newCopy=@'
memcpy(mmc.GPENMPC_M600_Canonical_U.inPWMs, inPWMs, sizeof(double)*16);
    if(original_input_calls!=UINT64_MAX)++original_input_calls;
    else original_input_calls=0;
    original_input_thread=GetCurrentThreadId();
'@
$cpp=ExactReplace $cpp $oldCopy $newCopy
$extra=@'

extern "C" DLLGEN_EXPORT void DllControlCacheObserverStatus(uint64_t out[5]) {
 if(!out)return;const auto&s=control_cache_observer.stats();
 const uint64_t v[5]={s.accepted_steps,s.sent,s.send_attempts,s.failure,s.duplicates};
 memcpy(out,v,sizeof(v));
}
'@
$cpp=$cpp+$extra
$inverse=$cpp.Substring($header.Length)
$inverse=$inverse.Replace($globals+"`n",'').Replace($newConfigure,$oldConfigure).Replace($newAfter,$oldAfter).Replace($newStep,$oldStep).Replace($newCopy,$oldCopy)
$inverse=$inverse.Substring(0,$inverse.Length-$extra.Length)
if($inverse -cne $original){throw 'Inverse patch does not reproduce exact prior wrapper'}
New-Item -ItemType Directory -Path $output | Out-Null
$compiled=Join-Path $output 'compiled';New-Item -ItemType Directory -Path $compiled | Out-Null
[IO.File]::WriteAllText((Join-Path $compiled 'modeldllgen.cpp'),$cpp,[Text.UTF8Encoding]::new($false))
Copy-Item -LiteralPath (Join-Path $oldCompiled 'modeldllgen.h'),(Join-Path $oldCompiled 'rfly_export_compat.def'),(Join-Path $observer 'rotor_observer.hpp'),(Join-Path $observer 'control_cache_observer.hpp') -Destination $compiled
$sink=[IO.File]::ReadAllText((Join-Path $oldCompiled 'rotor_observer_win32.hpp'))
$sink=ExactReplace $sink 'size!=packet_size' '(size!=packet_size && size!=216)'
[IO.File]::WriteAllText((Join-Path $compiled 'rotor_observer_win32.hpp'),$sink,[Text.UTF8Encoding]::new($false))
$compiler=(& (Join-Path $PSScriptRoot 'gpenmpc_install_path.ps1') 'llvm' 'bin\clang++.exe')
$dll=Join-Path $compiled 'GPENMPC_M600_Canonical.dll'
$compilerArgs=@('-std=c++17','-O2','-shared','-static','-fms-extensions','-fdeclspec','-Wl,--no-undefined',('-I'+$compiled),('-I'+$generated),('-I'+(& (Join-Path $PSScriptRoot 'gpenmpc_install_path.ps1') 'matlab' 'extern\include')),('-I'+(& (Join-Path $PSScriptRoot 'gpenmpc_install_path.ps1') 'matlab' 'rtw\c\src')),('-I'+(& (Join-Path $PSScriptRoot 'gpenmpc_install_path.ps1') 'matlab' 'simulink\include')),$core,(Join-Path $compiled 'modeldllgen.cpp'),(Join-Path $compiled 'rfly_export_compat.def'),'-lws2_32','-o',$dll)
& $compiler @compilerArgs 2>&1 | Tee-Object -FilePath (Join-Path $output 'COMPILE.log')
$rc=$LASTEXITCODE
$report=[ordered]@{schema='RFLY_SAME_OWNER_CONTROL_CACHE_OBSERVATION_BUILD_V1';pass=($rc -eq 0);returncode=$rc;parent_dll_sha256='B817CA67429871A64F2368B7D161B91F4E2E032607B1D76E73AE12D4255D33C1';parent_wrapper_sha256=$parentHash;generated_core_path=$core;generated_core_sha256=$coreHash;generated_core_unchanged=((Get-FileHash -LiteralPath $core).Hash -eq $coreHash);original_wrapper_inverse_patch_equal=($inverse -ceq $original);wrapper_sha256=(Get-FileHash -LiteralPath (Join-Path $compiled 'modeldllgen.cpp')).Hash;dll_path=$dll;dll_sha256='';packet_bytes=216;scope='EXACT_CALLER_INPUT_BUFFER_AT_ORIGINAL_ACCEPTED_MODEL_STEP__OBSERVATION_ONLY';same_original_model=$true;second_socket_created=$false;model_loaded=$false;COM_open=0;board_actions=0;compiler=$compiler;arguments=$compilerArgs}
if($rc -eq 0){$report.dll_sha256=(Get-FileHash -LiteralPath $dll).Hash}
$report | ConvertTo-Json -Depth 7 | Set-Content -LiteralPath (Join-Path $output 'RESULT.json') -Encoding utf8
if($rc -ne 0){throw "Compiler rc=$rc"}
$report | ConvertTo-Json -Depth 3
