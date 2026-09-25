#!/usr/bin/env python3
"""Link the PX4 application ELF in the isolated build directory."""
import json
import os
import re
import shlex
import subprocess
import sys
import time
from pathlib import Path
sys.dont_write_bytecode = True
import configure_private_nuttx as isolation

root = Path(__file__).resolve().parent
build = isolation.BUILD
database = json.loads((build / 'compile_commands.json').read_text())
compiler = shlex.split(next(r['command'] for r in database
    if Path(r['file']).name in ('Px4CanonicalLocalIo.cpp', 'Px4CanonicalIo.cpp')))[0]
if Path(compiler).name == 'ccache':
    compiler = shlex.split(next(r['command'] for r in database
        if Path(r['file']).name in ('Px4CanonicalLocalIo.cpp', 'Px4CanonicalIo.cpp')))[1]
assert 'arm-none-eabi-g++' in compiler
cfg_command = build / ('src/modules/uxrce_dds_client/tmp/'
    'libmicroxrceddsclient_project-cfgcmd.txt')
assert cfg_command.is_file()
# cfgcmd is an ExternalProject template, not its resolved command. Inspect the
# actual generated configure script and canonicalize only its explicit -S path.
cfg_script = build / ('src/modules/uxrce_dds_client/src/'
    'libmicroxrceddsclient_project-stamp/'
    'libmicroxrceddsclient_project-configure-MinSizeRel.cmake')
match = re.search(r';-S;([^;]+);-B;', cfg_script.read_text())
assert match and Path(match.group(1)).resolve() == (
    root / 'private_dependencies/Micro-XRCE-DDS-Client').resolve()
superbuild = root / 'private_dependencies/Micro-XRCE-DDS-Client/cmake/SuperBuild.cmake'
superbuild_text = superbuild.read_text()
assert 'https://github.com/eProsima/Micro-CDR.git' not in superbuild_text
assert 'DOWNLOAD_COMMAND ""' in superbuild_text and 'UPDATE_COMMAND ""' in superbuild_text
assert not (root / 'private_dependencies/Micro-XRCE-DDS-Client/client.config').exists()
assert not (build / 'px4_fmu-v6c_default.elf').exists(), 'Preserve existing ELF; inspect before another link'

# PX4's NuttX libapps custom rule does not depend on configure-generated
# px4.bdat/px4.pdat. A changed module configuration can therefore leave an old
# command table/archive. Refuse that state before linking; regenerate the real
# isolated nuttx_apps_build target, never edit a builtin list by hand.
registry = root / 'private_source/platforms/nuttx/NuttX/apps/builtin/registry'
for suffix in ('bdat', 'pdat'):
    assert (registry / ('px4.' + suffix)).read_bytes() == (
        build / ('NuttX/px4.' + suffix)).read_bytes(), 'Stale private NuttX command registry'
command_names = lambda text: set(re.findall(r'\{\s*"([^"\\]+)"\s*,', text))
expected_builtin_names = set()
for registration in registry.glob('*.bdat'):
    expected_builtin_names.update(command_names(registration.read_text()))
actual_builtin_names = command_names((registry.parent / 'builtin_list.h').read_text())
assert expected_builtin_names == actual_builtin_names, 'Stale private NuttX builtin list'

tracked = sorted({Path(r['file']) for r in database if
    'rfly_vendor_integration/' in r['file'] or 'arm_controller/' in r['file']})
tracked += [root / 'overlay/src/modules/mavlink/mavlink_messages.cpp', superbuild]
source_before = {str(p): isolation.sha(p) for p in tracked if p.is_file()}
anchors = isolation.stat_snapshot(focused=True)
env = dict(os.environ)
env['PATH'] = str(Path(compiler).parent) + ':' + env.get('PATH', '')
env.update(GIT_SUBMODULES_ARE_EVIL='1', GIT_OPTIONAL_LOCKS='0',
           PYTHONDONTWRITEBYTECODE='1', CCACHE_DISABLE='1',
           OMP_NUM_THREADS='1', OPENBLAS_NUM_THREADS='1', MKL_NUM_THREADS='1',
           CMAKE_BUILD_PARALLEL_LEVEL='2')
command = ['cmake', '--build', str(build), '--parallel', '2', '--target', 'px4']
label = 'APPLICATION_BUILD_' + time.strftime('%Y%m%d_%H%M%S')
log = root / (label + '.log')
print('Actual isolated FMUv6C ELF build/link -j2; no package/upload/run.', flush=True)
with log.open('w') as stream:
    run = subprocess.run(command, env=env, stdin=subprocess.DEVNULL,
                         stdout=stream, stderr=subprocess.STDOUT)
source_after = {p: isolation.sha(Path(p)) for p in source_before}
after = isolation.stat_snapshot(focused=True)
changed = [p for p in anchors if anchors[p] != after.get(p)]
elf = build / 'px4_fmu-v6c_default.elf'
result = {'scope': 'ACTUAL_PRIVATE_FMUV6C_APPLICATION_ELF_LINK_ONLY',
          'command': command, 'exit_code': run.returncode,
          'log': str(log), 'log_sha256': isolation.sha(log),
          'source_sha256_before': source_before, 'source_sha256_after': source_after,
          'old_write_target_and_config_anchor_changes': changed,
          'local_microcdr_source_used': True, 'microcdr_prebuilt_binary_reused': False,
          'production_context_supplied': False, 'canonical_module_autostart_installed': False,
          'target_code_executed': False, 'upload_performed': False,

          'passed': run.returncode == 0 and elf.is_file() and
                    source_before == source_after and not changed}
if elf.is_file():
    result['elf'] = {'path': str(elf), 'bytes': elf.stat().st_size,
                     'sha256': isolation.sha(elf)}
if run.returncode == 0 and elf.is_file():
    nm = Path(compiler).with_name('arm-none-eabi-nm')
    symbols = subprocess.run([str(nm), '--defined-only', str(elf)],
                            capture_output=True, text=True, check=True).stdout
    result['actual_main_symbol_defined'] = bool(re.search(
        r'\bT gpenmpc_rfly_canonical_main\s*$', symbols, re.M))
    result['actual_session_main_symbol_defined'] = bool(re.search(
        r'\bT gpenmpc_rfly_session_main\s*$', symbols, re.M))
    result['actual_generated_step_symbol_defined'] = bool(re.search(
        r'\bT GPENMPC_Rfly_Canonical_Controller_step\s*$', symbols, re.M))
    demangled = subprocess.run([str(nm), '-C', '--defined-only', str(elf)],
        capture_output=True, text=True, check=True).stdout
    required = ('gpenmpc_rfly_px4::RegisteredCanonicalModuleContext::poll(',
                'gpenmpc_rfly_px4::CanonicalApplicationOwner::prepare(',
                'gpenmpc_rfly_px4::Px4CanonicalIo::execute(',
                'gpenmpc_visit_locked_mavlink_device(')
    result['actual_connection_symbols'] = {name: [line for line in demangled.splitlines()
        if name in line] for name in required}
    result['passed'] &= (result['actual_main_symbol_defined'] and
        result['actual_session_main_symbol_defined'] and
        result['actual_generated_step_symbol_defined'] and
        all(result['actual_connection_symbols'].values()))
    size = Path(compiler).with_name('arm-none-eabi-size')
    result['elf_size_output'] = subprocess.run([str(size), str(elf)],
        capture_output=True, text=True, check=True).stdout
(root / (label + '.json')).write_text(json.dumps(result, indent=2) + '\n')
print(json.dumps({k: result[k] for k in ('scope', 'exit_code', 'passed')}))
print(log.read_text()[-8000:])
raise SystemExit(0 if result['passed'] else 1)
