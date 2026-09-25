#!/usr/bin/env python3
"""Build the selected integration targets."""
import hashlib
import json
import os
import shlex
import subprocess
import time
import re
import sys
from pathlib import Path
sys.dont_write_bytecode = True
import configure_private_nuttx as isolation

ROOT = Path(__file__).resolve().parent
TARGETS = ['gpenmpc_rfly_runtime_components', 'modules__gpenmpc_rfly_canonical',
           'modules__gpenmpc_rfly_session']


def main():
    database = json.loads((isolation.BUILD / 'compile_commands.json').read_text())
    required = ('Px4CanonicalIo.cpp', 'Px4ReadOnlyGuard.cpp',
                'SharedOutputRegistry.cpp', 'LinkLifetimeRegistry.cpp',
                'Px4ExecutionSessionGuard.cpp', 'ExecutionSessionRegistration.cpp',
                'CanonicalExchangePump.cpp', 'SnapshotRouteRegistry.cpp',
                'CommittedFeedbackRouteRegistry.cpp',
                'RegisteredCanonicalModuleContext.cpp', 'CanonicalApplicationOwner.cpp',
                'LegacyTrajectoryReadOnly.cpp', 'LinkCanonicalReservation.cpp',
                'GPENMPCTrajectoryExec.cpp',
                'CanonicalSessionEntry.cpp',
                'GPENMPCRflyCanonicalModule.cpp', 'GPENMPC_Rfly_Canonical_Controller.c')
    matches = {name: [row for row in database if Path(row['file']).name == name]
               for name in required}
    assert all(len(rows) == 1 for rows in matches.values()), {
        name: len(rows) for name, rows in matches.items()}
    old_nuttx = str(isolation.PX4 / 'platforms/nuttx/NuttX')
    for row in database:
        tokens = shlex.split(row['command'])
        for index, token in enumerate(tokens):
            assert not token.startswith('-I' + old_nuttx), row['file']
            if token == '-isystem' and index + 1 < len(tokens):
                assert not tokens[index + 1].startswith(old_nuttx), row['file']
    core = matches['GPENMPC_Rfly_Canonical_Controller.c'][0]['command']
    assert 'arm-none-eabi-gcc' in core and 'arm-none-eabi-g++' not in core
    assert (isolation.BUILD / 'build.ninja').is_file()
    cache = (isolation.BUILD / 'CMakeCache.txt').read_text()
    for variable, suffix in (('NUTTX_DIR', 'nuttx'), ('NUTTX_APPS_DIR', 'apps')):
        expected = str(isolation.PRIVATE / 'platforms/nuttx/NuttX' / suffix)
        assert variable + ':FILEPATH=' + expected in cache, variable
    ninja = (isolation.BUILD / 'build.ninja').read_text()
    for command_line in ninja.splitlines():
        if command_line.lstrip().startswith('COMMAND ='):
            for suffix in ('nuttx', 'apps'):
                old_write_cwd = str(isolation.PX4 / 'platforms/nuttx/NuttX' / suffix)
                assert 'cd ' + old_write_cwd + ' ' not in command_line
            assert 'cd ' + str(isolation.OLD_BUILD) + ' ' not in command_line
    before = isolation.stat_snapshot(focused=True)
    source_hashes = {row['file']: isolation.sha(Path(row['file']))
                     for rows in matches.values() for row in rows}
    command = ['cmake', '--build', str(isolation.BUILD), '--parallel', '2',
               '--target'] + TARGETS
    env = dict(os.environ)
    old_database = json.loads((isolation.OLD_BUILD / 'compile_commands.json').read_text())
    compiler = shlex.split(old_database[0]['command'])[0]
    env['PATH'] = str(Path(compiler).parent) + ':' + env.get('PATH', '')
    env.update(GIT_SUBMODULES_ARE_EVIL='1', GIT_OPTIONAL_LOCKS='0', PYTHONDONTWRITEBYTECODE='1',
               CCACHE_DISABLE='1', OMP_NUM_THREADS='1', OPENBLAS_NUM_THREADS='1',
               MKL_NUM_THREADS='1')
    name = 'COMPONENT_BUILD_' + time.strftime('%Y%m%d_%H%M%S')
    log = ROOT / (name + '.log')
    print('Actual isolated CMake target build -j2; no firmware link/run/upload.', flush=True)
    with log.open('w') as stream:
        run = subprocess.run(command, env=env, stdin=subprocess.DEVNULL,
                             stdout=stream, stderr=subprocess.STDOUT)
    after = isolation.stat_snapshot(focused=True)
    changed = [key for key in before if before[key] != after.get(key)]
    current_sources = {p: isolation.sha(Path(p)) for p in source_hashes}
    topics = isolation.BUILD / 'uORB/topics/uORBTopics.hpp'
    match = re.search(r'ORB_TOPICS_COUNT\{(\d+)\}', topics.read_text()) if topics.exists() else None
    actual_topic_count = int(match.group(1)) if match else None
    archives = [isolation.BUILD / 'external_modules/libgpenmpc_rfly_runtime_components.a',
                isolation.BUILD / 'external_modules/gpenmpc_rfly_canonical/libmodules__gpenmpc_rfly_canonical.a',
                isolation.BUILD / 'external_modules/gpenmpc_rfly_session/libmodules__gpenmpc_rfly_session.a']
    artifacts = [{'path': str(path), 'bytes': path.stat().st_size,
                  'sha256': isolation.sha(path)} for path in archives if path.exists()]
    main_symbol = False
    session_symbol = False
    if run.returncode == 0 and archives[1].exists():
        nm = Path(compiler).with_name('arm-none-eabi-nm')
        symbols = subprocess.run([str(nm), '--defined-only', str(archives[1])],
                                 text=True, capture_output=True, check=True).stdout
        (ROOT / (name + '_module_nm.txt')).write_text(symbols)
        main_symbol = bool(re.search(r'\bT gpenmpc_rfly_canonical_main\s*$', symbols, re.M))
        if archives[2].is_file():
            session_symbols = subprocess.run([str(nm), '--defined-only', str(archives[2])],
                text=True, capture_output=True, check=True).stdout
            session_symbol = bool(re.search(r'\bT gpenmpc_rfly_session_main\s*$', session_symbols, re.M))
    result = {'scope': 'ACTUAL_PRIVATE_CMAKE_COMPONENT_TARGET_BUILD',
              'command': command, 'exit_code': run.returncode,
              'log': str(log), 'log_sha256': isolation.sha(log),
              'source_sha256_before': source_hashes,
              'source_sha256_after': current_sources,
              'old_write_target_and_config_anchor_changes': changed,
              'generated_kernel_language': 'C',
              'private_nuttx_cache_and_make_working_directories_checked': True,
              'actual_generated_uorb_topic_count': actual_topic_count,
              'actual_archives': artifacts, 'actual_main_symbol_defined': main_symbol,
              'actual_session_main_symbol_defined': session_symbol,
              'module_entry': 'gpenmpc_rfly_canonical',
              'production_context_supplied': False,
              'firmware_linked': False, 'target_code_executed': False,

              'passed': run.returncode == 0 and not changed and current_sources == source_hashes and actual_topic_count == 311 and len(artifacts) == 3 and main_symbol and session_symbol}
    (ROOT / (name + '.json')).write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps({k: result[k] for k in ('scope', 'exit_code', 'passed')}))
    print(log.read_text()[-6000:])
    raise SystemExit(0 if result['passed'] else 1)


if __name__ == '__main__':
    main()
