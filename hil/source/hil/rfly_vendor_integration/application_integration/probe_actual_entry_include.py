#!/usr/bin/env python3
"""Compile the session entry with its MAVLink include dependencies."""
import json
import os
import shlex
import subprocess
import sys
import time
from pathlib import Path
sys.dont_write_bytecode = True
import configure_private_nuttx as isolation

root = Path(__file__).resolve().parent
database = json.loads((isolation.BUILD / 'compile_commands.json').read_text())
out = root / ('target_precheck_' + time.strftime('%Y%m%d_%H%M%S'))
out.mkdir()
results = []
for name in ('CanonicalSessionEntry.cpp', 'GPENMPCRflyCanonicalModule.cpp',
             'RegisteredCanonicalModuleContext.cpp', 'CanonicalApplicationOwner.cpp',
             'mavlink_messages.cpp', 'gpenmpc_mavlink_unity.cpp'):
    row, = [r for r in database if Path(r['file']).name == name]
    args = shlex.split(row['command'])
    if Path(args[0]).name == 'ccache':
        args = args[1:]
    assert '-nostdinc++' in args
    if name in ('CanonicalSessionEntry.cpp', 'GPENMPCRflyCanonicalModule.cpp'):
        assert '-Wframe-larger-than=2048' in args
    args[args.index('-o')+1] = str(out / (name + '.o'))
    if '-MF' in args:
        args[args.index('-MF')+1] = str(out / (name + '.d'))
    args += ['-fstack-usage']
    if name == 'CanonicalSessionEntry.cpp':
        # Exact official mavlink_c INTERFACE, CMakeLists.txt:85-98, including
        # its packed-wire diagnostic options. No frame/stack warning removed.
        for suffix in ('mavlink', 'mavlink/common', 'mavlink/uAvionix'):
            p = isolation.BUILD / suffix
            assert p.is_dir()
            args += ['-I' + str(p)]
        args += ['-Wno-address-of-packed-member', '-Wno-cast-align']
    source = Path(row['file'])
    before = isolation.sha(source)
    run = subprocess.run(args, cwd=row['directory'],
        env=dict(os.environ, CCACHE_DISABLE='1', PYTHONDONTWRITEBYTECODE='1'),
        capture_output=True, text=True)
    log = out / (name + '.log')
    log.write_text(run.stdout + run.stderr)
    record = {'file': name, 'command': args, 'exit_code': run.returncode,
        'source_unchanged': before == isolation.sha(source), 'source_sha256': before,
        'log': str(log)}
    results.append(record)
    print(json.dumps({k: record[k] for k in ('file','exit_code','source_unchanged')}), flush=True)
    if run.returncode:
        print(run.stderr[-3500:], flush=True)
result = {'scope': 'ACTUAL_NEW_TARGET_FLAGS_WITH_CORRECTED_MAVLINK_C_INTERFACE',
          'records': results,  'firmware_linked': False,
          'passed': all(r['exit_code'] == 0 and r['source_unchanged'] for r in results)}
(out / 'RESULT.json').write_text(json.dumps(result, indent=2) + '\n')
raise SystemExit(0 if result['passed'] else 1)
