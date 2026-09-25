#!/usr/bin/env python3
"""Compile one actual Guard/NativeLand specialization with this CMake's flags."""
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
source = root / 'probe_native_land_session_guard.cpp'
out = root / 'native_land_session_guard_arm'
out.mkdir(exist_ok=True)
database = json.loads((isolation.BUILD / 'compile_commands.json').read_text())
rows = [r for r in database if Path(r['file']).name in ('Px4CanonicalLocalIo.cpp', 'Px4CanonicalIo.cpp')]
assert len(rows) == 1
row = rows[0]
args = shlex.split(row['command'])
if Path(args[0]).name == 'ccache':
    args = args[1:]
compiler = args[0]
assert 'arm-none-eabi-g++' in compiler and '-nostdinc++' in args
for flag, value in (('-o', str(out / 'probe.o')), ('-MF', str(out / 'probe.d'))):
    if flag in args:
        args[args.index(flag)+1] = value
    elif flag == '-MF':
        args += ['-MMD', '-MF', value]
args[args.index(row['file'])] = str(source)
tracked = [source] + [root.parent / 'px4_runtime' / name for name in
    ('Px4ExecutionSessionGuard.hpp', 'SessionBoundGuard.hpp', 'Px4ReadOnlyGuard.hpp',
     'BoardSafetyEvidence.hpp', 'NativeLandOutputAuthority.hpp', 'NativeLandModeShape.hpp')]
before = {str(p): isolation.sha(p) for p in tracked}
env = dict(os.environ, GIT_OPTIONAL_LOCKS='0', CCACHE_DISABLE='1',
           PYTHONDONTWRITEBYTECODE='1', OMP_NUM_THREADS='1')
run = subprocess.run(args, cwd=row['directory'], env=env, capture_output=True, text=True)
log = out / ('compile_' + time.strftime('%Y%m%d_%H%M%S') + '.log')
log.write_text(run.stdout + run.stderr)
after = {str(p): isolation.sha(p) for p in tracked}
result = {'scope': 'ACTUAL_NUTTX_NATIVE_LAND_WITH_PX4_EXECUTION_SESSION_GUARD_COMPILE_ONLY',
          'exit_code': run.returncode, 'command': args,
          'source_sha256_before': before, 'source_sha256_after': after,
          'actual_guard_type': 'gpenmpc_rfly_px4::Px4ExecutionSessionGuard',
          'stub_guard': False, 'target_code_executed': False,
           'firmware_linked': False,
          'compile_log': str(log),
          'passed': run.returncode == 0 and before == after}
if (out / 'probe.o').exists() and run.returncode == 0:
    result['object_sha256'] = isolation.sha(out / 'probe.o')
    result['object_bytes'] = (out / 'probe.o').stat().st_size
receipt = out / 'RESULT.json'
if receipt.exists():
    previous = json.loads(receipt.read_text())
    history = previous.pop('attempt_history', [])
    result['attempt_history'] = history + [previous]
receipt.write_text(json.dumps(result, indent=2) + '\n')
print(json.dumps({k: result[k] for k in ('scope', 'exit_code', 'passed', 'actual_guard_type')}))
if run.returncode:
    print(run.stderr[-6000:])
raise SystemExit(0 if result['passed'] else 1)
