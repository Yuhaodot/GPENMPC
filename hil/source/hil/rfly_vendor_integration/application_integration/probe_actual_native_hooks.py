#!/usr/bin/env python3
"""Compile the heartbeat and native-writer hooks with target flags."""
import json
import os
import shlex
import subprocess
import time
from pathlib import Path
import configure_private_nuttx as isolation

ROOT = Path(__file__).resolve().parent
RFLY = ROOT.parent
database = json.loads((isolation.BUILD / 'compile_commands.json').read_text())
out = ROOT / ('native_hook_precheck_' + time.strftime('%Y%m%d_%H%M%S'))
out.mkdir()
records = []
for name in ('mavlink_receiver.cpp', 'gpenmpc_mavlink_unity.cpp', 'GPENMPCTrajectoryExec.cpp'):
    row, = [item for item in database if Path(item['file']).name == name]
    args = shlex.split(row['command'])
    if Path(args[0]).name == 'ccache':
        args = args[1:]
    assert 'arm-none-eabi-g++' in args[0] and '-nostdinc++' in args
    source = Path(row['file'])
    if name == 'GPENMPCTrajectoryExec.cpp':
        source = ROOT / 'overlay/src/modules/gpenmpc_trajectory_exec/GPENMPCTrajectoryExec.cpp'
        args[args.index('-c') + 1] = str(source)
        args += ['-I' + str(isolation.PX4 / 'src/modules/gpenmpc_trajectory_exec'),
                 '-I' + str(RFLY / 'px4_stream')]
    args[args.index('-o') + 1] = str(out / (name + '.o'))
    if '-MF' in args:
        args[args.index('-MF') + 1] = str(out / (name + '.d'))
    args += ['-fstack-usage']
    before = isolation.sha(source)
    run = subprocess.run(args, cwd=row['directory'],
        env=dict(os.environ, CCACHE_DISABLE='1', PYTHONDONTWRITEBYTECODE='1'),
        text=True, capture_output=True)
    log = out / (name + '.log')
    log.write_text(run.stdout + run.stderr)
    record = {'file': str(source), 'command': args, 'exit_code': run.returncode,
              'source_sha256': before, 'source_unchanged': before == isolation.sha(source),
              'log': str(log)}
    records.append(record)
    print(json.dumps({'file': name, 'exit_code': run.returncode}), flush=True)
    if run.returncode:
        print(run.stderr[-4000:], flush=True)
result = {'scope': 'ACTUAL_ARM_PRIVATE_NATIVE_WRITER_AND_ORIGINAL_HRT_HOOKS',
          'records': records,  'target_execution': False,
          'passed': all(item['exit_code'] == 0 and item['source_unchanged'] for item in records)}
(out / 'RESULT.json').write_text(json.dumps(result, indent=2) + '\n')
raise SystemExit(0 if result['passed'] else 1)
