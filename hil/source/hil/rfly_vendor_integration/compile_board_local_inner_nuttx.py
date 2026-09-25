#!/usr/bin/env python3
"""Compile the local controller template with FMUv6C flags."""

# Component inputs are supplied by the invoking build/test environment.
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parent))
from component_inputs import component_input
component_input('GPENMPC_BUILD_ROOT')

import hashlib
import json
from pathlib import Path
import os
import shlex
import subprocess
import sys

root = Path(__file__).resolve().parent
px4 = Path(os.environ['GPENMPC_PX4_ROOT'])
db = component_input('GPENMPC_BUILD_ROOT') / 'compile_commands.json'
entry, = [e for e in json.loads(db.read_text()) if e['file'] == str(px4 / 'src/modules/mavlink/mavlink_receiver.cpp')]
source = root / 'probe_board_local_inner_nuttx.cpp'
label = sys.argv[1] if len(sys.argv) == 2 else 'verification'
assert (label and all(c.isalnum() or c in '_-' for c in label))
obj = root / ('board_local_inner_nuttx_' + label + '.o')
receipt = root / ('BOARD_LOCAL_INNER_NUTTX_COMPILE_' + label + '.json')
if obj.exists() or receipt.exists():
    raise FileExistsError('Preserve previous artifacts')
args = shlex.split(entry['command'])
assert all(a in args for a in ['-std=gnu++14', '-nostdinc++', '-mcpu=cortex-m7'])
args[args.index('-o') + 1] = str(obj)
args[args.index('-c') + 1] = str(source)
args[1:1] = ['-ffp-contract=off', '-fstack-usage']
inputs = [source, Path(__file__), root / 'BoardLocalInnerSchedule.hpp', db,
          root.parent / 'px4_full_inner/consumption/ConsumptionBinding.hpp',
          root.parent / 'px4_full_inner/consumption/CanonicalSha256.hpp',
          root.parent / 'px4_full_inner/portable/CanonicalPortable.hpp']
hashes = lambda: {str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in inputs}
before = hashes()
run = subprocess.run(args, cwd=entry['directory'], capture_output=True, text=True)
after = hashes()
undefined = ''
sizes = ''
if run.returncode == 0:
    nm = args[0].replace('g++', 'nm')
    undefined = subprocess.run([nm, '-u', str(obj)], capture_output=True, text=True, check=True).stdout
    sizes = subprocess.run([nm, '-S', '--size-sort', str(obj)], capture_output=True, text=True, check=True).stdout
su = obj.with_suffix('.su')
r = dict(scope='ACTUAL_FMUV6C_GNU14_SCHEDULER_COMPILE_ONLY', command_argv=args,
         working_directory=entry['directory'], compiler_exit_code=run.returncode,
         sources_unchanged=before == after, source_sha256=before,
         object_sha256=hashlib.sha256(obj.read_bytes()).hexdigest() if obj.exists() else None,
         undefined_symbols=undefined, size_symbols=sizes,
         stack_usage_raw=su.read_text() if su.exists() else '',
         compiler_stdout=run.stdout, compiler_stderr=run.stderr,
         full_application_linked=False, target_execution=False, actual_providers=False,
         clock_binding_verified=False)
receipt.write_text(json.dumps(r, indent=2) + '\n')
print(json.dumps({k: r[k] for k in ['scope', 'compiler_exit_code', 'sources_unchanged', 'object_sha256']}))
print(run.stdout + run.stderr)
print(undefined)
print(r['stack_usage_raw'])
raise SystemExit(run.returncode or (0 if before == after else 3))
