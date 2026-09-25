#!/usr/bin/env python3
"""Compile the RflySim snapshot interface with PX4 GNU C++14 flags."""

# Component inputs are supplied by the invoking build/test environment.
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parent))
from component_inputs import component_input
component_input('GPENMPC_BUILD_ROOT')
component_input('GPENMPC_NUTTX_INCLUDE')

import hashlib
import json
from pathlib import Path
import os
import shlex
import subprocess
import sys

root = Path(__file__).resolve().parent
label = sys.argv[1] if len(sys.argv) == 2 else 'verification'
if not (label and all(c.isalnum() or c in '_-' for c in label)):
    raise ValueError('Use a result name containing letters, digits, underscores or hyphens')
out = root / ('nuttx_compile_' + label)
if out.exists():
    raise FileExistsError('Do not overwrite an existing compiler result')
out.mkdir()
px4 = Path(os.environ['GPENMPC_PX4_ROOT'])
database = component_input('GPENMPC_BUILD_ROOT') / 'compile_commands.json'
entries = json.loads(database.read_text())
entry, = [item for item in entries if item['file'] == str(px4 / 'src/modules/mavlink/mavlink_receiver.cpp')]
args = shlex.split(entry['command'])
assert '-std=gnu++14' in args and '-nostdinc++' in args
generated = root.parent / 'evidence/arm_controller/GPENMPC_Rfly_Canonical_Controller_ert_rtw'
source = root / 'probe_rfly_snapshot_nuttx.cpp'
obj = out / 'rfly_snapshot.obj'
args[args.index('-o') + 1] = str(obj)
args[args.index('-c') + 1] = str(source)
args[1:1] = ['-I' + str(generated), '-isystem',
             str(component_input('GPENMPC_NUTTX_INCLUDE')),
             '-ffp-contract=off', '-fstack-usage', '-MD', '-MF', str(out / 'rfly_snapshot.d')]
inputs = [source, root / 'SimulinkCanonicalPolicy.hpp', root / 'CanonicalRflyExecutor.hpp',
          root / 'RflySnapshotBoundExecutor.hpp', generated / 'GPENMPC_Rfly_Canonical_Controller.c',
          generated / 'GPENMPC_Rfly_Canonical_Controller.h']
for name in ['portable/CanonicalPortable.hpp', 'consumption/ConsumptionBinding.hpp',
             'px4_state_adapter/AtomicOdometryAdapter.hpp', 'argument_abi/CanonicalKernelArgumentCodec.hpp',
             'execution/CanonicalFullInnerExecutor.hpp', 'state_execution/SnapshotBoundExecutor.hpp']:
    inputs.append(root.parent / 'px4_full_inner' / name)
def hashes():
    return {str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in inputs}
before = hashes()
result = subprocess.run(args, cwd=entry['directory'], capture_output=True, text=True)
(out / 'COMPILE.log').write_text(result.stdout + result.stderr)
undefined = ''
if result.returncode == 0:
    nm = args[0].replace('g++', 'nm')
    report = subprocess.run([nm, '-S', '--size-sort', str(obj)], capture_output=True, text=True, check=True)
    (out / 'SYMBOL_SIZES.txt').write_text(report.stdout)
    undefined = subprocess.run([nm, '-u', str(obj)], capture_output=True, text=True, check=True).stdout
    (out / 'UNDEFINED.txt').write_text(undefined)
after = hashes()
record = {'scope': 'ACTUAL_PX4_FMU_V6C_NUTTX_GNU14_COMPILE_ONLY', 'command_argv': args,
          'compile_database': str(database), 'working_directory': entry['directory'],
          'compiler_exit_code': result.returncode, 'sources_unchanged': before == after,
          'source_hashes': before, 'object_sha256': hashlib.sha256(obj.read_bytes()).hexdigest() if obj.exists() else None,
          'host_stubs': False, 'libstdcpp_injected': False, 'same_host_target_policy': True,
          'firmware_linked': False, 'px4_executed': False,
          'undefined_symbols': undefined, 'stderr': result.stderr}
(out / 'RESULT.json').write_text(json.dumps(record, indent=2) + '\n')
print(json.dumps({k: record[k] for k in ['scope', 'compiler_exit_code', 'sources_unchanged', 'object_sha256']}))
print(result.stdout + result.stderr)
if result.returncode == 0:
    print('\n'.join(line for line in report.stdout.splitlines() if 'gpenmpc_rfly_target_size' in line))
    for stack in out.glob('*.su'):
        print(stack.read_text())
raise SystemExit(result.returncode or (0 if before == after else 3))
