#!/usr/bin/env python3
"""Compile the snapshot translation unit and collect object and stack sizes."""

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
prefix = 'slim_snapshot_nuttx_' + label
obj = root / (prefix + '.o')
receipt = root / ('SLIM_NUTTX_COMPILE_' + label + '.json')
if receipt.exists() or obj.exists():
    raise FileExistsError('Prior compile artifacts must not be overwritten')
px4 = Path(os.environ['GPENMPC_PX4_ROOT'])
database = component_input('GPENMPC_BUILD_ROOT') / 'compile_commands.json'
entries = json.loads(database.read_text())
entry, = [item for item in entries if item['file'] == str(px4 / 'src/modules/mavlink/mavlink_receiver.cpp')]
args = shlex.split(entry['command'])
assert '-std=gnu++14' in args and '-nostdinc++' in args and '-mcpu=cortex-m7' in args
generated = root.parent / 'evidence/arm_controller/GPENMPC_Rfly_Canonical_Controller_ert_rtw'
source = root / 'probe_slim_snapshot_nuttx.cpp'
args[args.index('-o') + 1] = str(obj)
args[args.index('-c') + 1] = str(source)
args[1:1] = ['-I' + str(generated), '-isystem',
             str(component_input('GPENMPC_NUTTX_INCLUDE')),
             '-ffp-contract=off', '-fstack-usage', '-MD', '-MF', str(root / (prefix + '.d'))]
inputs = [source, root / 'compile_slim_snapshot_nuttx.py', root / 'SlimSnapshotExecutor.hpp',
          root / 'SlimKernelCodec.hpp', root / 'SimulinkCanonicalPolicy.hpp', root / 'CanonicalRflyExecutor.hpp',
          root / 'RflySnapshotBoundExecutor.hpp', generated / 'GPENMPC_Rfly_Canonical_Controller.c',
          generated / 'GPENMPC_Rfly_Canonical_Controller.h', database]
for name in ['portable/CanonicalPortable.hpp', 'consumption/ConsumptionBinding.hpp',
             'px4_state_adapter/AtomicOdometryAdapter.hpp', 'argument_abi/CanonicalKernelArgumentCodec.hpp',
             'execution/CanonicalFullInnerExecutor.hpp', 'state_execution/SnapshotBoundExecutor.hpp']:
    inputs.append(root.parent / 'px4_full_inner' / name)
def hashes():
    return {str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in inputs}
before = hashes()
result = subprocess.run(args, cwd=entry['directory'], capture_output=True, text=True)
undefined = ''
sizes = {}
if result.returncode == 0:
    nm = args[0].replace('g++', 'nm')
    symbols = subprocess.run([nm, '-S', '--size-sort', str(obj)], capture_output=True, text=True, check=True).stdout
    for line in symbols.splitlines():
        columns = line.split()
        if len(columns) == 4 and columns[3].startswith('gpenmpc_slim_target_size_'):
            sizes[columns[3].removeprefix('gpenmpc_slim_target_size_')] = int(columns[1], 16)
    undefined = subprocess.run([nm, '-u', str(obj)], capture_output=True, text=True, check=True).stdout
after = hashes()
allowed = {'memcpy', 'memset', 'GPENMPC_Rfly_Canonical_Controller_step',
           'GPENMPC_Rfly_Canonical_Control_U', 'GPENMPC_Rfly_Canonical_Control_Y'}
unresolved = {line.split()[-1] for line in undefined.splitlines() if line.split()}
unexpected = sorted(unresolved - allowed)
stack_file = obj.with_suffix('.su')
record = {'scope': 'ACTUAL_FMUV6C_NUTTX_GNU14_SLIM_ALL_PATHS_COMPILE_ONLY',
          'compiler_exit_code': result.returncode, 'command_argv': args,
          'compile_database': str(database), 'working_directory': entry['directory'],
          'sources_unchanged': before == after, 'source_hashes': before,
          'object_sha256': hashlib.sha256(obj.read_bytes()).hexdigest() if obj.exists() else None,
          'target_sizes_bytes': sizes, 'stack_usage_raw': stack_file.read_text() if stack_file.exists() else '',
          'undefined_symbols': undefined, 'unexpected_undefined_symbols': unexpected,
          'no_libatomic_or_libstdcpp_exception_dependency': not unexpected and result.returncode == 0,
          'host_stubs': False, 'libstdcpp_injected': False,
          'stack_is_per_function_not_call_chain_peak': True, 'target_numerical_execution_tested': False,
          'firmware_linked': False, 'px4_executed': False,
          'compiler_stdout': result.stdout, 'compiler_stderr': result.stderr}
receipt.write_text(json.dumps(record, indent=2) + '\n')
print(json.dumps({key: record[key] for key in ['scope', 'compiler_exit_code', 'sources_unchanged',
      'target_sizes_bytes', 'object_sha256', 'unexpected_undefined_symbols']}))
print(result.stdout + result.stderr)
print(record['stack_usage_raw'])
raise SystemExit(result.returncode or (0 if before == after and not unexpected else 3))
