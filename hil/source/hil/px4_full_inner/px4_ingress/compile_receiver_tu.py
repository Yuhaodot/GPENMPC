#!/usr/bin/env python3
"""Compile the isolated receiver translation unit using the PX4 ARM command."""

# Component inputs are supplied by the invoking build/test environment.
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'rfly_vendor_integration'))
from component_inputs import component_input
component_input('GPENMPC_BUILD_ROOT')
component_input('GPENMPC_NUTTX_INCLUDE')
component_input('GPENMPC_RECEIVER_PATCH', directory=False)

import hashlib
import json
from pathlib import Path
import os
import shlex
import subprocess
import sys

root = Path(__file__).resolve().parent
label = sys.argv[1] if len(sys.argv) == 2 else ''
if label and not all(c.isalnum() or c in '_-' for c in label):
    raise ValueError('Use a result name containing letters, digits, underscores or hyphens')
suffix = '_' + label if label else ''
px4 = Path(os.environ['GPENMPC_PX4_ROOT'])
database = component_input('GPENMPC_BUILD_ROOT') / 'compile_commands.json'
source = px4 / 'src/modules/mavlink/mavlink_receiver.cpp'
entries = json.loads(database.read_text())
entry, = [item for item in entries if item['file'] == str(source)]
args = shlex.split(entry['command'])
overlay = root / 'receiver_tu_overlay/src/modules/mavlink'
object_path = root / ('mavlink_receiver.actual_arm' + suffix + '.obj')
args[args.index('-o') + 1] = str(object_path)
args[args.index('-c') + 1] = str(overlay / source.name)
args[1:1] = ['-I' + str(root / 'generated'), '-I' + str(overlay),
             '-I' + str(source.parent)]
if label:
    args[1:1] = ['-isystem', str(component_input('GPENMPC_NUTTX_INCLUDE')),
                 '-MD', '-MF', str(root / ('receiver_tu' + suffix + '.d'))]
# Reversing the delivered patch checks the isolated files contain that exact patch.
patch_check = subprocess.run(['git', 'apply', '--reverse', '--check', '--unsafe-paths',
                             '--directory=' + str(root / 'receiver_tu_overlay'),
                             str(component_input('GPENMPC_RECEIVER_PATCH', directory=False))],
                            cwd=root, capture_output=True, text=True)
if patch_check.returncode:
    raise RuntimeError('Isolated patch check failed: ' + patch_check.stderr)
print('Executing one actual ARM receiver translation-unit compiler command', flush=True)
result = subprocess.run(args, cwd=entry['directory'], capture_output=True, text=True)
(root / ('receiver_tu_compile' + suffix + '.log')).write_text(result.stdout + result.stderr)
record = {'compile_database': str(database), 'original_source': str(source),
          'working_directory': entry['directory'], 'command_argv': args,
          'patch_reverse_check': patch_check.returncode, 'compiler_exit_code': result.returncode,
          'object': str(object_path) if result.returncode == 0 else None,
          'object_sha256': hashlib.sha256(object_path.read_bytes()).hexdigest() if result.returncode == 0 else None,
          'stub_headers_used': False, 'firmware_linked': False, 'px4_executed': False,
          'uorb_broker_executed': False, 'board_actions': 0,
          'stdout': result.stdout, 'stderr': result.stderr}
(root / ('RESULT_receiver_tu' + suffix + '.json')).write_text(json.dumps(record, indent=2) + '\n')
print(json.dumps({key: record[key] for key in ['compiler_exit_code', 'object', 'object_sha256']}))
print(result.stdout + result.stderr)
raise SystemExit(result.returncode)
