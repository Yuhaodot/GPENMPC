#!/usr/bin/env python3
"""Compile the PX4 I/O interface and check topic table sizes."""

# Component inputs are supplied by the invoking build/test environment.
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from component_inputs import component_input
component_input('GPENMPC_BUILD_ROOT')
component_input('GPENMPC_NUTTX_INCLUDE')
component_input('GPENMPC_TOPIC_HEADERS')

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
receipt = root / ('PX4_IO_COMPILE_' + label + '.json')
if receipt.exists():
    raise FileExistsError('Preserve previous compile receipts')
px4 = Path(os.environ['GPENMPC_PX4_ROOT'])
build = root.parent.parent
database = component_input('GPENMPC_BUILD_ROOT') / 'compile_commands.json'
meta = component_input('GPENMPC_TOPIC_HEADERS')
generated = build / 'evidence/arm_controller/GPENMPC_Rfly_Canonical_Controller_ert_rtw'
entry, = [e for e in json.loads(database.read_text())
          if e['file'] == str(px4 / 'src/modules/mavlink/mavlink_receiver.cpp')]
base = shlex.split(entry['command'])
assert '-std=gnu++14' in base and '-nostdinc++' in base and '-mcpu=cortex-m7' in base
assert json.loads((meta / 'RESULT.json').read_text())['passed']
sources = [root / 'Px4CanonicalIo.cpp', root / 'probe_px4_io_size.cpp']
inputs = sources + [root / 'Px4CanonicalIo.hpp', root / 'CommittedFeedback.hpp', root / 'ExecutionAuthority.hpp', Path(__file__), database]
inputs += sorted((meta / 'uORB/topics').glob('*.h')) + sorted((meta / 'uORB/topics').glob('*.hpp'))
for name in ['SlimSnapshotExecutor.hpp', 'SlimKernelCodec.hpp', 'CanonicalRflyExecutor.hpp',
             'RflySnapshotBoundExecutor.hpp', 'SimulinkCanonicalPolicy.hpp']:
    inputs.append(root.parent / name)
for name in ['portable/CanonicalPortable.hpp', 'consumption/ConsumptionBinding.hpp',
             'px4_state_adapter/AtomicOdometryAdapter.hpp', 'argument_abi/CanonicalKernelArgumentCodec.hpp']:
    inputs.append(build / 'px4_full_inner' / name)
inputs += [generated / 'GPENMPC_Rfly_Canonical_Controller.c', generated / 'GPENMPC_Rfly_Canonical_Controller.h',
           px4 / 'platforms/common/uORB/Subscription.hpp', px4 / 'platforms/common/uORB/Publication.hpp']
def hashes():
    return {str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in inputs}
before = hashes()
records = []
size = None
passed = True
for source in sources:
    obj = root / (source.stem + '_' + label + '.o')
    if obj.exists():
        raise FileExistsError('Preserve previous object')
    args = list(base)
    args[args.index('-c') + 1] = str(source)
    args[args.index('-o') + 1] = str(obj)
    args[1:1] = ['-I' + str(meta), '-I' + str(generated), '-isystem',
                 str(component_input('GPENMPC_NUTTX_INCLUDE')),
                 '-ffp-contract=off', '-fstack-usage', '-MD', '-MF', str(obj.with_suffix('.d'))]
    run = subprocess.run(args, cwd=entry['directory'], capture_output=True, text=True)
    record = {'source': str(source), 'exact_command_argv': args, 'exit_code': run.returncode,
              'stdout': run.stdout, 'stderr': run.stderr, 'object': str(obj)}
    if run.returncode == 0:
        nm = base[0].replace('g++', 'nm')
        record['object_sha256'] = hashlib.sha256(obj.read_bytes()).hexdigest()
        record['undefined_symbols'] = subprocess.run([nm, '-u', '-C', str(obj)], capture_output=True, text=True, check=True).stdout
        record['symbols_with_size'] = subprocess.run([nm, '-S', '--size-sort', '-C', str(obj)], capture_output=True, text=True, check=True).stdout
        record['stack_usage_raw'] = obj.with_suffix('.su').read_text()
        deps = obj.with_suffix('.d').read_text().replace('\\\n', ' ')
        # All included topic headers/table must be from the single full overlay.
        record['old_generated_topic_headers_used'] = str(component_input('GPENMPC_BUILD_ROOT') / 'uORB/topics') in deps
        record['full_alias_table_used'] = str(meta / 'uORB/topics/uORBTopics.hpp') in deps
        passed = passed and record['full_alias_table_used'] and not record['old_generated_topic_headers_used']
        for line in record['symbols_with_size'].splitlines():
            if line.endswith(' gpenmpc_px4_io_target_size'):
                size = int(line.split()[1], 16)
    else:
        passed = False
    records.append(record)
    print(source.name, run.returncode, flush=True)
    if run.returncode:
        print(run.stderr, flush=True)
        break
after = hashes()
for record in records:
    undefined = record.get('undefined_symbols', '')
    record['libatomic_or_cpp_exception_dependency'] = any(mark in undefined for mark in
        ('__atomic_', '__cxa_', '__gxx_personality', '_Unwind_', 'std::', '__throw_'))
    passed = passed and not record['libatomic_or_cpp_exception_dependency']
passed = passed and before == after and len(records) == 2
report = {'scope': 'ACTUAL_FMUV6C_NUTTX_PX4_UORB_COMPONENT_COMPILE_ONLY', 'passed': passed,
          'compile_database': str(database), 'working_directory': entry['directory'], 'records': records,
          'source_hashes': before, 'sources_unchanged': before == after, 'target_object_size_bytes': size,
          'all_topic_headers_must_use_same_311_topic_alias_ingress_table': True,
          'authority_is_external_interface_only': True, 'concrete_authority_implemented': False,
          'scheduler_or_cli_implemented': False, 'publication_is_not_downstream_consumption': True,
          'stack_is_per_function_not_call_chain_peak': True, 'host_stubs': False,
          'firmware_linked': False, 'target_code_executed': False}
receipt.write_text(json.dumps(report, indent=2) + '\n')
print(json.dumps({'passed': passed, 'sources_unchanged': before == after, 'target_object_size_bytes': size}))
for record in records:
    print(record.get('undefined_symbols', ''))
    print('\n'.join(line for line in record.get('stack_usage_raw', '').splitlines() if 'Px4CanonicalIo' in line))
raise SystemExit(0 if passed else 1)
