#!/usr/bin/env python3
"""Compile the actuator stream and PX4 v1.16 registry in an isolated overlay."""

# Component inputs are supplied by the invoking build/test environment.
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from component_inputs import component_input
component_input('GPENMPC_BUILD_ROOT')
component_input('GPENMPC_NUTTX_INCLUDE')
component_input('GPENMPC_STREAM_PATCH', directory=False)
component_input('GPENMPC_TOPIC_HEADERS')
component_input('GPENMPC_TOPIC_REPORT', directory=False)

import hashlib
import json
from pathlib import Path
import os
import re
import shlex
import subprocess
import sys

ROOT = Path(__file__).resolve().parent
BUILD = ROOT.parent.parent
PX4 = Path(os.environ['GPENMPC_PX4_ROOT'])
PX4_BUILD = component_input('GPENMPC_BUILD_ROOT')
META = component_input('GPENMPC_TOPIC_HEADERS')
DB = PX4_BUILD / 'compile_commands.json'
ORIGINAL = PX4 / 'src/modules/mavlink/mavlink_messages.cpp'
label = sys.argv[1] if len(sys.argv) == 2 else 'stream_build'
if not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_-]*', label):
    raise ValueError('Use a result name containing letters, digits, underscores or hyphens')
OUT = ROOT / label
if OUT.exists():
    raise RuntimeError('Refusing to overwrite evidence')
OUT.mkdir()
entry, = [e for e in json.loads(DB.read_text()) if e['file'] == str(ORIGINAL)]
base = shlex.split(entry['command'])
inputs = [ORIGINAL, DB, Path(__file__), ROOT / 'GPENMPCRflyHilStream.hpp',
          ROOT / 'RflyStreamAuthority.hpp', ROOT / 'RflyHilPacket.hpp',
          ROOT / 'SharedOutputRegistry.hpp', ROOT / 'SharedOutputRegistry.cpp',
          ROOT / 'LinkLifetimeRegistry.hpp', ROOT / 'LinkLifetimeRegistry.cpp',
          ROOT.parent / 'px4_runtime/InheritingMutex.hpp',
          ROOT / 'compile_stream_entry.cpp', component_input('GPENMPC_STREAM_PATCH', directory=False),
          PX4 / 'src/modules/mavlink/mavlink_main.h',
          PX4 / 'src/modules/mavlink/mavlink_messages.h',
          PX4 / 'src/modules/mavlink/mavlink_stream.h',
          PX4 / 'platforms/common/uORB/Subscription.hpp']
inputs += sorted((META / 'uORB/topics').glob('*.h'))
inputs += sorted((META / 'uORB/topics').glob('*.hpp'))
def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()
before = {str(p): digest(p) for p in inputs}
records = []
report = {'passed': False, 'source_hashes_before': before, 'records': records,
          'default_authority': 'DYNAMIC_ROUTER_UNBOUND_UNAVAILABLE',
          'registry_implemented': True, 'live_authority_observed': False,
          'firmware_linked': False, 'px4_executed': False, 'stubs': False,
           'uorb_publications': 0, 'transport_send_calls_executed': 0}
def run(name, argv, cwd=Path(entry['directory'])):
    proc = subprocess.run(argv, cwd=cwd, capture_output=True, text=True)
    (OUT / (name + '.log')).write_text(proc.stdout + proc.stderr)
    records.append({'name': name, 'argv': argv, 'cwd': str(cwd), 'exit_code': proc.returncode})
    print(name, proc.returncode, flush=True)
    if proc.returncode:
        print(proc.stdout + proc.stderr, flush=True)
        raise RuntimeError(name + ' failed')
    return proc.stdout
try:
    assert json.loads(component_input('GPENMPC_TOPIC_REPORT', directory=False).read_text())['passed']
    run('patch_check', ['git', '-C', str(PX4), 'apply', '--check', '--unidiff-zero',
                        str(component_input('GPENMPC_STREAM_PATCH', directory=False))])
    text = ORIGINAL.read_text()
    # Mechanical derivative of the actual registry TU: one include, one class,
    # and the matching feature guard only. Every other original stream remains.
    for old, new, count in (
        ('#include "streams/HIL_ACTUATOR_CONTROLS.hpp"', '#include "GPENMPCRflyHilStream.hpp"', 1),
        ('HIL_ACTUATOR_CONTROLS_HPP', 'GPENMPC_RFLY_HIL_STREAM_HPP', 2),
        ('create_stream_list_item<MavlinkStreamHILActuatorControls>()',
         'create_stream_list_item<MavlinkStreamGPENMPCRflyHILActuatorControls>()', 1)):
        assert text.count(old) == count, (old, text.count(old))
        text = text.replace(old, new)
    registry = OUT / 'mavlink_messages.cpp'
    registry.write_text(text)
    objects = []
    for source in (ROOT / 'compile_stream_entry.cpp', registry,
                   ROOT / 'SharedOutputRegistry.cpp', ROOT / 'LinkLifetimeRegistry.cpp'):
        obj = OUT / (source.name + '.obj')
        args = list(base)
        args[args.index('-c') + 1] = str(source)
        args[args.index('-o') + 1] = str(obj)
        args[1:1] = ['-I' + str(ROOT), '-I' + str(META),
                     '-I' + str(PX4 / 'src/modules/mavlink'),
                     '-isystem', str(component_input('GPENMPC_NUTTX_INCLUDE')),
                     '-MD', '-MF', str(OUT / (source.name + '.d'))]
        run('compile_' + source.name, args)
        nm = run('nm_' + source.name, [base[0].replace('g++', 'nm'), '-C', str(obj)])
        run('readelf_' + source.name, [base[0].replace('g++', 'readelf'), '-h', '-A', str(obj)])
        if source.name == 'compile_stream_entry.cpp':
            assert 'MavlinkStreamGPENMPCRflyHILActuatorControls::send()' in nm
            assert '__orb_actuator_outputs_rfly' in nm
            assert '__orb_actuator_outputs_sim' in nm
            assert 'gpenmpc_rfly_stream::shared_output_registry()' in nm
        if source.name == 'SharedOutputRegistry.cpp':
            assert 'gpenmpc_rfly_stream::shared_output_registry()' in nm
            assert '__cxa_guard' not in nm and '__atomic_' not in nm
            assert '_GLOBAL__sub_I' not in nm, 'No task-unsafe global mutex constructor'
        objects.append({'source': str(source), 'object': str(obj), 'sha256': digest(obj)})
    after = {str(p): digest(p) for p in inputs}
    report.update(objects=objects, source_hashes_after=after, source_stable=before == after,
                  actual_registry_derivative_sha256=digest(registry), passed=before == after)
except Exception as exc:
    report['failure'] = str(exc)
finally:
    report['status'] = 'PASS_REAL_V116_MAVLINK_STREAM_OBJECTS_ONLY' if report['passed'] else 'REAL_STREAM_COMPILE_FAILURE'
    (OUT / 'RESULT.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps({k: report.get(k) for k in ('status', 'source_stable', 'default_authority', 'failure')}))
raise SystemExit(0 if report['passed'] else 1)
