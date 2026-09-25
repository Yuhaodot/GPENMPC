#!/usr/bin/env python3
'HIL firmware build support.'

# Component inputs are supplied by the invoking build/test environment.
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from component_inputs import component_input
component_input('GPENMPC_BUILD_ROOT')
component_input('GPENMPC_NUTTX_INCLUDE')
component_input('GPENMPC_TOPIC_HEADERS')

import hashlib
import difflib
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
DB = component_input('GPENMPC_BUILD_ROOT') / 'compile_commands.json'
META = component_input('GPENMPC_TOPIC_HEADERS')
label = sys.argv[1] if len(sys.argv) == 2 else 'output_guard'
if not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_-]*', label):
    raise ValueError('Use a result name containing letters, digits, underscores or hyphens')
OUT = ROOT / label
if OUT.exists():
    raise RuntimeError('Refusing to overwrite evidence')
OUT.mkdir()
entries = json.loads(DB.read_text())
pairs = [('Px4ReadOnlyGuard.cpp', 'src/modules/mavlink/mavlink_messages.cpp'),
         ('ReadOnlyPwmState.cpp', 'src/drivers/pwm_out/PWMOut.cpp'),
         ('probe_readonly_guard_authority.cpp', 'src/modules/mavlink/mavlink_messages.cpp')]
drivers = [
    ('px4io_readonly.cpp', 'src/drivers/px4io/px4io.cpp', 'PX4IO', 'gpenmpc_readonly_px4io_running', 'PX4IO'),
    ('DShot_readonly.cpp', 'src/drivers/dshot/DShot.cpp', 'DShot', 'gpenmpc_readonly_dshot_running', 'DSHOT')
]
pairs += [(local, original) for local, original, _, _, _ in drivers]
inputs = [ROOT / name for name in ('Px4ReadOnlyGuard.cpp', 'Px4ReadOnlyGuard.hpp',
          'ReadOnlyPwmState.cpp', 'BoardSafetyEvidence.hpp', 'test_board_safety_evidence.cpp')]
inputs += [ROOT / 'probe_readonly_guard_authority.cpp', ROOT / 'CanonicalOutputAuthority.hpp',
           ROOT / 'ExecutionAuthority.hpp', ROOT.parent / 'px4_stream/RflyStreamAuthority.hpp']
inputs += [Path(__file__), DB, PX4 / 'src/modules/mavlink/mavlink_main.cpp',
    PX4 / 'src/modules/mavlink/mavlink_main.h', PX4 / 'src/drivers/pwm_out/PWMOut.hpp',
    PX4 / 'platforms/common/include/px4_platform_common/module.h',
    PX4 / 'boards/px4/fmu-v6c/src/board_config.h', PX4 / 'src/lib/parameters/param.h',
    component_input('GPENMPC_BUILD_ROOT') / 'parameters.json', component_input('GPENMPC_BUILD_ROOT') / 'NuttX/nuttx/.config']
inputs += [component_input('GPENMPC_BUILD_ROOT') / 'px4_boardconfig.h']
for _, original, _, _, _ in drivers:
    inputs += [PX4 / original, (PX4 / original).parent / 'CMakeLists.txt']
    entry, = [e for e in entries if e['file'] == str(PX4 / original)]
    argv = shlex.split(entry['command'])
    inputs.append(Path(entry['directory']) / argv[argv.index('-o') + 1])
inputs += sorted((META / 'uORB/topics').glob('*.h')) + sorted((META / 'uORB/topics').glob('*.hpp'))
def hashes():
    return {str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in inputs}
before = hashes()
records = []
report = {'passed': False, 'records': records, 'source_hashes_before': before,
          'scope': 'REAL_NUTTX_BOARD_GUARD_COMPILE_ONLY', 'stubs': False,
          'board_api_calls_executed': 0, 'parameter_writes': 0,
          'firmware_linked': False, 'target_code_executed': False}
try:
    config = (component_input('GPENMPC_BUILD_ROOT') / 'px4_boardconfig.h').read_text()
    report['driver_probes'] = []
    for local, original, driver, symbol, option in drivers:
        # Presence is proven separately from a stopped/running observation.
        # This exact target builds both modules; no !CONFIG -> stopped shortcut.
        assert re.search(r'^#define CONFIG_DRIVERS_' + option + r' 1$', config, re.MULTILINE)
        original_path = PX4 / original
        original_bytes = original_path.read_bytes()
        appendix = ('\n// Project-only read-only observer; no start/stop/CLI or hardware command.\n'
                    'extern "C" __EXPORT bool ' + symbol + '() noexcept\n'
                    '{\n\treturn ' + driver + '::is_running();\n}\n')
        derived = OUT / local
        derived.write_bytes(original_bytes + appendix.encode())
        assert derived.read_bytes()[:len(original_bytes)] == original_bytes
        patch = OUT / (driver + '_readonly_probe.patch')
        patch.write_text(''.join(difflib.unified_diff(original_path.read_text().splitlines(True),
            derived.read_text().splitlines(True), fromfile='a/' + original, tofile='b/' + original)))
        check = subprocess.run(['git', '-C', str(PX4), 'apply', '--check', str(patch)], capture_output=True, text=True)
        (OUT / (driver + '_patch_check.log')).write_text(check.stdout + check.stderr)
        assert check.returncode == 0, check.stderr
        report['driver_probes'].append({'driver': driver, 'built_in_actual_target': True,
            'config_macro': 'CONFIG_DRIVERS_' + option, 'original_source': str(original_path),
            'derived_source': str(derived), 'original_bytes_preserved': True, 'patch': str(patch),
            'patch_check_exit_code': check.returncode, 'query_symbol': symbol,
            'query_is_point_observation_not_physical_isolation': True})
    assert json.loads((META / 'RESULT.json').read_text())['passed']
    topic_table = (META / 'uORB/topics/uORBTopics.hpp').read_text()
    topic_count = int(re.search(r'ORB_TOPICS_COUNT\{(\d+)\}', topic_table).group(1))
    assert topic_count == 311
    report['unified_topic_count'] = topic_count
    report['unified_topic_table'] = str(META / 'uORB/topics/uORBTopics.hpp')
    for local, original in pairs:
        entry, = [e for e in entries if e['file'] == str(PX4 / original)]
        args = shlex.split(entry['command'])
        obj = OUT / (local + '.obj')
        source = OUT / local if any(local == driver[0] for driver in drivers) else ROOT / local
        args[args.index('-c') + 1] = str(source)
        args[args.index('-o') + 1] = str(obj)
        args[1:1] = ['-I' + str(META), '-I' + str(ROOT.parent / 'official_io_nuttx/overlay'),
                     '-I' + str(PX4 / 'src/modules/mavlink'),
                     '-I' + str((PX4 / original).parent),
                     '-isystem', str(component_input('GPENMPC_NUTTX_INCLUDE')),
                     '-MD', '-MF', str(OUT / (local + '.d')), '-fstack-usage']
        proc = subprocess.run(args, cwd=entry['directory'], capture_output=True, text=True)
        (OUT / (local + '.log')).write_text(proc.stdout + proc.stderr)
        record = {'source': str(source), 'original_compile_source': entry['file'],
                  'argv': args, 'exit_code': proc.returncode}
        records.append(record)
        print(local, proc.returncode, flush=True)
        if proc.returncode:
            print(proc.stdout + proc.stderr, flush=True)
            raise RuntimeError(local + ' failed')
        record['object'] = str(obj)
        record['object_sha256'] = hashlib.sha256(obj.read_bytes()).hexdigest()
        for name, flags in [('nm', ['-C']), ('readelf', ['-h', '-A'])]:
            result = subprocess.run([args[0].replace('g++', name), *flags, str(obj)], capture_output=True, text=True, check=True)
            (OUT / (local + '.' + name)).write_text(result.stdout)
        deps = (OUT / (local + '.d')).read_text()
        record['old_topic_headers_used'] = str(component_input('GPENMPC_BUILD_ROOT') / 'uORB/topics') in deps
        record['baseline_topic_headers_used'] = str(Path(os.environ['GPENMPC_BASELINE_TOPIC_HEADERS'])) in deps
        record['unified_311_topic_table_used'] = str(META / 'uORB/topics/uORBTopics.hpp') in deps
        assert not record['old_topic_headers_used']
        assert not record['baseline_topic_headers_used']
        assert record['unified_311_topic_table_used']
        for driver_local, _, driver, symbol, _ in drivers:
            if local != driver_local:
                continue
            original_args = shlex.split(entry['command'])
            original_obj = Path(entry['directory']) / original_args[original_args.index('-o') + 1]
            nm_tool = args[0].replace('g++', 'nm')
            original_nm = subprocess.run([nm_tool, '-C', str(original_obj)], capture_output=True, text=True, check=True).stdout
            derived_nm = (OUT / (local + '.nm')).read_text()
            slot = 'ModuleBase<' + driver + '>::_task_id'
            original_slots = [line for line in original_nm.splitlines() if '::_task_id' in line]
            derived_slots = [line for line in derived_nm.splitlines() if '::_task_id' in line]
            assert len(original_slots) == 1 and len(derived_slots) == 1
            assert original_slots[0].split(maxsplit=2)[1:] == derived_slots[0].split(maxsplit=2)[1:]
            assert slot in original_slots[0]
            disassembly = subprocess.run([args[0].replace('g++', 'objdump'), '-drC', str(obj)],
                                         capture_output=True, text=True, check=True).stdout
            probe_assembly = disassembly.split('<' + symbol + '>:', 1)[1].split('\nDisassembly of section', 1)[0]
            assert slot in probe_assembly, 'Probe must relocate against actual original driver task slot'
            (OUT / (driver + '_probe_disassembly.log')).write_text(probe_assembly)
            record['original_task_slots'] = original_slots
            record['derived_task_slots'] = derived_slots
            record['same_original_task_slot_verified'] = True
            record['query_relocation_target'] = slot
            record['derived_source_sha256'] = hashlib.sha256(source.read_bytes()).hexdigest()
    after = hashes()
    report.update(source_hashes_after=after, source_stable=before == after, passed=before == after)
except Exception as exc:
    report['failure'] = str(exc)
finally:
    (OUT / 'RESULT.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps({k: report.get(k) for k in ('passed', 'source_stable', 'failure')}))
raise SystemExit(0 if report['passed'] else 1)
