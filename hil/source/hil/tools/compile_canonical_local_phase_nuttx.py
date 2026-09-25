#!/usr/bin/env python3
"""Independent phase C object/archive check with real FMUv6C compile flags.

No source tree/build graph modification, application build, model, COM or PX4.
"""
import hashlib
import json
import os
from pathlib import Path
import shlex
import subprocess

def matlab_include_path():
    root = os.environ.get('GPENMPC_MATLAB_ROOT', '').strip()
    if not root:
        raise RuntimeError('Set GPENMPC_MATLAB_ROOT to the MATLAB installation directory.')
    if os.name != 'nt' and len(root) >= 3 and root[1] == ':':
        root = subprocess.run(['wslpath', '-u', root], check=True, text=True,
                              capture_output=True).stdout.strip()
    include = Path(root) / 'extern' / 'include'
    if not include.is_dir():
        raise FileNotFoundError('MATLAB external headers were not found: ' + str(include))
    return str(include.resolve())


matlab_include = matlab_include_path()

build = Path(__file__).resolve().parent.parent
root = build / 'evidence/phase_controller'
generated = root / 'generated'
out = root / 'arm'
db = build / 'rfly_vendor_integration/application_integration/build_fmuv6c/compile_commands.json'
config = build / 'rfly_vendor_integration/application_integration/private_source/platforms/nuttx/NuttX/nuttx/include/nuttx/config.h'
assert config.is_file(), 'Wait for the generated NuttX configuration.'
native_path = root / 'native/RESULT.json'
native = json.loads(native_path.read_text(encoding='utf-8-sig'))
assert native['pass'] and native['numerical']['exact_rows'] == 2400
out.mkdir(exist_ok=False)
rows = [r for r in json.loads(db.read_text()) if Path(r['file']).name == 'GPENMPC_Rfly_Canonical_Controller.c']
assert len(rows) == 1
entry = rows[0]
base = shlex.split(entry['command'])
if Path(base[0]).name == 'ccache':
    base = base[1:]
assert all(x in base for x in ('-mcpu=cortex-m7', '-mfpu=fpv5-d16', '-mfloat-abi=hard', '-std=gnu11', '-ffp-contract=off'))
assert Path(base[0]).name == 'arm-none-eabi-gcc'
utility = lambda name: str(Path(base[0]).with_name('arm-none-eabi-' + name))
sha = lambda p: hashlib.sha256(Path(p).read_bytes()).hexdigest().upper()
sources = sorted(generated.glob('*.c'))
assert len(sources) == 3
inputs = sources + sorted(generated.glob('*.h')) + [db, config, native_path, Path(__file__)]
before = {str(p): sha(p) for p in inputs}
environment = dict(os.environ, CCACHE_DISABLE='1', PYTHONDONTWRITEBYTECODE='1', OMP_NUM_THREADS='1')
records = []
for source in sources:
    obj = out / (source.stem + '.o')
    command = list(base)
    command[command.index('-c') + 1] = str(source)
    command[command.index('-o') + 1] = str(obj)
    command[1:1] = ['-I' + str(generated), '-isystem', matlab_include,
                    '-fstack-usage', '-MD', '-MF', str(obj.with_suffix('.d'))]
    proc = subprocess.run(command, cwd=entry['directory'], env=environment, text=True, capture_output=True)
    (out / (source.stem + '.log')).write_text(proc.stdout + proc.stderr)
    records.append(dict(source=str(source), command=command, exit_code=proc.returncode,
                        object=str(obj), object_sha256=sha(obj) if obj.exists() else None))
    print(source.name, proc.returncode, flush=True)
    if proc.returncode:
        print(proc.stdout + proc.stderr, flush=True)
        break
passed = len(records) == 3 and all(r['exit_code'] == 0 for r in records)
size_text = undefined = defined = ''
totals = dict(text=0, data=0, bss=0)
stack = []
library = out / 'libcanonical_local_phase_m7.a'
if passed:
    objects = [r['object'] for r in records]
    subprocess.run([utility('ar'), 'rcs', str(library), *objects], check=True)
    size_text = subprocess.run([utility('size'), *objects], check=True, text=True, capture_output=True).stdout
    for line in size_text.splitlines()[1:]:
        parts = line.split()
        for name, value in zip(totals, parts[:3]):
            totals[name] += int(value)
    defined = subprocess.run([utility('nm'), '-g', '--defined-only', str(library)], check=True, text=True, capture_output=True).stdout
    undefined = subprocess.run([utility('nm'), '-u', str(library)], check=True, text=True, capture_output=True).stdout
    (out / 'SIZE.txt').write_text(size_text)
    (out / 'DEFINED_SYMBOLS.txt').write_text(defined)
    (out / 'UNDEFINED_SYMBOLS.txt').write_text(undefined)
    (out / 'DISASSEMBLY.txt').write_text(subprocess.run([utility('objdump'), '-dr', objects[0]], check=True, text=True, capture_output=True).stdout)
    for su in out.glob('*.su'):
        for line in su.read_text().splitlines():
            fields = line.split('\t')
            if len(fields) == 3:
                stack.append(dict(function=fields[0], bytes=int(fields[1]), kind=fields[2]))
    symbols = sorted(line.split()[-1] for line in defined.splitlines() if len(line.split()) == 3)
    expected = sorted('gpenmpcNative_canonicalLocalPhaseAdvance' + suffix for suffix in ('', '_initialize', '_terminate'))
    passed = passed and symbols == expected
after = {str(p): sha(p) for p in inputs}
passed = passed and before == after
report = dict(status='PASS_PHASE_C_2400_BITEXACT_AND_ACTUAL_M7_OBJECTS__NOT_RUNTIME_INTEGRATION' if passed else 'PHASE_NATIVE_OR_TARGET_BUILD_FAILED',
              passed=passed, native=native, compile_database=str(db), reference_compile_entry=entry,
              target_compilations=records, target_section_totals=totals, target_stack=stack,
              target_undefined_symbols=undefined, target_defined_symbols=defined,
              target_library=str(library), target_library_sha256=sha(library) if library.exists() else None,
              source_sha256_before=before, source_sha256_after=after, sources_unchanged=before == after,
              target_flags_scope='Compile with the FMUv6C C flags; generated MATLAB type headers precede legacy headers.',
              stack_scope='Per-function compiler static estimate; external libm and task context not bounded; not WCET.',
              symbol_isolation='The three phase-specific exports have unique names and require no prefix transformation.',
              raw_generated_c_unchanged=True, full_application_linked=False, phase_installed=False,
              matlab_runs=0, target_numerics_executed=False, board_actions=0, COM=0, UDP=0, solver=0, plant=0)
(root / 'NATIVE_AND_M7_RESULT.json').write_text(json.dumps(report, indent=2) + '\n')
print(json.dumps({k: report[k] for k in ('status', 'sources_unchanged', 'target_section_totals', 'target_stack')}, indent=2))
raise SystemExit(0 if passed else 1)
