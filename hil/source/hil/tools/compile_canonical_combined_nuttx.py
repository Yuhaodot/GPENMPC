#!/usr/bin/env python3
"""Compile the combined generated 74 C TUs with real private FMUv6C flags.

Object/archive/size/stack inspection only. Never links an application, runs
PX4, accesses a board, modifies either PX4 source tree, or invents type stubs.
"""
import hashlib
import json
import os
from pathlib import Path
from gpenmpc_external_path import external_path
import re
import shlex
import subprocess
import sys

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
source_set = sys.argv[2] if len(sys.argv) == 3 else 'combined_numerics'
assert source_set in ('combined_numerics','closed_evidence','tracking_controller')
root = (build / 'evidence' / 'tracking_controller' if source_set == 'tracking_controller'
        else (Path(external_path('canonical_combined_numerics')) if source_set == 'combined_numerics'
              else Path(external_path('local_inner_closed_evidence'))))
generated = root / 'generated'
if source_set != 'combined_numerics':
    parity=json.loads((root/'RESULT.json').read_text())
    assert parity['pass'] and parity['generated'] and parity['sources_unchanged']
db = build / 'rfly_vendor_integration/application_integration/build_fmuv6c/compile_commands.json'
label = sys.argv[1] if len(sys.argv) >= 2 else 'arm_objects'
if not re.fullmatch(r'arm_[a-z][a-z0-9_]*', label):
    raise ValueError('Use an arm_ output label containing letters, digits and underscores.')
out = root / label
out.mkdir(exist_ok=False)
rows = [r for r in json.loads(db.read_text())
        if Path(r['file']).name == 'GPENMPC_Rfly_Canonical_Controller.c']
if rows:
    assert len(rows) == 1
    row = rows[0]
else:
    # Reuse the recorded NuttX target C compiler flags with these source and output paths.
    prior_path=Path(external_path('closed_evidence_arm_compile_receipt'))
    prior=json.loads(prior_path.read_text())
    assert prior['passed'] and prior['compiled_tus']==74
    row=prior['reference_compile_entry']
base = shlex.split(row['command'])
if Path(base[0]).name == 'ccache':
    base = base[1:]
assert 'arm-none-eabi-gcc' in base[0]
assert all(x in base for x in ('-mcpu=cortex-m7', '-mfpu=fpv5-d16', '-mfloat-abi=hard', '-std=gnu11'))
assert '-ffp-contract=off' in base
sources = sorted(generated.glob('*.c'))
assert len(sources) == 74
inputs = sources + sorted(generated.glob('*.h')) + [db, Path(__file__)]
if not rows: inputs.append(prior_path)
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest().upper()
before = {str(p): sha(p) for p in inputs}
env = dict(os.environ, CCACHE_DISABLE='1', PYTHONDONTWRITEBYTECODE='1', OMP_NUM_THREADS='1')
records = []
for source in sources:
    obj = out / (source.stem + '.o')
    args = list(base)
    args[args.index('-o') + 1] = str(obj)
    args[args.index('-c') + 1] = str(source)
    args[1:1] = ['-I' + str(generated), '-isystem', matlab_include,
                 '-fstack-usage', '-MD', '-MF', str(obj.with_suffix('.d'))]
    run = subprocess.run(args, cwd=row['directory'], env=env, capture_output=True, text=True)
    (out / (source.stem + '.log')).write_text(run.stdout + run.stderr)
    records.append(dict(source=str(source), command=args, compiler_exit_code=run.returncode,
                        object=str(obj) if obj.exists() else None,
                        object_sha256=sha(obj) if obj.exists() else None,
                        stdout=run.stdout, stderr=run.stderr))
    print(source.name, run.returncode, flush=True)
    if run.returncode:
        print(run.stdout + run.stderr, flush=True)
        break
passed = len(records) == 74 and all(r['compiler_exit_code'] == 0 for r in records)
size_output = ''
undefined = ''
attributes = ''
totals = dict(text=0, data=0, bss=0)
stack = []
if passed:
    objects = [r['object'] for r in records]
    # Change the compiler executable only, never gcc in its parent directory.
    utility = lambda name: str(Path(base[0]).with_name(Path(base[0]).name.replace('gcc', name)))
    archive = out / 'libcanonical_combined_m7.a'
    subprocess.run([utility('ar'), 'rcs', str(archive), *objects], check=True)
    size_output = subprocess.run([utility('size'), *objects], capture_output=True, text=True, check=True).stdout
    for line in size_output.splitlines()[1:]:
        parts = line.split()
        if len(parts) >= 4:
            for key, value in zip(('text', 'data', 'bss'), parts[:3]):
                totals[key] += int(value)
    undefined = subprocess.run([utility('nm'), '-u', str(archive)], capture_output=True, text=True, check=True).stdout
    attributes = subprocess.run([utility('readelf'), '-h', '-A', objects[0]], capture_output=True, text=True, check=True).stdout
    (out / 'SIZE.txt').write_text(size_output)
    (out / 'UNDEFINED.txt').write_text(undefined)
    (out / 'ARM_ATTRIBUTES.txt').write_text(attributes)
    for su in sorted(out.glob('*.su')):
        for line in su.read_text().splitlines():
            parts = line.split('\t')
            if len(parts) == 3:
                stack.append(dict(function=parts[0], bytes=int(parts[1]), kind=parts[2]))
after = {str(p): sha(p) for p in inputs}
passed = passed and before == after
report = dict(status='PASS_ACTUAL_M7_GENERATED_C_OBJECTS_ONLY' if passed else 'ACTUAL_M7_COMPILE_BOUNDARY',
              passed=passed, compile_database=str(db), reference_compile_entry=row,
              records=records, source_sha256_before=before, source_sha256_after=after,
              sources_unchanged=before == after, compiled_tus=len([r for r in records if not r['compiler_exit_code']]),
              raw_object_section_totals=totals, stack_usage=sorted(stack, key=lambda s: s['bytes'], reverse=True),
              stack_scope='Per-function compiler estimate only; no cumulative call-chain/task-stack or WCET proof',
              flags_scope='Original real private C flags retained, including its existing math optimization flags; add only own generated/type-header includes and dependency/stack output',
              target_numerics_executed=False,
              full_application_linked=False, board_actions=0, COM_UDP_calls=0,
              firmware_or_model_executed=False, library_sha256=sha(archive) if passed else None)
(out / 'RESULT.json').write_text(json.dumps(report, indent=2) + '\n')
print(json.dumps({k: report[k] for k in ('status', 'compiled_tus', 'sources_unchanged', 'raw_object_section_totals')}))
print(json.dumps(report['stack_usage'][:10], indent=2))
raise SystemExit(0 if passed else 1)
