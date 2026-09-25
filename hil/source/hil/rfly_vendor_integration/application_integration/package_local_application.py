#!/usr/bin/env python3
"""Package a successful build through the PX4 package target and retain artifacts."""

# Component inputs are supplied by the invoking build/test environment.
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from component_inputs import component_input
component_input('GPENMPC_BUILD_ROOT')
component_input('GPENMPC_BASELINE_BUILD_ROOT')

import base64
import hashlib
import json
import os
from pathlib import Path
import re
import shlex
import shutil
import subprocess
import sys
import time
import zlib

sys.dont_write_bytecode = True
import configure_private_nuttx as iso
from generated_ingress_build_delta import adjudicate

ROOT = Path(__file__).resolve().parent
assert len(sys.argv) in (3, 4, 5) and all(re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_-]*', a) for a in sys.argv[1:])
link_suffix = '_' + sys.argv[3] if len(sys.argv) >= 4 else ''
package_suffix = '_' + sys.argv[4] if len(sys.argv) == 5 else ''
OUT = Path(os.environ.get('GPENMPC_LOCAL_RUNS', str(ROOT / 'local_runtime_integration'))) / sys.argv[1]
assert OUT.is_absolute()
PRIOR = ROOT / 'local_runtime_integration' / sys.argv[2]
if not PRIOR.is_dir():
    PRIOR = OUT.parent / sys.argv[2]
assert OUT != PRIOR
RECEIPT = OUT / ('APPLICATION_PACKAGE' + package_suffix + '.json')
assert not RECEIPT.exists(), 'Package receipt already exists'
LINK_RECEIPT = OUT / ('RESULT' + link_suffix + '.json')
link = json.loads(LINK_RECEIPT.read_text())
old = json.loads((PRIOR / 'APPLICATION_PACKAGE.json').read_text())
assert old['passed']
prior_artifacts = PRIOR / 'preserved_artifacts'
if not prior_artifacts.is_dir():
    prior_artifacts = PRIOR / 'artifacts'
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest().upper()
record = lambda p: dict(path=str(p), bytes=p.stat().st_size, sha256=sha(p))
link_sources = dict(link['sources_before'])
link_adjudication = None
if not link['passed']:
    # Validate the generated queue constant against its configured value.
    note, link_sources = adjudicate(link, iso.BUILD)
    note['original_link_receipt'] = record(LINK_RECEIPT)
    note['adjudicator'] = record(ROOT / 'generated_ingress_build_delta.py')
    note_path = OUT / 'GENERATED_SOURCE_BUILD_ADJUDICATION.json'
    with note_path.open('x') as stream:
        json.dump(note, stream, indent=2)
        stream.write('\n')
    link_adjudication = record(note_path)
name = lambda p: p.replace('\\', '/').rsplit('/', 1)[-1]
for row in old['artifacts']:
    saved = prior_artifacts / name(row['path'])
    assert saved.stat().st_size == row['bytes'] and sha(saved) == row['sha256']
elf = iso.BUILD / 'px4_fmu-v6c_default.elf'
expected = next(r['sha256'] for r in link['artifacts'] if r['path'].endswith('.elf'))
assert sha(elf) == expected
db = json.loads((iso.BUILD / 'compile_commands.json').read_text())
cmd = shlex.split(next(r['command'] for r in db if r['file'].endswith('/CanonicalLocalExchangeCore.cpp')))
compiler = Path(cmd[1] if Path(cmd[0]).name == 'ccache' else cmd[0])
assert compiler.name == 'arm-none-eabi-g++'
env = dict(os.environ, CCACHE_DISABLE='1', GIT_SUBMODULES_ARE_EVIL='1',
           GIT_OPTIONAL_LOCKS='0', PYTHONDONTWRITEBYTECODE='1', OMP_NUM_THREADS='1',
           MKL_NUM_THREADS='1', OPENBLAS_NUM_THREADS='1', CMAKE_BUILD_PARALLEL_LEVEL='2')
env['PATH'] = str(compiler.parent) + ':' + env.get('PATH', '')
protected = [elf, *sorted(prior_artifacts.iterdir()),
    component_input('GPENMPC_BASELINE_BUILD_ROOT') / 'px4_fmu-v6c_default.px4',
    iso.PX4 / 'boards/px4/fmu-v6c/extras/px4_fmu-v6c_bootloader.bin',
    iso.PRIVATE / 'boards/px4/fmu-v6c/extras/px4_fmu-v6c_bootloader.bin',
    iso.PX4 / 'boards/px4/fmu-v6c/extras/px4_io-v2_default.bin']
before = {str(p): record(p) for p in protected}
anchors = iso.stat_snapshot(focused=True)
result = dict(scope='FMUV6C_LOCAL_FULL_INNER_APPLICATION_PACKAGE',
    passed=False, commands=[])
result['generated_source_build_adjudication'] = link_adjudication

def run(label, argv):
    log = OUT / (label + '.log')
    started = time.monotonic()
    with log.open('x') as stream:
        proc = subprocess.run(argv, stdin=subprocess.DEVNULL, stdout=stream,
                              stderr=subprocess.STDOUT, env=env)
    result['commands'].append(dict(label=label, argv=argv, exit=proc.returncode,
        elapsed_s=time.monotonic() - started, log=record(log)))
    assert proc.returncode == 0, label
    return log.read_text()

try:
    dry = run('PACKAGE_DRYRUN' + package_suffix, ['ninja', '-n', '-C', str(iso.BUILD), 'px4_package'])
    actions = [s for s in dry.splitlines() if re.match(r'^\[\d+/\d+\]', s)]
    assert len(actions) == 2, 'Expected only the two packaging rules'
    assert 'Generating ../../px4_fmu-v6c_default.bin' in actions[0]
    assert 'Creating ' in actions[1] and actions[1].endswith('/px4_fmu-v6c_default.px4')
    result['audited_dryrun_actions'] = actions
    run('APPLICATION_PACKAGE' + package_suffix, ['cmake', '--build', str(iso.BUILD), '--parallel', '2',
                                '--target', 'px4_package'])
    binary = iso.BUILD / 'px4_fmu-v6c_default.bin'
    package = iso.BUILD / 'px4_fmu-v6c_default.px4'
    descriptor = json.loads(package.read_text())
    image = zlib.decompress(base64.b64decode(descriptor['image'], validate=True))
    extracted = subprocess.run([str(compiler.with_name('arm-none-eabi-objcopy')),
        '-O', 'binary', str(elf), '/dev/stdout'], check=True, stdin=subprocess.DEVNULL,
        stdout=subprocess.PIPE, stderr=subprocess.PIPE, env=env).stdout
    commit = subprocess.check_output(['git', '--git-dir', str(iso.PX4 / '.git'),
        'rev-parse', '--verify', 'HEAD'], env=env, text=True).strip()
    checks = dict(board_id_56=descriptor['board_id'] == 56,
        board_revision_0=descriptor['board_revision'] == 0,
        px4fw_v1=descriptor['magic'] == 'PX4FWv1',
        base_commit_exact=descriptor['git_hash'] == commit,
        expected_latest_link=sha(elf) == expected,
        independent_elf_objcopy_equals_bin=extracted == binary.read_bytes(),
        package_image_equals_bin=image == binary.read_bytes(),
        image_size_exact=descriptor['image_size'] == len(image),
        flash_limit_unchanged=descriptor['image_maxsize'] == 1966080,
        image_fits=len(image) <= descriptor['image_maxsize'],
        protected_products_unchanged=all(record(Path(p)) == r for p, r in before.items()),
        original_source_anchors_unchanged=anchors == iso.stat_snapshot(focused=True),
        linked_source_identities_still_exact=all(sha(Path(p)) == h for p, h in link_sources.items()),
        parameter_json_unchanged=(iso.BUILD / 'parameters.json').read_bytes() ==
            (prior_artifacts / 'parameters.json').read_bytes())
    for field, filename in (('parameter_xml', 'parameters.xml'), ('airframe_xml', 'airframes.xml')):
        raw = zlib.decompress(base64.b64decode(descriptor[field], validate=True))
        checks[field + '_exact'] = raw == (iso.BUILD / filename).read_bytes()
        checks[field + '_unchanged'] = raw == (prior_artifacts / filename).read_bytes()
    artifacts = OUT / ('artifacts' + package_suffix)
    artifacts.mkdir(exist_ok=False)
    filenames = ('px4_fmu-v6c_default.elf', 'px4_fmu-v6c_default.map',
        'px4_fmu-v6c_default.bin', 'px4_fmu-v6c_default.px4',
        'parameters.json', 'parameters.xml', 'airframes.xml')
    for filename in filenames:
        shutil.copy2(iso.BUILD / filename, artifacts / filename)
    checks['retained_artifacts_exact'] = all(sha(artifacts / f) == sha(iso.BUILD / f) for f in filenames)
    result.update(checks=checks, artifacts=[record(artifacts / f) for f in filenames],
        build_artifacts=[record(iso.BUILD / f) for f in filenames],
        package_image_sha256=hashlib.sha256(image).hexdigest().upper(),
        board_id=descriptor['board_id'], package_git_identity=descriptor['git_identity'],
        image_bytes=len(image), flash_limit_bytes=descriptor['image_maxsize'],
        flash_remaining_bytes=descriptor['image_maxsize'] - len(image),
        memory_report=link['memory_report'], parameter_declarations=old['parameter_declarations'],
        parameter_missing=[], parameter_extra=[], parameter_changed=[],
        linked_checks=link['linked_checks'], protected_files_before=before,
        link_receipt=record(LINK_RECEIPT), prior_package_receipt=record(PRIOR / 'APPLICATION_PACKAGE.json'),
        official_packager=record(iso.PX4 / 'Tools/px_mkfw.py'),
        source_identity_boundary='Package git identity records the PX4 base revision; the source manifest records integration changes.',
        passed=all(checks.values()))
except Exception as error:
    result['first_exception'] = type(error).__name__ + ': ' + str(error)
finally:
    with RECEIPT.open('x') as stream:
        json.dump(result, stream, indent=2)
        stream.write('\n')
    print(json.dumps({k: result.get(k) for k in ('passed', 'first_exception',
        'image_bytes', 'flash_remaining_bytes', 'package_image_sha256')}), flush=True)
sys.exit(0 if result['passed'] else 1)
