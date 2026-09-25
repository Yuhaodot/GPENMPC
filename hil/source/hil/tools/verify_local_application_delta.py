"""Record the source delta and package identity."""
from pathlib import Path
import argparse
import hashlib
import json
import re

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--run', type=Path, required=True)
parser.add_argument('--prior', type=Path, required=True)
parser.add_argument('--expected-source-sha', required=True)
args = parser.parse_args()
run, prior = args.run.resolve(), args.prior.resolve()
root = Path(__file__).resolve().parents[1]
source = root / 'rfly_vendor_integration/px4_runtime/CanonicalLocalSessionEntry.cpp'
compiled = root / 'rfly_vendor_integration/application_integration/build_fmuv6c'
receipt_path = run / 'SOURCE_DELTA_AND_IDENTITY.json'
assert not receipt_path.exists()
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest().upper()
record = lambda p: dict(path=str(p), bytes=p.stat().st_size, sha256=sha(p))
current = json.loads((run / 'RESULT.json').read_text())
previous = json.loads((prior / 'RESULT.json').read_text())
package = json.loads((run / 'APPLICATION_PACKAGE.json').read_text())
old_package = json.loads((prior / 'APPLICATION_PACKAGE.json').read_text())
assert current['passed'] and current['sources_stable'] and current['original_source_anchors_unchanged']
assert package['passed'] and all(package['checks'].values())
old, new = previous['sources_before'], current['sources_before']
assert set(old) == set(new)
changed = [dict(path=p, before=old[p], after=new[p]) for p in old if old[p] != new[p]]
assert len(changed) == 1 and changed[0]['path'] == str(source)
assert changed[0]['after'] == args.expected_source_sha.upper() == sha(source)
assert all(sha(Path(p)) == h for p, h in new.items())
for row in old_package['artifacts']:
    path = Path(row['path'])
    assert path.stat().st_size == row['bytes'] and sha(path) == row['sha256']
for row in package['artifacts']:
    path = Path(row['path'])
    assert path.stat().st_size == row['bytes'] and sha(path) == row['sha256']

# ver.cpp uses its own __DATE__/__TIME__, not the wall clock of a partial link.
ver_object = compiled / 'src/systemcmds/ver/CMakeFiles/systemcmds__ver.dir/ver.cpp.obj'
raw = ver_object.read_bytes()
dates = set(m.decode() for m in re.findall(rb'(?:Jan|Feb|Mar|Apr|May|Jun|Jul|Aug|Sep|Oct|Nov|Dec) [ 0-9][0-9] [0-9]{4}(?=\x00)', raw))
times = set(m.decode() for m in re.findall(rb'[0-9]{2}:[0-9]{2}:[0-9]{2}(?=\x00)', raw))
assert len(dates) == len(times) == 1
date, time = next(iter(dates)), next(iter(times))
image_path = run / 'artifacts/px4_fmu-v6c_default.bin'
image = image_path.read_bytes()
assert date.encode()+b'\x00' in image and time.encode()+b'\x00' in image
assert 'src/systemcmds/ver/libsystemcmds__ver.a(ver.cpp.obj)' in (run / 'artifacts/px4_fmu-v6c_default.map').read_text()
package_row = next(r for r in package['artifacts'] if r['path'].endswith('.px4'))
result = dict(passed=True, scope='OFFLINE_SOURCE_DELTA_AND_PACKAGE_IDENTITY',
    total_tracked_sources=len(new), unchanged_sources=len(new)-len(changed), approved_changes=changed,
    old_sources_receipt=record(prior / 'RESULT.json'), new_sources_receipt=record(run / 'RESULT.json'),
    package_receipt=record(run / 'APPLICATION_PACKAGE.json'), prior_artifacts_unchanged=True,
    source_identities_still_exact=True, package_path=package_row['path'],
    package_sha256=package_row['sha256'], package_bytes=package_row['bytes'],
    image_bytes=package['image_bytes'], image_sha256=package['package_image_sha256'],
    flash_limit_bytes=package['flash_limit_bytes'], flash_remaining_bytes=package['flash_remaining_bytes'],
    linked_ver_build_datetime=date+' '+time, linked_ver_object=record(ver_object),
    parameters_unchanged=package['checks']['parameter_json_unchanged'] and package['checks']['parameter_xml_unchanged'],
    airframes_unchanged=package['checks']['airframe_xml_unchanged'],
    metadata_scope='Build datetime from the linked ver.cpp object, checked against the package binary.',
    hardware_actions=0, COM=0, firmware_uploads=0, entry_bindings_changed=False)
with receipt_path.open('x') as stream:
    json.dump(result, stream, indent=2)
    stream.write('\n')
print(json.dumps(result))
