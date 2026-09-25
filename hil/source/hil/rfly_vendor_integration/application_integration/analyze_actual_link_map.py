#!/usr/bin/env python3
"""Compare linked section sizes and compile flags for two builds."""
import collections
import hashlib
import json
import os
import re
import shlex
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from component_inputs import component_input
component_input('GPENMPC_BUILD_ROOT')

ROOT = Path(__file__).resolve().parent
OLD = Path(os.environ['GPENMPC_BASELINE_LINK_MAP'])
NEW = Path(os.environ['GPENMPC_LINK_MAP'])


def read_map(path):
    raw = path.read_bytes()
    lines = raw.decode(errors='replace').split('Linker script and memory map', 1)[1].splitlines()
    records, outputs = [], {}
    pending = None
    for line in lines:
        out = re.match(r'^(\.[^\s]+)\s+(0x[\da-fA-F]+)\s+(0x[\da-fA-F]+)(.*)', line)
        if out:
            outputs[out[1]] = {'address': int(out[2], 16), 'bytes': int(out[3], 16), 'suffix': out[4]}
        section = re.match(r'^ (\.[^\s]+)(.*)', line)
        if section:
            pending = section[1]
            remainder = section[2]
        else:
            remainder = line
        entry = re.match(r'^\s*(0x[\da-fA-F]+)\s+(0x[\da-fA-F]+)\s+(.+\.(?:a\(.+\)|obj|o))\s*$', remainder)
        if pending and entry:
            address, size, owner = int(entry[1], 16), int(entry[2], 16), entry[3]
            if size:
                records.append({'section': pending, 'address': address, 'bytes': size,
                                'owner': owner, 'flash': 0x08000000 <= address < 0x10000000})
            pending = None
    libraries = collections.Counter()
    for record in records:
        if record['flash']:
            owner = record['owner'].split('(')[0]
            if '/arm-none-eabi/' in owner:
                owner = 'ARM_TOOLCHAIN/' + owner.split('/arm-none-eabi/', 1)[-1]
            libraries[owner] += record['bytes']
    return {'path': str(path), 'sha256': hashlib.sha256(raw).hexdigest().upper(),
            'outputs': outputs, 'flash_input_bytes': sum(libraries.values()),
            'libraries': dict(libraries), 'records': records}


old, new = read_map(OLD), read_map(NEW)
keys = set(old['libraries']) | set(new['libraries'])
deltas = sorted(({'library': key, 'old_bytes': old['libraries'].get(key, 0),
                  'new_bytes': new['libraries'].get(key, 0),
                  'delta': new['libraries'].get(key, 0)-old['libraries'].get(key, 0)}
                 for key in keys), key=lambda item: item['delta'], reverse=True)
new_items = [record for record in new['records'] if record['flash'] and
             ('external_modules/' in record['owner'] or 'modules__mavlink' in record['owner'])]
object_bytes = collections.Counter()
for record in new_items:
    object_bytes[record['owner']] += record['bytes']
database = json.loads((component_input('GPENMPC_BUILD_ROOT') / 'compile_commands.json').read_text())
optimizations = collections.Counter()
non_size = []
for row in database:
    tokens = shlex.split(row['command'])
    flags = [token for token in tokens if re.fullmatch(r'-O(?:[0123sgz]|fast)', token)]
    effective = flags[-1] if flags else 'none'
    optimizations[effective] += 1
    if effective != '-Os':
        non_size.append({'file': row['file'], 'optimization_flags': flags})
template_count = collections.Counter(record['section'] for record in new['records']
    if record['flash'] and record['section'].startswith('.text._Z'))
repeated_sections = {key: count for key, count in template_count.items() if count > 1}
clones = collections.defaultdict(list)
for record in new['records']:
    if record['flash'] and record['section'] in repeated_sections:
        clones[record['section']].append(record)
clone_excess = sum(sum(item['bytes'] for item in items) - max(item['bytes'] for item in items)
                   for items in clones.values())
result = {'scope': 'LINK_MAP_COMPARISON',
          'maps': [{key: item[key] for key in ('path', 'sha256', 'outputs', 'flash_input_bytes')}
                   for item in (old, new)],
          'library_deltas': deltas, 'new_component_object_flash_bytes': dict(object_bytes),
          'largest_new_component_sections': sorted(new_items, key=lambda item: item['bytes'], reverse=True)[:45],
          'effective_optimization_counts': dict(optimizations),
          'non_Os_compile_rows': non_size,
          'repeated_surviving_cpp_text_section_names': repeated_sections,
          'same_spelling_local_clone_gross_excess_bytes': clone_excess,
          'old_flash_load_bytes': sum(old['outputs'][s]['bytes'] for s in ('.text', '.init_section', '.ARM.exidx', '.data')),
          'new_flash_load_bytes': sum(new['outputs'][s]['bytes'] for s in ('.text', '.init_section', '.ARM.exidx', '.data')),
          'notes': ['Input attribution can include merged-string overlaps and excludes linker padding; output-section sizes are authoritative.',
                    'The earlier map supplies the comparison baseline.']}
(ROOT / 'ACTUAL_LINK_MAP_COMPARISON.json').write_text(json.dumps(result, indent=2) + '\n')
print(json.dumps({'maps': result['maps'], 'top_library_deltas': deltas[:12],
                  'component_objects': dict(object_bytes),
                  'effective_optimization_counts': dict(optimizations),
                  'non_Os_file_count': len(non_size), 'repeated_cpp_section_count': len(repeated_sections),
                  'same_spelling_clone_bytes': clone_excess}, indent=2))
