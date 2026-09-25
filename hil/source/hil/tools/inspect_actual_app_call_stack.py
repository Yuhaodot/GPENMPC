#!/usr/bin/env python3
"""Read actual ELF DWARF CFA + linked calls; no compile, simulator or target.

Known direct-call sums are conservative across a function's branches, but NOT
a full bound when indirect calls, recursion or missing CFA remain. Retain
those exclusions instead of treating them as zero-stack proven callees.
"""
import hashlib
import json
from pathlib import Path
from gpenmpc_external_path import external_path
import re
import subprocess

build = Path(__file__).resolve().parent.parent
app = build / 'rfly_vendor_integration/application_integration'
elf = app / 'build_fmuv6c/px4_fmu-v6c_default.elf'
inspection = Path(external_path('application_stack_inspection_receipt'))
out = inspection.with_name('LINKED_CALL_STACK_COMPLETE_GRAPH.json')
assert not out.exists(), 'Choose an unused output path.'
old = json.loads(inspection.read_text())
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest().upper()
assert old['passed'] and old['identity'][str(elf)] == sha(elf)
prefix = old['records'][0]['command'][0].removesuffix('g++')
commands = [[prefix + 'readelf', '--debug-dump=frames-interp', str(elf)],
            [prefix + 'objdump', '-d', '-C', '--no-show-raw-insn', str(elf)]]
texts = [subprocess.run(c, capture_output=True, text=True, check=True, timeout=120).stdout for c in commands]
frames = {}; current = None
for line in texts[0].splitlines():
    m = re.search(r' FDE .* pc=([0-9a-f]+)\.\.([0-9a-f]+)', line)
    if m:
        address = int(m[1], 16)
        current = dict(bytes=0, unknown_cfa=[], end=int(m[2], 16))
        if address: frames[address] = current
        else: current = None
    elif ' CIE ' in line:
        current = None
    elif current is not None:
        row = re.match(r'^([0-9a-f]{8})\s+(\S+)', line)
        if row:
            cfa = re.fullmatch(r'r13\+(\d+)', row[2])
            if cfa: current['bytes'] = max(current['bytes'], int(cfa[1]))
            else: current['unknown_cfa'].append(line.strip())

functions = {}; current = None
for line in texts[1].splitlines():
    m = re.match(r'^([0-9a-f]+) <(.*)>:$', line)
    if m:
        address = int(m[1], 16)
        current = dict(name=m[2], address=address, direct=set(), indirect=[], branches=[], unknown_targets=[])
        functions[address] = current
        continue
    if current is None: continue
    m = re.match(r'^\s*([0-9a-f]+):\s+(\S+)\s+(.*)', line)
    if not m: continue
    op, operands = m[2], m[3]
    if op.startswith(('bl', 'b.', 'beq', 'bne', 'bhi', 'bls', 'bge', 'bgt', 'ble', 'bcc', 'bcs')) or op == 'b':
        target = re.match(r'([0-9a-f]+) <', operands)
        if target:
            callee = int(target[1], 16)
            current['branches'].append((callee, op, line.strip()))
        elif op.startswith('blx'):
            current['indirect'].append(line.strip())
    elif op == 'bx' and operands.strip() not in ('lr', 'r14'):
        current['indirect'].append(line.strip())
starts = sorted(functions)
for index, address in enumerate(starts):
    f = functions[address]
    end = starts[index+1] if index+1 < len(starts) else frames.get(address, {}).get('end', address+1)
    for target, op, line in f['branches']:
        is_call = op in ('bl', 'bl.w', 'blx', 'blx.w')
        if address <= target < end and not is_call:
            continue  # Ordinary loop or basic block.
        if target in functions:
            f['direct'].add(target)  # includes genuine self BL recursion
        else:
            f['unknown_targets'].append(line)  # Retain unresolved targets.

def analyse(start):
    memo = {}; cycles = set(); unknown = set(); indirect = set(); unresolved = set()
    def visit(address, ancestors):
        if address in ancestors:
            cycles.add(tuple(functions[x]['name'] for x in (*ancestors, address)))
            return 0, []
        if address in memo: return memo[address]
        f = functions[address]; frame = frames.get(address)
        if frame is None or frame['unknown_cfa']: unknown.add(address)
        if f['indirect']: indirect.add(address)
        if f['unknown_targets']: unresolved.add(address)
        size = frame['bytes'] if frame else 0
        best = (0, [])
        for child in sorted(f['direct']):
            result = visit(child, (*ancestors, address))
            if result[0] > best[0]: best = result
        result = size + best[0], [dict(function=f['name'], address=hex(address), frame_bytes=size)] + best[1]
        memo[address] = result
        return result
    total, path = visit(start, ())
    return dict(root=functions[start]['name'], conservative_known_direct_sum_bytes=total, path=path,
                unknown_cfa=[dict(name=functions[x]['name'], frame=frames.get(x)) for x in sorted(unknown)],
                indirect_calls=[dict(name=functions[x]['name'], instructions=functions[x]['indirect']) for x in sorted(indirect)],
                unresolved_direct_targets=[dict(name=functions[x]['name'], instructions=functions[x]['unknown_targets']) for x in sorted(unresolved)],
                recursion_cycles=[list(x) for x in sorted(cycles)],
                full_path_bound_proven=not (cycles or unknown or indirect or unresolved))

names = ['canonicalLocalInnerFixedAbi', 'canonicalLocalInnerPreControl', 'canonicalLocalInnerPostControl',
         'gpenmpc_rfly_px4::CanonicalLocalExchangeCore::poll(bool)',
         'gpenmpc_rfly_px4::CanonicalLocalExecutionCycle::execute(',
         'gpenmpc_rfly_local_px4::GPENMPCRflyCanonicalLocalModule::run()']
selected = [address for address, f in functions.items() if any(f['name'] == n or (n.endswith('(') and f['name'].startswith(n)) for n in names)]
assert len(selected) >= 4, [functions[x]['name'] for x in selected]
roots = [analyse(x) for x in selected]
stable = old['identity'][str(elf)] == sha(elf)
report = dict(scope='ACTUAL_LINKED_ELF_CFA_AND_DIRECT_CALL_GRAPH_NO_TARGET', elf_sha256=sha(elf),
              inspection_sha256=sha(inspection), script_sha256=sha(Path(__file__)), commands=commands,
              actual_application_unchanged=stable, linked_function_count=len(functions), cfa_record_count=len(frames),
              supersedes_analysis_only=str(inspection.with_name('LINKED_CALL_STACK.json')),
              analysis_correction='Retain genuine self BL calls and unresolved direct targets; distinguish ordinary intra-function loops.',
              roots=roots, dedicated_task_stack_bytes=8192, full_task_bound_proven=False,
              WCET_measured=False, board_highwater_measured=False, hardware_actions=0,
              limitations='Direct-call sums include per-function maxima and treat tail branches conservatively. Missing CFA, indirect calls and recursion remain explicit. RTOS/interrupt headroom and virtual targets are not assumed zero.')
out.write_text(json.dumps(report, indent=2)+'\n')
assert stable
print(json.dumps(dict(application_unchanged=stable, roots=[dict(name=r['root'], known_bytes=r['conservative_known_direct_sum_bytes'],
      unknown=len(r['unknown_cfa']), indirect=len(r['indirect_calls']), cycles=len(r['recursion_cycles'])) for r in roots], hardware_actions=0)), flush=True)
