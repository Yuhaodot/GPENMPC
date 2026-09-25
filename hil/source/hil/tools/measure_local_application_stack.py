#!/usr/bin/env python3
"""Selected actual-app compiler invocations with .su only; no target access.
Compiled copies must have byte-identical disassembly/relocations to the linked
objects. No substitution into the application. Unknown external frames remain
explicit; this is not board high-water or WCET measurement.
"""
from pathlib import Path
import concurrent.futures
import hashlib
import json
import os
import re
import shlex
import subprocess
import sys

build=Path(__file__).resolve().parent.parent
app=build/'rfly_vendor_integration/application_integration'
compiled=app/'build_fmuv6c'
label=sys.argv[1] if len(sys.argv)>1 else 'application_stack'
assert re.fullmatch(r'[a-z][a-z0-9_]*',label)
out=app/'local_runtime_integration'/label/'stack_inspection'
out.mkdir(exist_ok=False)
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest().upper()
identity={str(p):sha(p) for p in [compiled/'px4_fmu-v6c_default.elf',compiled/'px4_fmu-v6c_default.px4',compiled/'parameters.json']}
selected={'GPENMPCRflyCanonicalLocalModule.cpp','RegisteredCanonicalLocalModuleContext.cpp',
          'CanonicalLocalExchangeCore.cpp','CanonicalLocalExecutionCycle.cpp','Px4CanonicalLocalIo.cpp',
          'CanonicalLocalTaskInputOwner.cpp','Px4OriginalHilReceiptReader.cpp','Px4SelectedSourceReader.cpp',
          'CanonicalLocalApplicationOwner.cpp','CanonicalFullInnerAbi.cpp'}
rows=[r for r in json.loads((compiled/'compile_commands.json').read_text()) if Path(r['file']).name in selected]
assert len(rows)==len(selected)
prior_value=os.environ.get('GPENMPC_STACK_REFERENCE_RECEIPT','')
prior_path=Path(prior_value) if prior_value else None
prior=json.loads(prior_path.read_text()) if prior_path and prior_path.is_file() and prior_path.resolve()!=(out/'RESULT.json').resolve() else None
prior_rows={r['source']:r for r in prior['records']} if prior and prior['passed'] else {}
env=dict(os.environ,CCACHE_DISABLE='1',OMP_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1',MKL_NUM_THREADS='1')

def run(row):
    name=Path(row['file']).stem
    args=shlex.split(row['command'])
    if Path(args[0]).name=='ccache':args=args[1:]
    original=(Path(row['directory'])/args[args.index('-o')+1]).resolve()
    old=prior_rows.get(row['file'])
    if old and old['original_sha256']==sha(original) and old['source_sha256']==sha(Path(row['file'])):
        info=dict(old)
        info['reused_exact_object_evidence']=dict(path=str(prior_path),sha256=sha(prior_path))
        return info
    new=out/(name+'.o')
    args[args.index('-o')+1]=str(new)
    args[1:1]=['-fstack-usage']
    if '-MF' in args:args[args.index('-MF')+1]=str(out/(name+'.d'))
    result=subprocess.run(args,cwd=row['directory'],env=env,capture_output=True,text=True,timeout=180)
    (out/(name+'.compile.txt')).write_text(result.stdout+result.stderr)
    info=dict(source=row['file'],source_sha256=sha(Path(row['file'])),command=args,exit=result.returncode)
    if result.returncode:return info
    prefix=args[0].removesuffix('g++')
    def dis(path):
        lines=subprocess.run([prefix+'objdump','-drw',str(path)],capture_output=True,text=True,check=True).stdout.splitlines()
        return '\n'.join(x for x in lines if 'file format elf32-littlearm' not in x)
    actual,copy=dis(original),dis(new)
    (out/(name+'.disassembly.txt')).write_text(copy)
    frames=[]
    for line in new.with_suffix('.su').read_text().splitlines():
        m=re.match(r'^.+?:\d+:\d+:(.+)\t(\d+)\t(.+)$',line)
        assert m,line
        frames.append(dict(function=m.group(1),bytes=int(m.group(2)),kind=m.group(3)))
    info.update(original_object=str(original),original_sha256=sha(original),copy_sha256=sha(new),
                instructions_and_relocations_equal=actual==copy,frames=frames,
                maximum_single_function_frame=max((f['bytes'] for f in frames),default=0))
    return info

results=[]
with concurrent.futures.ThreadPoolExecutor(max_workers=2) as pool:
    for r in pool.map(run,rows):
        results.append(r)
        print(Path(r['source']).name,r['exit'],r.get('instructions_and_relocations_equal'),r.get('maximum_single_function_frame'),flush=True)
stable=identity=={p:sha(Path(p)) for p in identity}
report=dict(scope='ACTUAL_APP_SELECTED_STACK_FRAMES_NO_TARGET_EXECUTION',
            passed=stable and all(r['exit']==0 and r.get('instructions_and_relocations_equal') for r in results),
            actual_application_unchanged=stable,identity=identity,records=results,
            dedicated_task_stack_bytes=8192,full_transitive_bound_proven=False,board_highwater_measured=False,
            WCET_measured=False,hardware_actions=0,COM=0,
            limitations='Single frames are not cumulative bounds; original 74C/facade direct-call graph and libm/libc/virtual-call/RTOS paths still require composition.')
(out/'RESULT.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps({k:report[k] for k in ['passed','actual_application_unchanged','full_transitive_bound_proven','hardware_actions']}),flush=True)
raise SystemExit(0 if report['passed'] else 1)
