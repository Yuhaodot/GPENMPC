#!/usr/bin/env python3
"""Test link borrowing and release using host pthreads."""
from pathlib import Path
import hashlib
import json
import subprocess
import sys
ROOT=Path(__file__).resolve().parent
label=sys.argv[1] if len(sys.argv)==2 else 'verification'
assert (label and all(c.isalnum() or c in '_-' for c in label))
OUT=ROOT/('link_lifetime_host_'+label)
OUT.mkdir(exist_ok=False)
inputs=[ROOT/'test_link_lifetime_registry.cpp',ROOT/'LinkLifetimeRegistry.hpp',ROOT/'LinkLifetimeRegistry.cpp',
        ROOT.parent/'px4_runtime/InheritingMutex.hpp',Path(__file__)]
def hashes():return {str(path):hashlib.sha256(path.read_bytes()).hexdigest() for path in inputs}
before=hashes()
args=['g++','-std=c++14','-O2','-Wall','-Wextra','-Werror','-pthread',str(inputs[0]),str(inputs[2]),
      '-Wl,--wrap=pthread_mutex_init','-Wl,--wrap=pthread_mutex_unlock','-o',str(OUT/'test')]
compiled=subprocess.run(args,capture_output=True,text=True)
report={'passed':False,'argv':args,'compile_exit':compiled.returncode,'compile_stderr':compiled.stderr,
        'source_hashes_before':before,'actual_nuttx_execution':False,
        'fault_injections':'Two labelled initialization/post-unlock API error cases; all other tests use actual pthread synchronization and Linux robust thread death.'}
if compiled.returncode==0:
    try:
        run=subprocess.run([str(OUT/'test')],capture_output=True,text=True,timeout=30)
        report.update(run_exit=run.returncode,stdout=run.stdout,stderr=run.stderr,
                      binary_sha256=hashlib.sha256((OUT/'test').read_bytes()).hexdigest())
        if run.stdout:report['result']=json.loads(run.stdout)
    except subprocess.TimeoutExpired:
        report['failure']='Host test watchdog expired'
report['source_hashes_after']=hashes();report['sources_unchanged']=before==report['source_hashes_after']
report['passed']=report.get('run_exit')==0 and report['sources_unchanged']
(OUT/'RESULT.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps({key:report.get(key) for key in ('passed','compile_stderr','run_exit','result','failure','stderr')},indent=2))
raise SystemExit(0 if report['passed'] else 1)
