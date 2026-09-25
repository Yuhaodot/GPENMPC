#!/usr/bin/env python3
"""Test the output registry with pthreads and a simulated owner."""

# Component inputs are supplied by the invoking build/test environment.
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from component_inputs import component_input
component_input('GPENMPC_TOPIC_HEADERS')

from pathlib import Path
import hashlib
import json
import subprocess
import sys

ROOT=Path(__file__).resolve().parent
label=sys.argv[1] if len(sys.argv)==2 else 'verification'
assert (label and all(c.isalnum() or c in '_-' for c in label))
OUT=ROOT/('registry_host_test_'+label)
OUT.mkdir(exist_ok=False)
META=component_input('GPENMPC_TOPIC_HEADERS')
DECLARATIONS=Path(__file__).resolve().parents[1] / 'px4_wire/pump_host_stub'
inputs=[ROOT/'test_shared_output_registry.cpp',ROOT/'SharedOutputRegistry.hpp',ROOT/'SharedOutputRegistry.cpp',
        ROOT/'LinkLifetimeRegistry.hpp',ROOT/'LinkLifetimeRegistry.cpp',
        ROOT/'RflyStreamAuthority.hpp',ROOT/'GPENMPCRflyHilStream.hpp',ROOT.parent/'px4_runtime/InheritingMutex.hpp',Path(__file__)]
def hashes():
    return {str(path):hashlib.sha256(path.read_bytes()).hexdigest() for path in inputs}
before=hashes()
args=['g++','-std=c++14','-O2','-Wall','-Wextra','-Werror','-pthread',
      '-I'+str(META),'-I'+str(DECLARATIONS),str(inputs[0]),str(ROOT/'SharedOutputRegistry.cpp'),
      str(ROOT/'LinkLifetimeRegistry.cpp'),
      '-Wl,--wrap=pthread_mutex_init','-Wl,--wrap=pthread_mutex_unlock','-o',str(OUT/'test')]
compiled=subprocess.run(args,capture_output=True,text=True)
report={'passed':False,'argv':args,'compile_exit':compiled.returncode,'compile_stdout':compiled.stdout,
        'compile_stderr':compiled.stderr,'source_hashes_before':before,'host_uorb_declaration_stub_only':True,
        'actual_nuttx_execution':False,'target_factory_execution':False,
        'fault_injection_note':'Only two explicitly labelled API-error cases alter pthread return behavior; concurrency, PI/robust attributes, waiting and callback owner-death use actual pthread/kernel operations.'}
if compiled.returncode==0:
    try:
        run=subprocess.run([str(OUT/'test')],capture_output=True,text=True,timeout=30)
        report.update(run_exit=run.returncode,stdout=run.stdout,stderr=run.stderr,
                      binary_sha256=hashlib.sha256((OUT/'test').read_bytes()).hexdigest())
        if run.stdout:report['result']=json.loads(run.stdout)
    except subprocess.TimeoutExpired as exc:
        report['failure']='Host test watchdog expired'
        report['stdout']=str(exc.stdout);report['stderr']=str(exc.stderr)
after=hashes()
report.update(source_hashes_after=after,sources_unchanged=before==after)
report['passed']=report.get('run_exit')==0 and report['sources_unchanged']
(OUT/'RESULT.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps({key:report.get(key) for key in ('passed','compile_exit','compile_stderr','run_exit','result','failure','stderr')},indent=2))
raise SystemExit(0 if report['passed'] else 1)
