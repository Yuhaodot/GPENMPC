#!/usr/bin/env python3
"""Compile and run host tests for output ownership."""

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

root=Path(__file__).resolve().parent
attempt=sys.argv[1]
assert attempt and all(c.isalnum() or c in '_-' for c in attempt)
dest=root/('output_authority_test_'+attempt)
dest.mkdir(exist_ok=False)
build=root.parent.parent
meta=component_input('GPENMPC_TOPIC_HEADERS')
stub=Path(__file__).resolve().parents[1] / 'px4_wire/pump_host_stub'
inputs=[root/'test_canonical_output_authority.cpp',root/'CanonicalOutputAuthority.hpp',
        root/'InheritingMutex.hpp',
        root/'ExecutionAuthority.hpp',root/'BoardSafetyEvidence.hpp',
        root.parent/'px4_stream/RflyStreamAuthority.hpp']
before={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in inputs}
args=['g++','-std=c++14','-O2','-Wall','-Wextra','-Werror',
      '-pthread','-I'+str(meta),'-I'+str(stub),str(inputs[0]),'-o',str(dest/'test')]
compile_result=subprocess.run(args,capture_output=True,text=True)
report={'argv':args,'compile_exit':compile_result.returncode,
        'compile_stdout':compile_result.stdout,'compile_stderr':compile_result.stderr,'sources':before,
        'host_uorb_declaration_stub_only':True,'actual_nuttx_execution':False}
if not compile_result.returncode:
    run=subprocess.run([str(dest/'test')],capture_output=True,text=True,timeout=30)
    report.update(run_exit=run.returncode,stdout=run.stdout,stderr=run.stderr,
                  binary_sha256=hashlib.sha256((dest/'test').read_bytes()).hexdigest())
    if run.stdout:report['result']=json.loads(run.stdout)
report['sources_unchanged']=before=={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in inputs}
report['passed']=report.get('run_exit')==0 and report['sources_unchanged']
(dest/'RESULT.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report,indent=2))
raise SystemExit(0 if report['passed'] else 1)
