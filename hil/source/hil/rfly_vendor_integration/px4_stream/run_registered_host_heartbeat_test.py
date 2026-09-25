"""Test host heartbeat registration and prestream policy."""
from pathlib import Path
import hashlib
import json
import subprocess
import sys

root=Path(__file__).resolve().parent
build=root.parent.parent
label=sys.argv[1] if len(sys.argv)==2 else 'verification'
assert (label and all(c.isalnum() or c in '_-' for c in label))
out=root/('heartbeat_prestream_'+label)
out.mkdir(exist_ok=False)
inputs=[root/'RegisteredHostHeartbeat.hpp',root.parent/'px4_runtime/DirectOffboardPrestream.hpp',
        root/'test_registered_host_heartbeat.cpp',Path(__file__)]
def hashes():return {str(p):hashlib.sha256(p.read_bytes()).hexdigest().upper() for p in inputs}
before=hashes()
argv=['g++','-std=c++14','-O2','-Wall','-Wextra','-Werror',str(inputs[2]),'-o',str(out/'test')]
c=subprocess.run(argv,capture_output=True,text=True)
report=dict(argv=argv,compile_exit=c.returncode,compile_stderr=c.stderr,source_hashes_before=before,
            policy_only=True,actual_uorb_publish=False,actual_board_execution=False)
if c.returncode==0:
    r=subprocess.run([str(out/'test')],capture_output=True,text=True,timeout=20)
    report.update(run_exit=r.returncode,stdout=r.stdout,stderr=r.stderr,
                  binary_sha256=hashlib.sha256((out/'test').read_bytes()).hexdigest().upper())
    if r.stdout:report['result']=json.loads(r.stdout)
report['source_hashes_after']=hashes()
report['sources_unchanged']=before==report['source_hashes_after']
report['passed']=report.get('run_exit')==0 and report['sources_unchanged']
(out/'RESULT.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report,indent=2))
raise SystemExit(0 if report['passed'] else 1)
