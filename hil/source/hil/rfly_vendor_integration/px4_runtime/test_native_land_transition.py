"""Test landing transitions with simulated guard, clock and topic inputs."""

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
out=Path(sys.argv[1]).resolve()
out.mkdir(exist_ok=False)
meta=component_input('GPENMPC_TOPIC_HEADERS')
stub=Path(__file__).resolve().parents[1] / 'px4_wire/pump_host_stub'
sources=['test_native_land_output_authority.cpp','test_canonical_output_authority.cpp','test_execution_session.cpp']
dependencies=[root/n for n in sources+['NativeLandOutputAuthority.hpp','CanonicalOutputAuthority.hpp',
    'BoardSafetyEvidence.hpp','NativeLandModeShape.hpp','SessionBoundGuard.hpp','ExecutionSessionRegistration.hpp',
    'ExecutionSessionRegistration.cpp','InheritingMutex.hpp','Px4ReadOnlyGuard.cpp']]
dependencies += [root.parent/'px4_stream'/n for n in ['RflyStreamAuthority.hpp','SharedOutputRegistry.hpp',
    'LinkLifetimeRegistry.hpp','LinkLifetimeRegistry.cpp']]
hashes=lambda:{str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in dependencies}
before=hashes()
report={'scope':'HOST_TEST__ACTUAL_CLASSES__GUARD_HRT_UORB_FIXTURES',
    'records':[],'sources':before}
for name in sources:
    binary=out/Path(name).stem
    argv=['g++','-std=c++14','-O2','-pthread','-Wall','-Wextra','-Werror','-I'+str(meta),
        '-I'+str(stub),str(root/name)]
    if name=='test_execution_session.cpp':
        argv += [str(root/'ExecutionSessionRegistration.cpp'),str(root.parent/'px4_stream/LinkLifetimeRegistry.cpp')]
    argv += ['-o',str(binary)]
    c=subprocess.run(argv,capture_output=True,text=True,timeout=120)
    record={'source':name,'compile_argv':argv,'compile_exit':c.returncode,'compile_log':c.stdout+c.stderr}
    if c.returncode==0:
        r=subprocess.run([str(binary)],capture_output=True,text=True,timeout=60)
        record.update(run_exit=r.returncode,stdout=r.stdout,stderr=r.stderr,
            binary_sha256=hashlib.sha256(binary.read_bytes()).hexdigest())
    report['records'].append(record)
    print(json.dumps(record),flush=True)
report['source_stable']=before==hashes()
report['passed']=report['source_stable'] and all(r.get('run_exit')==0 for r in report['records'])
(out/'RESULT.json').write_text(json.dumps(report,indent=2)+'\n')
raise SystemExit(0 if report['passed'] else 1)
