#!/usr/bin/env python3
"""Test session registration and compile its NuttX/Cortex-M7 interface."""

# Component inputs are supplied by the invoking build/test environment.
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from component_inputs import component_input
component_input('GPENMPC_BUILD_ROOT')
component_input('GPENMPC_NUTTX_INCLUDE')
component_input('GPENMPC_TOPIC_HEADERS')

from pathlib import Path
import os
import hashlib
import json
import shlex
import subprocess
import sys

ROOT=Path(__file__).resolve().parent
BUILD=ROOT.parent.parent
PX4=Path(os.environ['GPENMPC_PX4_ROOT'])
META=component_input('GPENMPC_TOPIC_HEADERS')
DB=component_input('GPENMPC_BUILD_ROOT') / 'compile_commands.json'
label=sys.argv[1] if len(sys.argv)==2 else 'verification'
assert (label and all(c.isalnum() or c in '_-' for c in label))
OUT=ROOT/('execution_session_'+label)
OUT.mkdir(exist_ok=False)
inputs=[ROOT/name for name in ['ExecutionSessionRegistration.hpp','ExecutionSessionRegistration.cpp',
    'SessionBoundGuard.hpp','Px4ExecutionSessionGuard.hpp','Px4ExecutionSessionGuard.cpp',
    'test_execution_session.cpp','probe_execution_session_authority.cpp','verify_execution_session.py',
    'Px4ReadOnlyGuard.hpp','Px4ReadOnlyGuard.cpp','BoardSafetyEvidence.hpp','InheritingMutex.hpp','CanonicalOutputAuthority.hpp']]
inputs += [ROOT.parent/'px4_stream/LinkLifetimeRegistry.hpp',ROOT.parent/'px4_stream/LinkLifetimeRegistry.cpp',DB]
def hashes():return {str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in inputs}
before=hashes()
report={'passed':False,'scope':'HOST_SESSION_FIXTURE_AND_REAL_NUTTX_OBJECTS_ONLY','source_hashes_before':before,
        'firmware_linked':False,'target_code_executed':False,'old_wire_modified':False,'records':[]}
try:
    args=['g++','-std=c++14','-O2','-Wall','-Wextra','-Werror','-pthread',str(ROOT/'test_execution_session.cpp'),
        str(ROOT/'ExecutionSessionRegistration.cpp'),str(ROOT.parent/'px4_stream/LinkLifetimeRegistry.cpp'),'-o',str(OUT/'host_test')]
    compiled=subprocess.run(args,capture_output=True,text=True)
    (OUT/'host_compile.log').write_text(compiled.stdout+compiled.stderr)
    report['host_compile']={'argv':args,'exit_code':compiled.returncode,'uorb_stubs':False}
    assert compiled.returncode==0,compiled.stderr
    run=subprocess.run([str(OUT/'host_test')],capture_output=True,text=True,timeout=30)
    (OUT/'host_run.log').write_text(run.stdout+run.stderr)
    report['host_run']={'exit_code':run.returncode,'stdout':run.stdout,'stderr':run.stderr}
    assert run.returncode==0,run.stdout+run.stderr
    report['host_result']=json.loads(run.stdout)
    rows=json.loads(DB.read_text())
    entry,=[r for r in rows if r['file']==str(PX4/'src/modules/mavlink/mavlink_messages.cpp')]
    for name in ['ExecutionSessionRegistration.cpp','Px4ExecutionSessionGuard.cpp','probe_execution_session_authority.cpp']:
        local=ROOT/name;obj=OUT/(name+'.obj');dep=OUT/(name+'.d')
        argv=shlex.split(entry['command'])
        argv[argv.index('-c')+1]=str(local);argv[argv.index('-o')+1]=str(obj)
        argv[1:1]=['-I'+str(ROOT),'-I'+str(META),'-I'+str(ROOT.parent/'official_io_nuttx/overlay'),
            '-I'+str(PX4/'src/modules/mavlink'),'-isystem',str(component_input('GPENMPC_NUTTX_INCLUDE')),
            '-MD','-MF',str(dep),'-fstack-usage']
        result=subprocess.run(argv,cwd=entry['directory'],capture_output=True,text=True)
        (OUT/(name+'.log')).write_text(result.stdout+result.stderr)
        record={'source':str(local),'argv':argv,'cwd':entry['directory'],'exit_code':result.returncode}
        report['records'].append(record);print(name,result.returncode,flush=True)
        assert result.returncode==0,result.stderr
        record['object_sha256']=hashlib.sha256(obj.read_bytes()).hexdigest()
        for tool,flags in [('nm',['-C']),('readelf',['-h','-A'])]:
            output=subprocess.run([argv[0].replace('g++',tool),*flags,str(obj)],capture_output=True,text=True,check=True).stdout
            (OUT/(name+'.'+tool)).write_text(output)
            if tool=='nm':assert not any('__atomic_' in line and ' U ' in line for line in output.splitlines())
        dependencies=dep.read_text()
        assert str(component_input('GPENMPC_BUILD_ROOT') / 'uORB/topics') not in dependencies
        if name!='ExecutionSessionRegistration.cpp':assert str(META/'uORB/topics/uORBTopics.hpp') in dependencies
    after=hashes();report.update(source_hashes_after=after,sources_unchanged=before==after,passed=before==after)
    report['test_inputs']='Synthetic UID, clock, safety and operator-declaration fixtures.'
except Exception as exc:
    report['failure']=str(exc)
finally:
    (OUT/'RESULT.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({k:report.get(k) for k in ['passed','host_result','failure','sources_unchanged']},indent=2))
raise SystemExit(0 if report['passed'] else 1)
