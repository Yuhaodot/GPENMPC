"""Test context registration using POSIX headers and simulated guard/clock inputs."""

# Component inputs are supplied by the invoking build/test environment.
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from component_inputs import component_input
component_input('GPENMPC_BUILD_ROOT')
component_input('GPENMPC_TOPIC_HEADERS')

import hashlib,json,subprocess,sys
from pathlib import Path
import os
root=Path(__file__).resolve().parent;label=sys.argv[1];assert (label and all(c.isalnum() or c in '_-' for c in label))
prestream=len(sys.argv)>2 and sys.argv[2]=='prestream'
module_abort=len(sys.argv)>2 and sys.argv[2]=='module_abort'
dest=root/('host_registered_context_'+label);dest.mkdir(exist_ok=False)
rfly=root.parent;build=rfly.parent;px4=Path(os.environ['GPENMPC_PX4_ROOT'])
generated=build/'evidence/arm_controller/GPENMPC_Rfly_Canonical_Controller_ert_rtw'
compiler=['g++','-std=c++14','-O2','-ffp-contract=off','-fno-fast-math','-Wall','-Wextra','-Werror','-Wno-address-of-packed-member','-pthread',
 '-D__PX4_POSIX','-D__PX4_LINUX','-DMODULE_NAME="gpenmpc_context_host_test"','-DGPENMPC_CONTEXT_HOST_TEST_GUARD="'+str(root/'RegisteredContextHostGuard.hpp')+'"',
 '-include',str(px4/'src/include/visibility.h'),'-include','px4_platform_common/defines.h','-I'+str(root/'pump_host_stub'),'-I'+str(component_input('GPENMPC_TOPIC_HEADERS')),
 '-I'+str(generated),'-I'+str(px4/'platforms/common/include'),'-I'+str(px4/'platforms/posix/include'),'-I'+str(px4/'src'),'-I'+str(px4/'src/lib'),
 '-I'+str(component_input('GPENMPC_BUILD_ROOT')),'-isystem',str(component_input('GPENMPC_BUILD_ROOT') / 'mavlink/common'),'-isystem',str(component_input('GPENMPC_BUILD_ROOT') / 'mavlink')]
sources=[root/('test_module_acquire_abort.cpp' if module_abort else 'test_registered_context_prestream.cpp' if prestream else 'test_registered_context.cpp')]+[rfly/'px4_runtime'/n for n in ['RegisteredCanonicalModuleContext.cpp','CanonicalExchangePump.cpp','Px4CanonicalIo.cpp','ExecutionSessionRegistration.cpp']]
sources +=[rfly/'px4_stream'/n for n in ['SharedOutputRegistry.cpp','LinkLifetimeRegistry.cpp','LinkCanonicalReservation.cpp','SnapshotRouteRegistry.cpp','CommittedFeedbackRouteRegistry.cpp']]
if module_abort:
 sources +=[rfly/'px4_runtime/GPENMPCRflyCanonicalModule.cpp',px4/'platforms/common/module.cpp']
 compiler+=['-fcheck-new','-Wl,--wrap=_Znwm','-Wno-unused-parameter']
before={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in sources};records=[];objects=[];passed=True
for stem in ['GPENMPC_Rfly_Canonical_Controller','rt_nonfinite','rtGetInf']:
 obj=dest/(stem+'.o');args=['gcc','-std=c11','-O2','-ffp-contract=off','-fno-fast-math','-I'+str(generated),'-c',str(generated/(stem+'.c')),'-o',str(obj)]
 r=subprocess.run(args,capture_output=True,text=True);records.append(dict(argv=args,returncode=r.returncode,output=r.stdout+r.stderr));objects.append(obj);passed &= r.returncode==0
args=compiler+[str(p) for p in sources+objects]+['-o',str(dest/'test')]
if passed:
 r=subprocess.run(args,capture_output=True,text=True);records.append(dict(argv=args,returncode=r.returncode,output=r.stdout+r.stderr));passed &= r.returncode==0
run=None
if passed:
 fixtures=Path(os.environ['GPENMPC_FLOAT_MAPPING_FIXTURES'])
 args=[str(dest/'test'),str(fixtures/'MATLAB_ARGUMENTS_AND_EXPECTED.bin'),str(fixtures/'MATLAB_NED_AND_MAPPED_STATE.bin')]
 run=subprocess.run(args,cwd=dest,capture_output=True,text=True,timeout=45);passed &= run.returncode==0
report=dict(passed=passed,records=records,source_hashes=before,sources_unchanged=before=={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in sources},
 run_exit=run.returncode if run else None,stdout=run.stdout if run else '',stderr=run.stderr if run else '',
 scope='PRODUCTION_CONTEXT_CPP_REAL_SESSION_LEDGER_REGISTRY_POSIX_MUTEX_AND_MODULEBASE_HEADERS_MOCK_RAWGUARD_UORB_HRT',actual_modulebase_sync_start_executed=module_abort,platform_tasks_executed=0)
(dest/'RESULT.json').write_text(json.dumps(report,indent=2)+'\n')
for r in records:
 if r['returncode']:print(r['output'])
print(report['stdout']);print(report['stderr']);print('passed='+str(passed));sys.exit(0 if passed else 1)
