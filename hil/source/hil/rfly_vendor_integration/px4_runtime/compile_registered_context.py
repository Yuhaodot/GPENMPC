"""Compile the registered context with FMUv6C/NuttX flags."""

# Component inputs are supplied by the invoking build/test environment.
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from component_inputs import component_input
component_input('GPENMPC_BUILD_ROOT')
component_input('GPENMPC_NUTTX_INCLUDE')
component_input('GPENMPC_TOPIC_HEADERS')

import hashlib,json,shlex,subprocess,sys
from pathlib import Path
import os
root=Path(__file__).resolve().parent
label=sys.argv[1];assert (label and all(c.isalnum() or c in '_-' for c in label));owner=len(sys.argv)>2 and sys.argv[2]=='owner'
stem='CanonicalApplicationOwner' if owner else 'RegisteredCanonicalModuleContext'
receipt=root/(('APPLICATION_OWNER_ARM_' if owner else 'REGISTERED_CONTEXT_ARM_')+label+'.json');obj=root/(stem+'_'+label+'.o')
assert not receipt.exists() and not obj.exists()
px4=Path(os.environ['GPENMPC_PX4_ROOT'])
db=component_input('GPENMPC_BUILD_ROOT') / 'compile_commands.json'
entry,=[r for r in json.loads(db.read_text()) if r['file']==str(px4/'src/modules/mavlink/mavlink_receiver.cpp')]
args=shlex.split(entry['command']);assert '-std=gnu++14' in args and '-nostdinc++' in args
source=root/(stem+'.cpp');args[args.index('-c')+1]=str(source);args[args.index('-o')+1]=str(obj)
args[1:1]=['-I'+str(component_input('GPENMPC_TOPIC_HEADERS')),
 '-I'+str(root.parent.parent/'evidence/arm_controller/GPENMPC_Rfly_Canonical_Controller_ert_rtw'),
 '-isystem',str(component_input('GPENMPC_NUTTX_INCLUDE')),'-ffp-contract=off','-fstack-usage','-MD','-MF',str(obj.with_suffix('.d'))]
inputs=[source]+[root/n for n in ['RegisteredCanonicalModuleContext.hpp','GPENMPCRflyCanonicalModule.hpp','Px4CanonicalIo.hpp','CanonicalExchangePump.hpp',
 'CommittedFeedback.hpp','CommittedFeedbackOutbox.hpp','CanonicalOutputAuthority.hpp','NativeLandOutputAuthority.hpp','Px4ExecutionSessionGuard.hpp','SessionBoundGuard.hpp','BoardSafetyEvidence.hpp','InheritingMutex.hpp']]
inputs+=list((root.parent/'px4_stream').glob('*.hpp'))
if owner:inputs +=[root/'CanonicalApplicationOwner.hpp',root.parent/'application_parameters/CanonicalApplicationParameters.hpp']
before={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in inputs}
r=subprocess.run(args,cwd=entry['directory'],capture_output=True,text=True);undefined=''
if r.returncode==0:undefined=subprocess.check_output([args[0].replace('g++','nm'),'-u','-C',str(obj)],text=True)
report=dict(command_argv=args,working_directory=entry['directory'],compiler_exit_code=r.returncode,compiler_output=r.stdout+r.stderr,
 undefined_symbols=undefined,source_hashes=before,sources_unchanged=before=={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in inputs},
 stack_usage=obj.with_suffix('.su').read_text() if obj.with_suffix('.su').exists() else '',
 object_sha256=hashlib.sha256(obj.read_bytes()).hexdigest() if obj.exists() else None,
 scope='ACTUAL_PRODUCTION_GUARD_IO_PUMP_REGISTRIES_AND_REAL_MODULECONTEXT_ARM_COMPILE_ONLY',host_test_seam=False)
receipt.write_text(json.dumps(report,indent=2)+'\n');print(r.stdout+r.stderr);print(undefined)
print('\n'.join(line for line in report['stack_usage'].splitlines() if 'RegisteredCanonicalModuleContext' in line));sys.exit(r.returncode)
