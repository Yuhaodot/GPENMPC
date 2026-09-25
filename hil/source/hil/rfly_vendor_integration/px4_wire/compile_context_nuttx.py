
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
label=sys.argv[1] if len(sys.argv)>1 else 'verification'
assert (label and all(c.isalnum() or c in '_-' for c in label))
obj=root/('context_nuttx_'+label+'.o');receipt=obj.with_suffix('.json')
assert not obj.exists() and not receipt.exists()
px4=Path(os.environ['GPENMPC_PX4_ROOT'])
db=component_input('GPENMPC_BUILD_ROOT') / 'compile_commands.json'
entry,=[x for x in json.loads(db.read_text()) if x['file']==str(px4/'src/modules/mavlink/mavlink_receiver.cpp')]
args=shlex.split(entry['command']);assert '-std=gnu++14' in args and '-nostdinc++' in args
args[args.index('-o')+1]=str(obj);args[args.index('-c')+1]=str(root/'probe_context_nuttx.cpp')
args[1:1]=['-I'+str(component_input('GPENMPC_TOPIC_HEADERS')),
 '-I'+str(root.parent.parent/'evidence/arm_controller/GPENMPC_Rfly_Canonical_Controller_ert_rtw'),
 '-isystem',str(component_input('GPENMPC_NUTTX_INCLUDE')),'-ffp-contract=off','-fstack-usage','-MD','-MF',str(obj.with_suffix('.d'))]
inputs=list(root.glob('*.hpp'))+[root/'probe_context_nuttx.cpp',root/'compile_context_nuttx.py']
before={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in inputs}
r=subprocess.run(args,cwd=entry['directory'],capture_output=True,text=True)
undefined='';sizes={}
if r.returncode==0:
 nm=args[0].replace('g++','nm')
 undefined=subprocess.check_output([nm,'-u',str(obj)],text=True)
 for line in subprocess.check_output([nm,'-S','--size-sort',str(obj)],text=True).splitlines():
  a=line.split()
  if len(a)==4 and a[3].startswith('gpenmpc_context_size_'):sizes[a[3]]=int(a[1],16)
record=dict(command_argv=args,working_directory=entry['directory'],compiler_exit_code=r.returncode,
 compiler_output=r.stdout+r.stderr,undefined_symbols=undefined,target_sizes_bytes=sizes,
 stack_usage=obj.with_suffix('.su').read_text() if obj.with_suffix('.su').exists() else '',source_hashes=before,
 source_unchanged=before=={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in inputs},
 object_sha256=hashlib.sha256(obj.read_bytes()).hexdigest() if obj.exists() else None,
 scope='TRUE_FMUV6C_GNU14_NUTTX_TU_COMPILE_ONLY_NOT_LIVE_CLOCK_BINDING')
receipt.write_text(json.dumps(record,indent=2)+'\n');print(json.dumps(record,indent=2));sys.exit(r.returncode)
