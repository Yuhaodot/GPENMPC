
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
feedback=len(sys.argv)>2 and sys.argv[2]=='feedback'
receipt=root/(('FEEDBACK' if feedback else 'SNAPSHOT')+'_OUTLET_NUTTX_'+label+'.json');assert not receipt.exists()
px4=Path(os.environ['GPENMPC_PX4_ROOT'])
db=component_input('GPENMPC_BUILD_ROOT') / 'compile_commands.json'
entry,=[x for x in json.loads(db.read_text()) if x['file']==str(px4/'src/modules/mavlink/mavlink_receiver.cpp')]
records=[]
for stem in (['CommittedFeedbackRouteRegistry','compile_committed_feedback_stream_entry'] if feedback else ['SnapshotRouteRegistry','compile_snapshot_stream_entry']):
 source=root.parent/'px4_stream'/(stem+'.cpp');obj=root/(stem+'_nuttx_'+label+'.o');assert not obj.exists()
 args=shlex.split(entry['command']);assert '-std=gnu++14' in args and '-nostdinc++' in args
 args[args.index('-o')+1]=str(obj);args[args.index('-c')+1]=str(source)
 args[1:1]=['-I'+str(component_input('GPENMPC_TOPIC_HEADERS')),
  '-I'+str(root.parent.parent/'evidence/arm_controller/GPENMPC_Rfly_Canonical_Controller_ert_rtw'),
  '-I'+str(root.parent/'px4_stream'),'-I'+str(px4/'src/modules/mavlink'),
  '-isystem',str(component_input('GPENMPC_NUTTX_INCLUDE')),'-ffp-contract=off','-fstack-usage','-MD','-MF',str(obj.with_suffix('.d'))]
 before=hashlib.sha256(source.read_bytes()).hexdigest();r=subprocess.run(args,cwd=entry['directory'],capture_output=True,text=True)
 undefined=''
 if r.returncode==0:undefined=subprocess.check_output([args[0].replace('g++','nm'),'-u',str(obj)],text=True)
 records.append(dict(source=str(source),source_sha256=before,source_unchanged=before==hashlib.sha256(source.read_bytes()).hexdigest(),
  command_argv=args,compiler_exit_code=r.returncode,compiler_output=r.stdout+r.stderr,undefined_symbols=undefined,
  stack_usage=obj.with_suffix('.su').read_text() if obj.with_suffix('.su').exists() else '',
  object_sha256=hashlib.sha256(obj.read_bytes()).hexdigest() if obj.exists() else None))
 print(stem+' exit='+str(r.returncode));print(r.stdout+r.stderr)
receipt.write_text(json.dumps(dict(scope='TRUE_FMUV6C_NUTTX_REGISTRY_AND_REAL_MAVLINK_FACTORY_COMPILE_ONLY',feedback=feedback,records=records),indent=2)+'\n')
sys.exit(0 if all(x['compiler_exit_code']==0 and x['source_unchanged'] for x in records) else 1)
