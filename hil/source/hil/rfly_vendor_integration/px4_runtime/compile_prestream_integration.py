"""Compile the prestream integration with FMUv6C/NuttX flags."""

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
label=sys.argv[1]; assert (label and all(c.isalnum() or c in '_-' for c in label))
stems=sys.argv[2:] or ['LegacyTrajectoryReadOnly']
allowed={'LegacyTrajectoryReadOnly':root,'RegisteredCanonicalModuleContext':root,
 'GPENMPCRflyCanonicalModule':root,'Px4ReadOnlyGuard':root,'Px4CanonicalIo':root,
 'LinkLifetimeRegistry':root.parent/'px4_stream','LinkCanonicalReservation':root.parent/'px4_stream','probe_prestream_size':root}
assert all(s in allowed for s in stems)
dest=root/('prestream_arm_'+label);dest.mkdir(exist_ok=False)
px4=Path(os.environ['GPENMPC_PX4_ROOT'])
db=component_input('GPENMPC_BUILD_ROOT') / 'compile_commands.json'
entry,=[r for r in json.loads(db.read_text()) if r['file']==str(px4/'src/modules/mavlink/mavlink_receiver.cpp')]
inputs=[p for d in [root,root.parent/'px4_stream'] for p in d.glob('*.hpp')]+[allowed[s]/(s+'.cpp') for s in stems]
before={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in inputs};records=[]
for stem in stems:
 source=allowed[stem]/(stem+'.cpp');obj=dest/(stem+'.o');args=shlex.split(entry['command'])
 assert '-std=gnu++14' in args and '-nostdinc++' in args
 args[args.index('-c')+1]=str(source);args[args.index('-o')+1]=str(obj)
 args[1:1]=['-I'+str(component_input('GPENMPC_TOPIC_HEADERS')),
  '-I'+str(root.parent/'application_integration/overlay/src/modules/mavlink'),
  '-I'+str(px4/'src/modules/mavlink'),
  '-I'+str(root.parent.parent/'px4_full_inner/px4_ingress'),
  '-I'+str(root.parent.parent/'evidence/arm_controller/GPENMPC_Rfly_Canonical_Controller_ert_rtw'),
  '-isystem',str(component_input('GPENMPC_NUTTX_INCLUDE')),
  '-ffp-contract=off','-fstack-usage','-MD','-MF',str(obj.with_suffix('.d'))]
 r=subprocess.run(args,cwd=entry['directory'],capture_output=True,text=True)
 undefined=subprocess.check_output([args[0].replace('g++','nm'),'-u','-C',str(obj)],text=True) if r.returncode==0 else ''
 records.append(dict(source=str(source),command_argv=args,working_directory=entry['directory'],exit_code=r.returncode,
  output=r.stdout+r.stderr,undefined_symbols=undefined,
  forbidden_runtime_dependency=any(x in undefined for x in ['__atomic_','__cxa_throw','__gxx_personality','std::']),
  stack_usage=obj.with_suffix('.su').read_text() if obj.with_suffix('.su').exists() else '',
  object_sha256=hashlib.sha256(obj.read_bytes()).hexdigest() if obj.exists() else None,
  object_bytes=obj.stat().st_size if obj.exists() else None,
  section_sizes=subprocess.check_output([args[0].replace('g++','size'),str(obj)],text=True) if r.returncode==0 else '',
  size_probe_symbols=subprocess.check_output([args[0].replace('g++','nm'),'-S','--size-sort',str(obj)],text=True) if r.returncode==0 and stem=='probe_prestream_size' else ''))
 print(stem,r.returncode,r.stdout+r.stderr)
report=dict(scope='ACTUAL_FMUV6C_NUTTX_GNUXX14_PRODUCTION_TU_COMPILE_ONLY',
 records=records,source_hashes=before,sources_unchanged=before=={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in inputs})
report['passed']=report['sources_unchanged'] and all(r['exit_code']==0 and not r['forbidden_runtime_dependency'] for r in records)
(dest/'RESULT.json').write_text(json.dumps(report,indent=2)+'\n')
print('passed='+str(report['passed']));sys.exit(0 if report['passed'] else 1)
