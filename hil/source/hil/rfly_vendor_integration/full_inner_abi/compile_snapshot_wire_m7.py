"""Compile the snapshot codec and reader with NuttX flags."""

# Component inputs are supplied by the invoking build/test environment.
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from component_inputs import component_input
component_input('GPENMPC_BUILD_ROOT')

from pathlib import Path
import hashlib,json,os,shlex,subprocess,sys,re
root=Path(__file__).resolve().parent
assert re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_-]*',sys.argv[1])
out=root/sys.argv[1];out.mkdir(exist_ok=False)
vi=root.parent;app=component_input('GPENMPC_BUILD_ROOT')
tracked=[vi/'px4_wire/CanonicalLocalSnapshotWire.hpp',vi/'px4_wire/CanonicalLocalGpWire.hpp',
 vi/'local_source_ingress/Px4OriginalHilReceiptReader.hpp',vi/'local_source_ingress/Px4OriginalHilReceiptReader.cpp',
 vi/'CanonicalLocalInnerInputBuilder.hpp',vi.parent/'px4_full_inner/px4_state_adapter/AtomicOdometryAdapter.hpp',
 app/'uORB/topics/gpenmpc_original_hil_receipt.h',app/'uORB/topics/uORBTopics.hpp',root/'probe_snapshot_wire_m7.cpp',Path(__file__)]
def hashes():return {str(p):hashlib.sha256(p.read_bytes()).hexdigest().upper() for p in tracked}
report=dict(scope='RLS1_AND_ACTUAL_READER_M7_FULL_TABLE_COMPILE_ONLY',source_sha256=hashes(),records=[],linked=False,executed=False)
try:
 entry=next(e for e in json.loads((app/'compile_commands.json').read_text()) if Path(e['file']).name in ('Px4CanonicalLocalIo.cpp', 'Px4CanonicalIo.cpp'))
 argv=shlex.split(entry['command'])
 if Path(argv[0]).name=='ccache':argv=argv[1:]
 assert all(f in argv for f in ['-std=gnu++14','-nostdinc++','-mcpu=cortex-m7','-mfloat-abi=hard','-D__PX4_NUTTX'])
 for name,source in [('wire',root/'probe_snapshot_wire_m7.cpp'),('reader',vi/'local_source_ingress/Px4OriginalHilReceiptReader.cpp')]:
  args=list(argv);obj=out/(name+'.o');args[args.index('-c')+1]=str(source);args[args.index('-o')+1]=str(obj)
  args[1:1]=['-fstack-usage','-MD','-MF',str(out/(name+'.d'))]
  p=subprocess.run(args,cwd=entry['directory'],env=dict(os.environ,CCACHE_DISABLE='1'),capture_output=True,text=True)
  r=dict(name=name,command=args,exit=p.returncode,output=p.stdout+p.stderr);report['records'].append(r)
  print(name,p.returncode,p.stdout+p.stderr,flush=True);assert p.returncode==0
  def tool(name,*options):
   binary=Path(args[0]).with_name(Path(args[0]).name.replace('g++',name))
   return subprocess.check_output([str(binary),*options,str(obj)],text=True)
  r.update(object_sha256=hashlib.sha256(obj.read_bytes()).hexdigest().upper(),size=tool('size'),symbols=tool('nm','-S','-C'),
   undefined=tool('nm','-u','-C'),stack_usage=obj.with_suffix('.su').read_text())
 report['pass']=True
except Exception as e:report['pass']=False;report['error']=str(e)
report['source_sha256_after']=hashes();report['source_stable']=report['source_sha256']==report['source_sha256_after']
(out/'RESULT.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps({k:v for k,v in report.items() if k not in ['records','source_sha256','source_sha256_after']},indent=2))
sys.exit(0 if report.get('pass') and report['source_stable'] else 1)
