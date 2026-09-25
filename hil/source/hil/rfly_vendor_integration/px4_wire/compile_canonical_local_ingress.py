"""Compile the local ingress interface with Cortex-M7/NuttX flags."""

# Component inputs are supplied by the invoking build/test environment.
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from component_inputs import component_input
component_input('GPENMPC_BUILD_ROOT')

from pathlib import Path
import hashlib,json,os,shlex,subprocess,sys
root=Path(__file__).resolve().parent;vi=root.parent
label=sys.argv[1] if len(sys.argv)>1 else 'verification'
assert label and all(c.isalnum() or c in '_-' for c in label)
out=vi/'full_inner_abi'/('local_ingress_m7_'+label);out.mkdir(exist_ok=False)
app=component_input('GPENMPC_BUILD_ROOT')
tracked=[root/'CanonicalLocalIngress.hpp',root/'CanonicalLocalGpWire.hpp',root/'probe_canonical_local_ingress.cpp',
    vi.parent/'px4_full_inner/argument_transport/CanonicalArgumentTransport.hpp',vi.parent/'px4_full_inner/px4_ingress/GPENMPCFullInnerIngress.hpp',
    app/'uORB/topics/gpenmpc_full_inner_ingress.h',app/'uORB/topics/uORBTopics.hpp']
def hashes():return {str(p):hashlib.sha256(p.read_bytes()).hexdigest().upper() for p in tracked}
report=dict(scope='ACTUAL_NUTTX_M7_GP_CENTRAL_INGRESS_TU_COMPILE_ONLY',source_sha256=hashes(),executed=False,linked=False)
try:
    entry=next(x for x in json.loads((app/'compile_commands.json').read_text()) if Path(x['file']).name in ('Px4CanonicalLocalIo.cpp', 'Px4CanonicalIo.cpp'))
    args=shlex.split(entry['command']);args=args[1:] if Path(args[0]).name=='ccache' else args
    assert all(x in args for x in ['-std=gnu++14','-nostdinc++','-mcpu=cortex-m7','-mfloat-abi=hard'])
    obj=out/'ingress.o';args[args.index('-c')+1]=str(root/'probe_canonical_local_ingress.cpp');args[args.index('-o')+1]=str(obj)
    args[1:1]=['-fstack-usage','-MD','-MF',str(out/'ingress.d')]
    p=subprocess.run(args,cwd=entry['directory'],env=dict(os.environ,CCACHE_DISABLE='1'),capture_output=True,text=True)
    report.update(command=args,exit=p.returncode,output=p.stdout+p.stderr);print(p.returncode,p.stdout+p.stderr,flush=True);assert p.returncode==0
    def tool(name,*flags):return subprocess.check_output([str(Path(args[0]).with_name(Path(args[0]).name.replace('g++',name))),*flags,str(obj)],text=True)
    report.update(pass_=True,object_sha256=hashlib.sha256(obj.read_bytes()).hexdigest().upper(),size=tool('size'),symbols=tool('nm','-S','-C'),undefined=tool('nm','-u','-C'),stack_usage=obj.with_suffix('.su').read_text())
except Exception as e:report['error']=str(e)
report['source_sha256_after']=hashes();report['source_stable']=report['source_sha256']==report['source_sha256_after']
(out/'RESULT.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps({k:v for k,v in report.items() if k not in ['source_sha256','source_sha256_after','command','symbols','stack_usage']},indent=2))
sys.exit(0 if report.get('pass_') and report['source_stable'] else 1)
