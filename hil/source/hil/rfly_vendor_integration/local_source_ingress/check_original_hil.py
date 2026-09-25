"""Generate uORB topics, test publication fixtures and compile the M7 interface."""

# Component inputs are supplied by the invoking build/test environment.
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from component_inputs import component_input
component_input('GPENMPC_BUILD_ROOT')

from pathlib import Path
import hashlib, json, os, re, shlex, subprocess, sys

root=Path(__file__).resolve().parent
label=sys.argv[1];assert re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_-]*',label)
out=root/label;out.mkdir(exist_ok=False)
px4=Path(os.environ['GPENMPC_PX4_ROOT'])
integration=root.parent;app=component_input('GPENMPC_BUILD_ROOT')
generated=out/'generated';headers=generated/'uORB/topics';headers.mkdir(parents=True)
report=dict(scope='OFFICIAL_UORB_SINGLE_TOPIC_AND_REAL_MAVLINK_MOCK_HOST_PUBLICATION_M7_COMPILE',
            application_message_table=False,actual_uorb_HOST=False,PX4_run=False,records=[])
tracked=[root/'Px4OriginalHilReceipt.hpp',root/'GPENMPCOriginalHilReceipt.msg',
         integration/'clock_tap_overlay/ClockObservationTap.hpp',root/'test_original_hil.cpp',root/'probe_original_hil.cpp']
def hashes():return {str(p):hashlib.sha256(p.read_bytes()).hexdigest().upper() for p in tracked}
report['source_sha256']=hashes()
def run(name,args,cwd=root):
    p=subprocess.run(list(map(str,args)),cwd=cwd,env=dict(os.environ,CCACHE_DISABLE='1'),capture_output=True,text=True)
    report['records'].append(dict(name=name,command=list(map(str,args)),cwd=str(cwd),exit=p.returncode,output=p.stdout+p.stderr))
    print(name,p.returncode,flush=True)
    if p.returncode:print(p.stdout+p.stderr,flush=True)
    return p
try:
    generator=px4/'Tools/msg/px_generate_uorb_topic_files.py'
    report['generator_sha256']=hashlib.sha256(generator.read_bytes()).hexdigest().upper()
    p=run('official_uorb_headers',['/usr/bin/python3',generator,'--headers','-f',root/'GPENMPCOriginalHilReceipt.msg',
          '-i',px4/'msg',px4/'msg/versioned','-o',headers,'-e',px4/'Tools/msg/templates/uorb'])
    assert p.returncode==0
    common=['-std=c++14','-O2','-Wall','-Wextra','-Werror','-Wno-address-of-packed-member',
            '-I'+str(root/'host_mock'),'-I'+str(generated),'-isystem',str(app/'mavlink')]
    executable=out/'test_original_hil'
    p=run('HOST_compile',['/usr/bin/g++',*common,root/'test_original_hil.cpp','-o',executable]);assert p.returncode==0
    p=run('HOST_test',[executable]);assert p.returncode==0
    report['HOST_result']=json.loads(p.stdout)
    entry=next(r for r in json.loads((app/'compile_commands.json').read_text()) if Path(r['file']).name in ('Px4CanonicalLocalIo.cpp', 'Px4CanonicalIo.cpp'))
    args=shlex.split(entry['command']);args=args[1:] if Path(args[0]).name=='ccache' else args
    assert all(x in args for x in ['-std=gnu++14','-nostdinc++','-mcpu=cortex-m7','-mfloat-abi=hard'])
    obj=out/'original_hil_m7.o';args[args.index('-o')+1]=str(obj);args[args.index('-c')+1]=str(root/'probe_original_hil.cpp')
    args[1:1]=['-I'+str(generated),'-fstack-usage','-MD','-MF',str(out/'original_hil_m7.d')]
    p=run('actual_M7_compile',args,Path(entry['directory']));assert p.returncode==0
    def tool(name,*a):return subprocess.check_output([str(Path(args[0]).with_name(Path(args[0]).name.replace('g++',name))),*map(str,a)],text=True)
    report['M7']=dict(object_sha256=hashlib.sha256(obj.read_bytes()).hexdigest().upper(),
        size=tool('size',obj),undefined=tool('nm','-u','-C',obj),stack_usage=obj.with_suffix('.su').read_text(),
        real_uorb_headers=True,real_generated_MAVLink=True,linked=False,executed=False)
    report['pass']=True
except Exception as error:
    report['pass']=False;report['error']=str(error)
report['source_sha256_after']=hashes();report['source_stable']=report['source_sha256']==report['source_sha256_after']
(out/'RESULT.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps({k:v for k,v in report.items() if k not in ['records','M7','source_sha256','source_sha256_after']},indent=2))
sys.exit(0 if report.get('pass') and report['source_stable'] else 1)
