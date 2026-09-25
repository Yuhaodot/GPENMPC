"""Compile the HIL reader using NuttX/Cortex-M7 flags."""

# Component inputs are supplied by the invoking build/test environment.
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from component_inputs import component_input
component_input('GPENMPC_BUILD_ROOT')

from pathlib import Path
import hashlib,json,os,shlex,subprocess,sys
root=Path(__file__).resolve().parent
out=root/sys.argv[1]
assert out.is_dir() and not (out/'M7_RESULT.json').exists()
app=component_input('GPENMPC_BUILD_ROOT')
header=app/'uORB/topics/gpenmpc_original_hil_receipt.h'
assert header.is_file(), 'Actual full-table topic not yet generated'
tracked=[root/'Px4OriginalHilReceiptReader.hpp',root/'Px4OriginalHilReceiptReader.cpp',root/'probe_reader_size.cpp',
         header,app/'uORB/topics/uORBTopics.hpp',root.parent/'clock_tap_overlay/ExactSourceReceiptLookup.hpp']
def hashes():return {str(p):hashlib.sha256(p.read_bytes()).hexdigest().upper() for p in tracked}
report=dict(scope='ACTUAL_CORTEX_M7_NUTTX_READER_TU_FULL_UORB_TABLE_COMPILE_ONLY',source_sha256=hashes(),records=[],linked=False,executed=False)
try:
    entry=next(e for e in json.loads((app/'compile_commands.json').read_text()) if Path(e['file']).name in ('Px4CanonicalLocalIo.cpp', 'Px4CanonicalIo.cpp'))
    argv=shlex.split(entry['command'])
    if Path(argv[0]).name=='ccache':argv=argv[1:]
    assert all(flag in argv for flag in ['-std=gnu++14','-nostdinc++','-mcpu=cortex-m7','-mfloat-abi=hard'])
    for stem,source in [('reader','Px4OriginalHilReceiptReader.cpp'),('sizes','probe_reader_size.cpp')]:
        args=list(argv);obj=out/(stem+'.o');args[args.index('-c')+1]=str(root/source);args[args.index('-o')+1]=str(obj)
        args[1:1]=['-fstack-usage','-MD','-MF',str(out/(stem+'.d'))]
        p=subprocess.run(args,cwd=entry['directory'],env=dict(os.environ,CCACHE_DISABLE='1'),capture_output=True,text=True)
        r=dict(name=stem,command=args,exit=p.returncode,output=p.stdout+p.stderr)
        report['records'].append(r);print(stem,p.returncode,p.stdout+p.stderr,flush=True)
        assert p.returncode==0
        def tool(name,*options):
            binary=Path(args[0]).with_name(Path(args[0]).name.replace('g++',name))
            return subprocess.check_output([str(binary),*options,str(obj)],text=True)
        r.update(object_sha256=hashlib.sha256(obj.read_bytes()).hexdigest().upper(),size=tool('size'),symbols=tool('nm','-S','-C'),
                 undefined=tool('nm','-u','-C'),stack_usage=obj.with_suffix('.su').read_text())
    report['pass']=True
except Exception as e:
    report['pass']=False;report['error']=str(e)
report['source_sha256_after']=hashes();report['source_stable']=report['source_sha256']==report['source_sha256_after']
(out/'M7_RESULT.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps({k:v for k,v in report.items() if k not in ['records','source_sha256','source_sha256_after']},indent=2))
sys.exit(0 if report.get('pass') and report['source_stable'] else 1)
