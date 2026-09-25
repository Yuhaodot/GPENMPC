"""Compile the selected-source reader against the FMUv6C topic table."""

# Component inputs are supplied by the invoking build/test environment.
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from component_inputs import component_input
component_input('GPENMPC_BUILD_ROOT')

from pathlib import Path
import hashlib,json,os,shlex,subprocess,sys
root=Path(__file__).resolve().parent
if len(sys.argv)!=2 or not Path(sys.argv[1]).is_absolute():raise ValueError('Provide an absolute result directory')
result_root=Path(sys.argv[1]).resolve()
if result_root.is_relative_to(root.parent.parent):raise ValueError('Result directory must be outside source inputs')
out=result_root/'m7';out.mkdir(exist_ok=True)
assert not (out/'RESULT.json').exists(), 'preserve prior compile result'
app=component_input('GPENMPC_BUILD_ROOT')
header=app/'uORB/topics/gpenmpc_original_hil_receipt.h'
assert 'gyro_topic_instance' in header.read_text(), 'actual full-table schema not updated'
tracked=[root/'Px4SelectedSourceReader.hpp',root/'Px4SelectedSourceReader.cpp',root/'probe_selected_source_size.cpp',
    root.parent.parent/'px4_full_inner/px4_state_adapter/AtomicOdometryAdapter.hpp',header,app/'uORB/topics/uORBTopics.hpp']
def hashes():return {str(p):hashlib.sha256(p.read_bytes()).hexdigest().upper() for p in tracked}
report=dict(scope='ACTUAL_FMUV6C_NUTTX_M7_SELECTED_READER_COMPILE_ONLY',source_sha256=hashes(),records=[],linked=False,executed=False)
try:
    entry=next(e for e in json.loads((app/'compile_commands.json').read_text()) if Path(e['file']).name in ('Px4CanonicalLocalIo.cpp', 'Px4CanonicalIo.cpp'))
    argv=shlex.split(entry['command']);argv=argv[1:] if Path(argv[0]).name=='ccache' else argv
    assert all(x in argv for x in ['-std=gnu++14','-nostdinc++','-mcpu=cortex-m7','-mfloat-abi=hard'])
    for stem,source in [('selected_reader','Px4SelectedSourceReader.cpp'),('sizes','probe_selected_source_size.cpp')]:
        args=list(argv);obj=out/(stem+'.o');args[args.index('-c')+1]=str(root/source);args[args.index('-o')+1]=str(obj)
        args[1:1]=['-fstack-usage','-MD','-MF',str(out/(stem+'.d'))]
        p=subprocess.run(args,cwd=entry['directory'],env=dict(os.environ,CCACHE_DISABLE='1'),capture_output=True,text=True)
        record=dict(source=source,command=args,exit=p.returncode,output=p.stdout+p.stderr);report['records'].append(record)
        print(stem,p.returncode,p.stdout+p.stderr,flush=True);assert p.returncode==0
        def tool(name,*options):return subprocess.check_output([str(Path(args[0]).with_name(Path(args[0]).name.replace('g++',name))),*options,str(obj)],text=True)
        record.update(object_sha256=hashlib.sha256(obj.read_bytes()).hexdigest().upper(),size=tool('size'),symbols=tool('nm','-S','-C'),undefined=tool('nm','-u','-C'),stack_usage=obj.with_suffix('.su').read_text())
    report['pass']=True
except Exception as e:report.update(error=str(e))
report['source_sha256_after']=hashes();report['source_stable']=report['source_sha256']==report['source_sha256_after']
(out/'RESULT.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps({k:v for k,v in report.items() if k not in ['records','source_sha256','source_sha256_after']},indent=2))
sys.exit(0 if report.get('pass') and report['source_stable'] else 1)
