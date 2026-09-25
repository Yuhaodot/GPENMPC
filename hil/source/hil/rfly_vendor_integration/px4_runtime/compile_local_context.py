"""Compile the local context and exchange core with target flags."""

# Component inputs are supplied by the invoking build/test environment.
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from component_inputs import component_input
component_input('GPENMPC_BUILD_ROOT')

from pathlib import Path
import hashlib,json,os,shlex,subprocess,sys
root=Path(__file__).resolve().parent;vi=root.parent
label=sys.argv[1];assert label and all(c.isalnum() or c in '_-' for c in label)
out=vi/'full_inner_abi'/('local_context_m7_'+label);out.mkdir(exist_ok=False)
app=component_input('GPENMPC_BUILD_ROOT')
sources=[root/'CanonicalLocalExchangeCore.cpp',root/'RegisteredCanonicalLocalModuleContext.cpp',root/'Px4CanonicalLocalIo.cpp']
tracked=sources+[p.with_suffix('.hpp') for p in sources]+[root/'LocalCommittedState.hpp',root/'CanonicalLocalGpPending.hpp',root/'CanonicalLocalWireOutbox.hpp',root/'CanonicalLocalExecutionCycle.hpp',vi/'px4_wire/CanonicalLocalIngress.hpp',vi/'local_source_ingress/Px4SelectedSourceReader.hpp',vi/'local_source_ingress/Px4OriginalHilReceiptReader.hpp',app/'uORB/topics/gpenmpc_full_inner_ingress.h',app/'uORB/topics/gpenmpc_original_hil_receipt.h']
def hashes():return {str(p):hashlib.sha256(p.read_bytes()).hexdigest().upper() for p in tracked}
report=dict(scope='ACTUAL_M7_LOCAL_CONTEXT_CORE_COMPILE_NOT_SELECTED_APPLICATION',source_before=hashes(),records=[])
try:
    entry=next(x for x in json.loads((app/'compile_commands.json').read_text()) if Path(x['file']).name in ('Px4CanonicalLocalIo.cpp', 'Px4CanonicalIo.cpp'))
    base=shlex.split(entry['command']);base=base[1:] if Path(base[0]).name=='ccache' else base
    assert all(x in base for x in ['-std=gnu++14','-nostdinc++','-mcpu=cortex-m7','-mfloat-abi=hard'])
    for s in sources:
        a=list(base);obj=out/(s.stem+'.o');a[a.index('-c')+1]=str(s);a[a.index('-o')+1]=str(obj)
        a[1:1]=['-fstack-usage','-MD','-MF',str(obj.with_suffix('.d'))]
        p=subprocess.run(a,cwd=entry['directory'],env=dict(os.environ,CCACHE_DISABLE='1'),capture_output=True,text=True)
        r=dict(source=str(s),command=a,exit=p.returncode,output=p.stdout+p.stderr);report['records'].append(r);print(s.name,p.returncode,p.stdout+p.stderr,flush=True)
        if p.returncode:continue
        def tool(name,*flags):return subprocess.check_output([str(Path(a[0]).with_name(Path(a[0]).name.replace('g++',name))),*flags,str(obj)],text=True)
        r.update(object_sha256=hashlib.sha256(obj.read_bytes()).hexdigest().upper(),size=tool('size'),undefined=tool('nm','-u','-C'),stack=obj.with_suffix('.su').read_text())
    report['passed']=all(r['exit']==0 for r in report['records']) and len(report['records'])==len(sources)
except Exception as e:report['error']=str(e)
report['source_after']=hashes();report['source_stable']=report['source_before']==report['source_after']
(out/'RESULT.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps({k:v for k,v in report.items() if k not in ['records','source_before','source_after']}))
sys.exit(0 if report.get('passed') and report['source_stable'] else 1)
