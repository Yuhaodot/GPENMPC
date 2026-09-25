"""Compile only the changed local exchange/stream-owner TUs against real NuttX.
No device access, process control, uploader or live firmware invocation.
"""
from pathlib import Path
import hashlib,json,os,shlex,subprocess,sys
build=Path(__file__).resolve().parent.parent
vi=build/'rfly_vendor_integration';app=vi/'application_integration/build_fmuv6c'
mode=sys.argv[1] if len(sys.argv)>1 else 'exchange_core'
assert mode in ['exchange_core','application_owner','task_input','task_input_owner','session_entry']
root=vi/'full_inner_abi'/(mode+'_arm_objects')
assert not root.exists();root.mkdir()
db=json.loads((app/'compile_commands.json').read_text())
entry=next(r for r in db if Path(r['file']).name=='Px4CanonicalIo.cpp')
flags=shlex.split(entry['command']);flags=flags[1:] if Path(flags[0]).name=='ccache' else flags
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest().upper()
records=[]
names=(['CanonicalLocalSessionEntry.cpp'] if mode=='session_entry' else
       ['CanonicalLocalTaskInputOwner.cpp'] if mode=='task_input_owner' else
       ['CanonicalLocalApplicationOwner.cpp'] if mode=='application_owner' else
       ['CanonicalLocalExchangeCore.cpp','CanonicalLocalTaskInputOwner.cpp','CanonicalLocalApplicationOwner.cpp','RegisteredCanonicalLocalModuleContext.cpp'] if mode=='task_input' else
       ['CanonicalLocalExchangeCore.cpp','CanonicalLocalWireOutbox.cpp','RegisteredCanonicalLocalModuleContext.cpp'])
for name in names:
    source=vi/'px4_runtime'/name;obj=root/(source.stem+'.o');args=list(flags)
    args[args.index('-o')+1]=str(obj);args[args.index('-c')+1]=str(source)
    args[1:1]=['-DGPENMPC_CANONICAL_CLOSED_EVIDENCE=1','-fstack-usage','-MD','-MF',str(obj.with_suffix('.d'))]
    before=sha(source)
    p=subprocess.run(args,cwd=entry['directory'],capture_output=True,text=True,env=dict(os.environ,CCACHE_DISABLE='1'))
    record=dict(source=str(source),sha256=before,stable=before==sha(source),command=args,exit=p.returncode,stdout=p.stdout,stderr=p.stderr)
    if p.returncode==0:
        record.update(object_sha256=sha(obj),stack_usage=obj.with_suffix('.su').read_text())
        tool=str(Path(flags[0]).with_name('arm-none-eabi-size'))
        record['size']=subprocess.run([tool,str(obj)],capture_output=True,text=True,check=True).stdout
    records.append(record);print(name,p.returncode,flush=True)
    if p.returncode:print(p.stdout+p.stderr,flush=True)
result=dict(passed=all(r['exit']==0 and r['stable'] for r in records),records=records,
    compiler_sha256=sha(Path(flags[0])),COM=0,board=0,flash=0,full_application_linked=False)
(root/'RESULT.json').write_text(json.dumps(result,indent=2)+'\n')
sys.exit(0 if result['passed'] else 1)
