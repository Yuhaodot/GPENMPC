
# Component inputs are supplied by the invoking build/test environment.
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from component_inputs import component_input
component_input('GPENMPC_BUILD_ROOT')

from pathlib import Path
import hashlib,json,os,re,shlex,subprocess,sys
root=Path(__file__).resolve().parent
if len(sys.argv)<2 or not Path(sys.argv[1]).is_absolute():raise ValueError('Provide the absolute host ABI result directory')
out=Path(sys.argv[1]).resolve();assert out.is_dir()
arm_label=sys.argv[2] if len(sys.argv)>2 else 'arm'
assert arm_label=='arm' or re.fullmatch('[A-Za-z0-9][A-Za-z0-9_-]*',arm_label)
receipt=out/(arm_label.upper()+'.json');assert not receipt.exists()
build=root.parent.parent
base=Path(os.environ['GPENMPC_KERNEL_BUILD'])
archive=Path(os.environ['GPENMPC_KERNEL_ARCHIVE'])
host=json.loads((out/'RESULT.json').read_text());assert host['run_exit']==0 and host['source_set_stable']
closed=host.get('closed_evidence_extension',False)
learning=host.get('learning_audit_extension',False)
if learning:
 assert closed
assert base.is_dir() and archive.is_file()
armout=out/arm_label;armout.mkdir(exist_ok=False)
identity=host['identity'].copy();identity['private_archive_sha256']=hashlib.sha256(archive.read_bytes()).hexdigest().upper()
header='#pragma once\n'
for name,key in [('rfi_generated_source_sha','generated_source_set_sha256'),('rfi_facade_source_sha','facade_source_set_sha256'),('rfi_private_archive_sha','private_archive_sha256')]:
 header+='static const unsigned char '+name+'[32]={'+','.join('0x%02X'%b for b in bytes.fromhex(identity[key]))+'};\n'
(armout/'FullInnerBuildIdentity.h').write_text(header)
db=component_input('GPENMPC_BUILD_ROOT') / 'compile_commands.json'
rows=json.loads(db.read_text())
row=next((r for r in rows if Path(r['file']).name=='Px4CanonicalLocalIo.cpp'),None)
if row is None:row=next(r for r in rows if Path(r['file']).name=='Px4CanonicalIo.cpp')
args=shlex.split(row['command']);args=args[1:] if Path(args[0]).name=='ccache' else args
assert all(x in args for x in ['-std=gnu++14','-nostdinc++','-mcpu=cortex-m7','-mfloat-abi=hard'])
source=root/'CanonicalFullInnerAbi.cpp';obj=armout/'facade.o'
args[args.index('-o')+1]=str(obj);args[args.index('-c')+1]=str(source)
args[1:1]=['-DGPENMPC_CANONICAL_EXPLICIT_WORKSPACE=1','-DGPENMPC_ABI_SIZE_PROBE=1','-I'+str(armout),'-I'+str(base/'generated'),
 '-isystem',str(Path(os.environ['GPENMPC_MATLAB_ROOT'])/'extern/include'),'-fstack-usage','-MD','-MF',str(armout/'facade.d')]
if closed:args[1:1]=['-DGPENMPC_CANONICAL_CLOSED_EVIDENCE=1']
if learning:args[1:1]=['-DGPENMPC_CANONICAL_LEARNING_AUDIT=1']
run=subprocess.run(args,cwd=row['directory'],env=dict(os.environ,CCACHE_DISABLE='1'),capture_output=True,text=True)
tool=lambda name:str(Path(args[0]).with_name(Path(args[0]).name.replace('g++',name)))
call=lambda name,*a:subprocess.check_output([tool(name),*map(str,a)],text=True)
report=dict(command=args,working_directory=row['directory'],compile_exit=run.returncode,output=run.stdout+run.stderr,
 identity=identity,actual_GNU14_NuttX_M7=True,generated_C_recompiled=0,board_execution=False,application_linked=False)
if not run.returncode:
 nm=call('nm','-S','--size-sort',obj);markers={p[-1]:int(p[1],16) for l in nm.splitlines() if len(p:=l.split())>=4 and p[-1].startswith('gpenmpc_full_inner_sizeof_')}
 undefined=call('nm','-u','-C',obj);size=call('size',obj).splitlines()[1].split()
 linked=armout/'facade_private74_relocatable.o'
 link_cmd=[tool('ld'),'-r',str(obj),'--whole-archive',str(archive),'--no-whole-archive','-o',str(linked)]
 link=subprocess.run(link_cmd,capture_output=True,text=True)
 report.update(sizeof=markers,object_sha256=hashlib.sha256(obj.read_bytes()).hexdigest().upper(),
  sections=dict(text_with_markers=int(size[0]),data=int(size[1]),bss=int(size[2]),markers=sum(markers.values()),text_excluding_markers=int(size[0])-sum(markers.values())),
  stack_usage=obj.with_suffix('.su').read_text(),facade_undefined=undefined,relocatable_link_command=link_cmd,
  relocatable_link_exit=link.returncode,relocatable_output=link.stdout+link.stderr,
  forbidden_dependencies=[l for l in undefined.splitlines() if re.search(r'atomic|__cxa|__gxx|std::|operator new|operator delete',l)])
 if not link.returncode:report['relocatable_undefined']=call('nm','-u','-C',linked)
receipt.write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps({k:report[k] for k in report if k not in ['command','stack_usage','working_directory']},indent=2));sys.exit(run.returncode or report.get('relocatable_link_exit',0))
