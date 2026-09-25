#!/usr/bin/env python3
"""Real M7 C++ smoke and conservative direct-call graph of retained objects.
No target execution, application link or firmware/stack configuration change.
"""
from pathlib import Path
from gpenmpc_external_path import external_path
import hashlib
import json
import os
import re
import shlex
import subprocess
import sys

def matlab_include_path():
    root = os.environ.get('GPENMPC_MATLAB_ROOT', '').strip()
    if not root:
        raise RuntimeError('Set GPENMPC_MATLAB_ROOT to the MATLAB installation directory.')
    if os.name != 'nt' and len(root) >= 3 and root[1] == ':':
        root = subprocess.run(['wslpath', '-u', root], check=True, text=True,
                              capture_output=True).stdout.strip()
    include = Path(root) / 'extern' / 'include'
    if not include.is_dir():
        raise FileNotFoundError('MATLAB external headers were not found: ' + str(include))
    return str(include.resolve())


matlab_include = matlab_include_path()

build=Path(__file__).resolve().parent.parent
root=Path(external_path('canonical_combined_numerics'))
generated=root/'generated'
arm=Path(external_path('combined_arm_objects'))
label=sys.argv[1] if len(sys.argv)==2 else 'arm_stack'
assert re.fullmatch(r'arm_stack(?:_[a-z][a-z0-9_]*)?',label)
out=root/label
out.mkdir(exist_ok=False)
db=build/'rfly_vendor_integration/application_integration/build_fmuv6c/compile_commands.json'
rows=[r for r in json.loads(db.read_text()) if Path(r['file']).name=='Px4CanonicalIo.cpp']
assert len(rows)==1
row=rows[0]
args=shlex.split(row['command'])
if Path(args[0]).name=='ccache':args=args[1:]
assert all(f in args for f in ['-std=gnu++14','-nostdinc++','-mcpu=cortex-m7','-mfloat-abi=hard'])
source=build/'rfly_vendor_integration/probe_canonical_combined_state_store.cpp'
header=build/'rfly_vendor_integration/CanonicalLocalInnerStateStore.hpp'
obj=out/'probe.o'
args[args.index('-o')+1]=str(obj)
args[args.index('-c')+1]=str(source)
args[1:1]=['-DGPENMPC_CANONICAL_EXPLICIT_WORKSPACE=1','-I'+str(generated),'-isystem',matlab_include,
           '-fstack-usage','-MD','-MF',str(out/'probe.d')]
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest().upper()
tracked=[source,header,db,Path(__file__)]
before={str(p):sha(p) for p in tracked}
run=subprocess.run(args,cwd=row['directory'],env=dict(os.environ,CCACHE_DISABLE='1'),capture_output=True,text=True)
(out/'COMPILE_LOG.txt').write_text(run.stdout+run.stderr)
report=dict(scope='ACTUAL_M7_HEADER_SMOKE_AND_STATIC_CALLGRAPH_ONLY',command=args,
            compiler_exit_code=run.returncode,source_sha256=before,source_stable=before=={str(p):sha(p) for p in tracked},
            explicit_workspace_macro=1,workspace_is_caller_owned=True,coder_stack_usage_max_is_not_task_bound=True,
            current_task_stack_bytes=8192,stack_configuration_changed=False,board_actions=0,
            target_execution=False,firmware_linked=False)
if run.returncode:
    (out/'RESULT.json').write_text(json.dumps(report,indent=2)+'\n')
    print(run.stdout+run.stderr);raise SystemExit(run.returncode)
tool=lambda name:str(Path(args[0]).with_name(Path(args[0]).name.replace('g++',name)))
nm=subprocess.run([tool('nm'),'-S','--size-sort','-C',str(obj)],capture_output=True,text=True,check=True).stdout
(out/'NM.txt').write_text(nm)
report['object_sha256']=sha(obj)
report['object_bytes']=obj.stat().st_size
report['probe_stack_raw']=(out/'probe.su').read_text()
sizes={}
for line in nm.splitlines():
    parts=line.split()
    if len(parts)>=4 and 'gpenmpc_size_' in parts[-1]:sizes[parts[-1]]=int(parts[1],16)
report['sizeof_symbols']=sizes
# Require generated sources and objects to match the recorded ARM build.
receipt=json.loads((arm/'RESULT.json').read_text())
assert receipt['passed'] and receipt['compiled_tus']==74 and receipt['sources_unchanged']
for path,digest in receipt['source_sha256_after'].items():
    assert sha(Path(path))==digest,('Changed after actual C compile',path)
for r in receipt['records']:
    assert sha(Path(r['object']))==r['object_sha256']
objects=[Path(r['object']) for r in receipt['records']]+[obj]
nodes={}; unresolved=[]; indirect=[]; stack_kinds=[]
def name_key(s):
    # .su C++ names include return types; c++filt names do not.
    # Use the maximum stack allocation among same-name clones.
    s=s.split(' [clone ')[0]
    s=s.split('(')[0].strip().split()[-1]
    # GCC .su retains evaluateAt.isra whereas its corresponding ELF symbol
    # is evaluateAt.isra.0. Retain the transformation kind and discard only
    # the clone ordinal; any same-key frames are conservatively max-combined.
    s=re.sub(r'\.(isra|constprop)\.\d+$',r'.\1',s)
    return s
for object_path in objects:
    stack={}
    for line in object_path.with_suffix('.su').read_text().splitlines():
        m=re.match(r'^.+?:\d+:\d+:(.+)\t(\d+)\t(.+)$',line)
        assert m,line
        key=name_key(m.group(1));v=int(m.group(2))
        stack[key]=max(stack.get(key,0),v)
        stack_kinds.append(dict(object=str(object_path),function=m.group(1),bytes=v,kind=m.group(3)))
    dis=subprocess.run([tool('objdump'),'-drw',str(object_path)],capture_output=True,text=True,check=True).stdout
    (out/(object_path.stem+'.disassembly.txt')).write_text(dis)
    symbols=re.findall(r'^\s*[0-9a-f]+ <([^>]+)>:$',dis,re.M)
    demangled=subprocess.run([tool('c++filt')],input='\n'.join(symbols)+'\n',capture_output=True,text=True,check=True).stdout.splitlines()
    names=dict(zip(symbols,demangled));current=None
    for line in dis.splitlines():
        match=re.match(r'^\s*[0-9a-f]+ <([^>]+)>:$',line)
        if match:
            symbol=match.group(1);current=(object_path.name,symbol)
            key=name_key(names[symbol]);nodes[current]=dict(symbol=symbol,demangled=names[symbol],
                object=str(object_path),frame_bytes=stack.get(key),calls=[])
            continue
        if current is None:continue
        rel=re.search(r'R_ARM_THM_(CALL|JUMP24)\s+([^\s]+)',line)
        if rel:
            target=rel.group(2).split('+')[0]
            if target.startswith('.text.'):target=target[6:]
            nodes[current]['calls'].append(dict(symbol=target,tail=rel.group(1)=='JUMP24'))
        elif re.search(r'\bblx\s+r\d+',line):indirect.append(dict(caller=current,instruction=line))
# Resolve direct targets locally and include ambiguous definitions conservatively.
# Charge tail branches as nested calls; the result is a bound, not a measured high-water mark.
by_symbol={}
for key,n in nodes.items():by_symbol.setdefault(n['symbol'],[]).append(key)
for key,n in nodes.items():
    n['edges']=[]
    for call in n['calls']:
        targets=[(key[0],call['symbol'])] if (key[0],call['symbol']) in nodes else by_symbol.get(call['symbol'],[])
        if not targets:unresolved.append(dict(caller=key,callee=call['symbol'],tail=call['tail']))
        n['edges']+=targets
memo={};cycles=[]
def longest(key,active):
    if key in active:cycles.append(list(active)+[key]);return (0,[])
    if key in memo:return memo[key]
    n=nodes[key];own=n['frame_bytes']
    if own is None:own=0 # Mark the missing allocation as incomplete below.
    child=max((longest(x,active+[key]) for x in n['edges']),default=(0,[]),key=lambda p:p[0])
    result=(own+child[0],[key]+child[1]);memo[key]=result;return result
entries=[k for k in nodes if k[1] in ['gpenmpc_probe_local_store_prepare','gpenmpc_probe_local_store_install',
         'gpenmpc_probe_local_store_prediction','gpenmpcNative_canonicalLocalInnerFixedFirst',
         'gpenmpcNative_canonicalLocalInnerFixedStep','gpenmpcNative_queryCanonicalReferenceWindow',
         'gpenmpcNative_canonicalReferenceTransitionFromJet']]
paths=[]
for key in entries:
    bound,path=longest(key,[])
    paths.append(dict(entry=key[1],known_frame_path_sum=bound,
                      path=[dict(symbol=nodes[k]['demangled'],frame_bytes=nodes[k]['frame_bytes']) for k in path]))
report.update(direct_callgraph_nodes=len(nodes),paths=paths,unresolved_external_calls=unresolved,
              compiler_stack_kinds=stack_kinds,
              generated_c_receipt=str(arm/'RESULT.json'),generated_c_receipt_sha256=sha(arm/'RESULT.json'),
              unknown_frames=[list(k) for k,n in nodes.items() if n['frame_bytes'] is None],
              indirect_calls=indirect,cycles=cycles,
              analysis_method='Longest path over actual object THM_CALL/THM_JUMP24 relocations plus compiler .su frames; tail calls overcharged conservatively; unknown external frames are explicitly excluded',
              total_task_stack_upper_bound_proven=False,
              stack_limit_result='UNPROVEN: external libm/libc, actual task caller frames and interrupt/context usage not bounded here',
              not_measured_high_water=True)
report['graph']=[dict(key=list(k),**n) for k,n in nodes.items()]
(out/'RESULT.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps({k:report[k] for k in ['compiler_exit_code','source_stable','sizeof_symbols','paths',
                                    'unknown_frames','stack_limit_result']},indent=2))
