"""Incrementally build FMUv6C firmware and record commands and source hashes."""
from pathlib import Path
import hashlib,json,os,re,shlex,shutil,subprocess,sys,time
sys.dont_write_bytecode=True
import configure_private_nuttx as iso

root=Path(__file__).resolve().parent
build=iso.BUILD
attempt=sys.argv[1] if len(sys.argv)>1 else 'application'
assert re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_-]*',attempt)
suffix='_'+sys.argv[2] if len(sys.argv)>2 else ''
assert not suffix or re.fullmatch(r'_[A-Za-z0-9][A-Za-z0-9_-]*',suffix)
area=Path(os.environ.get('GPENMPC_LOCAL_RUNS',str(root/'local_runtime_integration')))
assert area.is_absolute()
area.mkdir(exist_ok=True)
out=area/attempt
if suffix:
    assert out.is_dir() and (out/'RESULT.json').is_file()
    assert not json.loads((out/'RESULT.json').read_text())['passed']
else:
    out.mkdir(exist_ok=False)
result_path=out/('RESULT'+suffix+'.json')
assert not result_path.exists(), 'Build receipt already exists'
db=json.loads((build/'compile_commands.json').read_text())
paths=[Path(r['file']) for r in db]
selected={p.name for p in paths}
required={'CanonicalLocalExecutionCycle.cpp','Px4CanonicalLocalIo.cpp','CanonicalLocalExchangeCore.cpp',
          'CanonicalLocalTaskInputOwner.cpp','RegisteredCanonicalLocalModuleContext.cpp','CanonicalLocalApplicationOwner.cpp',
          'CanonicalLocalSessionEntry.cpp','GPENMPCRflyCanonicalLocalModule.cpp','CanonicalFullInnerAbi.cpp'}
retired={'CanonicalSessionEntry.cpp','CanonicalApplicationOwner.cpp','CanonicalExchangePump.cpp',
         'Px4CanonicalIo.cpp','RegisteredCanonicalModuleContext.cpp','GPENMPCRflyCanonicalModule.cpp',
         'GPENMPC_Rfly_Canonical_Controller.c','GPENMPCTrajectoryExec.cpp','rtrpdc_cg.c'}
assert required<=selected and not (selected&retired), (required-selected,selected&retired)
cmd=shlex.split(next(r['command'] for r in db if Path(r['file']).name=='Px4CanonicalLocalIo.cpp'))
compiler=Path(cmd[1] if Path(cmd[0]).name=='ccache' else cmd[0])
assert compiler.name=='arm-none-eabi-g++'
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest().upper()
rec=lambda p:dict(path=str(p),bytes=p.stat().st_size,sha256=sha(p))
previous=root/'local_runtime_integration'/'prior_application'
if not previous.exists():
    previous.mkdir()
    for name in ('px4_fmu-v6c_default.elf','px4_fmu-v6c_default.bin','px4_fmu-v6c_default.px4',
                 'px4_fmu-v6c_default.map','parameters.json','parameters.xml'):
        p=build/name
        if p.exists():shutil.copy2(p,previous/name)
    (previous/'IDENTITIES.json').write_text(json.dumps([rec(p) for p in sorted(previous.iterdir()) if p.is_file()],indent=2)+'\n')
env=dict(os.environ,CCACHE_DISABLE='1',GIT_SUBMODULES_ARE_EVIL='1',GIT_OPTIONAL_LOCKS='0',
         PYTHONDONTWRITEBYTECODE='1',OMP_NUM_THREADS='1',MKL_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1',MAKEFLAGS='-j2')
env['PATH']=str(compiler.parent)+':'+env.get('PATH','')
track=set(p for p in paths if p.is_file() and 'rfly_vendor_integration/' in str(p))
track.update([root/'external/src/CMakeLists.txt',root/'external/src/gpenmpc_rfly_canonical/CMakeLists.txt',
              root/'external/src/gpenmpc_rfly_session/CMakeLists.txt'])
before={str(p):sha(p) for p in sorted(track)};anchors=iso.stat_snapshot(focused=True)
result=dict(scope='ACTUAL_LOCAL_FULL_INNER_FMUV6C_APPLICATION_LINK',passed=False,commands=[],
            new_selected_sources=sorted(required),retired_selected_sources=sorted(selected&retired))
def run(label,argv):
    log=out/(label+suffix+'.log');started=time.monotonic()
    with log.open('x') as f:
        p=subprocess.run(argv,stdout=f,stderr=subprocess.STDOUT,stdin=subprocess.DEVNULL,env=env)
    result['commands'].append(dict(label=label,argv=argv,exit=p.returncode,elapsed_s=time.monotonic()-started,log=rec(log)))
    print(label+' rc='+str(p.returncode),flush=True)
    if p.returncode:print(log.read_text()[-8500:],flush=True)
    assert p.returncode==0,label
    return log.read_text()
try:
    # Rebuild the private app registry when configure-generated bdat changes.
    apps=build/'NuttX/apps/libapps.a'
    registry=root/'private_source/platforms/nuttx/NuttX/apps/builtin'
    names=lambda s:set(re.findall(r'\{\s*"([^"\\]+)"\s*,',s))
    declared=(build/'NuttX/px4.bdat').read_text()
    installed=(registry/'builtin_list.h').read_text()
    new_names=names(declared)
    actual=names(installed)
    # A same-name STACK_MAIN change must reach the actual NSH registry too.
    session_entry=lambda s:re.search(r'^\s*\{\s*"gpenmpc_rfly_session"[^\n]+',s,re.M).group(0).strip()
    session_changed=session_entry(declared)!=session_entry(installed)
    obsolete_names={'gpenmpc_rfly_canonical','gpenmpc_trajectory_exec'}
    if not new_names<=actual or (obsolete_names&actual) or session_changed:
        assert apps.resolve().is_relative_to(build.resolve())
        if apps.exists():os.replace(apps,out/'prior_libapps.a')
        run('APPS',['cmake','--build',str(build),'--parallel','2','--target','nuttx_apps_build'])
    actual=names((registry/'builtin_list.h').read_text())
    assert new_names<=actual and not (obsolete_names&actual)
    assert session_entry(declared)==session_entry((registry/'builtin_list.h').read_text())
    result['builtin_names']=sorted(n for n in actual if 'gpenmpc' in n)
    log=run('APPLICATION',['cmake','--build',str(build),'--parallel','2','--target','px4'])
    elf=build/'px4_fmu-v6c_default.elf'
    text=run('SYMBOLS',[str(compiler.with_name('arm-none-eabi-nm')),'-C','--defined-only',str(elf)])
    symbols=('gpenmpc_rfly_canonical_local_main','gpenmpc_rfly_session_main','gpenmpc_full_inner_prepare_numeric',
             'gpenmpc_full_inner_fill_gp','gpenmpc_full_inner_commit','gpenmpc_full_inner_copy_closed_evidence')
    result['linked_checks']={n:len(re.findall(r'\b'+re.escape(n)+r'\s*$',text,re.M))==1 for n in symbols}
    for old in ('gpenmpc_rfly_canonical_main','gpenmpc_se3_control_main','gpenmpc_tunnel_bridge_main','gpenmpc_trajectory_exec_main','rtrpdc_cg_step'):
        result['linked_checks']['absent_'+old]=not re.search(r'\b'+old+r'\s*$',text,re.M)
    result['linked_checks']['local_cycle_linked']='CanonicalLocalExecutionCycle::execute' in text
    result['linked_checks']['no_old_rpc_pump']='CanonicalExchangePump::' not in text
    result['linked_checks']['no_old_controller_step']='GPENMPC_Rfly_Canonical_Controller_step' not in text
    result['linked_checks']['actual_learning_kernel_linked']='gpenmpcNative_canonicalLocalInnerWithAuditStep' in text
    result['artifacts']=[rec(p) for p in (elf,build/'px4_fmu-v6c_default.map',build/'parameters.json')]
    result['memory_report']='\n'.join(l for l in log.splitlines() if any(k in l for k in ('FLASH:','AXI_SRAM:','Memory region')))
    assert all(result['linked_checks'].values())
    result['passed']=True
except Exception as e:
    result['first_exception']=type(e).__name__+': '+str(e)
finally:
    result['sources_before']=before
    result['sources_stable']=all(Path(p).is_file() and sha(Path(p))==h for p,h in before.items())
    result['original_source_anchors_unchanged']=anchors==iso.stat_snapshot(focused=True)
    result['passed']=result['passed'] and result['sources_stable'] and result['original_source_anchors_unchanged']
    with result_path.open('x') as stream:
        stream.write(json.dumps(result,indent=2)+'\n')
    print(json.dumps({k:result.get(k) for k in ('passed','first_exception','memory_report','sources_stable','original_source_anchors_unchanged')}),flush=True)
sys.exit(0 if result['passed'] else 1)
