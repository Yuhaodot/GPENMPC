"""Generate topics and test source readers with host publication fixtures."""

# Component inputs are supplied by the invoking build/test environment.
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from component_inputs import component_input
component_input('GPENMPC_BUILD_ROOT')

from pathlib import Path
import os
import hashlib,json,subprocess,sys
root=Path(__file__).resolve().parent
if len(sys.argv)!=2 or not Path(sys.argv[1]).is_absolute():raise ValueError('Provide an absolute result directory')
result_root=Path(sys.argv[1]).resolve()
if result_root.is_relative_to(root.parent.parent):raise ValueError('Result directory must be outside source inputs')
out=result_root;out.mkdir(exist_ok=True)
assert not (out/'RESULT.json').exists(), 'preserve previous result; select a distinct output explicitly'
px4=Path(os.environ['GPENMPC_PX4_ROOT'])
app=component_input('GPENMPC_BUILD_ROOT')
generated=out/'generated';headers=generated/'uORB/topics';headers.mkdir(parents=True,exist_ok=True)
files=[root/'Px4SelectedSourceReader.hpp',root/'Px4SelectedSourceReader.cpp',root/'test_selected_source_reader.cpp',root/'selected_host_stub/uORB/Subscription.hpp',
       root/'Px4OriginalHilReceiptReader.hpp',root/'Px4OriginalHilReceiptReader.cpp',root/'Px4OriginalHilReceipt.hpp',root/'GPENMPCOriginalHilReceipt.msg',
       root.parent.parent/'px4_full_inner/px4_state_adapter/AtomicOdometryAdapter.hpp',px4/'src/lib/parameters/param.h']
def hashes():return {str(p):hashlib.sha256(p.read_bytes()).hexdigest().upper() for p in files}
report=dict(scope='ACTUAL_READ_ONLY_SELECTED_SOURCE_READER_WITH_MOCK_UORB_AND_PARAM_API',source_sha256=hashes(),records=[],full_EKF_history_proven=False)
def run(name,args):
    p=subprocess.run([str(x) for x in args],capture_output=True,text=True)
    report['records'].append(dict(name=name,command=[str(x) for x in args],exit=p.returncode,output=p.stdout+p.stderr))
    print(name,p.returncode,p.stdout+p.stderr,flush=True);assert p.returncode==0
    return p
try:
    assert 'gyro_topic_instance' in (root/'GPENMPCOriginalHilReceipt.msg').read_text()
    run('official_topic_generator',['/usr/bin/python3',px4/'Tools/msg/px_generate_uorb_topic_files.py','--headers','-f',root/'GPENMPCOriginalHilReceipt.msg','-i',px4/'msg',px4/'msg/versioned','-o',headers,'-e',px4/'Tools/msg/templates/uorb'])
    args=['/usr/bin/g++','-std=c++14','-O2','-ffp-contract=off','-Wall','-Wextra','-Werror','-Wno-address-of-packed-member',
          '-I'+str(root/'selected_host_stub'),'-I'+str(root.parent/'px4_wire/pump_host_stub'),'-I'+str(generated),'-I'+str(component_input('GPENMPC_BUILD_ROOT')),
          '-I'+str(px4/'src/lib'),'-isystem',str(app/'mavlink'),root/'Px4SelectedSourceReader.cpp',root/'Px4OriginalHilReceiptReader.cpp',root/'test_selected_source_reader.cpp','-o',out/'test_selected_source']
    run('HOST_compile',args);p=run('HOST_test',[out/'test_selected_source']);report['HOST']=json.loads(p.stdout);report['pass']=True
except Exception as e:report.update(pass_=False,error=str(e))
report['source_sha256_after']=hashes();report['source_stable']=report['source_sha256']==report['source_sha256_after']
(out/'RESULT.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps({k:v for k,v in report.items() if k not in ['records','source_sha256','source_sha256_after']},indent=2))
sys.exit(0 if report.get('pass') and report['source_stable'] else 1)
