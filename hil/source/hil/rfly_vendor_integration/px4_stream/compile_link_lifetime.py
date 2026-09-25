#!/usr/bin/env python3
"""Compile the Mavlink lifetime hook and NuttX integration objects."""

# Component inputs are supplied by the invoking build/test environment.
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from component_inputs import component_input
component_input('GPENMPC_BUILD_ROOT')
component_input('GPENMPC_NUTTX_INCLUDE')
component_input('GPENMPC_TOPIC_HEADERS')

import difflib
import hashlib
import json
from pathlib import Path
import os
import shlex
import subprocess
import sys

ROOT=Path(__file__).resolve().parent
PX4=Path(os.environ['GPENMPC_PX4_ROOT'])
BUILD=ROOT.parent.parent
META=component_input('GPENMPC_TOPIC_HEADERS')
DB=component_input('GPENMPC_BUILD_ROOT') / 'compile_commands.json'
label=sys.argv[1] if len(sys.argv)==2 else 'verification'
assert (label and all(c.isalnum() or c in '_-' for c in label))
OUT=ROOT/('link_lifetime_arm_'+label)
OUT.mkdir(exist_ok=False)
original=PX4/'src/modules/mavlink/mavlink_main.cpp'
inputs=[original,ROOT/'LinkLifetimeRegistry.hpp',ROOT/'LinkLifetimeRegistry.cpp',
        ROOT.parent/'px4_runtime/Px4ReadOnlyGuard.hpp',ROOT.parent/'px4_runtime/Px4ReadOnlyGuard.cpp',
        ROOT.parent/'px4_runtime/InheritingMutex.hpp',DB,Path(__file__)]
def hashes():return {str(path):hashlib.sha256(path.read_bytes()).hexdigest() for path in inputs}
before=hashes()
report={'passed':False,'scope':'REAL_NUTTX_SCOPED_LINK_BORROW_AND_ISOLATED_MAVLINK_DERIVATIVE_COMPILE_ONLY',
        'source_hashes_before':before,'target_code_executed':False,'firmware_linked':False,
        'original_tree_modified':False,'records':[]}
try:
    source=original.read_text()
    def replace_once(old,new):
        global source
        assert source.count(old)==1,old
        source=source.replace(old,new,1)
    replace_once('#include "mavlink_main.h"\n','#include "mavlink_main.h"\n#include "LinkLifetimeRegistry.hpp"\n')
    replace_once('bool Mavlink::_boot_complete = false;\n',r'''bool Mavlink::_boot_complete = false;

// Project-only checked destruction boundary. Callback scope reads immutable
// link facts only; it never waits for ModuleBase or acquires another lock.
static bool gpenmpc_mavlink_retire_for_delete(Mavlink *instance)
{
    if (!instance) { return true; }
    auto *registry = gpenmpc_rfly_stream::link_lifetime_registry();
    if (!registry) { return false; }
    const auto result = registry->retire_instance(instance);
    return result == gpenmpc_rfly_stream::LinkRetirement::Quiesced ||
           result == gpenmpc_rfly_stream::LinkRetirement::NeverRegistered;
}
''')
    replace_once('\t_telemetry_status_pub.advertise();\n}',r'''	_telemetry_status_pub.advertise();

    // Constructor registration is not activation: task_main still initializes
    // _is_usb_uart. Only its final-property activation permits guard borrowing.
    auto *registry = gpenmpc_rfly_stream::link_lifetime_registry();
    gpenmpc_rfly_stream::LinkToken token{};
    if (!registry || registry->register_constructed(this, token) != gpenmpc_rfly_stream::LinkRegistration::Registered) {
        PX4_ERR("GPENMPC link registration unavailable");
    }
}'''.replace('\\t','\t'))
    replace_once('Mavlink::~Mavlink()\n{',r'''Mavlink::~Mavlink()
{
    // Recheck retirement after the checked delete boundary.
    if (!gpenmpc_mavlink_retire_for_delete(this)) {
        PX4_ERR("GPENMPC retirement incomplete");
    }''')
    replace_once('\t_task_running.store(true);',r'''    // _is_usb_uart's sole task_main assignment precedes this point. Publishing
    // Live under the borrow mutex synchronizes all later immutable reads.
    auto *gpenmpc_link_registry = gpenmpc_rfly_stream::link_lifetime_registry();
    if (!gpenmpc_link_registry || gpenmpc_link_registry->activate(this) != gpenmpc_rfly_stream::LinkAccess::Read) {
        PX4_ERR("GPENMPC link activation unavailable");
    }

	_task_running.store(true);'''.replace('\\t','\t'))
    replace_once('\t\t\tdelete inst_to_del;',r'''            if (!gpenmpc_mavlink_retire_for_delete(inst_to_del)) {
                PX4_ERR("GPENMPC lifetime unproven; preserving instance");
                return PX4_ERROR;
            }
			delete inst_to_del;'''.replace('\\t','\t'))
    replace_once('\t\tif (res != PX4_OK) {\n\t\t\tdelete instance;\n\t\t}',r'''		if (res != PX4_OK) {
            // A failed startup may not be in the global instance table. Keep
            // its sole local pointer/task alive on Unproven; do not leak the
            // only reference or enter the destructor's original forced stop.
            if (!gpenmpc_mavlink_retire_for_delete(instance)) {
                PX4_ERR("GPENMPC startup instance quarantined until borrow retirement is proven");
                while (!gpenmpc_mavlink_retire_for_delete(instance)) {
                    px4_usleep(MAIN_LOOP_DELAY);
                }
            }
			delete instance;
		}'''.replace('\\t','\t'))
    replace_once('\t\t\t\t\t\tdelete inst;',r'''                        if (!gpenmpc_mavlink_retire_for_delete(inst)) {
                            PX4_ERR("GPENMPC lifetime unproven; preserving instance");
                            return PX4_ERROR;
                        }
						delete inst;'''.replace('\\t','\t'))
    derived=OUT/'mavlink_main_lifetime.cpp'
    derived.write_text(source)
    patch=OUT/'Mavlink_link_lifetime.patch'
    patch.write_text(''.join(difflib.unified_diff(original.read_text().splitlines(True),source.splitlines(True),
                     fromfile='a/src/modules/mavlink/mavlink_main.cpp',tofile='b/src/modules/mavlink/mavlink_main.cpp')))
    checked=subprocess.run(['git','-C',str(PX4),'apply','--check',str(patch)],capture_output=True,text=True)
    report['patch_check']={'argv':['git','-C',str(PX4),'apply','--check',str(patch)],'exit_code':checked.returncode,'stderr':checked.stderr}
    assert checked.returncode==0,checked.stderr
    report['isolated_mavlink_patch']=str(patch)
    report['covered_deletes']=['Mavlink::destroy_all_instances delete inst_to_del','Mavlink::stop delete inst',
                               'Mavlink::start_helper startup-error delete instance']
    report['lifetime_semantics']=['Constructed is not readable; activate follows sole _is_usb_uart assignment.',
        'Guard holds only a token and reads USB property inside the PI-mutex callback.',
        'All three actual delete expressions are preceded by checked retirement; Unproven does not enter destructor.',
        'Unproven startup error retains the instance and quarantines its task at MAIN_LOOP_DELAY.',
        'Healthy exhaustive table absence proves no current registered borrow, not historically never constructed.',
        'Generation identifies the process-local object lifetime; guards validate boot identity and physical isolation.',
        'Compile only: no actual Mavlink start/stop, NuttX task death or delete path was executed.']
    entries=json.loads(DB.read_text())
    for local,original_name in [(ROOT/'LinkLifetimeRegistry.cpp','mavlink_messages.cpp'),
                               (ROOT.parent/'px4_runtime/Px4ReadOnlyGuard.cpp','mavlink_messages.cpp'),
                               (derived,'mavlink_main.cpp')]:
        entry,=[row for row in entries if row['file']==str(PX4/'src/modules/mavlink'/original_name)]
        args=shlex.split(entry['command']);obj=OUT/(local.name+'.obj');dep=OUT/(local.name+'.d')
        args[args.index('-c')+1]=str(local);args[args.index('-o')+1]=str(obj)
        args[1:1]=['-I'+str(ROOT),'-I'+str(META),'-I'+str(ROOT.parent/'official_io_nuttx/overlay'),
            '-I'+str(PX4/'src/modules/mavlink'),'-isystem',str(component_input('GPENMPC_NUTTX_INCLUDE')),
            '-MD','-MF',str(dep),'-fstack-usage']
        result=subprocess.run(args,cwd=entry['directory'],capture_output=True,text=True)
        (OUT/(local.name+'.log')).write_text(result.stdout+result.stderr)
        record={'source':str(local),'original_compile_source':entry['file'],'argv':args,'exit_code':result.returncode}
        report['records'].append(record);print(local.name,result.returncode,flush=True)
        assert result.returncode==0,result.stdout+result.stderr
        record.update(object=str(obj),object_sha256=hashlib.sha256(obj.read_bytes()).hexdigest())
        for tool,flags in [('nm',['-C']),('readelf',['-h','-A'])]:
            output=subprocess.run([args[0].replace('g++',tool),*flags,str(obj)],capture_output=True,text=True,check=True).stdout
            (OUT/(local.name+'.'+tool)).write_text(output)
        dependencies=dep.read_text()
        assert str(component_input('GPENMPC_BUILD_ROOT') / 'uORB/topics') not in dependencies
        assert str(Path(os.environ['GPENMPC_BASELINE_TOPIC_HEADERS'])) not in dependencies
        if local.name!='LinkLifetimeRegistry.cpp':
            assert str(META/'uORB/topics/uORBTopics.hpp') in dependencies
    after=hashes();report.update(source_hashes_after=after,sources_unchanged=before==after,passed=before==after)
except Exception as exc:
    report['failure']=str(exc)
finally:
    (OUT/'RESULT.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({key:report.get(key) for key in ('passed','sources_unchanged','failure')}))
raise SystemExit(0 if report['passed'] else 1)
