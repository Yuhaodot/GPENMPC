"""Compile the session entry and test its argument parser."""

# Component inputs are supplied by the invoking build/test environment.
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from component_inputs import component_input
component_input('GPENMPC_BUILD_ROOT')
component_input('GPENMPC_NUTTX_INCLUDE')
component_input('GPENMPC_TOPIC_HEADERS')

import hashlib,json,shlex,subprocess,sys
from pathlib import Path
import os
r=Path(__file__).resolve().parent;label=sys.argv[1];assert (label and all(c.isalnum() or c in '_-' for c in label))
out=r/('session_entry_checks_'+label);out.mkdir(exist_ok=False)
px4=Path(os.environ['GPENMPC_PX4_ROOT'])
build=component_input('GPENMPC_BUILD_ROOT');generated=r.parent.parent/'evidence/arm_controller/GPENMPC_Rfly_Canonical_Controller_ert_rtw'
sources=[r/n for n in ['CanonicalSessionEntry.cpp','CanonicalSessionMavlinkVisit.inc','compile_session_main_visit.cpp','test_session_entry_parse.cpp']]
before={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in sources}
db=json.loads((build/'compile_commands.json').read_text());records=[]
for source,base_name in [('CanonicalSessionEntry.cpp','mavlink_receiver.cpp'),('compile_session_main_visit.cpp','mavlink_main.cpp')]:
    entry,=[x for x in db if x['file']==str(px4/'src/modules/mavlink'/base_name)]
    args=shlex.split(entry['command']);obj=out/(Path(source).stem+'.o')
    args[args.index('-c')+1]=str(r/source);args[args.index('-o')+1]=str(obj)
    args[1:1]=['-I'+str(component_input('GPENMPC_TOPIC_HEADERS')),'-I'+str(generated),
        '-I'+str(px4/'src/modules/mavlink'),'-isystem',str(component_input('GPENMPC_NUTTX_INCLUDE')),
        '-fstack-usage','-MD','-MF',str(obj.with_suffix('.d'))]
    p=subprocess.run(args,cwd=entry['directory'],capture_output=True,text=True,timeout=120)
    rec=dict(source=source,argv=args,exit=p.returncode,output=p.stdout+p.stderr)
    if p.returncode==0:
        rec['stack_usage']=obj.with_suffix('.su').read_text()
        rec['undefined_symbols']=subprocess.check_output([args[0].replace('g++','nm'),'-u','-C',str(obj)],text=True)
        rec['object_sha256']=hashlib.sha256(obj.read_bytes()).hexdigest()
    records.append(rec);print(source,p.returncode,p.stdout,p.stderr,flush=True)
args=['g++','-std=gnu++14','-O2','-Wall','-Wextra','-Werror','-Wno-unused-parameter','-Wno-address-of-packed-member',
    '-ffunction-sections','-fdata-sections','-Wl,--gc-sections','-pthread','-D__PX4_POSIX','-D__PX4_LINUX','-DMODULE_NAME="gpenmpc_session_parser_test"',
    '-include',str(px4/'src/include/visibility.h'),'-include','px4_platform_common/defines.h',
    '-I'+str(component_input('GPENMPC_TOPIC_HEADERS')),'-I'+str(generated),
    '-I'+str(px4/'platforms/common'),'-I'+str(px4/'platforms/common/include'),'-I'+str(px4/'platforms/posix/include'),'-I'+str(px4/'platforms/posix/src/px4/generic/generic/include'),
    '-I'+str(px4/'platforms/posix/src/px4/common/include'),'-I'+str(px4/'boards/px4/sitl/src'),'-I'+str(px4/'src'),'-I'+str(px4/'src/include'),'-I'+str(px4/'src/lib'),
    '-I'+str(px4/'src/lib/matrix'),'-I'+str(px4/'src/modules/mavlink'),'-I'+str(px4/'src/modules'),'-I'+str(build),'-I'+str(build/'src/lib'),'-isystem',str(build/'mavlink/common'),'-isystem',str(build/'mavlink'),'-isystem',str(build/'mavlink/uAvionix'),
    str(r/'test_session_entry_parse.cpp'),str(r/'ExecutionSessionRegistration.cpp'),'-o',str(out/'test')]
p=subprocess.run(args,capture_output=True,text=True,timeout=120)
rec=dict(source='actual_entry_helper_test',argv=args,compile_exit=p.returncode,output=p.stdout+p.stderr)
if p.returncode==0:
    fixture=r.parent/'px4_wire/host_exchange_pump_feedback_final/PRODUCTION_IO_FEEDBACK_PACKETS.bin'
    run=subprocess.run([str(out/'test'),str(fixture)],capture_output=True,text=True,timeout=10)
    rec.update(exit=run.returncode,stdout=run.stdout,stderr=run.stderr)
    if run.returncode==0:
        # Verify the printed diagnostic.
        packet=fixture.read_bytes()[8:1120];expected={'ticket':packet[5:37].hex().upper()}
        for k in range(61):expected[f'actual61_bits[{k}]']=packet[496+k*8:504+k*8].hex().upper()
        for k in range(16):expected[f'control16_bits[{k}]']=packet[984+k*4:988+k*4].hex().upper()
        sha_names=['token.outer_payload_sha256','token.full_input_sha256','token.kernel_argument_sha256','token.kernel_source_sha256',
            'configuration_payload_sha256','approved_parameter_sha256','matlab_extraction_source_sha256','generated_arm_source_sha256','wrapper_matlab_source_sha256']
        for k,name in enumerate(sha_names):expected[name]=packet[208+k*32:240+k*32].hex().upper()
        token_names=['transaction','output_generation','sample_generation','timestamp_sample_us','source_generation_delta','state_publication_us',
            'state_board_rx_us','control_tick_us','sample_delta_us','actual_tick_delta_us','reference_generation','reference_timestamp_us',
            'reference_board_rx_us','reference_valid_until_us','outer_generation','outer_board_rx_us','outer_valid_until_us',
            'outer_based_on_sample_generation','outer_based_on_timestamp_sample_us']
        for k,name in enumerate(token_names):expected['token.'+name]=packet[56+k*8:64+k*8].hex().upper()
        expected['identity.uid']=packet[37:45].hex().upper();expected['identity.boot_generation']=packet[45:53].hex().upper()
        for name,offset in [('identity.system',53),('identity.component',54),('publication_path',55)]:expected[name]=f'{packet[offset]:016X}'
        for k,name in enumerate(['kernel_completed_us','publication_us','commit_completed_us']):expected[name]=packet[1048+k*8:1056+k*8].hex().upper()
        expected['original_valid_until_us']=f'{int.from_bytes(packet[1064:1072],"big")-1:016X}'
        lines=dict(x.split('=',1) for x in run.stdout.splitlines() if '=' in x)
        mismatches=[key for key,value in expected.items() if lines.get(key)!=value]
        rec.update(diagnostic_raw_fields_checked=len(expected),diagnostic_raw_field_mismatches=mismatches,
            late_fixture_modification='Revoked and original_valid_until_us=original_commit_completed_us-1; all actual61/control16/token/code bits inherited from prior actual production Io HOST fixture')
        if mismatches:rec['exit']=1
records.append(rec);print(rec,flush=True)
report=dict(records=records,passed=all(x.get('exit')==0 for x in records),source_hashes=before,
    sources_unchanged=before=={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in sources},
    actual_modulebase_executed=False,scope='ACTUAL_NUTTX_ARM_ENTRY_ORIGINAL_MAIN_PLUS_VISITOR_COMPILE_AND_HOST_HELPERS_ONLY')
(out/'RESULT.json').write_text(json.dumps(report,indent=2)+'\n')
sys.exit(0 if report['passed'] else 1)
