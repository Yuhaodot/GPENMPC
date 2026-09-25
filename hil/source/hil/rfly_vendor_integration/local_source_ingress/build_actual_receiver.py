"""Build the uORB and MAVLink receiver libraries."""

# Component inputs are supplied by the invoking build/test environment.
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from component_inputs import component_input
component_input('GPENMPC_BUILD_ROOT')

from pathlib import Path
import hashlib,json,os,re,subprocess,sys

root=Path(__file__).resolve().parent
label=sys.argv[1];assert re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_-]*',label)
out=root/label;out.mkdir(exist_ok=False)
integration=root.parent/'application_integration';build=component_input('GPENMPC_BUILD_ROOT')
px4=Path(os.environ['GPENMPC_PX4_ROOT'])
receiver=integration/'overlay/src/modules/mavlink/mavlink_receiver.cpp'
tracked=[receiver,receiver.with_suffix('.h'),root/'Px4OriginalHilReceipt.hpp',root/'GPENMPCOriginalHilReceipt.msg',
         root.parent/'clock_tap_overlay/ClockObservationTap.hpp',integration/'external/msg/CMakeLists.txt',
         integration/'external/src/CMakeLists.txt',integration/'PreparedPaths.cmake']
artifacts=[build/('px4_fmu-v6c_default.'+ext) for ext in ['elf','px4','bin','map']]
def digest(p):return hashlib.sha256(p.read_bytes()).hexdigest().upper()
def records(paths):return {str(p):dict(sha256=digest(p),bytes=p.stat().st_size) for p in paths if p.exists()}
report=dict(scope='ACTUAL_EXISTING_GRAPH_UORB_HEADERS_AND_MODULES_MAVLINK_LIBRARY_ONLY',
            firmware_link_requested=False,PX4_executed=False,commands=[])
report['source_before']=records(tracked);report['firmware_before']=records(artifacts)
table=build/'uORB/topics/uORBTopics.hpp'
def ids(p):return {n:int(v) for n,v in re.findall(r'^\s*(\w+)\s*=\s*(\d+),',p.read_text(),re.MULTILINE)}
report['topics_before']=ids(table)
start=b'void\nMavlinkReceiver::handle_message_hil_sensor(mavlink_message_t *msg)'
end=b'\nvoid\nMavlinkReceiver::handle_message_hil_gps'
def body(p):
    raw=p.read_bytes().replace(b'\r\n',b'\n');a=raw.index(start);b=raw.index(end,a);return raw[a:b]
baseline=body(px4/'src/modules/mavlink/mavlink_receiver.cpp');actual=body(receiver);stripped=actual
for line in [b'\tbool original_gyro_update_called = false;\n',b'\tbool original_accel_update_called = false;\n',
             b'\t\t\toriginal_gyro_update_called = true;\n',b'\t\t\toriginal_accel_update_called = true;\n']:
    assert stripped.count(line)==1;stripped=stripped.replace(line,b'',1)
observer=b'''\n\t// Preserve the existing receiver timestamp and payload. No second HRT read,
\t// wire-clock substitution, sensor change or control action. Failure of this
\t// observer never prevents the original PX4 sensor updates above.
\t(void)_gpenmpc_original_hil_receipt.record(*msg, hil_sensor, timestamp, this, &_mavlink,
\t\tstatic_cast<int32_t>(_mavlink.get_instance_id()), static_cast<int32_t>(_mavlink.get_channel()),
\t\t_MAV_PAYLOAD(msg), original_gyro_update_called, original_accel_update_called,
\t\toriginal_gyro_update_called ? static_cast<int16_t>(_px4_gyro->get_instance()) : static_cast<int16_t>(-1),
\t\toriginal_accel_update_called ? static_cast<int16_t>(_px4_accel->get_instance()) : static_cast<int16_t>(-1),
\t\toriginal_gyro_update_called ? _px4_gyro->get_device_id() : 0u,
\t\toriginal_accel_update_called ? _px4_accel->get_device_id() : 0u);
'''
assert stripped.count(observer)==1;stripped=stripped.replace(observer,b'',1)
for name,value in [('HIL_BASELINE.txt',baseline),('HIL_HOOKED.txt',actual),('HIL_REMOVE_OBSERVER.txt',stripped)]:
    (out/name).write_bytes(value)
report['function_proof']=dict(original_body_equal_after_removing_only_observer_lines=baseline==stripped,
    baseline_sha256=hashlib.sha256(baseline).hexdigest().upper(),
    stripped_sha256=hashlib.sha256(stripped).hexdigest().upper(),
    original_timestamp_call_count=baseline.count(b'const uint64_t timestamp = hrt_absolute_time();'),
    timestamp_call_count_after_hook=actual.count(b'const uint64_t timestamp = hrt_absolute_time();'),
    observer_return_discarded=True,observer_after_original_sensor_and_battery_updates=True,
    line_endings_normalized_only=True)
assert baseline==stripped,'Unexpected original HIL function change'
(out/'BEFORE.json').write_text(json.dumps(report,indent=2)+'\n')
def run(target):
    command=['ninja','-C',str(build),'-j','2',target]
    with (out/(target+'.log')).open('w') as log:
        p=subprocess.Popen(command,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,
                           env=dict(os.environ,CCACHE_DISABLE='1'))
        for line in p.stdout:
            log.write(line);log.flush();print(line,end='',flush=True)
        rc=p.wait()
    report['commands'].append(dict(command=command,exit=rc,log=target+'.log'))
    return rc
try:
    rc=run('uorb_headers');assert rc==0,'uorb_headers failed'
    report['topics_after']=ids(table)
    delta=set(report['topics_after'])-set(report['topics_before'])
    if 'gpenmpc_original_hil_receipt' in report['topics_before']:
        assert report['topics_before']==report['topics_after'],'Existing complete topic table changed'
    else:
        assert delta=={'gpenmpc_original_hil_receipt'},str(delta)
    report['new_topic_id']=report['topics_after']['gpenmpc_original_hil_receipt']
    report['full_table_count']=len(report['topics_after'])
    report['full_generated_header']=str(build/'uORB/topics/gpenmpc_original_hil_receipt.h')
    report['full_generated_header_sha256']=digest(Path(report['full_generated_header']))
    print('ACTUAL_FULL_TOPIC_HEADERS_READY '+report['full_generated_header'],flush=True)
    (out/'HEADERS_READY.json').write_text(json.dumps(report,indent=2)+'\n')
    rc=run('modules__mavlink');assert rc==0,'modules__mavlink failed'
    library=build/'src/modules/mavlink/libmodules__mavlink.a'
    report['actual_mavlink_library']=records([library]);report['pass']=True
except Exception as e:
    report['pass']=False;report['error']=str(e)
report['source_after']=records(tracked);report['source_stable']=report['source_before']==report['source_after']
report['firmware_after']=records(artifacts);report['prior_firmware_unchanged']=report['firmware_before']==report['firmware_after']
(out/'RESULT.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps({k:report[k] for k in ['pass','source_stable','prior_firmware_unchanged']},indent=2),flush=True)
sys.exit(0 if report.get('pass') and report['source_stable'] and report['prior_firmware_unchanged'] else 1)
