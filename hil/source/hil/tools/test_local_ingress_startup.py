"""One HOST transport fixture and three real M7 TUs; no graph/full link/device.

Run in existing Ubuntu WSL. Generated topic files stay in the new evidence
child, so the existing linked application/metadata are never overwritten.
"""
from pathlib import Path
from gpenmpc_external_path import external_path
import hashlib
import importlib.util
import json
import os
import re
import shlex
import shutil
import subprocess
import sys

BUILD = Path(__file__).resolve().parent.parent
VI = BUILD / 'rfly_vendor_integration'
APP = VI / 'application_integration/build_fmuv6c'
PX4 = Path(external_path('px4_source_root'))
label = sys.argv[1] if len(sys.argv) == 2 else 'local_ingress_startup_queue'
assert re.fullmatch(r'local_ingress_startup_queue(?:_[a-z][a-z0-9_]*)?', label)
OUT = BUILD / 'evidence/task_interface' / label
OUT.mkdir(exist_ok=False)
ENV = dict(os.environ, PYTHONDONTWRITEBYTECODE='1', CCACHE_DISABLE='1')
UORB = PX4 / 'platforms/common/uORB'
MSG = VI / 'application_integration/external/msg/GPENMPCFullInnerIngress.msg'
CPP = BUILD / 'tools/test_local_ingress_startup.cpp'
receiver = VI / 'application_integration/overlay/src/modules/mavlink/mavlink_receiver.cpp'
core = VI / 'px4_runtime/CanonicalLocalExchangeCore.cpp'
entry = VI / 'px4_runtime/CanonicalLocalSessionEntry.cpp'
tracked = [MSG, receiver, core, entry, CPP, Path(__file__),
           VI / 'px4_wire/CanonicalLocalWindowWire.cpp', VI / 'px4_wire/CanonicalLocalWindowAssembler.cpp',
           VI / 'px4_wire/CanonicalLocalIngress.hpp',
           BUILD / 'px4_full_inner/px4_ingress/GPENMPCFullInnerIngress.hpp',
           UORB / 'Subscription.cpp', UORB / 'Publication.hpp', UORB / 'uORBDeviceNode.hpp', UORB / 'uORBDeviceNode.cpp']
artifacts = [APP / ('px4_fmu-v6c_default.' + ext) for ext in ['elf', 'px4', 'bin', 'map']]
artifacts += [APP / 'uORB/topics/gpenmpc_full_inner_ingress.h', APP / 'msg/topics_sources/gpenmpc_full_inner_ingress.cpp']


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest().upper()


def hashes(paths):
    return {str(p): sha(p) for p in paths if p.is_file()}


report = dict(scope='INGRESS_STARTUP_QUEUE_HOST_AND_M7_TARGETED', commands=[],
              source_before=hashes(tracked), existing_artifacts_before=hashes(artifacts),
              queue_change=dict(old=8, new=16, category='PROJECT_ENGINEERING_TRANSPORT_CAPACITY',
                                reason='Existing prearm sender emits16 before a guaranteed board dequeue',
                                candidates_tested=[8, 16], ages_changed=False, numerical_changes=False),
              board=0, COM=0, MATLAB=0, full_build=False, firmware_link=False,
              generated_application_headers_overwritten=False)


def run(name, args, cwd=OUT):
    p = subprocess.run([str(x) for x in args], cwd=cwd, env=ENV, capture_output=True, text=True)
    (OUT / (name + '.log')).write_text(p.stdout + p.stderr)
    report['commands'].append(dict(name=name, command=[str(x) for x in args], exit=p.returncode,
                                   log=name + '.log'))
    print(name, p.returncode, flush=True)
    if p.returncode:
        print(p.stdout + p.stderr, flush=True)
    assert p.returncode == 0, name
    return p.stdout


def extract(source, signature, replacement=None):
    raw = source.read_text()
    start = raw.index(signature)
    opening = raw.index('{', start)
    depth = 1
    end = opening + 1
    while depth:
        if raw[end] == '{':
            depth += 1
        elif raw[end] == '}':
            depth -= 1
        end += 1
    text = raw[start:end]
    return text.replace(signature, replacement, 1) if replacement else text


try:
    # Generate the queue header with the SDK tool.
    gen = OUT / 'generated'
    stage = OUT / 'generation_stage'
    topics = gen / 'uORB/topics'
    topics.mkdir(parents=True)
    generator = PX4 / 'Tools/msg/px_generate_uorb_topic_files.py'
    template = PX4 / 'Tools/msg/templates/uorb'
    run('generate_header', ['/usr/bin/python3', generator, '--headers', '-f', MSG,
                            '-i', PX4 / 'msg', '-e', template, '-o', stage], PX4 / 'msg')
    shutil.copy2(stage / 'gpenmpc_full_inner_ingress.h', topics / 'gpenmpc_full_inner_ingress.h')
    # A single-file CLI assigns topic id0. Use the official generator's same
    # function with the ACTUAL complete topic table, preserving its real id.
    topic_table = APP / 'uORB/topics/uORBTopics.hpp'
    all_topics = re.findall(r'^\s*(\w+)\s*=\s*\d+,', topic_table.read_text(), re.MULTILINE)
    assert len(all_topics) > 200 and 'gpenmpc_full_inner_ingress' in all_topics
    sys.dont_write_bytecode = True
    sys.path.insert(0, str(generator.parent))
    spec = importlib.util.spec_from_file_location('original_px4_topic_generator', generator)
    official_generator = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(official_generator)
    assert official_generator.generate_output_from_file(1, str(MSG), str(stage), 'px4', str(template),
                                                        ['px4:' + str(PX4 / 'msg')], all_topics)
    report['official_source_generation'] = dict(generator_sha256=sha(generator), full_table_sha256=sha(topic_table),
                                                topic_count=len(all_topics), existing_id=all_topics.index('gpenmpc_full_inner_ingress'))
    assert 'ORB_QUEUE_LENGTH = 16' in (topics / 'gpenmpc_full_inner_ingress.h').read_text()
    assert re.search(r'ORB_DEFINE\(gpenmpc_full_inner_ingress,.*?, 16\);',
                     (stage / 'gpenmpc_full_inner_ingress.cpp').read_text())
    report['generated'] = hashes([topics / 'gpenmpc_full_inner_ingress.h', stage / 'gpenmpc_full_inner_ingress.cpp'])
    methods = {
        'actual_node_copy.inc': (UORB / 'uORBDeviceNode.hpp', 'bool copy(void *dst, unsigned &generation)', None),
        'actual_node_initial.inc': (UORB / 'uORBDeviceNode.cpp', 'unsigned uORB::DeviceNode::get_initial_generation()', 'unsigned get_initial_generation()'),
        'actual_node_range.inc': (UORB / 'uORBDeviceNode.hpp', 'static inline bool is_in_range(unsigned left, unsigned value, unsigned right)', None),
        'actual_publication_advertise.inc': (UORB / 'Publication.hpp', 'bool advertise()', None),
        'actual_subscription_subscribe.inc': (UORB / 'Subscription.cpp', 'bool Subscription::subscribe()', None),
    }
    report['original_method_bodies'] = {}
    for name, (source, signature, replacement) in methods.items():
        code = extract(source, signature, replacement)
        (OUT / name).write_text(code + '\n')
        report['original_method_bodies'][name] = dict(source=str(source), signature=signature,
                                                     enclosing_qualifier_only=replacement,
                                                     sha256=sha(OUT / name))
    # Verify the real call sites share the tested no-data startup ordering.
    constructor = extract(receiver, 'MavlinkReceiver::MavlinkReceiver(Mavlink &parent)')
    assert '_gpenmpc_full_inner_ingress_pub.advertise()' in constructor and '.publish(' not in constructor
    prepare = extract(entry, 'bool prepare_locked(Mavlink &actual, void *) noexcept')
    assert prepare.index('!ingress_node.valid()') < prepare.index('owner->prepare(')
    assert '!subscription_.valid()' in core.read_text()
    report['production_startup_guards'] = dict(receiver_constructor_empty_advertise=True,
                                              prepare_requires_existing_node=True,
                                              core_requires_existing_subscription=True)
    host = OUT / 'host_test'
    flags = ['c++', '-std=c++14', '-O2', '-Wall', '-Wextra', '-Werror',
             '-Wno-address-of-packed-member', '-I' + str(OUT), '-I' + str(gen),
             '-I' + str(VI / 'px4_wire/pump_host_stub'), '-I' + str(APP),
             '-isystem', str(APP / 'mavlink'), '-isystem', str(APP / 'mavlink/common')]
    run('host_compile', flags + [CPP, VI / 'px4_wire/CanonicalLocalWindowWire.cpp',
                                 VI / 'px4_wire/CanonicalLocalWindowAssembler.cpp', '-o', host])
    fixture = Path(external_path('canonical_combined_numerics')) / 'CONTINUOUS_REFERENCE_FIXTURE.bin'
    report['fixture_sha256'] = sha(fixture)
    output = run('host_run', [host, fixture])
    report['host'] = json.loads(output)
    assert report['host']['failed'] == 0
    database = json.loads((APP / 'compile_commands.json').read_text())
    report['m7_objects'] = []
    for source in [receiver, core, entry, stage / 'gpenmpc_full_inner_ingress.cpp']:
        basename = source.name
        if source.parent == stage:
            row, = [x for x in database if x['file'].endswith('/msg/topics_sources/' + basename)]
        else:
            row, = [x for x in database if Path(x['file']) == source]
        args = shlex.split(row['command'])
        if Path(args[0]).name == 'ccache':
            args = args[1:]
        assert '-mcpu=cortex-m7' in args and '-mfloat-abi=hard' in args and '-nostdinc++' in args
        obj = OUT / (basename + '.o')
        args[args.index('-o') + 1] = str(obj)
        args[args.index('-c') + 1] = str(source)
        if '-MF' in args:
            args[args.index('-MF') + 1] = str(obj.with_suffix('.d'))
        args[1:1] = ['-I' + str(gen), '-fstack-usage', '-MD', '-MF', str(obj.with_suffix('.d'))]
        run('m7_' + basename, args, Path(row['directory']))
        size_tool = Path(args[0]).with_name('arm-none-eabi-size')
        sizes = run('size_' + basename, [size_tool, obj])
        report['m7_objects'].append(dict(source=str(source), sha256=sha(obj), size=sizes,
                                         stack_usage=obj.with_suffix('.su').read_text()))
    report['passed'] = True
except Exception as error:
    report['passed'] = False
    report['error'] = repr(error)
finally:
    report['source_after'] = hashes(tracked)
    report['source_stable'] = report['source_before'] == report['source_after']
    report['existing_artifacts_after'] = hashes(artifacts)
    report['existing_artifacts_unchanged'] = report['existing_artifacts_before'] == report['existing_artifacts_after']
    report['passed'] = report.get('passed', False) and report['source_stable'] and report['existing_artifacts_unchanged']
    report['boundary'] = ('Actual PX4 method bodies and MAVLink parser/Receiver/ingress/window codec. HOST fixture '
                          'controls allocation, scheduling and locks in the HOST fixture. '
                          'Only affected M7 translation units plus generated metadata are compiled, no link. '
                          'Window and source/GP age bounds retain their configured values.')
    (OUT / 'RESULT.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps({k: report[k] for k in ['passed', 'source_stable', 'existing_artifacts_unchanged']}) , flush=True)
sys.exit(0 if report['passed'] else 1)
