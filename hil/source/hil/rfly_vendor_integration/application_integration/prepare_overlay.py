#!/usr/bin/env python3
"""Prepare the integration source overlays."""
from pathlib import Path
import os
import hashlib
import json
import re
import subprocess

ROOT = Path(__file__).resolve().parent
RFLY = ROOT.parent
BUILD = RFLY.parent
PX4 = Path(os.environ['GPENMPC_PX4_ROOT'])
INGRESS = BUILD / 'px4_full_inner/px4_ingress'
PATCH_ROOT = RFLY / 'official_io_nuttx/integration_patch'
META = PATCH_ROOT / 'generated_all'
GENERATED = BUILD / 'evidence/arm_controller/GPENMPC_Rfly_Canonical_Controller_ert_rtw'
OUT = ROOT / 'overlay'
RECEIPT = ROOT / 'RESULT.json'


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest().upper()


def main():
    if OUT.exists() or RECEIPT.exists():
        raise RuntimeError('Prepared output exists; preserve it and inspect explicitly')
    version = subprocess.run(['cmake', '--version'], check=True, text=True,
                             capture_output=True).stdout
    match = re.search(r'cmake version (\d+)\.(\d+)\.(\d+)', version)
    assert match and tuple(map(int, match.groups())) >= (3, 19, 0)
    assert json.loads((META / 'RESULT.json').read_text())['passed']
    table = (META / 'uORB/topics/uORBTopics.hpp').read_text()
    assert re.search(r'ORB_TOPICS_COUNT\{311\}', table)
    source_records = []

    def copy(source, target, role, content=None):
        data = source.read_bytes()
        source_records.append({'source': str(source), 'sha256': sha(source),
                               'bytes': len(data), 'role': role})
        target.parent.mkdir(parents=True, exist_ok=True)
        if target.exists():
            raise FileExistsError(str(target))
        target.write_bytes(data if content is None else content)
        return data

    # Keep Mavlink and its embedded receiver on the same source layout.
    original_mav = PX4 / 'src/modules/mavlink'
    mav = OUT / 'src/modules/mavlink'
    existing_receiver = INGRESS / 'receiver_tu_overlay/src/modules/mavlink'
    for source in sorted(original_mav.iterdir()):
        if source.is_file() and source.suffix in ('.c', '.cpp', '.h', '.hpp'):
            data = source.read_bytes()
            role = 'byte_identical_mavlink_layout_closure'
            if source.name in ('mavlink_receiver.cpp', 'mavlink_receiver.h'):
                patched = existing_receiver / source.name
                data = patched.read_bytes()
                role = 'receiver_patch'
                source_records.append({'source': str(patched), 'sha256': sha(patched),
                                       'bytes': len(data), 'role': role})
            elif source.name == 'mavlink_messages.cpp':
                before = b'#include "streams/HIL_ACTUATOR_CONTROLS.hpp"'
                assert data.count(before) == 1
                data = data.replace(before, b'#include "GPENMPCRflyHilStream.hpp"')
                before = (b'#if defined(HIL_ACTUATOR_CONTROLS_HPP)\n'
                          b'\tcreate_stream_list_item<MavlinkStreamHILActuatorControls>(),\n'
                          b'#endif // HIL_ACTUATOR_CONTROLS_HPP')
                assert data.count(before) == 1
                data = data.replace(before, (b'#if defined(GPENMPC_RFLY_HIL_STREAM_HPP)\n'
                    b'\tcreate_stream_list_item<MavlinkStreamGPENMPCRflyHILActuatorControls>(),\n'
                    b'#endif // GPENMPC_RFLY_HIL_STREAM_HPP'))
                role = 'exact_existing_registry_patch'
            copy(source, mav / source.name, role, data)
    for source in sorted((original_mav / 'streams').glob('*.hpp')):
        copy(source, mav / 'streams' / source.name, 'byte_identical_stream_include_closure')
    # mavlink_ftp.cpp includes this declaration even in non-unit-test builds.
    # Its relative main/receiver includes must share the same private layout.
    copy(original_mav / 'mavlink_tests/mavlink_ftp_test.h',
         mav / 'mavlink_tests/mavlink_ftp_test.h',
         'byte_identical_required_ftp_declaration')

    # Apply the receiver message aliases.
    for name in ('ActuatorArmed.msg', 'ActuatorOutputs.msg'):
        original = PX4 / 'msg' / name
        alias = PATCH_ROOT / 'msg' / name
        assert [x for x in original.read_text().splitlines() if not x.startswith('# TOPICS')] == [
            x for x in alias.read_text().splitlines() if not x.startswith('# TOPICS')]
        copy(alias, ROOT / 'external/msg' / name, 'exact_311_alias_schema')
    copy(INGRESS / 'GPENMPCFullInnerIngress.msg', ROOT / 'external/msg/GPENMPCFullInnerIngress.msg',
         'exact_311_ingress_schema')
    for name, original in [('px4io_readonly.cpp', 'src/drivers/px4io/px4io.cpp'),
                           ('DShot_readonly.cpp', 'src/drivers/dshot/DShot.cpp')]:
        source = RFLY / 'px4_runtime/output_guard' / name
        assert source.read_bytes().startswith((PX4 / original).read_bytes())
        copy(source, OUT / 'drivers' / name, 'actual_driver_plus_readonly_query')

    paths = {'GPENMPC_RFLY': RFLY, 'GPENMPC_RUNTIME': RFLY / 'px4_runtime',
             'GPENMPC_INGRESS': INGRESS, 'GPENMPC_GENERATED': GENERATED,
             'GPENMPC_MAVLINK_OVERLAY': mav, 'GPENMPC_DRIVER_OVERLAY': OUT / 'drivers'}
    (ROOT / 'PreparedPaths.cmake').write_text('\n'.join(
        'set(' + key + ' "' + str(value) + '")' for key, value in paths.items()) + '\n')
    init = PX4 / 'platforms/nuttx/cmake/init.cmake'
    text = init.read_text()
    required = ['${NUTTX_SRC_DIR}/Make.defs.in ${NUTTX_DIR}/Make.defs',
                '${NUTTX_DEFCONFIG} ${NUTTX_DIR}/.config',
                '${NUTTX_DEFCONFIG} ${NUTTX_DIR}/defconfig',
                'COMMAND ${NUTTX_SRC_DIR}/tools/px4_nuttx_make_olddefconfig.sh']
    assert all(s in text for s in required)
    checks = ['cmake_3_19_or_newer', 'actual_311_generated_metadata_passed',
              'mavlink_full_top_level_layout_closure', 'registry_exact_patch',
              'receiver_patch_reused', 'aliases_fields_unchanged',
              'exact_ingress_message_reused', 'driver_original_prefix_preserved']
    assert all(sha(Path(row['source'])) == row['sha256'] for row in source_records)
    files = [p for p in ROOT.rglob('*') if p.is_file()]
    report = {
        'status': 'OVERLAY_PREPARED',
        'cmake_version': version.strip(), 'checks_passed': len(checks), 'checks': checks,
        'full_px4_configure_attempted': False, 'full_px4_configure_passed': False,
        'firmware_linked': False, 'module_main_implemented': False,
        'scheduler_implemented': False, 'live_authority_granted': False,
        'px4_executed': False,  'old_source_writes': 0,
        'old_build_writes': 0, 'source_records': source_records,
        'source_hashes_unchanged': True,
        'configure_blocker': {'path': str(init), 'sha256': sha(init),
            'unconditional_source_mutations': required,
            'additional_directory_write': '${NUTTX_CONFIG_DIR}/src',
            'git_update_can_be_disabled_with': 'GIT_SUBMODULES_ARE_EVIL=1',
            'that_variable_does_not_prevent_nuttx_source_writes': True},
        'prepared_files': [{'path': str(p.relative_to(ROOT)), 'bytes': p.stat().st_size,
                            'sha256': sha(p)} for p in sorted(files)],
        'build_directory_requirement': 'Use an isolated NuttX/apps tree for configuration and compilation.'}
    RECEIPT.write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps({k: report[k] for k in ('status', 'checks_passed',
        'full_px4_configure_attempted', 'old_source_writes')}))


if __name__ == '__main__':
    main()
