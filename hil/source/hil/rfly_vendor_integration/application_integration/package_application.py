#!/usr/bin/env python3
"""Package the linked FMUv6C application."""
import base64
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time
import zlib

sys.dont_write_bytecode = True
import configure_private_nuttx as isolation

ROOT = Path(__file__).resolve().parent
BUILD = isolation.BUILD
PX4 = isolation.PX4
ELF = BUILD / 'px4_fmu-v6c_default.elf'
BIN = BUILD / 'px4_fmu-v6c_default.bin'
PACKAGE = BUILD / 'px4_fmu-v6c_default.px4'
RECEIPT = ROOT / 'application_package.json'
LOG = ROOT / 'application_package.log'
LINK_RECEIPT = Path(os.environ['GPENMPC_LINK_RECEIPT'])
RECOVERY = Path(os.environ['GPENMPC_RECOVERY_IMAGE'])
EXPECTED_ELF = os.environ['GPENMPC_ELF_SHA256'].upper()
EXPECTED_RECOVERY = os.environ['GPENMPC_RECOVERY_SHA256'].upper()
EXPECTED_RECOVERY_BYTES = int(os.environ['GPENMPC_RECOVERY_BYTES'])
TOOLS = Path(os.environ['GPENMPC_ARM_TOOLCHAIN']) / 'bin'


def record(path):
    return {'path': str(path), 'bytes': path.stat().st_size,
            'sha256': isolation.sha(path)}


def main():
    assert not RECEIPT.exists(), 'Preserve prior packaging receipt; inspect instead of rerun'
    assert isolation.sha(ELF) == EXPECTED_ELF, 'ELF identity changed'
    assert RECOVERY.stat().st_size == EXPECTED_RECOVERY_BYTES
    assert isolation.sha(RECOVERY) == EXPECTED_RECOVERY
    protected_paths = [RECOVERY, ELF,
        PX4 / 'boards/px4/fmu-v6c/extras/px4_fmu-v6c_bootloader.bin',
        ROOT / 'private_source/boards/px4/fmu-v6c/extras/px4_fmu-v6c_bootloader.bin',
        PX4 / 'boards/px4/fmu-v6c/extras/px4_io-v2_default.bin']
    protected_before = {str(p): record(p) for p in protected_paths}
    env = dict(os.environ, GIT_SUBMODULES_ARE_EVIL='1', GIT_OPTIONAL_LOCKS='0',
               PYTHONDONTWRITEBYTECODE='1', CCACHE_DISABLE='1',
               OMP_NUM_THREADS='1', OPENBLAS_NUM_THREADS='1', MKL_NUM_THREADS='1',
               CMAKE_BUILD_PARALLEL_LEVEL='2')
    env['PATH'] = str(TOOLS) + ':' + env.get('PATH', '')
    command = ['cmake', '--build', str(BUILD), '--parallel', '2',
               '--target', 'px4_package']
    reused = PACKAGE.exists()
    started = time.time()
    if not reused:
        # Build the PX4 application package from the linked ELF.
        with LOG.open('x') as stream:
            run = subprocess.run(command, env=env, stdin=subprocess.DEVNULL,
                                 stdout=stream, stderr=subprocess.STDOUT)
        assert run.returncode == 0, 'Official package target failed; raw log retained'
    assert PACKAGE.is_file() and BIN.is_file()
    descriptor = json.loads(PACKAGE.read_text())
    image = zlib.decompress(base64.b64decode(descriptor['image'], validate=True))
    actual_bin = BIN.read_bytes()
    independently_extracted = subprocess.run(
        [str(TOOLS / 'arm-none-eabi-objcopy'), '-O', 'binary', str(ELF), '/dev/stdout'],
        stdin=subprocess.DEVNULL, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
        check=True).stdout
    commit = subprocess.check_output(['git', '--git-dir', str(PX4 / '.git'),
        'rev-parse', '--verify', 'HEAD'], env=env, text=True).strip()
    checks = {
        'board_id_56': descriptor['board_id'] == 56,
        'board_revision_0': descriptor['board_revision'] == 0,
        'px4fw_v1': descriptor['magic'] == 'PX4FWv1',
        'package_commit_matches_base_source': descriptor['git_hash'] == commit,
        'image_size_matches': descriptor['image_size'] == len(image),
        'application_capacity_unchanged': descriptor['image_maxsize'] == 1966080,
        'image_fits_application_region': len(image) <= descriptor['image_maxsize'],
        'package_image_equals_official_bin': image == actual_bin,
        'independent_elf_objcopy_equals_official_bin': independently_extracted == actual_bin,
        'parameter_xml_exact': zlib.decompress(base64.b64decode(
            descriptor['parameter_xml'], validate=True)) == (BUILD / 'parameters.xml').read_bytes(),
        'airframe_xml_exact': zlib.decompress(base64.b64decode(
            descriptor['airframe_xml'], validate=True)) == (BUILD / 'airframes.xml').read_bytes(),
        'protected_files_unchanged': all(record(Path(p)) == before
            for p, before in protected_before.items()),
        'elf_exact_latest_link_identity': isolation.sha(ELF) == EXPECTED_ELF,
        'historical_recovery_exact': isolation.sha(RECOVERY) == EXPECTED_RECOVERY,
    }
    result = {
        'scope': 'FMUV6C_APPLICATION_PACKAGE',
        'passed': all(checks.values()), 'checks': checks,
        'command': command, 'reused_existing_package': reused,
        'elapsed_s': time.time() - started,
        'elf': record(ELF), 'application_bin': record(BIN), 'application_package': record(PACKAGE),
        'image_sha256': hashlib.sha256(image).hexdigest().upper(),
        'metadata': {k: descriptor[k] for k in ['board_id', 'board_revision', 'git_hash',
            'git_identity', 'image_size', 'image_maxsize', 'build_time']},
        'official_packager': record(PX4 / 'Tools/px_mkfw.py'),
        'prototype': record(PX4 / 'boards/px4/fmu-v6c/firmware.prototype'),
        'link_receipt': record(LINK_RECEIPT),
        'recovery_application': record(RECOVERY),
        'protected_files_before': protected_before,
        'bootloader_file_bytes_unchanged': checks['protected_files_unchanged'],
        'candidate_capability_delta': 'Private USB-only HIL application: UAVCAN optional group disabled; no CAN hardware support',
        'source_identity_boundary': 'git_hash identifies the PX4 base revision; source hashes identify the integration code.',
        'live_board_identity_verified': False,
        'runtime_context_installed': False, 'target_execution_performed': False,
    }
    if LOG.exists():
        result['log'] = record(LOG)
    RECEIPT.write_text(json.dumps(result, ensure_ascii=False, indent=2) + '\n')
    print(json.dumps({'passed': result['passed'], 'checks': len(checks),
        'receipt': str(RECEIPT), 'package': result['application_package'],
        'metadata': result['metadata']}, ensure_ascii=False), flush=True)
    assert result['passed']


if __name__ == '__main__':
    main()
