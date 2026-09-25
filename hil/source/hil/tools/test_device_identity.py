"""Test device configuration parsing without device access."""
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile

from gpenmpc_device_identity import device_identity


def main():
    fixture = dict(uid='1234605616436508552', px4_guid='000600000000111111112222222233333333',
                   bootloader_sn_display='333333332222222211111111')
    prior = os.environ.get('GPENMPC_DEVICE_CONFIG')
    with tempfile.TemporaryDirectory(prefix='gpenmpc_identity_') as folder:
        path = Path(folder) / 'device.json'
        os.environ['GPENMPC_DEVICE_CONFIG'] = str(path)
        cases = [None, {}, dict(fixture, uid=123), dict(fixture, uid='0'),
                 dict(fixture, uid='18446744073709551616'), dict(fixture, uid='*'),
                 dict(fixture, px4_guid='A' * 36), dict(fixture, bootloader_sn_display='0' * 24)]
        try:
            for case in cases:
                if case is not None:
                    path.write_text(json.dumps(case), encoding='utf-8')
                try:
                    device_identity()
                except (ValueError, FileNotFoundError):
                    pass
                else:
                    raise AssertionError('Invalid identity accepted')
            path.write_text(json.dumps(fixture), encoding='utf-8')
            assert device_identity() == fixture
            assert device_identity('uid') == fixture['uid']
            env = dict(os.environ)
            for key in ('GPENMPC_DEVICE_CONFIG', 'GPENMPC_EXTERNAL_PATHS', 'GPENMPC_RECOVERY_ROOT',
                        'HIL_RECOVERY_RECORDS', 'GPENMPC_RFLY_ROOT'):
                env.pop(key, None)
            source = Path(__file__).resolve().parent
            code = ('import sys; import gpenmpc_application_upload_once as u; '
                    'u.read_image("fmuv6c"); '
                    'assert "serial" not in sys.modules and "px_uploader" not in sys.modules')
            result = subprocess.run([sys.executable, '-B', '-c', code], cwd=source, env=env,
                                    capture_output=True, text=True, timeout=30)
            assert result.returncode == 0, result.stderr
            result = subprocess.run([sys.executable, '-B', str(source/'gpenmpc_application_upload_once.py'), '--help'],
                                    env=env, capture_output=True, text=True, timeout=30)
            assert result.returncode == 0, result.stderr
        finally:
            if prior is None:
                os.environ.pop('GPENMPC_DEVICE_CONFIG', None)
            else:
                os.environ['GPENMPC_DEVICE_CONFIG'] = prior
    print(json.dumps(dict(passed=True, invalid_configurations=len(cases), exact_identity=True,
                         import_without_private_records=True, help_without_private_records=True)))


if __name__ == '__main__':
    main()
