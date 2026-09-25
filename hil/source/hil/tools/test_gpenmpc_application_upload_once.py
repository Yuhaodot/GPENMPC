"""Injected owner tests."""
import copy
from datetime import datetime, timedelta, timezone
import json
import argparse
from contextlib import contextmanager
import os
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace

import gpenmpc_application_upload_once as subject


@contextmanager
def synthetic_device():
    prior = os.environ.get('GPENMPC_DEVICE_CONFIG')
    with tempfile.TemporaryDirectory(prefix='gpenmpc_device_') as folder:
        path = Path(folder) / 'device.json'
        path.write_text(json.dumps(dict(uid='1234605616436508552',
            px4_guid='000600000000111111112222222233333333',
            bootloader_sn_display='333333332222222211111111')), encoding='utf-8')
        os.environ['GPENMPC_DEVICE_CONFIG'] = str(path)
        try:
            yield
        finally:
            if prior is None:
                os.environ.pop('GPENMPC_DEVICE_CONFIG', None)
            else:
                os.environ['GPENMPC_DEVICE_CONFIG'] = prior


def main(include_recovery=False):
    now = datetime(2026, 9, 7, 12, 0, tzinfo=timezone.utc)
    rows = []
    cases = ('fmuv6c', 'identify_retry', 'unknown_identify', 'wrong_board', 'wrong_serial',
             'wrong_uid', 'wrong_guid', 'loader_bytes_mismatch', 'upload_failure', 'close_failure', 'expired', 'unsafe')
    if include_recovery:
        cases += ('restore_reference',)
    for case in cases:
        application = case if case in ('fmuv6c', 'restore_reference') else 'fmuv6c'
        _, sha, image = subject.read_image(application)
        handoff = dict(schema=subject.SCHEMA, transaction_id='HOST_FIXTURE', application=application,
            expected_image_sha256=sha, uid=subject.recovery.device_identity('uid'), px4_guid=subject.recovery.device_identity('px4_guid'),
            board_id=56, port='COM3', commit=subject.recovery.COMMIT,
            pre_reboot_pnp_instance='USB\\VID_3185&PID_0038\\FIXTURE_PRE',
            controlled_application_reboot_to_bootloader_count=1,
            guard_observed_utc=(now-timedelta(seconds=5)).isoformat(),
            reboot_requested_utc=(now-timedelta(seconds=3)).isoformat(),
            handoff_created_utc=(now-timedelta(seconds=1)).isoformat(),
            expires_utc=(now+timedelta(seconds=30)).isoformat())
        handoff.update({k: True for k in ('passed','reboot_ack_accepted','usb_only','disarmed','landed','physical_path_disabled',
            'virtual_path_disabled','pwm_out_stopped','all_modules_stopped','prior_com_owner_released',
            'COM_closed','same_transaction_fresh','usb_reenumeration_pending')})
        if case == 'expired':
            handoff['expires_utc'] = (now-timedelta(seconds=0.1)).isoformat()
        if case == 'unsafe':
            handoff['landed'] = False
        if case == 'wrong_uid':
            handoff['uid'] = '1'
        if case == 'wrong_guid':
            handoff['px4_guid'] = '0' * 36
        raw = json.dumps(handoff).encode()
        calls, events, consumed = [], [], []
        elapsed = [0.0]

        class FakeUploader:
            board_type = 55 if case == 'wrong_board' else 56
            fw_maxsize = 1966080
            bl_rev = 5

            def __init__(self, port, baud, rates):
                assert (port, baud, rates) == ('COM3', 115200, [])
                self.port = SimpleNamespace(is_open=True)
                calls.append('open')

            def identify(self):
                calls.append('identify')
                if case == 'unknown_identify':
                    raise RuntimeError('unknown protocol failure')
                if case == 'identify_retry' and calls.count('identify') == 1:
                    raise RuntimeError('timeout waiting for data (1 bytes)')

            def _uploader__getSN(self, offset):
                return (b'\0'*4 if case == 'wrong_serial' else
                        bytes.fromhex(subject.recovery.device_identity('bootloader_sn_display'))[offset:offset+4][::-1])

            def upload(self, firmware, force, boot_delay):
                assert consumed and force is False and boot_delay is None and bytes(firmware.image) == image
                calls.append('upload')
                if case == 'upload_failure':
                    raise RuntimeError('injected program/CRC failure')
                self.port.is_open = False

            def close(self):
                calls.append('close')
                if case == 'close_failure':
                    raise RuntimeError('injected close failure')
                self.port.is_open = False

        def journal(kind, data):
            events.append((kind, copy.deepcopy(data)))

        def sleep(dt):
            elapsed[0] += dt

        def loader(path):
            loaded = subject.read_image(application)[2]
            if case == 'loader_bytes_mismatch':
                loaded = bytes([loaded[0] ^ 1]) + loaded[1:]
            return SimpleNamespace(image=loaded)

        try:
            result = subject.upload_once(application, 'HOST_FIXTURE', lambda:(raw,handoff),
                FakeUploader, loader, journal, consumed.append,
                lambda:[dict(instance_id='USB\\BOOT_FIXTURE', friendly_name='PX4 (COM3)',status='OK')],
                now=lambda:now, monotonic=lambda:elapsed[0], sleep=sleep)
        except RuntimeError:
            assert case in ('expired','unsafe','wrong_uid','wrong_guid') and not calls and not consumed
        else:
            expected = case in ('fmuv6c','restore_reference','identify_retry')
            assert result['application_crc_stage_passed'] == expected
            assert calls.count('upload') <= 1 and len(consumed) == calls.count('upload')
            assert result['application_upload_attempts'] == calls.count('upload')
            assert not result['final_safe_restore_proven']
            if case == 'loader_bytes_mismatch':
                assert calls.count('upload') == 0 and not consumed
                assert 'official loaded image bytes differ' in result['first_failure']
            if case == 'identify_retry':
                assert calls.count('open') == calls.count('close') == 2
            if case in ('wrong_board','wrong_serial','unknown_identify','loader_bytes_mismatch'):
                assert calls.count('open') == 1 and not consumed and calls.count('close') == 1
        rows.append(dict(case=case, passed=True))
    vendor_or_serial_imported = any(name == 'serial' or name.startswith('serial.')
        or name == 'px_uploader' or name == 'gpenmpc_recovery_uploader'
        for name in sys.modules)
    assert not vendor_or_serial_imported
    print(json.dumps(dict(passed=True, count=len(rows), rows=rows,
                         actual_hardware_actions=0, vendor_or_serial_imported=vendor_or_serial_imported), indent=2))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--with-recovery', action='store_true', help='Also test the explicitly configured recovery image.')
    args = parser.parse_args()
    with synthetic_device():
        main(args.with_recovery)
