"""Injected-backend HOST tests."""
import base64
import copy
import json
from types import SimpleNamespace
import zlib

import restore_application as restore


def main():
    target = restore.frozen_inputs()
    image = zlib.decompress(base64.b64decode(json.loads(restore.image_path().read_bytes())['image']))
    guard = dict(uid=restore.device_identity('uid'), px4_guid=restore.device_identity('px4_guid'), board_id=56, port='COM3')
    guard.update({k: True for k in ('same_transaction_fresh', 'same_usb_reenumeration_chain',
        'usb_only', 'disarmed', 'landed', 'physical_path_disabled', 'virtual_path_disabled',
        'pwm_out_stopped', 'all_modules_stopped', 'prior_com_owner_released', 'bootloader_ready')})
    rows = []

    def exercise(name, altered=None, fault=None):
        events, calls = [], []

        class FakeUploader:
            board_type = 56
            fw_maxsize = 1966080
            bl_rev = 5

            def identify(self):
                calls.append('identify')
                if fault == 'identify':
                    raise RuntimeError('injected identify')
                if fault == 'board':
                    self.board_type = 55

            def _uploader__getSN(self, offset):
                if fault == 'serial':
                    return b'\0' * 4
                printed = bytes.fromhex(restore.device_identity('bootloader_sn_display'))
                return printed[offset:offset+4][::-1]

            def upload(self, fw, force, boot_delay):
                calls.append('upload')
                assert force is False and boot_delay is None and bytes(fw.image) == image
                if fault == 'upload':
                    raise RuntimeError('injected CRC failure')
                if fault == 'interrupt':
                    raise KeyboardInterrupt('injected interrupt')

            def close(self):
                calls.append('close')
                if fault == 'close':
                    raise RuntimeError('injected close')

        def factory(port, baud, rates):
            calls.append('open')
            assert (port, baud, rates) == ('COM3', 115200, [])
            return FakeUploader()

        def journal(event, data):
            events.append(event)
            if fault == 'journal_pre' and event == 'RESTORE_APPLICATION_UPLOAD_ATTEMPT':
                raise RuntimeError('injected journal')
            if fault == 'journal_exit' and event == 'RESTORE_APPLICATION_EXIT':
                raise RuntimeError('injected final journal')

        g = copy.deepcopy(guard)
        g.update(altered or {})
        try:
            result = restore.flash_once(g, factory, lambda _: SimpleNamespace(image=image), journal)
        except RuntimeError:
            return not calls
        except KeyboardInterrupt:
            return fault == 'interrupt' and calls.count('upload') == calls.count('close') == 1
        if altered is not None:
            return False  # Reject the negative guard case.
        if fault in ('identify', 'board', 'serial', 'journal_pre'):
            return (result['application_attempts'] == 0 and calls.count('upload') == 0 and
                    calls.count('close') == 1 and not result['application_crc_stage_passed'])
        if fault == 'upload':
            return (result['application_attempts'] == 1 and result['verified_successes'] == 0 and
                    calls.count('upload') == calls.count('close') == 1 and result['automatic_retries'] == 0)
        if fault in ('close', 'journal_exit'):
            return (result['application_attempts'] == result['verified_successes'] == 1 and
                    not result['application_crc_stage_passed'] and calls.count('upload') == 1)
        return (result['application_crc_stage_passed'] and not result['final_safe_restore_proven'] and
                result['application_attempts'] == result['verified_successes'] == 1 and
                calls == ['open', 'identify', 'upload', 'close'] and not result['uploader_main_invoked'])

    rows.append(dict(case='reference_parameter_rawbit_semantics', passed=target['selected_count'] == 167))
    rows.append(dict(case='one_application_no_udp_no_retry', passed=exercise('positive')))
    for key, value in dict(uid='0', px4_guid='0', board_id=55, port='COM4', same_transaction_fresh=False,
        same_usb_reenumeration_chain=False, usb_only=False, disarmed=False, landed=False,
        physical_path_disabled=False, virtual_path_disabled=False, pwm_out_stopped=False,
        all_modules_stopped=False, prior_com_owner_released=False, bootloader_ready=False).items():
        rows.append(dict(case='reject_' + key + '_before_open', passed=exercise(key, {key: value})))
    for fault in ('identify', 'board', 'serial', 'journal_pre', 'upload', 'interrupt', 'close', 'journal_exit'):
        rows.append(dict(case='injected_' + fault, passed=exercise(fault, fault=fault)))
    observed = [dict(name=n, mav_type=v[0], raw_bits_hex=v[1]) for n, v in target['selected_parameters'].items()]
    rows.append(dict(case='selected_final_readback_exact', passed=restore.verify_selected_readback(observed)['passed']))
    changed = copy.deepcopy(observed); changed[0]['raw_bits_hex'] = 'DEADBEEF'
    rows.append(dict(case='selected_changed_readback_reject', passed=not restore.verify_selected_readback(changed)['passed']))
    rows.append(dict(case='selected_missing_readback_reject', passed=not restore.verify_selected_readback(observed[1:])['passed']))
    try:
        restore.verify_selected_readback(observed + observed[:1]); duplicate_rejected = False
    except RuntimeError:
        duplicate_rejected = True
    rows.append(dict(case='duplicate_readback_reject', passed=duplicate_rejected))
    print(json.dumps(dict(passed=all(r['passed'] for r in rows), count=len(rows), rows=rows,
                          actual_hardware_actions=0, actual_serial_imports=0, actual_flash=0), indent=2))
    assert all(r['passed'] for r in rows)


if __name__ == '__main__':
    main()
