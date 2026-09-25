"""Upload a verified application using an explicit, fresh hardware handoff.

CLI execution performs the upload. Import and --help only load definitions.
"""
import argparse
import base64
from datetime import datetime, timezone
import hashlib
import json
import os
import re
from pathlib import Path
import subprocess
import time
import zlib

import restore_application as recovery

APPLICATIONS = {
    'fmuv6c': (Path(__file__).resolve().parents[3] / 'firmware' / 'fmuv6c' / 'px4_fmu-v6c_default.px4', 'AF571397E67013D23D59EF2827B879B76A408271713891686790BD64661B4743', 1855252, 1962048),
    'restore_reference': (None, recovery.IMAGE_SHA, 1846176, 1964180),
}
SCHEMA = 'GPENMPC_APPLICATION_FLASH_HANDOFF_V1'
# Bound acquisition time.
IDENTIFY_WINDOW_S = 20.0
MAX_IDENTIFY_ATTEMPTS = 10
MAX_ACQUISITION_POLLS = 80
MAX_HANDOFF_VALIDITY_S = 120.0


def require(ok, reason):
    if not ok:
        raise RuntimeError(reason)


def utc_now():
    return datetime.now(timezone.utc)


def parse_utc(value):
    result = datetime.fromisoformat(str(value).replace('Z', '+00:00'))
    require(result.utcoffset() is not None and result.utcoffset().total_seconds() == 0,
            'handoff timestamps require explicit UTC')
    return result


def read_image(application):
    path, sha, size, image_size = APPLICATIONS[application]
    if path is None:
        path = recovery.image_path()
    raw = recovery._read_exact(path, sha, size)
    descriptor = json.loads(raw)
    image = zlib.decompress(base64.b64decode(descriptor['image'], validate=True))
    require(descriptor['board_id'] == 56 and descriptor['git_hash'] == recovery.COMMIT and
            descriptor['image_maxsize'] == 1966080 and len(image) == image_size == descriptor['image_size'],
            'exact application descriptor mismatch')
    return path, sha, image


def validate_handoff(handoff, application, transaction_id, now):
    require(handoff.get('schema') == SCHEMA and handoff.get('application') == application and
            handoff.get('transaction_id') == transaction_id and bool(transaction_id), 'handoff operation mismatch')
    require(handoff.get('expected_image_sha256') == APPLICATIONS[application][1], 'handoff image mismatch')
    device = recovery.device_identity()
    require(str(handoff.get('uid')) == device['uid'] and handoff.get('px4_guid') == device['px4_guid'] and
            handoff.get('board_id') == 56 and handoff.get('port') == 'COM3' and
            handoff.get('commit') == recovery.COMMIT, 'handoff fresh board identity mismatch')
    require(type(handoff.get('controlled_application_reboot_to_bootloader_count')) is int and
            handoff['controlled_application_reboot_to_bootloader_count'] == 1, 'one controlled reboot receipt required')
    for key in ('passed', 'reboot_ack_accepted', 'usb_only', 'disarmed', 'landed', 'physical_path_disabled', 'virtual_path_disabled',
                'pwm_out_stopped', 'prior_com_owner_released', 'COM_closed',
                'same_transaction_fresh', 'usb_reenumeration_pending'):
        require(handoff.get(key) is True, 'handoff prerequisite false/missing: ' + key)
    closing_restore = (application == 'restore_reference' and handoff.get('operation') == 'REBOOT_RESTORE_REFERENCE'
        and handoff.get('recovery_only_nonexecuting_closing') is True
        and handoff.get('all_modules_stopped') is False)
    if closing_restore:
        names, states = handoff.get('module_names', []), handoff.get('module_status', [])
        require(len(names) == len(states) == 7 and names.count('gpenmpc_rfly_canonical_local') == 1,
                'closing recovery module observation missing')
        for name, state in zip(names, states):
            if name == 'gpenmpc_rfly_canonical_local':
                require(isinstance(state, str) and re.search(
                    r'\[gpenmpc_rfly_canonical_local\]\s+task=running phase=3 reason=[1-5] route=[01] plant_evidence=[0-3](?:\s|$)', state),
                    'recovery requires actual nonexecuting Closing/disarmed state')
            else:
                require(isinstance(state, str) and re.search(r'not running|task=stopped|command not found', state),
                        'another module is not stopped in closing recovery')
    require(handoff.get('all_modules_stopped') is True or closing_restore,
            'live modules are not eligible for application handoff')
    require(isinstance(handoff.get('pre_reboot_pnp_instance'), str) and
            len(handoff['pre_reboot_pnp_instance']) > 8, 'exact pre-reboot PnP instance required')
    observed, reboot, created, expires = [parse_utc(handoff.get(k)) for k in
        ('guard_observed_utc', 'reboot_requested_utc', 'handoff_created_utc', 'expires_utc')]
    require(observed <= reboot <= created <= now < expires and
            0 < (expires - observed).total_seconds() <= MAX_HANDOFF_VALIDITY_S,
            'handoff stale/future/unbounded or event ordering invalid')


def _retryable_identify(error):
    # Do not reopen after identity mismatch, access denial, unknown error or unconfirmed close.
    if isinstance(error, FileNotFoundError):
        return True
    if isinstance(error, OSError) and getattr(error, 'winerror', None) in (2, 3, 21, 1167):
        return True
    return isinstance(error, RuntimeError) and str(error).startswith('timeout waiting for data (')


def upload_once(application, transaction_id, load_handoff, factory, loader, journal,
                consume, pnp_probe, now=utc_now, monotonic=time.monotonic, sleep=time.sleep):
    """Injected owner lifecycle for a tiny offline test or the CLI live backend."""
    path, sha, image = read_image(application)
    initial_raw, handoff = load_handoff()
    handoff_sha = hashlib.sha256(initial_raw).hexdigest().upper()
    validate_handoff(handoff, application, transaction_id, now())
    result = dict(schema='GPENMPC_APPLICATION_UPLOAD_ONCE_RESULT_V1', application=application,
        transaction_id=transaction_id, handoff_sha256=handoff_sha, image_path=str(path), image_sha256=sha,
        application_upload_attempts=0, application_verified_successes=0,
        identify_open_attempts=0, close_attempts=0, close_successes=0, port_closed=True,
        bootloader_firmware_writes=0, parameter_writes=0, uploader_main_invoked=False,
        application_upload_retries=0, host_reboot_requests=0,
        vendor_successful_upload_includes_application_reboot=True,
        application_crc_stage_passed=False, final_safe_restore_proven=False,
        status='NOT_STARTED', identify_attempts=[])
    up = None
    start = monotonic()

    def close_owner():
        nonlocal up
        if up is None:
            return
        owner = up
        result['close_attempts'] += 1
        # Close the owner even if journal writing fails.
        log_error = None
        try:
            journal('UPLOAD_COM_CLOSE_ATTEMPT', dict(attempt=result['close_attempts']))
        except Exception as error:
            log_error = error
        try:
            owner.close()
            if hasattr(owner, 'port'):
                require(owner.port is None or owner.port.is_open is False, 'serial close not confirmed')
            result['close_successes'] += 1
            result['port_closed'] = True
            up = None
        except Exception as error:
            result['port_closed'] = False
            result['close_failure'] = type(error).__name__ + ': ' + str(error)
            raise
        journal('UPLOAD_COM_CLOSED', dict(closed=True, attempt=result['close_attempts']))
        if log_error is not None:
            raise log_error

    try:
        for attempt in range(1, MAX_ACQUISITION_POLLS + 1):
            validate_handoff(handoff, application, transaction_id, now())
            require(monotonic() - start < IDENTIFY_WINDOW_S, 'bootloader acquisition time exhausted')
            atom = dict(attempt=attempt, elapsed_s=monotonic()-start, identified=False)
            result['identify_attempts'].append(atom)
            pnp = pnp_probe()
            atom['pnp'] = pnp
            journal('UPLOAD_READONLY_COM3_PNP', pnp)
            require(isinstance(pnp, list), 'PnP schema invalid')
            if not pnp:
                require(attempt < MAX_ACQUISITION_POLLS, 'COM3 not re-enumerated within acquisition bound')
                sleep(min(0.25, max(0.0, IDENTIFY_WINDOW_S - (monotonic()-start))))
                continue
            require(len(pnp) == 1 and pnp[0].get('status') == 'OK' and
                    '(COM3)' in pnp[0].get('friendly_name', '') and
                    isinstance(pnp[0].get('instance_id'), str), 'COM3 PnP ambiguous/not started')
            require(result['identify_open_attempts'] < MAX_IDENTIFY_ATTEMPTS,
                    'maximum nonmutating bootloader identify attempts exhausted')
            journal('UPLOAD_IDENTIFY_OPEN_ATTEMPT', atom)
            result['identify_open_attempts'] += 1
            try:
                up = factory('COM3', 115200, [])
                result['port_closed'] = False
                up.identify()
                atom['identified'] = True
            except Exception as error:
                atom['error'] = type(error).__name__ + ': ' + str(error)
                close_owner()
                journal('UPLOAD_IDENTIFY_NOT_READY', atom)
                if not _retryable_identify(error) or result['identify_open_attempts'] == MAX_IDENTIFY_ATTEMPTS:
                    raise
                sleep(min(0.25, max(0.0, IDENTIFY_WINDOW_S - (monotonic()-start))))
                continue
            # Treat a fully observed wrong target as terminal.
            require(up.board_type == 56 and up.fw_maxsize == 1966080 and up.bl_rev in (4, 5),
                    'bootloader target/protocol mismatch')
            words = [up._uploader__getSN(offset) for offset in (0, 4, 8)]
            require(all(len(word) == 4 for word in words), 'partial bootloader serial')
            serial = b''.join(word[::-1] for word in words).hex().upper()
            result['observed_bootloader_sn'] = serial
            result['observed_board_id'] = up.board_type
            result['observed_image_maxsize'] = up.fw_maxsize
            result['observed_bl_protocol'] = up.bl_rev
            require(serial == recovery.device_identity('bootloader_sn_display'), 'bootloader serial differs from configured Pixhawk')
            result['pre_reboot_pnp_instance'] = handoff['pre_reboot_pnp_instance']
            result['bootloader_pnp_instance'] = pnp[0]['instance_id']
            result['same_usb_reenumeration_chain_observed'] = True
            result['physical_identity_binding_basis'] = 'FRESH_PRE_UID_GUID_TO_GET_SN__UNIQUE_PRESENT_COM3_PNP'
            require(monotonic()-start < IDENTIFY_WINDOW_S, 'bootloader identification exceeded acquisition bound')
            raw_again, handoff_again = load_handoff()
            require(raw_again == initial_raw, 'handoff file changed during acquisition')
            validate_handoff(handoff_again, application, transaction_id, now())
            # Rehash on the same side of the last pre-erase check as loader.
            _, _, exact_image = read_image(application)
            firmware = loader(str(path))
            require(bytes(firmware.image) == image == exact_image, 'official loaded image bytes differ')
            journal('UPLOAD_TARGET_VERIFIED_BEFORE_ERASE', dict(serial=serial, board_id=56,
                image_maxsize=1966080, image_sha256=sha, force=False, boot_delay=None))
            consume(dict(transaction_id=transaction_id, application=application,
                         handoff_sha256=handoff_sha, image_sha256=sha))
            journal('APPLICATION_UPLOAD_CALL_ATTEMPT', dict(image_sha256=sha, attempt=1))
            result['application_upload_attempts'] = 1
            # Preserve cleanup around the single erase/program call.
            up.upload(firmware, force=False, boot_delay=None)
            result['application_verified_successes'] = 1
            journal('APPLICATION_UPLOAD_CRC_VERIFIED_RETURN', dict(image_sha256=sha))
            break
    except BaseException as error:
        result['first_failure'] = type(error).__name__ + ': ' + str(error)
    finally:
        try:
            close_owner()
        except Exception as error:
            result.setdefault('first_failure', type(error).__name__ + ': ' + str(error))
        result['application_crc_stage_passed'] = (result['application_verified_successes'] == 1 and
            result['port_closed'] and 'first_failure' not in result)
        result['status'] = ('APPLICATION_CRC_VERIFIED__POSTBOOT_SAFETY_AND_167_SEMANTICS_REQUIRED'
            if result['application_crc_stage_passed'] else 'APPLICATION_TRANSACTION_ABORTED_NO_UPLOAD_RETRY')
        try:
            journal('APPLICATION_UPLOAD_EXIT', result)
        except Exception as error:
            result['final_journal_failure'] = type(error).__name__ + ': ' + str(error)
            result['application_crc_stage_passed'] = False
            result['status'] = 'EVIDENCE_FAILURE__POSTBOOT_RECOVERY_REQUIRED'
    return result


def windows_com3_pnp():
    """Enumerate present PnP devices."""
    script = ("[Console]::OutputEncoding=[System.Text.UTF8Encoding]::new(); "
        "$rows=@(Get-PnpDevice -PresentOnly -ErrorAction Stop | "
        "Where-Object { $_.FriendlyName -match '\\(COM3\\)$' } | "
        "ForEach-Object { [ordered]@{instance_id=$_.InstanceId;friendly_name=$_.FriendlyName;status=$_.Status} }); "
        "ConvertTo-Json -InputObject $rows -Compress")
    proc = subprocess.run(['powershell.exe', '-NoProfile', '-NonInteractive', '-Command', script],
                          capture_output=True, timeout=5, encoding='utf-8')
    require(proc.returncode == 0, 'read-only PnP failed: ' + proc.stderr)
    return json.loads(proc.stdout.lstrip('\ufeff'))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--application', choices=tuple(APPLICATIONS), required=True)
    parser.add_argument('--handoff', type=Path, required=True)
    parser.add_argument('--transaction-id', required=True)
    parser.add_argument('--journal', type=Path, required=True)
    parser.add_argument('--result', type=Path, required=True)
    args = parser.parse_args()
    handoff_path = args.handoff.resolve()
    marker = handoff_path.with_name(handoff_path.name + '.upload_consumed')

    def load():
        raw = handoff_path.read_bytes()
        return raw, json.loads(raw)

    raw, handoff = load()
    validate_handoff(handoff, args.application, args.transaction_id, utc_now())
    read_image(args.application)
    # Validate the recovery inputs before loading the device backend.
    recovery.frozen_inputs()
    require(not marker.exists(), 'this handoff already consumed an upload attempt')
    require(args.journal.resolve() != args.result.resolve(), 'distinct journal/result paths required')
    require(not args.journal.exists() and not args.result.exists(), 'refuse existing journal/result')
    require(args.journal.parent.is_dir() and args.result.parent.is_dir(), 'outer output directory must exist')
    with args.journal.open('x', encoding='utf-8') as log, args.result.open('x', encoding='utf-8') as result_file:
        def journal(kind, data):
            log.write(json.dumps(dict(utc=utc_now().isoformat(), event=kind, data=data), allow_nan=False)+'\n')
            log.flush()
            os.fsync(log.fileno())

        def consume(data):
            with marker.open('x', encoding='utf-8') as file:
                json.dump(data, file, allow_nan=False)
                file.flush()
                os.fsync(file.fileno())

        result = None
        try:
            journal('APPLICATION_UPLOAD_CLI_ENTERED', dict(handoff_path=str(handoff_path),
                handoff_sha256=hashlib.sha256(raw).hexdigest().upper(), application=args.application))
            factory, loader = recovery.official_methods()
            result = upload_once(args.application, args.transaction_id, load, factory, loader, journal,
                                 consume, windows_com3_pnp)
        except BaseException as error:
            result = dict(status='PRE_UPLOAD_WRAPPER_ABORT', first_failure=type(error).__name__+': '+str(error),
                          application_crc_stage_passed=False, final_safe_restore_proven=False)
        json.dump(result, result_file, indent=2, allow_nan=False)
        result_file.flush()
        os.fsync(result_file.fileno())
    print(json.dumps(result, allow_nan=False))
    return 0 if result['application_crc_stage_passed'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
