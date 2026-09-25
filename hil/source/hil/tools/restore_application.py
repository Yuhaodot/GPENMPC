"""Read verified recovery inputs and restore the configured board once."""
import base64
import hashlib
import importlib.util
import json
from pathlib import Path
from gpenmpc_external_path import external_path
from gpenmpc_device_identity import device_identity
import os
import re
import zlib

IMAGE_SHA = '7722616157AF96E3493D1827F01A0713D92373946C669B854B8F043552892FC7'
UPLOADER_SHA = 'C5A78438C3CCFD0B54CAF43EF2B2798365D01B7196159EFC2E1A1C2EB77D15CC'
PRE_SHA = '073E7CE165D8D158BC329CFE96AEC4F968A6847A2A0AB9D90C1C127D3CB65EBB'
POST_SHA = '629870FD3878CEAAE8C983FEB8C1C6B38502C21CE4AC2F77978844F9AFB4DC3A'
# STM32 board_get_px4_guid serializes word2,word1,word0 as big-endian.
# Official GET_SN reads addresses 0,4,8; uploader prints each word reversed.
# Sources: platforms/nuttx/src/px4/stm/stm32_common/version/board_identity.c
# and bootloader/common/bl.c -> stm/stm32_common/main.c flash_func_read_sn.
COMMIT = '6ea3539157ca358c70a515878b77077af7d4611d'


def recovery_root():
    """Resolve the explicitly supplied recovery archive when it is needed."""
    value = os.environ.get('GPENMPC_RECOVERY_ROOT', '')
    return Path(value or external_path('hil_configuration_archive'))


def image_path():
    return recovery_root() / 'firmware/px4_fmu-v6c_default.px4'


def recovery_records():
    value = os.environ.get('HIL_RECOVERY_RECORDS', '')
    return Path(value) if value else Path(external_path('recovery_parameter_records'))


def uploader_path():
    root = os.environ.get('GPENMPC_RFLY_ROOT', '')
    _require(bool(root), 'Configure GPENMPC_RFLY_ROOT with the RflySim/PX4PSP installation root.')
    return Path(root) / 'Firmware/Tools/px_uploader.py'


def _require(ok, message):
    if not ok:
        raise RuntimeError(message)


def _read_exact(path, expected_sha, expected_bytes=None):
    raw = path.read_bytes()
    _require(hashlib.sha256(raw).hexdigest().upper() == expected_sha, 'frozen file SHA mismatch: ' + str(path))
    _require(expected_bytes is None or len(raw) == expected_bytes, 'frozen file size mismatch')
    return raw


def _semantics(snapshot):
    rows = list(snapshot['parameters'])
    union = snapshot['native_hover_parameter_union']
    _require(union['passed'] is True and len(union['rows']) == 138, 'original 138 receipt invalid')
    for row in union['rows']:
        observed = row['observed']
        _require(row['passed'] is True and observed['mav_type'] == row['expected_type'] and
                 observed['raw_bits_hex'].upper() == row['expected_bits'].upper(), 'typed union inconsistency')
        rows.append(observed)
    out = {}
    for row in rows:
        name = row['name']
        bits = row['raw_bits_hex'].upper()
        _require(name != '_HASH_CHECK' and re.fullmatch(r'[0-9A-F]{8}', bits), 'not a writable typed target')
        value = (int(row['mav_type']), bits)
        _require(name not in out or out[name] == value, 'duplicate parameter disagreement')
        out[name] = value
    _require(len(out) == 167, 'selected 138+29 union changed; this is not the full parameter catalog')
    return out


def frozen_inputs():
    """Read and hash the raw-bit targets."""
    IMAGE = image_path()
    RUN = recovery_records()
    UPLOADER = uploader_path()
    device = device_identity()
    package = json.loads(_read_exact(IMAGE, IMAGE_SHA, 1846176))
    image = zlib.decompress(base64.b64decode(package['image'], validate=True))
    _require(package['board_id'] == 56 and package['git_hash'] == COMMIT and
             package['image_maxsize'] == 1966080 and package['image_size'] == len(image) == 1964180,
             'Reference application descriptor mismatch')
    _read_exact(UPLOADER, UPLOADER_SHA, 37979)
    pre = json.loads(_read_exact(RUN / 'SERIAL_PREFLIGHT.json', PRE_SHA, 128400))
    post = json.loads(_read_exact(RUN / 'SERIAL_POSTFLIGHT.json', POST_SHA, 128400))
    a, b = _semantics(pre), _semantics(post)
    _require(a == b, 'historical selected semantics changed')
    for snapshot in (pre, post):
        _require(str(snapshot['uid']) == device['uid'] and snapshot['commit'] == COMMIT and
                 bytes(snapshot['raw_identity']['autopilot_version']['uid2']).hex().upper() == device['px4_guid'],
                 'Recovery record identity mismatch')
    return dict(image_path=str(IMAGE), image_sha256=IMAGE_SHA,
                image_bytes=len(image), uploader_path=str(UPLOADER), uploader_sha256=UPLOADER_SHA,
                uid=device['uid'], px4_guid=device['px4_guid'], bootloader_sn_display=device['bootloader_sn_display'],
                selected_parameters=a, selected_count=167, full_catalog_proven=False)


def official_methods():
    """Load audited class definitions only; returned factory opens COM if called.

    Caller must use flash_once(), never the vendor CLI/main or send_reboot().
    This function itself neither constructs uploader nor accesses any socket.
    """
    UPLOADER = uploader_path()
    _read_exact(UPLOADER, UPLOADER_SHA, 37979)
    spec = importlib.util.spec_from_file_location('gpenmpc_recovery_uploader', UPLOADER)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.uploader, module.firmware


def flash_once(fresh_guard, uploader_factory, firmware_loader, journal):
    """Single application attempt in already-entered bootloader; no retries.

    journal must durably record each event before returning. Outer captures
    vendor stdout/stderr, performs bounded bootloader acquisition before this
    call, and handles post-reboot fresh identity + selected parameter recovery.
    Return is application upload/CRC evidence, NOT final safe-restoration PASS.
    """
    target = frozen_inputs()
    _require(str(fresh_guard.get('uid')) == target['uid'] and fresh_guard.get('px4_guid') == target['px4_guid'] and
             fresh_guard.get('board_id') == 56 and fresh_guard.get('port') == 'COM3', 'fresh board identity absent')
    for key in ('same_transaction_fresh', 'same_usb_reenumeration_chain', 'usb_only',
                'disarmed', 'landed', 'physical_path_disabled', 'virtual_path_disabled',
                'pwm_out_stopped', 'all_modules_stopped', 'prior_com_owner_released', 'bootloader_ready'):
        _require(fresh_guard.get(key) is True, 'restore prerequisite missing: ' + key)
    up = None
    receipt = dict(application_attempts=0, verified_successes=0, port_open_attempts=0,
                   port_closed=False, bootloader_firmware_writes=0, parameter_writes=0,
                   automatic_retries=0, uploader_main_invoked=False,
                   final_safe_restore_proven=False, image_sha256=IMAGE_SHA)
    try:
        journal('RESTORE_COM_OPEN_ATTEMPT', {'port': 'COM3'})
        receipt['port_open_attempts'] = 1
        up = uploader_factory('COM3', 115200, [])
        up.identify()
        _require(up.board_type == 56 and up.fw_maxsize == 1966080 and up.bl_rev in (4, 5),
                 'bootloader target/protocol mismatch')
        words = [up._uploader__getSN(offset) for offset in (0, 4, 8)]
        _require(all(len(w) == 4 for w in words), 'partial bootloader serial')
        serial = b''.join(w[::-1] for w in words).hex().upper()
        receipt['observed_bootloader_sn'] = serial
        _require(serial == target['bootloader_sn_display'], 'bootloader serial differs from the configured board')
        fw = firmware_loader(target['image_path'])
        _require(bytes(fw.image) == zlib.decompress(base64.b64decode(
                    json.loads(_read_exact(Path(target['image_path']), IMAGE_SHA, 1846176))['image'])),
                 'official firmware loader bytes differ')
        journal('RESTORE_APPLICATION_UPLOAD_ATTEMPT', {'image_sha256': IMAGE_SHA, 'force': False, 'boot_delay': None})
        receipt['application_attempts'] = 1
        up.upload(fw, force=False, boot_delay=None)
        receipt['verified_successes'] = 1
        journal('RESTORE_APPLICATION_VERIFIED_RETURN', {'image_sha256': IMAGE_SHA})
    except BaseException as error:
        receipt['first_failure'] = type(error).__name__ + ': ' + str(error)
        if not isinstance(error, Exception):
            raise
    finally:
        if up is not None:
            try:
                up.close()
                receipt['port_closed'] = True
            except Exception as error:
                receipt['close_failure'] = type(error).__name__ + ': ' + str(error)
        receipt['application_crc_stage_passed'] = (receipt['verified_successes'] == 1 and
                                                 receipt['port_closed'] and 'first_failure' not in receipt)
        try:
            journal('RESTORE_APPLICATION_EXIT', receipt)
        except Exception as error:
            receipt['final_journal_failure'] = type(error).__name__ + ': ' + str(error)
            receipt['application_crc_stage_passed'] = False
    return receipt


def verify_selected_readback(observed_rows):
    """Strict post-recovery check only; intentionally does not write parameters."""
    target = frozen_inputs()['selected_parameters']
    actual = {}
    for row in observed_rows:
        name = row['name']
        _require(name not in actual, 'duplicate final readback')
        actual[name] = (int(row['mav_type']), row['raw_bits_hex'].upper())
    missing = sorted(set(target) - set(actual))
    changed = sorted(k for k in target if k in actual and target[k] != actual[k])
    return dict(passed=not missing and not changed, selected_count=167,
                missing=missing, changed=changed, additional_observed=sorted(set(actual) - set(target)),
                full_catalog_proven=False, parameter_writes=0)


def restore_selected_once(fresh_guard, read_parameter, write_parameter_once, journal):
    """Restore original 167 values through the OUTER'S already-owned link.

    No port is opened here. fresh_guard() must inspect this owner's current
    identity/safety after the reference application upload and boot. read_parameter(name)
    and write_parameter_once(name, mav_type, raw_bits_hex) return typed rows;
    the latter must send one PARAM_SET only and wait for its matching echo,
    without retry. Its raw-bit encoding must not numerically cast UINT32 bits
    through Python float (some retained UINT32 payloads are NaN bit patterns).
    journal must persist the attempt before each setter call and may fail closed.
    A successful return proves selected parameter semantics only, not final
    module/output safety or COM release. The outer still owns those actions.
    """
    target = frozen_inputs()['selected_parameters']
    receipt = dict(selected_count=167, parameter_set_calls_attempted=0,
                   typed_echoes_verified=0, readbacks_verified_after_write=0,
                   unchanged=0, automatic_write_retries=0, port_open_attempts=0,
                   wire_attempt_count_not_inferred=True, rows=[], passed=False,
                   final_safe_restore_proven=False)

    def guard():
        state = fresh_guard()
        device = device_identity()
        _require(str(state.get('uid')) == device['uid'] and state.get('px4_guid') == device['px4_guid'] and
                 state.get('board_id') == 56 and state.get('commit') == COMMIT,
                 'Post-restore identity mismatch')
        for key in ('same_transaction_fresh', 'same_usb_reenumeration_chain',
                    'reference_application_verified', 'postboot_identity_fresh',
                    'exclusive_existing_owner', 'usb_only', 'disarmed', 'landed',
                    'physical_path_disabled', 'virtual_path_disabled',
                    'pwm_out_stopped', 'all_modules_stopped'):
            _require(state.get(key) is True, 'parameter restore prerequisite missing: ' + key)
        return state

    def matches(row, name, expected):
        return (row.get('name') == name and int(row.get('mav_type', -1)) == expected[0]
                and str(row.get('raw_bits_hex', '')).upper() == expected[1])

    def typed(row):
        # Encode UINT32 values as hexadecimal to preserve NaN payload bits in strict JSON.
        bits = str(row.get('raw_bits_hex', '')).upper()
        _require(re.fullmatch(r'[0-9A-F]{8}', bits) is not None, 'malformed typed raw bits')
        return dict(name=str(row['name']), mav_type=int(row['mav_type']), raw_bits_hex=bits)

    try:
        guard()
        # With outputs disabled by the guard, restore only the recorded parameter values.
        for name in sorted(target):
            wanted = target[name]
            before = typed(read_parameter(name))
            row = dict(name=name, mav_type=wanted[0], raw_bits_hex=wanted[1],
                       before=before, setter_called=False, echo_verified=False,
                       readback_verified=False)
            receipt['rows'].append(row)
            _require(before.get('name') == name and int(before.get('mav_type', -1)) == wanted[0],
                     'parameter missing/type drift: ' + name)
            if matches(before, name, wanted):
                receipt['unchanged'] += 1
                continue
            guard()
            journal('RESTORE_REFERENCE_PARAMETER_SET_CALL_ATTEMPT',
                    dict(name=name, mav_type=wanted[0], raw_bits_hex=wanted[1], before=before))
            row['setter_called'] = True
            receipt['parameter_set_calls_attempted'] += 1
            echo = typed(write_parameter_once(name, wanted[0], wanted[1]))
            row['echo'] = echo
            _require(matches(echo, name, wanted), 'PARAM_SET typed echo mismatch: ' + name)
            row['echo_verified'] = True
            receipt['typed_echoes_verified'] += 1
            journal('RESTORE_REFERENCE_PARAMETER_TYPED_ECHO', row)
            after = typed(read_parameter(name))
            row['after'] = after
            _require(matches(after, name, wanted), 'parameter independent readback mismatch: ' + name)
            row['readback_verified'] = True
            receipt['readbacks_verified_after_write'] += 1
            journal('RESTORE_REFERENCE_PARAMETER_READBACK', row)
        guard()
        final_rows = [read_parameter(name) for name in sorted(target)]
        receipt['final_selected_readback'] = verify_selected_readback(final_rows)
        guard()
        receipt['passed'] = receipt['final_selected_readback']['passed']
    except BaseException as error:
        receipt['first_failure'] = type(error).__name__ + ': ' + str(error)
        if not isinstance(error, Exception):
            raise
    finally:
        try:
            journal('RESTORE_REFERENCE_SELECTED_EXIT', receipt)
        except Exception as error:
            receipt['final_journal_failure'] = type(error).__name__ + ': ' + str(error)
            receipt['passed'] = False
    return receipt
