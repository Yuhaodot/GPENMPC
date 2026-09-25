"""Read the explicitly configured flight-controller identity."""
import json
import os
from pathlib import Path
import re


def device_identity(field=None):
    path = Path(os.environ.get("GPENMPC_DEVICE_CONFIG", "") or
                Path(__file__).resolve().parents[3] / "local" / "device.json")
    if not path.is_file():
        raise ValueError("Configure GPENMPC_DEVICE_CONFIG or local/device.json with the verified device identity.")
    device = json.loads(path.read_text(encoding="utf-8-sig"))
    patterns = {"uid": r"[1-9][0-9]{0,19}", "px4_guid": r"[0-9A-F]{36}",
                "bootloader_sn_display": r"[0-9A-F]{24}"}
    if not isinstance(device, dict) or any(not isinstance(device.get(k), str) or
            re.fullmatch(p, device[k]) is None for k, p in patterns.items()):
        raise ValueError("Device identity format is invalid.")
    if int(device["uid"]) > (1 << 64) - 1:
        raise ValueError("Device UID exceeds uint64.")
    serial = device["px4_guid"][-24:]
    if device["bootloader_sn_display"] != serial[16:24] + serial[8:16] + serial[:8]:
        raise ValueError("GUID and bootloader serial do not identify the same device.")
    return device if field is None else device[field]
