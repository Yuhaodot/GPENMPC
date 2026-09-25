"""Nonblocking, single-owner NSH poller for GPENMPC controller status.

The poller never reads a serial handle itself.  The sole HIL owner sends one
allowlisted ``SERIAL_CONTROL`` query and dispatches received shell frames back
through :meth:`accept_serial_control`.  This preserves the 100 Hz HIL writer.
"""
from __future__ import annotations

from dataclasses import dataclass
import math
import re
from typing import Any

from pymavlink import mavutil


QUERY = "listener gpenmpc_se3_control_status 0 1"
SOURCE = "PX4_UORB_GPENMPC_SE3_CONTROL_STATUS"
MAXIMUM_GUARD_AGE_NS = 100_000_000
QUERY_INTERVAL_NS = 50_000_000
QUERY_TIMEOUT_NS = 90_000_000
MAXIMUM_RESPONSE_BYTES = 32_768


class NshStatusError(RuntimeError):
    """Permanent fail-closed status observation failure."""


def _strip_ansi(text: str) -> str:
    return re.sub(r"\x1b\[[0-9;?]*[ -/]*[@-~]", "", text).replace("\r", "")


def _field(text: str, name: str) -> str | None:
    match = re.search(r"(?m)^\s*" + re.escape(name) + r":\s*([^\n]+)", text)
    return None if match is None else match.group(1).strip()


def _number(value: str | None) -> float | None:
    if value is None:
        return None
    token = re.match(r"^[^\s(]+", value)
    if token is None:
        return None
    try:
        parsed = float(token.group(0))
    except ValueError:
        return None
    return parsed if math.isfinite(parsed) else None


def _boolean(value: str | None) -> bool | None:
    if value is None:
        return None
    if value.lower() == "true":
        return True
    if value.lower() == "false":
        return False
    return None


def _vector(value: str | None) -> list[float] | None:
    if value is None:
        return None
    match = re.search(r"\[([^\]]+)\]", value)
    if match is None:
        return None
    try:
        parsed = [float(item.strip()) for item in match.group(1).split(",")]
    except ValueError:
        return None
    if len(parsed) != 3 or not all(math.isfinite(item) for item in parsed):
        return None
    return parsed


def parse_status_atom(raw: bytes, rx_ns: int) -> dict[str, Any]:
    text = _strip_ansi(raw.decode("utf-8", errors="replace"))
    if "TOPIC: gpenmpc_se3_control_status" not in text:
        raise NshStatusError("STATUS_TOPIC_MISSING")
    atom: dict[str, Any] = {
        "source": SOURCE,
        "transport": "PX4_SERIAL_CONTROL_NSH_LISTENER",
        "valid": True,
        "plant_truth_used": False,
        "rx_ns": int(rx_ns),
        "raw_text": text,
    }
    for name in (
        "timestamp",
        "sample_timestamp",
        "reference_timestamp",
        "segment_timestamp",
        "input_sample_count",
        "output_publish_count",
        "rejected_sample_count",
        "reset_count",
        "dt_s",
        "reference_age_s",
        "segment_age_s",
        "hover_thrust",
        "total_mass_kg",
        "yaw_setpoint_rad",
        "control_mode",
        "failure_reason",
    ):
        atom[name] = _number(_field(text, name))
    for name in (
        "active",
        "reference_fresh",
        "segment_fresh",
        "state_valid",
        "hover_thrust_fresh",
        "native_position_controller_disabled",
        "native_attitude_rate_allocator_enabled",
        "single_publisher_contract_pass",
    ):
        atom[name] = _boolean(_field(text, name))
    for name in (
        "robust_acceleration_ned_mps2",
        "commanded_acceleration_ned_mps2",
        "normalized_thrust_ned",
        "position_ned_m",
        "velocity_ned_mps",
        "reference_position_ned_m",
        "reference_velocity_ned_mps",
        "reference_acceleration_ned_mps2",
        "nominal_feedback_acceleration_ned_mps2",
        "drag_feedforward_acceleration_ned_mps2",
    ):
        atom[name] = _vector(_field(text, name))
    age = re.search(r"timestamp:\s*[^\n]*\(([0-9.eE+\-]+)\s+seconds\s+ago\)", text)
    atom["listener_age_s"] = None if age is None else float(age.group(1))
    required_identity = ("timestamp", "control_mode", "failure_reason")
    if any(atom[name] is None for name in required_identity):
        raise NshStatusError("STATUS_IDENTITY_FIELD_MISSING")
    return atom


@dataclass
class NshStatusCounters:
    queries_sent: int = 0
    responses_completed: int = 0
    chunks_received: int = 0
    bytes_received: int = 0
    active_guards_issued: int = 0


class NonblockingNshStatusPoller:
    def __init__(self) -> None:
        self.counters = NshStatusCounters()
        self.latest: dict[str, Any] | None = None
        self.failed = False
        self.failure_code = ""
        self._query_open = False
        self._query_started_ns = 0
        self._next_query_ns = 0
        self._buffer = bytearray()
        self._status_generation = 0

    def tick(self, link: Any, now_ns: int) -> bool:
        self._require_usable(now_ns)
        if self._query_open:
            if now_ns - self._query_started_ns > QUERY_TIMEOUT_NS:
                self._fail("STATUS_QUERY_TIMEOUT")
            return False
        if now_ns < self._next_query_ns:
            return False
        payload = ("\n" + QUERY + "\n").encode("ascii")
        assert len(payload) <= 70
        link.mav.serial_control_send(
            mavutil.mavlink.SERIAL_CONTROL_DEV_SHELL,
            mavutil.mavlink.SERIAL_CONTROL_FLAG_EXCLUSIVE
            | mavutil.mavlink.SERIAL_CONTROL_FLAG_RESPOND,
            0,
            0,
            len(payload),
            list(payload) + [0] * (70 - len(payload)),
        )
        self._query_open = True
        self._query_started_ns = now_ns
        self._buffer.clear()
        self.counters.queries_sent += 1
        return True

    def accept_serial_control(self, message: Any, rx_ns: int) -> bool:
        self._require_usable(rx_ns)
        if getattr(message, "get_type", lambda: "")() != "SERIAL_CONTROL":
            raise NshStatusError("NON_SERIAL_CONTROL_DISPATCH")
        if int(getattr(message, "device", -1)) != int(
            mavutil.mavlink.SERIAL_CONTROL_DEV_SHELL
        ):
            return False
        count = int(getattr(message, "count", 0))
        if count <= 0:
            return False
        if not self._query_open:
            self._fail("UNSOLICITED_SHELL_RESPONSE")
        self._buffer.extend(bytes(message.data[:count]))
        self.counters.chunks_received += 1
        self.counters.bytes_received += count
        if len(self._buffer) > MAXIMUM_RESPONSE_BYTES:
            self._fail("STATUS_RESPONSE_OVERSIZE")
        clean = _strip_ansi(self._buffer.decode("utf-8", errors="replace"))
        topic_at = clean.find("TOPIC: gpenmpc_se3_control_status")
        complete = topic_at >= 0 and clean.find("nsh>", topic_at) >= 0
        if not complete:
            return False
        atom = parse_status_atom(bytes(self._buffer), rx_ns)
        self._status_generation += 1
        atom["status_generation"] = self._status_generation
        if self.latest is not None:
            for field in ("timestamp", "input_sample_count", "output_publish_count"):
                before, after = self.latest.get(field), atom.get(field)
                if before is not None and after is not None:
                    if field == "timestamp" and after <= before:
                        self._fail("STATUS_TIMESTAMP_NOT_ADVANCING")
                    if field != "timestamp" and after < before:
                        self._fail("STATUS_COUNTER_REGRESSION")
        self.latest = atom
        self._query_open = False
        self._next_query_ns = rx_ns + QUERY_INTERVAL_NS
        self._buffer.clear()
        self.counters.responses_completed += 1
        return True

    def active_guard(self, now_ns: int) -> dict[str, Any]:
        self._require_usable(now_ns)
        if self.latest is None:
            raise NshStatusError("STATUS_GUARD_NOT_OBSERVED")
        guard = dict(self.latest)
        age_ns = now_ns - int(guard["rx_ns"])
        if age_ns < 0 or age_ns > MAXIMUM_GUARD_AGE_NS:
            raise NshStatusError("STATUS_GUARD_STALE")
        required_true = (
            "active",
            "state_valid",
            "reference_fresh",
            "segment_fresh",
            "native_position_controller_disabled",
            "native_attitude_rate_allocator_enabled",
            "single_publisher_contract_pass",
        )
        robust = guard.get("robust_acceleration_ned_mps2")
        if (
            guard.get("control_mode") != 1.0
            or guard.get("failure_reason") != 0.0
            or not all(guard.get(field) is True for field in required_true)
            or robust is None
            or any(abs(float(value)) > 1.0e-7 for value in robust)
        ):
            raise NshStatusError("STATUS_GUARD_ATOMS_NOT_ADMITTED")
        self.counters.active_guards_issued += 1
        return guard

    def status(self) -> dict[str, Any]:
        return {
            "schema": "GPENMPC_NONBLOCKING_NSH_STATUS_POLLER_V1",
            "query": QUERY,
            "query_interval_ns": QUERY_INTERVAL_NS,
            "query_timeout_ns": QUERY_TIMEOUT_NS,
            "maximum_guard_age_ns": MAXIMUM_GUARD_AGE_NS,
            "query_open": self._query_open,
            "failed": self.failed,
            "failure_code": self.failure_code,
            "latest_status_generation": self._status_generation,
            **vars(self.counters),
            "hardware_actions": 0,
            "com_owner_count": 0,
            "plant_truth_used": False,
        }

    def _require_usable(self, now_ns: int) -> None:
        if not isinstance(now_ns, int) or isinstance(now_ns, bool) or now_ns < 0:
            raise NshStatusError("INVALID_MONOTONIC_TIME")
        if self.failed:
            raise NshStatusError(self.failure_code)

    def _fail(self, code: str):
        self.failed = True
        self.failure_code = code
        raise NshStatusError(code)


__all__ = [
    "NonblockingNshStatusPoller",
    "NshStatusError",
    "parse_status_atom",
]
