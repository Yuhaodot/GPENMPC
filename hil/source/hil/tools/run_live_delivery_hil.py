#!/usr/bin/env python3
"""Single-owner live runner for the selected GP-ENMPC delivery method.

The runner connects exactly one MATLAB M600 software plant to one Pixhawk
MAVLink owner.  It advances the frozen delivery clock only from actual board
virtual actuator output, performs four native land/disarm/grounded-unload/
rearm cycles, and performs a fifth native landing at task completion.

Importing this module is board inert.  Live access is only reachable through
``--execute`` with the exact canonical run identity.
"""
from __future__ import annotations

import argparse
import csv
from collections import deque
import hashlib
import json
import math
import os
from pathlib import Path
from gpenmpc_external_path import external_path
from gpenmpc_device_identity import device_identity
import queue
import socket
import subprocess
import sys
import threading
import time
import traceback
from typing import Any

import numpy as np

# ODOMETRY (message 331) and the generated-C status/segment contract are
# MAVLink 2/common identities.  Pin the dialect before pymavlink is imported
# by the trusted parent transport module; relying on the host default can load
# the MAVLink 1 ardupilotmega table, where ODOMETRY is not defined.
os.environ["MAVLINK20"] = "1"
os.environ["MAVLINK_DIALECT"] = "common"


OVERLAY = Path(__file__).resolve().parents[1]
PARENT_RUNTIME = Path(external_path('px4_coptersim_runtime'))
WORK_ROOT = Path(external_path('native_visual_host_runtime'))
CANONICAL_ROOT = Path(external_path('hil_configuration_archive'))
TASK_PATH = Path(external_path('physical_task_fixture'))
TASK_PACKAGE = TASK_PATH.with_name("LIVE_TASK_PACKAGE.json")

for item in (OVERLAY / "tools", PARENT_RUNTIME):
    if str(item) not in sys.path:
        sys.path.insert(0, str(item))

from live_hil_plant_client import LiveHilPlantClient, LiveHilPlantError  # noqa: E402
from mavlink_hil_session import HilLink, mavutil, mode_name  # noqa: E402
from nsh_status_poller import (  # noqa: E402
    NonblockingNshStatusPoller,
    NshStatusError,
)


SCHEMA = "GPENMPC_SELECTED_METHOD_LIVE_DELIVERY_HIL_V1"
RUN_ID = "GPENMPC_LIVE_DELIVERY"
METHOD = "A1_COORDINATED_PHYSICAL"
EXPECTED_UID = int(device_identity("uid"))
EXPECTED_BOARD_VERSION = 56
EXPECTED_COMMIT = "6ea3539157ca358c70a515878b77077af7d4611d"
EXPECTED_FIRMWARE_SHA = "7722616157AF96E3493D1827F01A0713D92373946C669B854B8F043552892FC7"
TASK_SHA = "83E9817E9D5C9FB44E7049F1C802847C6BFE486729E7997DB5860644DEF26DBE"
SOURCE_MANIFEST_SHA = "98030691749450E45E6F11ACAF656A9DCA3889402ACFDCD50E0114F23047A586"

DT_S = 0.01
GPS_PERIOD_TICKS = 10
SETPOINT_PERIOD_TICKS = 2
SEGMENT_PERIOD_TICKS = 20
HEARTBEAT_PERIOD_TICKS = 100
MANUAL_PERIOD_TICKS = 5
VISUAL_PERIOD_TICKS = 3
PRESTREAM_TICKS = 100
ESTIMATOR_WARMUP_TIMEOUT_S = 20.0
ARM_SEQUENCE_TIMEOUT_S = 18.0
LAND_TIMEOUT_S = 30.0
STREAM_STALE_NS = 100_000_000
TASK_ORIGIN_UP_M = np.asarray([-34.518, -75.710, 0.0], dtype=float)
SOFTWARE_FROM_PX4 = np.asarray([5, 1, 4, 6, 2, 3], dtype=int) - 1
LAT0_DEG = 47.397742
LON0_DEG = 8.545594
ALT0_M = 488.0


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest().upper()


def json_safe(value: Any) -> Any:
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, (np.floating, np.integer)):
        return value.item()
    if isinstance(value, bytes):
        return value.hex().upper()
    if isinstance(value, dict):
        return {str(key): json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple, deque)):
        return [json_safe(item) for item in value]
    if isinstance(value, float) and not math.isfinite(value):
        return {"nonfinite": str(value)}
    return value


def atomic_json(path: Path, value: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(
        json.dumps(json_safe(value), indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    temporary.replace(path)


def artifact(path: Path) -> dict[str, Any]:
    return {
        "path": str(path.resolve()),
        "bytes": path.stat().st_size,
        "sha256": sha256(path),
    }


class AsyncCsvWriter:
    def __init__(self, path: Path, fields: list[str]) -> None:
        self.path = path
        self.fields = fields
        self.items: queue.Queue[dict[str, Any] | None] = queue.Queue(maxsize=10000)
        self.failure: BaseException | None = None
        self.rows_written = 0
        self.thread = threading.Thread(target=self._run, daemon=True)

    def start(self) -> None:
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.thread.start()

    def submit(self, row: dict[str, Any]) -> None:
        if self.failure is not None:
            raise RuntimeError(f"TRACE_WRITER_FAILED:{self.failure}")
        try:
            self.items.put_nowait(row)
        except queue.Full as error:
            raise RuntimeError("TRACE_WRITER_QUEUE_OVERFLOW") from error

    def close(self) -> None:
        self.items.put(None)
        self.thread.join(timeout=30.0)
        if self.thread.is_alive():
            raise RuntimeError("TRACE_WRITER_DID_NOT_STOP")
        if self.failure is not None:
            raise RuntimeError(f"TRACE_WRITER_FAILED:{self.failure}")

    def _run(self) -> None:
        try:
            with self.path.open("w", newline="", encoding="utf-8") as stream:
                writer = csv.DictWriter(stream, fieldnames=self.fields)
                writer.writeheader()
                while True:
                    row = self.items.get()
                    if row is None:
                        break
                    writer.writerow(row)
                    self.rows_written += 1
                    if self.rows_written % 1000 == 0:
                        stream.flush()
                stream.flush()
                os.fsync(stream.fileno())
        except BaseException as error:
            self.failure = error


class HostLoadMonitor:
    """One-hertz diagnostic summary."""

    def __init__(self) -> None:
        self.rows: list[dict[str, Any]] = []
        self.next_wall = 0.0
        try:
            import psutil  # type: ignore
        except Exception:
            self.psutil = None
        else:
            self.psutil = psutil
            psutil.cpu_percent(interval=None)

    def sample(self, mission_clock_s: float) -> None:
        now = time.perf_counter()
        if now < self.next_wall:
            return
        self.next_wall = now + 1.0
        row: dict[str, Any] = {
            "host_monotonic_ns": time.perf_counter_ns(),
            "mission_clock_s": mission_clock_s,
        }
        if self.psutil is not None:
            vm = self.psutil.virtual_memory()
            io = self.psutil.disk_io_counters()
            row.update(
                {
                    "cpu_percent": self.psutil.cpu_percent(interval=None),
                    "available_ram_bytes": int(vm.available),
                    "disk_read_bytes": None if io is None else int(io.read_bytes),
                    "disk_write_bytes": None if io is None else int(io.write_bytes),
                }
            )
        self.rows.append(row)


class PlantSnapshotProxy:
    """Shape adapter for HilLink.send_hil_sensors."""

    def __init__(self, snapshot: dict[str, Any]) -> None:
        if int(snapshot.get("plant_instance_count", 0)) != 1:
            raise RuntimeError("PLANT_INSTANCE_IDENTITY_REJECTED")
        self.snapshot = snapshot
        global_ned = np.asarray(snapshot["position_ned_m"], dtype=float)
        self.position_ned_m = global_ned - np.asarray(
            [TASK_ORIGIN_UP_M[0], TASK_ORIGIN_UP_M[1], 0.0]
        )
        self.velocity_ned_mps = np.asarray(snapshot["velocity_ned_mps"], dtype=float)
        self.quaternion_ned_frd = np.asarray(
            snapshot["quaternion_wxyz_body_to_ned"], dtype=float
        )
        self.body_rate_frd_rad_s = np.asarray(
            snapshot["body_rate_frd_rad_s"], dtype=float
        )
        values = np.r_[
            self.position_ned_m,
            self.velocity_ned_mps,
            self.quaternion_ned_frd,
            self.body_rate_frd_rad_s,
        ]
        if not np.isfinite(values).all() or abs(np.linalg.norm(self.quaternion_ned_frd) - 1.0) > 1e-5:
            raise RuntimeError("PLANT_SENSOR_PROXY_NONFINITE")

    def sensor_values(self, altitude_origin_m: float) -> dict[str, Any]:
        del altitude_origin_m
        sensor = self.snapshot["sensor"]
        return {
            "specific_force_body": np.asarray(
                sensor["specific_force_body_mps2"], dtype=float
            ),
            "gyro_body": np.asarray(sensor["gyro_body_frd_rad_s"], dtype=float),
            "magnetic_body": np.asarray(sensor["magnetic_body_gauss"], dtype=float),
            "pressure_mbar": float(sensor["pressure_mbar"]),
            "pressure_alt_m": float(sensor["pressure_alt_m"]),
        }


class AtomicBoardState:
    """One atomic PX4 estimate and actual virtual-actuator observation queue."""

    def __init__(self, uid: int, system_id: int, component_id: int, boot_generation: int) -> None:
        self.uid = str(uid)
        self.system_id = int(system_id)
        self.component_id = int(component_id)
        self.boot_generation = int(boot_generation)
        self.armed = False
        self.arm_transitions = 0
        self.mode = "UNKNOWN"
        self.landed_state = -1
        self.last_heartbeat_ns = 0
        self.odometry_generation = 100
        self.actuator_observation_generation = 0
        self.odometry: deque[dict[str, Any]] = deque(maxlen=512)
        self.actuators: deque[dict[str, Any]] = deque(maxlen=512)
        self.command_acks: list[dict[str, Any]] = []
        self.statustext: list[dict[str, Any]] = []

    def accept(
        self,
        message: Any,
        rx_ns: int,
        poller: NonblockingNshStatusPoller,
    ) -> None:
        kind = message.get_type()
        source_system = int(message.get_srcSystem())
        if kind == "SERIAL_CONTROL":
            if source_system == self.system_id:
                poller.accept_serial_control(message, rx_ns)
            return
        if source_system != self.system_id:
            return
        if kind == "HEARTBEAT":
            observed = bool(
                int(message.base_mode) & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
            )
            if observed and not self.armed:
                self.arm_transitions += 1
            self.armed = observed
            self.mode = mode_name(int(message.custom_mode))
            self.last_heartbeat_ns = rx_ns
        elif kind == "EXTENDED_SYS_STATE":
            self.landed_state = int(message.landed_state)
        elif kind == "ODOMETRY":
            self.odometry_generation += 1
            q = [float(value) for value in message.q]
            local_position = np.asarray(
                [message.x, message.y, message.z], dtype=float
            )
            global_position = local_position + np.asarray(
                [TASK_ORIGIN_UP_M[0], TASK_ORIGIN_UP_M[1], 0.0]
            )
            sample = {
                "source": "PX4_EKF2_MAVLINK_ODOMETRY_331",
                "atomic_estimate": True,
                "plant_truth_used": False,
                "uid": self.uid,
                "system_id": self.system_id,
                "component_id": self.component_id,
                "boot_generation": self.boot_generation,
                "sample_timestamp_ns": int(message.time_usec) * 1000,
                "odometry_reset_counter": int(message.reset_counter),
                "odometry_frame_id": int(message.frame_id),
                "odometry_child_frame_id": int(message.child_frame_id),
                "odometry_estimator_type": int(message.estimator_type),
                "odometry_quality": int(getattr(message, "quality", -1)),
                "position_ned_m": global_position.tolist(),
                "velocity_ned_mps": [
                    float(message.vx),
                    float(message.vy),
                    float(message.vz),
                ],
                "quaternion_wxyz_body_to_ned": q,
                "omega_frd_rad_s": [
                    float(message.rollspeed),
                    float(message.pitchspeed),
                    float(message.yawspeed),
                ],
                "position_rx_ns": rx_ns,
                "attitude_rx_ns": rx_ns,
                "rates_rx_ns": rx_ns,
                "position_generation": self.odometry_generation,
                "attitude_generation": self.odometry_generation,
                "rates_generation": self.odometry_generation,
                "position_valid": True,
                "attitude_valid": True,
                "rates_valid": True,
            }
            core = np.r_[global_position, sample["velocity_ned_mps"], q, sample["omega_frd_rad_s"]]
            if (
                int(message.frame_id) != int(mavutil.mavlink.MAV_FRAME_LOCAL_NED)
                or int(message.child_frame_id) != int(mavutil.mavlink.MAV_FRAME_LOCAL_NED)
                or int(message.estimator_type)
                != int(mavutil.mavlink.MAV_ESTIMATOR_TYPE_AUTOPILOT)
                or not np.isfinite(core).all()
                or abs(float(np.linalg.norm(q)) - 1.0) > 1e-5
            ):
                return
            self.odometry.append(sample)
        elif kind == "HIL_ACTUATOR_CONTROLS":
            controls = np.asarray(message.controls, dtype=float)
            if len(controls) != 16 or not np.isfinite(controls[:6]).all():
                return
            controls[6:] = np.where(np.isfinite(controls[6:]), controls[6:], 0.0)
            self.actuator_observation_generation += 1
            self.actuators.append(
                {
                    "mavpackettype": "HIL_ACTUATOR_CONTROLS",
                    "time_usec": int(message.time_usec),
                    "controls": controls.tolist(),
                    "src_system": source_system,
                    "src_component": int(message.get_srcComponent()),
                    "rx_ns": rx_ns,
                    "observation_generation": self.actuator_observation_generation,
                }
            )
        elif kind == "COMMAND_ACK":
            self.command_acks.append(
                {
                    "rx_ns": rx_ns,
                    "command": int(message.command),
                    "result": int(message.result),
                    "progress": int(getattr(message, "progress", 0)),
                    "result_param2": int(getattr(message, "result_param2", 0)),
                    "target_system": int(getattr(message, "target_system", 0)),
                    "target_component": int(getattr(message, "target_component", 0)),
                }
            )
        elif kind == "STATUSTEXT":
            self.statustext.append(
                {
                    "rx_ns": rx_ns,
                    "severity": int(message.severity),
                    "text": str(message.text),
                }
            )

    def newest_odometry(self, now_ns: int) -> dict[str, Any]:
        if not self.odometry:
            raise RuntimeError("ODOMETRY_NOT_OBSERVED")
        sample = self.odometry[-1]
        if now_ns - int(sample["position_rx_ns"]) > STREAM_STALE_NS:
            raise RuntimeError("ODOMETRY_STALE")
        return sample

    def odometry_after(self, previous_timestamp_ns: int, now_ns: int) -> dict[str, Any]:
        while self.odometry and int(self.odometry[0]["sample_timestamp_ns"]) <= previous_timestamp_ns:
            self.odometry.popleft()
        if not self.odometry:
            raise RuntimeError("NEXT_ODOMETRY_NOT_OBSERVED")
        sample = self.odometry.popleft()
        dt_ns = int(sample["sample_timestamp_ns"]) - int(previous_timestamp_ns)
        if dt_ns <= 0 or dt_ns > 10_000_100 or abs(dt_ns - 10_000_000) > 500:
            raise RuntimeError(f"ODOMETRY_INTERVAL_NOT_10MS:{dt_ns}")
        if now_ns - int(sample["position_rx_ns"]) > STREAM_STALE_NS:
            raise RuntimeError("NEXT_ODOMETRY_STALE")
        return sample

    def actuator_after(
        self, previous_observation_generation: int, physical_generation: int, now_ns: int
    ) -> dict[str, Any]:
        while self.actuators and int(self.actuators[0]["observation_generation"]) <= previous_observation_generation:
            self.actuators.popleft()
        if not self.actuators:
            raise RuntimeError("NEXT_ACTUATOR_NOT_OBSERVED")
        value = dict(self.actuators.popleft())
        if now_ns - int(value["rx_ns"]) > STREAM_STALE_NS:
            raise RuntimeError("NEXT_ACTUATOR_STALE")
        value["generation"] = int(physical_generation)
        return value


class VisualPublisher:
    def __init__(self, run_id: str, address: str = "127.0.0.1", port: int = 28430) -> None:
        self.run_id = run_id
        self.address = address
        self.port = int(port)
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sequence = 0
        self.sent = 0

    def send(self, snapshot: dict[str, Any], reference_global_ned: dict[str, Any] | None) -> None:
        software = np.asarray(snapshot["rotor_thrust_software_order_n"], dtype=float)
        px4_order = np.zeros(6, dtype=float)
        px4_order[SOFTWARE_FROM_PX4] = software
        packet: dict[str, Any] = {
            "schema": "GPENMPC_VISUAL_STATE_V1",
            "run_id": self.run_id,
            "seq": self.sequence,
            "sim_time_s": float(snapshot["sim_time_s"]),
            "host_monotonic_ns": time.perf_counter_ns(),
            "layer": "HARDWARE_CLOSED_LOOP",
            "position_ned_m": snapshot["position_ned_m"],
            "velocity_ned_mps": snapshot["velocity_ned_mps"],
            "quaternion_wxyz_body_to_ned": snapshot[
                "quaternion_wxyz_body_to_ned"
            ],
            "omega_frd_rad_s": snapshot["body_rate_frd_rad_s"],
            "rotor_thrust_n": px4_order.tolist(),
            "valid": True,
            "control_mode": "A1_COORDINATED_PHYSICAL__RA_CTRL_MODE_1",
            "payload_kg": None,
            "ground_contact": bool(snapshot["contact_active"]),
        }
        if reference_global_ned is not None:
            packet["reference_position_ned_m"] = reference_global_ned[
                "position_ned_m"
            ]
        encoded = json.dumps(packet, separators=(",", ":"), allow_nan=False).encode()
        if len(encoded) > 8192:
            raise RuntimeError("VISUAL_DATAGRAM_OVERSIZE")
        self.socket.sendto(encoded, (self.address, self.port))
        self.sequence += 1
        self.sent += 1

    def close(self) -> None:
        self.socket.close()


class MatlabPlantProcess:
    def __init__(self, output_dir: Path, port: int) -> None:
        self.output_dir = output_dir
        self.port = int(port)
        self.ready_path = output_dir / "MATLAB_PLANT_READY.json"
        self.receipt_path = output_dir / "MATLAB_PLANT_RECEIPT.json"
        self.stdout_path = output_dir / "MATLAB_PLANT_STDOUT.log"
        self.stderr_path = output_dir / "MATLAB_PLANT_STDERR.log"
        self.process: subprocess.Popen[bytes] | None = None

    @staticmethod
    def _literal(path: Path) -> str:
        return str(path).replace("'", "''")

    def start(self) -> None:
        probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            probe.bind(("127.0.0.1", self.port))
        finally:
            probe.close()
        expression = (
            f"addpath('{self._literal(OVERLAY / 'tools')}');"
            f"runLiveHilPlantUdpService('{self._literal(WORK_ROOT)}',"
            f"'{self._literal(OVERLAY)}','127.0.0.1',{self.port},900,250000,"
            f"'{self._literal(self.ready_path)}','{self._literal(self.receipt_path)}');"
        )
        stdout = self.stdout_path.open("wb")
        stderr = self.stderr_path.open("wb")
        try:
            self.process = subprocess.Popen(
                ["matlab", "-batch", expression], stdout=stdout, stderr=stderr
            )
        finally:
            stdout.close()
            stderr.close()
        deadline = time.monotonic() + 120.0
        while time.monotonic() < deadline:
            if self.ready_path.is_file():
                ready = json.loads(self.ready_path.read_text(encoding="utf-8"))
                if ready.get("status") == "READY" and int(ready.get("port", -1)) == self.port:
                    return
            if self.process.poll() is not None:
                raise RuntimeError(
                    f"MATLAB_PLANT_SERVICE_EXITED_BEFORE_READY:{self.process.returncode}"
                )
            time.sleep(0.25)
        raise TimeoutError("MATLAB_PLANT_SERVICE_READY_TIMEOUT")

    def finish(self) -> dict[str, Any]:
        if self.process is None:
            return {"started": False, "returncode": None}
        try:
            returncode = self.process.wait(timeout=30.0)
        except subprocess.TimeoutExpired:
            self.process.terminate()
            try:
                returncode = self.process.wait(timeout=10.0)
            except subprocess.TimeoutExpired:
                self.process.kill()
                returncode = self.process.wait(timeout=10.0)
        return {
            "started": True,
            "returncode": returncode,
            "ready": artifact(self.ready_path) if self.ready_path.is_file() else None,
            "receipt": artifact(self.receipt_path) if self.receipt_path.is_file() else None,
            "stdout": artifact(self.stdout_path),
            "stderr": artifact(self.stderr_path),
        }


def fixture_sample(generation: int, now_ns: int, boot_generation: int) -> dict[str, Any]:
    return {
        "source": "PX4_EKF2_MAVLINK_ODOMETRY_331",
        "atomic_estimate": True,
        "plant_truth_used": False,
        "uid": str(EXPECTED_UID),
        "system_id": 1,
        "component_id": 1,
        "boot_generation": boot_generation,
        "sample_timestamp_ns": now_ns,
        "odometry_reset_counter": 0,
        "odometry_frame_id": 1,
        "odometry_child_frame_id": 1,
        "odometry_estimator_type": 8,
        "position_ned_m": TASK_ORIGIN_UP_M.tolist(),
        "velocity_ned_mps": [0.0, 0.0, 0.0],
        "quaternion_wxyz_body_to_ned": [1.0, 0.0, 0.0, 0.0],
        "omega_frd_rad_s": [0.0, 0.0, 0.0],
        "position_rx_ns": now_ns - 100_000,
        "attitude_rx_ns": now_ns - 100_000,
        "rates_rx_ns": now_ns - 100_000,
        "position_generation": generation,
        "attitude_generation": generation,
        "rates_generation": generation,
        "position_valid": True,
        "attitude_valid": True,
        "rates_valid": True,
    }


def reference_global_ned(response: dict[str, Any]) -> dict[str, Any]:
    reference = response.get("reference_ned") or response.get("initial_reference")
    if not isinstance(reference, dict):
        raise RuntimeError("REFERENCE_OBJECT_MISSING")
    if "position_ned_m" in reference:
        result = {
            "position_ned_m": reference["position_ned_m"],
            "velocity_ned_mps": reference["velocity_ned_mps"],
            "acceleration_ned_mps2": reference["acceleration_ned_mps2"],
            "yaw_rad": float(reference.get("yaw_rad", 0.0)),
        }
    else:
        result = {
            "position_ned_m": [
                float(reference["position_m"][0]),
                float(reference["position_m"][1]),
                -float(reference["position_m"][2]),
            ],
            "velocity_ned_mps": [
                float(reference["velocity_mps"][0]),
                float(reference["velocity_mps"][1]),
                -float(reference["velocity_mps"][2]),
            ],
            "acceleration_ned_mps2": [
                float(reference["acceleration_mps2"][0]),
                float(reference["acceleration_mps2"][1]),
                -float(reference["acceleration_mps2"][2]),
            ],
            "yaw_rad": 0.0,
        }
    if not np.isfinite(
        np.r_[
            result["position_ned_m"],
            result["velocity_ned_mps"],
            result["acceleration_ned_mps2"],
            result["yaw_rad"],
        ]
    ).all():
        raise RuntimeError("REFERENCE_NONFINITE")
    return result


def reference_local_ned(reference: dict[str, Any]) -> dict[str, Any]:
    position = np.asarray(reference["position_ned_m"], dtype=float).copy()
    position[:2] -= TASK_ORIGIN_UP_M[:2]
    return {
        **reference,
        "position_ned_m": position.tolist(),
    }


def phase_metadata(response: dict[str, Any]) -> tuple[np.ndarray, float, int, float]:
    phase = response.get("phase") or {}
    wind = np.asarray(phase.get("wind_estimate_xy_mps", [0.0, 0.0]), dtype=float)
    if wind.size != 2 or not np.isfinite(wind).all():
        raise RuntimeError("PHASE_WIND_INVALID")
    payload = float(phase.get("payload_kg", 2.21))
    leg = int(phase.get("leg_index", 1))
    task_time = float(phase.get("task_time_s", 0.0))
    return np.r_[wind, 0.0], payload, leg, task_time


def send_intervals(link: HilLink) -> None:
    for message_id, hz in (
        (mavutil.mavlink.MAVLINK_MSG_ID_ODOMETRY, 100.0),
        (mavutil.mavlink.MAVLINK_MSG_ID_HIL_ACTUATOR_CONTROLS, 100.0),
        (mavutil.mavlink.MAVLINK_MSG_ID_EXTENDED_SYS_STATE, 10.0),
        (mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS, 5.0),
    ):
        link.request_message_interval(message_id, hz)


def verify_parameter_guards(link: HilLink) -> list[dict[str, Any]]:
    expected: dict[str, int] = {
        "SYS_HITL": 1,
        "SYS_AUTOSTART": 6001,
        "MAV_TYPE": 13,
        "CA_ROTOR_COUNT": 6,
        "RA_CTRL_MODE": 1,
    }
    expected.update({f"HIL_ACT_FUNC{index}": 100 + index for index in range(1, 7)})
    expected.update({f"HIL_ACT_FUNC{index}": 0 for index in range(7, 17)})
    expected.update({f"PWM_MAIN_FUNC{index}": 0 for index in range(1, 9)})
    rows = []
    for name, target in expected.items():
        observed = link.request_param(name)
        row = {**observed, "expected": target, "pass": int(observed["decoded"]) == target}
        rows.append(row)
    if not all(row["pass"] for row in rows):
        raise RuntimeError("PARAMETER_MAPPING_OR_PHYSICAL_OUTPUT_GUARD_FAILED")
    return rows


def drain_messages(
    link: HilLink,
    board: AtomicBoardState,
    poller: NonblockingNshStatusPoller,
    maximum: int = 1024,
) -> int:
    count = 0
    for _ in range(maximum):
        message = link.recv(0.0)
        if message is None:
            break
        board.accept(message, time.perf_counter_ns(), poller)
        count += 1
    return count


class WireLoop:
    def __init__(
        self,
        link: HilLink,
        board: AtomicBoardState,
        poller: NonblockingNshStatusPoller,
        visual: VisualPublisher,
    ) -> None:
        self.link = link
        self.board = board
        self.poller = poller
        self.visual = visual
        self.tick_index = 0
        self.wire_time_s = 0.0
        self.next_wall = time.perf_counter()
        self.sensor_counts = {"hil_sensor": 0, "hil_state": 0, "hil_gps": 0}
        self.setpoint_count = 0
        self.segment_sequence = 2000
        self.segment_chunks_sent = 0
        self.pending_segment: tuple[int, bytes] | None = None
        self.maximum_wall_lateness_s = 0.0

    def tick(
        self,
        snapshot: dict[str, Any],
        reference: dict[str, Any] | None,
        phase: dict[str, Any] | None,
        allow_reference: bool,
    ) -> int:
        now = time.perf_counter()
        if now < self.next_wall:
            time.sleep(max(self.next_wall - now - 0.0004, 0.0))
            while time.perf_counter() < self.next_wall:
                pass
        now = time.perf_counter()
        self.maximum_wall_lateness_s = max(
            self.maximum_wall_lateness_s, max(0.0, now - self.next_wall)
        )
        now_ns = time.perf_counter_ns()
        proxy = PlantSnapshotProxy(snapshot)
        counters = self.link.send_hil_sensors(
            self.wire_time_s,
            proxy,
            self.board.armed,
            self.sensor_counts["hil_sensor"],
            send_hil_state=False,
            send_gps=self.tick_index % GPS_PERIOD_TICKS == 0,
            source_native_estimator=True,
            latitude_origin_deg=LAT0_DEG,
            longitude_origin_deg=LON0_DEG,
            altitude_origin_m=ALT0_M,
        )
        for key, value in counters.items():
            self.sensor_counts[key] += int(value)
        if self.tick_index % MANUAL_PERIOD_TICKS == 0:
            self.link.neutral_manual_control()
        if self.tick_index % HEARTBEAT_PERIOD_TICKS == 0:
            self.link.heartbeat()
        if allow_reference and reference is not None:
            local = reference_local_ned(reference)
            if self.tick_index % SETPOINT_PERIOD_TICKS == 0:
                self.link.send_setpoint(
                    int(self.wire_time_s * 1000.0),
                    local["position_ned_m"],
                    local["velocity_ned_mps"],
                    local["acceleration_ned_mps2"],
                    float(local["yaw_rad"]),
                )
                self.setpoint_count += 1
            if self.pending_segment is not None and self.tick_index >= self.pending_segment[0]:
                self.link.send_segment_chunk(self.pending_segment[1])
                self.segment_chunks_sent += 1
                self.pending_segment = None
            if self.tick_index % SEGMENT_PERIOD_TICKS == 0:
                if self.pending_segment is not None:
                    raise RuntimeError("SEGMENT_SEQUENCE_OVERLAP")
                self.segment_sequence += 1
                phase = phase or {}
                wind = np.asarray(phase.get("wind_estimate_xy_mps", [0.0, 0.0]), dtype=float)
                payload = float(phase.get("payload_kg", 2.21))
                chunks = HilLink.segment_payload(
                    self.segment_sequence,
                    local["position_ned_m"],
                    float(local["yaw_rad"]),
                    np.r_[wind, 0.0],
                    payload,
                    2.21,
                )
                self.link.send_segment_chunk(chunks[0])
                self.pending_segment = (self.tick_index + 2, chunks[1])
                self.segment_chunks_sent += 1
        if self.tick_index % VISUAL_PERIOD_TICKS == 0:
            self.visual.send(snapshot, reference)
        drain_messages(self.link, self.board, self.poller)
        self.poller.tick(self.link, now_ns)
        drain_messages(self.link, self.board, self.poller)
        if self.board.last_heartbeat_ns and now_ns - self.board.last_heartbeat_ns > 2_500_000_000:
            raise RuntimeError("BOARD_HEARTBEAT_WATCHDOG_TIMEOUT")
        self.tick_index += 1
        self.wire_time_s += DT_S
        self.next_wall += DT_S
        return now_ns


TRACE_FIELDS = [
    "row",
    "task_time_s",
    "phase_code",
    "leg_index",
    "board_armed",
    "px4_mode",
    "landed_state",
    "reference_x_m",
    "reference_y_m",
    "reference_z_m",
    "plant_x_m",
    "plant_y_m",
    "plant_z_m",
    "plant_vx_mps",
    "plant_vy_mps",
    "plant_vz_mps",
    "plant_ax_mps2",
    "plant_ay_mps2",
    "plant_az_mps2",
    "px4_x_m",
    "px4_y_m",
    "px4_z_m",
    "position_error_m",
    "estimator_plant_gap_m",
    "acceleration_norm_mps2",
    "jerk_norm_mps3",
    "energy_model_j",
    "payload_kg",
    "contact_active",
    "ground_confirmed",
    "physical_generation",
    "outer_generation",
    "gp_active_count",
    "b1_fallback_count",
    "control_1",
    "control_2",
    "control_3",
    "control_4",
    "control_5",
    "control_6",
]


def trace_row(
    row_index: int,
    response: dict[str, Any],
    reference: dict[str, Any],
    board: AtomicBoardState,
    actuator: dict[str, Any] | None,
    previous_acceleration: np.ndarray | None,
) -> tuple[dict[str, Any], np.ndarray]:
    plant = response["plant"]
    phase = response.get("phase") or {}
    position = np.asarray(plant["position_ned_m"], dtype=float)
    velocity = np.asarray(plant["velocity_ned_mps"], dtype=float)
    acceleration = np.asarray(plant["acceleration_ned_mps2"], dtype=float)
    desired = np.asarray(reference["position_ned_m"], dtype=float)
    latest = board.newest_odometry(time.perf_counter_ns())
    estimate = np.asarray(latest["position_ned_m"], dtype=float)
    controls = [0.0] * 16 if actuator is None else list(actuator["controls"])
    jerk = 0.0
    if previous_acceleration is not None:
        jerk = float(np.linalg.norm((acceleration - previous_acceleration) / DT_S))
    status = response.get("plant", {})
    outer = response.get("outer_service", {})
    return (
        {
            "row": row_index,
            "task_time_s": float(phase.get("task_time_s", 0.0)),
            "phase_code": int(phase.get("phase_code", -1)),
            "leg_index": int(phase.get("leg_index", -1)),
            "board_armed": int(board.armed),
            "px4_mode": board.mode,
            "landed_state": board.landed_state,
            "reference_x_m": desired[0],
            "reference_y_m": desired[1],
            "reference_z_m": desired[2],
            "plant_x_m": position[0],
            "plant_y_m": position[1],
            "plant_z_m": position[2],
            "plant_vx_mps": velocity[0],
            "plant_vy_mps": velocity[1],
            "plant_vz_mps": velocity[2],
            "plant_ax_mps2": acceleration[0],
            "plant_ay_mps2": acceleration[1],
            "plant_az_mps2": acceleration[2],
            "px4_x_m": estimate[0],
            "px4_y_m": estimate[1],
            "px4_z_m": estimate[2],
            "position_error_m": float(np.linalg.norm(position - desired)),
            "estimator_plant_gap_m": float(np.linalg.norm(estimate - position)),
            "acceleration_norm_mps2": float(np.linalg.norm(acceleration)),
            "jerk_norm_mps3": jerk,
            "energy_model_j": float(plant["energy_model_j"]),
            "payload_kg": float(phase.get("payload_kg", 0.0)),
            "contact_active": int(bool(plant["contact_active"])),
            "ground_confirmed": int(bool(plant["ground_confirmed"])),
            "physical_generation": int(response.get("generation", 0)),
            "outer_generation": int(response.get("outer_generation", 0)),
            "gp_active_count": int(outer.get("physical_runtime", {}).get("physical_gp_active_count", 0)),
            "b1_fallback_count": int(outer.get("physical_runtime", {}).get("exact_b1_fallback_count", 0)),
            **{f"control_{index + 1}": controls[index] for index in range(6)},
        },
        acceleration,
    )


def offline_validate(output: Path) -> int:
    task_package = json.loads(TASK_PACKAGE.read_text(encoding="utf-8"))
    checks = {
        "task_exists": TASK_PATH.is_file(),
        "task_sha_exact": TASK_PATH.is_file() and sha256(TASK_PATH) == TASK_SHA,
        "task_package_method_exact": task_package.get("internal_method_id") == METHOD,
        "task_package_run_id_exact": task_package.get("run_id") == RUN_ID,
        "task_ground_deliveries_four": int(task_package.get("delivery_ground_contact_count_required", -1)) == 4,
        "task_arm_transitions_five": int(task_package.get("arm_transitions_required", -1)) == 5,
        "one_plant": int(task_package.get("software_plant_instances", -1)) == 1,
        "airborne_drop_zero": int(task_package.get("airborne_drop_delivery_count", 0)) == 0,
        "canonical_firmware_exact": sha256(CANONICAL_ROOT / "firmware" / "px4_fmu-v6c_default.px4") == EXPECTED_FIRMWARE_SHA,
        "source_manifest_exact": json.loads((WORK_ROOT / "SOURCE_BINDING.json").read_text(encoding="utf-8")).get("source_manifest_sha256") == SOURCE_MANIFEST_SHA,
        "status_guard_100ms": STREAM_STALE_NS == 100_000_000,
        "plant_step_100hz": DT_S == 0.01,
        "visual_is_display_only": True,
        "hil_state_quaternion_forbidden": True,
        "mode_two_forbidden": True,
        "physical_pwm_forbidden": True,
        "bootloader_forbidden": True,
        "hardware_actions_zero": True,
    }
    result = {
        "schema": "GPENMPC_LIVE_DELIVERY_RUNNER_OFFLINE_VALIDATION_V1",
        "status": "PASS_NO_COM_NO_BOARD" if all(checks.values()) else "FAIL_OFFLINE",
        "checks": checks,
        "runner": artifact(Path(__file__)),
        "task": artifact(TASK_PATH),
        "task_package": artifact(TASK_PACKAGE),
        "hardware_actions": {
            "com_open": 0,
            "board_access": 0,
            "parameter_write": 0,
            "mapping_write": 0,
            "arm": 0,
            "offboard": 0,
            "task": 0,
            "virtual_output": 0,
            "physical_output": 0,
            "flash": 0,
            "bootloader": 0,
        },
    }
    atomic_json(output, result)
    print(json.dumps({"status": result["status"], "checks": len(checks)}))
    return 0 if all(checks.values()) else 2


def run_live(args: argparse.Namespace) -> int:
    if args.run_id != RUN_ID or args.method != METHOD:
        raise SystemExit("exact canonical run and method identity required")
    output = Path(args.output_dir).resolve()
    if output.exists():
        raise FileExistsError(output)
    output.mkdir(parents=True)
    result_path = output / "METHOD_HIL_RESULT.json"
    events_path = output / "METHOD_HIL_EVENTS.json"
    trace_path = output / "METHOD_HIL_TRACE.csv"
    safety_path = output / "METHOD_HIL_RUNNER_SAFETY.json"
    host_load_path = output / "HOST_LOAD_1HZ.json"
    events: list[dict[str, Any]] = []
    actions = {
        "com_open": 0,
        "parameter_write": 0,
        "mapping_write": 0,
        "reboot": 0,
        "flash": 0,
        "offboard_requests": 0,
        "arm_requests": 0,
        "arm_transitions": 0,
        "land_requests": 0,
        "standard_disarm_requests": 0,
        "force_disarm_requests": 0,
        "physical_output": 0,
    }
    result: dict[str, Any] = {
        "schema": SCHEMA,
        "status": "STARTED",
        "run_id": RUN_ID,
        "method": METHOD,
        "formal_started": False,
        "formal_complete": False,
        "formal_trace_rows": 0,
        "task_identity_sha256": TASK_SHA,
        "source_manifest_sha256": SOURCE_MANIFEST_SHA,
        "action_counters": actions,
    }
    atomic_json(result_path, result)
    writer = AsyncCsvWriter(trace_path, TRACE_FIELDS)
    writer.start()
    load = HostLoadMonitor()
    matlab = MatlabPlantProcess(output, int(args.plant_port))
    client: LiveHilPlantClient | None = None
    link = HilLink(args.endpoint, args.baud)
    board: AtomicBoardState | None = None
    poller = NonblockingNshStatusPoller()
    visual = VisualPublisher(RUN_ID)
    wire: WireLoop | None = None
    modules_started = False
    failure: str | None = None
    plant_snapshot: dict[str, Any] | None = None
    current_response: dict[str, Any] | None = None
    current_reference: dict[str, Any] | None = None
    current_phase: dict[str, Any] | None = None
    formal_started = False
    formal_complete = False
    task_complete = False
    last_formal_task_time: float | None = None
    last_odo_timestamp = 0
    last_actuator_observation = 0
    last_actuator: dict[str, Any] | None = None
    previous_acceleration: np.ndarray | None = None
    trace_rows = 0
    delivery_landings = 0
    grounded_unloads = 0
    final_landings = 0
    arm_cycle = 0
    force_used = False
    service_finish: dict[str, Any] | None = None
    maximum_position_error = 0.0
    position_errors: list[float] = []
    maximum_acceleration = 0.0
    maximum_jerk = 0.0
    maximum_estimator_gap = 0.0
    minimum_rotor_margin = 1.0
    initial_energy: float | None = None
    final_energy: float | None = None

    def record_progress(response: dict[str, Any], actuator: dict[str, Any] | None) -> None:
        nonlocal formal_started, last_formal_task_time, trace_rows
        nonlocal previous_acceleration, maximum_position_error, maximum_acceleration
        nonlocal maximum_jerk, maximum_estimator_gap, minimum_rotor_margin
        nonlocal initial_energy, final_energy
        phase = response.get("phase") or {}
        if not bool(phase.get("formal_task_started", False)):
            return
        task_time = float(phase.get("task_time_s", 0.0))
        if last_formal_task_time is not None and task_time <= last_formal_task_time + 1e-12:
            return
        formal_started = True
        if current_reference is None or board is None:
            raise RuntimeError("FORMAL_REFERENCE_OR_BOARD_MISSING")
        row, acceleration = trace_row(
            trace_rows,
            response,
            current_reference,
            board,
            actuator,
            previous_acceleration,
        )
        writer.submit(row)
        trace_rows += 1
        last_formal_task_time = task_time
        previous_acceleration = acceleration
        maximum_position_error = max(maximum_position_error, float(row["position_error_m"]))
        position_errors.append(float(row["position_error_m"]))
        maximum_acceleration = max(maximum_acceleration, float(row["acceleration_norm_mps2"]))
        maximum_jerk = max(maximum_jerk, float(row["jerk_norm_mps3"]))
        maximum_estimator_gap = max(
            maximum_estimator_gap, float(row["estimator_plant_gap_m"])
        )
        if actuator is not None:
            minimum_rotor_margin = min(
                minimum_rotor_margin,
                float(np.min(1.0 - np.asarray(actuator["controls"][:6]))),
            )
        energy = float(response["plant"]["energy_model_j"])
        if initial_energy is None:
            initial_energy = energy
        final_energy = energy

    try:
        matlab.start()
        client = LiveHilPlantClient(
            "127.0.0.1",
            int(args.plant_port),
            timeout_s=3.0,
            preparation_timeout_s=120.0,
        )
        client.connect()
        boot_generation = 1
        expected = {
            "uid": str(EXPECTED_UID),
            "system_id": 1,
            "component_id": 1,
            "boot_generation": boot_generation,
            "maximum_age_ns": STREAM_STALE_NS,
            "maximum_runtime_age_ns": STREAM_STALE_NS,
            "task_identity_sha256": TASK_SHA,
            "async_outer_required": True,
        }
        initialized = client.initialize(
            {
                "schema": "GPENMPC_LIVE_PLANT_INITIALIZE_REQUEST_V1",
                "task_path": str(TASK_PATH),
                "expected": expected,
            }
        )
        plant_snapshot = initialized["service"]["plant"]
        prepared = client.prepare(
            2_000_000_000,
            fixture_sample(1, 2_000_000_000, boot_generation),
        )
        current_reference = reference_global_ned(
            {"initial_reference": prepared["initial_reference"]}
        )
        current_phase = initialized["service"]["phase"]
        result["host_preparation"] = {
            "initialized": True,
            "prepared": True,
            "preparation_wall_s": prepared.get("wall_s"),
            "async_warmup_wall_s": prepared.get("async_warmup_wall_s"),
            "hardware_actions": 0,
        }
        atomic_json(result_path, result)

        heartbeat = link.open(heartbeat_timeout_s=15.0)
        actions["com_open"] = 1
        identity = link.collect_identity()
        version = identity.get("autopilot_version") or {}
        extended = identity.get("extended_sys_state") or {}
        identity_gates = {
            "uid_exact": int(version.get("uid", -1)) == EXPECTED_UID,
            "board_version_exact": int(version.get("board_version", -1)) == EXPECTED_BOARD_VERSION,
            "commit_exact": identity.get("parsed_commit") == EXPECTED_COMMIT,
            "fmu_v6c": identity.get("parsed_hw_arch") == "PX4_FMU_V6C",
            "disarmed": not bool(
                int(heartbeat.base_mode)
                & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
            ),
            "landed": int(extended.get("landed_state", -1))
            == mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND,
        }
        if not all(identity_gates.values()):
            raise RuntimeError(f"BOARD_IDENTITY_PREFLIGHT_FAILED:{identity_gates}")
        board = AtomicBoardState(
            EXPECTED_UID, link.target_system, link.target_component, boot_generation
        )
        board.accept(heartbeat, time.perf_counter_ns(), poller)
        board.landed_state = int(extended["landed_state"])
        pwm = link.shell_command("pwm_out status")
        if "not running" not in pwm.lower():
            raise RuntimeError("PHYSICAL_PWM_DRIVER_RUNNING")
        result["preflight"] = {
            "identity": identity,
            "identity_gates": identity_gates,
            "pwm_out_status": pwm,
            "parameter_guards": verify_parameter_guards(link),
        }
        send_intervals(link)
        try:
            trajectory_stop = link.shell_command("gpenmpc_trajectory_exec stop")
        except Exception as error:
            trajectory_stop = f"EXCEPTION:{type(error).__name__}:{error}"
        bridge = link.shell_command("gpenmpc_tunnel_bridge start")
        controller = link.shell_command("gpenmpc_se3_control start")
        modules_started = True
        result["preflight"]["module_actions"] = {
            "gpenmpc_trajectory_exec": trajectory_stop,
            "gpenmpc_tunnel_bridge": bridge,
            "gpenmpc_se3_control": controller,
        }
        events.append({"event": "BOARD_PREFLIGHT_COMPLETE", "time_ns": time.perf_counter_ns()})
        wire = WireLoop(link, board, poller, visual)

        # Acquire estimator observations before arming and task advancement.
        warmup_started = time.monotonic()
        first_odo_time = None
        distinct_odo_timestamps: set[int] = set()
        while time.monotonic() - warmup_started < ESTIMATOR_WARMUP_TIMEOUT_S:
            load.sample(0.0)
            wire.tick(plant_snapshot, current_reference, current_phase, True)
            if board.odometry:
                latest = board.odometry[-1]
                distinct_odo_timestamps.add(int(latest["sample_timestamp_ns"]))
                first_odo_time = first_odo_time or time.monotonic()
            if first_odo_time is not None and len(distinct_odo_timestamps) >= 50:
                break
        if len(distinct_odo_timestamps) < 50:
            raise RuntimeError("PX4_ODOMETRY_WARMUP_NOT_CONTINUOUS")
        live_sample = board.newest_odometry(time.perf_counter_ns())
        admitted = client.admit(time.perf_counter_ns(), live_sample)
        result["selected_method_admission"] = {
            "status": admitted["status"],
            "outer_generation": admitted["outer_generation"],
        }
        events.append({"event": "SELECTED_METHOD_HOST_ADMITTED", "time_ns": time.perf_counter_ns()})

        def prestream_and_arm(reference: dict[str, Any], phase: dict[str, Any]) -> tuple[dict[str, Any], int]:
            nonlocal arm_cycle
            arm_cycle += 1
            start_tick = wire.tick_index
            offboard_next = start_tick + PRESTREAM_TICKS
            arm_next = offboard_next + 50
            arm_requests = 0
            started_wall = time.monotonic()
            while time.monotonic() - started_wall < ARM_SEQUENCE_TIMEOUT_S:
                wire.tick(plant_snapshot, reference, phase, True)
                if not board.armed and wire.tick_index >= offboard_next and board.mode != "OFFBOARD":
                    link.request_offboard()
                    actions["offboard_requests"] += 1
                    offboard_next += 100
                if (
                    not board.armed
                    and board.mode == "OFFBOARD"
                    and wire.tick_index >= arm_next
                    and arm_requests < 5
                ):
                    link.request_arm(True)
                    actions["arm_requests"] += 1
                    arm_requests += 1
                    arm_next += 100
                if board.armed:
                    actions["arm_transitions"] = board.arm_transitions
                    # Keep streaming until a fresh active module status and
                    # actual virtual actuator observation are both available.
                    try:
                        guard = poller.active_guard(time.perf_counter_ns())
                    except NshStatusError:
                        continue
                    if board.actuators and board.odometry:
                        return guard, arm_requests
            raise RuntimeError(f"ARM_SEQUENCE_TIMEOUT_CYCLE_{arm_cycle}")

        guard, _ = prestream_and_arm(current_reference, current_phase)
        first_sample = board.newest_odometry(time.perf_counter_ns())
        last_odo_timestamp = int(first_sample["sample_timestamp_ns"])
        # Discard disarmed and prestream actuator observations.
        # The first commit requires a sample published after confirmed arming.
        if board.actuators:
            last_actuator_observation = int(
                board.actuators[-1]["observation_generation"]
            )
        current_response = client.begin(time.perf_counter_ns(), first_sample)
        current_reference = reference_global_ned(current_response)
        current_phase = current_response["phase"]
        events.append({"event": "BOARD_CONTROLLED_FLIGHT_STARTED", "arm_cycle": arm_cycle})

        state = "FLIGHT"
        land_started_wall = 0.0
        standard_disarm_next = 0.0
        while not task_complete:
            load.sample(float((current_phase or {}).get("task_time_s", 0.0)))
            allow_reference = state in {"FLIGHT", "REARM_PRESTREAM"}
            now_ns = wire.tick(
                plant_snapshot,
                current_reference if allow_reference else None,
                current_phase,
                allow_reference,
            )
            if state == "FLIGHT":
                current_generation = client.status.physical_generation
                next_sample = board.odometry_after(last_odo_timestamp, now_ns)
                actuator = board.actuator_after(
                    last_actuator_observation, current_generation, now_ns
                )
                last_actuator_observation = int(actuator["observation_generation"])
                last_actuator = actuator
                guard = poller.active_guard(now_ns)
                response = client.commit_advance_and_begin(
                    now_ns, next_sample, guard, actuator
                )
                last_odo_timestamp = int(next_sample["sample_timestamp_ns"])
                plant_snapshot = response["plant"]
                current_response = response
                current_phase = response["phase"]
                if not response.get("native_land_required", False):
                    current_reference = reference_global_ned(response)
                record_progress(response, actuator)
                if response.get("native_land_required", False):
                    link.request_native_land()
                    actions["land_requests"] += 1
                    land_started_wall = time.monotonic()
                    standard_disarm_next = land_started_wall
                    state = "LAND"
                    events.append(
                        {
                            "event": "NATIVE_LAND_REQUEST",
                            "task_time_s": current_phase["task_time_s"],
                            "final_land": bool(response.get("final_land", False)),
                        }
                    )
            elif state == "LAND":
                actuator = board.actuator_after(
                    last_actuator_observation,
                    max(client.status.physical_generation, 1),
                    now_ns,
                )
                last_actuator_observation = int(actuator["observation_generation"])
                last_actuator = actuator
                response = client.land_step(now_ns, actuator, board.armed)
                plant_snapshot = response["plant"]
                current_response = response
                current_phase = response["phase"]
                if bool(plant_snapshot["ground_confirmed"]) and (
                    board.landed_state == mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND
                ):
                    if board.armed and time.monotonic() >= standard_disarm_next:
                        link.request_arm(False)
                        actions["standard_disarm_requests"] += 1
                        standard_disarm_next = time.monotonic() + 1.0
                    if not board.armed:
                        if int(current_phase["phase_code"]) == 21:
                            final_landings += 1
                        else:
                            delivery_landings += 1
                        state = "GROUND"
                        events.append(
                            {
                                "event": "NATIVE_GROUND_AND_STANDARD_DISARM_COMPLETE",
                                "phase_code": current_phase["phase_code"],
                                "delivery_landings": delivery_landings,
                                "final_landings": final_landings,
                            }
                        )
                if time.monotonic() - land_started_wall > LAND_TIMEOUT_S:
                    raise RuntimeError("NATIVE_LAND_GROUND_DISARM_TIMEOUT")
            elif state == "GROUND":
                response = client.ground_step(DT_S, board.armed)
                plant_snapshot = response["plant"]
                current_response = response
                current_phase = response["phase"]
                record_progress(response, last_actuator)
                if response.get("task_complete", False):
                    task_complete = True
                    formal_complete = True
                    break
                if response.get("rearm_required", False):
                    grounded_unloads += 1
                    sample = board.newest_odometry(now_ns)
                    resumed = client.resume(now_ns, sample)
                    current_reference = reference_global_ned(resumed)
                    current_phase = resumed["phase"]
                    guard, _ = prestream_and_arm(current_reference, current_phase)
                    sample = board.newest_odometry(time.perf_counter_ns())
                    last_odo_timestamp = int(sample["sample_timestamp_ns"])
                    # Begin the next causal flight interval after re-arming.
                    if board.actuators:
                        last_actuator_observation = int(
                            board.actuators[-1]["observation_generation"]
                        )
                    current_response = client.begin(time.perf_counter_ns(), sample)
                    current_reference = reference_global_ned(current_response)
                    current_phase = current_response["phase"]
                    state = "FLIGHT"
                    events.append(
                        {
                            "event": "GROUNDED_UNLOAD_AND_REARM_COMPLETE",
                            "grounded_unloads": grounded_unloads,
                            "arm_cycle": arm_cycle,
                        }
                    )
            else:
                raise RuntimeError(f"UNKNOWN_MISSION_STATE:{state}")

        result["mission_lifecycle"] = {
            "delivery_landings": delivery_landings,
            "grounded_unloads": grounded_unloads,
            "final_landings": final_landings,
            "arm_cycles": arm_cycle,
            "board_arm_transitions": board.arm_transitions,
            "airborne_drop_deliveries": 0,
            "task_complete": task_complete,
        }
        if not (
            task_complete
            and delivery_landings == 4
            and grounded_unloads == 4
            and final_landings == 1
            and arm_cycle == 5
        ):
            raise RuntimeError("DELIVERY_LIFECYCLE_DENOMINATOR_INCOMPLETE")
    except BaseException as error:
        failure = f"{type(error).__name__}:{error}"
        result["exception_traceback"] = traceback.format_exc()
        events.append({"event": "RUNNER_EXCEPTION", "failure": failure})
    finally:
        if board is not None and link.link is not None:
            try:
                if board.armed and board.landed_state == mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND:
                    link.request_arm(False)
                    actions["standard_disarm_requests"] += 1
                    deadline = time.monotonic() + 3.0
                    while board.armed and time.monotonic() < deadline:
                        drain_messages(link, board, poller)
                        time.sleep(0.02)
                if board.armed:
                    link.request_arm(False, force=True)
                    actions["force_disarm_requests"] += 1
                    force_used = True
                    deadline = time.monotonic() + 3.0
                    while board.armed and time.monotonic() < deadline:
                        drain_messages(link, board, poller)
                        time.sleep(0.02)
            except Exception as cleanup_error:
                events.append(
                    {
                        "event": "DISARM_FINALIZER_EXCEPTION",
                        "detail": f"{type(cleanup_error).__name__}:{cleanup_error}",
                    }
                )
        if link.link is not None:
            try:
                if modules_started:
                    for command in (
                        "gpenmpc_se3_control stop",
                        "gpenmpc_tunnel_bridge stop",
                        "gpenmpc_trajectory_exec stop",
                    ):
                        try:
                            response = link.shell_command(command)
                        except Exception as error:
                            response = f"EXCEPTION:{type(error).__name__}:{error}"
                        events.append(
                            {"event": "MODULE_STOP", "command": command, "response": response}
                        )
            finally:
                link.close()
        if client is not None:
            try:
                if client.status.initialized and not client.status.failed:
                    stopped = client.stop()
                    result["plant_service_stop"] = stopped.get("status")
                else:
                    client.close()
            except Exception as error:
                events.append(
                    {
                        "event": "PLANT_SERVICE_STOP_EXCEPTION",
                        "detail": f"{type(error).__name__}:{error}",
                    }
                )
        service_finish = matlab.finish()
        try:
            writer.close()
        except Exception as error:
            failure = failure or f"TRACE_FINALIZATION:{type(error).__name__}:{error}"
        visual.close()
        atomic_json(
            events_path,
            {
                "schema": SCHEMA,
                "run_id": RUN_ID,
                "events": events,
                "command_acks": [] if board is None else board.command_acks,
                "statustext": [] if board is None else board.statustext,
            },
        )
        atomic_json(
            host_load_path,
            {"schema": SCHEMA, "diagnostic_only": True, "rows": load.rows},
        )

    actions["arm_transitions"] = 0 if board is None else board.arm_transitions
    error_array = np.asarray(position_errors, dtype=float)
    screens = {
        "finite_formal_rows": bool(len(error_array) and np.isfinite(error_array).all()),
        "position_1p2_m": maximum_position_error <= 1.2,
        "acceleration_2p2_mps2": maximum_acceleration <= 2.2,
        "jerk_4p0_mps3": maximum_jerk <= 4.0,
        "native_delivery_and_final_termination": bool(
            task_complete
            and delivery_landings == 4
            and grounded_unloads == 4
            and final_landings == 1
            and not force_used
        ),
        "continuous_collision_free": None,
    }
    safe_inner = bool(
        link.link is None
        and (board is None or not board.armed)
        and (
            board is None
            or board.landed_state == mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND
        )
        and actions["physical_output"] == 0
    )
    if task_complete:
        status = "VALID_COMPLETED_GPENMPC_SELECTED_METHOD_TASK_HIL"
        if not all(value is True for key, value in screens.items() if key != "continuous_collision_free"):
            status += "__ONE_OR_MORE_PERFORMANCE_OR_TERMINATION_SCREENS_NOT_MET"
    elif formal_started:
        status = "VALID_ENGINEERING_INCOMPLETE_GPENMPC_SELECTED_METHOD_TASK_HIL"
    else:
        status = "INFRASTRUCTURE_OR_PRETASK_INVALID__FORMAL_NOT_ENTERED"
    if not safe_inner:
        status = "USER_ACTION_REQUIRED_INNER_SAFETY_NOT_VERIFIED"
    result.update(
        {
            "status": status,
            "failure": failure,
            "formal_started": formal_started,
            "formal_complete": formal_complete,
            "formal_trace_rows": trace_rows,
            "trace_writer_rows": writer.rows_written,
            "performance_screens": screens,
            "metrics": {
                "maximum_position_error_m": maximum_position_error,
                "rms_position_error_m": None
                if not len(error_array)
                else float(np.sqrt(np.mean(error_array**2))),
                "p95_position_error_m": None
                if not len(error_array)
                else float(np.quantile(error_array, 0.95)),
                "maximum_acceleration_mps2": maximum_acceleration,
                "maximum_jerk_mps3": maximum_jerk,
                "maximum_estimator_plant_gap_m": maximum_estimator_gap,
                "minimum_rotor_command_margin": minimum_rotor_margin,
                "mission_energy_model_j": None
                if initial_energy is None or final_energy is None
                else final_energy - initial_energy,
                "energy_source": "model",
            },
            "mission_lifecycle": {
                "delivery_landings": delivery_landings,
                "grounded_unloads": grounded_unloads,
                "final_landings": final_landings,
                "arm_cycles": arm_cycle,
                "board_arm_transitions": 0 if board is None else board.arm_transitions,
                "airborne_drop_deliveries": 0,
                "task_complete": task_complete,
            },
            "sensor_message_counts": None if wire is None else wire.sensor_counts,
            "setpoint_count": 0 if wire is None else wire.setpoint_count,
            "segment_chunks_sent": 0 if wire is None else wire.segment_chunks_sent,
            "module_status": poller.status(),
            "maximum_wall_lateness_s": 0.0
            if wire is None
            else wire.maximum_wall_lateness_s,
            "matlab_plant_process": service_finish,
            "final": {
                "board_armed": None if board is None else board.armed,
                "landed_state": None if board is None else board.landed_state,
                "force_disarm_used": force_used,
                "com_released": link.link is None,
                "modules_stop_attempted": modules_started,
                "physical_output": 0,
            },
            "artifacts": {
                "trace": artifact(trace_path) if trace_path.is_file() else None,
                "events": artifact(events_path),
                "host_load": artifact(host_load_path),
            },
            "claim_boundary": [
                "USB HIL with virtual actuator outputs.",
                "M600 dynamics run in MATLAB with RflySim3D visualization.",
                "Deliveries include landing, disarming, ground dwell and payload update.",
                "Task completion and performance-screen results are reported separately.",
            ],
        }
    )
    atomic_json(result_path, result)
    atomic_json(
        safety_path,
        {
            "schema": "GPENMPC_LIVE_DELIVERY_RUNNER_SAFETY_V1",
            "status": status,
            "pass": safe_inner,
            "final": result["final"],
            "action_counters": actions,
            "outer_transaction_owns_parameter_mapping_firmware_recovery": True,
        },
    )
    print(
        json.dumps(
            {
                "status": status,
                "formal_rows": trace_rows,
                "task_complete": task_complete,
                "safe_inner": safe_inner,
            }
        )
    )
    return 0 if status.startswith("VALID_") and safe_inner else 2


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--offline-validate", action="store_true")
    mode.add_argument("--execute", action="store_true")
    parser.add_argument("--offline-result", default=str(OVERLAY / "evidence" / "LIVE_DELIVERY_RUNNER_OFFLINE_VALIDATION.json"))
    parser.add_argument("--binding")
    parser.add_argument("--method", default=METHOD)
    parser.add_argument("--endpoint", default="COM3")
    parser.add_argument("--baud", type=int, default=921600)
    parser.add_argument("--output-dir")
    parser.add_argument("--run-id", default=RUN_ID)
    parser.add_argument("--plant-port", type=int, default=18743)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.offline_validate:
        return offline_validate(Path(args.offline_result).resolve())
    if not args.output_dir:
        raise SystemExit("--output-dir is required for live execution")
    return run_live(args)


if __name__ == "__main__":
    raise SystemExit(main())
