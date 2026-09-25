"""Strict loopback client for one authoritative MATLAB M600 HIL plant.

The client contains no serial, MAVLink, parameter, mode, arm, or PWM API.
Only the later single-owner HIL runner may compose it with those endpoints.
"""
from __future__ import annotations

import json
import socket
from dataclasses import dataclass
from typing import Any


class LiveHilPlantError(RuntimeError):
    """Fail-closed service, IPC, or sequencing failure."""


@dataclass(frozen=True)
class LiveHilPlantClientStatus:
    next_request_id: int
    initialized: bool
    prepared: bool
    admitted: bool
    physical_generation: int
    physical_open: bool
    failed: bool


class LiveHilPlantClient:
    def __init__(
        self,
        address: str,
        port: int,
        *,
        timeout_s: float = 1.0,
        preparation_timeout_s: float = 90.0,
    ) -> None:
        if address != "127.0.0.1":
            raise ValueError("live plant service must use IPv4 loopback")
        if not 1 <= int(port) <= 65535:
            raise ValueError("invalid port")
        if not 0.0 < float(timeout_s) <= 10.0:
            raise ValueError("timeout outside bounded range")
        if not float(timeout_s) <= float(preparation_timeout_s) <= 120.0:
            raise ValueError("preparation timeout outside bounded range")
        self._address = address
        self._port = int(port)
        self._timeout_s = float(timeout_s)
        self._preparation_timeout_s = float(preparation_timeout_s)
        self._socket: socket.socket | None = None
        self._next_request_id = 0
        self._initialized = False
        self._prepared = False
        self._admitted = False
        self._physical_generation = 0
        self._physical_open = False
        self._failed = False

    @property
    def status(self) -> LiveHilPlantClientStatus:
        return LiveHilPlantClientStatus(
            self._next_request_id,
            self._initialized,
            self._prepared,
            self._admitted,
            self._physical_generation,
            self._physical_open,
            self._failed,
        )

    def connect(self) -> None:
        if self._socket is not None:
            raise LiveHilPlantError("client already connected")
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            sock.bind(("127.0.0.1", 0))
            sock.connect((self._address, self._port))
            sock.settimeout(self._timeout_s)
            self._socket = sock
        except Exception:
            sock.close()
            self._failed = True
            raise

    def initialize(self, payload: dict[str, Any]) -> dict[str, Any]:
        response = self._transact(
            "INITIALIZE", payload, timeout_s=self._preparation_timeout_s
        )
        self._require_zero_authority(response)
        if response.get("status") != "INITIALIZED":
            return self._reject(f"initialization failed: {response}")
        self._initialized = True
        return response

    def prepare(self, now_ns: int, px4_sample: dict[str, Any]) -> dict[str, Any]:
        self._require_state(self._initialized and not self._prepared, "prepare state")
        response = self._transact(
            "PREPARE",
            {
                "schema": "GPENMPC_LIVE_PLANT_PREPARE_REQUEST_V1",
                "now_ns": int(now_ns),
                "px4_sample": px4_sample,
            },
            timeout_s=self._preparation_timeout_s,
        )
        self._require_zero_authority(response)
        if response.get("status") != "PASS_HOST_PREPARATION":
            return self._reject(f"preparation failed: {response}")
        self._prepared = True
        return response

    def admit(self, now_ns: int, px4_sample: dict[str, Any]) -> dict[str, Any]:
        self._require_state(self._prepared and not self._admitted, "admit state")
        response = self._transact(
            "ADMIT",
            {
                "schema": "GPENMPC_LIVE_PLANT_ADMIT_REQUEST_V1",
                "now_ns": int(now_ns),
                "px4_sample": px4_sample,
            },
        )
        self._require_zero_authority(response)
        if response.get("status") != "PASS_SELECTED_METHOD_ADMITTED":
            return self._reject(f"admission failed: {response}")
        self._admitted = True
        return response

    def begin(self, now_ns: int, px4_sample: dict[str, Any]) -> dict[str, Any]:
        self._require_state(self._admitted and not self._physical_open, "begin state")
        response = self._transact(
            "BEGIN",
            {
                "schema": "GPENMPC_LIVE_PLANT_BEGIN_REQUEST_V1",
                "now_ns": int(now_ns),
                "px4_sample": px4_sample,
            },
        )
        self._accept_open_reference(response)
        return response

    def commit_advance_and_begin(
        self,
        now_ns: int,
        px4_sample: dict[str, Any],
        module_guard: dict[str, Any],
        actuator_output: dict[str, Any],
    ) -> dict[str, Any]:
        self._require_state(self._physical_open, "commit state")
        response = self._transact(
            "COMMIT_ADVANCE_AND_BEGIN",
            {
                "schema": "GPENMPC_LIVE_PLANT_COMMIT_ADVANCE_REQUEST_V1",
                "now_ns": int(now_ns),
                "px4_sample": px4_sample,
                "module_guard": module_guard,
                "actuator_output": actuator_output,
            },
        )
        self._require_zero_authority(response)
        commit = response.get("commit", {})
        if not commit.get("accepted", False):
            return self._reject(f"actual board control commit rejected: {response}")
        if response.get("native_land_required", False):
            if response.get("reference_publication_allowed", True):
                return self._reject("native LAND transition retained reference authority")
            self._physical_open = False
            return response
        self._accept_open_reference(response)
        return response

    def land_step(
        self, now_ns: int, actuator_output: dict[str, Any], board_armed: bool
    ) -> dict[str, Any]:
        self._require_state(self._admitted and not self._physical_open, "LAND state")
        response = self._transact(
            "LAND_STEP",
            {
                "schema": "GPENMPC_LIVE_PLANT_LAND_STEP_REQUEST_V1",
                "now_ns": int(now_ns),
                "actuator_output": actuator_output,
                "board_armed": bool(board_armed),
            },
        )
        self._require_zero_authority(response)
        if response.get("status") != "NATIVE_LAND_PLANT_ADVANCED" or response.get(
            "reference_publication_allowed", True
        ):
            return self._reject(f"LAND plant step failed: {response}")
        return response

    def ground_step(self, dt_s: float, board_armed: bool) -> dict[str, Any]:
        self._require_state(self._admitted and not self._physical_open, "ground state")
        response = self._transact(
            "GROUND_STEP",
            {
                "schema": "GPENMPC_LIVE_PLANT_GROUND_STEP_REQUEST_V1",
                "dt_s": float(dt_s),
                "board_armed": bool(board_armed),
            },
        )
        self._require_zero_authority(response)
        if response.get("reference_publication_allowed", True):
            return self._reject("ground step gained board reference authority")
        if response.get("status") not in {
            "GROUND_DWELL_RUNNING",
            "REARM_REQUIRED",
            "TASK_COMPLETE_ON_GROUND",
        }:
            return self._reject(f"ground step failed: {response}")
        return response

    def resume(self, now_ns: int, px4_sample: dict[str, Any]) -> dict[str, Any]:
        self._require_state(self._admitted and not self._physical_open, "resume state")
        response = self._transact(
            "RESUME",
            {
                "schema": "GPENMPC_LIVE_PLANT_RESUME_REQUEST_V1",
                "now_ns": int(now_ns),
                "px4_sample": px4_sample,
            },
        )
        self._require_zero_authority(response)
        if response.get("status") != "PASS_REARM_REFERENCE_READY":
            return self._reject(f"resume admission failed: {response}")
        return response

    def query_status(self) -> dict[str, Any]:
        response = self._transact("STATUS")
        self._require_zero_authority(response)
        return response

    def stop(self) -> dict[str, Any]:
        try:
            response = self._transact("STOP")
            self._require_zero_authority(response)
            return response
        finally:
            self.close()

    def close(self) -> None:
        sock, self._socket = self._socket, None
        if sock is not None:
            sock.close()

    def _accept_open_reference(self, response: dict[str, Any]) -> None:
        self._require_zero_authority(response)
        if response.get("status") != "REFERENCE_READY_FOR_SINGLE_BOARD_PUBLISHER":
            self._reject(f"reference publication failed: {response}")
        if not response.get("reference_publication_allowed", False):
            self._reject("reference publication was suppressed")
        generation = int(response.get("generation", -1))
        if generation != self._physical_generation + 1:
            self._reject("physical generation is not exact and gap-free")
        self._physical_generation = generation
        self._physical_open = True

    def _transact(
        self,
        action: str,
        payload: dict[str, Any] | None = None,
        *,
        timeout_s: float | None = None,
    ) -> dict[str, Any]:
        if self._socket is None:
            raise LiveHilPlantError("client is not connected")
        if self._failed:
            raise LiveHilPlantError("client is fail-closed")
        request_id = self._next_request_id
        message: dict[str, Any] = {"action": action, "request_id": request_id}
        if payload is not None:
            message["payload"] = payload
        try:
            encoded = json.dumps(
                message, separators=(",", ":"), allow_nan=False
            ).encode()
            if len(encoded) > 65507:
                raise LiveHilPlantError("request exceeds one UDP datagram")
            self._socket.settimeout(
                self._timeout_s if timeout_s is None else float(timeout_s)
            )
            self._socket.send(encoded)
            response = json.loads(self._socket.recv(65535).decode())
            self._socket.settimeout(self._timeout_s)
            if not isinstance(response, dict):
                raise LiveHilPlantError("response is not an object")
            if int(response.get("request_id", -1)) != request_id:
                raise LiveHilPlantError("stale, duplicate, or reordered response")
            if response.get("fail_closed", False):
                raise LiveHilPlantError(
                    f"service fail-closed: {response.get('failure_code', 'unknown')}"
                )
            self._next_request_id += 1
            return response
        except Exception as exc:
            self._failed = True
            self.close()
            if isinstance(exc, LiveHilPlantError):
                raise
            raise LiveHilPlantError(f"loopback transaction failed: {exc}") from exc

    @staticmethod
    def _require_zero_authority(response: dict[str, Any]) -> None:
        for field in (
            "hardware_actions",
            "com_open",
            "board_access",
            "parameter_writes",
            "mapping_writes",
            "arm_requests",
            "mode_requests",
            "task_rows",
            "physical_output_commands",
        ):
            if int(response.get(field, 0)) != 0:
                raise LiveHilPlantError(f"host service claimed nonzero {field}")

    def _require_state(self, condition: bool, label: str) -> None:
        if not condition:
            self._reject(f"invalid {label}")

    def _reject(self, message: str):
        self._failed = True
        self.close()
        raise LiveHilPlantError(message)

    def __enter__(self) -> "LiveHilPlantClient":
        self.connect()
        return self

    def __exit__(self, exc_type, exc, traceback) -> None:
        self.close()
