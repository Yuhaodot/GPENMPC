"""Strict loopback-only client for the host board-reference service.

This module contains no serial, MAVLink, PX4, parameter, mode, arm, or output
API. A single-COM HIL runner may use it only as an outer-reference sidecar.
"""
from __future__ import annotations

import json
import socket
from dataclasses import dataclass
from typing import Any


class BoardOuterServiceError(RuntimeError):
    """Fail-closed IPC or response-contract failure."""


@dataclass(frozen=True)
class BoardOuterClientStatus:
    next_request_id: int
    initialized: bool
    prepared: bool
    failed: bool


class BoardOuterUdpClient:
    def __init__(self, address: str, port: int, *, timeout_s: float = 1.0) -> None:
        if address != "127.0.0.1":
            raise ValueError("board outer service must be IPv4 loopback")
        if not 1 <= int(port) <= 65535:
            raise ValueError("invalid UDP port")
        if not 0.0 < float(timeout_s) <= 10.0:
            raise ValueError("timeout outside bounded range")
        self._address = address
        self._port = int(port)
        self._timeout_s = float(timeout_s)
        self._socket: socket.socket | None = None
        self._next_request_id = 0
        self._initialized = False
        self._prepared = False
        self._failed = False
        self._physical_generation = 0
        self._physical_open = False

    @property
    def status(self) -> BoardOuterClientStatus:
        return BoardOuterClientStatus(
            self._next_request_id, self._initialized, self._prepared, self._failed
        )

    def connect(self) -> None:
        if self._socket is not None:
            raise BoardOuterServiceError("client already connected")
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
        response = self._transact("INITIALIZE", payload)
        self._require_zero_authority(response)
        if response.get("status") != "INITIALIZED":
            return self._reject(f"initialization failed: {response}")
        if response.get("method") != "B2_ENMPC_GP_MEAN_TOTAL_TUBE":
            return self._reject("method identity mismatch")
        self._initialized = True
        return response

    def prepare(self, payload: dict[str, Any]) -> dict[str, Any]:
        if not self._initialized:
            raise BoardOuterServiceError("initialize before preparation")
        response = self._transact("PREPARE", payload)
        self._require_zero_authority(response)
        if response.get("status") != "PASS_HOST_PREPARATION":
            return self._reject(f"preparation failed: {response}")
        if response.get("generation_consumed") or response.get("command_committed"):
            return self._reject("preparation consumed runtime authority")
        self._prepared = True
        return response

    def update(self, payload: dict[str, Any]) -> dict[str, Any]:
        if not self._prepared:
            raise BoardOuterServiceError("prepare before update")
        response = self._transact("UPDATE", payload)
        self._require_zero_authority(response)
        if response.get("schema") != "GPENMPC_BOARD_OUTER_UPDATE_RESPONSE_V1":
            return self._reject("update response schema mismatch")
        if int(response.get("generation", -1)) != int(payload.get("generation", -2)):
            return self._reject("update generation mismatch")
        if response.get("failed") or response.get("hard_invalid"):
            return self._reject(f"outer service fail-closed: {response}")
        return response

    def async_update_submit(self, payload: dict[str, Any]) -> dict[str, Any]:
        if not self._prepared:
            raise BoardOuterServiceError("prepare before async update submit")
        response = self._transact("ASYNC_UPDATE_SUBMIT", payload)
        self._require_zero_authority(response)
        if response.get("status") != "SUBMITTED":
            return self._reject(f"async update submit failed: {response}")
        return response

    def async_update_poll(self, now_ns: int) -> dict[str, Any]:
        response = self._transact(
            "ASYNC_UPDATE_POLL",
            {
                "schema": "GPENMPC_BOARD_OUTER_ASYNC_POLL_REQUEST_V1",
                "now_ns": int(now_ns),
            },
        )
        self._require_zero_authority(response)
        if response.get("status") in {
            "FAIL_CLOSED_WORKER_ERROR",
            "REJECTED_BEFORE_SUBMIT",
        } or response.get("failed") or response.get("hard_invalid"):
            return self._reject(f"async outer service fail-closed: {response}")
        return response

    def reference_step(self, dt_s: float) -> dict[str, Any]:
        response = self._transact(
            "REFERENCE_STEP",
            {"schema": "GPENMPC_BOARD_REFERENCE_STEP_REQUEST_V1", "dt_s": dt_s},
        )
        self._require_zero_authority(response)
        if response.get("status") != "REFERENCE":
            return self._reject("reference step failed")
        reference = response.get("reference", {})
        if not reference.get("publication_allowed", False):
            return self._reject("reference publication suppressed")
        return response

    def physical_reference_step(self, payload: dict[str, Any]) -> dict[str, Any]:
        if not self._prepared:
            raise BoardOuterServiceError("prepare before physical reference step")
        if self._physical_open:
            return self._reject("commit current physical control before next reference")
        generation = int(payload.get("generation", -1))
        if generation != self._physical_generation + 1:
            return self._reject("physical generation must be exact and gap-free")
        response = self._transact("PHYSICAL_REFERENCE_STEP", payload)
        self._require_zero_authority(response)
        if response.get("schema") != "GPENMPC_BOARD_PHYSICAL_REFERENCE_RESPONSE_V1":
            return self._reject("physical reference response schema mismatch")
        if int(response.get("generation", -2)) != generation:
            return self._reject("physical reference generation mismatch")
        if not response.get("accepted") or not response.get("publication_allowed"):
            return self._reject(f"physical reference fail-closed: {response}")
        if response.get("board_control_mode") != "RA_CTRL_MODE_NOMINAL_SINGLE_PUBLISHER":
            return self._reject("physical reference selected an unapproved board mode")
        if response.get("compute_partition") != (
            "HOST_A1_AUGMENTATION__BOARD_NOMINAL_POSITION_CONTROL"
        ):
            return self._reject("physical compute partition mismatch")
        self._physical_generation = generation
        self._physical_open = True
        return response

    def ground_reference_step(self, dt_s: float) -> dict[str, Any]:
        if not self._prepared:
            raise BoardOuterServiceError("prepare before ground reference step")
        if self._physical_open:
            return self._reject("commit current physical control before ground step")
        response = self._transact(
            "GROUND_REFERENCE_STEP",
            {
                "schema": "GPENMPC_BOARD_GROUND_REFERENCE_REQUEST_V1",
                "dt_s": float(dt_s),
                "ground_confirmed": True,
                "board_armed": False,
            },
        )
        self._require_zero_authority(response)
        if response.get("status") != "GROUND_CLOCK_ADVANCED_NO_PUBLICATION":
            return self._reject(f"ground reference step failed: {response}")
        if response.get("publication_allowed") or int(
            response.get("board_reference_publication_count", -1)
        ) != 0:
            return self._reject("ground reference gained board publication authority")
        return response

    def physical_control_commit(self, payload: dict[str, Any]) -> dict[str, Any]:
        generation = int(payload.get("generation", -1))
        if not self._physical_open or generation != self._physical_generation:
            return self._reject("no matching physical sample is open")
        response = self._transact("PHYSICAL_CONTROL_COMMIT", payload)
        self._require_zero_authority(response)
        if response.get("schema") != (
            "GPENMPC_BOARD_PHYSICAL_CONTROL_COMMIT_RESPONSE_V1"
        ):
            return self._reject("physical control commit response schema mismatch")
        if int(response.get("generation", -2)) != generation:
            return self._reject("physical control commit generation mismatch")
        if not response.get("accepted"):
            return self._reject(f"physical control commit fail-closed: {response}")
        self._physical_open = False
        return response

    def physical_v7_control_commit(self, payload: dict[str, Any]) -> dict[str, Any]:
        """Commit observed V7 status plus actual MAVLink actuator output.

        Unlike ``physical_control_commit``, this path does not accept a
        caller-constructed desired force or rotor command.  The MATLAB
        service reconstructs the frozen V7 nominal force from its open
        atomic estimate/reference and maps only MAVLink message 93.
        """
        generation = int(payload.get("generation", -1))
        if not self._physical_open or generation != self._physical_generation:
            return self._reject("no matching physical sample is open")
        response = self._transact("PHYSICAL_V7_CONTROL_COMMIT", payload)
        self._require_zero_authority(response)
        if response.get("schema") != (
            "GPENMPC_BOARD_V7_PHYSICAL_CONTROL_COMMIT_RESPONSE_V1"
        ):
            return self._reject("V7 physical control commit response schema mismatch")
        if int(response.get("generation", -2)) != generation:
            return self._reject("V7 physical control commit generation mismatch")
        if not response.get("accepted"):
            return self._reject(f"V7 physical control commit fail-closed: {response}")
        if response.get("actual_actuator_message_id") != 93:
            return self._reject("V7 commit is not bound to HIL_ACTUATOR_CONTROLS 93")
        self._physical_open = False
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

    def _transact(self, action: str, payload: dict[str, Any] | None = None) -> dict[str, Any]:
        if self._socket is None:
            raise BoardOuterServiceError("client is not connected")
        if self._failed:
            raise BoardOuterServiceError("client is fail-closed")
        request_id = self._next_request_id
        message: dict[str, Any] = {"action": action, "request_id": request_id}
        if payload is not None:
            message["payload"] = payload
        try:
            encoded = json.dumps(message, separators=(",", ":"), allow_nan=False).encode()
            if len(encoded) > 65507:
                raise BoardOuterServiceError("request exceeds one UDP datagram")
            self._socket.send(encoded)
            response = json.loads(self._socket.recv(65535).decode())
            if not isinstance(response, dict):
                raise BoardOuterServiceError("response is not an object")
            if int(response.get("request_id", -1)) != request_id:
                raise BoardOuterServiceError("stale, reordered, or duplicate response")
            if response.get("fail_closed", False):
                raise BoardOuterServiceError(
                    f"service fail-closed: {response.get('failure_code', 'unknown')}"
                )
            self._next_request_id += 1
            return response
        except Exception as exc:
            self._failed = True
            self.close()
            if isinstance(exc, BoardOuterServiceError):
                raise
            raise BoardOuterServiceError(f"loopback transaction failed: {exc}") from exc

    @staticmethod
    def _require_zero_authority(response: dict[str, Any]) -> None:
        fields = (
            "hardware_actions",
            "com_open",
            "board_access",
            "parameter_writes",
            "mapping_writes",
            "arm_requests",
            "mode_requests",
            "task_rows",
            "physical_output_commands",
        )
        for field in fields:
            if int(response.get(field, 0)) != 0:
                raise BoardOuterServiceError(f"service claimed nonzero {field}")

    def _reject(self, message: str):
        self._failed = True
        self.close()
        raise BoardOuterServiceError(message)

    def __enter__(self) -> "BoardOuterUdpClient":
        self.connect()
        return self

    def __exit__(self, exc_type, exc, traceback) -> None:
        self.close()
