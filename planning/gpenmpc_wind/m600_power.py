"""M600 public phase-average power surface with strict source-domain checks.

The model deliberately stays at the resolution supported by DOE/INL Table B.1.
It is not a rotor-level identification and it never extrapolates outside the
published payload or forward-airspeed interpolation rectangle.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from .io_utils import canonical_bytes, load_json, sha256_bytes


PHASES = ("ASCEND", "DESCEND", "FORWARD", "HOVER")


@dataclass(frozen=True)
class M600PowerModel:
    profile: dict[str, Any]
    speed_knots_mps: tuple[float, float]
    payload_knots_kg: tuple[float, ...]
    values: dict[float, dict[float, tuple[float, ...]]]

    @classmethod
    def from_profile(cls, path: Path) -> "M600PowerModel":
        profile = load_json(path)
        if profile.get("platform_id") != "M600_PRO_OSTI_PUBLIC_POWER_REFERENCE":
            raise ValueError("Profile does not match the supported M600 public-power reference")
        table = profile["energy_model"]["phase_average_power_table_w"]
        expected_hash = str(table["table_sha256"]).upper()
        actual_hash = sha256_bytes(canonical_bytes(table["values_by_speed_and_payload"]))
        if expected_hash != actual_hash:
            raise ValueError("M600 public power table content hash mismatch")
        speeds = tuple(float(item) for item in table["speed_knots_mps"])
        payloads = tuple(float(item) for item in table["payload_knots_kg"])
        if speeds != (6.71, 13.41) or payloads != (0.0, 1.13, 2.27, 4.54):
            raise ValueError("Unexpected M600 public-data interpolation domain")
        raw = table["values_by_speed_and_payload"]
        values: dict[float, dict[float, tuple[float, ...]]] = {}
        for speed in speeds:
            speed_key = f"{speed:.2f}"
            values[speed] = {}
            for payload in payloads:
                payload_key = f"{payload:.2f}"
                row = tuple(float(item) for item in raw[speed_key][payload_key])
                if len(row) != len(PHASES) or not all(math.isfinite(item) and item > 0.0 for item in row):
                    raise ValueError("Invalid M600 phase-power row")
                values[speed][payload] = row
        model = cls(profile=profile, speed_knots_mps=(speeds[0], speeds[-1]), payload_knots_kg=payloads, values=values)
        model.validate_knots()
        return model

    @property
    def base_mass_kg(self) -> float:
        return float(self.profile["mass_properties"]["base_mass_kg"])

    @property
    def battery_energy_j(self) -> float:
        return float(self.profile["battery"]["initial_energy_j"])

    @property
    def planning_radius_m(self) -> float:
        return float(self.profile["collision_envelope"]["planning_radius_m"])

    @property
    def manufacturer_max_ground_speed_mps(self) -> float:
        return float(self.profile["limits"]["manufacturer_max_no_wind_speed_mps"])

    @property
    def manufacturer_wind_resistance_mps(self) -> float:
        return float(self.profile["limits"]["manufacturer_wind_resistance_mps"])

    @property
    def airspeed_min_mps(self) -> float:
        return self.speed_knots_mps[0]

    @property
    def airspeed_max_mps(self) -> float:
        return self.speed_knots_mps[1]

    def _bracket(self, value: float, knots: tuple[float, ...], label: str) -> tuple[float, float, float]:
        value = float(value)
        if not math.isfinite(value) or value < knots[0] - 1e-10 or value > knots[-1] + 1e-10:
            raise ValueError(f"{label} outside the public-data interpolation domain: {value}")
        value = min(knots[-1], max(knots[0], value))
        for left, right in zip(knots[:-1], knots[1:]):
            if value <= right + 1e-12:
                weight = 0.0 if right == left else (value - left) / (right - left)
                return left, right, min(1.0, max(0.0, weight))
        return knots[-1], knots[-1], 0.0

    def phase_power_w(self, phase: str, airspeed_mps: float, payload_kg: float) -> float:
        phase_key = str(phase).upper()
        if phase_key not in PHASES:
            raise ValueError(f"Unsupported M600 phase: {phase}")
        phase_index = PHASES.index(phase_key)
        s0, s1, ws = self._bracket(float(airspeed_mps), self.speed_knots_mps, "airspeed")
        p0, p1, wp = self._bracket(float(payload_kg), self.payload_knots_kg, "payload")

        def at(speed: float, payload: float) -> float:
            return self.values[speed][payload][phase_index]

        lower = (1.0 - wp) * at(s0, p0) + wp * at(s0, p1)
        upper = (1.0 - wp) * at(s1, p0) + wp * at(s1, p1)
        result = (1.0 - ws) * lower + ws * upper
        if not math.isfinite(result) or result <= 0.0:
            raise ValueError("Non-positive interpolated phase power")
        return result

    def validate_knots(self) -> None:
        for speed in self.speed_knots_mps:
            for payload in self.payload_knots_kg:
                for phase_index, phase in enumerate(PHASES):
                    expected = self.values[speed][payload][phase_index]
                    actual = self.phase_power_w(phase, speed, payload)
                    if abs(actual - expected) > 1e-9:
                        raise ValueError("M600 phase-power knot reproduction failed")

    def service(
        self,
        payload_before_kg: float,
        delivered_kg: float,
        service_duration_s: float,
        service_start_altitude_m: float,
    ) -> dict[str, float]:
        payload_before = float(payload_before_kg)
        delivered = float(delivered_kg)
        ground_duration = float(service_duration_s)
        start_altitude = float(service_start_altitude_m)
        service_target_altitude = self.planning_radius_m + 0.02
        if not math.isfinite(start_altitude):
            raise ValueError("Service start altitude must be finite")
        if not math.isfinite(ground_duration) or ground_duration < 0.0:
            raise ValueError("Ground service duration must be finite and nonnegative")
        if start_altitude < service_target_altitude - 1e-12:
            raise ValueError(
                "Service start altitude lies below the retained low service target"
            )
        payload_after = max(0.0, payload_before - delivered)
        vertical_distance = max(0.0, start_altitude - service_target_altitude)
        vertical_duration = vertical_distance / 1.0 + 1.0
        descent = self.phase_power_w("DESCEND", self.airspeed_min_mps, payload_before) * vertical_duration
        ascent = self.phase_power_w("ASCEND", self.airspeed_min_mps, payload_after) * vertical_duration
        ground = float(self.profile["energy_model"].get("ground_auxiliary_power_w", 0.0)) * ground_duration
        return {
            "payload_before_kg": payload_before,
            "delivered_kg": delivered,
            "payload_after_kg": payload_after,
            "service_start_altitude_m": start_altitude,
            "service_target_altitude_m": service_target_altitude,
            "vertical_distance_each_m": vertical_distance,
            "descent_duration_s": vertical_duration,
            "ground_service_duration_s": ground_duration,
            "ascent_duration_s": vertical_duration,
            "descent_energy_j": descent,
            "ground_service_energy_j": ground,
            "ascent_energy_j": ascent,
            "service_total_energy_j": descent + ground + ascent,
        }
