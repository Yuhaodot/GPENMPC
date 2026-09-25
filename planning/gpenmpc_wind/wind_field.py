"""Deterministic horizontal wind forecasts and bounded-error interfaces."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any


def _unit(vector: tuple[float, float]) -> tuple[float, float]:
    norm = math.hypot(*vector)
    if norm <= 1e-12:
        raise ValueError("Cannot normalize a zero vector")
    return vector[0] / norm, vector[1] / norm


@dataclass(frozen=True)
class WindScenario:
    scenario_id: str
    kind: str
    definition: dict[str, Any]
    anchor_unit_xy: tuple[float, float]
    error_vertices_xy_mps: tuple[tuple[float, float], ...]
    error_temporal_coupling: str
    claim_boundary: str = "Prescribed horizontal wind scenario with bounded forecast error."

    @property
    def uses_bounded_error(self) -> bool:
        return bool(self.definition.get("use_forecast_error", False))

    @property
    def planning_robust(self) -> bool:
        return self.uses_bounded_error

    def declared_maximum_wind_magnitude_mps(self) -> float:
        """Return a conservative analytic envelope for mean wind plus error.

        Uniform, route-relative, and gust fields are evaluated at their exact
        vector endpoints.  The linear spatial field publishes only a scalar
        maximum-mean envelope, so its error contribution is combined by the
        triangle inequality.
        """

        item = self.definition
        errors = self.uncertainty_vertices()
        if self.kind == "LINEAR_SPATIAL_VECTOR_FIELD":
            maximum_mean = float(item["maximum_mean_magnitude_mps"])
            maximum_error = max(math.hypot(*error) for error in errors)
            return maximum_mean + maximum_error
        if self.kind in {"UNIFORM", "UNIFORM_WITH_BOUNDED_ERROR"}:
            base_vectors = [(float(item["mean_xy_mps"][0]), float(item["mean_xy_mps"][1]))]
        elif self.kind == "ROUTE_RELATIVE":
            base_vectors = [self.mean_xy_mps(0.0, 0.0, 0.0, 1.0)]
        elif self.kind == "UNIFORM_PLUS_ONE_MINUS_COSINE_GUST":
            base = (float(item["mean_xy_mps"][0]), float(item["mean_xy_mps"][1]))
            direction = _unit((float(item["gust_direction_xy"][0]), float(item["gust_direction_xy"][1])))
            amplitude = float(item["gust_amplitude_mps"])
            base_vectors = [base, (base[0] + amplitude * direction[0], base[1] + amplitude * direction[1])]
        else:
            raise ValueError(f"Unsupported wind scenario kind: {self.kind}")
        return max(
            math.hypot(base[0] + error[0], base[1] + error[1])
            for base in base_vectors
            for error in errors
        )

    def mean_xy_mps(self, x_m: float, y_m: float, local_time_s: float, leg_cruise_duration_s: float) -> tuple[float, float]:
        kind = self.kind
        item = self.definition
        if kind in {"UNIFORM", "UNIFORM_WITH_BOUNDED_ERROR"}:
            return float(item["mean_xy_mps"][0]), float(item["mean_xy_mps"][1])
        if kind == "ROUTE_RELATIVE":
            magnitude = float(item["mean_magnitude_mps"])
            ux, uy = self.anchor_unit_xy
            direction = str(item["relative_direction"])
            if direction == "HEADWIND":
                return -magnitude * ux, -magnitude * uy
            if direction == "TAILWIND":
                return magnitude * ux, magnitude * uy
            if direction == "LEFT_CROSSWIND":
                return -magnitude * uy, magnitude * ux
            raise ValueError(f"Unsupported route-relative direction: {direction}")
        if kind == "LINEAR_SPATIAL_VECTOR_FIELD":
            origin = item["origin_xy_m"]
            base = item["base_xy_mps"]
            gradient = item["gradient_per_s"]
            dx = float(x_m) - float(origin[0])
            dy = float(y_m) - float(origin[1])
            wx = float(base[0]) + float(gradient[0][0]) * dx + float(gradient[0][1]) * dy
            wy = float(base[1]) + float(gradient[1][0]) * dx + float(gradient[1][1]) * dy
            if math.hypot(wx, wy) > float(item["maximum_mean_magnitude_mps"]) + 1e-9:
                raise ValueError("Linear spatial wind exceeds its declared wind-magnitude envelope")
            return wx, wy
        if kind == "UNIFORM_PLUS_ONE_MINUS_COSINE_GUST":
            base = (float(item["mean_xy_mps"][0]), float(item["mean_xy_mps"][1]))
            direction = _unit((float(item["gust_direction_xy"][0]), float(item["gust_direction_xy"][1])))
            duration = float(item["gust_duration_s"])
            start = float(item["gust_start_fraction_of_leg_cruise"]) * max(0.0, float(leg_cruise_duration_s))
            time = float(local_time_s)
            gain = 0.0
            if duration > 0.0 and start <= time <= start + duration:
                phase = (time - start) / duration
                gain = 0.5 * (1.0 - math.cos(2.0 * math.pi * phase))
            amplitude = float(item["gust_amplitude_mps"]) * gain
            return base[0] + amplitude * direction[0], base[1] + amplitude * direction[1]
        raise ValueError(f"Unsupported wind scenario kind: {kind}")

    def uncertainty_vertices(self) -> tuple[tuple[float, float], ...]:
        return self.error_vertices_xy_mps if self.uses_bounded_error else ((0.0, 0.0),)

    def descriptor(self) -> dict[str, Any]:
        return {
            "scenario_id": self.scenario_id,
            "kind": self.kind,
            "anchor_unit_xy": list(self.anchor_unit_xy),
            "uses_bounded_error": self.uses_bounded_error,
            "error_vertices_xy_mps": [list(item) for item in self.uncertainty_vertices()],
            "error_temporal_coupling": self.error_temporal_coupling,
            "declared_maximum_wind_magnitude_mps": self.declared_maximum_wind_magnitude_mps(),
            "claim_boundary": self.claim_boundary,
        }


def _task_anchor(task: dict[str, Any]) -> tuple[float, float]:
    depot = task["depot"]["xyz_m"]
    points = task["service_points"]
    centroid_x = sum(float(item["xyz_m"][0]) for item in points) / len(points)
    centroid_y = sum(float(item["xyz_m"][1]) for item in points) / len(points)
    vector = (centroid_x - float(depot[0]), centroid_y - float(depot[1]))
    if math.hypot(*vector) <= 1e-12:
        vector = (1.0, 0.0)
    return _unit(vector)


def build_wind_scenario(config: dict[str, Any], scenario_id: str, task: dict[str, Any]) -> WindScenario:
    matches = [item for item in config["scenarios"] if item["id"] == scenario_id]
    if len(matches) != 1:
        raise ValueError(f"Wind scenario identity is missing or ambiguous: {scenario_id}")
    item = matches[0]
    error = config["forecast_error_set"]
    vertices = tuple((float(row[0]), float(row[1])) for row in error["vertices_xy_mps"])
    if len(vertices) != 4 or len(set(vertices)) != 4:
        raise ValueError("The declared horizontal box must contain four unique vertices")
    return WindScenario(
        scenario_id=scenario_id,
        kind=str(item["kind"]),
        definition=dict(item),
        anchor_unit_xy=_task_anchor(task),
        error_vertices_xy_mps=vertices,
        error_temporal_coupling=str(error["temporal_coupling"]),
    )
