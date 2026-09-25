"""Finite-pool wind-aware route, visit-order, speed, and payload co-design.

The order oracle is exact over the provided finite route pool. Ground speed
is selected on a deterministic grid; the public M600 power surface is
queried only with vector-resolved airspeed inside its measured domain.
"""

from __future__ import annotations

import hashlib
import math
from dataclasses import dataclass
from typing import Any, Callable

from .io_utils import canonical_bytes
from .m600_power import M600PowerModel
from .wind_field import WindScenario, build_wind_scenario


M600_ID = "M600_PRO_OSTI_PUBLIC_POWER_REFERENCE"
WIND_METHOD_ACCELERATION_SCREEN_MPS2 = 2.0
PARENT_COMPARATOR_ACCELERATION_MPS2 = 3.0
WIND_METHOD_JERK_SCREEN_MPS3 = 4.0
SPEED_GRID_COUNT = 257
MAX_WIND_INTEGRATION_CELL_M = 12.5
DEVELOPMENT_GROUND_SPEED_SCREEN_MPS = 16.0
C3_CLOSED_LOOP_SPEED_HEADROOM_MPS = 0.40
C3_REFERENCE_ACCELERATION_SCREEN_MPS2 = 1.20
QUINTIC_NORMALIZED_MAX_SPEED = 1.875
QUINTIC_NORMALIZED_MAX_ACCELERATION = 5.773502691896258
QUINTIC_NORMALIZED_MAX_JERK = 60.0
C3_CORRIDOR_EXECUTION_RESERVE_FACTOR = 1.40

METHOD_PARENT = "PARENT_DYNAMIC_PAYLOAD_COMPARATOR"
METHOD_WIND_DYNAMIC = "WIND_AWARE_DYNAMIC_PAYLOAD"
METHOD_WIND_STATIC = "WIND_AWARE_STATIC_INITIAL_MASS_ABLATION"
METHOD_CALM_REALIZED = "WIND_UNAWARE_PLAN_REALIZED_WIND"
METHODS = (METHOD_PARENT, METHOD_WIND_DYNAMIC, METHOD_WIND_STATIC, METHOD_CALM_REALIZED)


def travel_time_ground(
    length_m: float,
    ground_speed_mps: float,
    acceleration_mps2: float = WIND_METHOD_ACCELERATION_SCREEN_MPS2,
) -> float:
    speed = float(ground_speed_mps)
    if speed <= 0.0:
        raise ValueError("Ground speed must be positive")
    acceleration = float(acceleration_mps2)
    if acceleration <= 0.0:
        raise ValueError("Acceleration screen must be positive")
    length = float(length_m)
    threshold = speed * speed / acceleration
    return length / speed + speed / acceleration if length >= threshold else 2.0 * math.sqrt(length / acceleration)


def parent_travel_time(length_m: float, commanded_airspeed_mps: float, tailwind_mps: float) -> tuple[float, float]:
    ground_speed = max(0.5, float(commanded_airspeed_mps) + float(tailwind_mps))
    return travel_time_ground(
        float(length_m), ground_speed, PARENT_COMPARATOR_ACCELERATION_MPS2
    ), ground_speed


def execution_duration_screen(
    route: dict[str, Any],
    wind: WindScenario,
    public_power_airspeed_ceiling_mps: float,
) -> dict[str, Any]:
    """Conservative pre-C3 duration screen derived from quintic derivatives.

    Each retained polyline segment is assigned the maximum duration required
    by the normalized quintic speed, acceleration, and jerk envelopes.  The
    segment bounds are summed, representing a conservative transition at each
    route turn, then multiplied by the specified corridor/C3 reserve. The
    speed term subtracts the configured wind envelope and empirically selected
    closed-loop tracking headroom from the public power airspeed ceiling.
    Exact C3 optimization and continuous-feasibility checks follow separately.
    """

    path = route["visibility_path_xy_m"]
    if len(path) < 2:
        raise ValueError("Route must have at least two points")
    wind_envelope_mps = float(wind.declared_maximum_wind_magnitude_mps())
    reference_ground_speed_mps = (
        float(public_power_airspeed_ceiling_mps)
        - wind_envelope_mps
        - C3_CLOSED_LOOP_SPEED_HEADROOM_MPS
    )
    if reference_ground_speed_mps <= 0.5:
        raise ValueError("Declared wind envelope leaves no usable C3 reference ground-speed domain")
    segment_rows: list[dict[str, float]] = []
    geometric_length = 0.0
    for left, right in zip(path[:-1], path[1:]):
        length = math.hypot(float(right[0]) - float(left[0]), float(right[1]) - float(left[1]))
        if length <= 1e-10:
            raise ValueError("Degenerate route segment")
        geometric_length += length
        speed_s = QUINTIC_NORMALIZED_MAX_SPEED * length / reference_ground_speed_mps
        acceleration_s = math.sqrt(
            QUINTIC_NORMALIZED_MAX_ACCELERATION
            * length
            / C3_REFERENCE_ACCELERATION_SCREEN_MPS2
        )
        jerk_s = (
            QUINTIC_NORMALIZED_MAX_JERK * length / WIND_METHOD_JERK_SCREEN_MPS3
        ) ** (1.0 / 3.0)
        segment_rows.append(
            {
                "length_m": length,
                "speed_bound_s": speed_s,
                "acceleration_bound_s": acceleration_s,
                "jerk_bound_s": jerk_s,
                "selected_bound_s": max(speed_s, acceleration_s, jerk_s),
            }
        )
    declared_length = float(route["route_length_m"])
    if abs(geometric_length - declared_length) > max(1e-6, 1e-8 * declared_length):
        raise ValueError("Execution screen route length does not reproduce its polyline")
    raw = sum(item["selected_bound_s"] for item in segment_rows)
    return {
        "schema": "GPENMPC_WIND_PRE_C3_EXECUTION_DURATION_SCREEN_V3",
        "classification": "Engineering feasibility screen for reference generation.",
        "formula": "1.40 * sum_segments max(1.875*L/(PUBLIC_POWER_AIRSPEED_MAX-DECLARED_MAX_WIND-0.40), sqrt(5.773502691896258*L/1.20), cbrt(60*L/4))",
        "segment_count": len(segment_rows),
        "segment_bounds": segment_rows,
        "raw_quintic_derivative_bound_s": raw,
        "c3_corridor_reserve_factor": C3_CORRIDOR_EXECUTION_RESERVE_FACTOR,
        "public_power_airspeed_ceiling_mps": float(public_power_airspeed_ceiling_mps),
        "declared_maximum_wind_magnitude_mps": wind_envelope_mps,
        "closed_loop_speed_headroom_mps": C3_CLOSED_LOOP_SPEED_HEADROOM_MPS,
        "c3_reference_acceleration_screen_mps2": C3_REFERENCE_ACCELERATION_SCREEN_MPS2,
        "c3_acceptance_acceleration_limit_mps2": WIND_METHOD_ACCELERATION_SCREEN_MPS2,
        "wind_adjusted_reference_ground_speed_screen_mps": reference_ground_speed_mps,
        "mission_ground_speed_decision_screen_mps": DEVELOPMENT_GROUND_SPEED_SCREEN_MPS,
        "duration_s": C3_CORRIDOR_EXECUTION_RESERVE_FACTOR * raw,
        "calibration_evidence": "Development flight simulations informed the 1.40 duration reserve, 0.40 m/s speed headroom and 1.20 m/s2 reference-acceleration setting. The declared wind envelope reduces the available reference ground speed.",
        "claim_boundary": "Segment-duration estimate used before C3 trajectory generation and six-degree-of-freedom simulation.",
    }


def _turning_rad(path_xy: list[list[float]]) -> float:
    total = 0.0
    for index in range(1, len(path_xy) - 1):
        ax = float(path_xy[index][0]) - float(path_xy[index - 1][0])
        ay = float(path_xy[index][1]) - float(path_xy[index - 1][1])
        bx = float(path_xy[index + 1][0]) - float(path_xy[index][0])
        by = float(path_xy[index + 1][1]) - float(path_xy[index][1])
        an = math.hypot(ax, ay)
        bn = math.hypot(bx, by)
        if an > 0.0 and bn > 0.0:
            total += math.acos(max(-1.0, min(1.0, (ax * bx + ay * by) / (an * bn))))
    return total


def _route_hash(route: dict[str, Any]) -> str:
    existing = route.get("route_hash") or route.get("geometry_sha256")
    if existing:
        return str(existing).upper()
    identity = {
        "visibility_path_xy_m": route["visibility_path_xy_m"],
        "route_length_m": float(route["route_length_m"]),
    }
    return hashlib.sha256(canonical_bytes(identity)).hexdigest().upper()


def _route_candidate_id(route: dict[str, Any]) -> str:
    left = int(route.get("from_roster_index", -1))
    right = int(route.get("to_roster_index", -1))
    rank = int(route.get("selected_rank", 1))
    return f"RC_{left:04d}_TO_{right:04d}_R{rank:02d}_{_route_hash(route)[:12]}"


def _wind_cache_identity(wind: WindScenario) -> str:
    identity = {
        "scenario_id": wind.scenario_id,
        "kind": wind.kind,
        "definition": wind.definition,
        "anchor_unit_xy": list(wind.anchor_unit_xy),
        "error_vertices_xy_mps": [list(item) for item in wind.uncertainty_vertices()],
        "error_temporal_coupling": wind.error_temporal_coupling,
    }
    return hashlib.sha256(canonical_bytes(identity)).hexdigest().upper()


def _geometry_identity_hash(route: dict[str, Any]) -> str:
    identity = {
        "visibility_path_xy_m": [
            [round(float(point[0]), 9), round(float(point[1]), 9)] for point in route["visibility_path_xy_m"]
        ],
        "route_length_m": round(float(route["route_length_m"]), 9),
    }
    return hashlib.sha256(canonical_bytes(identity)).hexdigest().upper()


def _route_cells(route: dict[str, Any], ground_speed_mps: float) -> tuple[list[dict[str, float]], float]:
    path = route["visibility_path_xy_m"]
    if len(path) < 2:
        raise ValueError("Route must have at least two points")
    raw_segments: list[tuple[float, float, float, float, float]] = []
    geometric_length = 0.0
    for left, right in zip(path[:-1], path[1:]):
        x0, y0 = float(left[0]), float(left[1])
        x1, y1 = float(right[0]), float(right[1])
        length = math.hypot(x1 - x0, y1 - y0)
        if length <= 1e-10:
            raise ValueError("Degenerate route segment")
        raw_segments.append((x0, y0, x1, y1, length))
        geometric_length += length
    declared_length = float(route["route_length_m"])
    if abs(geometric_length - declared_length) > max(1e-6, 1e-8 * declared_length):
        raise ValueError("Route length does not reproduce its polyline geometry")
    cruise_duration = geometric_length / float(ground_speed_mps)
    cells: list[dict[str, float]] = []
    accumulated = 0.0
    for x0, y0, x1, y1, length in raw_segments:
        count = max(1, int(math.ceil(length / MAX_WIND_INTEGRATION_CELL_M)))
        tx = (x1 - x0) / length
        ty = (y1 - y0) / length
        cell_length = length / count
        for index in range(count):
            local_fraction = (index + 0.5) / count
            center_distance = accumulated + (index + 0.5) * cell_length
            cells.append(
                {
                    "x_m": x0 + local_fraction * (x1 - x0),
                    "y_m": y0 + local_fraction * (y1 - y0),
                    "tx": tx,
                    "ty": ty,
                    "ds_m": cell_length,
                    "local_time_s": center_distance / float(ground_speed_mps),
                }
            )
        accumulated += length
    return cells, cruise_duration


def evaluate_vector_route(
    model: M600PowerModel,
    route: dict[str, Any],
    payload_kg: float,
    ground_speed_mps: float,
    wind: WindScenario,
    fixed_execution_duration_s: float | None = None,
    execution_duration_screen_record: dict[str, Any] | None = None,
) -> dict[str, Any]:
    """Integrate a ground-track leg under vector wind and all declared vertices."""

    speed = float(ground_speed_mps)
    if speed <= 0.0 or speed > model.manufacturer_max_ground_speed_mps + 1e-10:
        raise ValueError("Ground speed lies outside the manufacturer no-wind speed context")
    cells, cruise_duration = _route_cells(route, speed)
    total_length = float(route["route_length_m"])
    longitudinal_duration = travel_time_ground(total_length, speed)
    if fixed_execution_duration_s is None:
        execution_screen = execution_duration_screen(route, wind, model.airspeed_max_mps)
        total_duration = max(longitudinal_duration, float(execution_screen["duration_s"]))
        duration_binding = "WIND_SCENARIO_PRE_C3_EXECUTION_SCREEN"
    else:
        fixed_duration = float(fixed_execution_duration_s)
        if fixed_duration + 1e-10 < cruise_duration:
            raise ValueError("Fixed selected-plan duration is shorter than constant-speed cruise time")
        total_duration = fixed_duration
        execution_screen = execution_duration_screen_record or {
            "schema": "GPENMPC_FIXED_SELECTED_PLAN_DURATION_V1",
            "classification": "SELECTED_COMPARATOR_TIMING_REPLAY",
            "duration_s": fixed_duration,
        }
        duration_binding = "FIXED_SELECTED_PLAN_DURATION_REPLAY"
    acceleration_allowance_s = max(0.0, total_duration - cruise_duration)

    def evaluate_error(error_xy: tuple[float, float]) -> dict[str, Any]:
        weighted_power_wm = 0.0
        min_airspeed = math.inf
        max_airspeed = -math.inf
        max_crab_deg = 0.0
        max_wind = 0.0
        feasible = True
        violations: list[dict[str, float]] = []
        for cell_index, cell in enumerate(cells):
            mean_wind = wind.mean_xy_mps(cell["x_m"], cell["y_m"], cell["local_time_s"], cruise_duration)
            wx = mean_wind[0] + error_xy[0]
            wy = mean_wind[1] + error_xy[1]
            air_x = speed * cell["tx"] - wx
            air_y = speed * cell["ty"] - wy
            airspeed = math.hypot(air_x, air_y)
            min_airspeed = min(min_airspeed, airspeed)
            max_airspeed = max(max_airspeed, airspeed)
            max_wind = max(max_wind, math.hypot(wx, wy))
            denominator = max(1e-15, airspeed)
            cosine = max(-1.0, min(1.0, (air_x * cell["tx"] + air_y * cell["ty"]) / denominator))
            max_crab_deg = max(max_crab_deg, math.degrees(math.acos(cosine)))
            in_domain = model.airspeed_min_mps - 1e-9 <= airspeed <= model.airspeed_max_mps + 1e-9
            if not in_domain:
                feasible = False
                if len(violations) < 6:
                    violations.append({"cell_index": cell_index, "airspeed_mps": airspeed})
                continue
            weighted_power_wm += model.phase_power_w("FORWARD", airspeed, payload_kg) * cell["ds_m"]
        energy = None
        if feasible:
            cruise_energy = weighted_power_wm / speed
            mean_power = weighted_power_wm / total_length
            energy = cruise_energy + mean_power * acceleration_allowance_s
        domain_margin = min(min_airspeed - model.airspeed_min_mps, model.airspeed_max_mps - max_airspeed)
        return {
            "error_xy_mps": list(error_xy),
            "feasible": feasible,
            "energy_j": energy,
            "minimum_airspeed_mps": min_airspeed,
            "maximum_airspeed_mps": max_airspeed,
            "minimum_public_power_domain_margin_mps": domain_margin,
            "maximum_crab_angle_deg": max_crab_deg,
            "maximum_wind_magnitude_mps": max_wind,
            "manufacturer_wind_context_margin_mps": model.manufacturer_wind_resistance_mps - max_wind,
            "violations": violations,
        }

    nominal = evaluate_error((0.0, 0.0))
    vertex_rows = [evaluate_error(vertex) for vertex in wind.uncertainty_vertices()]
    robust_feasible = all(item["feasible"] for item in vertex_rows)
    robust_energy = max(float(item["energy_j"]) for item in vertex_rows) if robust_feasible else None
    worst = None
    if robust_feasible:
        worst = max(vertex_rows, key=lambda item: (float(item["energy_j"]), item["error_xy_mps"]))
    all_rows = [nominal] + vertex_rows
    return {
        "route_hash": _route_hash(route),
        "ground_speed_mps": speed,
        "duration_s": total_duration,
        "longitudinal_duration_s": longitudinal_duration,
        "cruise_duration_s": cruise_duration,
        "acceleration_allowance_s": acceleration_allowance_s,
        "execution_duration_screen": execution_screen,
        "execution_duration_binding": duration_binding,
        "execution_duration_screen_active": total_duration > longitudinal_duration + 1e-12,
        "integration_cell_count": len(cells),
        "integration_max_cell_m": MAX_WIND_INTEGRATION_CELL_M,
        "nominal_feasible": bool(nominal["feasible"]),
        "robust_feasible": robust_feasible,
        "nominal_energy_j": nominal["energy_j"],
        "robust_energy_j": robust_energy,
        "worst_error_xy_mps": None if worst is None else worst["error_xy_mps"],
        "minimum_airspeed_mps": min(float(item["minimum_airspeed_mps"]) for item in all_rows),
        "maximum_airspeed_mps": max(float(item["maximum_airspeed_mps"]) for item in all_rows),
        "minimum_public_power_domain_margin_mps": min(float(item["minimum_public_power_domain_margin_mps"]) for item in all_rows),
        "maximum_crab_angle_deg": max(float(item["maximum_crab_angle_deg"]) for item in all_rows),
        "maximum_wind_magnitude_mps": max(float(item["maximum_wind_magnitude_mps"]) for item in all_rows),
        "manufacturer_wind_context_margin_mps": min(float(item["manufacturer_wind_context_margin_mps"]) for item in all_rows),
        "nominal": nominal,
        "uncertainty_vertices": vertex_rows,
        "power_model_domain": [model.airspeed_min_mps, model.airspeed_max_mps],
        "wind_resistance_value_role": "Manufacturer reference value for comparison with the modeled wind.",
    }


def _package_schedule(task: dict[str, Any], schedule: str) -> dict[int, float]:
    points = task["service_points"]
    scale = len(points)
    total = 4.54
    masses = [total * float(index) / sum(range(1, scale + 1)) for index in range(1, scale + 1)]
    depot = task["depot"]["xyz_m"]
    ranks = sorted(
        range(scale),
        key=lambda index: (
            math.hypot(float(points[index]["xyz_m"][0]) - float(depot[0]), float(points[index]["xyz_m"][1]) - float(depot[1])),
            int(points[index]["roster_index"]),
        ),
    )
    if schedule == "HEAVY_FAR":
        assigned = sorted(masses)
    elif schedule == "HEAVY_NEAR":
        assigned = sorted(masses, reverse=True)
    else:
        raise ValueError(f"Unsupported payload schedule: {schedule}")
    result: dict[int, float] = {}
    for rank, local_index in enumerate(ranks):
        result[int(points[local_index]["roster_index"])] = assigned[rank]
    result[int(points[ranks[-1]]["roster_index"])] += total - sum(result.values())
    return result


@dataclass
class WindAwarePlanner:
    model: M600PowerModel
    leg_cache: dict[str, Any]
    multiroute_cache: dict[str, Any]
    wind_config: dict[str, Any]

    def __post_init__(self) -> None:
        if self.leg_cache.get("status") != "PASS_SHARED_PUBLIC_ROAD_TASKS_AND_EXACT_PAIR_ROUTE_CACHE":
            raise ValueError("Single-route cache validation failed")
        if self.multiroute_cache.get("status") != "PASS_BOUNDED_MULTIROUTE_CANDIDATE_POOL":
            raise ValueError("Multiroute cache validation failed")
        if not self.multiroute_cache["validation"]["all_continuous_exact_replay_pass"]:
            raise ValueError("Multiroute cache failed exact replay")
        self._vector_cache: dict[tuple[Any, ...], dict[str, Any] | None] = {}
        self._parent_cache: dict[tuple[Any, ...], dict[str, Any]] = {}

    def task(self, scale: int) -> dict[str, Any]:
        return self.leg_cache["tasks"][str(int(scale))]

    def scenario(self, scale: int, scenario_id: str) -> WindScenario:
        return build_wind_scenario(self.wind_config, scenario_id, self.task(scale))

    def _baseline_route(self, left: int, right: int) -> dict[str, Any]:
        return self.leg_cache["platform_pair_routes"][M600_ID][f"{left}->{right}"]

    def _candidate_routes(self, left: int, right: int) -> list[dict[str, Any]]:
        pair = self.multiroute_cache["platform_pair_candidates"][M600_ID][f"{left}->{right}"]
        candidates = list(pair["candidates"])
        if not candidates:
            raise ValueError("Empty multiroute candidate set")
        return candidates

    def _speed_grid(self) -> list[float]:
        lower = 0.5
        manufacturer_upper = self.model.manufacturer_max_ground_speed_mps
        values = [lower + (manufacturer_upper - lower) * index / (SPEED_GRID_COUNT - 1) for index in range(SPEED_GRID_COUNT)]
        values.extend([self.model.airspeed_min_mps, self.model.airspeed_max_mps, DEVELOPMENT_GROUND_SPEED_SCREEN_MPS])
        return sorted(
            set(
                round(item, 12)
                for item in values
                if lower <= item <= DEVELOPMENT_GROUND_SPEED_SCREEN_MPS + 1e-12
            )
        )

    def _optimize_vector_route(self, route: dict[str, Any], payload_kg: float, wind: WindScenario, robust: bool) -> dict[str, Any] | None:
        key = ("VECTOR", _wind_cache_identity(wind), _route_hash(route), round(float(payload_kg), 10), bool(robust))
        if key in self._vector_cache:
            return self._vector_cache[key]
        best: tuple[tuple[float, ...], dict[str, Any]] | None = None
        for speed in self._speed_grid():
            evaluation = evaluate_vector_route(self.model, route, payload_kg, speed, wind)
            feasible = evaluation["robust_feasible"] if robust else evaluation["nominal_feasible"]
            if not feasible:
                continue
            objective_energy = float(evaluation["robust_energy_j"] if robust else evaluation["nominal_energy_j"])
            key_value = (
                objective_energy,
                float(evaluation["nominal_energy_j"]),
                float(evaluation["duration_s"]),
                float(speed),
            )
            candidate = {
                "plan_model": "SEGMENTWISE_VECTOR_WIND",
                "route": route,
                "route_hash": _route_hash(route),
                "route_candidate_id": _route_candidate_id(route),
                "selected_candidate_rank": int(route.get("selected_rank", 1)),
                "commanded_airspeed_mps": None,
                "ground_speed_mps": float(speed),
                "planned_duration_s": float(evaluation["duration_s"]),
                "planned_nominal_energy_j": float(evaluation["nominal_energy_j"]),
                "planned_robust_energy_j": float(evaluation["robust_energy_j"]) if evaluation["robust_energy_j"] is not None else None,
                "planned_objective_energy_j": objective_energy,
                "planning_robust": robust,
                "planning_evaluation": evaluation,
                "speed_grid_count": len(self._speed_grid()),
            }
            if best is None or key_value < best[0]:
                best = (key_value, candidate)
        result = None if best is None else best[1]
        self._vector_cache[key] = result
        return result

    def _select_vector_candidate(self, left: int, right: int, payload_kg: float, wind: WindScenario, robust: bool) -> dict[str, Any] | None:
        best: tuple[tuple[Any, ...], dict[str, Any]] | None = None
        ledger: list[dict[str, Any]] = []
        for route in self._candidate_routes(left, right):
            candidate = self._optimize_vector_route(route, payload_kg, wind, robust)
            if candidate is None:
                ledger.append(
                    {
                        "cost_identity": f"c({left},{right},{float(payload_kg):.10f},{wind.scenario_id},{_route_candidate_id(route)})",
                        "route_candidate_id": _route_candidate_id(route),
                        "route_hash": _route_hash(route),
                        "status": "NO_PUBLIC_POWER_DOMAIN_FEASIBLE_GROUND_SPEED",
                        "candidate_cost_j": None,
                    }
                )
                continue
            ledger.append(
                {
                    "cost_identity": f"c({left},{right},{float(payload_kg):.10f},{wind.scenario_id},{candidate['route_candidate_id']})",
                    "route_candidate_id": candidate["route_candidate_id"],
                    "route_hash": candidate["route_hash"],
                    "status": "FEASIBLE",
                    "candidate_cost_j": candidate["planned_objective_energy_j"],
                    "candidate_nominal_energy_j": candidate["planned_nominal_energy_j"],
                    "candidate_robust_energy_j": candidate["planned_robust_energy_j"],
                    "ground_speed_mps": candidate["ground_speed_mps"],
                }
            )
            key = (
                float(candidate["planned_objective_energy_j"]),
                float(candidate["planned_nominal_energy_j"]),
                float(candidate["planned_duration_s"]),
                float(route["route_length_m"]),
                candidate["route_hash"],
            )
            if best is None or key < best[0]:
                best = (key, candidate)
        if best is None:
            return None
        selected = dict(best[1])
        selected["candidate_cost_ledger"] = ledger
        selected["finite_pool_candidate_count"] = len(ledger)
        selected["finite_pool_selection_identity"] = "ARGMIN_R_C_I_J_M_W_R_BEFORE_HELD_KARP_STATE_COMBINATION"
        return selected

    def _parent_plan(self, left: int, right: int, payload_kg: float, wind: WindScenario) -> dict[str, Any]:
        route = self._baseline_route(left, right)
        key = ("PARENT", _wind_cache_identity(wind), _route_hash(route), round(float(payload_kg), 10))
        if key in self._parent_cache:
            return self._parent_cache[key]
        path = route["visibility_path_xy_m"]
        x0, y0 = float(path[0][0]), float(path[0][1])
        x1, y1 = float(path[-1][0]), float(path[-1][1])
        dx, dy = x1 - x0, y1 - y0
        chord = math.hypot(dx, dy)
        if chord <= 0.0:
            raise ValueError("Degenerate parent route chord")
        tx, ty = dx / chord, dy / chord
        mean_wind = wind.mean_xy_mps(0.5 * (x0 + x1), 0.5 * (y0 + y1), 0.0, float(route["route_length_m"]) / self.model.airspeed_min_mps)
        tailwind = tx * mean_wind[0] + ty * mean_wind[1]
        best: tuple[tuple[float, float, float], dict[str, Any]] | None = None
        for index in range(129):
            airspeed = self.model.airspeed_min_mps + (self.model.airspeed_max_mps - self.model.airspeed_min_mps) * index / 128.0
            duration, ground_speed = parent_travel_time(float(route["route_length_m"]), airspeed, tailwind)
            energy = self.model.phase_power_w("FORWARD", airspeed, payload_kg) * duration
            candidate = {
                "plan_model": "PARENT_CHORD_PROJECTED_SCALAR_WIND",
                "route": route,
                "route_hash": _route_hash(route),
                "route_candidate_id": "PARENT_" + _route_candidate_id(route),
                "selected_candidate_rank": 1,
                "commanded_airspeed_mps": airspeed,
                "ground_speed_mps": ground_speed,
                "tailwind_projection_mps": tailwind,
                "planned_duration_s": duration,
                "planned_nominal_energy_j": energy,
                "planned_robust_energy_j": None,
                "planned_objective_energy_j": energy,
                "planning_robust": False,
                "ignored_physics": ["SEGMENTWISE_DIRECTION", "CROSSWIND_CRAB", "FORECAST_ERROR", "GUST_AFTER_LEG_START"],
                "speed_grid_count": 129,
                "finite_pool_candidate_count": 1,
                "finite_pool_selection_identity": "PARENT_SINGLE_RETAINED_ROUTE_COMPARATOR",
            }
            candidate["candidate_cost_ledger"] = [
                {
                    "cost_identity": f"c({left},{right},{float(payload_kg):.10f},{wind.scenario_id},{candidate['route_candidate_id']})",
                    "route_candidate_id": candidate["route_candidate_id"],
                    "route_hash": candidate["route_hash"],
                    "status": "FEASIBLE_PARENT_SCALAR_MODEL",
                    "candidate_cost_j": energy,
                    "commanded_airspeed_mps": airspeed,
                    "ground_speed_mps": ground_speed,
                }
            ]
            rank = (energy, airspeed, ground_speed)
            if best is None or rank < best[0]:
                best = (rank, candidate)
        assert best is not None
        self._parent_cache[key] = best[1]
        return best[1]

    def _select_for_method(
        self,
        method: str,
        left: int,
        right: int,
        decision_payload_kg: float,
        actual_wind: WindScenario,
        calm_wind: WindScenario,
    ) -> dict[str, Any] | None:
        if method == METHOD_PARENT:
            return self._parent_plan(left, right, decision_payload_kg, actual_wind)
        if method in {METHOD_WIND_DYNAMIC, METHOD_WIND_STATIC}:
            return self._select_vector_candidate(left, right, decision_payload_kg, actual_wind, actual_wind.planning_robust)
        if method == METHOD_CALM_REALIZED:
            return self._select_vector_candidate(left, right, decision_payload_kg, calm_wind, False)
        raise ValueError(f"Unsupported planning method: {method}")

    @staticmethod
    def _held_karp(
        depot: int,
        targets: list[int],
        transition_cost: Callable[[int, int, int, int | None], float],
    ) -> tuple[list[int] | None, float | None, int]:
        scale = len(targets)
        states: dict[tuple[int, int], tuple[float, tuple[int, ...]]] = {}
        transitions = 0
        for local, target in enumerate(targets):
            value = transition_cost(depot, target, 0, local)
            transitions += 1
            if math.isfinite(value):
                states[(1 << local, local)] = (value, (local,))
        for mask in range(1, 1 << scale):
            for last in range(scale):
                current = states.get((mask, last))
                if current is None:
                    continue
                for nxt in range(scale):
                    if mask & (1 << nxt):
                        continue
                    edge = transition_cost(targets[last], targets[nxt], mask, nxt)
                    transitions += 1
                    if not math.isfinite(edge):
                        continue
                    new_mask = mask | (1 << nxt)
                    new_path = current[1] + (nxt,)
                    new_value = current[0] + edge
                    prior = states.get((new_mask, nxt))
                    if prior is None or new_value < prior[0] - 1e-8 or (abs(new_value - prior[0]) <= 1e-8 and new_path < prior[1]):
                        states[(new_mask, nxt)] = (new_value, new_path)
        full = (1 << scale) - 1
        best: tuple[float, tuple[int, ...]] | None = None
        for last in range(scale):
            current = states.get((full, last))
            if current is None:
                continue
            edge = transition_cost(targets[last], depot, full, None)
            transitions += 1
            if not math.isfinite(edge):
                continue
            candidate = (current[0] + edge, current[1])
            if best is None or candidate < best:
                best = candidate
        if best is None:
            return None, None, transitions
        return [targets[index] for index in best[1]], float(best[0]), transitions

    def solve_case(self, scale: int, schedule: str, scenario_id: str, method: str) -> dict[str, Any]:
        if method not in METHODS:
            raise ValueError(f"Unknown method: {method}")
        task = self.task(scale)
        depot = int(task["depot"]["roster_index"])
        targets = [int(item["roster_index"]) for item in task["service_points"]]
        service_duration = {int(item["roster_index"]): float(item["service_duration_s"]) for item in task["service_points"]}
        service_altitude = {
            int(item["roster_index"]): float(item["xyz_m"][2])
            for item in task["service_points"]
        }
        packages = _package_schedule(task, schedule)
        initial_payload = sum(packages.values())
        actual_wind = self.scenario(scale, scenario_id)
        calm_wind = self.scenario(scale, "CALM")

        def remaining(mask: int) -> float:
            return sum(packages[targets[index]] for index in range(len(targets)) if not (mask & (1 << index)))

        def transition(left: int, right: int, mask: int, target_local: int | None) -> float:
            actual_payload = remaining(mask)
            decision_payload = initial_payload if method == METHOD_WIND_STATIC else actual_payload
            selected = self._select_for_method(method, left, right, decision_payload, actual_wind, calm_wind)
            if selected is None:
                return math.inf
            value = float(selected["planned_objective_energy_j"])
            if target_local is not None:
                delivered = packages[targets[target_local]]
                target = targets[target_local]
                value += self.model.service(
                    decision_payload,
                    delivered,
                    service_duration[target],
                    service_altitude[target],
                )["service_total_energy_j"]
            return value

        order, objective, transition_count = self._held_karp(depot, targets, transition)
        common = {
            "platform_id": M600_ID,
            "profile_hash": self.model.profile["profile_hash"],
            "scale": int(scale),
            "schedule": schedule,
            "scenario_id": scenario_id,
            "wind": actual_wind.descriptor(),
            "method": method,
            "initial_payload_kg": initial_payload,
            "package_mass_kg_by_roster_index": {str(key): value for key, value in sorted(packages.items())},
            "order_oracle": "HELD_KARP_EXACT_OVER_FINITE_DIRECTED_ROUTE_POOL",
            "optimization_identity": "FIRST_COMPUTE_C_I_J_M_W_R_FOR_EACH_RETAINED_ROUTE_CANDIDATE_THEN_EXACT_HELD_KARP_OVER_THE_FINITE_POOL",
            "ground_speed_discretization": {
                "count_before_boundary_insertion": SPEED_GRID_COUNT,
                "minimum_mps": 0.5,
                "maximum_mps": DEVELOPMENT_GROUND_SPEED_SCREEN_MPS,
                "development_execution_screen_mps": DEVELOPMENT_GROUND_SPEED_SCREEN_MPS,
                "manufacturer_no_wind_context_mps": self.model.manufacturer_max_ground_speed_mps,
                "role": "Discrete ground-speed choices filtered by the reference-generation feasibility screen.",
            },
            "transition_evaluations": transition_count,
            "claim_boundary": "Minimum modeled cost over the directed candidate routes and discrete speed choices.",
        }
        if order is None or objective is None:
            row = {
                **common,
                "status": "NO_COMPLETE_MODEL_DOMAIN_FEASIBLE_PLAN",
                "visit_order_roster_indices": None,
                "closed_sequence_roster_indices": None,
                "planned_objective_energy_j": None,
                "legs": [],
                "totals": {"cross_evaluation_valid": False},
            }
            row["row_sha256"] = hashlib.sha256(canonical_bytes(row)).hexdigest().upper()
            return row

        sequence = [depot] + order + [depot]
        payload = initial_payload
        mask = 0
        target_lookup = {target: index for index, target in enumerate(targets)}
        leg_rows: list[dict[str, Any]] = []
        distance = 0.0
        airborne_time = 0.0
        service_time = 0.0
        service_energy_actual = 0.0
        nominal_energy_sum = 0.0
        robust_energy_sum = 0.0
        nominal_valid = True
        robust_valid = True
        route_changes_vs_parent = 0
        task_vertex_energy_by_error: dict[tuple[float, float], float] = {
            tuple(vertex): 0.0 for vertex in actual_wind.uncertainty_vertices()
        }
        task_vertex_validity: dict[tuple[float, float], bool] = {
            tuple(vertex): True for vertex in actual_wind.uncertainty_vertices()
        }
        independent_legwise_worst_ledger: list[dict[str, Any]] = []
        for ordinal, (left, right) in enumerate(zip(sequence[:-1], sequence[1:]), start=1):
            decision_payload = initial_payload if method == METHOD_WIND_STATIC else payload
            selected = self._select_for_method(method, left, right, decision_payload, actual_wind, calm_wind)
            if selected is None:
                raise RuntimeError("A transition selected by the oracle became infeasible during replay")
            route = selected["route"]
            planning_evaluation = selected.get("planning_evaluation")
            planning_screen = None if planning_evaluation is None else planning_evaluation.get("execution_duration_screen")
            actual_evaluation = evaluate_vector_route(
                self.model,
                route,
                payload,
                float(selected["ground_speed_mps"]),
                actual_wind,
                fixed_execution_duration_s=float(selected["planned_duration_s"]),
                execution_duration_screen_record=planning_screen,
            )
            if actual_evaluation["nominal_feasible"]:
                nominal_energy_sum += float(actual_evaluation["nominal_energy_j"])
            else:
                nominal_valid = False
            if actual_evaluation["robust_feasible"]:
                robust_energy_sum += float(actual_evaluation["robust_energy_j"])
            else:
                robust_valid = False
            vertex_lookup = {
                tuple(float(value) for value in item["error_xy_mps"]): item
                for item in actual_evaluation["uncertainty_vertices"]
            }
            for vertex in task_vertex_energy_by_error:
                vertex_row = vertex_lookup[vertex]
                if vertex_row["feasible"]:
                    task_vertex_energy_by_error[vertex] += float(vertex_row["energy_j"])
                else:
                    task_vertex_validity[vertex] = False
            independent_legwise_worst_ledger.append(
                {
                    "ordinal": ordinal,
                    "from_roster_index": left,
                    "to_roster_index": right,
                    "route_candidate_id": selected["route_candidate_id"],
                    "worst_error_xy_mps": actual_evaluation["worst_error_xy_mps"],
                    "complete_leg_energy_j": actual_evaluation["robust_energy_j"],
                    "all_declared_vertex_complete_leg_energies": [
                        {
                            "error_xy_mps": item["error_xy_mps"],
                            "feasible": item["feasible"],
                            "energy_j": item["energy_j"],
                        }
                        for item in actual_evaluation["uncertainty_vertices"]
                    ],
                }
            )
            baseline_route = self._baseline_route(left, right)
            baseline_hash = _route_hash(baseline_route)
            selected_geometry_hash = _geometry_identity_hash(route)
            parent_geometry_hash = _geometry_identity_hash(baseline_route)
            changed = selected_geometry_hash != parent_geometry_hash
            route_changes_vs_parent += int(changed)
            leg = {
                "ordinal": ordinal,
                "from_roster_index": left,
                "to_roster_index": right,
                "payload_departure_kg": payload,
                "decision_payload_kg": decision_payload,
                "route_hash": selected["route_hash"],
                "route_candidate_id": selected["route_candidate_id"],
                "parent_route_hash": baseline_hash,
                "geometry_identity_sha256": selected_geometry_hash,
                "parent_geometry_identity_sha256": parent_geometry_hash,
                "route_changed_vs_parent": changed,
                "selected_candidate_rank": selected["selected_candidate_rank"],
                "visibility_path_xy_m": route["visibility_path_xy_m"],
                "route_length_m": float(route["route_length_m"]),
                "minimum_exact_buffered_clearance_m": float(route["minimum_exact_buffered_clearance_m"]),
                "total_heading_change_deg": math.degrees(_turning_rad(route["visibility_path_xy_m"])),
                "plan_model": selected["plan_model"],
                "commanded_airspeed_mps": selected["commanded_airspeed_mps"],
                "ground_speed_mps": selected["ground_speed_mps"],
                "planned_duration_s": selected["planned_duration_s"],
                "planned_nominal_energy_j": selected["planned_nominal_energy_j"],
                "planned_robust_energy_j": selected["planned_robust_energy_j"],
                "planned_objective_energy_j": selected["planned_objective_energy_j"],
                "candidate_cost_ledger": selected["candidate_cost_ledger"],
                "actual_vector_cross_evaluation": actual_evaluation,
            }
            distance += float(route["route_length_m"])
            airborne_time += float(actual_evaluation["duration_s"])
            if right != depot:
                delivered = packages[right]
                service = self.model.service(
                    payload,
                    delivered,
                    service_duration[right],
                    service_altitude[right],
                )
                leg["service"] = service
                service_energy_actual += float(service["service_total_energy_j"])
                service_time += float(service["descent_duration_s"] + service["ground_service_duration_s"] + service["ascent_duration_s"])
                payload = float(service["payload_after_kg"])
                mask |= 1 << target_lookup[right]
            else:
                leg["service"] = {"status": "NOT_RUN_RETURN_DEPOT"}
            leg_rows.append(leg)

        nominal_task_energy = nominal_energy_sum + service_energy_actual if nominal_valid else None
        robust_task_energy = robust_energy_sum + service_energy_actual if robust_valid else None
        mission_constant_vertex_ledger = []
        for vertex in sorted(task_vertex_energy_by_error):
            valid = task_vertex_validity[vertex]
            airborne = task_vertex_energy_by_error[vertex] if valid else None
            mission_constant_vertex_ledger.append(
                {
                    "error_xy_mps": list(vertex),
                    "all_legs_feasible": valid,
                    "complete_task_airborne_energy_j": airborne,
                    "complete_task_energy_j": None if airborne is None else airborne + service_energy_actual,
                }
            )
        mission_constant_worst = None
        valid_constant_vertices = [item for item in mission_constant_vertex_ledger if item["all_legs_feasible"]]
        if len(valid_constant_vertices) == len(mission_constant_vertex_ledger):
            mission_constant_worst = max(valid_constant_vertices, key=lambda item: (float(item["complete_task_energy_j"]), item["error_xy_mps"]))
        uncertainty_task_ledger = {
            "declared_primary_coupling": actual_wind.error_temporal_coupling,
            "full_energy_evaluation_policy": "Power is integrated over every route cell for each declared uncertainty vertex.",
            "ground_speed_and_duration_policy": "GROUND_SPEED_AND_LEG_DURATION_ARE_FIXED_FOR_EACH_SELECTED_PLAN_WHILE_ERROR_VERTICES_CHANGE_REQUIRED_AIRSPEED_AND_POWER",
            "mission_constant_error_vertex_ledger": mission_constant_vertex_ledger,
            "mission_constant_error_worst_vertex": mission_constant_worst,
            "independent_legwise_cartesian_product_exact_worst": {
                "feasible": robust_valid,
                "complete_task_energy_j": robust_task_energy,
                "legwise_selected_worst_vertices": independent_legwise_worst_ledger,
                "exactness_reason": "THE_DECLARED_ERROR_IS_CONSTANT_WITHIN_EACH_DIRECTED_LEG_AND_INDEPENDENT_BETWEEN_LEGS_SO_THE_CARTESIAN_PRODUCT_MAXIMUM_SEPARATES_AS_THE_SUM_OF_COMPLETE_LEG_MAXIMA",
            },
        }
        energy_for_soc = robust_task_energy if actual_wind.planning_robust else nominal_task_energy
        terminal_soc = None if energy_for_soc is None else max(0.0, 1.0 - energy_for_soc / self.model.battery_energy_j)
        totals = {
            "distance_m": distance,
            "airborne_time_s": airborne_time,
            "service_time_s": service_time,
            "mission_time_s": airborne_time + service_time,
            "actual_service_energy_j": service_energy_actual,
            "actual_nominal_airborne_energy_j": nominal_energy_sum if nominal_valid else None,
            "actual_robust_airborne_energy_j": robust_energy_sum if robust_valid else None,
            "actual_nominal_task_energy_j": nominal_task_energy,
            "actual_robust_task_energy_j": robust_task_energy,
            "cross_evaluation_nominal_valid": nominal_valid,
            "cross_evaluation_robust_valid": robust_valid,
            "cross_evaluation_valid": robust_valid if actual_wind.planning_robust else nominal_valid,
            "terminal_payload_kg": payload,
            "terminal_soc": terminal_soc,
            "route_changes_vs_parent_count": route_changes_vs_parent,
            "minimum_exact_buffered_clearance_m": min(float(leg["minimum_exact_buffered_clearance_m"]) for leg in leg_rows),
            "minimum_public_power_domain_margin_mps": min(float(leg["actual_vector_cross_evaluation"]["minimum_public_power_domain_margin_mps"]) for leg in leg_rows),
            "maximum_airspeed_mps": max(float(leg["actual_vector_cross_evaluation"]["maximum_airspeed_mps"]) for leg in leg_rows),
            "maximum_crab_angle_deg": max(float(leg["actual_vector_cross_evaluation"]["maximum_crab_angle_deg"]) for leg in leg_rows),
            "maximum_wind_magnitude_mps": max(float(leg["actual_vector_cross_evaluation"]["maximum_wind_magnitude_mps"]) for leg in leg_rows),
            "manufacturer_wind_context_minimum_margin_mps": min(float(leg["actual_vector_cross_evaluation"]["manufacturer_wind_context_margin_mps"]) for leg in leg_rows),
        }
        row = {
            **common,
            "status": "PASS_COMPLETE_PLAN" if totals["cross_evaluation_valid"] else "SELECTED_PLAN_OUTSIDE_PUBLIC_POWER_CROSS_EVALUATION_DOMAIN",
            "visit_order_roster_indices": order,
            "closed_sequence_roster_indices": sequence,
            "planned_objective_energy_j": objective,
            "legs": leg_rows,
            "totals": totals,
            "uncertainty_task_ledger": uncertainty_task_ledger,
        }
        row["row_sha256"] = hashlib.sha256(canonical_bytes(row)).hexdigest().upper()
        return row
