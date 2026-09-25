"""Run two planners on six common multi-city missions."""

from __future__ import annotations

import argparse
import glob
import json
import math
import sys
from pathlib import Path
from typing import Any, Callable

from multi_urban_common import file_reference, write_content_addressed_json


METHOD_DISTANCE = "P_DIST_FIXED"
METHOD_ENERGY = "P_ENERGY_WIND_PAYLOAD"
METHODS = (METHOD_DISTANCE, METHOD_ENERGY)


def load_unique(pattern: str) -> tuple[Path, dict[str, Any]]:
    paths = sorted(Path(item) for item in glob.glob(pattern))
    if len(paths) != 1:
        raise RuntimeError(f"Expected one file for {pattern}, found {len(paths)}")
    return paths[0], json.loads(paths[0].read_text(encoding="utf-8"))


def held_karp(
    depot: int,
    targets: list[int],
    transition: Callable[[int, int, int, int | None], float],
) -> tuple[list[int], float, int]:
    size = len(targets)
    states: dict[tuple[int, int], tuple[float, tuple[int, ...]]] = {}
    evaluations = 0
    for local, target in enumerate(targets):
        value = transition(depot, target, 0, local)
        evaluations += 1
        if math.isfinite(value):
            states[(1 << local, local)] = (value, (local,))
    for mask in range(1, 1 << size):
        for last in range(size):
            current = states.get((mask, last))
            if current is None:
                continue
            for nxt in range(size):
                if mask & (1 << nxt):
                    continue
                edge = transition(targets[last], targets[nxt], mask, nxt)
                evaluations += 1
                if not math.isfinite(edge):
                    continue
                proposal = (current[0] + edge, current[1] + (nxt,))
                key = (mask | (1 << nxt), nxt)
                if key not in states or proposal < states[key]:
                    states[key] = proposal
    full = (1 << size) - 1
    best: tuple[float, tuple[int, ...]] | None = None
    for last in range(size):
        current = states.get((full, last))
        if current is None:
            continue
        edge = transition(targets[last], depot, full, None)
        evaluations += 1
        if not math.isfinite(edge):
            continue
        proposal = (current[0] + edge, current[1])
        if best is None or proposal < best:
            best = proposal
    if best is None:
        raise RuntimeError("No complete Held-Karp plan")
    return [targets[index] for index in best[1]], float(best[0]), evaluations


def speed_grid(model: Any) -> list[float]:
    """Construct the shared finite ground-speed decision grid."""
    lower = 0.5
    upper = 16.0
    count = 257
    values = [lower + (upper - lower) * index / (count - 1) for index in range(count)]
    values.extend([model.airspeed_min_mps, model.airspeed_max_mps, 10.0, 16.0])
    return sorted(set(round(value, 12) for value in values if lower <= value <= upper + 1e-12))


def compact_exact_evaluation(evaluation: dict[str, Any]) -> dict[str, Any]:
    """Keep the scientific scalars and summarize the per-segment bound rows."""
    screen = dict(evaluation["execution_duration_screen"])
    segment_rows = list(screen.pop("segment_bounds", []))
    screen["segment_bound_row_count"] = len(segment_rows)
    return {
        "flight_nominal_energy_j": float(evaluation["nominal_energy_j"]),
        "flight_robust_energy_j": None
        if evaluation["robust_energy_j"] is None
        else float(evaluation["robust_energy_j"]),
        "flight_duration_s": float(evaluation["duration_s"]),
        "longitudinal_duration_s": float(evaluation["longitudinal_duration_s"]),
        "cruise_duration_s": float(evaluation["cruise_duration_s"]),
        "acceleration_allowance_s": float(evaluation["acceleration_allowance_s"]),
        "ground_speed_mps": float(evaluation["ground_speed_mps"]),
        "minimum_airspeed_mps": float(evaluation["minimum_airspeed_mps"]),
        "maximum_airspeed_mps": float(evaluation["maximum_airspeed_mps"]),
        "minimum_public_power_domain_margin_mps": float(evaluation["minimum_public_power_domain_margin_mps"]),
        "maximum_crab_angle_deg": float(evaluation["maximum_crab_angle_deg"]),
        "maximum_wind_magnitude_mps": float(evaluation["maximum_wind_magnitude_mps"]),
        "nominal_feasible": bool(evaluation["nominal_feasible"]),
        "robust_feasible": bool(evaluation["robust_feasible"]),
        "execution_duration_screen_active": bool(evaluation["execution_duration_screen_active"]),
        "execution_duration_screen": screen,
        "integration_cell_count": int(evaluation["integration_cell_count"]),
        "worst_error_xy_mps": evaluation["worst_error_xy_mps"],
    }


def solve_mission(
    model: Any,
    evaluate_vector_route: Callable[..., dict[str, Any]],
    wind_scenario_class: Any,
    mission: dict[str, Any],
    pool: dict[str, Any],
    method: str,
) -> dict[str, Any]:
    depot = int(mission["depot"]["roster_index"])
    services = list(mission["service_points"])
    targets = [int(row["roster_index"]) for row in services]
    packages = {int(row["roster_index"]): float(row["package_mass_kg"]) for row in services}
    service_time = {int(row["roster_index"]): float(row["service_duration_s"]) for row in services}
    service_altitude = {
        int(row["roster_index"]): float(row["xyz_m"][2]) for row in services
    }
    initial_payload = float(mission["initial_payload_kg"])
    wind_xy = tuple(float(value) for value in mission["wind_mean_xy_mps"])
    wind = wind_scenario_class(
        scenario_id=f"{mission['mission_id']}_UNIFORM_BOUNDED_ERROR",
        kind="UNIFORM_WITH_BOUNDED_ERROR",
        definition={"mean_xy_mps": list(wind_xy), "use_forecast_error": True},
        anchor_unit_xy=(1.0, 0.0),
        error_vertices_xy_mps=tuple(
            (float(row[0]), float(row[1])) for row in mission["forecast_error_vertices_xy_mps"]
        ),
        error_temporal_coupling="ONE_STATIC_VERTEX_SHARED_WITHIN_EACH_LEG",
    )
    candidate_cache: dict[tuple[int, int, int], dict[str, Any] | None] = {}

    def remaining(mask: int) -> float:
        return sum(packages[target] for index, target in enumerate(targets) if not (mask & (1 << index)))

    def best_leg(left: int, right: int, mask: int) -> dict[str, Any] | None:
        key = (left, right, mask)
        if key in candidate_cache:
            return candidate_cache[key]
        routes = pool["platform_pair_candidates"][f"{left}->{right}"]["candidates"]
        payload = remaining(mask)
        candidates: list[tuple[tuple[float, ...], dict[str, Any]]] = []
        for route in routes:
            speeds = [10.0] if method == METHOD_DISTANCE else speed_grid(model)
            for speed in speeds:
                raw_evaluation = evaluate_vector_route(model, route, payload, float(speed), wind)
                required_feasible = (
                    raw_evaluation["nominal_feasible"]
                    if method == METHOD_DISTANCE
                    else raw_evaluation["robust_feasible"]
                )
                if not required_feasible:
                    continue
                evaluation = compact_exact_evaluation(raw_evaluation)
                if method == METHOD_DISTANCE:
                    ranking = (
                        float(route["route_length_m"]),
                        float(evaluation["flight_duration_s"]),
                        float(evaluation["flight_nominal_energy_j"]),
                    )
                else:
                    ranking = (
                        float(evaluation["flight_robust_energy_j"]),
                        float(evaluation["flight_nominal_energy_j"]),
                        float(evaluation["flight_duration_s"]),
                        float(route["route_length_m"]),
                    )
                candidates.append(
                    (
                        ranking,
                        {
                            "route": route,
                            "evaluation": evaluation,
                            "payload_departure_kg": payload,
                        },
                    )
                )
        selected = None if not candidates else min(candidates, key=lambda row: row[0])[1]
        candidate_cache[key] = selected
        return selected

    def transition(left: int, right: int, mask: int, target_local: int | None) -> float:
        selected = best_leg(left, right, mask)
        if selected is None:
            return math.inf
        if method == METHOD_DISTANCE:
            return float(selected["route"]["route_length_m"])
        value = float(selected["evaluation"]["flight_robust_energy_j"])
        if target_local is not None:
            target = targets[target_local]
            service = model.service(
                remaining(mask),
                packages[target],
                service_time[target],
                service_altitude[target],
            )
            value += float(service["service_total_energy_j"])
        return value

    order, objective, evaluations = held_karp(depot, targets, transition)
    sequence = [depot, *order, depot]
    mask = 0
    payload = initial_payload
    local_lookup = {target: index for index, target in enumerate(targets)}
    legs = []
    totals = {
        "distance_m": 0.0,
        "flight_time_s": 0.0,
        "service_time_s": 0.0,
        "nominal_modeled_energy_j": 0.0,
        "robust_modeled_energy_bound_j": 0.0,
        "robust_modeled_energy_bound_valid": True,
    }
    for ordinal, (left, right) in enumerate(zip(sequence[:-1], sequence[1:]), start=1):
        selected = best_leg(left, right, mask)
        if selected is None:
            raise RuntimeError("Selected transition became infeasible")
        route = selected["route"]
        evaluation = selected["evaluation"]
        service_record = None
        if right in local_lookup:
            service_record = model.service(
                payload,
                packages[right],
                service_time[right],
                service_altitude[right],
            )
            mask |= 1 << local_lookup[right]
            payload -= packages[right]
        totals["distance_m"] += float(route["route_length_m"])
        totals["flight_time_s"] += float(evaluation["flight_duration_s"])
        totals["nominal_modeled_energy_j"] += float(evaluation["flight_nominal_energy_j"])
        if evaluation["flight_robust_energy_j"] is None:
            totals["robust_modeled_energy_bound_valid"] = False
        else:
            totals["robust_modeled_energy_bound_j"] += float(evaluation["flight_robust_energy_j"])
        if service_record is not None:
            totals["service_time_s"] += float(
                service_record["descent_duration_s"]
                + service_record["ground_service_duration_s"]
                + service_record["ascent_duration_s"]
            )
            totals["nominal_modeled_energy_j"] += float(service_record["service_total_energy_j"])
            if totals["robust_modeled_energy_bound_valid"]:
                totals["robust_modeled_energy_bound_j"] += float(service_record["service_total_energy_j"])
        legs.append(
            {
                "ordinal": ordinal,
                "from_roster_index": left,
                "to_roster_index": right,
                "payload_departure_kg": float(selected["payload_departure_kg"]),
                "route_candidate_id": route["route_candidate_id"],
                "route_hash": route["route_hash"],
                "route_length_m": float(route["route_length_m"]),
                "ground_speed_mps": float(evaluation["ground_speed_mps"]),
                "flight_duration_s": float(evaluation["flight_duration_s"]),
                "flight_nominal_energy_j": float(evaluation["flight_nominal_energy_j"]),
                "flight_robust_energy_bound_j": evaluation["flight_robust_energy_j"],
                "minimum_airspeed_mps": float(evaluation["minimum_airspeed_mps"]),
                "maximum_airspeed_mps": float(evaluation["maximum_airspeed_mps"]),
                "minimum_public_power_domain_margin_mps": float(evaluation["minimum_public_power_domain_margin_mps"]),
                "execution_duration_screen": evaluation["execution_duration_screen"],
                "minimum_exact_buffered_clearance_m": float(route["minimum_exact_buffered_clearance_m"]),
                "visibility_path_xy_m": route["visibility_path_xy_m"],
                "service": service_record,
            }
        )
    totals["mission_time_s"] = totals["flight_time_s"] + totals["service_time_s"]
    if not totals["robust_modeled_energy_bound_valid"]:
        totals["robust_modeled_energy_bound_j"] = None
    return {
        "schema": "GPENMPC_MULTI_CITY_PLANNING_CASE_V1",
        "status": "PASS_COMPLETE_FINITE_POOL_PLAN",
        "mission_id": mission["mission_id"],
        "city_id": mission["city_id"],
        "method": method,
        "visit_order_roster_indices": order,
        "closed_sequence_roster_indices": sequence,
        "objective_value": objective,
        "objective_unit": "m" if method == METHOD_DISTANCE else "J",
        "held_karp_transition_evaluations": evaluations,
        "wind_mean_xy_mps": list(wind_xy),
        "forecast_error_vertices_xy_mps": mission["forecast_error_vertices_xy_mps"],
        "initial_payload_kg": initial_payload,
        "legs": legs,
        "totals": totals,
        "claim_boundary": "Minimum modeled cost over the directed candidate routes and discrete speed choices.",
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--assets-root", required=True, type=Path)
    parser.add_argument("--planner-src", required=True, type=Path)
    parser.add_argument("--m600-profile", required=True, type=Path)
    parser.add_argument("--output-root", required=True, type=Path)
    args = parser.parse_args()
    sys.path.insert(0, str(args.planner_src.resolve()))
    from gpenmpc_wind.m600_power import M600PowerModel
    from gpenmpc_wind.planner import evaluate_vector_route
    from gpenmpc_wind.wind_field import WindScenario

    model = M600PowerModel.from_profile(args.m600_profile.resolve())
    cases = []
    planning_refs = []
    for city_folder in sorted(path for path in args.assets_root.iterdir() if path.is_dir()):
        mission_paths = sorted((city_folder / "missions").glob("MISSION_*.json"))
        route_paths = sorted((city_folder / "routes").glob("ROUTE_POOL_*.json"))
        missions = {json.loads(path.read_text(encoding="utf-8"))["mission_id"]: path for path in mission_paths}
        routes = {json.loads(path.read_text(encoding="utf-8"))["mission_id"]: path for path in route_paths}
        if set(missions) != set(routes) or len(missions) != 2:
            raise RuntimeError(f"Mission/route identity mismatch for {city_folder}")
        for mission_id in sorted(missions):
            mission = json.loads(missions[mission_id].read_text(encoding="utf-8"))
            pool = json.loads(routes[mission_id].read_text(encoding="utf-8"))
            for method in METHODS:
                result = solve_mission(model, evaluate_vector_route, WindScenario, mission, pool, method)
                reference = write_content_addressed_json(
                    args.output_root / "cases" / mission_id,
                    "PLANNING_CASE",
                    result,
                )
                cases.append(result)
                planning_refs.append(reference)

    paired = []
    for mission_id in sorted({case["mission_id"] for case in cases}):
        by_method = {case["method"]: case for case in cases if case["mission_id"] == mission_id}
        distance = by_method[METHOD_DISTANCE]
        energy = by_method[METHOD_ENERGY]
        distance_energy = float(distance["totals"]["nominal_modeled_energy_j"])
        optimized_energy = float(energy["totals"]["nominal_modeled_energy_j"])
        distance_robust_value = distance["totals"]["robust_modeled_energy_bound_j"]
        optimized_robust = float(energy["totals"]["robust_modeled_energy_bound_j"])
        route_changes = sum(
            left["route_candidate_id"] != right["route_candidate_id"]
            for left, right in zip(distance["legs"], energy["legs"])
        ) if distance["closed_sequence_roster_indices"] == energy["closed_sequence_roster_indices"] else None
        paired.append(
            {
                "mission_id": mission_id,
                "city_id": distance["city_id"],
                "energy_minus_distance_modeled_energy_j": optimized_energy - distance_energy,
                "energy_reduction_fraction": (distance_energy - optimized_energy) / distance_energy,
                "distance_plan_robust_bound_valid": distance_robust_value is not None,
                "robust_bound_difference_j": None
                if distance_robust_value is None
                else optimized_robust - float(distance_robust_value),
                "robust_bound_reduction_fraction": None
                if distance_robust_value is None
                else (float(distance_robust_value) - optimized_robust) / float(distance_robust_value),
                "mission_time_difference_s": float(energy["totals"]["mission_time_s"]) - float(distance["totals"]["mission_time_s"]),
                "distance_difference_m": float(energy["totals"]["distance_m"]) - float(distance["totals"]["distance_m"]),
                "visit_order_changed": distance["visit_order_roster_indices"] != energy["visit_order_roster_indices"],
                "route_changes_if_same_order": route_changes,
                "speed_changed": any(abs(float(left["ground_speed_mps"]) - float(right["ground_speed_mps"])) > 1e-12 for left, right in zip(distance["legs"], energy["legs"])),
            }
        )
    matrix = {
        "schema": "GPENMPC_MULTI_CITY_PLANNING_MATRIX_V1",
        "status": "PASS_6_MISSIONS_X_2_PLANNERS_COMPLETE",
        "planned_cases": 12,
        "executed_cases": len(cases),
        "valid_complete_cases": sum(case["status"] == "PASS_COMPLETE_FINITE_POOL_PLAN" for case in cases),
        "implementation_failures": 0,
        "methods": list(METHODS),
        "cases": cases,
        "paired_energy_minus_distance": paired,
        "inputs": {
            "assets_root": str(args.assets_root.resolve()),
            "m600_profile": file_reference(args.m600_profile.resolve()),
        },
    }
    reference = write_content_addressed_json(args.output_root, "PLANNING_MATRIX", matrix)
    print(json.dumps(reference, indent=2))


if __name__ == "__main__":
    main()
