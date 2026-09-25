"""Run one bundled city mission with the finite-pool planner.

The command selects bundled inputs, runs the planner and writes a
content-addressed result.
"""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
from typing import Any

from multi_urban_common import write_content_addressed_json
from gpenmpc_wind.m600_power import M600PowerModel
from gpenmpc_wind.planner import evaluate_vector_route
from gpenmpc_wind.wind_field import WindScenario
from run_multicity_planning_matrix import METHODS, METHOD_ENERGY, solve_mission


HERE = Path(__file__).resolve().parent
REPOSITORY = HERE.parent
CITIES = {
    "cambridge_ma": "CAMBRIDGE_MA",
    "manhattan_ny": "MANHATTAN_NY",
    "seattle_wa": "SEATTLE_WA",
}
PROFILE_SHA256 = "F8D1CF856FA4EAEC3CD209BFB3398169F9F987379998EF9B3068B33BE0C70764"


def read_asset(path: Path) -> dict[str, Any]:
    payload = path.read_bytes()
    expected_prefix = path.stem.rsplit("_", 1)[-1].upper()
    actual = hashlib.sha256(payload).hexdigest().upper()
    if len(expected_prefix) != 16 or actual[:16] != expected_prefix:
        raise ValueError(f"Asset content hash mismatch: {path}")
    return json.loads(payload.decode("utf-8"))


def bundled_inputs() -> tuple[M600PowerModel, dict[str, tuple[dict[str, Any], dict[str, Any]]]]:
    profile_path = HERE / "assets" / "m600_profile.json"
    if hashlib.sha256(profile_path.read_bytes()).hexdigest().upper() != PROFILE_SHA256:
        raise ValueError("Bundled M600 profile content hash mismatch")
    model = M600PowerModel.from_profile(profile_path)
    missions: dict[str, tuple[dict[str, Any], dict[str, Any]]] = {}
    for folder, city_id in CITIES.items():
        city = REPOSITORY / "data" / "cities" / folder
        map_files = sorted((city / "map").glob("*.json"))
        if len(map_files) != 2:
            raise ValueError(f"Expected the exact and controller maps in {folder}")
        for map_path in map_files:
            read_asset(map_path)
        mission_files = sorted((city / "missions").glob("MISSION_*.json"))
        route_files = sorted((city / "routes").glob("ROUTE_POOL_*.json"))
        if len(mission_files) != 2 or len(route_files) != 2:
            raise ValueError(f"Expected two missions and route pools in {folder}")
        route_rows = [read_asset(path) for path in route_files]
        pools = {row["mission_id"]: row for row in route_rows}
        if len(pools) != 2:
            raise ValueError(f"Duplicate route-pool mission identities in {folder}")
        for path in mission_files:
            mission = read_asset(path)
            mission_id = mission["mission_id"]
            if mission["city_id"] != city_id or mission_id in missions or mission_id not in pools:
                raise ValueError(f"Mission/route/city identity mismatch: {path}")
            services = mission["service_points"]
            if len(services) != 4:
                raise ValueError(f"Expected four delivery points: {path}")
            for point in services:
                if abs(float(point["xyz_m"][2]) - 10.0) > 1e-12:
                    raise ValueError(f"Bundled task height differs from 10 m: {path}")
                service = model.service(
                    float(mission["initial_payload_kg"]),
                    float(point["package_mass_kg"]),
                    float(point["service_duration_s"]),
                    float(point["xyz_m"][2]),
                )
                duration = sum(service[key] for key in (
                    "descent_duration_s", "ground_service_duration_s", "ascent_duration_s"
                ))
                if abs(service["service_target_altitude_m"] - 1.004) > 1e-12:
                    raise ValueError("Service target does not match the bundled model")
                if abs(duration - 27.992) > 1e-12:
                    raise ValueError("Service duration does not match the bundled model")
            missions[mission_id] = (mission, pools[mission_id])
    return model, missions


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--list", action="store_true", help="List bundled missions")
    parser.add_argument("--check-inputs", action="store_true", help="Validate bundled asset hashes and service-height settings")
    parser.add_argument("--mission", default="MU_CAMBRIDGE_MA_01")
    parser.add_argument("--method", choices=METHODS, default=METHOD_ENERGY)
    parser.add_argument("--output-root", type=Path, default=REPOSITORY / "outputs" / "planning")
    args = parser.parse_args()
    model, missions = bundled_inputs()
    if args.list or args.check_inputs:
        print(json.dumps({
            "missions": sorted(missions),
            "mission_count": len(missions),
            "deliveries_per_mission": 4,
            "service_start_altitude_m": 10.0,
            "service_target_altitude_m": 1.004,
            "service_duration_s": 27.992,
            "planning_executed": False,
            "inputs_valid": True,
        }, indent=2))
        return
    if args.mission not in missions:
        parser.error(f"Unknown mission {args.mission}; use --list")
    destination = args.output_root.resolve()
    for protected in (REPOSITORY / "data", HERE / "assets", HERE / "reference"):
        protected = protected.resolve()
        if destination == protected or protected in destination.parents:
            parser.error("Output must not be written into bundled input/reference directories")
    mission, pool = missions[args.mission]
    result = solve_mission(model, evaluate_vector_route, WindScenario, mission, pool, args.method)
    reference = write_content_addressed_json(destination / args.mission, "PLANNING_CASE", result)
    print(json.dumps({
        "mission_id": result["mission_id"],
        "method": result["method"],
        "visit_order_roster_indices": result["visit_order_roster_indices"],
        "totals": result["totals"],
        "result": reference,
    }, indent=2))


if __name__ == "__main__":
    main()
