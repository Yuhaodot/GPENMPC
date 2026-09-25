"""Build exact-polygon multi-city maps, missions and finite route pools.

This is geometry-only preparation. Road centre-lines seed a connected graph,
while collision validity is determined by full public building polygons
buffered by the common software-platform envelope and the inner planning
boundary.
"""

from __future__ import annotations

import argparse
import heapq
import json
import math
from collections import Counter, deque
from pathlib import Path
from typing import Any, Iterable

import numpy as np
import pyarrow.parquet as pq
from pyproj import Transformer
from shapely import affinity, normalize, wkb
from shapely.geometry import LineString, MultiLineString, MultiPolygon, Point, Polygon, box
from shapely.geometry.base import BaseGeometry
from shapely.ops import transform, unary_union
from shapely.validation import make_valid

from multi_urban_common import file_reference, sha256_bytes, write_content_addressed_json


SCHEMA = "GPENMPC_MULTI_CITY_EXACT_POLYGON_ASSET_BUILD_V1"
SOURCE_CRS = "EPSG:4326"
CITIES = {
    "CAMBRIDGE_MA": {
        "folder": "cambridge_ma",
        "projected_crs": "EPSG:32618",
        "morphology_hypothesis": "IRREGULAR_LOWER_DENSITY_NETWORK",
    },
    "MANHATTAN_NY": {
        "folder": "manhattan_ny",
        "projected_crs": "EPSG:32618",
        "morphology_hypothesis": "REGULAR_HIGH_DENSITY_URBAN_CANYON",
    },
    "SEATTLE_WA": {
        "folder": "seattle_wa",
        "projected_crs": "EPSG:32610",
        "morphology_hypothesis": "MIXED_GRID_NETWORK",
    },
}


def iter_polygons(geometry: BaseGeometry) -> Iterable[Polygon]:
    if geometry.is_empty:
        return
    if isinstance(geometry, Polygon):
        yield geometry
    elif isinstance(geometry, MultiPolygon):
        yield from geometry.geoms
    elif hasattr(geometry, "geoms"):
        for item in geometry.geoms:
            yield from iter_polygons(item)


def iter_lines(geometry: BaseGeometry) -> Iterable[LineString]:
    if geometry.is_empty:
        return
    if isinstance(geometry, LineString):
        yield geometry
    elif isinstance(geometry, MultiLineString):
        yield from geometry.geoms
    elif hasattr(geometry, "geoms"):
        for item in geometry.geoms:
            yield from iter_lines(item)


def clean_geometry(geometry: BaseGeometry) -> BaseGeometry:
    if geometry.is_empty:
        return geometry
    result = geometry if geometry.is_valid else make_valid(geometry)
    return normalize(result)


def read_table(path: Path, names: list[str]) -> tuple[dict[str, list[Any]], int]:
    table = pq.read_table(path, columns=names)
    return ({name: table[name].to_pylist() for name in names}, table.num_rows)


def dominant_road_axis_degrees(lines: list[LineString]) -> tuple[float, dict[str, Any]]:
    weights = np.zeros(180, dtype=float)
    total = 0.0
    for line in lines:
        coordinates = np.asarray(line.coords, dtype=float)
        delta = np.diff(coordinates, axis=0)
        lengths = np.linalg.norm(delta, axis=1)
        angles = np.mod(np.degrees(np.arctan2(delta[:, 1], delta[:, 0])), 90.0)
        indices = np.clip(np.floor(angles * 2.0).astype(int), 0, 179)
        np.add.at(weights, indices, lengths)
        total += float(np.sum(lengths))
    if total <= 0.0:
        raise RuntimeError("No positive-length road segments")
    winner = int(np.argmax(weights))
    probabilities = weights / total
    positive = probabilities[probabilities > 0.0]
    entropy = float(-np.sum(positive * np.log(positive)) / math.log(len(weights)))
    order = np.argsort(weights)[::-1][:10]
    return (
        (winner + 0.5) / 2.0,
        {
            "normalized_orientation_entropy": entropy,
            "dominant_half_degree_bin_fraction": float(weights[winner] / total),
            "top_bins": [
                {
                    "center_degrees_modulo_90": float((int(index) + 0.5) / 2.0),
                    "length_fraction": float(weights[int(index)] / total),
                }
                for index in order
            ],
        },
    )


def geometry_to_polygon_json(geometry: BaseGeometry) -> list[dict[str, Any]]:
    records: list[dict[str, Any]] = []
    for polygon in iter_polygons(clean_geometry(geometry)):
        if polygon.area <= 1e-9:
            continue
        records.append(
            {
                "exterior_xy_m": [[float(x), float(y)] for x, y in polygon.exterior.coords],
                "holes_xy_m": [
                    [[float(x), float(y)] for x, y in ring.coords]
                    for ring in polygon.interiors
                ],
            }
        )
    records.sort(
        key=lambda row: (
            min(point[0] for point in row["exterior_xy_m"]),
            min(point[1] for point in row["exterior_xy_m"]),
            len(row["exterior_xy_m"]),
        )
    )
    return records


def geometry_to_line_json(geometry: BaseGeometry) -> list[list[list[float]]]:
    records = [
        [[float(x), float(y)] for x, y in line.coords]
        for line in iter_lines(geometry)
        if line.length > 1e-9
    ]
    records.sort(key=lambda row: (row[0][0], row[0][1], len(row)))
    return records


def width_values(rules: Any) -> list[float]:
    values: list[float] = []
    for rule in rules or []:
        value = rule.get("value") if isinstance(rule, dict) else None
        if value is not None and math.isfinite(float(value)) and float(value) > 0.0:
            values.append(float(value))
    return values


def coordinate_key(point: tuple[float, float]) -> tuple[int, int]:
    return (int(round(point[0] * 1000.0)), int(round(point[1] * 1000.0)))


def build_road_graph(
    roads: list[LineString],
    inner: Polygon,
    buffered_obstacles: BaseGeometry,
    spacing_m: float,
) -> tuple[list[tuple[float, float]], list[dict[int, float]]]:
    noded = unary_union(roads).intersection(inner)
    points: list[tuple[float, float]] = []
    index_by_key: dict[tuple[int, int], int] = {}
    edges: list[tuple[int, int, float]] = []

    def node_index(point: Point) -> int:
        key = coordinate_key((point.x, point.y))
        if key not in index_by_key:
            index_by_key[key] = len(points)
            points.append((key[0] / 1000.0, key[1] / 1000.0))
        return index_by_key[key]

    for line in iter_lines(noded):
        count = max(1, int(math.ceil(line.length / spacing_m)))
        samples = [line.interpolate(index / count, normalized=True) for index in range(count + 1)]
        indices: list[int | None] = []
        for point in samples:
            valid = (
                inner.covers(point)
                and not buffered_obstacles.intersects(point)
                and point.distance(buffered_obstacles) > 1e-6
            )
            indices.append(node_index(point) if valid else None)
        for first, second, left, right in zip(indices[:-1], indices[1:], samples[:-1], samples[1:]):
            if first is None or second is None or first == second:
                continue
            segment = LineString([(left.x, left.y), (right.x, right.y)])
            if not inner.covers(segment) or not segment.disjoint(buffered_obstacles):
                continue
            edges.append((first, second, float(segment.length)))

    adjacency: list[dict[int, float]] = [dict() for _ in points]
    for first, second, distance in edges:
        prior = adjacency[first].get(second)
        if prior is None or distance < prior:
            adjacency[first][second] = distance
            adjacency[second][first] = distance
    return points, adjacency


def largest_component(adjacency: list[dict[int, float]]) -> list[int]:
    visited = [False] * len(adjacency)
    components: list[list[int]] = []
    for start in range(len(adjacency)):
        if visited[start] or not adjacency[start]:
            continue
        queue: deque[int] = deque([start])
        visited[start] = True
        component: list[int] = []
        while queue:
            node = queue.popleft()
            component.append(node)
            for neighbor in adjacency[node]:
                if not visited[neighbor]:
                    visited[neighbor] = True
                    queue.append(neighbor)
        components.append(component)
    if not components:
        raise RuntimeError("No connected free road graph survived")
    return max(components, key=lambda row: (len(row), -min(row)))


def dijkstra(
    adjacency: list[dict[int, float]],
    start: int,
    goal: int,
    banned_nodes: frozenset[int] = frozenset(),
    banned_edges: frozenset[tuple[int, int]] = frozenset(),
) -> tuple[float, tuple[int, ...]]:
    if start in banned_nodes or goal in banned_nodes:
        return math.inf, ()
    queue: list[tuple[float, tuple[int, ...], int]] = [(0.0, (start,), start)]
    best: dict[int, tuple[float, tuple[int, ...]]] = {start: (0.0, (start,))}
    while queue:
        distance, path, node = heapq.heappop(queue)
        if best.get(node) != (distance, path):
            continue
        if node == goal:
            return distance, path
        for neighbor in sorted(adjacency[node]):
            if neighbor in banned_nodes or (node, neighbor) in banned_edges or neighbor in path:
                continue
            proposal = (
                distance + float(adjacency[node][neighbor]),
                path + (neighbor,),
            )
            if neighbor not in best or proposal < best[neighbor]:
                best[neighbor] = proposal
                heapq.heappush(queue, (proposal[0], proposal[1], neighbor))
    return math.inf, ()


def path_cost(adjacency: list[dict[int, float]], path: tuple[int, ...]) -> float:
    return float(sum(adjacency[left][right] for left, right in zip(path[:-1], path[1:])))


def yen_paths(
    adjacency: list[dict[int, float]], start: int, goal: int, maximum: int
) -> list[tuple[float, tuple[int, ...]]]:
    distance, first = dijkstra(adjacency, start, goal)
    if not first:
        return []
    accepted: list[tuple[float, tuple[int, ...]]] = [(distance, first)]
    accepted_paths = {first}
    candidates: list[tuple[float, tuple[int, ...]]] = []
    candidate_paths: set[tuple[int, ...]] = set()
    while len(accepted) < maximum:
        previous = accepted[-1][1]
        for spur_index in range(len(previous) - 1):
            root = previous[: spur_index + 1]
            banned_edges: set[tuple[int, int]] = set()
            for _, path in accepted:
                if len(path) > spur_index and path[: spur_index + 1] == root:
                    banned_edges.add((path[spur_index], path[spur_index + 1]))
            _, spur = dijkstra(
                adjacency,
                root[-1],
                goal,
                banned_nodes=frozenset(root[:-1]),
                banned_edges=frozenset(banned_edges),
            )
            if not spur:
                continue
            total = root[:-1] + spur
            if total in accepted_paths or total in candidate_paths:
                continue
            cost = path_cost(adjacency, total)
            heapq.heappush(candidates, (cost, total))
            candidate_paths.add(total)
        if not candidates:
            break
        candidate = heapq.heappop(candidates)
        candidate_paths.remove(candidate[1])
        accepted.append(candidate)
        accepted_paths.add(candidate[1])
    return accepted


def select_task_nodes(
    points: list[tuple[float, float]],
    component: list[int],
    inner: Polygon,
    buffered_obstacles: BaseGeometry,
    task_ordinal: int,
) -> list[int]:
    candidates = [
        index
        for index in component
        if Point(points[index]).distance(inner.boundary) >= 25.0
        and Point(points[index]).distance(buffered_obstacles) >= 2.0
    ]
    if len(candidates) < 20:
        raise RuntimeError("Too few mission-node candidates")
    if task_ordinal == 1:
        depot = min(candidates, key=lambda index: (points[index][0] + points[index][1], points[index]))
    elif task_ordinal == 2:
        depot = max(candidates, key=lambda index: (points[index][0] - points[index][1], points[index]))
    else:
        raise ValueError(task_ordinal)
    eligible = [
        index
        for index in candidates
        if 110.0 <= math.dist(points[depot], points[index]) <= 430.0
    ]
    selected = [depot]
    while len(selected) < 5:
        scored = []
        for index in eligible:
            if index in selected:
                continue
            minimum = min(math.dist(points[index], points[item]) for item in selected)
            if minimum < 70.0:
                continue
            scored.append((minimum, math.dist(points[depot], points[index]), points[index], index))
        if not scored:
            raise RuntimeError(f"Unable to select four separated delivery nodes for task {task_ordinal}")
        maximum = max(row[0] for row in scored)
        selected.append(min((row for row in scored if abs(row[0] - maximum) <= 1e-12), key=lambda row: row[2])[3])
    return selected


def simplify_path(
    points: list[tuple[float, float]],
    inner: Polygon,
    buffered_obstacles: BaseGeometry,
) -> list[list[float]]:
    """Remove road-sampling points without weakening continuous collision checks.

    Overture road features are sampled densely to construct the graph.  Those
    samples define graph connectivity, but they are not physical stop/turn
    waypoints.  The C3 duration screen applies a transition bound to
    every retained waypoint, so passing the raw samples to it would create a
    fictitious duration penalty.  This deterministic farthest-visible pruning
    retains a point only when a direct segment to a later point would leave the
    planning polygon or intersect a buffered building.
    """
    if len(points) <= 2:
        return [[float(x), float(y)] for x, y in points]

    kept = [points[0]]
    left_index = 0
    final_index = len(points) - 1
    while left_index < final_index:
        chosen = left_index + 1
        for right_index in range(final_index, left_index, -1):
            segment = LineString([points[left_index], points[right_index]])
            if inner.covers(segment) and segment.disjoint(buffered_obstacles):
                chosen = right_index
                break
        kept.append(points[chosen])
        left_index = chosen

    simplified = LineString(kept)
    if not inner.covers(simplified) or not simplified.disjoint(buffered_obstacles):
        raise RuntimeError("Collision-safe path simplification failed exact replay")
    return [[float(x), float(y)] for x, y in kept]


def task_and_routes(
    city_id: str,
    ordinal: int,
    points: list[tuple[float, float]],
    adjacency: list[dict[int, float]],
    component: list[int],
    inner: Polygon,
    planning_obstacles: BaseGeometry,
    controller_obstacles: BaseGeometry,
) -> tuple[dict[str, Any], dict[str, Any]]:
    selected = select_task_nodes(points, component, inner, planning_obstacles, ordinal)
    local_node_id = {graph_index: local + 1 for local, graph_index in enumerate(selected)}
    nodes = [
        {
            "roster_index": local_node_id[index],
            "graph_node_index": int(index),
            "role": "DEPOT" if local == 0 else "DELIVERY",
            "xyz_m": [float(points[index][0]), float(points[index][1]), 10.0],
        }
        for local, index in enumerate(selected)
    ]
    package_pattern = [0.62, 0.48, 0.71, 0.39] if ordinal == 1 else [0.43, 0.77, 0.55, 0.46]
    mission = {
        "schema": "GPENMPC_MULTI_CITY_MISSION_V1",
        "mission_id": f"MU_{city_id}_{ordinal:02d}",
        "city_id": city_id,
        "task_ordinal": ordinal,
        "depot": nodes[0],
        "service_points": [
            {
                **node,
                "package_mass_kg": package_pattern[index],
                "service_duration_s": 8.0,
            }
            for index, node in enumerate(nodes[1:])
        ],
        "initial_payload_kg": float(sum(package_pattern)),
        "wind_mean_xy_mps": [2.5, 1.5] if ordinal == 1 else [-2.0, 2.5],
        "forecast_error_vertices_xy_mps": [[-0.35, -0.35], [-0.35, 0.35], [0.35, -0.35], [0.35, 0.35]],
        "method_blind_selection": True,
        "claim_boundary": "SOFTWARE_TASK_ON_PUBLIC_STATIC_BUILDING_GEOMETRY__SYNTHETIC_DEPOT_AND_DELIVERY_LOCATIONS",
    }
    pair_routes: dict[str, Any] = {}
    for left_pos, left in enumerate(selected):
        for right_pos, right in enumerate(selected):
            if left == right:
                continue
            raw = yen_paths(adjacency, left, right, 12)
            if not raw:
                raise RuntimeError(f"No route for {city_id} task {ordinal}: {left}->{right}")
            shortest = raw[0][0]
            retained: list[dict[str, Any]] = []
            retained_lines: list[LineString] = []
            for distance, path in raw:
                if distance > 1.75 * shortest + 1e-9:
                    continue
                coordinates = [points[index] for index in path]
                graph_line = LineString(coordinates)
                if not inner.covers(graph_line) or not graph_line.disjoint(planning_obstacles):
                    raise RuntimeError("Graph route failed exact continuous collision replay")
                path_xy = simplify_path(coordinates, inner, planning_obstacles)
                line = LineString(path_xy)
                if retained_lines:
                    different = all(line.hausdorff_distance(prior) >= 2.0 for prior in retained_lines)
                    if not different:
                        continue
                payload = {
                    "from_roster_index": left_pos + 1,
                    "to_roster_index": right_pos + 1,
                    "selected_rank": len(retained) + 1,
                    "visibility_path_xy_m": path_xy,
                    "route_length_m": float(line.length),
                    "road_graph_length_m": float(graph_line.length),
                    "raw_graph_point_count": len(coordinates),
                    "visibility_waypoint_count": len(path_xy),
                    "minimum_exact_buffered_clearance_m": float(line.distance(controller_obstacles)),
                    "minimum_c3_reserve_boundary_clearance_m": float(line.distance(planning_obstacles)),
                    "continuous_collision_pass": True,
                }
                identity = {
                    "from": left_pos + 1,
                    "to": right_pos + 1,
                    "path": path_xy,
                    "length_m": round(float(line.length), 9),
                }
                digest = sha256_bytes(json.dumps(identity, sort_keys=True, separators=(",", ":")).encode("utf-8"))
                payload["route_hash"] = digest
                payload["route_candidate_id"] = f"{city_id}_{ordinal:02d}_{left_pos + 1}_TO_{right_pos + 1}_R{len(retained) + 1:02d}_{digest[:12]}"
                retained.append(payload)
                retained_lines.append(line)
                if len(retained) >= 4:
                    break
            if not retained:
                raise RuntimeError("All Yen candidates rejected")
            pair_routes[f"{left_pos + 1}->{right_pos + 1}"] = {
                "candidate_count": len(retained),
                "candidates": retained,
            }
    return mission, {
        "schema": "GPENMPC_MULTI_CITY_FINITE_DIRECTED_ROUTE_POOL_V1",
        "mission_id": mission["mission_id"],
        "city_id": city_id,
        "nodes": nodes,
        "platform_pair_candidates": pair_routes,
        "status": "PASS_EXACT_POLYGON_CONTINUOUS_COLLISION_REPLAY",
    }


def quantiles(values: list[float]) -> dict[str, float | None]:
    if not values:
        return {"p05": None, "p50": None, "p95": None}
    array = np.asarray(values, dtype=float)
    return {
        "p05": float(np.quantile(array, 0.05)),
        "p50": float(np.quantile(array, 0.50)),
        "p95": float(np.quantile(array, 0.95)),
    }


def build_city(
    city_id: str,
    city: dict[str, str],
    source_root: Path,
    output_root: Path,
    spacing_m: float,
    trajectory_corridor_reserve_m: float,
) -> dict[str, Any]:
    folder = source_root / city["folder"]
    building_path = folder / "overture_buildings.geoparquet"
    road_path = folder / "overture_segments.geoparquet"
    for path in (building_path, road_path):
        if not path.is_file():
            raise FileNotFoundError(path)

    building_columns = ["id", "height", "num_floors", "subtype", "class", "has_parts", "geometry"]
    road_columns = ["id", "subtype", "class", "subclass", "width_rules", "geometry"]
    buildings, building_count = read_table(building_path, building_columns)
    roads, road_count = read_table(road_path, road_columns)
    transformer = Transformer.from_crs(SOURCE_CRS, city["projected_crs"], always_xy=True)
    building_utm = [clean_geometry(transform(transformer.transform, wkb.loads(value))) for value in buildings["geometry"]]
    road_utm_all = [clean_geometry(transform(transformer.transform, wkb.loads(value))) for value in roads["geometry"]]
    road_lines = [line for geometry in road_utm_all for line in iter_lines(geometry)]
    road_centroids = np.asarray(
        [[geometry.centroid.x, geometry.centroid.y] for geometry in road_utm_all],
        dtype=float,
    )
    # Overture bbox extraction retains complete source features, so a few long
    # segments can extend far beyond the requested region.  A union-bounds
    # midpoint is therefore not a valid window centre.  The component-wise
    # median of source-feature centroids is deterministic and outlier-robust.
    center_array = np.median(road_centroids, axis=0)
    center = (float(center_array[0]), float(center_array[1]))
    preliminary = box(
        center[0] - 450.0,
        center[1] - 450.0,
        center[0] + 450.0,
        center[1] + 450.0,
    )
    nearby_lines = [
        part
        for line in road_lines
        for part in iter_lines(line.intersection(preliminary))
    ]
    if not nearby_lines:
        raise RuntimeError("No roads survived the robust preliminary city window")
    angle, orientation = dominant_road_axis_degrees(nearby_lines)

    def localize(geometry: BaseGeometry) -> BaseGeometry:
        translated = affinity.translate(geometry, xoff=-center[0], yoff=-center[1])
        return clean_geometry(affinity.rotate(translated, -angle, origin=(0.0, 0.0)))

    outer = box(-350.0, -350.0, 350.0, 350.0)
    inner = box(-250.0, -250.0, 250.0, 250.0)
    selected_building_indices = [
        index for index, geometry in enumerate(building_utm) if localize(geometry).intersects(outer)
    ]
    source_heights = [
        float(buildings["height"][index])
        for index in selected_building_indices
        if buildings["height"][index] is not None and float(buildings["height"][index]) > 0.0
    ]
    median_height = float(np.median(source_heights)) if source_heights else 15.0
    building_records: list[dict[str, Any]] = []
    controller_prisms: list[dict[str, Any]] = []
    collision_geometries: list[BaseGeometry] = []
    height_sources: Counter[str] = Counter()
    for index in selected_building_indices:
        geometry = localize(building_utm[index])
        height = buildings["height"][index]
        floors = buildings["num_floors"][index]
        if height is not None and float(height) > 0.0:
            selected_height = float(height)
            height_source = "OVERTURE_HEIGHT"
        elif floors is not None and float(floors) > 0.0:
            selected_height = 3.2 * float(floors)
            height_source = "OVERTURE_NUM_FLOORS_TIMES_3P2_M"
        else:
            selected_height = median_height
            height_source = "CITY_WINDOW_MEDIAN_IMPUTATION"
        height_sources[height_source] += 1
        collision_geometries.append(geometry)
        buffered_parts = geometry_to_polygon_json(geometry.buffer(0.4, quad_segs=32))
        min_x, min_y, max_x, max_y = geometry.bounds
        controller_prisms.append(
            {
                "source_building_id": str(buildings["id"][index]),
                "broad_phase_bounds_xyz_m": [
                    [float(min_x - 0.4), float(max_x + 0.4)],
                    [float(min_y - 0.4), float(max_y + 0.4)],
                    [0.0, float(selected_height)],
                ],
                "buffered_polygons": [
                    {"exterior_xy_m": item["exterior_xy_m"]}
                    for item in buffered_parts
                ],
                "height_source": height_source,
            }
        )
        building_records.append(
            {
                "id": str(buildings["id"][index]),
                "polygons": geometry_to_polygon_json(geometry),
                "source_height_m": selected_height,
                "height_source": height_source,
                "source_num_floors": floors,
                "source_subtype": buildings["subtype"][index],
                "source_class": buildings["class"][index],
                "source_has_parts": buildings["has_parts"][index],
                "full_polygon_not_clipped_at_inner_boundary": True,
            }
        )
    occupied = unary_union(collision_geometries)
    controller_buffered = occupied.buffer(0.4, quad_segs=32)
    planning_buffered = occupied.buffer(
        0.4 + float(trajectory_corridor_reserve_m), quad_segs=32
    )

    road_records: list[dict[str, Any]] = []
    planning_lines: list[LineString] = []
    explicit_width_segments = 0
    explicit_width_values: list[float] = []
    road_class_counts: Counter[str] = Counter()
    for index, geometry in enumerate(road_utm_all):
        local = localize(geometry).intersection(outer)
        if local.is_empty:
            continue
        values = width_values(roads["width_rules"][index])
        if values:
            explicit_width_segments += 1
            explicit_width_values.extend(values)
        road_class = str(roads["class"][index] or "UNSPECIFIED")
        road_class_counts[road_class] += 1
        for line in iter_lines(local):
            planning_lines.append(line)
        road_records.append(
            {
                "id": str(roads["id"][index]),
                "line_parts_xy_m": geometry_to_line_json(local),
                "source_subtype": roads["subtype"][index],
                "source_class": roads["class"][index],
                "source_subclass": roads["subclass"][index],
                "source_width_rule_values_m": values,
                "width_used_as_collision_truth": False,
            }
        )

    points, adjacency = build_road_graph(
        planning_lines, inner, planning_buffered, spacing_m
    )
    component = largest_component(adjacency)
    edge_count = sum(len(row) for row in adjacency) // 2
    component_edges = sum(
        1 for left in component for right in adjacency[left] if right in set(component) and left < right
    )
    sample_clearances = [Point(points[index]).distance(occupied) for index in component]
    map_payload = {
        "schema": "GPENMPC_MULTI_CITY_EXACT_POLYGON_MAP_V1",
        "status": "PASS_EXACT_PUBLIC_POLYGON_STATIC_MAP_PREPARED",
        "map_id": f"{city_id}_700_CONTEXT_500_PLANNING",
        "city_id": city_id,
        "source_crs": SOURCE_CRS,
        "projected_crs": city["projected_crs"],
        "local_transform": {
            "projected_center_xy_m": [float(center[0]), float(center[1])],
            "clockwise_rotation_degrees": float(angle),
            "center_selection": "COMPONENTWISE_MEDIAN_OF_OVERTURE_ROAD_FEATURE_CENTROIDS__ROBUST_TO_UNCLIPPED_LONG_FEATURES",
            "orientation_selection": "DOMINANT_LENGTH_WEIGHTED_AXIS_WITHIN_900_M_PRELIMINARY_WINDOW",
        },
        "outer_context_bounds_xy_m": [-350.0, -350.0, 350.0, 350.0],
        "inner_planning_bounds_xy_m": [-250.0, -250.0, 250.0, 250.0],
        "collision_source_of_truth": "FULL_PUBLIC_BUILDING_POLYGONS_BUFFERED_0P4_M_AND_INNER_PLANNING_BOUNDARY",
        "trajectory_generation_corridor": {
            "controller_collision_buffer_m": 0.4,
            "additional_c3_reserve_m": float(trajectory_corridor_reserve_m),
            "graph_exclusion_total_from_building_footprint_m": 0.4
            + float(trajectory_corridor_reserve_m),
            "role": "Common C3 trajectory-formation margin.",
        },
        "road_semantics": "Road centerlines define graph connectivity; building polygons define collision geometry.",
        "source_buildings": building_records,
        "roads": road_records,
        "source_files": {
            "overture_buildings": file_reference(building_path),
            "overture_segments": file_reference(road_path),
        },
    }
    controller_map_payload = {
        "schema": "GPENMPC_MULTI_CITY_CONTROLLER_COLLISION_MAP_V1",
        "status": "PASS_EXACT_PUBLIC_BUILDING_POLYGON_CONTROLLER_MAP_PREPARED",
        "map_id": f"{city_id}_500_PLANNING_CONTROLLER_COLLISION",
        "city_id": city_id,
        "inner_planning_bounds_xy_m": [-250.0, -250.0, 250.0, 250.0],
        "platform_collision_geometry": {
            "M600_PRO_OSTI_PUBLIC_POWER_REFERENCE": {
                "nominal_collision_envelope_m": 0.25,
                "conservative_buffer_radius_m": 0.4,
                "building_prisms": controller_prisms,
            }
        },
        "claim_boundary": "STATIC_PUBLIC_BUILDING_POLYGON_SOFTWARE_COLLISION_MODEL__ROUTES_AVOID_ALL_BUFFERED_FOOTPRINTS_INDEPENDENT_OF_BUILDING_HEIGHT",
    }
    audit = {
        "schema": "GPENMPC_MULTI_CITY_GEOMETRY_AUDIT_V1",
        "status": "PASS_GEOMETRY_ONLY_ASSET_BUILD",
        "city_id": city_id,
        "morphology_hypothesis": city["morphology_hypothesis"],
        "source_counts": {"buildings": building_count, "road_segments": road_count},
        "context_counts": {
            "buildings": len(building_records),
            "road_records": len(road_records),
            "road_graph_nodes": len(points),
            "road_graph_edges": edge_count,
            "largest_component_nodes": len(component),
            "largest_component_edges": component_edges,
        },
        "building_height_sources": dict(sorted(height_sources.items())),
        "building_coverage_fraction_inner": float(occupied.intersection(inner).area / inner.area),
        "road_length_density_m_per_m2_inner": float(
            unary_union(planning_lines).intersection(inner).length / inner.area
        ),
        "road_orientation": orientation,
        "road_class_counts": dict(sorted(road_class_counts.items())),
        "road_width_metadata": {
            "context_segments_with_explicit_width_rules": explicit_width_segments,
            "context_road_record_count": len(road_records),
            "fraction_with_explicit_width_rules": (
                explicit_width_segments / len(road_records) if road_records else 0.0
            ),
            "width_rule_values_m_quantiles": quantiles(explicit_width_values),
            "used_as_collision_truth": False,
        },
        "free_road_node_clearance_to_unbuffered_buildings_m": quantiles(sample_clearances),
        "full_buildings_retained_across_inner_boundary": True,
        "inner_boundary_building_clipping_count": 0,
        "trajectory_c3_corridor_reserve_m": float(trajectory_corridor_reserve_m),
    }
    city_output = output_root / city_id.lower()
    map_ref = write_content_addressed_json(city_output / "map", "EXACT_CITY_MAP", map_payload)
    controller_map_ref = write_content_addressed_json(
        city_output / "map", "CONTROLLER_EXACT_MAP", controller_map_payload
    )
    audit_ref = write_content_addressed_json(city_output / "audit", "CITY_GEOMETRY_AUDIT", audit)
    mission_refs = []
    route_refs = []
    for ordinal in (1, 2):
        mission, routes = task_and_routes(
            city_id,
            ordinal,
            points,
            adjacency,
            component,
            inner,
            planning_buffered,
            controller_buffered,
        )
        mission_refs.append(
            write_content_addressed_json(city_output / "missions", "MISSION", mission)
        )
        route_refs.append(
            write_content_addressed_json(city_output / "routes", "ROUTE_POOL", routes)
        )
    return {
        "city_id": city_id,
        "map": map_ref,
        "controller_map": controller_map_ref,
        "audit": audit_ref,
        "missions": mission_refs,
        "route_pools": route_refs,
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--source-root", required=True, type=Path)
    parser.add_argument("--output-root", required=True, type=Path)
    parser.add_argument("--road-sample-spacing-m", type=float, default=6.0)
    parser.add_argument("--trajectory-corridor-reserve-m", type=float, default=0.6)
    args = parser.parse_args()
    result = {
        "schema": SCHEMA,
        "status": "PASS_THREE_CITY_GEOMETRY_ASSETS_PREPARED",
        "source_root": str(args.source_root.resolve()),
        "output_root": str(args.output_root.resolve()),
        "cities": [
            build_city(
                city_id,
                city,
                args.source_root.resolve(),
                args.output_root.resolve(),
                float(args.road_sample_spacing_m),
                float(args.trajectory_corridor_reserve_m),
            )
            for city_id, city in CITIES.items()
        ],
    }
    reference = write_content_addressed_json(
        args.output_root.resolve(), "MULTICITY_ASSET_BUILD_RESULT", result
    )
    print(json.dumps(reference, indent=2))


if __name__ == "__main__":
    main()
