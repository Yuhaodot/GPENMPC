# City mission planning

The planner selects delivery order, a route from a fixed directed candidate pool
and ground speed for numerical urban-delivery experiments. Flight-control
simulation is run separately through the repository's MATLAB entry point.

## Start here

Run these commands from the repository root with Python 3.10 or newer:

```text
python planning/run.py --check-inputs
python planning/run.py --list
python planning/run.py --mission MU_CAMBRIDGE_MA_01 --method P_ENERGY_WIND_PAYLOAD
```

`--check-inputs` checks the bundled input hashes and service-height accounting.
The third command solves one mission.
Results are written under `outputs/planning`, relative to this repository, unless
`--output-root` specifies another directory. Each plan is identified by its
content hash. The finite-pool planner uses the Python standard library.

The distance baseline is `--method P_DIST_FIXED`: it minimizes distance at a
fixed 10 m/s speed. The energy aware method chooses from a speed grid and
accounts for wind-forecast uncertainty, payload updates and a reference-based
power model.

## Bundled tasks

| Folder | Location | Mission identifiers |
| --- | --- | --- |
| `data/cities/cambridge_ma` | Cambridge, Massachusetts, USA | `MU_CAMBRIDGE_MA_01`, `MU_CAMBRIDGE_MA_02` |
| `data/cities/manhattan_ny` | Manhattan, New York, USA | `MU_MANHATTAN_NY_01`, `MU_MANHATTAN_NY_02` |
| `data/cities/seattle_wa` | Seattle, Washington, USA | `MU_SEATTLE_WA_01`, `MU_SEATTLE_WA_02` |

Each city has `map`, `missions` and `routes` directories. There are four delivery
points per mission. The building polygons and roads come from public geographic
data; depot and delivery locations are study-defined.

The service model accounts for a descent from the task's 10.0 m altitude to
1.004 m: 8.996 m each way, 9.996 s for each vertical leg plus 8 s service, giving
27.992 s per delivery and 111.968 s for four deliveries. The calculation is
implemented in `gpenmpc_wind/m600_power.py`. Service phases are represented by
time and modeled-energy updates between the integrated MATLAB flight legs.

## Source layout

- `run.py`: repository-relative, single-mission command-line entry.
- `run_multicity_planning_matrix.py`: delivery-order, route and speed selection;
  also runs the six-mission matrix with both planning methods.
- `gpenmpc_wind/planner.py`: route-cost evaluation and planning primitives.
- `gpenmpc_wind/m600_power.py`, `gpenmpc_wind/wind_field.py`: power and wind models.
- `multi_urban_common.py`: content-addressed JSON output helpers.
- `assets/m600_profile.json`: reference-based aircraft power profile.
- `build_multicity_assets.py`: building geometry, mission and route-pool generation.

To plan all six missions with both methods:

```text
python planning/run_multicity_planning_matrix.py --assets-root data/cities --planner-src planning --m600-profile planning/assets/m600_profile.json --output-root outputs/planning_matrix
```

## Rebuild city geometry

`build_multicity_assets.py` constructs maps from building polygons and road
centerlines, then generates missions and candidate route pools. Road centerlines
seed connectivity. Collision checks use building polygons and the aircraft
envelope.

Geometry generation additionally requires NumPy, PyArrow, pyproj and Shapely 2,
plus `overture_buildings.geoparquet` and `overture_segments.geoparquet` in each
city's subfolder (`cambridge_ma`, `manhattan_ny`, `seattle_wa`) below a source
directory. The bundled maps and route pools are ready for mission planning;
reconstruction uses these additional raw geographic inputs.

```text
python planning/build_multicity_assets.py --source-root /path/to/city_sources --output-root outputs/city_geometry
```

Save reconstructed geometry in a new output directory. To simulate a new plan,
prepare a MATLAB task file with its sampled reference and environment inputs.

## Data sources

Geographic-data credits and model references are listed in
[THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md).
