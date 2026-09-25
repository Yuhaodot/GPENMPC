# Numerical study results

The tables summarize the 96-case numerical study: 72 controller comparisons,
12 payload cases and 12 event-replanning cases.

| File | Contents |
| --- | --- |
| `case_metrics.csv` | Per-case completion, performance criteria and metrics |
| `method_summary.csv` | Summary by controller and experiment group |
| `controller_comparisons.csv`, `controller_pairs.csv` | Paired controller comparisons |
| `payload_comparisons.csv`, `payload_pairs.csv` | Dynamic-payload experiments |
| `replanning_comparisons.csv`, `replanning_pairs.csv` | Event-replanning experiments |

All 96 tasks reached completion; 70 satisfied every performance criterion.
Metrics include modeled energy,
flight and service duration, tracking error, acceleration, jerk, compensation,
rotor-command variation and computation time.

The experiment identifiers and artifact references associate rows with their
source runs. New results can be generated with `main` and viewed with
`view_result`. A saved run contains the complete trace in `simulation.mat`.
