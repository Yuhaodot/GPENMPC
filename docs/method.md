# Model and simulation

## Planning and flight control

The finite-candidate mission planner selects delivery order, routes and ground
speed using modeled energy, wind and remaining payload. Its geographic inputs
cover Cambridge, Massachusetts; Seattle, Washington; and Manhattan, New York.
Building footprints and roads come from public geographic data. Depot and
delivery locations are defined for the numerical study.

The twelve MATLAB task files provide trajectory samples, mission plans,
initial conditions and environment inputs. Each combines one of six missions
with either energy aware or distance based planning. `main` and `run_case`
simulate a selected task with any of the six controller methods.

The Gaussian process (GP) uses 17 causal features, an ARD-RBF kernel and 256 inducing points to
estimate acceleration-model residuals. GP/eNMPC coordinates the predicted
model correction and bounded physical compensation. The outer loop optimizes
route progress and acceleration correction; robust SE(3) tracking and
six-rotor allocation execute the resulting motion reference.

The main configuration uses a 0.01 s inner step, a 0.30 s outer update,
eight prediction steps of 0.20 s and a 0.28 s solver deadline. Runtime settings
are assembled from `config` by the MATLAB configuration functions.

## Dynamics and disturbances

The numerical plant integrates translation, attitude and rotor response.
Disturbance models include wind and acceleration residuals associated with
speed, payload, turning motion, drag and rotor effectiveness. The first task
in each city uses steady background wind; the second uses a smooth change in
the horizontal wind. These settings are stored in the task inputs.

The M600 reference profile combines published platform values, a phase-average
power table and modeling assumptions for the dynamics. Inertia and rotor
response parameters are model priors. Battery state of charge is calculated
from accumulated modeled energy and a 600 Wh reference capacity. Sources are
listed in [Third-party notices](../planning/THIRD_PARTY_NOTICES.md).

## Delivery service

Cruise references use an altitude of 10.0 m in the task coordinate frame.
Delivery service uses a target altitude of 1.004 m, 8.996 m vertical travel
each way, 9.996 s for each vertical leg, and 8 s at the delivery point.
This gives 27.992 s per delivery and 111.968 s for four deliveries.

Service is modeled between flight legs as an elapsed-time and energy update,
followed by a payload change and a node-hover state reset. The flight traces
contain a time gap for each service interval. This representation is shared
by the mission-energy calculation and the numerical task execution.

## Outputs

Each run creates a uniquely named directory under `outputs` containing
`simulation.mat` with `trace`, `result`, `runInfo` and `configuration`.
`main` also exports a response figure. The trace stores state and control
histories; the result stores task completion and performance metrics.

`MaximumInnerSamples` selects a shorter initial segment for testing. The
default value, `Inf`, runs the complete selected task. To choose an output
location, use the `OutputRoot` option of `main` or `run_case`.

The study tables cover controller comparisons, dynamic payload and event
replanning. The interactive entry point selects among the six main
controller methods; the extension results are provided in `results`.
