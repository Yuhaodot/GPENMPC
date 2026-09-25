# Gaussian Process Enhanced Energy Aware Nonlinear Model Predictive Control

GPENMPC combines mission planning that accounts for payload, Gaussian process
(GP) model correction, energy aware nonlinear model predictive control
(eNMPC), and robust SE(3) tracking for urban drone delivery with a six rotor
aircraft.

Developed for the MathWorks Challenge Project
[Energy-Optimal Trajectory Planning for Multirotor Drones](https://github.com/mathworks/MATLAB-Simulink-Challenge-Project-Hub/tree/main/projects/Energy-Optimal%20Trajectory%20Planning%20for%20Multirotor%20Drones).

## Results

The numerical study covers 96 cases across Cambridge, Seattle and Manhattan:
72 controller comparisons, 12 payload cases and 12 event-replanning cases.
All tasks reached completion; 70 satisfied the full set of performance criteria.

![Modeled mission energy and position tracking across six controllers](docs/images/controller-comparison.png)

Each controller is evaluated on the same six tasks with two planning methods.
Compared with PID, GP/eNMPC reduces modeled mission energy by 3.10% and position
RMSE by 65.23%, averaged over the 12 paired configurations. Compared with eNMPC,
it reduces position RMSE by 24.89% with 1.34% higher modeled mission energy.

The [numerical results](results/README.md) include three-axis tracking,
trajectory, power and cumulative energy plots, together with the full tables.
The [hardware in the loop implementation](hil/README.md) provides the controller
source, Pixhawk firmware, software requirements and session instructions.

## Run

Open this folder in MATLAB and run:

```matlab
main
```

The default example runs the Cambridge 01 delivery task using the energy aware
plan and GP/eNMPC controller, then displays the trajectory and response plots.
The task inputs and pre-trained GP are included. Results and a PNG figure are
saved in a new, uniquely named folder under `outputs`.

For a quick, three-second flight segment:

```matlab
main(MaximumInnerSamples=300)
```

To select a different task:

```matlab
[trace, result, folder] = main("seattle_01", "energy");
main("manhattan_02", "distance", Method="robust_se3");
```

To reopen a saved run:

```matlab
view_result(fullfile(folder, "simulation.mat"));
```

### Requirements

| Component | Environment |
| --- | --- |
| Numerical simulation and plots | MATLAB; tested with R2026a Update 2 |
| Mission planning | Python 3.10+, standard library |
| Editable system diagram | MATLAB and Simulink |
| Project overview page | Web browser |
| GP training | MATLAB and Deep Learning Toolbox, with training inputs |
| City geometry reconstruction | Python, NumPy, PyArrow, pyproj and Shapely 2 |

## Method

The mission planner jointly selects delivery order, candidate routes and nominal
speed. Its energy model accounts for changing payload and wind. Flight control
then follows the sampled trajectory reference:

- **GP model correction:** a sparse GP predicts three acceleration residuals
  from 17 causal features. Prediction uncertainty and consistency determine the
  weight of the model correction and bounded compensation.
- **eNMPC:** optimizes progress along the route and an acceleration correction,
  balancing modeled energy, tracking accuracy and control variation.
- **Robust SE(3) tracking:** computes force and moment commands from the motion
  reference and aircraft state. Control allocation distributes these commands
  across six rotors.

The plant includes six-degree-of-freedom motion, rotor response, wind, and
structured model mismatch. The main controller uses a 0.01 s inner step,
0.30 s outer update, and eight prediction steps of 0.20 s. Further model and
service details are in [docs/method.md](docs/method.md).

## Tasks and controllers

| Task names | Location |
| --- | --- |
| `cambridge_01`, `cambridge_02` | Cambridge, Massachusetts, USA |
| `seattle_01`, `seattle_02` | Seattle, Washington, USA |
| `manhattan_01`, `manhattan_02` | Manhattan, New York, USA |

Each task has four delivery points. The `energy` plan accounts for energy,
wind and payload; the `distance` plan uses distance-based routing.

| `Method` | Controller |
| --- | --- |
| `gp_enmpc` (default) | GP enhanced eNMPC with robust SE(3) tracking |
| `enmpc` | eNMPC with robust SE(3) tracking |
| `gp_robust_se3` | GP enhanced robust geometric tracking |
| `robust_se3` | Robust geometric tracking |
| `nominal_se3` | Nominal geometric tracking |
| `pid` | Cascaded PID |

## Mission planning

Run from the repository root:

```text
python planning/run.py --list
python planning/run.py --mission MU_CAMBRIDGE_MA_01 --method P_ENERGY_WIND_PAYLOAD
```

Plans are saved under `outputs/planning`. The [planning guide](planning/README.md)
describes the distance baseline, six-mission batch and geographic inputs.

## Project overview and hardware in the loop

Open [docs/project-overview.html](docs/project-overview.html) in a browser for an
interactive overview of the delivery task, planning, control, disturbances
and hardware in the loop setup.
Use the tabs or arrow keys to change pages and **Present** for full screen.

The [Simulink system diagram](models/README.md) documents the manual HIL setup:
USB transmitter, MATLAB pilot-command processing, Pixhawk tracking control,
CopterSim dynamics and RflySim3D visualization.

The [hardware guide](docs/hardware.md) describes the experiment software,
firmware target, compilation and installation process, and session controls.
The [HIL directory](hil/README.md) contains the Pixhawk 6C controller source,
reference firmware, MATLAB host interface, configuration templates and offline
tests. See [Firmware build](hil/docs/build.md), [Session setup](hil/docs/setup.md)
and [Simulator setup](hil/docs/platform_inputs.md) for the development tools,
board configuration, CopterSim model and SDK inputs.

```matlab
open_system(fullfile('models', 'Hexarotor_HIL.slx'))
```

## Tests

```matlab
setup_project
addpath("tests")
test_release
test_main
```

`test_release` checks twelve task files and runs short cases for all six
controllers. `test_main` checks the simulation-to-figure workflow. Successful
runs return test reports; simulation outputs remain under `outputs`.

```text
python planning/run.py --check-inputs
```

The planning check reports six missions, four deliveries per mission, and
`inputs_valid: true`.

See [Sparse GP training](docs/training.md) for raw-log inputs, inducing-point
construction and retraining.

## Source guide

- `main.m`: simulation and figure entry point.
- `run_case.m`, `view_result.m`: simulation execution and plotting functions.
- `matlab/core/enmpc`: prediction and command optimization.
- `matlab/core/inference`, `matlab/core/training`: GP inference and training.
- `matlab/core/dynamics`: tracking, allocation, plant and disturbance models.
- `matlab/extensions`: comparison controllers.
- `config`: controller and model settings.
- `data/tasks`, `data/gp`: task inputs and pre-trained model.
- `data/cities`, `planning`: maps, delivery tasks and route planning.
- `models`, `docs`: system diagram and project presentation.
- `results`: numerical study figures, tables and plotting code.
- `tests`: executable numerical tests.

## Acknowledgments and license

Thanks to Giordano Scarciotti, Martina Sciola and Roberto Valenti for the
opportunity to undertake this project and for their feedback and discussions.

Project code is distributed under the [MIT License](LICENSE). Geographic data
and referenced model sources retain the terms listed in
[Third-party notices](planning/THIRD_PARTY_NOTICES.md).
