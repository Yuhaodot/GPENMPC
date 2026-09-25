# Hardware in the loop system diagram

`Hexarotor_HIL.slx` is an editable Simulink system diagram showing the signal
flow of the manual-flight demonstration. Use it to explore the system
architecture and adapt the drawing for presentations.

From the repository root, open it in MATLAB with Simulink installed:

```matlab
open_system(fullfile('models', 'Hexarotor_HIL.slx'))
```

To edit its layout, unlock the library:

```matlab
set_param('Hexarotor_HIL', 'Lock', 'off')
```

## Signal flow

1. A USB transmitter supplies the pilot's stick inputs to MATLAB.
2. MATLAB converts those inputs to velocity and yaw-rate commands.
3. The Pixhawk shapes the motion reference and runs robust SE(3) tracking
   and six-rotor control allocation.
4. CopterSim advances the simulated M600 dynamics and returns simulated sensor
   data to the flight controller.
5. RflySim3D displays the vehicle motion; the monitoring panel plots the live
   responses.

The manual HIL setup exercises the tracking controller. The complete method
combining a Gaussian process (GP) with eNMPC is run through `run_case.m`
in the numerical simulation code.

## Related source

- `matlab/core/dynamics/gpenmpcRobustSe3Control.m`: tracking force and moment.
- `matlab/core/dynamics/gpenmpcRobustSe3Augmentation.m`: robust augmentation.
- `matlab/core/dynamics/gpenmpcM600Allocation.m`: six-rotor allocation.
- `matlab/core/dynamics/gpenmpcM600SixDofPlantDerivative.m`: numerical plant.

See the [hardware guide](../docs/hardware.md) for the recorded software versions,
firmware artifacts, and compilation and installation workflow.
