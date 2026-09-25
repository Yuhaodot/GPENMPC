# Hardware in the loop

The manual-flight experiment runs the robust SE(3) tracking controller and
six-rotor allocation on a Pixhawk 6C. A USB transmitter supplies the motion
reference. The computer advances the M600 vehicle model, displays its motion
in RflySim3D, and plots the live responses.

## Signal flow

| Component | Function |
| --- | --- |
| FlySky FS-i6S connected by USB | Pilot input |
| MATLAB host program | Stick calibration, velocity and yaw-rate commands, session controls and live plots |
| Pixhawk 6C | Reference shaping, robust SE(3) tracking and six-rotor allocation |
| CopterSim | M600 dynamics and simulated sensor exchange with the flight controller |
| RflySim3D | Vehicle and scene visualization |

The computer sends the simulated motion and sensor data back to the flight
controller, closing the loop. RflySim provides the dynamics-to-hardware
interface and three-dimensional display used in this experiment.

The [editable system diagram](../models/Hexarotor_HIL.slx) shows these
connections. The [numerical examples](../README.md#run) evaluate the complete
GP and eNMPC method on the urban delivery tasks.

## Experimental environment

| Software or hardware | Recorded version or target | Purpose |
| --- | --- | --- |
| MATLAB | R2026a Update 2, 26.1.0.3251617 | Host program, model development and monitoring |
| Simulink | R2026a | M600 model and system diagram |
| MATLAB Coder and Simulink code generation tools | R2026a | Controller C code and model build |
| UAV Toolbox | MATLAB installation | MAVLink interfaces |
| Instrument Control Toolbox | MATLAB installation | Host communication interfaces |
| RflySim | V4.12, 2026-03-26 installation | CopterSim, RflySim3D and simulator tools |
| Windows Python | 3.12.10 | Firmware upload support |
| SDL2 | 2.0.20 | USB transmitter input |
| PX4 | v1.16.0 with project integration | Flight-controller application |
| ARM GNU toolchain | GCC 10.3.1, 10.3-2021.10 package | FMUv6C firmware compilation |
| CMake | 3.28.3 | Firmware build configuration |
| Ninja | CMake build backend | Firmware build execution |
| Linux Python | 3.12.3 | Build scripts |
| Pixhawk 6C | FMUv6C, board ID 56 | Physical flight controller |
| FlySky FS-i6S | USB joystick connection | Manual-flight input |

The host uses Windows and the firmware build uses a Linux toolchain. The MEX
interfaces are compiled for Windows x64. MATLAB toolboxes and RflySim are
installed through their respective distributions.

Official resources: [RflySim](https://rflysim.com/doc/en/1.Start/overview.html),
[PX4 development](https://docs.px4.io/v1.16/en/dev_setup/building_px4), and
[MathWorks products](https://www.mathworks.com/products.html).

## Firmware

The GPENMPC application targets `px4_fmu-v6c_default` and uses PX4 base commit
`6ea3539157ca358c70a515878b77077af7d4611d`.

| Artifact | Purpose |
| --- | --- |
| `px4_fmu-v6c_default.px4` | Packaged application for the PX4 uploader |
| `px4_fmu-v6c_default.bin` | Raw application image |
| `px4_fmu-v6c_default.elf` | Linked application and debugging symbols |
| `px4_fmu-v6c_default.map` | Linker map |
| `parameters.json` and `parameters.xml` | Firmware parameter definitions |
| `airframes.xml` | Airframe metadata |

The accompanying HIL package provides the GPENMPC application. Its
`px4_fmu-v6c_default.px4` package is 1,855,252 bytes, with SHA256:

```text
AF571397E67013D23D59EF2827B879B76A408271713891686790BD64661B4743
```

The accompanying `firmware.json` records its target, compiler versions and
artifact hashes. The firmware combines the generated tracking kernel, project
C++ interfaces, MAVLink integration and FMUv6C configuration with the PX4 base
listed above.

The reference image uses the experiment board's device identity. Building for
another board requires its matching UID and host configuration. The build also
produces ELF and map files for debugging.

## Compilation and installation

1. Prepare the recorded PX4 base, project integration sources, generated
   controller kernel and ARM toolchain.
2. Configure the FMUv6C application, including the project runtime and
   controller library.
3. Build the `px4` target, then the `px4_package` target. These produce the
   linked application and the uploadable `.px4` package.
4. Connect the matching flight-controller board by USB. The experiment launcher
   checks the device and simulator setup before invoking the PX4 uploader.
5. Verify the uploaded image and start the HIL session with the corresponding
   host program and dynamics model.

Compilation is needed when the firmware source changes. Installation is needed
when selecting a different firmware image. Repeated sessions reuse the
installed application; stopping or resetting a session resets its runtime
state.

The hardware setup uses USB power with physical actuators disconnected. The
firmware is configured for the simulated vehicle and its HIL communication
interfaces.

## Session controls

The experiment dashboard provides **Start manual session**, **Live data**, and
**Reset simulation**. Opening the dashboard displays the interface. Starting a
session prepares the transmitter, dynamics model and flight-controller link,
then enables manual control. The plots show height, velocity, attitude, six
virtual rotor inputs, disturbance and angular-rate response.

## Software attribution

PX4 is distributed under its BSD 3-Clause license. MATLAB-generated controller
files retain the license notices produced by the installed MathWorks tools.
RflySim and the MathWorks products use their respective software licenses.
