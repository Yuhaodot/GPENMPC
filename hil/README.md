# GPENMPC HIL

Source code, reference firmware and build instructions.

The experiment connects a Pixhawk 6C to an M600 vehicle model in RflySim.
A USB transmitter provides velocity and yaw-rate commands. The flight
controller shapes the reference, runs robust SE(3) tracking control and
allocates commands to the six rotors. MATLAB manages the session and plots
the response; RflySim3D displays the vehicle motion.

## Source layout

| Directory | Contents |
| --- | --- |
| `source/hil/rfly_vendor_integration` | Board runtime, controller interfaces, MAVLink exchange and firmware integration |
| `source/hil/px4_full_inner` | PX4 state and message adapters |
| `source/hil/host_runtime` | MATLAB communication and session services |
| `source/hil/tools` | Launchers, device setup, model interfaces and tests |
| `source/hil/m600_coptersim` | M600 dynamics model and simulator interfaces |
| `source/hil/evidence` and `source/hil/live` | Generated controller code, model libraries and runtime assets |
| `source/hil/matlab_validation` | MATLAB integration tests |
| `source/hil/rfly_vendor_integration/application_integration` | PX4 application overlays, board files and build dependencies |
| `assets/canonical` | Controller configuration, GP model and numerical-method sources |
| `source/hil/task_packages` | Delivery task data and reference trajectories |
| `firmware/fmuv6c` | FMUv6C firmware, parameters and build information |
| `environment` | Recorded MATLAB and RflySim versions |
| `licenses` | Component license notices |

## Software

| Component | Version |
| --- | --- |
| MATLAB and Simulink | R2026a Update 2 |
| RflySim | V4.12, 2026-03-26 distribution |
| PX4 base | v1.16.0, commit `6ea3539157ca358c70a515878b77077af7d4611d` |
| ARM GNU toolchain | GCC 10.3.1, 10.3-2021.10 |
| CMake | 3.28.3 |
| Ninja | 1.11.1 |
| Linux Python | 3.12.3 |
| Windows Python | 3.12.10 |
| SDL2 | 2.0.20 |
| Flight controller | Pixhawk 6C, FMUv6C, board ID 56 |
| Transmitter | FlySky FS-i6S, USB |

The Windows host uses MATLAB, UAV Toolbox, Instrument Control Toolbox and
RflySim. Controller and dynamics builds use the corresponding MATLAB code
generation tools. Firmware compilation uses the Linux ARM toolchain.

## Firmware

The GPENMPC application is provided in `firmware/fmuv6c/px4_fmu-v6c_default.px4`.
The `.px4` package is 1,855,252 bytes; the raw application
image is 1,962,048 bytes. The package SHA256 is:

```text
AF571397E67013D23D59EF2827B879B76A408271713891686790BD64661B4743
```

`firmware/fmuv6c/firmware.json` lists the target, compiler versions and artifact
hashes. The build also produces ELF and map files for debugging.

## Simulator setup

The firmware build uses the supplied controller sources and the development
tools listed in [Build](docs/build.md). The full CopterSim session additionally uses the project
model and SDK bridge described in [Simulator setup](docs/platform_inputs.md),
together with a licensed RflySim installation. These platform-specific files
are supplied separately under their applicable vendor permissions.

## Build and session setup

The firmware combines the PX4 base at the commit listed above, project
overlays, generated tracking kernel and board runtime. Follow [Build](docs/build.md)
to configure the PX4 source, ARM toolchain, MATLAB installation and output directory.

The host configuration supplies the MATLAB and simulator installations,
model libraries, board identity and content hashes. Configure these paths
and assets together when setting up another computer.

Use [Local configuration](docs/setup.md) to validate the recovery records and
set the MATLAB session configuration. The hardware setup uses USB power with
physical actuators disconnected.

`source/hil/OPEN_GPENMPC_HIL.cmd` opens the experiment dashboard.
**Start manual session** prepares the transmitter, model and flight-controller
link. **Live data** shows the response curves. **Reset simulation** returns
the session to its initial state. Repeated sessions reuse the installed
firmware. A new firmware image is installed when selecting a different
board application.

## Licenses

Original GPENMPC source and documentation are licensed under [MIT](LICENSE).
PX4, NuttX, MAVLink and other components retain their own licenses.
MATLAB/Simulink-generated code retains its Academic License notices; generated
code and combined binaries follow the applicable component and deployment terms.
See [Licensing and distribution](docs/licensing.md) and the
[component notices](licenses/README.md). MATLAB and RflySim are separate installations.
