# Local configuration

The Windows session uses MATLAB, RflySim and the USB flight controller.
Firmware build paths are configured separately in [Build](build.md).
Run the commands below from the `hil/` directory.

## Configuration files

| Input | Location or setting | Use |
| --- | --- | --- |
| Board identity | [device.example.json](../config/device.example.json); set `GPENMPC_DEVICE_CONFIG` to the completed private copy | MAVLink UID, PX4 GUID and bootloader serial |
| Transmitter calibration | `source/hil/tools/usb_rc_runtime/fs_i6s_calibration.mat`, included | FS-i6S stick axes, centres and ranges |
| Parameter records | `CONTENT_MANIFEST.json`, `OUTER_RESULT.json`, `PLAN.json`, `SERIAL_PREFLIGHT.json`; set `HIL_RECOVERY_RECORDS` to their directory | Parameter setup and return to the recorded configuration |
| Hardware setup record | Local text file selected by `HIL_SETUP_RECORD` | Description of the USB bench connection and actuator isolation |
| Recovery application | `SERIAL_POSTFLIGHT.json` plus `GPENMPC_RECOVERY_ROOT/firmware/px4_fmu-v6c_default.px4` | Restore a selected board application |
| Simulator inputs | [Model and runtime files](platform_inputs.md) | CopterSim dynamics, transport and visualization |

The USB transmitter is a FlySky FS-i6S with vendor ID `284E` and product ID
`7FFF`. Its supplied calibration is checked against SHA256
`622F6A22DFE2E6B9E224F93D50DF7EE2B97AED53F0EA1D068B99672F44F682CA`.
The transmitter and flight controller connect to the host by separate USB
cables. The bench uses USB power with physical actuators disconnected.

The parameter snapshots and calibration belong to the reference bench.
Adapting another board or transmitter requires matching records and updating
the corresponding source bindings, followed by the relevant component checks
and firmware build. The configuration files carry these identities into a
session.

## Device identity

Copy [device.example.json](../config/device.example.json) to a private file,
then enter the verified MAVLink UID, PX4 GUID and bootloader serial as strings.
Set `GPENMPC_DEVICE_CONFIG` to that file. Alternatively, create `local/device.json`
under `hil/`; `local/` is ignored by Git. Fill in the template with the connected
board's identifiers before starting a hardware session.

The UID is checked as an exact unsigned 64-bit decimal integer. The GUID and
bootloader serial must identify the same board. Runtime comparisons reject a
different device. Keep this file and hardware records outside material being
shared. Building for another board also requires the corresponding firmware
device binding described in [Build](build.md).

## Parameter records

The recovery configuration consists of `CONTENT_MANIFEST.json`,
`OUTER_RESULT.json`, `PLAN.json` and `SERIAL_PREFLIGHT.json`.
The loader checks the files against the recorded SHA256 values before using
the parameter definitions. The plan can be stored alongside the other three
files when moving the configuration to another directory.

To copy an existing record set:

```powershell
python source/hil/tools/export_recovery_data.py --records "E:/HIL records/session" --plan "E:/HIL records/PLAN.json" --destination "E:/HIL configuration/recovery"
```

From `hil/` in MATLAB:

```matlab
config = configure_hil("E:/HIL configuration/recovery", "E:/HIL configuration/setup.txt", "E:/HIL configuration/device.json");
```

`configure_hil` validates the recovery data and sets `HIL_RECOVERY_RECORDS`
and `HIL_SETUP_RECORD` for the current MATLAB process. The setup record
describes the connected hardware and is checked again by the session entry.
Omitted setup-record and device-file arguments retain their environment settings.

Application restoration additionally requires `SERIAL_POSTFLIGHT.json` and the
matching recovery application. Set `GPENMPC_RECOVERY_ROOT` to an archive containing
`firmware/px4_fmu-v6c_default.px4`; use `HIL_RECOVERY_RECORDS` for the recorded JSON
files. Their fixed hashes, byte counts, board identity and parameter comparisons
are checked before live upload. These hardware-specific records are supplied
separately. For a different board, provide its matching device file, parameter
records and recovery application, and update the reference bindings as described
above.

## Host installation

The session launchers use the local MATLAB, RflySim/PX4PSP, SDL2, model-library
installations. Canonical assets are loaded from this distribution and checked
against its fixed hashes and source manifest. Configure the installation paths
before starting a hardware session:

- `GPENMPC_MATLAB_ROOT`: MATLAB installation directory. The launcher otherwise
  resolves `matlab.exe` from `PATH`; child MATLAB sessions use the current installation.
- `GPENMPC_LLVM_ROOT`: LLVM MinGW installation used by the Windows C/C++ build
  tools. `GPENMPC_GCC_ROOT` selects a MinGW GCC installation for the GCC-based
  tools. Both can use MATLAB's `MW_MINGW64_LOC` setting when the corresponding
  project variable is unset. Each tool checks its required compiler executable.
- `GPENMPC_RFLY_ROOT`: RflySim/PX4PSP installation directory containing
  `RflySim3D/RflySim3D.exe`, `QGroundControl/SDL2.dll` and
  `Firmware/Tools/px_uploader.py`, together with the bundled
  `Python38/python.exe` and `CopterSim/external/map` assets.
- `GPENMPC_COPTERSIM_RUNTIME`: a simulator runtime directory
  containing `CopterSimNoUI.exe` and its dependencies, plus
  `external/model/GPENMPC_M600_Diagnostic.dll`. Use the matching project model described in
  [Simulator setup](platform_inputs.md). The executable and
  model hashes are checked by the session entry. Preparation copies this runtime
  into its managed session directory. When this variable is unset, the launcher
  checks its repository-relative runtime location.

The shipped firmware targets Pixhawk 6C; the device identity, USB calibration
and parameter records belong to the corresponding hardware setup.
The supplied firmware is device-bound. Rebuild it with your verified UID for a
different flight controller.

Session records default to `logs` under `hil/`. Set
`GPENMPC_HIL_LOG_ROOT` to select another recording directory. The optional
`GPENMPC_REPLAY_FILE` setting selects a saved trace for the replay display.

Analysis and development tools use `GPENMPC_EXTERNAL_PATHS`, the path to a JSON
object mapping dependency names to local paths. Missing entries report the
required dependency. Copy [external_paths.example.json](../config/external_paths.example.json)
to a private configuration file and fill in the entries used by the selected
tool. The template covers plan-builder records, recovery inputs, model files
and test fixtures. `recovery_application_upload_receipt` selects the upload
record used to verify a recovery application. Keep this configuration outside
the source repository.

Task generation uses `public_hil_method_contract` and
`native_visual_host_runtime`. Session tests use `unregistered_session_fixture`
and `retired_session_fixture` for the corresponding input cases.

Component checks use `native_position_parameter_receipt` for position-controller
parameters, `sensor_mount_reference_observation` for sensor orientation,
`utc_pair_validation_receipt` for paired timestamps, and
`canonical_actuator_encoding_fixture` for actuator encoding inputs.
`same_io_processing_profile_fixture` supplies the I/O processing profile used
by timing comparisons.

## Offline checks

From `hil/` in MATLAB:

```matlab
test_hil
```

The injected uploader test uses a synthetic device identity and the distributed
firmware, without opening a serial port:

```powershell
python source/hil/tools/test_gpenmpc_application_upload_once.py
```

Add `--with-recovery` to include an explicitly configured recovery image.
