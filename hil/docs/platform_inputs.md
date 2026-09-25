# Simulator setup

The CopterSim host build requires Windows, Python 3, LLVM MinGW, MATLAB/Simulink
R2026a and a licensed RflySim installation. Firmware compilation is described
separately in [Build](build.md).

## Required model and SDK files

The HIL session uses the matching project model, generated C++ sources,
wrapper and UDP SDK bridge listed below, supplied separately under the
applicable component permissions. See [Licensing](licensing.md) for the terms.

Place the files at these destinations, relative to `source/hil`:

| Files | Purpose | Destination |
| --- | --- | --- |
| `MavLinkStruct.mat` from the installed RflySim distribution | Simulator message and bus definitions | `m600_coptersim/MavLinkStruct.mat` |
| Project `GPENMPC_M600_Canonical.slx` (same file in both locations) | Simulink M600 dynamics and sensor model | `m600_coptersim/model_source/GPENMPC_M600_Canonical.slx` and `m600_coptersim/model/generated/GPENMPC_M600_Canonical.slx` |
| Generated `GPENMPC_M600_Canonical.cpp`, `GPENMPC_M600_Canonical.h`, `multiword_types.h`, `rtwtypes.h` | Model implementation and data types used by the DLL build | `m600_coptersim/model/generated/GPENMPC_M600_Canonical_ert_rtw/` |
| Project `modeldllgen.cpp`, `modeldllgen.h`, `rfly_export_compat.def` | CopterSim DLL entry points and model interface | `m600_coptersim/model/wrapper/` |
| Project `gpenmpc_rfly_udp_sdk_bound.hpp` | RflySim UDP transport interface | `tools/gpenmpc_rfly_udp_sdk_bound.hpp` |

The vendor MAT file is located at
`RflySimAPIs/4.RflySimModel/3.CustExps/e0_AdvApiExps/15.inFromUE/MavLinkStruct.mat`
under the RflySim installation. Expected input SHA256 values are:

| Input | SHA256 |
| --- | --- |
| `MavLinkStruct.mat` | `4824B7599E51D7C2BEF6215FEB5E26A64BAA54FF5ED24991EA7A69E4D605029D` |
| Project `.slx` model | `9E366CBE116A92FA3EF6A53B4C8D984A1B15078BF6117EAF3BA8C3A12BD144AA` |
| UDP SDK bridge | `5AF8E6EDD6AD943D3CCD8F1F2734542508369F9146260D5E73950B0E56B8EB47` |

The project model, generated sources and wrapper form one matched set. The
[M600 display model](../source/hil/tools/GPENMPC_M600_Display.slx)
included in the repository presents the system diagram; CopterSim loads the
compiled dynamics model described here.

## Runtime files

| File | Location | Purpose |
| --- | --- | --- |
| `CopterSimNoUI.exe` and its runtime dependencies | `GPENMPC_COPTERSIM_RUNTIME` | Numerical aircraft and simulated sensor execution |
| `GPENMPC_M600_Diagnostic.dll` | `external/model/` in that runtime, and `source/hil/m600_coptersim/model/` | Project M600 dynamics library |
| `gpenmpc_rfly_udp_transport_mex.mexw64` | `source/hil/tools/usb_rc_runtime/` | MATLAB transport interface |
| `3DDisplay.txt`, `3DDisplay.png` | `CopterSim/external/map/` in `GPENMPC_RFLY_ROOT` | Matching terrain calibration and heightfield |
| `RflySim3D/RflySim3D.exe` | `GPENMPC_RFLY_ROOT` | Vehicle and scene visualization |

The reference session expects the following runtime identities:

| File | SHA256 |
| --- | --- |
| `CopterSimNoUI.exe` | `94B81EFB44058176DD5353669D9C28FC5331CC8411AB9EA3F2D27C1E8C343241` |
| `GPENMPC_M600_Diagnostic.dll` | `D536EACE85EBA30A6CE07EF5B38FF108B132C9E3B230A3C46E93F3E1C9CA6E12` |
| `gpenmpc_rfly_udp_transport_mex.mexw64` | `6D207DE4797594C88AA9502C33C800503F7CC1C50806CC70C85D1C76E25AFFC0` |

The board identity, transmitter calibration and parameter records are listed
in [Session configuration](setup.md#configuration-files). Obtain RflySim
executables, maps and SDK components through the simulator distribution;
project model files follow the component terms in [Licensing](licensing.md).

## Build

Run both commands in PowerShell from the **`hil/` directory**, using your
installed compiler and MATLAB paths. The UDP output directory must not exist.

```powershell
python source/hil/tools/build_m600_model.py --compiler "D:/download/software/llvm-mingw/bin/clang++.exe" --matlab-root "C:/Program Files/MATLAB/R2026a"
powershell -NoProfile -File source/hil/tools/build_udp_transport.ps1 -CompilerBin "D:/download/software/llvm-mingw/bin" -MatlabRoot "C:/Program Files/MATLAB/R2026a" -OutputDirectory "D:/download/cache/gpenmpc-udp-build"
```

The first command compiles the supplied generated sources and wrapper into
`source/hil/m600_coptersim/model/GPENMPC_M600_Diagnostic.dll`. Place that DLL in
`external/model/` of the configured CopterSim runtime.

The second command verifies the SDK bridge and builds
`gpenmpc_rfly_udp_transport_mex.mexw64` in the selected output directory. Its
runtime destination is `source/hil/tools/usb_rc_runtime/`.

Configure the simulator paths in [Local configuration](setup.md). The session
configuration must contain matching model and transport binary hashes before
using rebuilt artifacts with hardware.
