# Simulator setup

The CopterSim host build requires Windows, Python 3, LLVM MinGW, MATLAB/Simulink
R2026a and a licensed RflySim installation. Firmware compilation is described
separately in [Build](build.md).

## Required model and SDK files

The HIL session uses the matching project model, generated C++ sources,
wrapper and UDP SDK bridge listed below, supplied separately under the
applicable component permissions. See [Licensing](licensing.md) for the terms.

Place the files at these destinations, relative to `source/hil`:

| Files | Destination |
| --- | --- |
| `MavLinkStruct.mat` from the installed RflySim distribution | `m600_coptersim/MavLinkStruct.mat` |
| Project `GPENMPC_M600_Canonical.slx` (same file in both locations) | `m600_coptersim/model_source/GPENMPC_M600_Canonical.slx` and `m600_coptersim/model/generated/GPENMPC_M600_Canonical.slx` |
| Generated `GPENMPC_M600_Canonical.cpp`, `GPENMPC_M600_Canonical.h`, `multiword_types.h`, `rtwtypes.h` | `m600_coptersim/model/generated/GPENMPC_M600_Canonical_ert_rtw/` |
| Project `modeldllgen.cpp`, `modeldllgen.h`, `rfly_export_compat.def` | `m600_coptersim/model/wrapper/` |
| Project `gpenmpc_rfly_udp_sdk_bound.hpp` | `tools/gpenmpc_rfly_udp_sdk_bound.hpp` |

The vendor MAT file is located at
`RflySimAPIs/4.RflySimModel/3.CustExps/e0_AdvApiExps/15.inFromUE/MavLinkStruct.mat`
under the RflySim installation. Expected input SHA256 values are:

| Input | SHA256 |
| --- | --- |
| `MavLinkStruct.mat` | `4824B7599E51D7C2BEF6215FEB5E26A64BAA54FF5ED24991EA7A69E4D605029D` |
| Project `.slx` model | `9E366CBE116A92FA3EF6A53B4C8D984A1B15078BF6117EAF3BA8C3A12BD144AA` |
| UDP SDK bridge | `5AF8E6EDD6AD943D3CCD8F1F2734542508369F9146260D5E73950B0E56B8EB47` |

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
