# Licensing and distribution

## Original source

The [MIT license](../LICENSE) covers original GPENMPC algorithms, scripts,
adapters, tests and documentation. Third-party and generated files retain their
own licenses. Combined firmware, model libraries and MEX binaries are subject
to the terms of their constituent components. See the [component list](../licenses/README.md).

## MATLAB and Simulink material

MATLAB and Simulink material follows the agreement supplied with the producing
installation and the [Program Offering Guide](https://www.mathworks.com/help/pdf_doc/offering/offering.pdf).
The R2026a agreement includes the March 2026 guide; Part Two covers user-created,
derivative and generated forms, including Coder application distribution.

Generated code under `source/hil/evidence`, `source/hil/live` and
`source/hil/matlab_validation/reference_kernel_sources` and `mex_kernel`
carries Academic License notices. Preserve those notices and follow the
applicable academic and generated-form terms for the code, firmware, controller
archives, model DLLs and MEX files. Commercial deployment requires an appropriate
license offering and a build produced under it.

Builds use SDK headers such as `mex.h` and `tmwtypes.h` from the recipient's
licensed MATLAB installation.

MATLAB(r). (c) 1984-2026 The MathWorks, Inc.

## Binary attachments

The FMUv6C images combine project code, PX4, NuttX, compiler runtime and generated
controller code. Keep the component notices and exact artifact hashes with
each attachment. The supplied image is bound to the reference bench's board
identity. A different controller requires the explicit UID configuration in
[Build](build.md).

MathWorks components retain their applicable agreement. No MATLAB product
license or development/deployment right is transferred by this package.
MathWorks and its licensors provide no warranty for their components and are
excluded from liability for damages or remedies. Preserve all embedded and
accompanying copyright, trademark, proprietary, disclaimer and warning notices.
The agreement's Application License requirements apply where applicable.

## Firmware build inputs

The firmware uses the following inputs under their respective licenses:

- `source/hil/rfly_vendor_integration` and `source/hil/px4_full_inner`;
- the top-level C/H files in `source/hil/evidence/tracking_controller/generated`
  and `source/hil/evidence/phase_controller/generated`;
- `source/hil/live/vertical_observer/gpenmpcObserveCausalVerticalDisturbance.c`;
- the license notices, canonical assets and build documentation.

Follow [Build](build.md) with a compatible PX4 tree, ARM toolchain and licensed
MATLAB headers to build the board application.

## Simulator components

Obtain RflySim from its distributor. The following project model and interface
files use vendor material and require the applicable vendor permission for
redistribution. They are supplied separately for licensed simulator setups.

| Path, relative to `source/hil` | Provenance |
| --- | --- |
| `m600_coptersim/MavLinkStruct.mat` | Byte-identical RflySim model-interface data |
| `m600_coptersim/model_source/GPENMPC_M600_Canonical.slx` | Project model derived from the simulator template |
| `m600_coptersim/model/generated/GPENMPC_M600_Canonical.slx` | Derived model used for generation |
| `m600_coptersim/model/wrapper/` | Simulator-interface wrapper with project extensions |
| `m600_coptersim/model/GPENMPC_M600_Diagnostic.dll` | Binary built from the generated model and wrapper |
| `m600_coptersim/model/generated/GPENMPC_M600_Canonical_ert_rtw/` | C++ output derived from that simulator model |
| `tools/gpenmpc_rfly_udp_sdk_bound.hpp` | Three methods copied from the installed RflySim UDP SDK source listing |
| `tools/usb_rc_runtime/gpenmpc_rfly_udp_transport_mex.mexw64` | Transport binary incorporating those SDK methods |

Model sharing and review follow the relevant institutional and vendor license
arrangements. See [Simulator setup](platform_inputs.md) for the required files.
