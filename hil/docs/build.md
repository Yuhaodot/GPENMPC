# Building GPENMPC firmware

The firmware targets FMUv6C (Pixhawk 6C). The supplied application and its
artifact hashes are in [firmware/fmuv6c](../firmware/fmuv6c/firmware.json).

## Inputs

Run the configuration entry with Linux Python, including under WSL. Provide:

- The RflySim-integrated PX4 source tree at commit
  6ea3539157ca358c70a515878b77077af7d4611d, with its existing submodules.
  The source tree includes the project's SE(3) controller and trajectory
  execution parameter definitions. The configuration entry locates the paired
  SE(3) controller and trajectory modules and maps their common prefix to
  the GPENMPC namespace in its isolated copy.
- GNU Arm Embedded 10.3-2021.10 (GCC 10.3.1), CMake 3.28.3, Ninja 1.11.1,
  Git, Make and Python 3.12. The Python environment requires the PX4 build
  dependencies, including kconfiglib, Jinja2, EmPy and PyYAML.
- A licensed MATLAB installation containing extern/include/tmwtypes.h.
- This source package, including the selected FMUv6C configuration, generated
  controller sources and local Micro-CDR/XRCE sources.

The configuration entry checks the PX4 commit, compiler version, required
files and the complete controller source set. It uses the selected local inputs.

## Configure

Set the four paths and the expected board UID for the local machine. Keep the output directory separate
from the package, PX4 tree, MATLAB installation and toolchain.

    PACKAGE=/absolute/path/to/GPENMPC/hil
    PX4_DIR=/absolute/path/to/integrated-px4
    ARM_DIR=/absolute/path/to/gcc-arm-none-eabi-10.3-2021.10
    MATLAB_DIR=/absolute/path/to/MATLAB
    BUILD_DIR=/absolute/path/to/gpenmpc-build
    read -r -p "Expected board UID (decimal or 0x hexadecimal): " BOARD_UID

    python3 "$PACKAGE/source/hil/rfly_vendor_integration/application_integration/configure_clean_application.py" \
      --px4-root "$PX4_DIR" \
      --arm-toolchain "$ARM_DIR" \
      --matlab-root "$MATLAB_DIR" \
      --output-root "$BUILD_DIR" \
      --board-uid "$BOARD_UID"

Use the UID recorded for the intended flight controller in the local host
configuration. The firmware requires this exact nonzero 64-bit identity during
session registration. Changing the board requires a matching rebuild and host
configuration. Keep device-specific configuration outside the source repository.

The --check-only option validates inputs without preparing an output directory.
The --prepare-only option creates isolated inputs and records the CMake command
without running CMake. The --configure-timeout option sets the CMake time limit
in seconds (default: 900).

The output contains px4/ with the selected PX4 source and local extensions,
integration/ with writable NuttX, board and dependency copies, and build/
for CMake products. First-party extension names are mapped to GPENMPC in the
isolated PX4 copy. Its Git metadata uses independent indexes and the original
local object stores. For WSL, a Linux filesystem output directory avoids
cross-filesystem overhead during NuttX configuration. An interrupted preparation can be
repeated with the same arguments; missing source files are filled in.
The build_inputs.json file binds that output directory to its selected inputs.
Create a fresh output directory with this configuration entry when changing
inputs.

## Build and inspect

Use the same compiler and local-dependency policy for the build:

    export PATH="$ARM_DIR/bin:$PATH"
    export GIT_SUBMODULES_ARE_EVIL=1
    export GIT_OPTIONAL_LOCKS=0
    export PYTHONDONTWRITEBYTECODE=1
    export CCACHE_DISABLE=1

    cmake --build "$BUILD_DIR/build" --parallel 2 --target px4
    cmake --build "$BUILD_DIR/build" --parallel 2 --target px4_package

The output directory receives px4_fmu-v6c_default.elf, .bin, .map and .px4,
together with generated parameter metadata. Keep these artifacts and their
hashes together. A rebuild has its own build timestamp and identity; use its
corresponding host configuration when installing it.

The build compiles all 74 controller C translation units, using the selected
observer source from live/vertical_observer. The resulting static
archive and source hashes generate the ABI identity header before compiling
the facade. The identity record is build/gpenmpc_identity/source_identity.json.

## Offline entry tests

    export GPENMPC_TEST_OUTPUT=/absolute/path/to/gpenmpc-test-output
    python3 -B "$PACKAGE/source/hil/rfly_vendor_integration/application_integration/test_configure_clean_application.py"

## Component build tools

The component scripts under source/hil/rfly_vendor_integration use environment
variables for their external inputs. Set the variables needed by the selected
script to absolute paths.

| Variable | Input |
| --- | --- |
| GPENMPC_PX4_ROOT | Integrated PX4 tree with its FMUv6C compile database |
| GPENMPC_BUILD_ROOT | Configured PX4 build directory used by component tools |
| GPENMPC_BASELINE_BUILD_ROOT | Separate baseline build directory used for component comparisons |
| GPENMPC_TOPIC_HEADERS | Directory containing the generated `uORB/topics` headers |
| GPENMPC_TOPIC_REPORT | Generated topic validation JSON used by adapter checks |
| GPENMPC_NUTTX_INCLUDE | NuttX compatibility includes used by component compilation |
| GPENMPC_RECEIVER_PATCH | MAVLink receiver integration patch |
| GPENMPC_STREAM_PATCH | HIL stream integration patch |
| GPENMPC_SOURCE_ROOT | Project source/hil directory |
| GPENMPC_MATLAB_ROOT | MATLAB installation |
| GPENMPC_ARM_TOOLCHAIN | GNU Arm Embedded installation root, containing `bin/arm-none-eabi-gcc` |
| GPENMPC_WSL_DISTRO | Optional WSL distribution for PowerShell component tools; defaults to the system selection |
| GPENMPC_LOCAL_RUNS | Directory for application build results |
| GPENMPC_KERNEL_BUILD | Controller build directory containing generated/ |
| GPENMPC_KERNEL_ARCHIVE | Compiled controller static archive |
| GPENMPC_REFERENCE_FIXTURES | Directory containing CONTINUOUS_REFERENCE_FIXTURE.bin |
| GPENMPC_GP_CHAIN_FIXTURE | SJC binary output fixture |
| GPENMPC_FLOAT_MAPPING_FIXTURES | Directory containing MATLAB_ARGUMENTS_AND_EXPECTED.bin and MATLAB_NED_AND_MAPPED_STATE.bin |
| GPENMPC_BASELINE_TOPIC_HEADERS | Baseline uORB topic headers checked against build dependencies |
| GPENMPC_BASELINE_LINK_MAP | Baseline linker map |
| GPENMPC_LINK_MAP | Linker map to compare |
| GPENMPC_LINK_RECEIPT | Build record for the linked application |
| GPENMPC_ELF_SHA256 | Expected ELF SHA-256 from that build record |
| GPENMPC_RECOVERY_IMAGE | Recovery application package |
| GPENMPC_RECOVERY_SHA256 | Expected recovery package SHA-256 |
| GPENMPC_RECOVERY_BYTES | Expected recovery package size in bytes |
| GPENMPC_STACK_REFERENCE_RECEIPT | Optional cached application stack build record |

Result names can contain letters, digits, underscores and hyphens. For example,
build_local_application.py accepts an output name such as application and an
optional build label. package_local_application.py takes the current and
baseline output names, followed by optional build and package labels. It reads
the matching build records before packaging.

Host test scripts accept explicit compiler, PX4 build, fixture and library
paths. Supply the inputs required by the selected script and use an output
directory outside the source package.
