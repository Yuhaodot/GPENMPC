# Component licenses

The root MIT license covers original GPENMPC code and documentation. File-level
copyright notices and the following component terms remain applicable.

| Component | Included material | Terms |
| --- | --- | --- |
| PX4 | Board files, application overlays, uORB headers and firmware | [BSD 3-Clause](PX4-LICENSE.txt); individual file notices |
| Apache NuttX | Operating-system code linked into the firmware | [Apache 2.0](NuttX-LICENSE.txt), [NuttX notices](NuttX-NOTICE.txt), [application notices](NuttX-apps-NOTICE.txt) and individual file notices |
| MAVLink | Generated C headers in `source/hil/host_runtime/native_include/mavlink` and the firmware | [MAVLink COPYING](MAVLink-COPYING.txt), including the generated-code exception; generator licensing is separate |
| Micro-CDR | `source/hil/rfly_vendor_integration/application_integration/private_dependencies/Micro-CDR` | [Apache 2.0](../source/hil/rfly_vendor_integration/application_integration/private_dependencies/Micro-CDR/LICENSE) |
| Micro-XRCE-DDS Client | Adjacent `private_dependencies/Micro-XRCE-DDS-Client` | [Apache 2.0](../source/hil/rfly_vendor_integration/application_integration/private_dependencies/Micro-XRCE-DDS-Client/LICENSE) |
| PX4 GPS and libevents | Components linked into the firmware | [GPS license](PX4-GPS-LICENSE.txt), [libevents license](libevents-LICENSE.txt) |
| heatshrink | Compression code linked into the firmware | [ISC-style license](heatshrink-LICENSE.txt) |
| GNU Arm runtime | Runtime objects linked by GNU Arm Embedded 10.3-2021.10 | [Toolchain notices](GNU-Arm-Embedded-LICENSE.txt), including the GCC Runtime Library Exception and component terms |
| MATLAB/Simulink | Generated source, supporting runtime code and dependent binaries | [Generated-code and application terms](../docs/licensing.md); original file notices |
| RflySim and MATLAB installations; SDL2 runtime | Separately installed dependencies | Their respective vendor or upstream licenses |

The toolchain and installed simulators are separate dependencies. Inclusion of
their notices does not distribute those installations or grant a license to use
them. `components.json` records the accompanying license-file hashes.
