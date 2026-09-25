# GPENMPC FMUv6C firmware

Target: PX4 FMUv6C, board ID 56, revision 0.
PX4 base: `6ea3539157ca358c70a515878b77077af7d4611d`.

The `.px4` package contains the 1,962,048-byte application image. Its SHA256 is:

```text
AF571397E67013D23D59EF2827B879B76A408271713891686790BD64661B4743
```

`firmware.json` records artifact sizes and hashes, compiler versions, controller
identity and matching task, configuration and GP identities. The parameter
files describe this build.

`libgpenmpc_full_inner_private.a` contains the 74 compiled controller translation
units. `FullInnerBuildIdentity.h` supplies the controller build identity used by
the firmware interface.

See [Build](../../docs/build.md) for configuration and compilation, and
[Local configuration](../../docs/setup.md) for matching host assets. License and
source requirements are described in the [package README](../../README.md).
