"""Link-only resource test using the existing exact PX4 link command.

No rebuild, configuration change, flash, CLI start or packaging. The original
application/archives stay byte-exact. This is NOT the runtime integration:
retaining explicit entry points proves link closure/FLASH use, not execution.
Run with WSL Python, so the original ARM toolchain and link paths are real.
"""
import argparse
import hashlib
import json
from pathlib import Path
import shlex
import subprocess


def sha(path):
    h = hashlib.sha256()
    with path.open("rb") as f:
        for block in iter(lambda: f.read(1024 * 1024), b""):
            h.update(block)
    return h.hexdigest().upper()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--build", type=Path, required=True)
    parser.add_argument("--archive", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--facade-object", type=Path)
    parser.add_argument("--extra-object", type=Path, action="append", default=[])
    parser.add_argument("--entry", action="append", default=[])
    a = parser.parse_args()
    a.build = a.build.resolve(strict=True)
    a.archive = a.archive.resolve(strict=True)
    if a.output.exists():
        raise RuntimeError("Preserve existing link-only receipt")
    a.output.mkdir(parents=True)
    original = a.build / "px4_fmu-v6c_default.elf"
    before = sha(original)
    # Print Ninja build commands.
    listing = subprocess.run(["ninja", "-t", "commands", original.name],
                             cwd=a.build, text=True, capture_output=True, check=True)
    candidates = [s for s in listing.stdout.splitlines()
                  if " -o " + original.name + " " in s and "arm-none-eabi-g++" in s]
    if len(candidates) != 1:
        raise RuntimeError(f"Exact final link command not unique: {len(candidates)}")
    parts = shlex.split(candidates[0])
    start = next(i for i, p in enumerate(parts) if p.endswith("/arm-none-eabi-g++"))
    end = parts.index("&&", start) if "&&" in parts[start:] else len(parts)
    command = parts[start:end]
    if command.count("-o") != 1:
        raise RuntimeError("Unexpected original output count")
    derived = a.output / "px4_local74_resource_only.elf"
    map_path = a.output / "px4_local74_resource_only.map"
    command[command.index("-o") + 1] = str(derived)
    map_count = 0
    for i, item in enumerate(command):
        if item.startswith("-Wl,-Map="):
            command[i] = "-Wl,-Map=" + str(map_path)
            map_count += 1
    if map_count != 1:
        raise RuntimeError(f"Expected one exact map output, got {map_count}")
    entries = a.entry or [
        "gpenmpcNative_canonicalLocalInnerFixedFirst",
        "gpenmpcNative_canonicalLocalInnerFixedStep",
        "gpenmpcNative_queryCanonicalReferenceWindow",
        "gpenmpcNative_canonicalReferenceTransitionFromJet",
    ]
    if any(not n.replace("_", "").isalnum() for n in entries):
        raise RuntimeError("Literal function names only")
    # Resolve the closure through the NuttX system libraries before libm and libgcc.
    index = command.index("-lm")
    extra = ["-Wl,--undefined=" + name for name in entries]
    if a.facade_object:
        a.facade_object = a.facade_object.resolve(strict=True)
        extra.append(str(a.facade_object))
    extra.extend(str(p.resolve(strict=True)) for p in a.extra_object)
    extra.append(str(a.archive))
    command[index:index] = extra
    inputs = []
    for item in command:
        if item.endswith((".a", ".obj", ".o")):
            p = Path(item)
            if not p.is_absolute():
                p = a.build / p
            p = p.resolve(strict=True)
            inputs.append({"path": str(p), "sha256": sha(p)})
    (a.output / "COMMAND.json").write_text(json.dumps(command, indent=2), encoding="utf-8")
    run = subprocess.run(command, cwd=a.build, text=True, capture_output=True, timeout=90)
    (a.output / "LINK.log").write_text(run.stdout + run.stderr, encoding="utf-8")
    stable = sha(original) == before and all(sha(Path(p["path"])) == p["sha256"] for p in inputs)
    result = {
        "scope": "PX4_LINK_RESOURCE_ASSESSMENT",
        "pass": run.returncode == 0 and stable,
        "link_returncode": run.returncode, "original_application_sha256": before,
        "original_and_inputs_unchanged": stable, "inputs": inputs,
        "retained_entry_points": entries, "command": command,
        "hardware_actions": 0, "application_started": False,
        "limitation": "Undefined anchors retain the linked code. Scheduler integration, heap/stores, IO ownership, task stack and WCET require separate owner configuration.",
    }
    if run.returncode == 0:
        result.update(derived_sha256=sha(derived), derived_bytes=derived.stat().st_size)
        nm = command[0].replace("g++", "nm")
        symbols = subprocess.run([nm, "-g", "--defined-only", str(derived)],
                                 text=True, capture_output=True, check=True)
        (a.output / "DEFINED_SYMBOLS.txt").write_text(symbols.stdout, encoding="utf-8")
        names = [line.split()[-1] for line in symbols.stdout.splitlines() if line.split()]
        result["retained_entries_exactly_once"] = all(names.count(n) == 1 for n in entries)
        result["pass"] = result["pass"] and result["retained_entries_exactly_once"]
        result["link_memory_report"] = run.stdout
    (a.output / "RESULT.json").write_text(json.dumps(result, indent=2), encoding="utf-8")
    print(json.dumps({k: result[k] for k in ("scope", "pass", "link_returncode", "original_and_inputs_unchanged")}))
    print(run.stdout[-2500:])
    if run.returncode:
        print(run.stderr[-6000:])
    return 0 if result["pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
