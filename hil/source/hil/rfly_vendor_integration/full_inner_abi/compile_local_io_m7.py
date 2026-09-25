"""Compile the local I/O interface with FMUv6C flags."""

# Component inputs are supplied by the invoking build/test environment.
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from component_inputs import component_input
component_input('GPENMPC_BUILD_ROOT')

from pathlib import Path
import hashlib
import json
import os
import re
import shlex
import subprocess
import sys

root = Path(__file__).resolve().parent
label = sys.argv[1]
assert re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_-]*", label)
out = root / label
out.mkdir(exist_ok=False)
integration = root.parent
source = integration / "px4_runtime/Px4CanonicalLocalIo.cpp"
tracked = [source, source.with_suffix(".hpp"),
           integration / "CanonicalFullInnerConsumption.hpp",
           integration / "CanonicalLocalInnerInputBuilder.hpp",
           root / "CanonicalFullInnerAbi.h"]
def hashes():
    return {str(p): hashlib.sha256(p.read_bytes()).hexdigest().upper()
            for p in tracked}
before = hashes()
database = component_input('GPENMPC_BUILD_ROOT') / 'compile_commands.json'
entry = next(r for r in json.loads(database.read_text())
             if Path(r["file"]).name in ('Px4CanonicalLocalIo.cpp', 'Px4CanonicalIo.cpp'))
args = shlex.split(entry["command"])
if Path(args[0]).name == "ccache":
    args = args[1:]
assert all(f in args for f in ["-std=gnu++14", "-nostdinc++", "-mcpu=cortex-m7",
                               "-mfloat-abi=hard", "-D__PX4_NUTTX"])
obj = out / "Px4CanonicalLocalIo.o"
args[args.index("-o") + 1] = str(obj)
args[args.index("-c") + 1] = str(source)
args[1:1] = ["-fstack-usage", "-MD", "-MF", str(out / "LocalIo.d")]
process = subprocess.run(args, cwd=entry["directory"],
                         env=dict(os.environ, CCACHE_DISABLE="1"),
                         capture_output=True, text=True)
after = hashes()
report = dict(scope="ACTUAL_NEW_LOCAL_IO_GNU14_NUTTX_M7_SINGLE_TU_COMPILE_ONLY",
              command=args, working_directory=entry["directory"],
              compile_exit=process.returncode,
              output=process.stdout + process.stderr,
              source_sha256=before, source_sha256_after=after,
              source_stable=before == after, generated_C_recompiled=0,
              application_linked=False, PX4_run=False)
if process.returncode == 0:
    def tool(name, *arguments):
        compiler = Path(args[0])
        return subprocess.check_output([
            str(compiler.with_name(compiler.name.replace("g++", name))),
            *map(str, arguments)], text=True)
    report.update(object_sha256=hashlib.sha256(obj.read_bytes()).hexdigest().upper(),
                  size=tool("size", obj), undefined=tool("nm", "-u", "-C", obj),
                  stack_usage=obj.with_suffix(".su").read_text())
    report["unexpected_generated_private_symbols"] = [
        line for line in report["undefined"].splitlines()
        if "gpenmpcNative_canonicalLocalInner" in line or "gpenmpc_private74" in line]
    report["forbidden_dependencies"] = [
        line for line in report["undefined"].splitlines()
        if re.search(r"__atomic|__cxa|__gxx|std::|operator new", line)]
(out / "RESULT.json").write_text(json.dumps(report, indent=2) + "\n")
print(json.dumps({k: v for k, v in report.items()
                  if k not in ["command", "stack_usage", "source_sha256_after"]}, indent=2))
sys.exit(process.returncode or (0 if before == after else 2))
