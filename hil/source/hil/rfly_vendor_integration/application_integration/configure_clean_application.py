#!/usr/bin/env python3
"""Configure the GPENMPC FMUv6C application in a separate output directory."""
import argparse
from concurrent.futures import ThreadPoolExecutor
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import signal
import subprocess
import sys
from prepare_px4_sources import prepare as prepare_px4
from prepare_px4_sources import extension_prefix
from private_git_metadata import prepare as prepare_git

ROOT = Path(__file__).resolve().parent
PX4_COMMIT = "6ea3539157ca358c70a515878b77077af7d4611d"


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest().upper()


def capture(args, cwd=None, env=None):
    return subprocess.check_output(args, cwd=cwd, env=env, text=True).strip()


def absolute_directory(value):
    path = Path(value)
    if not path.is_absolute() or not path.is_dir():
        raise argparse.ArgumentTypeError("Expected an existing absolute directory: " + value)
    return path.resolve()


def output_path(value):
    path = Path(value)
    if not path.is_absolute():
        raise argparse.ArgumentTypeError("The output root must be absolute")
    return path.resolve()


def board_uid(value):
    try:
        parsed = int(value, 16 if value.lower().startswith("0x") else 10)
    except ValueError as error:
        raise argparse.ArgumentTypeError("Expected a decimal or hexadecimal board UID") from error
    if not 0 < parsed <= 0xFFFFFFFFFFFFFFFF:
        raise argparse.ArgumentTypeError("Board UID must be a nonzero unsigned 64-bit integer")
    return parsed


def validate_output(output, inputs):
    for source in inputs:
        if output == source or output.is_relative_to(source) or source.is_relative_to(output):
            raise ValueError("Output and input directories must be separate: " + str(source))


def require_file(path):
    if not path.is_file():
        raise FileNotFoundError("Required input is missing: " + str(path))


def copy_missing(source, target):
    target.parent.mkdir(parents=True, exist_ok=True)
    if target.is_file() and source.read_bytes() == target.read_bytes():
        return 0
    if target.is_symlink():
        raise ValueError("Prepared package target must be a regular file: " + str(target))
    shutil.copy2(source, target)
    return 1


def copy_tracked(source, target, px4, private, env, restored=None):
    """Copy tracked working-tree files, excluding generated build products."""
    entries = subprocess.check_output(
        ["git", "-C", str(source), "ls-files", "--stage", "-z"], env=env)
    def copy_entry(entry):
        if not entry:
            return 0
        info, name = entry.decode().split("\t", 1)
        mode, blob = info.split()[:2]
        if mode == "160000":
            return 0
        relative = Path(name)
        if relative.is_absolute() or ".." in relative.parts:
            raise ValueError("Invalid tracked path: " + name)
        src, dst = source / relative, target / relative
        if dst.exists() or dst.is_symlink():
            return 0
        dst.parent.mkdir(parents=True, exist_ok=True)
        if not src.exists() and not src.is_symlink():
            content = subprocess.check_output(
                ["git", "-C", str(source), "cat-file", "blob", blob], env=env)
            if mode == "120000":
                value = content.decode()
                resolved = (src.parent/value).resolve()
                mapped = private / resolved.relative_to(px4)
                dst.symlink_to(os.path.relpath(mapped, dst.parent))
            else:
                dst.write_bytes(content)
                dst.chmod(0o755 if mode == "100755" else 0o644)
            if restored is not None:
                restored.append({"source": str(src), "git_blob": blob})
            return 1
        if src.is_symlink():
            resolved = src.resolve()
            mapped = private / resolved.relative_to(px4)
            mapped.relative_to(private)
            dst.symlink_to(os.path.relpath(mapped, dst.parent))
            return 1
        else:
            temporary = dst.with_name(dst.name + ".gpenmpc-copy")
            shutil.copyfile(src, temporary)
            temporary.chmod(0o755 if mode == "100755" else 0o644)
            temporary.replace(dst)
            return 1
    with ThreadPoolExecutor(max_workers=16) as pool:
        return sum(pool.map(copy_entry, entries.split(b"\0")))


def copy_package_tree(source, target):
    copied = 0
    for path in sorted(source.rglob("*")):
        rel = path.relative_to(source)
        if any(p in {".git", "build", "CMakeFiles", "__pycache__"} for p in rel.parts):
            continue
        if path.is_symlink():
            raise ValueError("Package source symlink requires inspection: " + str(path))
        if path.is_file() and path.suffix not in {".o", ".a", ".pyc"}:
            copied += copy_missing(path, target / rel)
    return copied


def cmake_path(path):
    text = str(path)
    if any(c in text for c in ('"', ";", "\n", "\r", "$")):
        raise ValueError("Unsupported CMake path character: " + text)
    return text.replace("\\", "/")


def write_generated(path, text):
    path.parent.mkdir(parents=True, exist_ok=True)
    if not path.exists() or path.read_text() != text:
        path.write_text(text)


def environment(toolchain):
    env = dict(os.environ)
    env.update(GIT_OPTIONAL_LOCKS="0", GIT_SUBMODULES_ARE_EVIL="1",
               PYTHONDONTWRITEBYTECODE="1", CCACHE_DISABLE="1")
    env["PATH"] = str(toolchain / "bin") + os.pathsep + env.get("PATH", "")
    return env


def validate(args):
    validate_output(args.output_root, [ROOT.parents[3], args.px4_root,
                                      args.arm_toolchain, args.matlab_root])
    env = environment(args.arm_toolchain)
    compiler = args.arm_toolchain / "bin/arm-none-eabi-g++"
    for path in [compiler, args.arm_toolchain/"bin/arm-none-eabi-gcc",
                 args.matlab_root/"extern/include/tmwtypes.h",
                 args.px4_root/"CMakeLists.txt",
                 args.px4_root/"platforms/nuttx/NuttX/nuttx/Makefile",
                 args.px4_root/"platforms/nuttx/NuttX/apps/Application.mk",
                 ROOT/"private_source/boards/px4/fmu-v6c/default.px4board",
                 ROOT.parent.parent/"live/vertical_observer/gpenmpcObserveCausalVerticalDisturbance.c"]:
        require_file(path)
    extension_prefix(args.px4_root)
    require_file(args.px4_root / "src/modules/mavlink/CMakeLists.txt")
    if capture(["git", "-C", str(args.px4_root), "rev-parse", "HEAD"], env=env) != PX4_COMMIT:
        raise ValueError("PX4 base commit does not match the packaged application")
    version = capture([str(compiler), "-dumpfullversion"], env=env)
    if version != "10.3.1":
        raise ValueError("Expected ARM GCC 10.3.1; found " + version)
    generated = ROOT.parent.parent/"evidence/tracking_controller/generated"
    if len(list(generated.glob('*.c'))) != 74:
        raise ValueError("Expected the complete 74-file controller source set")
    for tool in ["cmake", "ninja", "git", "make"]:
        if not shutil.which(tool, path=env["PATH"]):
            raise FileNotFoundError("Required executable: " + tool)
    capture([sys.executable, "-c", "import kconfiglib, menuconfig, jinja2, em, yaml"], env=env)
    return env, {"px4_commit": PX4_COMMIT, "arm_gcc": version,
                 "cmake": capture(["cmake", "--version"], env=env).splitlines()[0],
                 "ninja": capture(["ninja", "--version"], env=env)}


def prepare(args, env, versions):
    out = args.output_root
    binding = {"schema": "GPENMPC_FMUV6C_BUILD_V1", "px4_root": str(args.px4_root),
               "arm_toolchain": str(args.arm_toolchain), "matlab_root": str(args.matlab_root),
               "hil_source": str(ROOT.parent.parent), "versions": versions}
    marker = out / "build_inputs.json"
    if marker.exists():
        if json.loads(marker.read_text()) != binding:
            raise ValueError("Output directory belongs to different build inputs")
    elif out.exists() and any(out.iterdir()):
        raise ValueError("Use an empty output directory or a matching prepared directory")
    out.mkdir(parents=True, exist_ok=True)
    write_generated(marker, json.dumps(binding, indent=2)+"\n")
    integration = out / "integration"
    private = integration / "private_source"
    copied = {}
    restored = []
    for relative in ["platforms/nuttx/NuttX", "platforms/nuttx/NuttX/nuttx",
                     "platforms/nuttx/NuttX/apps", "boards/px4/fmu-v6c"]:
        copied[relative] = copy_tracked(args.px4_root/relative, private/relative,
                                       args.px4_root, private, env, restored)
    board_source = ROOT / "private_source/boards/px4/fmu-v6c"
    for path in board_source.rglob("*"):
        if path.is_file():
            destination = private/"boards/px4/fmu-v6c"/path.relative_to(board_source)
            destination.parent.mkdir(parents=True, exist_ok=True)
            if not destination.exists() or sha(path) != sha(destination):
                shutil.copy2(path, destination)
    for submodule in ["nuttx", "apps"]:
        source = args.px4_root/"platforms/nuttx/NuttX"/submodule
        prepare_git(source, private/"platforms/nuttx/NuttX"/submodule, env)
    for name in ["external", "private_cmake", "private_dependencies"]:
        copied[name] = copy_package_tree(ROOT/name, integration/name)
    for name in ["PreparedPaths.cmake", "LateRuntimeSources.cmake", "PrivateExternalProjects.cmake",
                 "KernelLibrary.cmake", "write_build_identity.py"]:
        write_generated(integration/name, (ROOT/name).read_text())
    text = (args.px4_root/"platforms/nuttx/cmake/init.cmake").read_text()
    replacements = {
        "set(NUTTX_SRC_DIR  ${PX4_SOURCE_DIR}/platforms/nuttx/NuttX)":
            'set(NUTTX_SRC_DIR "${CMAKE_CURRENT_LIST_DIR}/../private_source/platforms/nuttx/NuttX")',
        'set(NUTTX_DIR      ${PX4_SOURCE_DIR}/platforms/nuttx/NuttX/nuttx CACHE FILEPATH "NuttX directory" FORCE)':
            'set(NUTTX_DIR "${NUTTX_SRC_DIR}/nuttx" CACHE FILEPATH "NuttX directory" FORCE)',
        'set(NUTTX_APPS_DIR ${PX4_SOURCE_DIR}/platforms/nuttx/NuttX/apps CACHE FILEPATH "NuttX apps directory" FORCE)':
            'set(NUTTX_APPS_DIR "${NUTTX_SRC_DIR}/apps" CACHE FILEPATH "NuttX apps directory" FORCE)'}
    for old, new in replacements.items():
        if text.count(old) != 1:
            raise ValueError("Unsupported PX4 NuttX init layout: " + old)
        text = text.replace(old, new)
    text += '\ninclude("${CMAKE_CURRENT_LIST_DIR}/private_board_subdirectory.cmake")\n'
    text += 'include("${CMAKE_CURRENT_LIST_DIR}/px4_impl_os.cmake")\n'
    write_generated(integration/"private_cmake/init.cmake", text)
    flags = (args.px4_root/"platforms/nuttx/cmake/px4_impl_os.cmake").read_text()
    if flags.count("${PX4_SOURCE_DIR}/platforms/nuttx/NuttX") != 7:
        raise ValueError("Unsupported PX4 NuttX include layout")
    write_generated(integration/"private_cmake/px4_impl_os.cmake",
                    flags.replace("${PX4_SOURCE_DIR}/platforms/nuttx/NuttX", "${NUTTX_SRC_DIR}"))
    superbuild = integration/"private_dependencies/Micro-XRCE-DDS-Client/cmake/SuperBuild.cmake"
    text = superbuild.read_text()
    text, count = re.subn(r'(ExternalProject_Add\(microcdr\s+SOURCE_DIR\s+)"[^"]+"',
                         r'\1"${CMAKE_CURRENT_LIST_DIR}/../../Micro-CDR"', text)
    if count != 1:
        raise ValueError("Expected one local Micro-CDR source declaration")
    write_generated(superbuild, text)
    for relative in ["platforms/nuttx/NuttX/nuttx/Makefile",
                     "platforms/nuttx/NuttX/apps/Application.mk",
                     "boards/px4/fmu-v6c/default.px4board"]:
        require_file(private/relative)
    copied["restored_tracked_files"] = restored
    args.staged_px4 = prepare_px4(args.px4_root, out/"px4", env)
    for name in ['nuttx','apps']:
        alias=args.staged_px4/'platforms/nuttx/NuttX'/name
        source=private/'platforms/nuttx/NuttX'/name
        if not alias.exists(): alias.symlink_to(os.path.relpath(source,alias.parent),target_is_directory=True)
    return integration, copied


def configure_command(args, integration):
    board = integration/"private_source/boards/px4/fmu-v6c"
    return ["cmake", "-S", str(getattr(args, 'staged_px4', args.px4_root)), "-B", str(args.output_root/"build"),
            "-G", "Ninja", "-DCONFIG=px4_fmu-v6c_default",
            "-DEXTERNAL_MODULES_LOCATION="+cmake_path(integration/"external"),
            "-DCMAKE_MODULE_PATH="+cmake_path(integration/"private_cmake"),
            "-DPX4_CONFIG_FILE="+cmake_path(board/"default.px4board"),
            "-DPX4_BOARD_DIR="+cmake_path(board),
            "-DGPENMPC_HIL_SOURCE_DIR="+cmake_path(ROOT.parent.parent),
            "-DGPENMPC_MATLAB_INCLUDE="+cmake_path(args.matlab_root/"extern/include"),
            "-DGPENMPC_BOARD_UID="+str(args.board_uid),
            "-DMODEL=fmu-v6c", "-DVENDOR=px4", "-DLABEL=default",
            "-DPYTHON_EXECUTABLE="+sys.executable, "-DCMAKE_EXPORT_COMPILE_COMMANDS=ON"]


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--px4-root", required=True, type=absolute_directory)
    parser.add_argument("--arm-toolchain", required=True, type=absolute_directory)
    parser.add_argument("--matlab-root", required=True, type=absolute_directory)
    parser.add_argument("--output-root", required=True, type=output_path)
    parser.add_argument("--board-uid", required=True, type=board_uid)
    parser.add_argument("--check-only", action="store_true")
    parser.add_argument("--prepare-only", action="store_true")
    parser.add_argument("--configure-timeout", type=int, default=900)
    args = parser.parse_args(argv)
    if os.name != "posix":
        parser.error("Run this entry with Linux Python, for example inside WSL")
    if args.configure_timeout < 1:
        parser.error("--configure-timeout must be positive")
    env, versions = validate(args)
    if args.check_only:
        print(json.dumps({"inputs_valid": True, "versions": versions}, indent=2))
        return 0
    integration, copied = prepare(args, env, versions)
    command = configure_command(args, integration)
    receipt = {"inputs": versions, "configure_command": command, "copied_files": copied,
               "prepare_only": args.prepare_only, "configure_exit_code": None}
    code = 0
    if not args.prepare_only:
        with (args.output_root/"configure.log").open("w") as log:
            process = subprocess.Popen(command, env=env, stdout=log, stderr=subprocess.STDOUT,
                                       stdin=subprocess.DEVNULL, start_new_session=True)
            try:
                code = process.wait(timeout=args.configure_timeout)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGTERM)
                try:
                    process.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait()
                code = 124
        receipt["configure_exit_code"] = code
    (args.output_root/"configure_result.json").write_text(json.dumps(receipt, indent=2)+"\n")
    print(json.dumps(receipt, indent=2))
    return code


if __name__ == "__main__":
    sys.exit(main())
