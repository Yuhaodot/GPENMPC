#!/usr/bin/env python3
"""Configure PX4 using private writable NuttX and board source areas."""

# Component inputs are supplied by the invoking build/test environment.
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from component_inputs import component_input
component_input('GPENMPC_BUILD_ROOT')
component_input('GPENMPC_BASELINE_BUILD_ROOT')

from pathlib import Path
import hashlib
import json
import os
import shutil
import subprocess
import time
import argparse

ROOT = Path(__file__).resolve().parent
PX4 = Path(os.environ['GPENMPC_PX4_ROOT'])
PRIVATE = ROOT / 'private_source'
BUILD = component_input('GPENMPC_BUILD_ROOT')
INIT = ROOT / 'private_cmake/init.cmake'
OLD_BUILD = component_input('GPENMPC_BASELINE_BUILD_ROOT')
if BUILD == OLD_BUILD:
    raise ValueError('The isolated build directory must differ from the baseline build directory')


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest().upper()


def stat_snapshot(focused=False):
    """Capture metadata and selected configuration hashes."""
    result = {}
    if focused:
        # Capture the selected source and build configuration anchors.
        for relative in ('CMakeLists.txt', 'platforms/nuttx/cmake/init.cmake',
                'platforms/nuttx/NuttX/CMakeLists.txt',
                'platforms/nuttx/cmake/px4_impl_os.cmake',
                'platforms/nuttx/NuttX/nuttx/.config',
                'platforms/nuttx/NuttX/nuttx/defconfig',
                'platforms/nuttx/NuttX/nuttx/Make.defs',
                'boards/px4/fmu-v6c/default.px4board',
                'boards/px4/fmu-v6c/nuttx-config/defconfig',
                'build/px4_fmu-v6c_default/CMakeCache.txt',
                'build/px4_fmu-v6c_default/compile_commands.json',
                'build/px4_fmu-v6c_default/build.ninja'):
            p = PX4 / relative
            if p.exists():
                st = p.lstat()
                result[relative] = [st.st_size, st.st_mtime_ns, sha(p)]
            else:
                result[relative] = None
        return result
    for subtree in ('platforms/nuttx/NuttX', 'boards/px4/fmu-v6c',
                    'build/px4_fmu-v6c_default'):
        for directory, names, files in os.walk(PX4 / subtree, followlinks=False):
            for name in names + files:
                p = Path(directory) / name
                st = p.lstat()
                result[str(p.relative_to(PX4))] = [st.st_size, st.st_mtime_ns,
                                                  os.readlink(p) if p.is_symlink() else None]
    return result


def assert_private(path):
    path.resolve().relative_to(PRIVATE.resolve())


def capture_config_restat():
    # NuttX's actual olddefconfig still runs. Preserve timestamps afterwards
    # only for byte-identical private build products, equivalent to restat.
    paths = [PRIVATE / 'platforms/nuttx/NuttX/nuttx/.config',
             BUILD / 'NuttX/nuttx/.config',
             PRIVATE / 'platforms/nuttx/NuttX/nuttx/defconfig',
             PRIVATE / 'platforms/nuttx/NuttX/nuttx/include/nuttx/config.h',
             PRIVATE / 'platforms/nuttx/NuttX/nuttx/include/nuttx/version.h',
             BUILD / 'NuttX/extra_config_options',
             BUILD / 'defconfig_inflate_stamp']
    result = {}
    for p in paths:
        if p.is_file():
            st = p.stat()
            result[str(p)] = {'sha256': sha(p), 'atime_ns': st.st_atime_ns,
                              'mtime_ns': st.st_mtime_ns}
    return result


def apply_equal_config_restat(before, configure_rc):
    configurations = [PRIVATE / 'platforms/nuttx/NuttX/nuttx/.config',
                      BUILD / 'NuttX/nuttx/.config']
    same = configure_rc == 0 and all(str(p) in before and p.is_file()
        and sha(p) == before[str(p)]['sha256'] for p in configurations)
    rows = []
    for name, prior in before.items():
        p = Path(name)
        # Never restat the original source or prior build; exact captured paths
        # above are inside our two private locations only.
        assert p.is_relative_to(PRIVATE) or p.is_relative_to(BUILD)
        unchanged = p.is_file() and sha(p) == prior['sha256']
        restored = same and unchanged
        if restored:
            os.utime(p, ns=(prior['atime_ns'], prior['mtime_ns']))
        rows.append({'path': name, 'sha256_before': prior['sha256'],
                     'bytes_unchanged': unchanged, 'mtime_restated': restored})
    return {'actual_olddefconfig_not_skipped': True,
            'both_complete_configurations_byte_identical': same, 'files': rows}


def prepare():
    if PRIVATE.exists():
        assert INIT.exists(), 'An incomplete private copy needs explicit inspection'
        return
    PRIVATE.mkdir()
    for relative in ('platforms/nuttx/NuttX', 'boards/px4/fmu-v6c'):
        target = PRIVATE / relative
        target.parent.mkdir(parents=True, exist_ok=True)
        print('Copying only private mutable subtree ' + relative, flush=True)
        subprocess.run(['cp', '-a', str(PX4 / relative), str(target)], check=True)
    links = []
    for directory, names, files in os.walk(PRIVATE, followlinks=False):
        for name in names + files:
            path = Path(directory) / name
            if path.is_symlink():
                value = os.readlink(path)
                if value.startswith(str(PX4) + '/'):
                    new_value = str(PRIVATE) + value[len(str(PX4)):]
                    # Only remove this exact newly created private symlink.
                    path.parent.resolve().relative_to(PRIVATE.resolve())
                    path.unlink()
                    path.symlink_to(new_value)
                    links.append({'path': str(path), 'original_target': value,
                                  'private_target': new_value})
                assert_private(path)
    text = (PX4 / 'platforms/nuttx/cmake/init.cmake').read_text()
    replacements = {
        'set(NUTTX_SRC_DIR  ${PX4_SOURCE_DIR}/platforms/nuttx/NuttX)':
        'set(NUTTX_SRC_DIR "' + str(PRIVATE / 'platforms/nuttx/NuttX') + '")',
        'set(NUTTX_DIR      ${PX4_SOURCE_DIR}/platforms/nuttx/NuttX/nuttx CACHE FILEPATH "NuttX directory" FORCE)':
        'set(NUTTX_DIR "${NUTTX_SRC_DIR}/nuttx" CACHE FILEPATH "NuttX directory" FORCE)',
        'set(NUTTX_APPS_DIR ${PX4_SOURCE_DIR}/platforms/nuttx/NuttX/apps CACHE FILEPATH "NuttX apps directory" FORCE)':
        'set(NUTTX_APPS_DIR "${NUTTX_SRC_DIR}/apps" CACHE FILEPATH "NuttX apps directory" FORCE)'}
    for before, after in replacements.items():
        assert text.count(before) == 1, before
        text = text.replace(before, after)
    INIT.parent.mkdir()
    INIT.write_text(text)
    (ROOT / 'PRIVATE_COPY.json').write_text(json.dumps({
        'private_root': str(PRIVATE), 'source_root': str(PX4),
        'init_original_sha256': sha(PX4 / 'platforms/nuttx/cmake/init.cmake'),
        'init_derived_sha256': sha(INIT), 'exact_init_replacements': replacements,
        'absolute_symlinks_retargeted': links,
        'all_private_symlinks_resolve_within_private_source': True}, indent=2) + '\n')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--focused-check', action='store_true')
    args = parser.parse_args()
    label = 'CONFIGURE_' + time.strftime('%Y%m%d_%H%M%S')
    before = stat_snapshot(args.focused_check)
    (ROOT / (label + '_before.json')).write_text(json.dumps(before, indent=2) + '\n')
    prepare()
    # The parent OS flag helper hardcodes seven include roots rather than
    # using NUTTX_DIR. Keep every flag/function intact; redirect only those
    # include paths to the real private NuttX context generated by this build.
    os_flags_source = PX4 / 'platforms/nuttx/cmake/px4_impl_os.cmake'
    os_flags_text = os_flags_source.read_text()
    os_flags_prefix = '${PX4_SOURCE_DIR}/platforms/nuttx/NuttX'
    assert os_flags_text.count(os_flags_prefix) == 7
    private_os_flags = INIT.parent / 'px4_impl_os.cmake'
    os_flags_derived = os_flags_text.replace(os_flags_prefix, '${NUTTX_SRC_DIR}')
    if not private_os_flags.exists() or private_os_flags.read_text() != os_flags_derived:
        private_os_flags.write_text(os_flags_derived)
    include = '\ninclude("' + str(INIT.parent / 'private_board_subdirectory.cmake') + '")\n'
    init_text = INIT.read_text()
    if include not in init_text:
        INIT.write_text(init_text + include)
    # Top-level CMakeLists includes the original OS helper by an explicit
    # relative filename before include(init), overriding the earlier kconfig
    # module-path definition. Reload the exact derived helper here, after that
    # include and before the top-level px4_os_add_flags() call.
    os_include = '\ninclude("' + str(private_os_flags) + '")\n'
    init_text = INIT.read_text()
    if os_include not in init_text:
        INIT.write_text(init_text + os_include)
    assert_private(PRIVATE / 'platforms/nuttx/NuttX/nuttx')
    assert_private(PRIVATE / 'platforms/nuttx/NuttX/apps')
    board = PRIVATE / 'boards/px4/fmu-v6c'
    argv = ['cmake', '-S', str(PX4), '-B', str(BUILD), '-G', 'Ninja',
            '-DCONFIG=px4_fmu-v6c_default',
            '-DEXTERNAL_MODULES_LOCATION=' + str(ROOT / 'external'),
            '-DCMAKE_MODULE_PATH=' + str(INIT.parent),
            '-DPX4_CONFIG_FILE=' + str(board / 'default.px4board'),
            '-DPX4_BOARD_DIR=' + str(board), '-DMODEL=fmu-v6c',
            '-DVENDOR=px4', '-DLABEL=default', '-DPYTHON_EXECUTABLE=/usr/bin/python3',
            '-DCMAKE_EXPORT_COMPILE_COMMANDS=ON']
    # Reuse the actually recorded compiler path without invoking its old build.
    database = json.loads((OLD_BUILD / 'compile_commands.json').read_text())
    import shlex
    compiler = shlex.split(database[0]['command'])[0]
    environment = dict(os.environ)
    environment['PATH'] = str(Path(compiler).parent) + ':' + environment.get('PATH', '')
    environment['GIT_SUBMODULES_ARE_EVIL'] = '1'
    environment['GIT_OPTIONAL_LOCKS'] = '0'
    environment['PYTHONDONTWRITEBYTECODE'] = '1'
    environment['OMP_NUM_THREADS'] = '1'
    environment['OPENBLAS_NUM_THREADS'] = '1'
    environment['MKL_NUM_THREADS'] = '1'
    print('Real FMUv6C configure; all NuttX writes must resolve to ' + str(PRIVATE), flush=True)
    log = ROOT / (label + '.log')
    config_before = capture_config_restat()
    with log.open('w') as out:
        try:
            run = subprocess.run(argv, env=environment, stdin=subprocess.DEVNULL,
                                 stdout=out, stderr=subprocess.STDOUT, timeout=1800)
            return_code = run.returncode
        except subprocess.TimeoutExpired:
            return_code = 124
            out.write('\nCMake configuration timed out.\n')
    # Regenerate headers removed by olddefconfig before comparing content.
    context_return_code = None
    if return_code == 0:
        context_log = ROOT / (label + '_context.log')
        with context_log.open('w') as context_out:
            context_run = subprocess.run(['cmake', '--build', str(BUILD),
                '--parallel', '2', '--target', 'nuttx_context'],
                env=environment, stdin=subprocess.DEVNULL,
                stdout=context_out, stderr=subprocess.STDOUT)
        context_return_code = context_run.returncode
        if context_return_code:
            return_code = context_return_code
    config_restat = apply_equal_config_restat(config_before, return_code)
    after = stat_snapshot(args.focused_check)
    differences = [p for p in sorted(set(before) | set(after)) if before.get(p) != after.get(p)]
    result = {'scope': 'ACTUAL_FMUV6C_CMAKE_CONFIGURE_ONLY', 'command': argv,
        'exit_code': return_code, 'log': str(log), 'log_sha256': sha(log),
        'metadata_check_scope': 'EXACT_WRITE_TARGETS_AND_CONFIG_ANCHORS' if args.focused_check else 'ORIGINAL_NUTTX_BOARD_AND_BUILD',
        'old_mutable_location_stat_entries': len(before),
        'old_mutable_location_stat_changes': differences,
        'old_source_and_old_build_metadata_unchanged': not differences,
        'private_equal_content_restat': config_restat,
        'actual_nuttx_context_return_code': context_return_code,
        'private_init': str(INIT), 'private_init_sha256': sha(INIT),
        'private_os_flags': str(private_os_flags),
        'private_os_flags_sha256': sha(private_os_flags),
        'original_os_flags_sha256': sha(os_flags_source),
        'os_flags_exact_include_prefix_replacements': 7,
        'git_submodule_update_disabled': True, 'python_bytecode_writes_disabled': True,
        'module_entry_implemented': True, 'production_context_implemented': True,
        'runtime_context_supplied': False,
        'firmware_linked': False,
        'firmware_executed': False,
        'passed': return_code == 0 and not differences}
    (ROOT / (label + '.json')).write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps({key: result[key] for key in ('scope', 'exit_code', 'passed',
        'old_source_and_old_build_metadata_unchanged')}), flush=True)
    print(log.read_text()[-6000:], flush=True)
    raise SystemExit(0 if result['passed'] else 1)


if __name__ == '__main__':
    main()
