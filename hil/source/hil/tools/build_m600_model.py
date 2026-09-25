"""Build the M600 model DLL with LLVM MinGW."""
import argparse
from pathlib import Path
import subprocess


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--compiler', type=Path, required=True)
    parser.add_argument('--matlab-root', type=Path, required=True)
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    model = root / 'm600_coptersim/model'
    generated = model / 'generated/GPENMPC_M600_Canonical_ert_rtw'
    wrapper = model / 'wrapper'
    includes = [generated, wrapper, root / 'host_runtime/copter_observer',
                args.matlab_root / 'simulink/include',
                args.matlab_root / 'extern/include', args.matlab_root / 'rtw/c/src']
    sources = [generated / 'GPENMPC_M600_Canonical.cpp',
               wrapper / 'modeldllgen.cpp', wrapper / 'rfly_export_compat.def']
    for path in [args.compiler, *includes, *sources]:
        if not path.exists():
            parser.error(f'Missing build input: {path}')
    output = model / 'GPENMPC_M600_Diagnostic.dll'
    command = [str(args.compiler), '-std=c++17', '-O2', '-shared', '-static',
               '-fms-extensions', '-fdeclspec', '-Wl,--no-undefined']
    command += ['-I' + str(path) for path in includes]
    command += [str(path) for path in sources]
    command += ['-lws2_32', '-o', str(output)]
    subprocess.run(command, check=True)
    print(output)


if __name__ == '__main__':
    main()
