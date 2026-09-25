"""Copy the GPENMPC parameter records into a local configuration directory."""
import argparse
import hashlib
from pathlib import Path
import shutil

HASHES = {
    'CONTENT_MANIFEST.json': '97DEC48D91951A02BE4F6454B973AE9E20BE1D2B08F388F601DFA796CDBD414F',
    'OUTER_RESULT.json': 'EA791258BF280E24A5499D9516B40D71138E66A2D51A90A7B628C4DF86F7A0FC',
    'PLAN.json': '054B621899393A74F7A515C04FFC1FFF5FCB8943B30BB7CDD4030B5452ABA47E',
    'SERIAL_PREFLIGHT.json': '073E7CE165D8D158BC329CFE96AEC4F968A6847A2A0AB9D90C1C127D3CB65EBB',
}


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest().upper()


def export(records, plan, destination):
    sources = {name: plan if name == 'PLAN.json' else records / name for name in HASHES}
    for name, source in sources.items():
        if digest(source) != HASHES[name]:
            raise ValueError('Recovery checksum differs: ' + name)
        target = destination / name
        if target.exists() and digest(target) != HASHES[name]:
            raise FileExistsError('Destination contains different data: ' + str(target))
    destination.mkdir(parents=True, exist_ok=True)
    for name, source in sources.items():
        target = destination / name
        if not target.exists():
            with source.open('rb') as src, target.open('xb') as dst:
                shutil.copyfileobj(src, dst)
        if digest(target) != HASHES[name]:
            raise IOError('Copy verification failed: ' + name)
    return destination.resolve()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--records', type=Path, required=True)
    parser.add_argument('--plan', type=Path, required=True)
    parser.add_argument('--destination', type=Path, required=True)
    args = parser.parse_args()
    print(export(args.records, args.plan, args.destination))


if __name__ == '__main__':
    main()
