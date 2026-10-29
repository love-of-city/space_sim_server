"""Write a new saved scene with stereo23/shared-gyro capture explicitly enabled."""
import argparse
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / 'backend')]
from simulation.near23_capture import configure_scene


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scene', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--evidence', type=Path, required=True)
    args = parser.parse_args()
    scene = configure_scene(json.loads(args.scene.read_text('utf-8-sig')), ROOT, args.evidence)
    with args.output.open('x', encoding='utf-8') as stream:
        json.dump(scene, stream, ensure_ascii=False, indent=2)


if __name__ == '__main__':
    main()
