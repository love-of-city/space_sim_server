"""Build an isolated, placement-only two-board task-box layout prototype."""
from pathlib import Path
import argparse
import json
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from simulation.task_box_layout import build_layout, sampled_offsets


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, default=ROOT / "model/SARM/platform/sarm_20261012.xml")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--seed", type=int, default=20261022)
    parser.add_argument("--range-mm", type=float, default=20.)
    parser.add_argument("--offsets-json", type=Path, help="Replay explicit module [dx,dy] metre offsets")
    args = parser.parse_args()
    offsets = (json.loads(args.offsets_json.read_text()) if args.offsets_json
               else sampled_offsets(args.seed, args.range_mm / 1000))
    result = build_layout(args.source, args.output, offsets, seed=args.seed)
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
