"""Lightweight launcher preflight; no Basilisk, UE or LeRobot import."""

import sys as _sys
from pathlib import Path as _Path
_sys.path.insert(0, str(_Path(__file__).resolve().parents[2]))

import argparse
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "backend"))
from space_arm_platform.protocol import check_backend_capabilities


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8766)
    args = parser.parse_args()
    try:
        check_backend_capabilities(args.host, args.port)
    except RuntimeError as error:
        print(str(error), file=sys.stderr)
        return 1
    print("Backend state/RGB acknowledgement protocols verified.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
