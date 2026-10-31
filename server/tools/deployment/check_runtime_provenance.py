"""Check collection code and native/UE build provenance without starting a task."""
from pathlib import Path
import argparse
import json
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT), str(ROOT / "backend")]
from collection.runtime_provenance import runtime_identity, fingerprint


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--adapter", type=Path)
    parser.add_argument("--unreal", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    identity = runtime_identity(adapter=args.adapter, unreal=args.unreal)
    document = {"verified": True, "runtime_sha256": fingerprint(identity), "identity": identity}
    if args.output:
        args.output.write_text(json.dumps(document, indent=2) + "\n")
    print(json.dumps({"verified": True, "runtime_sha256": document["runtime_sha256"]}))


if __name__ == "__main__":
    main()
