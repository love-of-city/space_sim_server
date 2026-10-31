"""Write UE build provenance after a successful full build, or verify it."""
from __future__ import annotations

import argparse
import importlib.util
import json
from pathlib import Path


def load_identity_module():
    path = Path(__file__).resolve().parents[1] / "Adapters/bsk_render_adapter/build_identity.py"
    spec = importlib.util.spec_from_file_location("spacesim_build_identity", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--project", required=True, type=Path)
    parser.add_argument("--engine", required=True, type=Path)
    parser.add_argument("--platform", choices=("Linux", "Win64"), required=True)
    parser.add_argument("--target", default="BskUnrealRendererEditor")
    parser.add_argument("--configuration", default="Development", choices=("Development", "DebugGame", "Shipping"))
    parser.add_argument("--output", type=Path)
    parser.add_argument("--toolchain-json", type=Path)
    parser.add_argument("--verify", action="store_true")
    args = parser.parse_args(argv)
    identity = load_identity_module()
    output = args.output or identity.default_manifest_path(args.project, args.platform, args.target, args.configuration)
    if args.verify:
        document = identity.verify_build_manifest(args.project, args.engine, output)
        print(json.dumps({"manifest": str(output), "build_id": document["build_id"], "verified": True}))
    else:
        toolchain = json.loads(args.toolchain_json.read_text()) if args.toolchain_json else None
        result = identity.write_build_manifest(args.project, args.engine, args.platform, args.target,
                                               args.configuration, toolchain, output)
        print(result)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
