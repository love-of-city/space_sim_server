"""Autonomy benchmark tooling; performance microbenchmarks remain tools/benchmarks.

At this implementation stage the command exposes validated requirements only.
It does not pretend that a catalogue entry is a registered simulation backend.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
from typing import get_args

SERVER_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SERVER_ROOT / "backend"))

from space_arm_platform.benchmark_catalog import (  # noqa: E402
    AutonomyCatalog, CATALOG_PATH, Family, load_catalog,
)


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--catalog", type=Path, default=CATALOG_PATH)
    commands = parser.add_subparsers(dest="command", required=True)
    listing = commands.add_parser("list", help="list requirements, not implemented environments")
    listing.add_argument("--priority", choices=["P0", "P1", "P2", "P3"])
    listing.add_argument("--family", choices=get_args(Family))
    listing.add_argument("--vision", choices=["V0", "V1", "V2", "V3"])
    listing.add_argument("--group", choices=list("SDMT"))
    listing.add_argument("--format", choices=["json", "markdown"], default="json")
    commands.add_parser("validate", help="validate the complete final source mapping")
    commands.add_parser("schema", help="print the requirement catalogue JSON Schema")
    args = parser.parse_args(argv)
    if args.command == "schema":
        print(json.dumps(AutonomyCatalog.model_json_schema(), ensure_ascii=False, indent=2))
        return 0
    try:
        catalog = load_catalog(args.catalog)
    except (OSError, ValueError) as error:
        print(f"invalid autonomy catalogue: {error}", file=sys.stderr)
        return 2
    if args.command == "validate":
        print(json.dumps(catalog.coverage(), ensure_ascii=False, indent=2))
        return 0
    selected = catalog.select(priority=args.priority, family=args.family,
                              vision=args.vision, source_group=args.group)
    if args.format == "json":
        print(json.dumps({"source_id": catalog.source.id,
                          "scope": "requirement_catalogue_only",
                          "count": len(selected),
                          "tasks": [t.model_dump(mode="json") for t in selected]},
                         ensure_ascii=False, indent=2))
    else:
        print("Requirement catalogue only; entries do not imply runtime readiness.\n")
        print("| Source | Stable ID | Task | Priority | Vision | Target |")
        print("| --- | --- | --- | --- | --- | --- |")
        for task in selected:
            print(f"| {task.source_id} | {task.id} | {task.title} | {task.priority} | "
                  f"{'/'.join(task.requested_vision_modes)} | {'/'.join(task.target_cooperation)} |")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
