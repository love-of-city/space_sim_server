"""Prepare random layouts and UE assets serially, then publish a read-only pool.

Run with the simulation Python. Geometry/native MuJoCo subprocesses use the
separate --geometry-python to avoid loading it into the Basilisk process.
"""
from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import uuid

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / "backend")]
from space_arm_platform.collection_layouts import LayoutPool, pool_entry
from space_arm_platform.collection_processes import ManagerLease


def prepare(args):
    if args.count < 0 or args.count + len(args.existing) < 2 or args.seed < 0:
        raise ValueError("Require at least two layouts and a nonnegative seed/count")
    if not math.isfinite(args.yaw_range_deg) or not 0 <= args.yaw_range_deg <= 180:
        raise ValueError("yaw-range-deg must be in [0,180]")
    if args.count and args.geometry_python is None:
        raise ValueError("Generated layouts require --geometry-python")
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    entries = [pool_entry(f"existing-{i:03d}", *paths) for i, paths in enumerate(args.existing)]
    # A project can be shared by multiple workers, but only one preparation
    # process may write its imported asset registry at a time.
    lease = ManagerLease(args.adapter / "Unreal/BskUnrealRenderer/Saved/collection-layout-import.lock")
    env = dict(os.environ, OPENBLAS_NUM_THREADS="1", OMP_NUM_THREADS="4")

    def run(command, directory, stage):
        print(json.dumps(dict(stage=stage, directory=str(directory))), flush=True)
        with (directory / (stage + ".log")).open("w") as log:
            subprocess.run(command, cwd=ROOT, env=env, stdout=log, stderr=subprocess.STDOUT, check=True)

    try:
        for index in range(args.count):
            seed = args.seed + index
            directory = output / f"layout-{index:03d}-seed-{seed}"
            directory.mkdir()
            model = directory / "model/layout.xml"
            native = directory / "native-geometry.json"
            run([str(args.geometry_python), str(ROOT / "tools/build_independent_task_box_layout.py"),
                 "--source", str(args.source.resolve()), "--output", str(model), "--seed", str(seed),
                 "--target-mounts", "inset", "--pickup-mounts", "inset",
                 "--yaw-range-deg", str(args.yaw_range_deg)], directory, "build")
            run([str(args.geometry_python), str(ROOT / "tools/check_independent_task_box_layout.py"),
                 "--source", str(args.source.resolve()), "--layout", str(model), "--report", str(native)],
                directory, "native")
            run([str(args.geometry_python), str(ROOT / "tools/check_inset_target_geometry.py"),
                 "--layout", str(model), "--report", str(directory / "inset-geometry.json")], directory, "geometry")
            run([sys.executable, str(ROOT / "tools/prepare_independent_layout_catalog.py"),
                 "--layout", str(model), "--base-catalog", str(args.base_catalog.resolve()),
                 "--adapter", str(args.adapter.resolve()), "--unreal", str(args.unreal.resolve()),
                 "--output", str(directory / "ue-assets"),
                 "--destination", "/Game/BSK/Generated/Collection_" + uuid.uuid4().hex], directory, "ue")
            entries.append(pool_entry(f"seed-{seed}", model, directory / "ue-assets/layout.catalog.json", native))
        pool = output / "pool.json"
        temporary = output / "pool.tmp"
        temporary.write_text(json.dumps(dict(schema="collection-layout-pool/1", layouts=entries), indent=2) + "\n")
        LayoutPool(temporary)
        temporary.replace(pool)
        print(json.dumps(dict(stage="complete", pool=str(pool), layouts=len(entries))), flush=True)
    finally:
        lease.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--count", type=int, default=0, help="Number of new layouts to generate")
    parser.add_argument("--existing", type=Path, nargs=3, action="append", default=[],
                        metavar=("MODEL", "CATALOG", "NATIVE_REPORT"))
    parser.add_argument("--geometry-python", type=Path)
    parser.add_argument("--source", type=Path, default=ROOT / "model/SARM/platform/sarm_20261012.xml")
    parser.add_argument("--seed", type=int, default=2026102300)
    parser.add_argument("--yaw-range-deg", type=float, default=180)
    parser.add_argument("--adapter", type=Path, required=True)
    parser.add_argument("--unreal", type=Path, required=True)
    parser.add_argument("--base-catalog", type=Path, required=True)
    prepare(parser.parse_args())
