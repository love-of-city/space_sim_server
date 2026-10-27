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
from space_arm_platform.layout_trajectories import (
    INITIAL_JOINTS_DEG, TASKS, bind_trajectory_acceptance, validate_scene,
)


def validation_scene(source, output, seed):
    """Snapshot an explicit scene, or use the official fixed SARM template."""
    path = output / "validation-scene.json"
    if source is not None:
        validate_scene(source)
        path.write_bytes(Path(source).read_bytes())
    else:
        from space_arm_platform.collection_randomization import CollectionRandomization
        from space_arm_platform.models import SceneInstanceCreate
        from space_arm_platform.scene_runtime import _sample_instance
        request = SceneInstanceCreate(template_id="sarm-task-box-20261012",
                                      randomization_profile="teleop-zero-prepare-v1",
                                      operating_arm_joint_position_deg=INITIAL_JOINTS_DEG,
                                      randomize_orbit_phase=False, randomize_overview_camera=False)
        scene = _sample_instance(request, seed)
        scene.update(initialization_enabled=False, collection_layout_randomization=False,
                     collection_randomization=CollectionRandomization().model_dump(mode="json"))
        path.write_text(json.dumps(scene, ensure_ascii=False, indent=2)+"\n")
    validate_scene(path)
    return path


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
    scene = validation_scene(getattr(args, "validation_scene", None), output, args.seed) if args.count else None
    # A project can be shared by multiple workers, but only one preparation
    # process may write its imported asset registry at a time.
    lease = ManagerLease(args.adapter / "Unreal/BskUnrealRenderer/Saved/collection-layout-import.lock")
    env = dict(os.environ, OPENBLAS_NUM_THREADS="1", OMP_NUM_THREADS="1")
    env["PYTHONPATH"] = os.pathsep.join(filter(None, (
        str(args.adapter.resolve() / "Adapters"), env.get("PYTHONPATH", ""))))
    current_stage = {}

    def run(command, directory, stage, *, check=True):
        current_stage.update(stage=stage, directory=str(directory))
        print(json.dumps(dict(stage=stage, directory=str(directory))), flush=True)
        with (directory / (stage + ".log")).open("w") as log:
            return subprocess.run(command, cwd=ROOT, env=env, stdout=log, stderr=subprocess.STDOUT, check=check)

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
            trajectories = {}
            for task in TASKS:
                trajectories[task] = directory / ("trajectory-" + task)
                result = run([sys.executable, str(ROOT / "tools/validate_task_trajectory_contacts.py"),
                              "--model", str(model), "--scene", str(scene), "--task", task,
                              "--output", str(trajectories[task]), "--native-geometry", "--ignore-free-bolt",
                              "--initial-joints-deg", *map(str, INITIAL_JOINTS_DEG)],
                             directory, "trajectory-" + task, check=False)
                if result.returncode not in (0, 1):
                    raise RuntimeError(f"Native trajectory validation crashed for {task}")
            # Exit 1 can mean target stability failed despite clear contacts.
            # Require every operation stage and clear, complete sampling;
            # keep final target success separate.
            try:
                bind_trajectory_acceptance(model, native, scene, trajectories,
                                           directory / "trajectory-acceptance.json")
            except (OSError, ValueError, KeyError) as error:
                rejection = dict(schema="collection-layout-rejection/1", status="rejected",
                                 reason=str(error), model=str(model), scene=str(scene),
                                 pool_published=False, free_bolt_deferred=True, tasks={})
                for task, folder in trajectories.items():
                    path = folder / "acceptance.json"
                    report = json.loads(path.read_text()) if path.is_file() else {}
                    rejection["tasks"][task] = {k: report.get(k) for k in (
                        "task_success", "trajectory_clear", "coverage_complete", "operational_path_complete",
                        "observed_stages", "missing_operation_stages", "task_result", "task_reason")}
                    rejection["tasks"][task]["report"] = str(path)
                    rejection["tasks"][task]["deferred_bolt_contact_point_samples"] = report.get("categories", {}).get("deferred_bolt_contact", 0)
                for path in (directory / "trajectory-rejection.json", output / "rejection.json"):
                    path.write_text(json.dumps(rejection, ensure_ascii=False, indent=2)+"\n")
                raise ValueError(f"Layout rejected before UE import; no pool published: {error}") from error
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
    except Exception as error:
        rejection = output / "rejection.json"
        if not rejection.exists():
            rejection.write_text(json.dumps(dict(schema="collection-layout-rejection/1", status="rejected",
                                                pool_published=False, free_bolt_deferred=True,
                                                reason=f"{type(error).__name__}: {error}",
                                                **current_stage), ensure_ascii=False, indent=2)+"\n")
        raise
    finally:
        lease.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--count", type=int, default=0, help="Number of new layouts to generate")
    parser.add_argument("--existing", type=Path, nargs=3, action="append", default=[],
                        metavar=("MODEL", "CATALOG", "NATIVE_REPORT"))
    parser.add_argument("--geometry-python", type=Path)
    parser.add_argument("--validation-scene", type=Path,
                        help="Fixed, disturbance-free SARM scene; otherwise use the official template and fixed 90 -60 60 0 -90 0 pose")
    parser.add_argument("--source", type=Path, default=ROOT / "model/SARM/platform/sarm_20261012.xml")
    parser.add_argument("--seed", type=int, default=2026102300)
    parser.add_argument("--yaw-range-deg", type=float, default=0,
                        help="Optional independent yaw range in degrees; default 0 varies XY positions only")
    parser.add_argument("--adapter", type=Path, required=True)
    parser.add_argument("--unreal", type=Path, required=True)
    parser.add_argument("--base-catalog", type=Path, required=True)
    prepare(parser.parse_args())
