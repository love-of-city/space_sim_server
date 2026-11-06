"""Autonomy benchmark tooling; performance microbenchmarks remain tools/benchmarks.

Requirements and executable configurations have separate listing commands.
Native execution runs in this dedicated CLI process; scoring is simulator-free.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
from typing import get_args

SERVER_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SERVER_ROOT / "backend"))
sys.path.insert(0, str(SERVER_ROOT))

from space_arm_platform.benchmark_catalog import (  # noqa: E402
    AutonomyCatalog, CATALOG_PATH, Family, load_catalog,
)
from space_arm_platform.benchmark_contract import (  # noqa: E402
    SPEC_DIRECTORY, ForceAction, NavigationObservation, WaypointTask, load_task,
)
from space_arm_platform.benchmark_attitude import AttitudeObservation, AttitudeTask, WheelTorqueAction  # noqa: E402
from space_arm_platform.benchmark_orbit import OrbitObservation, OrbitTask  # noqa: E402
from space_arm_platform.benchmark_navigation import GNSSMeasurements, NavigationEstimateAction, NavigationTask  # noqa: E402
from space_arm_platform.benchmark_navigation_recovery import NavigationRecoveryTask, NavigationRecoveryObservation  # noqa: E402
from space_arm_platform.benchmark_relative_flight import RelativeFlightTask  # noqa: E402
from space_arm_platform.benchmark_flyaround import FlyAroundTask  # noqa: E402
from space_arm_platform.benchmark_formation import FormationAction, FormationObservation, FormationTask  # noqa: E402
from space_arm_platform.benchmark_pointing import PointingTask, PointingObservation  # noqa: E402
from space_arm_platform.benchmark_vision import VisionPoseTask, VisionPoseObservation, VisionPoseAction  # noqa: E402
from space_arm_platform.benchmark_vision_motion import VisionMotionTask, VisionMotionObservation, VisionMotionAction  # noqa: E402
from space_arm_platform.benchmark_vision_approach import VisionApproachTask, VisionApproachObservation, VisionApproachAction  # noqa: E402
from space_arm_platform.benchmark_wheel_recovery import WheelRecoveryTask, WheelRecoveryObservation, WheelRecoveryAction  # noqa: E402
from space_arm_platform.benchmark_avoidance import AvoidanceTask, AvoidanceObservation  # noqa: E402
from space_arm_platform.benchmark_execution import rescore_episode, run_episode  # noqa: E402
from space_arm_platform.benchmark_external import PythonMethodSpec, MethodRequest, MethodResponse  # noqa: E402
from space_arm_platform.benchmark_registry import capability_matrix, method_ids  # noqa: E402


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
    schema = commands.add_parser("schema", help="print a versioned JSON Schema")
    schema.add_argument("--kind", choices=["catalogue", "waypoint", "observation", "action", "attitude", "attitude-observation", "wheel-action", "pointing", "pointing-observation", "vision-pose", "vision-observation", "vision-estimate", "vision-spin", "vision-spin-observation", "vision-spin-estimate", "vision-approach", "vision-approach-observation", "vision-approach-action", "wheel-recovery", "wheel-recovery-observation", "wheel-recovery-action", "debris-avoidance", "debris-observation", "orbit", "orbit-observation", "navigation-task", "navigation-recovery", "navigation-recovery-observation", "gnss-measurements", "navigation-estimate", "relative-flight", "flyaround", "formation", "formation-observation", "formation-action", "method", "method-request", "method-response"], default="catalogue")
    commands.add_parser("tasks", help="list registered executable configurations, separately from requirements")
    matrix = commands.add_parser("matrix", help="join all 122 requirements to registered configurations; not an acceptance report")
    matrix.add_argument("--format", choices=["json", "markdown"], default="json")
    spec = commands.add_parser("spec", help="print an executable task specification")
    spec.add_argument("configuration")
    run = commands.add_parser("run", help="run one native episode in a new output directory")
    run.add_argument("configuration")
    run_method = run.add_mutually_exclusive_group()
    run_method.add_argument("--method", choices=method_ids(), help="defaults to the task family's conventional baseline")
    run_method.add_argument("--method-spec", type=Path, help="trusted external Python method JSON specification")
    run.add_argument("--seed", type=int, required=True)
    run.add_argument("--output", type=Path, required=True)
    run.add_argument("--stop-file", type=Path, help="stop cooperatively when this control file appears")
    rescore = commands.add_parser("rescore", help="verify artifact hashes and independently replay physical evidence")
    rescore.add_argument("output", type=Path)
    batch = commands.add_parser("batch", help="run a frozen split in isolated processes; resume does not retry failures")
    batch.add_argument("configuration")
    batch_method = batch.add_mutually_exclusive_group()
    batch_method.add_argument("--method", choices=method_ids(), help="defaults to the task family's conventional baseline")
    batch_method.add_argument("--method-spec", type=Path, help="trusted external Python method JSON specification")
    batch.add_argument("--split", choices=["train", "validation", "test"], default="test")
    batch.add_argument("--output", type=Path, required=True)
    batch.add_argument("--resume", action="store_true")
    batch.add_argument("--stop-file", type=Path, help="stop the current attempt and leave unstarted seeds pending")
    batch.add_argument("--seed", type=int, help="one diagnostic trial from the selected frozen split")
    dataset = commands.add_parser("export-dataset", help="export verified public observations/actions; never relabel frozen splits")
    dataset.add_argument("episodes", type=Path, nargs="+", help="completed run directories with recorded public context")
    dataset.add_argument("--output", type=Path, required=True, help="new portable dataset directory")
    dataset.add_argument("--split", choices=["train", "validation", "test"], default="train")
    verify_data = commands.add_parser("verify-dataset", help="verify exported dataset hashes, schemas and temporal alignment")
    verify_data.add_argument("output", type=Path)
    args = parser.parse_args(argv)
    if args.command == "schema":
        model = {"catalogue": AutonomyCatalog, "waypoint": WaypointTask,
                 "observation": NavigationObservation, "action": ForceAction, "attitude": AttitudeTask,
                 "attitude-observation": AttitudeObservation, "wheel-action": WheelTorqueAction,
                 "pointing": PointingTask, "pointing-observation": PointingObservation,
                 "vision-pose": VisionPoseTask, "vision-observation": VisionPoseObservation, "vision-estimate": VisionPoseAction,
                 "vision-spin": VisionMotionTask, "vision-spin-observation": VisionMotionObservation, "vision-spin-estimate": VisionMotionAction,
                 "vision-approach": VisionApproachTask, "vision-approach-observation": VisionApproachObservation, "vision-approach-action": VisionApproachAction,
                 "wheel-recovery": WheelRecoveryTask, "wheel-recovery-observation": WheelRecoveryObservation, "wheel-recovery-action": WheelRecoveryAction,
                 "orbit": OrbitTask, "orbit-observation": OrbitObservation,
                 "debris-avoidance": AvoidanceTask, "debris-observation": AvoidanceObservation,
                 "navigation-task": NavigationTask, "gnss-measurements": GNSSMeasurements, "navigation-estimate": NavigationEstimateAction,
                 "navigation-recovery": NavigationRecoveryTask, "navigation-recovery-observation": NavigationRecoveryObservation,
                 "relative-flight": RelativeFlightTask, "flyaround": FlyAroundTask,
                 "formation": FormationTask, "formation-observation": FormationObservation, "formation-action": FormationAction,
                 "method": PythonMethodSpec, "method-request": MethodRequest, "method-response": MethodResponse}[args.kind]
        print(json.dumps(model.model_json_schema(), ensure_ascii=False, indent=2))
        return 0
    try:
        catalog = load_catalog(args.catalog)
    except (OSError, ValueError) as error:
        print(f"invalid autonomy catalogue: {error}", file=sys.stderr)
        return 2
    if args.command == "validate":
        print(json.dumps(catalog.coverage(), ensure_ascii=False, indent=2))
        return 0
    if args.command in {"tasks", "matrix", "spec", "run", "rescore", "batch", "export-dataset", "verify-dataset"}:
        try:
            if args.command in {"export-dataset", "verify-dataset"}:
                from space_arm_platform.benchmark_dataset import export_dataset, verify_dataset
                result = (export_dataset(args.episodes, args.output, split=args.split)
                          if args.command == "export-dataset" else verify_dataset(args.output))
                print(json.dumps(result, ensure_ascii=False, indent=2))
                return 0
            if args.command == "rescore":
                print(json.dumps(rescore_episode(args.output), ensure_ascii=False, indent=2))
                return 0
            registered = [load_task(path, catalog) for path in sorted(SPEC_DIRECTORY.glob("*.json"))]
            if args.command == "matrix":
                matrix = capability_matrix(catalog, registered)
                if args.format == "json":
                    print(json.dumps(matrix, ensure_ascii=False, indent=2))
                else:
                    print("Execution registration only; acceptance results require native evidence.\n")
                    print("| Source | Task | Priority | Requested vision | Registered configurations |")
                    print("| --- | --- | --- | --- | --- |")
                    for row in matrix["tasks"]:
                        configs = ", ".join(item["id"]+" ("+item["vision_mode"]+")" for item in row["configurations"]) or "Not registered"
                        print(f"| {row['source_id']} | {row['title']} | {row['priority']} | {'/'.join(row['requested_vision_modes'])} | {configs} |")
                return 0
            if args.command == "tasks":
                print(json.dumps({"scope": "registered_execution_configurations", "configurations": [
                    {"id": task.configuration_id, "requirement_id": task.requirement_id,
                     "source_id": task.source_id, "backend": task.backend, "vision_mode": task.vision_mode,
                     "target_cooperation": task.target_cooperation, "fingerprint": task.fingerprint(),
                     "methods": method_ids(task)}
                    for task in registered]}, ensure_ascii=False, indent=2))
                return 0
            matches = [task for task in registered if task.configuration_id == args.configuration]
            if len(matches) != 1:
                raise ValueError("configuration is not registered; use 'tasks' to list executable configurations")
            task = matches[0]
            if args.command == "spec":
                print(task.model_dump_json(indent=2))
                return 0
            if args.command == "batch":
                from space_arm_platform.benchmark_batch import run_batch
                result = run_batch(task, args.method, args.split, args.output, resume=args.resume,
                                   method_spec=args.method_spec, stop_file=args.stop_file, seed=args.seed)
                print(json.dumps(result, ensure_ascii=False, indent=2))
                return 0 if result["complete"] and result["successes"] == result["planned"] else 1
            result = run_episode(task, args.method, args.seed, args.output, method_spec=args.method_spec, stop_file=args.stop_file)
            print(json.dumps(result, ensure_ascii=False, indent=2))
            return 0 if result["success"] else {"infrastructure_error": 3, "cancelled": 130}.get(result["outcome"], 1)
        except (OSError, ValueError, KeyError) as error:
            print(f"benchmark command failed: {error}", file=sys.stderr)
            return 2
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
