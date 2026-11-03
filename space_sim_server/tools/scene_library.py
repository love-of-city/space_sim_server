"""List, resolve, validate and run versioned scene recipes (no shell commands)."""
import argparse
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from simulation.scene_library.catalog import catalog
from simulation.scene_library.manifest import compose, verify_files, write_json


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    listing = commands.add_parser("list")
    listing.add_argument("--phase", type=int, choices=(1, 2))
    status = commands.add_parser("status")
    status.add_argument("output_root", type=Path)
    assets = commands.add_parser("assets")
    assets.add_argument("--repository", type=Path, default=ROOT)
    library = commands.add_parser('build-phase-one')
    library.add_argument('--repository', type=Path, default=ROOT)
    library.add_argument('--output', type=Path, required=True)
    library.add_argument('--geometry-python', type=Path, required=True)
    library.add_argument('--native-python', type=Path)
    library.add_argument('--scene-id', action='append')
    library.add_argument('--capture-reference', type=Path)
    library.add_argument('--arm-mount-offset', type=float, nargs=3, default=(.2, -.2, .25))
    run_library = commands.add_parser('run-library')
    run_library.add_argument('library', type=Path)
    run_library.add_argument('--output', type=Path, required=True)
    run_library.add_argument('--python', type=Path)
    run_library.add_argument('--geometry-python', type=Path)
    export = commands.add_parser('export-verified')
    export.add_argument('output_root', type=Path)
    export.add_argument('--output', type=Path, required=True)
    build = commands.add_parser("compose")
    build.add_argument("scene_id")
    build.add_argument("--repository", type=Path, default=ROOT)
    build.add_argument("--seed", type=int, default=42)
    build.add_argument("--output", type=Path, required=True)
    operation = commands.add_parser("compose-mounted-operation")
    operation.add_argument("scene_id")
    operation.add_argument("--repository", type=Path, default=ROOT)
    operation.add_argument("--assembly", type=Path, required=True)
    operation.add_argument("--initial-scene", type=Path, required=True)
    operation.add_argument("--mount-inspection", type=Path, required=True)
    operation.add_argument("--transport-clearance", type=float)
    operation.add_argument('--part-velocity-damping', type=float)
    operation.add_argument('--transfer-offset-xy', type=float, nargs=2)
    operation.add_argument('--release-speed', type=float)
    operation.add_argument("--output", type=Path, required=True)
    preheld = commands.add_parser('compose-preheld-operation')
    preheld.add_argument('scene_id')
    preheld.add_argument('--repository', type=Path, default=ROOT)
    preheld.add_argument('--assembly', type=Path, required=True)
    preheld.add_argument('--initial-scene', type=Path, required=True)
    preheld.add_argument('--mount-inspection', type=Path, required=True)
    preheld.add_argument('--transport-clearance', type=float)
    preheld.add_argument('--seating-overtravel', type=float)
    preheld.add_argument('--release-speed', type=float)
    preheld.add_argument('--target-site')
    preheld.add_argument('--output', type=Path, required=True)
    batch = commands.add_parser("batch")
    batch.add_argument("instances", type=Path, nargs="+")
    batch.add_argument("--output", type=Path, required=True)
    batch.add_argument("--python", type=Path)
    batch.add_argument('--geometry-python', type=Path)
    fixture = commands.add_parser("build-mounted-fixture")
    fixture.add_argument("--model", type=Path, default=ROOT / "model/SARM/platform/sarm_20261012.xml")
    fixture.add_argument("--scale", type=float, default=1.5)
    fixture.add_argument("--deck-raise", type=float, default=.45)
    fixture.add_argument("--cross-transfer", action="store_true")
    fixture.add_argument("--arm-mount-offset", type=float, nargs=3)
    fixture.add_argument("--receiver-shift-xy", type=float, nargs=2, default=(0., 0.))
    fixture.add_argument("--output", type=Path, required=True)
    for command in ("validate", "run"):
        sub = commands.add_parser(command)
        sub.add_argument("instance", type=Path)
        if command == "run":
            sub.add_argument("--output", type=Path, required=True)
            sub.add_argument("--python", type=Path)
            sub.add_argument('--geometry-python', type=Path)
    args = parser.parse_args(argv)
    if args.command == "list":
        document = catalog()
        if args.phase:
            document["scenes"] = [s for s in document["scenes"] if s["phase"] == args.phase]
    elif args.command == "assets":
        from simulation.scene_library.assets import inventory
        document = inventory(args.repository)
    elif args.command == "status":
        from simulation.scene_library.status import summarize
        document = summarize(args.output_root)
    elif args.command == 'build-phase-one':
        from simulation.scene_library.materialize import materialize
        document = materialize(args.repository, args.output, geometry_python=args.geometry_python,
            native_python=args.native_python, scene_ids=args.scene_id,
            capture_reference=args.capture_reference, arm_mount_offset=args.arm_mount_offset)
        document = dict(library=str(args.output / 'library.json'), status=document['status'],
                        scenes=len(document['entries']), task_accepted=False)
    elif args.command == 'export-verified':
        from simulation.scene_library.materialize import export_verified
        document = export_verified(args.output_root, args.output)
        document = dict(library=str(args.output / 'library.json'), scenes=len(document['entries']),
                        phase_one_accepted=document['phase_one_accepted'])
    elif args.command == 'run-library':
        from simulation.scene_library.materialize import library_instances
        from simulation.scene_library.runner import run_batch
        document = run_batch(library_instances(args.library), args.output, python=args.python,
                             geometry_python=args.geometry_python)
    elif args.command == "batch":
        from simulation.scene_library.runner import run_batch
        instances = [json.loads(p.read_text(encoding="utf-8-sig")) for p in args.instances]
        document = run_batch(instances, args.output, python=args.python, geometry_python=args.geometry_python)
    elif args.command == "build-mounted-fixture":
        from simulation.scene_library.assembly import mounted_target_fixture
        document = mounted_target_fixture(args.model, args.output, scale=args.scale,
                                          deck_raise_m=args.deck_raise, cross_transfer=args.cross_transfer,
                                          arm_mount_offset_m=args.arm_mount_offset,
                                          receiver_shift_xy_m=args.receiver_shift_xy)
        document = {key: document[key] for key in ("model", "assembly_status", "physical_docking_accepted", "task_accepted")}
    elif args.command in ("compose", "compose-mounted-operation", 'compose-preheld-operation'):
        if args.command == "compose":
            document = compose(args.scene_id, args.repository, seed=args.seed)
        elif args.command == 'compose-preheld-operation':
            from simulation.scene_library.manifest import compose_preheld_operation
            document = compose_preheld_operation(args.scene_id, args.repository, args.assembly,
                args.initial_scene, mount_inspection=args.mount_inspection,
                transport_clearance_m=args.transport_clearance, seating_overtravel_m=args.seating_overtravel,
                release_speed_rad_s=args.release_speed, target_site=args.target_site)
        else:
            from simulation.scene_library.manifest import compose_mounted_operation
            document = compose_mounted_operation(args.scene_id, args.repository,
                args.assembly, args.initial_scene, transport_clearance_m=args.transport_clearance,
                mount_inspection=args.mount_inspection, part_velocity_damping=args.part_velocity_damping,
                transfer_offset_xy_m=args.transfer_offset_xy, release_speed_rad_s=args.release_speed)
        args.output.parent.mkdir(parents=True, exist_ok=True)
        write_json(args.output, document)
        document = dict(instance=str(args.output), assembly_status=document["assembly_status"],
                        missing_capabilities=document["recipe"]["missing_capabilities"])
    else:
        document = json.loads(args.instance.read_text(encoding="utf-8-sig"))
        verify_files(document)
        if args.command == "run":
            from simulation.scene_library.runner import run
            document = run(document, args.output, python=args.python, geometry_python=args.geometry_python)
        else:
            document = dict(files_verified=True, task_accepted=False,
                            runnable=document["recipe"]["adapter"] is not None or bool(document.get("operation_fixture")))
    print(json.dumps(document, ensure_ascii=False, indent=2))
    if args.command in ('batch', 'run-library'):
        return 0 if document["requested_scopes_passed"] else 1
    return 1 if args.command == "run" and not (document["accepted"] or document.get("operation_accepted")) else 0


if __name__ == "__main__":
    sys.stdout.reconfigure(encoding="utf-8")
    raise SystemExit(main())
