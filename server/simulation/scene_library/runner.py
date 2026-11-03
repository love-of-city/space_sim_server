"""Execute production adapters with separate preparation and physical gates."""
import json
import os
from pathlib import Path
import subprocess
import sys
from datetime import datetime, timezone

from .manifest import digest, verify_files, write_json

ROOT = Path(__file__).resolve().parents[2]


def runtime_fingerprints():
    """Record configuration and native binaries separately from Python sources."""
    paths = {*ROOT.glob('config/**/*.json'), *ROOT.glob('run/native_acceleration/*.json')}
    override = os.environ.get('SPACE_SIM_PLUG_RETENTION_CONFIG')
    if override:
        paths.add(Path(override).resolve())
    for manifest in ROOT.glob('run/native_acceleration/*.json'):
        record = json.loads(manifest.read_text(encoding='utf-8-sig'))
        if record.get('library'):
            paths.add(ROOT / record['library'])
    return {str(path): digest(path) for path in sorted(paths)}


def run_batch(instances, output, *, python=None, geometry_python=None):
    """Run explicit instances in order; preserve failed cases and keep going."""
    if not instances:
        raise ValueError("Batch requires at least one instance")
    for instance in instances:
        verify_files(instance)
    output = Path(output).resolve()
    output.mkdir(parents=True, exist_ok=False)
    write_json(output / "batch-input.json", dict(schema="space-arm-scene-batch-input/1", instances=instances))
    runs = []
    for index, instance in enumerate(instances):
        folder = output / f"{index:03d}"
        try:
            receipt = run(instance, folder, python=python, geometry_python=geometry_python)
        except Exception as exc:
            receipt = dict(scene_id=instance["recipe"]["id"], accepted=False,
                           status="infrastructure_error", error=f"{type(exc).__name__}: {exc}")
        runs.append(dict(output=str(folder), **receipt))
    result = dict(schema="space-arm-scene-batch/1", runs=runs,
                  requested_scopes_passed=all(r.get("accepted") or r.get("operation_accepted") for r in runs),
                  complete_scenes_passed=all(r.get("accepted") for r in runs))
    write_json(output / "batch-result.json", result)
    return result


def run(instance, output, *, python=None, geometry_python=None):
    verify_files(instance)
    output = Path(output).resolve()
    output.mkdir(parents=True, exist_ok=False)
    write_json(output / "instance.json", instance)
    scene = instance["recipe"]
    operation_fixture = instance.get("operation_fixture")
    preheld = operation_fixture and operation_fixture.get('scope') == 'dual_arm_preheld_operation'
    if preheld and geometry_python is None:
        raise ValueError('Preheld operation requires a separate geometry Python for its independent holding audit')
    adapter = "task_box" if operation_fixture else scene["adapter"]
    receipt = dict(schema="space-arm-scene-run/1", scene_id=scene["id"],
                   started_utc=datetime.now(timezone.utc).isoformat(), accepted=False,
                   status="preparing", missing_capabilities=scene["missing_capabilities"],
                   evidence_scope="native_expert_headless", ue_visual_verified=False)
    if not adapter:
        receipt.update(status="unsupported", reason="required_capabilities_not_implemented")
        write_json(output / "receipt.json", receipt)
        return receipt
    interpreter = str(python or sys.executable)
    preparation = None
    if adapter in ('task_box', 'locking_rod'):
        write_json(output / "initial-scene.json", instance["initial_scene"])
        command = [interpreter, str(ROOT / "tools/diagnostics/validate_task_trajectory_contacts.py"),
                   "--model", instance["model"], "--scene", str(output / "initial-scene.json"),
                   "--task", scene["task"], "--output", str(output / "native"),
                   "--max-sim-seconds", "420", "--native-geometry", "--scene-expert"]
        report_path = output / "native/acceptance.json"
        if scene['task'] == 'locking_rod_cycle':
            preparation = [interpreter, str(ROOT / 'tools/diagnostics/prepare_locking_rod_path.py'),
                '--model', instance['model'], '--scene', str(output / 'initial-scene.json'),
                '--output', str(output / 'rod-preflight'), '--seed', str(instance['seed'])]
            command[command.index('--scene')+1] = str(output / 'rod-preflight/initial-scene.json')
            command += ['--rod-path', str(output / 'rod-preflight/path.json')]
            receipt['resolved_initial_scene'] = str(output / 'rod-preflight/initial-scene.json')
        if operation_fixture and operation_fixture["transport_clearance_m"] is not None:
            command += ["--transport-clearance", str(operation_fixture["transport_clearance_m"])]
        if operation_fixture and operation_fixture.get('seating_overtravel_m') is not None:
            command += ['--seating-overtravel', str(operation_fixture['seating_overtravel_m'])]
        if operation_fixture and operation_fixture.get('part_velocity_damping') is not None:
            command += ['--part-velocity-damping', str(operation_fixture['part_velocity_damping'])]
        if operation_fixture and operation_fixture.get('transfer_offset_xy_m') is not None:
            command += ['--transfer-offset-xy', *map(str,operation_fixture['transfer_offset_xy_m'])]
        if operation_fixture and operation_fixture.get('release_speed_rad_s') is not None:
            command += ['--release-speed', str(operation_fixture['release_speed_rad_s'])]
        if operation_fixture and operation_fixture.get('target_site') is not None:
            command += ['--target-site', operation_fixture['target_site']]
    else:
        # Flight adapter currently resolves its source model relative to this ROOT.
        if Path(instance["repository"]).resolve() != ROOT:
            raise ValueError("Flight source must be in the runtime repository")
        command = [interpreter, "-m", "simulation.orbital_rendezvous.run", "--scenario", "in-orbit",
                   "--seed", str(instance["seed"]), "--output-dir", str(output / "native")]
        report_path = output / "native/mission_report.json"
    sources = sorted({*ROOT.glob("simulation/**/*.py"), *ROOT.glob("backend/**/*.py"),
                      *ROOT.glob("tools/**/*.py"), *ROOT.glob("collection/**/*.py"), *ROOT.glob("native/**/*.cpp"), *ROOT.glob("native/**/*.h")})
    write_json(output / "launch.json", dict(argv=command, python=interpreter, cwd=str(ROOT),
               preparation_argv=preparation,
               runtime_configuration_and_binary_sha256=runtime_fingerprints(),
               source_sha256={str(p.relative_to(ROOT)): digest(p) for p in sources}))
    env = os.environ.copy()
    env["PYTHONPATH"] = os.pathsep.join([str(ROOT), str(ROOT / "backend"),
        str(ROOT.parent / "ue-adapter/Adapters"),
        str(ROOT.parent / "ue-adapter/Unreal/BskUnrealRenderer/examples"), env.get("PYTHONPATH", "")])
    try:
        if preparation:
            with (output / 'preparation.stdout.log').open('w', encoding='utf-8') as log, \
                    (output / 'preparation.stderr.log').open('w', encoding='utf-8') as error:
                prepared = subprocess.run(preparation, cwd=ROOT, env=env, stdout=log, stderr=error, check=False)
            if prepared.returncode != 0:
                receipt.update(status='preparation_failed', returncode=prepared.returncode,
                    reason='native_rod_geometry_preflight_failed', finished_utc=datetime.now(timezone.utc).isoformat())
                write_json(output / 'receipt.json', receipt)
                return receipt
        with (output / "stdout.log").open("w", encoding="utf-8") as stdout, \
                (output / "stderr.log").open("w", encoding="utf-8") as stderr:
            result = subprocess.run(command, cwd=ROOT, env=env, stdout=stdout, stderr=stderr, check=False)
        report = json.loads(report_path.read_text(encoding="utf-8-sig")) if report_path.is_file() else {}
        if adapter in ('task_box', 'locking_rod'):
            passed = report.get("accepted") is True and report.get("independent_evaluation", {}).get("success") is True
        else:
            checks = report.get("checks", {})
            passed = bool(checks) and all(value is True for value in checks.values())
            if result.returncode == 0 and passed:
                with (output / "independent-flight.log").open("w", encoding="utf-8") as log:
                    verification = subprocess.run(
                        [interpreter, str(ROOT / "tools/diagnostics/verify_team_flight.py"), str(output / "native")],
                        cwd=ROOT, env=env, stdout=log, stderr=subprocess.STDOUT, check=False)
                independent_path = output / "native/independent_validation.json"
                independent = json.loads(independent_path.read_text(encoding="utf-8")) if independent_path.is_file() else {}
                passed = verification.returncode == 0 and independent.get("passed") is True
                receipt["independent_validation"] = str(independent_path)
            receipt["evidence_scope"] = "far_field_rendezvous_only_not_physical_docking"
        receipt.update(returncode=result.returncode, native_report=str(report_path),
                       accepted=result.returncode == 0 and passed)
        receipt["status"] = "passed" if receipt["accepted"] else "failed"
        if operation_fixture:
            if preheld and result.returncode == 0 and passed:
                hold_path = output / 'preheld-capture-validation.json'
                resolved_scene = command[command.index('--scene') + 1]
                audit_command = [str(geometry_python), str(ROOT / 'tools/diagnostics/verify_preheld_capture.py'),
                    '--model', instance['model'], '--scene', resolved_scene,
                    '--native', str(output / 'native'), '--output', str(hold_path)]
                with (output / 'preheld-capture-validation.log').open('x', encoding='utf-8') as log:
                    audited = subprocess.run(audit_command, cwd=ROOT, env=env, stdout=log,
                                             stderr=subprocess.STDOUT, check=False)
                hold = json.loads(hold_path.read_text(encoding='utf-8')) if hold_path.is_file() else {}
                receipt['accepted'] = audited.returncode == 0 and hold.get('passed') is True
                receipt.update(preheld_capture_validation=str(hold_path), resolved_initial_scene=resolved_scene,
                    preheld_capture_validation_sha256=digest(hold_path) if hold_path.is_file() else None)
            receipt.update(operation_accepted=receipt["accepted"], accepted=False,
                           physical_docking_accepted=False,
                           evidence_scope=operation_fixture["scope"])
            receipt["status"] = "operation_passed" if receipt["operation_accepted"] else "operation_failed"
    except Exception as exc:
        receipt.update(status="infrastructure_error", error=f"{type(exc).__name__}: {exc}")
    receipt["finished_utc"] = datetime.now(timezone.utc).isoformat()
    write_json(output / "receipt.json", receipt)
    return receipt
