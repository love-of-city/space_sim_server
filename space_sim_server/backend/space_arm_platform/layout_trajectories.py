"""Bind sampled native task clearance to immutable layout-pool inputs."""
from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path

SCHEMA = "collection-layout-trajectories/1"
CONTACT_SCHEMA = "task-trajectory-contacts/2"
TASKS = ("drawer_cycle", "drawer_lift", "aviation_plug_insert",
         "guide_plug_insert", "cylindrical_plug_insert")
# Pin the production path contract independently of the observer's booleans.
# A failed plan can have complete sampling of its short, incomplete path.
REQUIRED_OPERATION_STAGES = {
    **{task: ("settle", "approach", "grip", "extract", "lift", "transfer", "above_hole",
              "insert", "release", "retract", "verify") for task in TASKS[2:]},
    "drawer_cycle": ("settle", "approach", "grip", "lift", "return", "release", "retract", "verify"),
    "drawer_lift": ("settle", "raise", "above_drawer", "align", "open", "descend", "grip", "lift", "hold"),
}
INITIAL_JOINTS_DEG = [90., -60., 60., 0., -90., 0.]
SERVER_ROOT = Path(__file__).resolve().parents[2]
OBSERVER = SERVER_ROOT / "tools/validate_task_trajectory_contacts.py"


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def reference(path):
    path = Path(path).resolve()
    return dict(path=str(path), sha256=digest(path))


def pinned(item):
    path = Path(item["path"])
    if not path.is_absolute() or digest(path) != item["sha256"]:
        raise ValueError(f"Changed trajectory evidence: {path}")
    return path


def document(item):
    return json.loads(pinned(item).read_text())


def validate_scene(scene_path):
    scene = json.loads(Path(scene_path).read_text(encoding="utf-8-sig"))
    from .collection_randomization import CollectionRandomization
    options = CollectionRandomization.model_validate(scene.get("collection_randomization", {}))
    joints = scene.get("randomization", {}).get("arm_joint_position_rad", [])
    if (scene.get("template_id") != "sarm-task-box-20261012" or len(joints) != 8
            or not all(math.isfinite(float(q)) for q in joints)
            or any(abs(float(q)-math.radians(want)) > 1e-9
                   for q, want in zip(joints[:6], INITIAL_JOINTS_DEG))
            or any(abs(float(q)) > 1e-9 for q in joints[6:])
            or any(options.flags().values()) or scene.get("randomize_orbit_phase", False)
            or scene.get("arm_preparation_required", False)):
        raise ValueError("Trajectory validation requires the fixed SARM scene, zero disturbances, and initial joints 90 -60 60 0 -90 0")
    return scene


def _case(task, item, model, manifest, scene):
    report, launch = document(item["report"]), document(item["launch"])
    if (report.get("schema") != CONTACT_SCHEMA or report.get("policy", {}).get("task") != task
            or report.get("policy", {}).get("ignore_free_bolt") is not True
            or report.get("coverage_complete") is not True or report.get("trajectory_clear") is not True
            or report.get("operational_path_complete") is not True
            or report.get("execution_error") is not None or report.get("native_returncode") not in (0, 1)
            or report.get("forbidden_contact_point_samples") != 0
            or not isinstance(report.get("task_success"), bool)
            or report["task_success"] != (report.get("task_result") == "success")):
        raise ValueError(f"Task {task} lacks complete, clear native trajectory evidence")
    observed = report.get("observed_stages")
    required = REQUIRED_OPERATION_STAGES[task]
    if (not isinstance(observed, list) or not all(isinstance(stage, str) for stage in observed)
            or len(set(observed)) != len(observed)
            or report.get("required_operation_stages") != list(required)
            or report.get("missing_operation_stages") != []
            or report.get("planning_failed") is not False
            or report.get("operation_stages_in_order") is not True
            or any(stage not in observed for stage in required)):
        raise ValueError(f"Task {task} lacks complete observed trajectory stages")
    positions = [observed.index(stage) for stage in required]
    if positions != sorted(positions):
        raise ValueError(f"Task {task} has out-of-order trajectory stages")
    coverage = report.get("coverage", {})
    samples, last = coverage.get("samples", 0), coverage.get("last_time_ns", -1)
    if (coverage.get("observed_hz") != 400 or coverage.get("expected_physics_hz") != 400
            or not isinstance(samples, int) or samples < 1 or not isinstance(last, int) or last < 0
            or coverage.get("first_time_ns") != 0 or last != coverage.get("expected_last_time_ns")
            or samples != last // 2_500_000 + 1 or samples != coverage.get("expected_samples")
            or coverage.get("max_interval_ns", 2_500_001) > 2_500_000
            or coverage.get("nonmonotonic_samples") != 0 or coverage.get("substeps") != [1]
            or coverage.get("invalid_rows") != 0):
        raise ValueError(f"Task {task} has incomplete 400 Hz coverage")
    sources = launch.get("source_sha256", {})
    for path in (model, manifest, scene):
        if sources.get(str(path)) != digest(path):
            raise ValueError(f"Task {task} trajectory belongs to different input {path}")
    argv = launch.get("argv", [])
    for flag, expected in (("--model", str(model)), ("--scene", str(scene)), ("--task", task)):
        if flag not in argv or argv[argv.index(flag)+1] != expected:
            raise ValueError(f"Task {task} launch does not match {flag}")
    if "--initial-joints-deg" in argv:
        start = argv.index("--initial-joints-deg") + 1
        if [float(v) for v in argv[start:start+6]] != INITIAL_JOINTS_DEG:
            raise ValueError(f"Task {task} used different initial joints")
    evidence = item.get("source_evidence", {})
    if set(evidence) != set(sources):
        raise ValueError(f"Task {task} lacks pinned execution sources")
    for source, sha in sources.items():
        resolved = pinned(evidence[source])
        if digest(resolved) != sha:
            raise ValueError(f"Task {task} has mismatched source evidence")
        # Only the observer may use an archived exact source snapshot. The
        # production runner/physics and concrete inputs must still match.
        if Path(source) != OBSERVER and (resolved != Path(source) or digest(source) != sha):
            raise ValueError(f"Task {task} used a different production source: {source}")
    for source in (OBSERVER, SERVER_ROOT / "tools/run_scripted_task.py",
                   SERVER_ROOT / "simulation/local_mujoco_stepper.py",
                   SERVER_ROOT / "simulation/scripted_tasks/native_state.py"):
        if str(source) not in sources:
            raise ValueError(f"Task {task} lacks execution source {source}")
    if report.get("analyzed_existing"):
        if digest(pinned(report["original_launch"])) != digest(pinned(item["launch"])):
            raise ValueError(f"Task {task} analysis is not bound to its original launch")
        analysis_sources = report.get("analysis_source_sha256", {})
        original = Path(report["analyzed_existing"])
        required_inputs = (model, OBSERVER, original / "contacts-400hz.jsonl.gz", original / "names.json",
                           original / "run/summary.json", original / "states-200hz.jsonl.gz")
        if not original.is_absolute() or not {str(p) for p in required_inputs}.issubset(analysis_sources):
            raise ValueError(f"Task {task} analysis lacks complete original trajectory inputs")
        if (analysis_sources.get(str(model)) != digest(model)
                or analysis_sources.get(str(OBSERVER)) != digest(pinned(item["analysis_observer"]))):
            raise ValueError(f"Task {task} analysis lacks matching model/observer provenance")
        for source, sha in analysis_sources.items():
            if Path(source) != OBSERVER and digest(source) != sha:
                raise ValueError(f"Task {task} has a changed trajectory analysis input: {source}")
    return report


def validate_trajectory_acceptance(binding, model, manifest):
    aggregate = document(binding)
    model, manifest = Path(model).resolve(), Path(manifest).resolve()
    if (aggregate.get("schema") != SCHEMA or aggregate.get("model_sha256") != digest(model)
            or aggregate.get("manifest_sha256") != digest(manifest)
            or aggregate.get("required_tasks") != list(TASKS)
            or aggregate.get("initial_joints_deg") != INITIAL_JOINTS_DEG
            or aggregate.get("coverage_complete") is not True
            or aggregate.get("trajectory_clear") is not True
            or aggregate.get("operational_path_complete") is not True):
        raise ValueError("Layout lacks matching complete trajectory acceptance; revalidate all five tasks and prepare a new pool")
    scene = pinned(aggregate["scene"])
    validate_scene(scene)
    tasks = aggregate.get("tasks", {})
    if set(tasks) != set(TASKS):
        raise ValueError("Layout trajectory coverage must include all five supported tasks")
    reports = {task: _case(task, tasks[task], model, manifest, scene) for task in TASKS}
    if aggregate.get("free_bolt_deferred") is not True:
        raise ValueError("Trajectory acceptance must explicitly defer the free bolt in the authorized scope")
    if (aggregate.get("observed_stages") != {task: r["observed_stages"] for task, r in reports.items()}
            or aggregate.get("required_operation_stages") != {
                task: r["required_operation_stages"] for task, r in reports.items()}
            or aggregate.get("deferred_bolt_contact_point_samples") != {
                task: r.get("categories", {}).get("deferred_bolt_contact", 0) for task, r in reports.items()}):
        raise ValueError("Aggregate trajectory stages or deferred bolt counts differ from the native reports")
    if aggregate.get("task_success") != all(r["task_success"] for r in reports.values()):
        raise ValueError("Aggregate task success differs from the native reports")
    return aggregate


def bind_trajectory_acceptance(model, native_report, scene, task_directories, output, *, source_snapshots=None):
    """Bind complete existing paths; final target verification may still fail."""
    model, native_report, scene = (Path(p).resolve() for p in (model, native_report, scene))
    manifest = model.with_suffix(".layout.json")
    validate_scene(scene)
    tasks = {}
    for task in TASKS:
        directory = Path(task_directories[task]).resolve()
        launch = json.loads((directory / "launch.json").read_text())
        evidence = {}
        for source, sha in launch["source_sha256"].items():
            path = Path(source)
            if digest(path) != sha and path == OBSERVER and source_snapshots is not None:
                path = next((p for p in Path(source_snapshots).glob("*.py") if digest(p) == sha), path)
            evidence[source] = reference(path)
        tasks[task] = dict(report=reference(directory / "acceptance.json"),
                           launch=reference(directory / "launch.json"), source_evidence=evidence)
        report = json.loads((directory / "acceptance.json").read_text())
        if report.get("analyzed_existing"):
            analysis_sha = report["analysis_source_sha256"][str(OBSERVER)]
            analysis_source = OBSERVER
            if digest(analysis_source) != analysis_sha and source_snapshots is not None:
                analysis_source = next((p for p in Path(source_snapshots).glob("*.py")
                                        if digest(p) == analysis_sha), analysis_source)
            tasks[task]["analysis_observer"] = reference(analysis_source)
    reports = {task: _case(task, tasks[task], model, manifest, scene) for task in TASKS}
    aggregate = dict(schema=SCHEMA, model_sha256=digest(model), manifest_sha256=digest(manifest),
                     scene=reference(scene), initial_joints_deg=INITIAL_JOINTS_DEG,
                     required_tasks=list(TASKS), tasks=tasks, coverage_complete=True, trajectory_clear=True,
                     operational_path_complete=True,
                     observed_stages={task: r["observed_stages"] for task, r in reports.items()},
                     required_operation_stages={task: r["required_operation_stages"] for task, r in reports.items()},
                     free_bolt_deferred=True,
                     deferred_bolt_contact_point_samples={
                         task: r.get("categories", {}).get("deferred_bolt_contact", 0)
                         for task, r in reports.items()},
                     task_success=all(r["task_success"] for r in reports.values()),
                     scope="All saved 400 Hz native ticks through every required operation stage; fixed baseline scene only; free bolt deferred; task success is a separate result")
    output = Path(output).resolve()
    if output.exists():
        raise ValueError("Use a fresh trajectory aggregate report path")
    output.write_text(json.dumps(aggregate, ensure_ascii=False, indent=2)+"\n")
    binding = reference(output)
    validate_trajectory_acceptance(binding, model, manifest)
    native = json.loads(native_report.read_text())
    if native.get("model_sha256") != digest(model):
        raise ValueError("Native geometry report belongs to another layout")
    native["trajectory_acceptance"] = binding
    native_report.write_text(json.dumps(native, ensure_ascii=False, indent=2)+"\n")
    return aggregate
