"""Layout selection must bind preflight, capture and accepted episode metadata."""
import copy
import json
import math
from pathlib import Path
from types import SimpleNamespace

import pytest

from space_arm_platform.collection_layouts import (
    OPERATION_CLEARANCE_SCHEMA, LayoutPool, attach_layout, digest, pool_entry, validate_episode_layout,
)
from simulation.independent_task_box_layout import (
    MODULES, PICKUPS, TARGETS, make_operational_clearance, sample_placements,
)
from simulation.scripted_tasks.drawer_initialization import randomized_scene
from space_arm_platform.layout_trajectories import (
    INITIAL_JOINTS_DEG, OBSERVER, REQUIRED_OPERATION_STAGES, SERVER_ROOT, TASKS,
    bind_trajectory_acceptance, reference,
)


def trajectory_evidence(directory, model, native, *, successful=True):
    scene = directory / "trajectory-scene.json"
    scene.write_text(json.dumps(dict(template_id="sarm-task-box-20261012",
                                    randomization=dict(arm_joint_position_rad=[
                                        *map(math.radians, INITIAL_JOINTS_DEG), 0., 0.]))))
    sources = [model, model.with_suffix(".layout.json"), scene, OBSERVER,
               SERVER_ROOT / "tools/run_scripted_task.py",
               SERVER_ROOT / "simulation/local_mujoco_stepper.py",
               SERVER_ROOT / "simulation/scripted_tasks/native_state.py"]
    directories = {}
    for task in TASKS:
        folder = directory / task
        folder.mkdir(exist_ok=True)
        directories[task] = folder
        (folder / "launch.json").write_text(json.dumps(dict(
            argv=["runner", "--model", str(model), "--scene", str(scene), "--task", task],
            source_sha256={str(p): digest(p) for p in sources})))
        (folder / "acceptance.json").write_text(json.dumps(dict(
            schema="task-trajectory-contacts/2", task_success=successful,
            task_result="success" if successful else "failed", native_returncode=0 if successful else 1,
            coverage_complete=True, trajectory_clear=True, execution_error=None,
            operational_path_complete=True, observed_stages=list(REQUIRED_OPERATION_STAGES[task]),
            required_operation_stages=list(REQUIRED_OPERATION_STAGES[task]),
            missing_operation_stages=[], planning_failed=False, operation_stages_in_order=True,
            categories=dict(deferred_bolt_contact=7),
            forbidden_contact_point_samples=0, policy=dict(task=task, ignore_free_bolt=True),
            coverage=dict(observed_hz=400, expected_physics_hz=400, samples=3, expected_samples=3,
                          first_time_ns=0, last_time_ns=5_000_000, expected_last_time_ns=5_000_000,
                          max_interval_ns=2_500_000, nonmonotonic_samples=0, substeps=[1], invalid_rows=0))))
    return scene, directories


@pytest.fixture
def layout_pool(tmp_path):
    entries = []
    # Tiny, disjoint envelopes exercise the actual manifest validator; physics
    # and renderer integration are checked separately with the real SARM model.
    footprints = {name: [[.01 + .03*i, .01], [.02 + .03*i, .02]]
                  for i, name in enumerate(MODULES)}
    anchors = {name: [(bounds[0][0]+bounds[1][0])/2,
                      (bounds[0][1]+bounds[1][1])/2, 0.]
               for name, bounds in footprints.items()}
    for i in range(2):
        directory = tmp_path / f"layout-{i}"
        directory.mkdir()
        model, catalog, native = (directory / name for name in ("layout.xml", "catalog.json", "native.json"))
        model.write_text(f'<mujoco model="layout-{i}"/>')
        visuals, assets = [], {}
        for name in (*PICKUPS, *TARGETS, "task_box_base"):
            file = directory / (name + ".obj")
            file.write_text(f"{i}-{name}")
            visuals.append(dict(name=name, file=str(file), sha256=digest(file)))
            assets[str(file)] = dict(source_sha256=digest(file))
        offsets, _ = sample_placements(footprints, i, operation_anchors=anchors)
        record = dict(schema="task-box-layout/2", seed=i, model_sha256=digest(model), source_sha256="original",
                      initial_states_randomized=False, orientations_randomized=False,
                      adapted_base=dict(visual=visuals[-1]["file"]), generated_visuals=visuals,
                      offsets_body_xy_m=offsets, original_footprints_body_xy_m=footprints,
                      modules={name: dict(original_anchor_body_m=anchor)
                               for name, anchor in anchors.items()},
                      operational_clearance=make_operational_clearance(footprints, anchors),
                      fixed_obstacles_body_xy_m=[], deck_edge_margin_m=.005, minimum_module_gap_m=.008)
        model.with_suffix(".layout.json").write_text(json.dumps(record))
        catalog.write_text(json.dumps(dict(source_mjcf=str(model), assets=assets)))
        native.write_text(json.dumps(dict(passed=True, initial_states_preserved=True,
                                         operation_clearance_schema=OPERATION_CLEARANCE_SCHEMA,
                                         model_sha256=digest(model), source_model_sha256="original")))
        scene, trajectories = trajectory_evidence(directory, model, native)
        bind_trajectory_acceptance(model, native, scene, trajectories, directory / "trajectory-acceptance.json")
        entries.append(pool_entry(f"id-{i}", model, catalog, native))
    path = tmp_path / "pool.json"
    path.write_text(json.dumps(dict(schema="collection-layout-pool/1", layouts=entries)))
    return path


def test_selection_reproduces_and_does_not_change_arm_noise_or_orbit(layout_pool):
    pool = LayoutPool(layout_pool)
    source = dict(randomization=dict(arm_joint_position_rad=[0.]*8),
                  randomize_orbit_phase=True, collection_randomization=dict(joint_bias=dict(enabled=True)))
    first = randomized_scene(source, 77)
    untouched = copy.deepcopy(first)
    model, catalog = attach_layout(first, pool, 77)
    replay = randomized_scene(source, 77)
    assert attach_layout(replay, pool, 77) == (model, catalog)
    assert replay == first
    assert first["randomization"] == untouched["randomization"]
    assert first["environment"] == untouched["environment"]
    assert first["collection_config"]["realized_randomization"] == untouched["collection_config"]["realized_randomization"]
    assert len({pool.select(seed)[3]["layout_id"] for seed in range(100)}) == 2
    assert all(not pool.select(seed)[3]["initial_states_randomized"] for seed in range(2))
    for seed in range(2):
        selection = pool.select(seed)[3]
        assert selection["operation_clearance_schema"] == OPERATION_CLEARANCE_SCHEMA
        assert selection["native_report_sha256"] == pool.document["layouts"][selection["pool_index"]]["native_report_sha256"]
        assert selection["trajectory_operational_path_complete"] is True


def test_clear_trajectory_with_failed_target_is_not_a_success_demonstration(layout_pool):
    entry = json.loads(layout_pool.read_text())["layouts"][0]
    directory, model, native = Path(entry["model"]).parent, Path(entry["model"]), Path(entry["native_report"])
    scene, trajectories = trajectory_evidence(directory, model, native, successful=False)
    aggregate = bind_trajectory_acceptance(model, native, scene, trajectories, directory / "failed-target.json")
    assert aggregate["trajectory_clear"] is True and aggregate["task_success"] is False
    assert pool_entry("clear-failed-target", model, entry["catalog"], native)


@pytest.mark.parametrize("changed", ["missing_binding", "missing_task", "forbidden_contact", "incomplete_coverage", "wrong_model", "strict_free_bolt", "incomplete_path", "missing_stage", "wrong_stage_order", "shortened_contract", "planning_failed"])
def test_native_trajectory_gate_fails_closed(layout_pool, changed):
    entry = json.loads(layout_pool.read_text())["layouts"][0]
    model, native = Path(entry["model"]), Path(entry["native_report"])
    report = json.loads(native.read_text())
    path = Path(report["trajectory_acceptance"]["path"])
    aggregate = json.loads(path.read_text())
    if changed == "missing_binding":
        del report["trajectory_acceptance"]
    elif changed == "missing_task":
        del aggregate["tasks"][TASKS[-1]]
    elif changed == "wrong_model":
        aggregate["model_sha256"] = "wrong"
    else:
        case = aggregate["tasks"][TASKS[0]]
        acceptance_path = Path(case["report"]["path"])
        acceptance = json.loads(acceptance_path.read_text())
        if changed == "forbidden_contact":
            acceptance["forbidden_contact_point_samples"] = 1
        elif changed == "strict_free_bolt":
            acceptance["policy"]["ignore_free_bolt"] = False
        elif changed == "incomplete_path":
            acceptance["operational_path_complete"] = False
        elif changed == "missing_stage":
            # A stale true flag cannot certify unexecuted stages.
            acceptance["observed_stages"].remove("retract")
        elif changed == "wrong_stage_order":
            acceptance["observed_stages"] = list(reversed(acceptance["observed_stages"]))
        elif changed == "shortened_contract":
            acceptance["required_operation_stages"] = ["settle"]
        elif changed == "planning_failed":
            acceptance["planning_failed"] = True
        else:
            # A stale true flag cannot conceal missing 400 Hz ticks.
            acceptance["coverage"]["samples"] = 2
        acceptance_path.write_text(json.dumps(acceptance))
        case["report"]["sha256"] = digest(acceptance_path)
    path.write_text(json.dumps(aggregate))
    if changed != "missing_binding":
        report["trajectory_acceptance"]["sha256"] = digest(path)
    native.write_text(json.dumps(report))
    with pytest.raises(ValueError, match="trajectory|coverage"):
        pool_entry("invalid-trajectory", model, entry["catalog"], native)


@pytest.mark.parametrize("changed", ["observed_stages", "deferred_bolt_contact_point_samples", "operational_path_complete"])
def test_aggregate_cannot_hide_native_path_or_deferred_contacts(layout_pool, changed):
    entry = json.loads(layout_pool.read_text())["layouts"][0]
    native = Path(entry["native_report"])
    report = json.loads(native.read_text())
    path = Path(report["trajectory_acceptance"]["path"])
    aggregate = json.loads(path.read_text())
    if changed == "observed_stages":
        aggregate[changed][TASKS[0]] = ["settle"]
    elif changed == "deferred_bolt_contact_point_samples":
        aggregate[changed][TASKS[0]] = 0
    else:
        aggregate[changed] = False
    path.write_text(json.dumps(aggregate))
    report["trajectory_acceptance"] = reference(path)
    native.write_text(json.dumps(report))
    with pytest.raises(ValueError, match="trajectory|stages|bolt"):
        pool_entry("invalid-aggregate", entry["model"], entry["catalog"], native)


def test_reanalysis_binds_original_launch_and_exact_archived_observer(layout_pool):
    entry = json.loads(layout_pool.read_text())["layouts"][0]
    model, native = Path(entry["model"]), Path(entry["native_report"])
    directory = model.parent
    scene, trajectories = trajectory_evidence(directory, model, native)
    snapshots = directory / "snapshots"
    snapshots.mkdir()
    archived = snapshots / "observer.py"
    archived.write_bytes(OBSERVER.read_bytes() + b"\n# Earlier observer snapshot.\n")
    folder = trajectories[TASKS[0]]
    launch_path = folder / "launch.json"
    launch = json.loads(launch_path.read_text())
    launch["source_sha256"][str(OBSERVER)] = digest(archived)
    launch_path.write_text(json.dumps(launch))
    original = folder / "original-launch.json"
    original.write_bytes(launch_path.read_bytes())
    report_path = folder / "acceptance.json"
    report = json.loads(report_path.read_text())
    original_inputs = [folder / name for name in (
        "contacts-400hz.jsonl.gz", "names.json", "run/summary.json", "states-200hz.jsonl.gz")]
    for path in original_inputs:
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(b"original trajectory evidence")
    report.update(analyzed_existing=str(folder), original_launch=reference(original),
                  analysis_source_sha256={str(p): digest(p) for p in (model, OBSERVER, *original_inputs)})
    report_path.write_text(json.dumps(report))
    aggregate = bind_trajectory_acceptance(model, native, scene, trajectories,
                                           directory / "reanalysis.json", source_snapshots=snapshots)
    assert aggregate["tasks"][TASKS[0]]["source_evidence"][str(OBSERVER)] == reference(archived)
    assert aggregate["deferred_bolt_contact_point_samples"][TASKS[0]] == 7
    assert pool_entry("reanalysis", model, entry["catalog"], native)
    original_inputs[0].write_bytes(b"changed trajectory evidence")
    with pytest.raises(ValueError, match="changed trajectory analysis input"):
        pool_entry("changed-input", model, entry["catalog"], native)
    original_inputs[0].write_bytes(b"original trajectory evidence")
    # Even rehashing the report cannot pair a reanalysis with another launch.
    original.write_text(json.dumps(dict(argv=["different-run"])))
    report["original_launch"] = reference(original)
    report_path.write_text(json.dumps(report))
    aggregate["tasks"][TASKS[0]]["report"] = reference(report_path)
    aggregate_path = directory / "reanalysis.json"
    aggregate_path.write_text(json.dumps(aggregate))
    native_report = json.loads(native.read_text())
    native_report["trajectory_acceptance"] = reference(aggregate_path)
    native.write_text(json.dumps(native_report))
    with pytest.raises(ValueError, match="original launch"):
        pool_entry("invalid-reanalysis", model, entry["catalog"], native)


@pytest.mark.parametrize("changed", ["missing_manifest_policy", "old_manifest_policy", "missing_native_policy", "old_native_policy"])
def test_old_layout_acceptance_is_rejected_even_with_matching_file_hashes(layout_pool, changed):
    document = json.loads(layout_pool.read_text())
    entry = document["layouts"][0]
    manifest = Path(entry["model"]).with_suffix(".layout.json")
    if changed.endswith("manifest_policy"):
        path, hash_key = manifest, "manifest_sha256"
        record = json.loads(path.read_text())
        if changed.startswith("missing"):
            del record["operational_clearance"]
        else:
            record["operational_clearance"]["schema"] = "task-box-operation-clearance/0"
    else:
        path, hash_key = Path(entry["native_report"]), "native_report_sha256"
        record = json.loads(path.read_text())
        if changed.startswith("missing"):
            del record["operation_clearance_schema"]
        else:
            record["operation_clearance_schema"] = "task-box-operation-clearance/0"
    path.write_text(json.dumps(record))
    entry[hash_key] = digest(path)
    layout_pool.write_text(json.dumps(document))
    with pytest.raises(ValueError, match="regenerate layouts and prepare a new pool"):
        LayoutPool(layout_pool)
    with pytest.raises(ValueError, match="regenerate layouts and prepare a new pool"):
        pool_entry("old-layout", entry["model"], entry["catalog"], entry["native_report"])


@pytest.mark.parametrize("changed", ["model", "catalog", "native_report", "manifest", "visual"])
def test_asset_mutation_fails_closed_even_after_pool_load(layout_pool, changed):
    pool = LayoutPool(layout_pool)
    entry = pool.document["layouts"][0]
    seed = next(seed for seed in range(100) if pool.select(seed)[3]["pool_index"] == 0)
    if changed == "manifest":
        path = Path(entry["model"]).with_suffix(".layout.json")
    elif changed == "visual":
        path = Path(json.loads(Path(entry["model"]).with_suffix(".layout.json").read_text())["generated_visuals"][0]["file"])
    else:
        path = Path(entry[changed])
    path.write_text("changed")
    with pytest.raises((ValueError, KeyError)):
        pool.select(seed)
    with pytest.raises((ValueError, KeyError)):
        LayoutPool(layout_pool)


def test_requires_two_distinct_layouts_and_valid_native_acceptance(layout_pool):
    document = json.loads(layout_pool.read_text())
    document["layouts"] = [document["layouts"][0]] * 2
    layout_pool.write_text(json.dumps(document))
    with pytest.raises(ValueError, match="unique"):
        LayoutPool(layout_pool)
    entry = document["layouts"][0]
    native = Path(entry["native_report"])
    report = json.loads(native.read_text())
    report["passed"] = False
    native.write_text(json.dumps(report))
    with pytest.raises(ValueError, match="native geometry"):
        pool_entry("bad", entry["model"], entry["catalog"], native)


@pytest.mark.parametrize("mismatched_recording", [False, True])
def test_collector_passes_same_layout_to_probe_renderer_and_recording(layout_pool, tmp_path, monkeypatch, mismatched_recording):
    import tools.collect_scripted_tasks as collector
    import tools.record_drawer_task as recorder

    scene = tmp_path / "scene.json"
    scene.write_text(json.dumps(dict(randomization=dict(arm_joint_position_rad=[0.]*8),
                                    collection_layout_randomization=True)))
    output, episodes = tmp_path / "batch", tmp_path / "episodes"
    episodes.mkdir()
    calls, probes = [], []

    class Probe:
        def __init__(self, model):
            self.model = model
        def validate(self, model, scene, task):
            assert self.model == model
            assert digest(model) == scene["task_box_layout"]["model_sha256"]
            probes.append(model)
            return dict(accepted=True)
        def close(self):
            pass

    def popen(command, **kwargs):
        calls.append(command)
        arg = lambda key: Path(command[command.index(key)+1])
        capture_scene = json.loads(arg("--scene").read_text())
        model, catalog = arg("--model"), arg("--catalog")
        assert probes[-1] == model
        assert json.loads(catalog.read_text())["source_mjcf"] == str(model)
        assert digest(model) == capture_scene["task_box_layout"]["model_sha256"]
        if mismatched_recording:
            capture_scene["collection_config"]["layout_randomization"]["layout_id"] = "wrong"
        folder = arg("--output")
        folder.mkdir()
        key = f"episode-{len(calls)}"
        (folder / "summary.json").write_text(json.dumps(dict(episode_id=key, task=dict(result="success"))))
        episode = episodes / key
        episode.mkdir()
        (episode / "metadata.json").write_text(json.dumps(dict(scene_instance=capture_scene)))
        return SimpleNamespace(poll=lambda: 0, returncode=0)

    monkeypatch.setattr(collector, "NativePoseProbe", Probe)
    monkeypatch.setattr(collector, "check_port_available", lambda _: None)
    monkeypatch.setattr(recorder, "recording_settings", lambda _: None)
    monkeypatch.setattr(collector.subprocess, "Popen", popen)
    monkeypatch.setattr(collector, "validate_recording", lambda *_: dict(passed=True))
    monkeypatch.setattr("sys.argv", ["collect", "--scene", str(scene), "--output", str(output),
                                   "--episodes-root", str(episodes), "--layout-pool", str(layout_pool),
                                   "--count", "2", "--max-attempts", "2", "--seed", "1"])
    if mismatched_recording:
        with pytest.raises(ValueError, match="different layout"):
            collector.main()
        report = json.loads((output / "summary.json").read_text())
        assert report["status"] == "failed" and report["accepted_episodes"] == []
        assert report["attempts"][0]["status"] == "validation_failed"
        return
    assert collector.main() == 0
    report = json.loads((output / "summary.json").read_text())
    assert report["status"] == "complete" and len(report["accepted_episodes"]) == 2
    for attempt, episode in zip(report["attempts"], report["accepted_episodes"]):
        validate_episode_layout(episode, attempt["layout_randomization"])
        bad = copy.deepcopy(attempt["layout_randomization"])
        bad["model_sha256"] = "wrong"
        with pytest.raises(ValueError, match="different layout"):
            validate_episode_layout(episode, bad)


def test_api_manager_requires_pool_and_snapshots_it(layout_pool, tmp_path, monkeypatch):
    from space_arm_platform.collection_jobs import CollectionCreate, CollectionManager

    manager = CollectionManager(tmp_path, tmp_path / "episodes", tmp_path, tmp_path)
    request = CollectionCreate(workers=1, count=1, max_attempts=1, layout_randomization=True)
    try:
        with pytest.raises(ValueError, match="布局池"):
            manager.start(request, {}, {})
        manager.layout_pool = layout_pool
        (tmp_path / "tools").mkdir()
        (tmp_path / "tools/collect_drawer_cycles.py").touch()
        base = tmp_path / "Unreal/BskUnrealRenderer/Saved/AssetImport/sarm_20261012.catalog.json"
        base.parent.mkdir(parents=True)
        base.touch()
        editor = tmp_path / "editor"
        editor.touch()
        monkeypatch.setattr("space_arm_platform.collection_jobs.unreal_editor", lambda _: editor)
        monkeypatch.setattr(manager, "_run", lambda _: None)
        owner = dict(user_id="u", username="user", role="admin")
        job = manager.start(request, owner, {})
        snapshot = manager.root / job["id"] / "layout-pool.json"
        assert LayoutPool(snapshot).document == LayoutPool(layout_pool).document
        saved = json.loads((snapshot.parent / "scene.json").read_text())
        assert saved["collection_layout_randomization"] is True
    finally:
        manager.close()


def test_parallel_workers_receive_same_pool_with_disjoint_attempt_seeds(layout_pool, tmp_path, monkeypatch):
    import tools.collect_parallel as parallel

    # Exercise real leases without sharing lock files with other host users.
    monkeypatch.setattr(parallel.tempfile, "gettempdir", lambda: str(tmp_path))

    config = tmp_path / "collection.json"
    config.write_text(json.dumps(dict(workers=[{}, {}], layout_pool=layout_pool.name,
                                     episodes_per_worker=1, max_attempts_per_worker=3,
                                     port_base=31000, seed=101)))
    (tmp_path / "recording.json").write_text("{}")
    commands = []

    class Process:
        returncode = 0
        pid = 99999999
        def poll(self):
            return 0
        def wait(self, **kwargs):
            return 0

    def popen(command, **kwargs):
        plan = json.loads(Path(command[-1]).read_text())
        commands.append(plan["command"])
        worker_output = Path(plan["command"][plan["command"].index("--output")+1])
        worker_output.mkdir()
        (worker_output / "summary.json").write_text(json.dumps(dict(status="complete", accepted_episodes=["episode"])))
        return Process()

    monkeypatch.setattr(parallel, "check_port_available", lambda _: None)
    monkeypatch.setattr(parallel.subprocess, "Popen", popen)
    monkeypatch.setattr(parallel, "WindowsProcessGroup", lambda _: SimpleNamespace(close=lambda: None))
    args = SimpleNamespace(config=config, workers=None, episodes_per_worker=None,
                           output=tmp_path / "parallel", episodes_root=None, scene=tmp_path / "scene.json",
                           task="drawer_cycle", adapter=tmp_path, unreal=tmp_path)
    assert parallel.run(args) == 0
    assert [cmd[cmd.index("--seed")+1] for cmd in commands] == ["101", "104"]
    snapshot = args.output / "layout-pool.json"
    assert all(cmd[cmd.index("--layout-pool")+1] == str(snapshot) for cmd in commands)
    assert LayoutPool(snapshot).document == LayoutPool(layout_pool).document
