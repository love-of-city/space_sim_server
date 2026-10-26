"""Layout selection must bind preflight, capture and accepted episode metadata."""
import copy
import json
from pathlib import Path
from types import SimpleNamespace

import pytest

from space_arm_platform.collection_layouts import (
    LayoutPool, attach_layout, digest, pool_entry, validate_episode_layout,
)
from simulation.independent_task_box_layout import MODULES, PICKUPS, TARGETS, sample_placements
from simulation.scripted_tasks.drawer_initialization import randomized_scene


@pytest.fixture
def layout_pool(tmp_path):
    entries = []
    # Tiny, disjoint envelopes exercise the actual manifest validator; physics
    # and renderer integration are checked separately with the real SARM model.
    footprints = {name: [[.01 + .03*i, .01], [.02 + .03*i, .02]]
                  for i, name in enumerate(MODULES)}
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
        offsets, _ = sample_placements(footprints, i)
        record = dict(schema="task-box-layout/2", seed=i, model_sha256=digest(model), source_sha256="original",
                      initial_states_randomized=False, orientations_randomized=False,
                      adapted_base=dict(visual=visuals[-1]["file"]), generated_visuals=visuals,
                      offsets_body_xy_m=offsets, original_footprints_body_xy_m=footprints,
                      fixed_obstacles_body_xy_m=[], deck_edge_margin_m=.005, minimum_module_gap_m=.008)
        model.with_suffix(".layout.json").write_text(json.dumps(record))
        catalog.write_text(json.dumps(dict(source_mjcf=str(model), assets=assets)))
        native.write_text(json.dumps(dict(passed=True, initial_states_preserved=True,
                                         model_sha256=digest(model), source_model_sha256="original")))
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
