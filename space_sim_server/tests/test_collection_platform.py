"""Regression coverage for the authenticated batch lifecycle and independent clocks."""

import json
from pathlib import Path
import time
import os
import subprocess
import sys
import psutil

from fastapi.testclient import TestClient
import pytest
from pydantic import ValidationError

from space_arm_platform.app import PlatformConfig, create_app
from space_arm_platform.collection_jobs import (
    CollectionCreate,
    CollectionManager,
    CollectionTask,
    TASKS,
    TERMINAL,
)
from space_arm_platform.dataset_view import Store
from space_arm_platform.recorder import EpisodeRecorder
from space_arm_platform.collection_processes import WindowsProcessGroup
from test_collection_layouts import layout_pool
from test_collection_fixed_layout import fixed_inputs

ROOT = Path(__file__).resolve().parents[1]
OWNER = dict(user_id="owner", username="alice", role="operator")
OTHER = dict(user_id="other", username="bob", role="operator")
ADMIN = dict(user_id="admin", username="admin", role="admin")


@pytest.mark.parametrize(
    "payload",
    [
        dict(workers=5),
        dict(workers=2, count=1),
        dict(count=5, max_attempts=4),
        dict(task="shell"),
        dict(range_deg=51),
        dict(range_deg=float("nan")),
        dict(linear_speed_m_s=0),
        dict(linear_speed_m_s=0.2),
        dict(linear_speed_m_s=float("nan")),
        dict(task="aviation_plug_insert", linear_speed_m_s=0.1),
        dict(threaded_speed_profile="fast_collection"),
        dict(task="aviation_plug_insert", threaded_speed_profile="default"),
        dict(task="aviation_bolt_insert", threaded_speed_profile="unknown"),
        dict(seed=-1),
        dict(owner=ADMIN),
    ],
)
def test_reject_invalid_plan(payload):
    with pytest.raises(ValidationError):
        CollectionCreate(**payload)


FAKE_COLLECTOR = r"""
import argparse,json,time
from pathlib import Path
p=argparse.ArgumentParser()
for name in ('output','count','seed','cancel-file','linear-speed','range-deg','overview-camera-sampling','threaded-speed-profile'):
    p.add_argument('--'+name)
a,_=p.parse_known_args()
folder=Path(a.output);folder.mkdir()
(folder/'motion.json').write_text(json.dumps(dict(linear_speed_m_s=a.linear_speed, range_deg=a.range_deg, overview_camera_sampling=a.overview_camera_sampling, threaded_speed_profile=a.threaded_speed_profile)))
seed=int(a.seed)
if seed==99:
    while not Path(a.cancel_file).exists(): time.sleep(.05)
    status='cancelled';accepted=[]
else:
    time.sleep(.5)
    status='complete';accepted=[str(folder/('episode-'+str(seed+i))) for i in range(int(a.count))]
(folder/'summary.json').write_text(json.dumps(dict(status=status,attempts=[],accepted_episodes=accepted)))
raise SystemExit(0 if status=='complete' else 1)
"""


@pytest.fixture
def manager(tmp_path, monkeypatch):
    project = tmp_path / "project"
    project.mkdir()
    (project / "fake.py").write_text(FAKE_COLLECTOR)
    unreal = tmp_path / "ue"
    file = unreal / ("Engine/Binaries/Win64/UnrealEditor-Cmd.exe" if os.name == "nt"
                     else "Engine/Binaries/Linux/UnrealEditor-Cmd")
    file.parent.mkdir(parents=True)
    file.touch()
    file.chmod(0o755)
    adapter = tmp_path / "adapter"
    file = (
        adapter
        / "Unreal/BskUnrealRenderer/Saved/AssetImport/sarm_20261012.catalog.json"
    )
    file.parent.mkdir(parents=True)
    file.touch()
    monkeypatch.setitem(
        TASKS, "drawer_cycle", CollectionTask("fake", "fake.py", "fake")
    )
    manager = CollectionManager(project, tmp_path / "data/episodes", adapter, unreal)
    yield manager
    manager.close()


def wait_job(manager, key):
    deadline = time.monotonic() + 15
    while time.monotonic() < deadline:
        job = manager.get(key, OWNER)
        if job["status"] in TERMINAL:
            return job
        time.sleep(0.05)
    pytest.fail("collector did not terminate")


@pytest.fixture
def exiting_collector(manager, monkeypatch):
    """Finish a real child after the manager reads its penultimate summary."""
    (manager.project_root / "fake.py").write_text(r'''
import argparse, json, time
from pathlib import Path
p = argparse.ArgumentParser()
for name in ('output', 'seed'):
    p.add_argument('--' + name)
a, _ = p.parse_known_args()
folder = Path(a.output)
folder.mkdir()
def save(value):
    temporary = folder / 'summary.tmp'
    temporary.write_text(json.dumps(value))
    temporary.replace(folder / 'summary.json')
save(dict(status='running', attempts=[], accepted_episodes=[]))
(folder / 'ready').touch()
deadline = time.monotonic() + 10
while not (folder / 'finish.request').exists():
    if time.monotonic() > deadline:
        raise RuntimeError('manager never released test collector')
    time.sleep(.01)
seed = int(a.seed)
status = {31: 'attempt_limit', 32: 'failed', 35: 'attempt_limit',
          36: 'attempt_limit', 37: 'attempt_limit'}.get(seed, 'complete')
accepted = [str(folder / ('episode-' + str(seed)))] if status == 'complete' else []
attempt = dict(attempt=1, seed=seed, directory=str(folder / 'attempt-001'),
               status='accepted' if accepted else 'initialization_rejected')
save(dict(status=status, attempts=[attempt], accepted_episodes=accepted,
          error='renderer process failed' if status == 'failed' else None))
raise SystemExit(2 if seed == 37 else 0 if status == 'complete' else 1)
''', encoding="utf-8")
    processes = {}
    original_popen = subprocess.Popen
    original_refresh = manager._refresh
    released = []

    def launch(*args, **kwargs):
        process = original_popen(*args, **kwargs)
        processes[process.pid] = process
        return process

    def refresh_then_finish(job, worker):
        original_refresh(job, worker)
        folder = manager.root / job["id"] / f"worker-{worker['index']}"
        gate = folder / "finish.request"
        if (folder / "ready").exists() and not gate.exists():
            assert worker["accepted_episodes"] == []
            gate.touch()
            processes[worker["pid"]].wait(timeout=10)
            released.append(worker["index"])

    monkeypatch.setattr(subprocess, "Popen", launch)
    monkeypatch.setattr(manager, "_refresh", refresh_then_finish)
    return released


@pytest.mark.parametrize("seed,expected,successes", [
    (30, "complete", 1), (31, "attempt_limit", 0), (32, "failed", 0),
    (37, "failed", 0),  # A crash exit code must not become an ordinary limit.
])
def test_worker_exit_reads_final_summary_before_persisting_status(
    manager, exiting_collector, seed, expected, successes
):
    job = manager.start(CollectionCreate(workers=1, count=1, max_attempts=1, seed=seed), OWNER, {})
    result = wait_job(manager, job["id"])
    assert exiting_collector == [0]
    assert result["status"] == result["workers"][0]["status"] == expected
    assert result["success_count"] == successes
    assert len(result["workers"][0]["attempts"]) == 1
    if seed == 32:
        assert result["workers"][0]["error"] == "renderer process failed"
    saved = json.loads((manager.root / job["id"] / "job.json").read_text(encoding="utf-8"))
    assert saved["status"] == expected
    assert saved["workers"][0]["accepted_episodes"] == result["accepted_episodes"]
    manager.close()
    recovered = CollectionManager(manager.project_root, manager.data_root, manager.adapter, manager.unreal)
    try:
        reloaded = recovered.get(job["id"], OWNER)
        assert reloaded["status"] == expected and reloaded["success_count"] == successes
    finally:
        recovered.close()


@pytest.mark.parametrize("seed,expected,statuses,successes", [
    (30, "attempt_limit", ["complete", "attempt_limit"], 1),
    (31, "failed", ["attempt_limit", "failed"], 0),
    (33, "complete", ["complete", "complete"], 2),
    (35, "attempt_limit", ["attempt_limit", "attempt_limit"], 0),
])
def test_batch_completion_combines_final_worker_results(
    manager, exiting_collector, seed, expected, statuses, successes
):
    job = manager.start(CollectionCreate(workers=2, count=2, max_attempts=2, seed=seed), OWNER, {})
    result = wait_job(manager, job["id"])
    assert sorted(exiting_collector) == [0, 1]
    assert result["status"] == expected and result["success_count"] == successes
    assert [worker["status"] for worker in result["workers"]] == statuses
    # An exhausted batch must release its resources and permit another batch.
    next_job = manager.start(CollectionCreate(workers=1, count=1, max_attempts=1, seed=40), OWNER, {})
    assert wait_job(manager, next_job["id"])["status"] == "complete"


def test_four_workers_exact_success_budget_isolation_and_reload(manager):
    job = manager.start(
        CollectionCreate(workers=4, count=5, max_attempts=9, seed=30),
        OWNER,
        {"created_by": ADMIN},
    )
    with pytest.raises(ValueError):
        manager.start(CollectionCreate(), OWNER, {})
    with pytest.raises(KeyError):
        manager.get(job["id"], OTHER)
    result = wait_job(manager, job["id"])
    assert result["status"] == "complete" and result["success_count"] == 5
    assert result["owner"] == OWNER
    assert len({p for w in result["workers"] for p in w["ports"]}) == 16
    seeds = [
        s
        for w in result["workers"]
        for s in range(w["seed"], w["seed"] + w["max_attempts"])
    ]
    assert len(seeds) == len(set(seeds)) == 9
    assert manager.list(OTHER) == [] and len(manager.list(ADMIN)) == 1
    manager.close()
    recovered = CollectionManager(
        manager.project_root, manager.data_root, manager.adapter, manager.unreal
    )
    assert recovered.get(job["id"], OWNER)["success_count"] == 5
    recovered.close()


def test_cooperative_stop_keeps_results_and_exits(manager):
    job = manager.start(
        CollectionCreate(workers=1, count=1, max_attempts=1, seed=99), OWNER, {}
    )
    time.sleep(0.3)
    stopped = manager.stop(job["id"], OWNER)
    assert stopped["status"] == "stopping"
    result = wait_job(manager, job["id"])
    assert result["status"] == "cancelled" and result["success_count"] == 0
    assert manager.stop(job["id"], OWNER)["status"] == "cancelled"


def test_restart_does_not_silently_resume(tmp_path):
    folder = tmp_path / "collection_jobs/collection-stale"
    folder.mkdir(parents=True)
    (folder / "job.json").write_text(
        json.dumps(
            dict(
                id=folder.name,
                status="running",
                owner=OWNER,
                workers=[],
                started_at=time.time(),
            )
        )
    )
    manager = CollectionManager(tmp_path, tmp_path / "episodes")
    assert manager.get(folder.name, OWNER)["status"] == "interrupted"
    manager.close()


def make_episode(root, key, user=None):
    path = root / key
    path.mkdir(parents=True)
    (path / "metadata.json").write_text(
        json.dumps(
            dict(
                operator_user_id=user,
                dataset_status="complete",
                dataset_fps=50,
                dataset_frame_count=2,
            )
        )
    )
    (path / "steps.jsonl").write_text(
        "\n".join(
            json.dumps(
                {
                    "observation": dict(
                        sim_time_ns=str(1_000_000_000 + i * 20_000_000),
                        render_frame_id=str(i),
                        joint_position_rad=[0] * 8,
                        wrist_wrench={
                            "valid": True,
                            "force_n": [1, 2, 3],
                            "torque_nm": [4, 5, 6],
                        },
                    )
                }
            )
            for i in range(2)
        )
    )
    sidecar = path / "lerobot/high_frequency/wrist_wrench.jsonl"
    sidecar.parent.mkdir(parents=True)
    sidecar.write_text(
        "\n".join(
            json.dumps(
                dict(
                    sim_time_ns=str(1_000_000_000 + i * 5_000_000),
                    valid=True,
                    force_n=[i, 2, 3],
                    torque_nm=[4, 5, 6],
                )
            )
            for i in range(5)
        )
    )
    video = path / "lerobot/videos/camera/chunk-000/file-000.mp4"
    video.parent.mkdir(parents=True)
    video.write_bytes(b"0123456789")
    return path


def test_native_timestamps_no_upsampling_and_sidecar_cache(tmp_path):
    path = make_episode(tmp_path, "episode-one")
    store = Store(tmp_path)
    data = json.loads(store.load("episode-one"))
    force = next(g for g in data["groups"] if g["id"] == "force_200hz")
    assert data["times"] == [0, 0.02] and force["times"] == [
        0,
        0.005,
        0.01,
        0.015,
        0.02,
    ]
    assert data["count"] == 2 and len(force["series"][0]["values"]) == 5
    assert data["videos"][0]["url"].startswith("/api/datasets/episode-one/media/")
    sidecar = path / "lerobot/high_frequency/wrist_wrench.jsonl"
    sidecar.write_text(
        sidecar.read_text().replace('"force_n": [0,', '"force_n": [123,')
    )
    updated = json.loads(store.load("episode-one"))
    assert (
        next(g for g in updated["groups"] if g["id"] == "force_200hz")["series"][0][
            "values"
        ][0]
        == 123
    )


def test_api_ownership_media_ranges_and_auth(tmp_path, monkeypatch):
    monkeypatch.setattr(EpisodeRecorder, "prepare", lambda self: None)
    app = create_app(
        PlatformConfig(
            project_root=ROOT,
            data_root=tmp_path / "episodes",
            simulation_port=0,
            capture_port=0,
            auth_database=tmp_path / "auth.sqlite3",
        )
    )
    user = app.state.auth.create_operator("viewer", "Viewer-Password!")
    make_episode(tmp_path / "episodes", "episode-owned", user["user_id"])
    make_episode(tmp_path / "episodes", "episode-other", "another")
    make_episode(tmp_path / "episodes", "episode-legacy")
    with TestClient(app) as client:
        assert client.get("/api/datasets").status_code == 401
        assert client.get("/api/collections").status_code == 401
        client.post(
            "/api/auth/login", json=dict(username="viewer", password="Viewer-Password!")
        ).raise_for_status()
        assert [e["id"] for e in client.get("/api/datasets").json()["episodes"]] == [
            "episode-owned"
        ]
        assert client.get("/api/datasets/episode-other").status_code == 404
        assert client.get("/api/datasets/episode-legacy").status_code == 404
        assert (
            client.get(
                "/api/datasets/episode-other/media/lerobot/videos/camera/chunk-000/file-000.mp4"
            ).status_code
            == 404
        )
        assert (
            client.get("/api/datasets/episode-owned/media/metadata.json").status_code
            == 404
        )
        response = client.get(
            "/api/datasets/episode-owned/media/lerobot/videos/camera/chunk-000/file-000.mp4",
            headers={"Range": "bytes=2-5"},
        )
        assert response.status_code == 206 and response.content == b"2345"
        assert client.get("/api/datasets/episode-owned").status_code == 200
        assert client.get("/collection").status_code == 200
        assert client.get("/datasets").status_code == 200
        shell = client.get("/").text
        for route in ("/collection", "/datasets?episode=episode-owned", "/users"):
            response = client.get(route)
            assert response.status_code == 200
            assert response.text == shell
        assert 'id="platformNav"' in shell
        assert client.get("/api/collections/catalog").json()["rates"] == dict(
            dynamics=400, ik=100, main=50, wrench=200
        )
        camera_catalog = client.get("/api/collections/catalog").json()["overview_camera_randomization"]
        assert {item["preset_id"] for item in camera_catalog["presets"]} == {"P0", "P1", "P2", "P3", "P4"}
        assert client.post("/api/collections", json=dict(workers=5)).status_code == 422
        client.post("/api/auth/logout")
        client.post(
            "/api/auth/login", json=dict(username="admin", password="ChangeMe123!")
        ).raise_for_status()
        assert len(client.get("/api/datasets").json()["episodes"]) == 3


def test_second_coordinator_cannot_take_over_running_directory(manager):
    with pytest.raises(RuntimeError, match="第二个"):
        CollectionManager(
            manager.project_root, manager.data_root, manager.adapter, manager.unreal
        )


@pytest.mark.skipif(os.name != "nt", reason="Windows process ownership integration")
def test_windows_group_cleans_orphan_descendant(tmp_path):
    child_file = tmp_path / "child.pid"
    script = tmp_path / "parent.py"
    script.write_text(
        "import subprocess,sys,time\nfrom pathlib import Path\ntime.sleep(.2)\nchild=subprocess.Popen([sys.executable,'-c','import time;time.sleep(60)'])\nPath(sys.argv[1]).write_text(str(child.pid))\ntime.sleep(.5)\n"
    )
    process = subprocess.Popen(
        [sys.executable, str(script), str(child_file)],
        creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0),
    )
    group = WindowsProcessGroup(process)
    try:
        process.wait(timeout=10)
        pid = int(child_file.read_text())
        assert psutil.pid_exists(pid)
        group.close()
        deadline = time.monotonic() + 5
        while psutil.pid_exists(pid) and time.monotonic() < deadline:
            time.sleep(0.05)
        assert not psutil.pid_exists(pid)
    finally:
        group.close()
        if process.poll() is None:
            process.kill()


def test_drawer_speed_reaches_spawned_collector_and_is_persisted(manager):
    job = manager.start(CollectionCreate(workers=1, count=1, max_attempts=1,
                                        linear_speed_m_s=0.05), OWNER, {})
    result = wait_job(manager, job["id"])
    assert result["status"] == "complete"
    assert result["config"]["linear_speed_m_s"] == 0.05
    received = json.loads((manager.root / job["id"] / "worker-0" / "motion.json").read_text())
    assert float(received["linear_speed_m_s"]) == 0.05
    assert CollectionCreate().linear_speed_m_s == 0.1
    assert CollectionCreate(task="aviation_plug_insert").linear_speed_m_s is None


@pytest.mark.parametrize("task", ["aviation_bolt_insert", "locking_rod_cycle"])
@pytest.mark.parametrize("profile", [None, "default", "fast_collection"])
def test_threaded_speed_reaches_spawned_collector_and_is_persisted(
    manager, monkeypatch, task, profile, fixed_inputs
):
    manager.fixed_model, manager.fixed_catalog = fixed_inputs
    monkeypatch.setitem(TASKS, task, CollectionTask("fake", "fake.py", "fake"))
    request = CollectionCreate(
        task=task, workers=1, count=1, max_attempts=1,
        threaded_speed_profile=profile,
    )
    job = manager.start(request, OWNER, {})
    result = wait_job(manager, job["id"])
    expected = profile or "fast_collection"
    assert result["status"] == "complete"
    assert result["config"]["threaded_speed_profile"] == expected
    received = json.loads(
        (manager.root / job["id"] / "worker-0" / "motion.json").read_text()
    )
    assert received["threaded_speed_profile"] == expected
    assert received["linear_speed_m_s"] is None


@pytest.mark.parametrize("task", ["aviation_bolt_insert", "locking_rod_cycle"])
def test_saved_threaded_job_without_profile_keeps_original_speed(manager, monkeypatch, task, fixed_inputs):
    manager.fixed_model, manager.fixed_catalog = fixed_inputs
    monkeypatch.setitem(TASKS, task, CollectionTask("fake", "fake.py", "fake"))
    # Prepare an old job document before its worker is launched.
    with monkeypatch.context() as context:
        context.setattr("threading.Thread.start", lambda self: None)
        started = manager.start(
            CollectionCreate(task=task, workers=1, count=1, max_attempts=1), OWNER, {}
        )
    job = manager.jobs[started["id"]]
    job["config"].pop("threaded_speed_profile")
    manager._run(job)
    assert job["status"] == "complete"
    received = json.loads(
        (manager.root / job["id"] / "worker-0" / "motion.json").read_text()
    )
    assert received["threaded_speed_profile"] == "default"


def test_worker_configuration_preserves_independent_options_and_disabled_initialization(manager):
    options={"joint_bias":{"enabled":True,"max_abs_deg":.2},"wrist_wrench_noise":{"enabled":True,"force_std_n":.3}}
    source={"created_by":ADMIN,"seed":40,"template_id":"sarm-task-box-20261012"}
    request=CollectionCreate(workers=1,count=1,max_attempts=1,seed=40,initialization_enabled=False,randomization=options,randomize_overview_camera=True)
    job=manager.start(request,OWNER,source)
    result=wait_job(manager,job["id"])
    assert result["status"]=="complete"
    saved=json.loads((manager.root/job["id"]/"scene.json").read_text(encoding="utf-8"))
    assert saved["collection_randomization"]["wrist_wrench_noise"]["force_std_n"]==.3
    assert saved["collection_randomization"]["joint_bias"]["enabled"]
    assert saved["initialization_enabled"] is False
    from space_arm_platform.overview_camera import sample_overview_camera
    assert saved["overview_camera"] == sample_overview_camera(40)
    assert result["config"]["randomize_overview_camera"] is True
    assert result["config"]["randomization"]==request.randomization.model_dump()
    assert "collection_randomization" not in source


@pytest.mark.parametrize("half_range", [30, 50])
def test_accepted_joint_range_and_fixed_camera_mode_reach_worker(manager, half_range):
    job = manager.start(CollectionCreate(workers=1, count=1, max_attempts=1,
                                        range_deg=half_range), OWNER, {})
    result = wait_job(manager, job["id"])
    assert result["status"] == "complete"
    received = json.loads((manager.root / job["id"] / "worker-0" / "motion.json").read_text())
    assert float(received["range_deg"]) == half_range
    assert received["overview_camera_sampling"] == "presets"
    saved = json.loads((manager.root / job["id"] / "scene.json").read_text())
    assert saved["overview_camera_sampling"] == "presets"
