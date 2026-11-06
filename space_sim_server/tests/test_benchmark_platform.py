"""Platform lifecycle tests use failing worker processes, never fake physics passes."""
import json
import csv
import io
from pathlib import Path
import shutil
import sys
import time

from fastapi.testclient import TestClient
import psutil
import pytest
from pydantic import ValidationError

from space_arm_platform.app import PlatformConfig, create_app
from space_arm_platform import benchmark_jobs as jobs
from space_arm_platform.benchmark_batch import run_batch, summarize
from space_arm_platform.benchmark_contract import load_task, SPEC_DIRECTORY
from space_arm_platform.benchmark_execution import write_json, sha256
from space_arm_platform.benchmark_api import results_csv
from space_arm_platform.collection_processes import launch_collector, kill_owned_tree
from space_arm_platform.recorder import EpisodeRecorder

ROOT = Path(__file__).resolve().parents[1]
OWNER = dict(user_id="owner", username="alice", role="operator")
OTHER = dict(user_id="other", username="bob", role="operator")
ADMIN = dict(user_id="admin", username="admin", role="admin")


def test_csv_keeps_nested_orbit_metrics_vectors_and_missing_values():
    score = dict(final_time_s=300., success=True, fuel_kg=None,
                 final_orbit=dict(semimajor_axis_error_m=1.25, eccentricity=1e-6),
                 delta_velocity_j2000_m_s=[0., -2., 0.], safety_violations=[])
    rows = [dict(trial_id="one", seed=201, outcome="success", physics_score=score),
            dict(trial_id="two", seed=202, outcome="cancelled", physics_score=None)]
    document = dict(job=dict(configuration="s11-orbit-raise-v1", method="gnss-orbit-maneuver-v1",
                            request=dict(split="test", selection="split"), summary=dict(trials=rows)))
    exported = list(csv.DictReader(io.StringIO(results_csv(document))))
    assert len(exported) == 2
    assert exported[0]["final_orbit.semimajor_axis_error_m"] == "1.25"
    assert exported[0]["final_orbit.eccentricity"] == "1e-06"
    assert exported[0]["delta_velocity_j2000_m_s[1]"] == "-2.0"
    assert exported[0]["fuel_kg"] == exported[1]["final_time_s"] == ""
    assert exported[1]["outcome"] == "cancelled"
    assert "success" not in exported[0] and "safety_violations" not in exported[0]
CONFIGURATION = "s02-wheel-attitude-v1"

# This coordinator exercises real process ownership and durable batch recovery.
# All attempted cases are explicitly cancelled or infrastructure failures.
WORKER = r'''
import argparse,json,time,subprocess,sys
from pathlib import Path
from space_arm_platform.benchmark_contract import load_task
from space_arm_platform.benchmark_execution import source_identity,write_json
p=argparse.ArgumentParser()
p.add_argument('command');p.add_argument('configuration')
for key in ('split','output','stop-file','method','method-spec','seed'): p.add_argument('--'+key)
p.add_argument('--resume',action='store_true');a=p.parse_args()
root=Path.cwd();output=Path(a.output);output.mkdir(exist_ok=True)
task=load_task(root/'config/benchmarks/tasks'/f'{a.configuration}.json')
seeds=[int(a.seed)] if a.seed else getattr(task.seeds,a.split)
planned=[dict(trial_id=f'{i:04d}-seed-{seed}',seed=seed) for i,seed in enumerate(seeds)]
plan=dict(task_fingerprint=task.fingerprint(),method=a.method,source_sha256=source_identity(root)['source_sha256'],trials=planned)
if not a.resume: write_json(output/'plan.json',plan)
for index,trial in enumerate(planned):
    directory=output/trial['trial_id']
    if (directory/'status.json').exists(): continue
    if Path(a.stop_file).exists(): break
    directory.mkdir();write_json(directory/'status.json',dict(outcome='running'))
    if index==0 and not a.resume:
        child=subprocess.Popen([sys.executable,'-c','import time;time.sleep(120)'])
        (output/'descendant.pid').write_text(str(child.pid))
        while not Path(a.stop_file).exists(): time.sleep(.02)
        outcome='cancelled'
    else: outcome='infrastructure_error'
    write_json(directory/'status.json',dict(outcome=outcome))
'''


@pytest.fixture
def manager(tmp_path, monkeypatch):
    project = tmp_path / "project"
    shutil.copytree(ROOT / "config/benchmarks", project / "config/benchmarks")
    (project / "native").mkdir()
    (project / "native/local_mujoco_stepper.cpp").write_text("// lifecycle fixture\n")
    (project / "tools").mkdir()
    (project / "tools/autonomy_benchmark.py").write_text(WORKER)
    monkeypatch.setattr(jobs, "dependency_check", lambda root, **kwargs: {"available": True, "problems": []})
    manager = jobs.BenchmarkManager(project, tmp_path / "data/episodes")
    try:
        yield manager
    finally:
        manager.close()


def eventually(probe, timeout=15):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        result = probe()
        if result:
            return result
        time.sleep(.03)
    pytest.fail("timed out waiting for owned benchmark worker")


def start(manager):
    job = manager.start(jobs.BenchmarkCreate(configuration=CONFIGURATION, split="validation"), OWNER)
    directory = manager.root / job["id"] / "batch"
    eventually(lambda: (directory / "descendant.pid").is_file())
    return job, directory


@pytest.mark.parametrize("extra", [dict(selection="single"), dict(seed=201), dict(owner=ADMIN),
    dict(configuration="../other"), dict(method_spec="arbitrary.py"), dict(seed=True, selection="single")])
def test_web_contract_rejects_paths_identity_and_ambiguous_selection(extra):
    with pytest.raises(ValidationError):
        jobs.BenchmarkCreate.model_validate(dict(configuration=CONFIGURATION) | extra)


def test_running_and_unstarted_samples_remain_in_denominator():
    result = summarize([dict(outcome=value) for value in ("success", "running", "pending")])
    assert result["finished"] == 1 and result["planned"] == 3
    assert result["success_rate_all_planned"] == 1 / 3
    assert result["wilson_95_interval"] is None and not result["complete"]


def test_atomic_result_rejects_nan_without_destroying_previous_snapshot(tmp_path):
    path = tmp_path / "result.json"
    write_json(path, {"success": False})
    original = path.read_bytes()
    with pytest.raises(ValueError):
        write_json(path, {"value": float("nan")})
    assert path.read_bytes() == original


def test_batch_stop_and_resume_only_unstarted_with_frozen_single_selection(tmp_path, monkeypatch):
    from space_arm_platform import benchmark_batch
    task = load_task(SPEC_DIRECTORY / (CONFIGURATION + ".json"))
    stop = tmp_path / "stop"
    stop.touch()
    output = tmp_path / "single"
    summary = run_batch(task, None, "validation", output, stop_file=stop, seed=101)
    assert summary["planned"] == 1 and summary["outcomes"] == {"pending": 1}
    with pytest.raises(ValueError, match="frozen batch plan"):
        run_batch(task, None, "validation", output, resume=True, stop_file=stop, seed=102)
    with pytest.raises(ValueError, match="selected frozen split"):
        run_batch(task, None, "validation", tmp_path / "invalid", seed=201)
    launches = []
    def fail_worker(command, **kwargs):
        launches.append(command)
        return launch_collector([sys.executable, "-c", "raise SystemExit(3)"], **kwargs)
    monkeypatch.setattr(benchmark_batch, "launch_collector", fail_worker)
    stop.unlink()
    summary = run_batch(task, None, "validation", output, resume=True, stop_file=stop, seed=101)
    assert summary["outcomes"] == {"infrastructure_error": 1}
    run_batch(task, None, "validation", output, resume=True, stop_file=stop, seed=101)
    assert len(launches) == 1  # A failed attempt is never replaced by a retry.


def test_owned_stop_resume_keeps_failed_seed_and_cleans_descendants(manager):
    job, directory = start(manager)
    key = job["id"]
    child = psutil.Process(int((directory / "descendant.pid").read_text()))
    with pytest.raises(ValueError, match="运行中"):
        manager.start(jobs.BenchmarkCreate(configuration=CONFIGURATION), OWNER)
    assert manager.list(OTHER) == []
    with pytest.raises(KeyError):
        manager.stop(key, OTHER)
    assert manager.get(key, ADMIN)["owner"] == OWNER
    manager.stop(key, OWNER)
    eventually(lambda: manager.get(key, OWNER)["status"] in jobs.TERMINAL)
    eventually(lambda: not child.is_running())
    stopped = manager.get(key, OWNER)
    assert stopped["summary"]["outcomes"] == {"cancelled": 1, "pending": 2}
    assert stopped["can_resume"]
    first = directory / "0000-seed-101/status.json"
    recorded = json.loads(first.read_text())
    manager.resume(key, OWNER)
    eventually(lambda: manager.get(key, OWNER)["status"] in jobs.TERMINAL)
    complete = manager.get(key, OWNER)
    assert complete["status"] == "complete" and not complete["can_resume"]
    assert complete["summary"]["planned"] == 3
    assert complete["summary"]["outcomes"] == {"cancelled": 1, "infrastructure_error": 2}
    assert json.loads(first.read_text()) == recorded
    assert manager.rescore(key, OWNER)["trials"][0]["replay"] == "no_complete_episode_evidence"
    assert manager.export(key, OWNER)["job"]["summary"]["successes"] == 0
    with pytest.raises(KeyError):
        manager.artifact(key, "../other", "result", OWNER)


def test_killed_coordinator_recovers_failure_without_retry(manager):
    job, directory = start(manager)
    identity = manager.jobs[job["id"]]["process"]
    child = psutil.Process(int((directory / "descendant.pid").read_text()))
    kill_owned_tree(identity["pid"], identity["created"], guarded=identity["guard"] == "linux_subreaper/1")
    eventually(lambda: manager.get(job["id"], OWNER)["status"] in jobs.TERMINAL)
    eventually(lambda: not child.is_running())
    result = manager.get(job["id"], OWNER)
    assert result["summary"]["outcomes"] == {"infrastructure_error": 1, "pending": 2}
    assert result["can_resume"]


def test_source_change_blocks_resume_and_second_manager_cannot_steal(manager):
    with pytest.raises(RuntimeError):
        jobs.BenchmarkManager(manager.project, manager.root.parent / "episodes")
    job, directory = start(manager)
    manager.stop(job["id"], OWNER)
    eventually(lambda: manager.get(job["id"], OWNER)["status"] in jobs.TERMINAL)
    before = sha256(directory / "0000-seed-101/status.json")
    (manager.project / "native/local_mujoco_stepper.cpp").write_text("// changed\n")
    with pytest.raises(ValueError, match="源码已改变"):
        manager.resume(job["id"], OWNER)
    assert sha256(directory / "0000-seed-101/status.json") == before


def test_api_uses_existing_login_and_owner_checks(tmp_path, monkeypatch):
    monkeypatch.setattr(EpisodeRecorder, "prepare", lambda self: None)
    app = create_app(PlatformConfig(project_root=ROOT, data_root=tmp_path / "episodes", simulation_port=0,
                                   capture_port=0, auth_database=tmp_path / "auth.sqlite3"))
    user = app.state.auth.create_operator("viewer", "Viewer-Password!")
    manager = app.state.benchmarks
    key = "benchmark-" + "a" * 32
    manager.jobs[key] = dict(id=key, status="failed", configuration=CONFIGURATION, method="attitude-wheel-pd-v1",
        request=dict(split="test", selection="single"), seeds=[201], owner=dict(user_id="another",username="alice",role="operator"),
        started_at=time.time(), finished_at=time.time())
    directory = manager.root / key
    directory.mkdir()
    write_json(directory / "task.json", load_task(SPEC_DIRECTORY / (CONFIGURATION + ".json")).model_dump(mode="json"))
    with TestClient(app) as client:
        for path in ("/api/benchmarks", "/api/benchmarks/catalog", f"/api/benchmarks/{key}/export"):
            assert client.get(path).status_code == 401
        client.post("/api/auth/login", json=dict(username="viewer",password="Viewer-Password!")).raise_for_status()
        assert client.get("/api/benchmarks").json() == {"jobs": []}
        for suffix in ("", "/export", "/trials/0000-seed-201/artifacts/result"):
            assert client.get(f"/api/benchmarks/{key}" + suffix).status_code == 404
        for action in ("stop", "resume", "rescore"):
            assert client.post(f"/api/benchmarks/{key}/{action}").status_code == 404
        assert client.post("/api/benchmarks", json=dict(configuration=CONFIGURATION,owner=user)).status_code == 422
        catalog = client.get("/api/benchmarks/catalog").json()
        assert catalog["requirements"]["requirements"] == 122
        assert {item["source_id"] for item in catalog["configurations"]} == {"S01", "S02", "S03", "S04", "S05", "S06", "S09", "S11", "S13", "S14", "S16", "S22", "S23", "D01", "D05", "D12", "D14", "D15", "D16", "D19", "D23", "D24", "D27", "D29", "D30", "D32", "D33", "D37", "D40", "M01", "M02", "M03"}
        client.post("/api/auth/logout")
        client.post("/api/auth/login", json=dict(username="admin",password="ChangeMe123!")).raise_for_status()
        assert len(client.get("/api/benchmarks").json()["jobs"]) == 1
        response = client.get(f"/api/benchmarks/{key}/export?format=csv")
        assert response.status_code == 200 and "attachment" in response.headers["content-disposition"]
        assert "pending" in response.text


def incomplete_archive(manager):
    key = "benchmark-" + "b" * 32
    job = dict(id=key, status="complete", configuration=CONFIGURATION, method="attitude-wheel-pd-v1",
        request=dict(split="test", selection="single"), seeds=[201], owner=OWNER,
        started_at=time.time(), finished_at=time.time())
    manager.jobs[key] = job
    directory = manager.root / key / "batch/0000-seed-201/run"
    directory.mkdir(parents=True)
    task = load_task(SPEC_DIRECTORY / (CONFIGURATION + ".json"))
    write_json(directory / "task.json", task.model_dump(mode="json"))
    write_json(directory / "manifest.json", {"task_fingerprint": task.fingerprint()})
    result = dict(outcome="infrastructure_error", physics_score=None, evidence_sha256={
        name: sha256(directory / name) for name in ("task.json", "manifest.json")})
    write_json(directory / "result.json", result)
    row = dict(trial_id="0000-seed-201", seed=201, outcome="infrastructure_error", result_sha256=sha256(directory / "result.json"))
    write_json(directory.parents[1] / "summary.json", summarize([row]))
    return key, directory, row, result


def test_initialization_failure_keeps_archive_without_advertising_nonexistent_logs(manager):
    key, directory, _, _ = incomplete_archive(manager)
    row = manager.get(key, OWNER)["summary"]["trials"][0]
    assert set(row["artifacts"]) == {"result", "task", "manifest"}
    report = manager.rescore(key, OWNER)["trials"][0]
    assert report["outcome"] == "infrastructure_error" and report["archive_hashes_verified"]
    assert report["replay"] == "no_complete_episode_evidence"
    with pytest.raises(KeyError):
        manager.artifact(key, row["trial_id"], "physics", OWNER)
    with (directory / "manifest.json").open("a") as stream:
        stream.write(" ")
    with pytest.raises(ValueError, match="changed or invalid evidence"):
        manager.rescore(key, OWNER)
    with pytest.raises(ValueError, match="已改变"):
        manager.artifact(key, row["trial_id"], "manifest", OWNER)


def test_changed_result_or_raw_physics_cannot_be_downloaded_as_verified(manager):
    key, directory, row, result = incomplete_archive(manager)
    write_json(directory / "manifest.json", {"physics_path": "physics.jsonl"})
    (directory / "physics.jsonl").write_text('{}\n')
    result["evidence_sha256"].update({name: sha256(directory / name) for name in ("manifest.json", "physics.jsonl")})
    write_json(directory / "result.json", result)
    row["result_sha256"] = sha256(directory / "result.json")
    write_json(directory.parents[1] / "summary.json", summarize([row]))
    assert manager.artifact(key, row["trial_id"], "physics", OWNER) == directory / "physics.jsonl"
    (directory / "physics.jsonl").write_text('{"changed":true}\n')
    with pytest.raises(ValueError, match="已改变"):
        manager.artifact(key, row["trial_id"], "physics", OWNER)
    write_json(directory / "result.json", dict(result, outcome="success"))
    assert manager.get(key, OWNER)["summary"]["outcomes"] == {"invalid_evidence": 1}
    with pytest.raises(KeyError):
        manager.artifact(key, row["trial_id"], "result", OWNER)


@pytest.mark.parametrize('name', ['context', 'runtime_final'])
def test_public_context_and_runtime_diagnostics_keep_artifact_hash_checks(manager, name):
    key, directory, row, result = incomplete_archive(manager)
    write_json(directory/(name+'.json'), {'recorded': True})
    result['evidence_sha256'][name+'.json'] = sha256(directory/(name+'.json'))
    write_json(directory/'result.json', result)
    row['result_sha256'] = sha256(directory/'result.json')
    write_json(directory.parents[1]/'summary.json', summarize([row]))
    assert name in manager.get(key, OWNER)['summary']['trials'][0]['artifacts']
    assert manager.artifact(key, row['trial_id'], name, OWNER) == directory/(name+'.json')
    write_json(directory/(name+'.json'), {'changed': True})
    with pytest.raises(ValueError, match='已改变'):
        manager.artifact(key, row['trial_id'], name, OWNER)
