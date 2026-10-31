"""Faults must not erase accepted data or leave the batch coordinator occupied."""

from copy import deepcopy
import json
import os
import subprocess
import sys
import time

import psutil
import pytest

from space_arm_platform.collection_jobs import CollectionCreate, CollectionManager, TERMINAL, kill_owned_tree
from test_collection_platform import FAKE_COLLECTOR, OWNER, exiting_collector, manager, wait_job


def save(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value), encoding="utf-8")


@pytest.mark.parametrize("failure", ["persist", "thread"])
def test_start_failure_releases_batch_slot(manager, monkeypatch, failure):
    def fail(*args, **kwargs):
        raise RuntimeError("injected startup failure")

    with monkeypatch.context() as patch:
        if failure == "persist":
            patch.setattr(manager, "_save", fail)
        else:
            patch.setattr("threading.Thread.start", fail)
        with pytest.raises(RuntimeError, match="injected startup failure"):
            manager.start(CollectionCreate(workers=1, count=1, max_attempts=1), OWNER, {})
    assert all(job["status"] in TERMINAL for job in manager.jobs.values())
    next_job = manager.start(CollectionCreate(workers=1, count=1, max_attempts=1), OWNER, {})
    assert wait_job(manager, next_job["id"])["status"] == "complete"


def prepared_job(manager, monkeypatch):
    # Capture exactly the durable startup state, before a worker is launched.
    with monkeypatch.context() as patch:
        patch.setattr("threading.Thread.start", lambda self: None)
        result = manager.start(CollectionCreate(workers=1, count=2, max_attempts=3), OWNER, {})
    job = manager.jobs[result["id"]]
    job["workers"][0]["status"] = "running"
    return job


def report_for(manager, job):
    folder = manager.root / job["id"] / "worker-0"
    episode = str(manager.data_root / "episode-recovered")
    return folder / "summary.json", dict(
        status="running", accepted_episodes=[episode],
        attempts=[dict(attempt=1, seed=10, status="accepted", directory=str(folder / "attempt-001"))],
    )


@pytest.mark.parametrize("collector_status", ["running", "complete", "failed"])
def test_restart_recovers_durable_progress_without_resuming(manager, monkeypatch, collector_status):
    job = prepared_job(manager, monkeypatch)
    path, report = report_for(manager, job)
    report["status"] = collector_status
    if collector_status == "failed":
        report["error"] = "renderer disconnected after first accepted episode"
    save(path, report)
    # The worker saved a later result than the API coordinator's snapshot.
    manager._save(job)
    manager.jobs.clear()
    manager.close()
    recovered = CollectionManager(manager.project_root, manager.data_root, manager.adapter, manager.unreal)
    try:
        result = recovered.get(job["id"], OWNER)
        assert result["status"] == "interrupted"
        assert result["success_count"] == 1
        assert result["accepted_episodes"] == ["episode-recovered"]
        worker = result["workers"][0]
        assert worker["status"] == "interrupted" and worker["collector_status"] == collector_status
        assert len(worker["attempts"]) == 1
        if collector_status == "failed":
            assert worker["error"] == report["error"]
        saved = json.loads((manager.root / job["id"] / "job.json").read_text("utf-8"))
        assert saved["workers"][0]["accepted_episodes"] == result["accepted_episodes"]
        assert recovered.thread is None
    finally:
        recovered.close()


@pytest.mark.parametrize("invalid", [
    "missing", "truncated", "not_object", "wrong_attempts", "wrong_episodes", "duplicates", "regressed",
])
def test_bad_progress_preserves_last_accepted_results(manager, monkeypatch, invalid):
    job = prepared_job(manager, monkeypatch)
    path, report = report_for(manager, job)
    save(path, report)
    manager._refresh(job, job["workers"][0])
    previous = deepcopy(job["workers"][0])
    if invalid == "missing":
        path.unlink()
    elif invalid == "truncated":
        path.write_text('{"status":', encoding="utf-8")
    elif invalid == "not_object":
        save(path, [1])
    else:
        bad = deepcopy(report)
        if invalid == "wrong_attempts":
            bad["attempts"] = [None]
        elif invalid == "wrong_episodes":
            bad["accepted_episodes"] = [None]
        elif invalid == "duplicates":
            bad["accepted_episodes"] *= 2
        else:
            bad["accepted_episodes"] = []
        save(path, bad)
    manager._refresh(job, job["workers"][0])
    worker = job["workers"][0]
    assert worker["accepted_episodes"] == previous["accepted_episodes"]
    assert worker["attempts"] == previous["attempts"]
    assert worker.get("summary_error")
    save(path, report)
    manager._refresh(job, worker)
    assert not worker.get("summary_error")
    # No background thread exists for this deliberately interrupted snapshot.
    job["status"] = "interrupted"


@pytest.mark.parametrize("final_report", ["missing", "invalid"])
def test_invalid_final_summary_fails_and_keeps_previous_success(manager, monkeypatch, final_report):
    # A real process publishes an accepted record, then loses its final report.
    script = r'''
import argparse, json, time
from pathlib import Path
p=argparse.ArgumentParser()
p.add_argument('--output')
a,_=p.parse_known_args()
folder=Path(a.output);folder.mkdir()
path=folder/'summary.json'
path.write_text(json.dumps(dict(status='running', accepted_episodes=['episode-kept'], attempts=[])))
deadline=time.monotonic()+10
while not (folder/'finish.request').exists():
    if time.monotonic()>deadline: raise RuntimeError('manager did not observe progress')
    time.sleep(.01)
if FINAL_REPORT == 'missing': path.unlink()
else: path.write_text('[]')
'''.replace("FINAL_REPORT", repr(final_report))
    (manager.project_root / "fake.py").write_text(script, encoding="utf-8")
    original_refresh = manager._refresh

    def release_after_progress(job, worker):
        result = original_refresh(job, worker)
        if worker["accepted_episodes"]:
            (manager.root / job["id"] / "worker-0/finish.request").touch()
        return result

    monkeypatch.setattr(manager, "_refresh", release_after_progress)
    job = manager.start(CollectionCreate(workers=1, count=2, max_attempts=2), OWNER, {})
    result = wait_job(manager, job["id"])
    assert result["status"] == "failed"
    assert result["success_count"] == 1 and result["accepted_episodes"] == ["episode-kept"]
    assert result["workers"][0].get("summary_error")
    assert result["workers"][0].get("error")


@pytest.mark.parametrize("tail", ["null", "[1, 2]"])
def test_non_object_progress_log_does_not_break_collection(manager, monkeypatch, tail):
    job = prepared_job(manager, monkeypatch)
    path, report = report_for(manager, job)
    save(path, report)
    log = path.parent / "attempt-001/recording.log"
    log.parent.mkdir()
    log.write_text('{"stage":"hold"}\n' + tail + '\n', encoding="utf-8")
    manager._refresh(job, job["workers"][0])
    assert job["workers"][0]["progress"]["stage"] == "hold"
    assert job["workers"][0]["accepted_episodes"] == ["episode-recovered"]
    job["status"] = "interrupted"


@pytest.mark.skipif(os.name != "nt", reason="Windows kernel job and crash recovery integration")
def test_killed_coordinator_recovers_worker_result_and_releases_processes(manager):
    # The API never reads this worker's newer summary before it is killed.
    manager.close()
    (manager.project_root / "fake.py").write_text(r'''
import argparse,json,os,time,psutil
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('--output');a,_=p.parse_known_args()
folder=Path(a.output);folder.mkdir()
(folder/'process.json').write_text(json.dumps(dict(pid=os.getpid(),created=psutil.Process().create_time())))
(folder/'summary.json').write_text(json.dumps(dict(status='running',
    attempts=[dict(status='accepted',directory=str(folder/'attempt-001'))],
    accepted_episodes=['episode-before-crash'])))
time.sleep(60)
''', encoding="utf-8")
    coordinator_script = manager.project_root / "coordinator.py"
    coordinator_script.write_text(r'''
import json,os,sys,time,psutil
from pathlib import Path
from space_arm_platform.collection_jobs import CollectionManager,CollectionCreate,TASKS,CollectionTask
TASKS['drawer_cycle']=CollectionTask('fake','fake.py','fake')
m=CollectionManager(*sys.argv[1:])
(Path(sys.argv[1])/'coordinator-process.json').write_text(json.dumps(dict(pid=os.getpid(),created=psutil.Process().create_time())))
m._refresh=lambda job,worker: None
m.start(CollectionCreate(workers=1,count=2,max_attempts=3),
        dict(user_id='owner',username='alice',role='operator'),{})
time.sleep(60)
''', encoding="utf-8")
    recovered = None
    coordinator_identity = worker_identity = None
    with (manager.project_root / "coordinator.log").open("w", encoding="utf-8") as log:
        process = subprocess.Popen(
            [sys.executable, str(coordinator_script), str(manager.project_root),
             str(manager.data_root), str(manager.adapter), str(manager.unreal)],
            stdout=log, stderr=subprocess.STDOUT,
            creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0),
        )
        try:
            deadline = time.monotonic() + 15
            while time.monotonic() < deadline:
                summaries = list(manager.root.glob("*/worker-0/summary.json"))
                if summaries:
                    break
                assert process.poll() is None, "coordinator exited before acceptance"
                time.sleep(.05)
            assert summaries, "worker did not publish its acceptance"
            job_path = summaries[0].parent.parent / "job.json"
            snapshot = json.loads(job_path.read_text("utf-8"))
            assert snapshot["workers"][0]["accepted_episodes"] == []
            worker_pid = snapshot["workers"][0]["pid"]
            coordinator_identity = json.loads((manager.project_root / 'coordinator-process.json').read_text('utf-8'))
            worker_identity = json.loads((summaries[0].parent / 'process.json').read_text('utf-8'))
            # A Windows venv redirector may own Popen.pid while its child runs
            # Python. Kill the process that actually holds the coordinator lease.
            coordinator = psutil.Process(coordinator_identity['pid'])
            assert abs(coordinator.create_time() - coordinator_identity['created']) < .01
            assert coordinator.pid in {process.pid, *(p.pid for p in psutil.Process(process.pid).children(recursive=True))}
            coordinator.kill()
            coordinator.wait(timeout=10)
            process.wait(timeout=10)
            recovered = CollectionManager(manager.project_root, manager.data_root, manager.adapter, manager.unreal)
            result = recovered.get(snapshot["id"], OWNER)
            assert result["status"] == "interrupted"
            assert result["accepted_episodes"] == ["episode-before-crash"]
            deadline = time.monotonic() + 5
            worker_pids = {worker_pid, worker_identity['pid']}
            while any(psutil.pid_exists(pid) for pid in worker_pids) and time.monotonic() < deadline:
                time.sleep(.05)
            assert not any(psutil.pid_exists(pid) for pid in worker_pids)
            (manager.project_root / "fake.py").write_text(FAKE_COLLECTOR, encoding="utf-8")
            next_job = recovered.start(CollectionCreate(workers=1, count=1, max_attempts=1), OWNER, {})
            assert wait_job(recovered, next_job["id"])["status"] == "complete"
        finally:
            if coordinator_identity is not None:
                kill_owned_tree(coordinator_identity['pid'], coordinator_identity['created'])
            if worker_identity is not None:
                kill_owned_tree(worker_identity['pid'], worker_identity['created'])
            if process.poll() is None:
                process.kill()
                process.wait(timeout=10)
            if recovered is not None:
                recovered.close()


@pytest.mark.parametrize("reported_status,error", [
    ("running", None), ("failed", None), ("complete", "renderer cleanup failed"),
])
def test_clean_exit_cannot_override_incomplete_or_failed_report(manager, reported_status, error):
    script = r'''
import argparse,json,time
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('--output');a,_=p.parse_known_args()
folder=Path(a.output);folder.mkdir()
time.sleep(.3)
(folder/'summary.json').write_text(json.dumps(dict(status=REPORTED_STATUS,error=REPORTED_ERROR,
    attempts=[],accepted_episodes=['episode-one','episode-two'])))
'''.replace('REPORTED_STATUS', repr(reported_status)).replace('REPORTED_ERROR', repr(error))
    (manager.project_root / 'fake.py').write_text(script, encoding='utf-8')
    job = manager.start(CollectionCreate(workers=1, count=2, max_attempts=2), OWNER, {})
    result = wait_job(manager, job['id'])
    assert result['status'] == 'failed'
    assert result['success_count'] == 2
    assert result['workers'][0]['error']


def test_child_failure_before_first_report_has_a_useful_error(manager):
    (manager.project_root / 'fake.py').write_text('import time\ntime.sleep(.3)\nraise SystemExit(1)\n')
    job = manager.start(CollectionCreate(workers=1, count=1, max_attempts=1), OWNER, {})
    result = wait_job(manager, job['id'])
    assert result['status'] == 'failed' and result['success_count'] == 0
    assert '进度' in result['workers'][0]['error']
    assert not result.get('error')  # The worker failed; the manager itself did not crash.


def test_manager_failure_recovers_acceptance_written_during_cleanup(manager, monkeypatch, exiting_collector):
    refresh = manager._refresh
    failed = False

    def fail_after_child_finishes(job, worker):
        nonlocal failed
        refresh(job, worker)
        if exiting_collector and not failed:
            failed = True
            raise RuntimeError('injected coordinator failure after child acceptance')

    monkeypatch.setattr(manager, '_refresh', fail_after_child_finishes)
    job = manager.start(CollectionCreate(workers=1, count=1, max_attempts=1, seed=30), OWNER, {})
    result = wait_job(manager, job['id'])
    assert failed and result['status'] == 'failed'
    assert result['success_count'] == 1
    assert 'injected coordinator failure' in result['error']
