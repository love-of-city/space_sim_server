"""Real Linux processes: exit, crash, detached descendants and owner isolation.

Run with --confcutdir=tests/linux, without the recorder fixture or a
simulation/GPU environment.
"""

import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import psutil
import pytest

from space_arm_platform.collection_processes import kill_owned_tree, launch_collector


TREE = r'''
import json, os, subprocess, sys, time
from pathlib import Path
import psutil
folder = Path(sys.argv[1])
def record(name):
    (folder / (name + '.json')).write_text(json.dumps(dict(
        pid=os.getpid(), created=psutil.Process().create_time())))
role = sys.argv[2]
record(role)
if role != 'leaf':
    next_role = 'child' if role == 'worker' else 'leaf'
    subprocess.Popen([sys.executable, __file__, str(folder), next_role], start_new_session=True)
if role == 'worker':
    while not (folder / 'leaf.json').exists(): time.sleep(.01)
    (folder / 'ready').touch()
    while not (folder / 'finish').exists(): time.sleep(.01)
    raise SystemExit(int((folder / 'finish').read_text()))
time.sleep(120)
'''


def wait_for(predicate, message, timeout=15):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(.02)
    pytest.fail(message)


def alive(record):
    try:
        process = psutil.Process(record['pid'])
        return (abs(process.create_time() - record['created']) < .01
                and process.status() != psutil.STATUS_ZOMBIE)
    except psutil.NoSuchProcess:
        return False


def identities(folder):
    return [json.loads((folder / (name + '.json')).read_text())
            for name in ('worker', 'child', 'leaf')]


def assert_tree_reaped(records):
    # The guard is the subreaper, so these must be reaped, not merely zombies.
    wait_for(lambda: all(not psutil.pid_exists(r['pid']) for r in records),
             'collector descendants survived their lifetime owner')


@pytest.fixture
def tree(tmp_path):
    script = tmp_path / 'worker with spaces.py'
    script.write_text(TREE)
    groups = []

    def start(name='batch'):
        folder = tmp_path / name
        folder.mkdir()
        process, group = launch_collector(
            [sys.executable, str(script), str(folder), 'worker'], cwd=tmp_path,
        )
        groups.append((group, folder))
        wait_for(lambda: (folder / 'ready').exists(), 'worker did not create its tree')
        return process, group, folder, identities(folder)

    yield start
    for group, folder in groups:
        try:
            group.close()
        finally:
            # Cleanup only processes created by this test if an assertion fails.
            for path in folder.glob('*.json'):
                record = json.loads(path.read_text())
                if alive(record):
                    kill_owned_tree(record['pid'], record['created'])


@pytest.mark.parametrize('code', [0, 1, 7])
def test_worker_exit_reaps_detached_children_and_keeps_exit_code(tree, code):
    process, group, folder, records = tree()
    assert all(alive(r) for r in records)
    assert os.getsid(records[1]['pid']) != os.getsid(process.pid)
    assert os.getsid(records[2]['pid']) != os.getsid(process.pid)
    (folder / 'finish').write_text(str(code))
    assert process.wait(timeout=15) == code
    assert_tree_reaped(records)
    group.close()
    group.close()


@pytest.mark.parametrize('signum', [signal.SIGTERM, signal.SIGKILL])
def test_worker_signal_exit_reaps_tree_and_preserves_signal(tree, signum):
    process, group, folder, records = tree()
    psutil.Process(records[0]['pid']).send_signal(signum)
    assert process.wait(timeout=15) == -signum
    assert_tree_reaped(records)
    group.close()


def test_close_one_batch_does_not_touch_another_batch_or_owner(tree):
    first, first_group, _, first_records = tree('first')
    second, second_group, folder, second_records = tree('second')
    first_group.close()
    assert first.returncode != 0
    assert_tree_reaped(first_records)
    assert second.poll() is None
    assert all(alive(r) for r in second_records)
    (folder / 'finish').write_text('0')
    assert second.wait(timeout=15) == 0
    assert_tree_reaped(second_records)
    second_group.close()


def test_recovery_uses_guard_identity_and_reaps_detached_children(tree):
    process, group, _, records = tree()
    kill_owned_tree(process.pid, group.created + 10, guarded=True)
    assert process.poll() is None
    assert all(alive(r) for r in records)
    kill_owned_tree(process.pid, group.created, guarded=True)
    process.wait(timeout=15)
    assert_tree_reaped(records)
    group.close()


def test_short_lived_orphan_is_reaped_while_worker_keeps_running(tmp_path):
    script = tmp_path / 'orphan.py'
    script.write_text(r'''
import json, subprocess, sys, time
from pathlib import Path
import psutil
folder = Path(sys.argv[1])
if len(sys.argv) == 2:
    subprocess.run([sys.executable, __file__, str(folder), 'middle'], check=True)
    (folder / 'ready').touch()
    while not (folder / 'finish').exists(): time.sleep(.01)
    raise SystemExit(7)
child = subprocess.Popen([sys.executable, '-c', 'import time;time.sleep(.3)'], start_new_session=True)
(folder / 'leaf.json').write_text(json.dumps(dict(pid=child.pid, created=psutil.Process(child.pid).create_time())))
''')
    process, group = launch_collector([sys.executable, str(script), str(tmp_path)])
    try:
        wait_for(lambda: (tmp_path / 'ready').exists(), 'worker did not orphan its child')
        leaf = json.loads((tmp_path / 'leaf.json').read_text())
        assert_tree_reaped([leaf])
        assert process.poll() is None
        (tmp_path / 'finish').touch()
        assert process.wait(timeout=15) == 7
    finally:
        group.close()


def test_coordinator_sigkill_reaps_tree_without_backend_restart(tmp_path):
    script = tmp_path / 'worker.py'
    script.write_text(TREE)
    coordinator = tmp_path / 'coordinator.py'
    coordinator.write_text(r'''
import json, sys, time
from pathlib import Path
from space_arm_platform.collection_processes import launch_collector
folder = Path(sys.argv[1])
process, group = launch_collector([sys.executable, sys.argv[2], str(folder), 'worker'])
(folder / 'guard.json').write_text(json.dumps(dict(pid=process.pid, created=group.created)))
time.sleep(120)
''')
    env = dict(os.environ)
    env['PYTHONPATH'] = str(Path(__file__).resolve().parents[2] / 'backend')
    with (tmp_path / 'coordinator.log').open('w') as log:
        owner = subprocess.Popen([sys.executable, str(coordinator), str(tmp_path), str(script)],
                                 env=env, stdout=log, stderr=subprocess.STDOUT)
        records = []
        guard = None
        try:
            wait_for(lambda: (tmp_path / 'ready').exists(), 'coordinator did not launch collector')
            records = identities(tmp_path)
            guard = json.loads((tmp_path / 'guard.json').read_text())
            owner.kill()
            owner.wait(timeout=10)
            assert_tree_reaped(records)
            wait_for(lambda: not alive(guard), 'supervisor survived coordinator SIGKILL')
        finally:
            if owner.poll() is None:
                owner.kill()
            owner.wait(timeout=10)
            if guard is not None and alive(guard):
                kill_owned_tree(guard['pid'], guard['created'], guarded=True)
            for record in records:
                if alive(record):
                    kill_owned_tree(record['pid'], record['created'])


@pytest.mark.parametrize('code', [0, 1, 2])
def test_immediate_exit_and_execution_environment(tmp_path, code):
    with (tmp_path / 'output.log').open('w') as log:
        process, group = launch_collector(
            [sys.executable, '-c',
             "import os,sys;print(os.getcwd());print(os.environ['COLLECTOR_TEST']);sys.exit(int(sys.argv[1]))",
             str(code)], cwd=tmp_path, env=dict(os.environ, COLLECTOR_TEST='仿真'),
            stdout=log, stderr=subprocess.STDOUT,
        )
        try:
            assert process.wait(timeout=15) == code
        finally:
            group.close()
    assert (tmp_path / 'output.log').read_text().splitlines() == [str(tmp_path), '仿真']


def test_failed_launch_closes_pipe_and_allows_next_worker(tmp_path):
    owner = psutil.Process()
    before = owner.num_fds()
    for _ in range(3):
        with pytest.raises(FileNotFoundError):
            launch_collector([sys.executable, '-c', 'pass'], cwd=tmp_path / 'absent')
    assert owner.num_fds() == before
    with (tmp_path / 'error.log').open('w') as log:
        process, group = launch_collector([str(tmp_path / 'missing-command')],
                                         stdout=log, stderr=subprocess.STDOUT)
        try:
            assert process.wait(timeout=15) == 127
        finally:
            group.close()
    assert 'FileNotFoundError' in (tmp_path / 'error.log').read_text()
    assert owner.num_fds() == before
    process, group = launch_collector([sys.executable, '-c', 'pass'])
    try:
        assert process.wait(timeout=15) == 0
    finally:
        group.close()
    assert owner.num_fds() == before


@pytest.mark.parametrize('mode', ['complete', 'cancel', 'crash'])
def test_manager_terminal_state_follows_cleanup_and_survives_reload(tmp_path, monkeypatch, mode):
    from space_arm_platform.collection_jobs import (
        CollectionCreate, CollectionManager, CollectionTask, TASKS, TERMINAL,
    )

    project = tmp_path / 'project'
    project.mkdir()
    script = project / 'collector.py'
    script.write_text(r'''
import argparse, json, os, signal, subprocess, sys, time
from pathlib import Path
import psutil
p = argparse.ArgumentParser()
for name in ('output', 'cancel-file', 'seed'): p.add_argument('--' + name)
a, _ = p.parse_known_args()
folder = Path(a.output); folder.mkdir()
child = subprocess.Popen([sys.executable, '-c', 'import time;time.sleep(120)'], start_new_session=True)
(folder / 'child.json').write_text(json.dumps(dict(pid=child.pid, created=psutil.Process(child.pid).create_time())))
(folder / 'ready').touch()
while not (folder / 'finish').exists() and not Path(a.cancel_file).exists(): time.sleep(.01)
if int(a.seed) == 3: os.kill(os.getpid(), signal.SIGKILL)
cancelled = Path(a.cancel_file).exists()
(folder / 'summary.json').write_text(json.dumps(dict(
    status='cancelled' if cancelled else 'complete', attempts=[],
    accepted_episodes=[] if cancelled else ['episode-kept'])))
raise SystemExit(1 if cancelled else 0)
''')
    adapter = tmp_path / 'adapter'
    catalog = adapter / 'Unreal/BskUnrealRenderer/Saved/AssetImport/sarm_20261012.catalog.json'
    catalog.parent.mkdir(parents=True)
    catalog.touch()
    unreal = tmp_path / 'unreal'
    editor = unreal / 'Engine/Binaries/Linux/UnrealEditor-Cmd'
    editor.parent.mkdir(parents=True)
    editor.touch()
    editor.chmod(0o755)
    monkeypatch.setitem(TASKS, 'drawer_cycle', CollectionTask('fake', script.name, 'fake'))
    owner = dict(user_id='owner', username='alice', role='operator')
    manager = CollectionManager(project, tmp_path / 'data/episodes', adapter, unreal)
    recovered = None
    try:
        job = manager.start(CollectionCreate(workers=1, count=1, max_attempts=1,
                                            seed=3 if mode == 'crash' else 1), owner, {})
        folder = manager.root / job['id'] / 'worker-0'
        wait_for(lambda: (folder / 'ready').exists(), 'manager did not launch collector')
        child = json.loads((folder / 'child.json').read_text())
        if mode == 'cancel':
            manager.stop(job['id'], owner)
        else:
            (folder / 'finish').touch()
        wait_for(lambda: manager.get(job['id'], owner)['status'] in TERMINAL,
                 'manager did not publish a terminal state')
        result = manager.get(job['id'], owner)
        expected = dict(complete='complete', cancel='cancelled', crash='failed')[mode]
        assert result['status'] == expected
        assert result['workers'][0]['process_guard'] == 'linux_subreaper/1'
        assert result['success_count'] == (1 if mode == 'complete' else 0)
        assert_tree_reaped([child])
        manager.close()
        recovered = CollectionManager(project, manager.data_root, adapter, unreal)
        assert recovered.get(job['id'], owner)['status'] == expected
        assert recovered.get(job['id'], owner)['accepted_episodes'] == result['accepted_episodes']
    finally:
        manager.close()
        if recovered is not None:
            recovered.close()
