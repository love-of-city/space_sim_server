"""Exercise the actual process/mailbox boundary, including cleanup and failure."""
import json
import os
from pathlib import Path
import subprocess
import sys
import time

import psutil
import pytest

from space_arm_platform.dynamics_worker import EpisodeProcess


@pytest.fixture
def worker(tmp_path, request):
    script = tmp_path / 'fake_worker.py'
    script.write_text('''from pathlib import Path
import sys,time,json
from space_arm_platform.dynamics_worker import serve
def run(argv):
    seed, output, mode = argv
    if mode == 'crash': raise RuntimeError('injected infrastructure failure')
    if mode == 'leak':
        import threading
        threading.Thread(target=threading.Event().wait, name='leaked-transport', daemon=True).start()
    if mode == 'wait':
        while not Path(output+'.stop').exists(): time.sleep(.02)
        return 1
    import random
    values = [random.Random(int(seed)).random()]
    Path(output).write_text(json.dumps(values))
    return 1 if mode=='failure' else 0
serve(Path(sys.argv[1]), dict(pid=int(sys.argv[2]),created=float(sys.argv[3])), run, **json.loads(sys.argv[4]))
''')
    mailbox = tmp_path / 'mailbox'
    mailbox.mkdir()
    env = os.environ.copy()
    env['PYTHONPATH'] = str(Path(__file__).resolve().parents[1] / 'backend')
    parent = psutil.Process()
    process = subprocess.Popen([sys.executable, str(script), str(mailbox), str(parent.pid),
                                str(parent.create_time()), json.dumps(getattr(request, 'param', {}))], env=env)
    deadline = time.monotonic() + 10
    while not (mailbox / 'ready.json').exists():
        assert process.poll() is None and time.monotonic() < deadline
        time.sleep(.02)
    yield mailbox, process
    (mailbox / 'shutdown').touch()
    try:
        process.wait(5)
    except subprocess.TimeoutExpired:
        process.kill()
        process.wait(5)


def test_sequential_requests_have_unique_results_and_release_mailbox(worker, tmp_path):
    mailbox, process = worker
    requests = []
    for i, seed in enumerate([12, 99, 12]):
        output = tmp_path / f'{i}.json'
        request = EpisodeProcess(mailbox, [str(seed), str(output), 'success'], tmp_path/'stop')
        assert request.wait(10) == 0
        assert request.result['completed_episodes'] == i+1
        # Windows venv launchers may have a different PID from the interpreter.
        assert request.pid == json.loads((mailbox/'ready.json').read_text())['pid']
        assert not (mailbox/'busy').exists()
        requests.append(request)
    assert len({r.request_id for r in requests}) == 3
    assert (tmp_path/'0.json').read_bytes() == (tmp_path/'2.json').read_bytes()
    assert (tmp_path/'0.json').read_bytes() != (tmp_path/'1.json').read_bytes()


def test_recorder_routes_identical_episode_arguments_through_worker(worker, tmp_path):
    from tools.record_drawer_task import ROOT, submit_dynamics

    mailbox, _ = worker
    pids = []
    for index in range(2):
        output = tmp_path / f"record-{index}.json"
        command = [sys.executable, str(ROOT / "tools/run_scripted_task.py"),
                   "12", str(output), "success"]

        def unexpected_launch(*_):
            pytest.fail("Persistent capture must not spawn a second dynamics process")

        episode = submit_dynamics(command, mailbox, tmp_path / "stop", unexpected_launch, {})
        assert episode.wait(10) == 0
        assert episode.result["completed_episodes"] == index + 1
        assert output.exists()
        pids.append(episode.pid)
    assert pids[0] == pids[1]


def test_recorder_dedicated_process_fallback_preserves_command(tmp_path):
    from tools.record_drawer_task import submit_dynamics

    command = ["python", "episode.py", "--seed", "12"]
    handle = object()
    seen = []
    def launch(*args):
        seen.append(args)
        return handle
    assert submit_dynamics(command, None, tmp_path / "stop", launch, {"KEY": "value"}) is handle
    assert seen == [(command, "simulation", {"KEY": "value"})]


@pytest.mark.parametrize('mode,code', [('failure',1),('crash',2)])
def test_failed_episode_retires_worker(worker,tmp_path,mode,code):
    mailbox, process = worker
    request=EpisodeProcess(mailbox,['1',str(tmp_path/'result'),mode],tmp_path/'stop')
    assert request.wait(10)==code
    assert (mailbox/'retired').exists()
    process.wait(5)
    with pytest.raises(RuntimeError,match='not alive'):
        EpisodeProcess(mailbox,[],tmp_path/'stop')


def test_concurrent_request_rejected_and_cancel_is_episode_scoped(worker,tmp_path):
    mailbox, _ = worker
    output=tmp_path/'waiting'
    request=EpisodeProcess(mailbox,['1',str(output),'wait'],Path(str(output)+'.stop'))
    with pytest.raises(FileExistsError):
        EpisodeProcess(mailbox,[],tmp_path/'unrelated-stop')
    with pytest.raises(subprocess.TimeoutExpired):
        request.wait(.05)
    request.terminate()
    assert request.wait(10)==1
    assert not (tmp_path/'unrelated-stop').exists()


def test_stale_result_is_rejected(worker,tmp_path):
    mailbox, _ = worker
    output=tmp_path/'waiting'
    request=EpisodeProcess(mailbox,['1',str(output),'wait'],Path(str(output)+'.stop'))
    request.reply.write_text(json.dumps(dict(request_id='old',token='old',returncode=0)))
    with pytest.raises(RuntimeError,match='Stale'):
        request.poll()
    request.reply.unlink()
    request.terminate()
    assert request.wait(10)==1


def test_incomplete_cleanup_retires_worker_without_reusing_transport(worker, tmp_path):
    mailbox, process = worker
    request = EpisodeProcess(mailbox, ['1', str(tmp_path/'result'), 'leak'], tmp_path/'stop')
    assert request.wait(10) == 0  # Completed sample is valid; its worker is not reusable.
    assert request.result['extra_threads'] == ['leaked-transport']
    assert 'wait' in request.result['extra_thread_details'][0]['stack']
    assert (mailbox/'retired').exists()
    with pytest.raises(RuntimeError):
        EpisodeProcess(mailbox, [], tmp_path/'stop')
    process.wait(5)


def test_worker_death_cannot_be_mistaken_for_success(worker, tmp_path):
    mailbox, process = worker
    request = EpisodeProcess(mailbox, ['1', str(tmp_path/'waiting'), 'wait'], tmp_path/'stop')
    request.kill()
    assert request.wait(10) == 2
    assert request.result is None
    process.wait(5)


def test_shutdown_worker_rejects_new_episode(worker, tmp_path):
    mailbox, _ = worker
    (mailbox/'shutdown').touch()
    with pytest.raises(RuntimeError):
        EpisodeProcess(mailbox, [], tmp_path/'stop')
    assert not (mailbox/'request.json').exists()


@pytest.mark.parametrize('worker,reason,count', [
    ({'max_episodes': 2}, 'episode_limit', 2),
    ({'max_rss_mb': 1}, 'rss_limit', 1),
], indirect=['worker'])
def test_recycling_happens_only_after_completed_episode(worker, tmp_path, reason, count):
    mailbox, process = worker
    for index in range(count):
        request = EpisodeProcess(mailbox, ['1', str(tmp_path/str(index)), 'success'], tmp_path/'stop')
        assert request.wait(10) == 0
        assert (tmp_path/str(index)).exists()
        assert request.result['retired'] == (index == count-1)
    assert request.result['retirement_reasons'] == [reason]
    process.wait(5)
