"""Coordinator timing only; real UE cancellation is validated separately."""
from types import SimpleNamespace
import subprocess

import pytest

from space_arm_platform import benchmark_batch as batch
from space_arm_platform.benchmark_contract import SPEC_DIRECTORY, load_task


@pytest.mark.parametrize('configuration,finish_after,forced', [
    ('d37-stereo-body-approach-v1', 12., False),
    ('d37-stereo-body-approach-v1', 100., True),
    ('s02-wheel-attitude-v1', 12., True),
])
def test_cancellation_allows_render_finalization_but_bounds_a_hung_worker(
        tmp_path, monkeypatch, configuration, finish_after, forced):
    task = load_task(SPEC_DIRECTORY/(configuration+'.json'))
    clock = [100.]
    stop = tmp_path/'stop'
    state = dict(forced=False)

    class Child:
        pid = 123
        returncode = None

        def poll(self):
            if clock[0] >= 100.+finish_after and self.returncode is None:
                self.returncode = 130
            return self.returncode

        def wait(self, timeout):
            if self.poll() is not None:
                return self.returncode
            clock[0] += timeout
            raise subprocess.TimeoutExpired('fixture', timeout)

    child = Child()

    def close():
        state['forced'] = child.poll() is None
        child.returncode = child.returncode or -1

    def launch(*args, **kwargs):
        stop.touch()
        return child, SimpleNamespace(created=1., close=close)

    monkeypatch.setattr(batch, 'time', SimpleNamespace(monotonic=lambda: clock[0]))
    monkeypatch.setattr(batch, 'launch_collector', launch)
    monkeypatch.setattr(batch, 'source_identity', lambda root: {'source_sha256': 'fixture', 'git_revision': 'fixture'})
    monkeypatch.setattr(batch, 'runtime_identity', lambda *a, **k: {'fingerprint': 'fixture', 'errors': []})
    monkeypatch.setattr(batch, '_completed_trial', lambda directory, trial, *a, **k:
        dict(trial, outcome='cancelled', finalized=not state['forced']))
    result = batch.run_batch(task, None, 'train', tmp_path/'batch', seed=1, stop_file=stop)
    assert state['forced'] is forced
    assert result['trials'][0]['finalized'] is (not forced)
    assert result['success_rate_all_planned'] == 0.
    assert clock[0] < 100.+batch.cancellation_grace_s(task)+1.
