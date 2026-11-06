"""Long missions pass the former infrastructure cutoff; hangs still stop without retries."""
import json
from types import SimpleNamespace
import subprocess

import pytest

from space_arm_platform import benchmark_batch as batch
from space_arm_platform.benchmark_contract import SPEC_DIRECTORY, load_task


@pytest.mark.parametrize('configuration,expected',[
    ('d05-cooperative-far-rendezvous-v1',12000),('s10-continuous-low-thrust-v1',3600),
    ('s12-gnss-orbit-plane-v1',2400),('s02-wheel-attitude-v1',900)])
def test_budget_is_declared_before_outcomes_and_preserves_physical_task(configuration,expected):
    task = load_task(SPEC_DIRECTORY/(configuration+'.json'))
    before = task.fingerprint()
    assert batch.trial_wall_time_limit_s(task) == expected
    assert task.fingerprint() == before


def fake_worker(monkeypatch, finish_at):
    clock = [0.]
    lifecycle = dict(launches=0,closed_at=None,closed=False)
    class Child:
        pid = 123456789
        returncode = 0
        def poll(self):
            return 0 if lifecycle['closed'] or finish_at is not None and clock[0] >= finish_at else None
        def wait(self,timeout):
            if self.poll() is not None: return 0
            clock[0] += 100
            raise subprocess.TimeoutExpired('owned test worker',timeout)
    def close():
        lifecycle.update(closed=True,closed_at=clock[0])
    def launch(*args,**kwargs):
        lifecycle['launches'] += 1
        return Child(),SimpleNamespace(created=1.,close=close)
    monkeypatch.setattr(batch,'time',SimpleNamespace(monotonic=lambda:clock[0]))
    monkeypatch.setattr(batch,'launch_collector',launch)
    monkeypatch.setattr(batch,'source_identity',lambda root:{'source_sha256':{'checked.py':'fixed'}})
    monkeypatch.setattr(batch,'runtime_identity',lambda *args,**kwargs:{'fingerprint':'fixed-test-runtime'})
    return lifecycle


def test_far_worker_is_not_killed_by_old_900_second_cutoff(tmp_path,monkeypatch):
    task = load_task(SPEC_DIRECTORY/'d05-cooperative-far-rendezvous-v1.json')
    lifecycle = fake_worker(monkeypatch,finish_at=1300.)
    # This isolates infrastructure scheduling, not native simulation or scoring.
    monkeypatch.setattr(batch,'_completed_trial',lambda directory,trial,*args,**kwargs:{**trial,'outcome':'success'})
    result = batch.run_batch(task,None,'train',tmp_path/'long-mission',seed=1)
    plan = json.loads((tmp_path/'long-mission/plan.json').read_text())
    assert plan['wall_time_limit_s'] == 12000 and result['outcomes'] == {'success':1}
    assert lifecycle['closed_at'] == 1300 and lifecycle['launches'] == 1


def test_hang_keeps_timeout_evidence_and_resume_never_retries(tmp_path,monkeypatch):
    task = load_task(SPEC_DIRECTORY/'d05-cooperative-far-rendezvous-v1.json')
    lifecycle = fake_worker(monkeypatch,finish_at=None)
    output = tmp_path/'hung-mission'
    result = batch.run_batch(task,None,'train',output,seed=1)
    assert result['outcomes'] == {'infrastructure_error':1} and result['planned'] == result['finished'] == 1
    trial = result['trials'][0]
    assert trial['detail'] == 'trial wall-time limit'
    assert trial['wall_time_limit_s'] == 12000 and trial['wall_time_elapsed_s'] == 12100
    assert lifecycle['closed_at'] == 12100
    repeated = batch.run_batch(task,None,'train',output,seed=1,resume=True)
    assert repeated == result and lifecycle['launches'] == 1
    plan = json.loads((output/'plan.json').read_text())
    plan['wall_time_limit_s'] = 900
    (output/'plan.json').write_text(json.dumps(plan))
    with pytest.raises(ValueError,match='frozen batch plan'):
        batch.run_batch(task,None,'train',output,seed=1,resume=True)
    assert lifecycle['launches'] == 1
