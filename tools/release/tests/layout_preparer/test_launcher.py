import argparse
import importlib.util
import json
from pathlib import Path
import sys
import pytest

BASE=Path(__file__).resolve().parents[2]/'layout_preparer'
spec=importlib.util.spec_from_file_location('launcher',BASE/'app/main.py')
launcher=importlib.util.module_from_spec(spec)
spec.loader.exec_module(launcher)

@pytest.fixture(autouse=True)
def isolated_bundle(tmp_path, monkeypatch):
    bundle = tmp_path/'bundle'
    root = bundle/'repo/server'
    model = root/'model/SARM/platform/sarm_20261012.xml'
    model.parent.mkdir(parents=True)
    model.write_text('<mujoco/>')
    monkeypatch.setattr(launcher, 'BASE', bundle)
    monkeypatch.setattr(launcher, 'ROOT', root)


def args(tmp_path, **kwargs):
    values=dict(output=tmp_path/'job',count=1,seed=123,yaw_range_deg=0,command='prepare',resume=False,stage_timeout=10,assets_only=True)
    values.update(kwargs)
    return argparse.Namespace(**values)

def test_stop_and_resume_preserve_completed(tmp_path):
    a=args(tmp_path)
    job=launcher.Job(a)
    job.items=[dict(id='seed-123',trajectory_status='pending')]
    job.state['prepared']=1
    job.save();job.lock.close()
    (a.output/'cancel.request').touch()
    with pytest.raises(ValueError,match='stopped'):
        launcher.Job(a)
    a.resume=True
    resumed=launcher.Job(a)
    assert resumed.items[0]['id']=='seed-123'
    assert not (a.output/'cancel.request').exists()
    resumed.lock.close()

def test_second_process_cannot_own_output(tmp_path):
    a=args(tmp_path)
    first=launcher.Job(a)
    try:
        with pytest.raises(BlockingIOError):
            launcher.Job(a)
    finally:
        first.lock.close()

def test_resume_rejects_changed_configuration(tmp_path):
    a=args(tmp_path)
    first=launcher.Job(a);first.lock.close()
    a.seed+=1
    with pytest.raises(ValueError,match='unchanged'):
        launcher.Job(a)

def test_resume_rejects_changed_source(tmp_path,monkeypatch):
    a=args(tmp_path)
    first=launcher.Job(a);first.lock.close()
    monkeypatch.setattr(launcher,'fingerprints',lambda:{'different':'source'})
    with pytest.raises(ValueError,match='unchanged'):
        launcher.Job(a)

def test_failure_is_not_success(tmp_path):
    job=launcher.Job(args(tmp_path))
    try:
        with pytest.raises(launcher.StageFailure):
            job.run([sys.executable,'-c','raise SystemExit(7)'],job.out,'deliberate_failure')
        row=json.loads((job.out/'timings.jsonl').read_text())
        assert row['returncode']==7
        assert not (job.out/'pool.json').exists()
    finally:
        job.lock.close()

def test_stop_prevents_next_subprocess(tmp_path):
    job=launcher.Job(args(tmp_path))
    try:
        (job.out/'cancel.request').touch()
        with pytest.raises(KeyboardInterrupt):
            job.run([sys.executable,'-c','raise RuntimeError()'],job.out,'must_not_launch')
        assert not (job.out/'must_not_launch.log').exists()
    finally:
        job.lock.close()

def test_unrelated_directory_is_not_overwritten(tmp_path):
    a=args(tmp_path);a.output.mkdir();(a.output/'keep.txt').write_text('keep')
    with pytest.raises(ValueError,match='empty'):
        launcher.Job(a)
    assert (a.output/'keep.txt').read_text()=='keep'
