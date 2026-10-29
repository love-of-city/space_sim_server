"""The scheduler must preserve one layout's appearance and never retry a bad task."""
import json
from copy import deepcopy
from pathlib import Path
from types import SimpleNamespace

import pytest
from pydantic import ValidationError

from collection import sequence
from collection import collect_scripted_tasks as batch
from space_arm_platform.collection_layouts import LayoutPool
from space_arm_platform.overview_camera import validate_overview_camera
from simulation.scripted_tasks.randomization import prepare_episode
from test_collection_layouts import layout_pool


def config(pool, **kwargs):
    return sequence.SequenceConfig(layout_pool=str(pool), **kwargs)


def test_full_pass_visits_each_layout_once_and_keeps_sixth_task_available(layout_pool):
    pool = LayoutPool(layout_pool)
    cfg = config(layout_pool, passes=2)
    plan = sequence.make_plan(cfg, pool)
    assert plan == sequence.make_plan(cfg, pool)
    assert plan['total_attempts'] == len(pool.document['layouts']) * 2 * 5
    for pass_number in (1, 2):
        groups = [g for g in plan['groups'] if g['pass_number'] == pass_number]
        assert sorted(g['pool_index'] for g in groups) == list(range(len(pool.document['layouts'])))
        assert all([j['task'] for j in g['jobs']] == list(sequence.TASK_NAMES[:5]) for g in groups)
    cfg.tasks[-1].enabled = True
    assert sequence.make_plan(cfg, pool)['task_order'][-1] == 'aviation_bolt_insert'


def test_tasks_keep_colors_camera_and_change_arm_seed_without_reentry_resampling(layout_pool):
    pool = LayoutPool(layout_pool)
    cfg = config(layout_pool)
    cfg.tasks[-1].enabled = True
    source = sequence.source_scene(cfg)
    groups = sequence.make_plan(cfg, pool)['groups']
    group = groups[0]
    layout = sequence.prepare_layout(source, group, pool)
    scenes = [sequence.prepare_task_scene(source, cfg, group, j, layout) for j in group['jobs']]
    for scene, job in zip(scenes, group['jobs']):
        assert scene['seed'] == job['seed']
        assert scene['collection_config']['initialization']['half_range_deg'] == 30
        assert max(abs(v) for v in scene['collection_config']['initialization']['offset_deg']) <= 30
        assert scene['overview_camera'] == layout['overview_camera']
        assert validate_overview_camera(scene) == layout['overview_camera']
        assert scene['collection_config']['episode_color_assignment'] == layout['color_assignment']
        assert scene['task_box_layout'] == layout['placement']
        before = deepcopy(scene)
        prepare_episode(scene)
        assert scene == before
        assert scene['collection_config']['instruction_randomization']['instruction'].isascii()
    assert len({tuple(s['randomization']['arm_joint_position_rad']) for s in scenes}) == 6
    assert len(set(layout['color_assignment']['hole_colors'].values())) == 7
    scenes[0]['task_box_layout']['model_sha256'] = 'mutated'
    assert scenes[1]['task_box_layout'] == layout['placement']


@pytest.mark.parametrize('invalid', [dict(tasks=[]), dict(tasks=[{'name':'bad'}]),
    dict(range_deg=float('nan')), dict(layouts_per_pass=0),
    dict(tasks=[{'name':'drawer_cycle'}, {'name':'drawer_cycle'}])])
def test_bad_settings_are_rejected(invalid):
    with pytest.raises(ValidationError):
        config('pool.json', **invalid)


class Session:
    def __init__(self, output):
        self.closed = False
    def close(self):
        self.closed = True


def test_failed_tasks_continue_next_task_and_layout_and_resume_does_not_repeat(layout_pool, tmp_path):
    pool = LayoutPool(layout_pool)
    cfg = config(layout_pool)
    source = sequence.source_scene(cfg)
    plan = sequence.make_plan(cfg, pool)
    calls = []
    sessions = []
    def factory(output):
        session = Session(output)
        sessions.append(session)
        return session
    def runner(c, source_path, out, group, job, layout, scene, session):
        assert session is sessions[0] and not session.closed
        calls.append(job['id'])
        return dict(status='skipped' if job['task'] in ('drawer_cycle','guide_plug_insert') else 'accepted')
    progress = sequence.execute_plan(cfg, pool, plan, source, tmp_path, runner=runner, session_factory=factory)
    assert calls == [j['id'] for g in plan['groups'] for j in g['jobs']]
    assert progress['status'] == 'complete'
    assert progress['accepted'] == 3*len(plan['groups'])
    assert progress['skipped'] == 2*len(plan['groups'])
    assert sessions[0].closed
    sequence.execute_plan(cfg, pool, plan, source, tmp_path, resume=True, runner=runner, session_factory=factory)
    assert len(calls) == plan['total_attempts']


def test_infrastructure_failure_persists_and_resume_skips_interrupted_job(layout_pool, tmp_path):
    pool = LayoutPool(layout_pool)
    cfg = config(layout_pool)
    source = sequence.source_scene(cfg)
    plan = sequence.make_plan(cfg, pool)
    def fail(c, source_path, out, group, job, *args):
        episode = out/job['relative_output']/'episodes'/'unfinished'
        episode.mkdir(parents=True)
        (episode/'evidence.txt').write_text('partial capture')
        raise OSError('disk full')
    with pytest.raises(OSError, match='disk full'):
        sequence.execute_plan(cfg, pool, plan, source, tmp_path, runner=fail, session_factory=Session)
    progress = json.loads((tmp_path/'progress.json').read_text())
    assert progress['status'] == 'infrastructure_failed'
    first = plan['groups'][0]['jobs'][0]
    folder = tmp_path/first['relative_output']
    assert not (folder/'episodes').exists()
    assert (folder/'rejected'/'unfinished'/'evidence.txt').read_text() == 'partial capture'
    called = []
    def run(c, source_path, out, group, job, *args):
        called.append(job['id'])
        return dict(status='accepted')
    result = sequence.execute_plan(cfg, pool, plan, source, tmp_path, resume=True, runner=run, session_factory=Session)
    assert len(called) == plan['total_attempts']-1
    assert plan['groups'][0]['jobs'][0]['id'] not in called
    assert result['status'] == 'complete' and result['skipped'] == 1
    assert 'error' not in result


def test_shared_batch_session_reuses_renderer_dynamics_and_identical_probe(tmp_path, monkeypatch):
    from collection import record_scripted_task as recorder
    from simulation.scripted_tasks import randomization
    source, model, catalog = (tmp_path/n for n in ('scene.json','model.xml','catalog.json'))
    source.write_text('{}'); model.write_text('<mujoco/>'); catalog.write_text('{}')
    created, closed, commands, ports = [], [], [], []
    class Probe:
        def __init__(self, path): created.append('probe')
        def validate(self, model, scene, task): return dict(accepted=True)
        def close(self): closed.append('probe')
    class Renderer:
        def __init__(self, *args):
            created.append('renderer'); self.process=SimpleNamespace(pid=101)
        def check(self): pass
        def close(self): closed.append('renderer')
    class Dynamics:
        def __init__(self, *args, **kwargs):
            created.append('dynamics'); self.directory=tmp_path/'worker'; self.pid=102
        def reusable(self): return True
        def close(self): closed.append('dynamics')
    def popen(command, **kwargs):
        commands.append(command)
        out=Path(command[command.index('--output')+1]); out.mkdir()
        (out/'summary.json').write_text(json.dumps(dict(episode_id='episode', task=dict(result='success'))))
        return SimpleNamespace(returncode=0, poll=lambda:0)
    monkeypatch.setattr(batch, 'NativePoseProbe', Probe)
    monkeypatch.setattr(batch, 'PersistentRenderer', Renderer)
    monkeypatch.setattr(batch, 'PersistentDynamics', Dynamics)
    monkeypatch.setattr(batch, 'check_port_available', ports.append)
    monkeypatch.setattr(batch, 'validate_recording', lambda *a: {})
    monkeypatch.setattr(batch.subprocess, 'Popen', popen)
    monkeypatch.setattr(recorder, 'recording_settings', lambda *a: None)
    monkeypatch.setattr(randomization, 'prepare_episode', lambda s:s)
    monkeypatch.setattr(randomization, 'write_model_variant', lambda m,*a:m)
    session = batch.CollectionSession(tmp_path)
    for i, task in enumerate(('drawer_cycle', 'aviation_plug_insert')):
        scene=dict(seed=i, collection_config=dict(task=task, initialization={}), task_box_layout={})
        assert batch.main(argv=['--task', task, '--scene', str(source), '--model', str(model),
            '--catalog', str(catalog), '--output', str(tmp_path/f'task-{i}'),
            '--episodes-root',str(tmp_path/'episodes'), '--seed',str(i), '--count','1', '--max-attempts','1',
            '--reuse-renderer','--reuse-dynamics','--warmup-seconds','5','--reuse-warmup-seconds','2'],
            session=session, prepared_scene=scene) == 0
        assert not closed
    assert created == ['probe','renderer','dynamics']
    assert len(ports) == 7  # Shared renderer's occupied port isn't treated as a conflict.
    assert [c[c.index('--warmup-seconds')+1] for c in commands] == ['5.0','2.0']
    assert session.recorded == 2
    session.close()
    assert sorted(closed) == ['dynamics','probe','renderer']
