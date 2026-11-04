"""Regression guards for shared planning state and deferred episode acceptance."""

from collection import recording_validation as validation
import json
from types import SimpleNamespace

import numpy as np
import pytest

from collection import collect_scripted_tasks as collector
from collection.record_scripted_task import InitializationRejected, check_initialization_rejection
from simulation.scripted_tasks import planning_context
from simulation.scripted_tasks.native_geometry_probe import NativePoseProbe


def test_planning_cache_restores_query_state_and_invalidates_changed_content(tmp_path, monkeypatch):
    from simulation import compiled_model_cache, mujoco_plugins, mujoco_runtime
    model = tmp_path/'model.xml'
    model.write_text('first')
    created, closed = [], []

    class Probe:
        def __init__(self, path, **kwargs):
            self.home = np.arange(8, dtype=float)
            self.q, self.v = self.home.copy(), np.zeros(8)
            self.passive_position_offsets = {}
            created.append(self)

        def close(self):
            closed.append(self)

    planning_context.clear_cache()
    monkeypatch.setattr('simulation.scripted_tasks.native_geometry_probe.NativePoseProbe', Probe)
    monkeypatch.setattr(mujoco_plugins, 'ensure_model_plugins', lambda p: None)
    monkeypatch.setattr(compiled_model_cache, 'configure_asset_cache', lambda: None)
    monkeypatch.setattr(compiled_model_cache, 'model_cache_key', lambda p, r: p.read_text())
    monkeypatch.setattr(mujoco_runtime, 'basilisk_mujoco_runtime', lambda: 'runtime')
    try:
        first = planning_context.PlanningContext(model)
        lease = first.acquire()
        lease.home[:] = 42
        lease.q[:] = 77
        lease.v[:] = 99
        lease.passive_position_offsets['finger'] = 1
        lease.close()
        second = planning_context.PlanningContext(model)
        reset = second.acquire()
        assert reset._probe is lease._probe and len(created) == 1 and not closed
        np.testing.assert_array_equal(reset.q, np.arange(8))
        np.testing.assert_array_equal(reset.home, np.arange(8))
        assert not reset.v.any() and not reset.passive_position_offsets
        model.write_text('other')
        changed = planning_context.PlanningContext(model).acquire()
        assert changed._probe is not lease._probe and closed == [lease._probe]
    finally:
        planning_context.clear_cache()


@pytest.mark.parametrize('body,gap,blocked', [
    ('ee_finger1', .001, False), ('ee_finger1', -.0001, True),
    ('wrist_ft_sensor', .001, True), ('link3', .001, True),
])
def test_relaxed_finger_gap_never_exempts_penetration_or_arm(body, gap, blocked):
    probe = NativePoseProbe.__new__(NativePoseProbe)
    probe.q = probe.home = np.zeros(8)
    probe.names = {1: body, 2: 'neighbor'}
    probe.contacts = lambda q: [(1, 2, 1, 2, gap)]
    hit = probe.pose_collision(np.zeros(8), 'free_motion', clearance_m=.002,
                               arm_penetration_m=0., finger_clearance_m=0.)
    assert bool(hit) is blocked


@pytest.mark.parametrize('accepted', [None, True, False])
def test_only_explicit_initial_pose_rejection_is_skippable(tmp_path, accepted):
    (tmp_path/'dynamics').mkdir()
    (tmp_path/'dynamics/summary.json').write_text(json.dumps(
        {'preflight': {'accepted': accepted, 'reason': 'initial_collision'}}))
    if accepted is False:
        with pytest.raises(InitializationRejected, match='initial_collision'):
            check_initialization_rejection(tmp_path)
    else:
        assert check_initialization_rejection(tmp_path) is None


def test_deferred_decode_failure_removes_quick_accepted_episode(tmp_path, monkeypatch):
    attempt, episode = tmp_path/'attempt', tmp_path/'episode'
    attempt.mkdir()
    summary = tmp_path/'summary.json'
    summary.write_text(json.dumps(dict(status='complete', accepted_episodes=[str(episode)],
        attempts=[dict(directory=str(attempt), status='accepted', full_validation_pending=True)])))
    session = collector.CollectionSession(tmp_path)
    session.pending_validations.append((summary, attempt, episode, 'drawer_cycle'))

    def corrupt(*args):
        raise ValueError('video truncated')

    monkeypatch.setattr(validation, 'validate_recording', corrupt)
    results = session.audit_pending()
    saved = json.loads(summary.read_text())
    assert results[0]['passed'] is False and not session.pending_validations
    assert saved['status'] == 'attempt_limit' and not saved['accepted_episodes']
    assert saved['attempts'][0]['status'] == 'validation_failed'
    assert saved['attempts'][0]['full_validation_pending'] is False
