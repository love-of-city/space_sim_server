"""Release clearance must follow measured jaws and the command before retreat."""

from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from simulation.scripted_tasks import drawer_initialization
from simulation.scripted_tasks import plug_clearance


@pytest.fixture
def geometry(monkeypatch):
    class Probe:
        instances = []
        required_opening = .45
        reference_required_opening = .45
        blocked_arm = False

        def __init__(self, model_path, clearance_m=0.):
            self.model_path = model_path
            self.clearance_m = clearance_m
            self.loaded = []
            self.queries = []
            self.closed = False
            self.instances.append(self)

        def use_measured_qpos(self, qpos, joints):
            self.loaded.append((np.asarray(qpos).copy(), np.asarray(joints).copy()))

        def pose_collision(self, candidate, *args, **kwargs):
            q = np.asarray(candidate).copy()
            self.queries.append(q)
            if self.blocked_arm:
                return dict(body1="link4", body2="neighbor_bolt", separation_m=.001)
            needed = self.reference_required_opening if q[0] > .05 else self.required_opening
            if min(q[6], -q[7]) < needed-1e-10:
                return dict(body1="ee_finger2", body2="cylindrical_plug", separation_m=.001)
            return None

        def close(self):
            self.closed = True

    monkeypatch.setattr(drawer_initialization, "NativePoseProbe", Probe)
    return Probe


def task_and_sample(opening=.4):
    reference = np.r_[.1, np.zeros(5), opening, -opening]
    sample = SimpleNamespace(
        time_s=0.,
        joints=np.r_[np.zeros(6), opening, -opening],
        native_qpos=np.arange(50., dtype=float),
    )
    task = SimpleNamespace(
        model_path=Path("unused-private-model.xml"), name="cylindrical_plug_insert",
        config=SimpleNamespace(release_open_rad=.65, plug_body="cylindrical_plug"),
        reference=reference, _release_clearance_probe=None,
        _release_clearance_opening=None, release_clearance_checks=[],
    )
    return task, sample


def set_opening(joints, opening):
    joints[6:] = [opening, -opening]


def test_release_opens_only_to_measured_geometry_clearance_and_reuses_the_probe(geometry):
    task, sample = task_and_sample()
    original_qpos = sample.native_qpos.copy()
    original_joints = sample.joints.copy()
    original_reference = task.reference.copy()
    assert not plug_clearance.cylindrical_release_clearance(task, sample)
    assert len(geometry.instances) == 1
    probe = geometry.instances[0]
    assert probe.clearance_m == pytest.approx(.002)
    threshold = task._release_clearance_opening
    assert .45 <= threshold < task.config.release_open_rad
    np.testing.assert_array_equal(sample.native_qpos, original_qpos)
    np.testing.assert_array_equal(sample.joints, original_joints)
    np.testing.assert_array_equal(task.reference, original_reference)

    # Opening still progresses outside this helper; do not re-query geometry
    # at every control tick while both jaws remain below the known clearance.
    counts = len(probe.queries), len(probe.loaded)
    set_opening(sample.joints, .41)
    set_opening(task.reference, .41)
    assert not plug_clearance.cylindrical_release_clearance(task, sample)
    assert (len(probe.queries), len(probe.loaded)) == counts

    # A command that is clear does not prove that the actual servo has opened.
    set_opening(task.reference, threshold+.001)
    assert not plug_clearance.cylindrical_release_clearance(task, sample)
    set_opening(sample.joints, threshold+.001)
    sample.native_qpos[-1] += .01
    assert plug_clearance.cylindrical_release_clearance(task, sample)
    assert len(geometry.instances) == 1
    assert len(probe.loaded) > counts[1]
    np.testing.assert_array_equal(probe.loaded[-1][0], sample.native_qpos)
    assert {round(q[0], 3) for q in probe.queries[counts[0]:]} == {0., .1}


def test_release_threshold_checks_commanded_arm_as_well_as_actual_arm(geometry):
    geometry.reference_required_opening = .51
    task, sample = task_and_sample()
    assert not plug_clearance.cylindrical_release_clearance(task, sample)
    assert task._release_clearance_opening >= .51
    set_opening(task.reference, task._release_clearance_opening+.001)
    set_opening(sample.joints, task._release_clearance_opening+.001)
    assert plug_clearance.cylindrical_release_clearance(task, sample)


def test_threshold_rechecks_current_geometry_instead_of_using_a_stale_clear_result(geometry):
    task, sample = task_and_sample()
    assert not plug_clearance.cylindrical_release_clearance(task, sample)
    first_threshold = task._release_clearance_opening
    set_opening(sample.joints, first_threshold+.001)
    set_opening(task.reference, first_threshold+.001)
    geometry.required_opening = .55
    assert not plug_clearance.cylindrical_release_clearance(task, sample)
    assert task._release_clearance_opening >= .55


def test_neighbor_arm_collision_is_not_cured_by_fabricating_jaw_clearance(geometry):
    geometry.blocked_arm = True
    task, sample = task_and_sample()
    with pytest.raises(ValueError, match="[Cc]lear|[Bb]lock|[Oo]pen"):
        plug_clearance.cylindrical_release_clearance(task, sample)


def test_probe_cleanup_is_idempotent_and_clears_its_task_handle(geometry):
    task, sample = task_and_sample()
    assert not plug_clearance.cylindrical_release_clearance(task, sample)
    probe = geometry.instances[0]
    plug_clearance.close_cylindrical_release_probe(task)
    assert probe.closed
    assert task._release_clearance_probe is None
    plug_clearance.close_cylindrical_release_probe(task)
