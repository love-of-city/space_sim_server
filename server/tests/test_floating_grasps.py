"""Selection keeps object poses fixed and binds the executed local grasp frame."""
from copy import deepcopy
from dataclasses import replace
import numpy as np
import pytest
from test_floating_capture import realized, sample_at_grasp
from simulation.scripted_tasks.floating_assets import FLOATING_TASKS
from simulation.scripted_tasks.floating_capture import create_floating_task
from simulation.scripted_tasks.floating_grasps import (
    grasp_candidates, requested_candidates, select_grasp, validate_selection,
)


class Probe:
    names = {1: "bolt", 2: "ee_finger1", 3: "ee_finger2"}
    def __init__(self, depth=1e-7):
        self.depth, self.carry_poses = depth, []
    def pose_collision(self, q, name, **kwargs):
        joints = kwargs.get('joint_values', {})
        if 'bolt_freejoint' in joints:
            self.carry_poses.append(joints['bolt_freejoint'])
        return None
    def grasp_opening(self, q, body):
        return .5
    def contacts(self, q):
        return np.array([[0,1,1,2,-self.depth],[0,2,1,3,0.]]) if q[6] <= .5 else np.empty((0,5))
    def body_pose(self, name):
        return np.zeros(3), np.eye(3)


def test_registry_has_stable_unique_candidates_for_every_object():
    for name in FLOATING_TASKS:
        choices = grasp_candidates(name)
        assert choices[0].id == 'baseline'
        assert len({c.id for c in choices}) == len(choices) >= 4
    with pytest.raises(ValueError, match='Invalid candidate_ids'):
        requested_candidates('floating_bolt_capture', {'floating_grasp': {'candidate_ids':['missing']}})


def test_fixed_scene_selection_freezes_candidate_and_rejects_changed_context(tmp_path):
    name = 'floating_bolt_capture'
    scene, model = realized(tmp_path, name, 75)
    scene['floating_grasp'] = {'candidate_ids':['baseline']}
    before = deepcopy(scene['collection_config']['floating_initial_state'])
    probe = Probe()
    result = select_grasp(probe, model, scene, name)
    assert result['accepted'], result
    assert scene['collection_config']['floating_initial_state'] == before
    record = deepcopy(scene['collection_config']['floating_grasp'])
    assert select_grasp(Probe(), model, scene, name)['replay']
    assert scene['collection_config']['floating_grasp'] == record
    task = create_floating_task(name, model, scene)
    assert task.grasp.id == 'baseline'
    assert probe.carry_poses and np.linalg.norm(np.subtract(probe.carry_poses[0][:3], probe.carry_poses[-1][:3])) > .05
    scene['floating_grasp']['selection'] = 'random'
    with pytest.raises(ValueError, match='Saved floating grasp'):
        validate_selection(name, model, scene)


def test_deep_one_sided_overlap_is_not_a_feasible_grasp(tmp_path):
    scene, model = realized(tmp_path, seed=75)
    scene['floating_grasp'] = {'candidate_ids':['baseline']}
    report = select_grasp(Probe(depth=.01), model, scene, 'floating_bolt_capture')
    assert not report['accepted']
    assert report['candidates'][0]['reason'] == 'asymmetric_contact'
    assert 'floating_grasp' not in scene['collection_config']


def test_seeded_random_selection_is_repeatable_without_resampling_object(tmp_path):
    scene, model = realized(tmp_path, seed=75)
    scene['floating_grasp'] = {'selection':'random', 'candidate_ids':['baseline','deep']}
    other = deepcopy(scene)
    a = select_grasp(Probe(), model, scene, 'floating_bolt_capture')
    b = select_grasp(Probe(), model, other, 'floating_bolt_capture')
    assert a == b and a['accepted']
    assert scene == other


def test_pregrasp_waits_at_offset_before_interception(tmp_path):
    scene, model = realized(tmp_path, seed=75)
    task = create_floating_task('floating_bolt_capture', model)
    sample = sample_at_grasp(task, scene)
    task.stage = 'pregrasp'
    task.step(sample)
    assert task.stage == 'pregrasp'  # already at TCP is not at the pregrasp waypoint
    point, rotation = task.grasp_pose(sample)
    from simulation.scripted_tasks.motion import solve_pose
    q = solve_pose(task.chain, point-task.grasp.pregrasp_m*rotation[:,2], rotation,
                   sample.joints[:6], task.lower, task.upper)
    sample = replace(sample, joints=np.r_[q,1.,-1.], time_s=.005)
    task.step(sample)
    assert task.stage == 'intercept'
