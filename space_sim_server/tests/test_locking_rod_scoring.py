from copy import deepcopy

import numpy as np

from simulation.evaluation.locking_rod import LockingRodScorer


def observations():
    base = dict(joint_position=[0.]*6+[.39, -.39], wrench=[0.]*6, wrench_valid=True,
        contacts=[0, 0], rod_angle_rad=0., rod_velocity_rad_s=0., fastener_slide_m=0.,
        fastener_spin_rad=0., fastener_spin_velocity_rad_s=0., hook_normal_force_n=0.,
        tool_fastener_distance_m=.02)
    index = 0
    for seconds, changes in [(.2, {}), (.3, dict(contacts=[2, 2])),
            (.5, dict(rod_angle_rad=-.5)), (.5, dict(rod_angle_rad=0.)),
            (.5, dict(fastener_slide_m=.005, fastener_spin_rad=.005*2*np.pi/.016,
                      hook_normal_force_n=2.)),
            (2.5, dict(contacts=[0, 0], joint_position=[0.]*6+[1., -1.],
                       tool_fastener_distance_m=.15))]:
        base.update(changes)
        for _ in range(round(seconds*200)):
            yield dict(deepcopy(base), time_s=index/200.)
            index += 1


def test_measured_cycle_and_release_are_required():
    scorer = LockingRodScorer()
    for row in observations():
        scorer.update(row)
    assert scorer.report()['success'] is True
    assert scorer.report()['measured_milestones'] == 4
    row['time_s'] += .005
    row['hook_normal_force_n'] = 0.
    scorer.update(row)
    assert scorer.report()['success'] is False


def test_expert_success_without_fold_cannot_pass():
    scorer = LockingRodScorer()
    for row in observations():
        row.update(rod_angle_rad=0., stage='success', result='success', accepted=True)
        scorer.update(row)
    assert scorer.report()['measured_milestones'] == 1
    assert scorer.report()['success'] is False


def test_force_dwell_rejection_persists_after_later_success():
    scorer = LockingRodScorer()
    for row in observations():
        if 1. < row['time_s'] < 1.2:
            row['wrench'][0] = 41.
        scorer.update(row)
    assert scorer.report()['reason'] == 'wrist_overload'
    assert scorer.report()['success'] is False


def test_missing_samples_and_unavailable_wrench_are_unscorable():
    for field, value in [('time_s', .02), ('wrench_valid', False)]:
        rows = iter(observations())
        scorer = LockingRodScorer()
        scorer.update(next(rows))
        row = next(rows)
        row[field] = value
        scorer.update(row)
        assert scorer.report()['status'] == 'unscorable'


def test_zero_preload_or_unreleased_fingers_cannot_prove_lock():
    for mutation in [dict(hook_normal_force_n=0.), dict(contacts=[1, 1])]:
        scorer = LockingRodScorer()
        for row in observations():
            row.update(mutation)
            scorer.update(row)
        assert scorer.report()['success'] is False
