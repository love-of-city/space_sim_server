"""Estimated gate completion never overrides independent physical limits."""
from copy import deepcopy

from tools.diagnostics.score_vbar_near23 import node_window


def samples():
    return [dict(time_s=i*.05, true_com_position_m=[50., 0., 0.],
                 true_com_velocity_m_s=[0., 0., 0.], true_center_gripper_m=[0., 0., -.5])
            for i in range(101)]


def test_scores_entire_continuous_window_and_both_platform_errors():
    rows = samples()
    assert node_window(rows, 5., [50., 0., 0.], tool_gap=.5)['passed']
    bad = deepcopy(rows)
    bad[40]['true_center_gripper_m'][0] = .051
    assert not node_window(bad, 5., [50., 0., 0.], tool_gap=.5)['passed']
    bad = deepcopy(rows)
    bad[40]['true_com_velocity_m_s'][0] = .0101
    assert not node_window(bad, 5., [50., 0., 0.])['passed']
    bad = deepcopy(rows)
    bad[40]['true_com_position_m'][1] = .0501
    assert not node_window(bad, 5., [50., 0., 0.])['passed']


def test_gap_and_missing_completion_are_not_passes():
    rows = samples()
    assert not node_window(rows, None, None)['passed']
    rows.pop(40)
    assert not node_window(rows, 5., [50., 0., 0.])['passed']
