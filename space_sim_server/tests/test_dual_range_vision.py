import copy

import numpy as np
import pytest

from simulation.dual_range_vision import (
    AcceptedImageGate, ImageGateLimits, RangeChannelHandoff, long_focus_candidate,
)


def sample(gate, i, **overrides):
    packet = dict(now_s=i*.1, stamp_s=i*.1, frame_id=i, image_accepted=True,
                  ekf_accepted=True, used_marker_edges_px=dict(left=45., right=42.),
                  depth_sigma_m=.008, navigation_sigma_m=.003)
    packet.update(overrides)
    return gate.observe(**packet)


def test_image_rejection_and_filter_rejection_revoke_continuity():
    for override in [dict(image_accepted=False), dict(ekf_accepted=False),
                     dict(used_marker_edges_px=dict(left=40, right=11)),
                     dict(depth_sigma_m=.03)]:
        gate = AcceptedImageGate()
        assert not any(sample(gate, i) for i in range(9))
        assert not sample(gate, 9, **override)
        assert not any(sample(gate, i) for i in range(10, 19))
        assert sample(gate, 19)


def test_duplicate_stale_and_interrupted_images_never_release():
    gate = AcceptedImageGate(ImageGateLimits(samples=2))
    assert not sample(gate, 0)
    assert sample(gate, 1)
    assert not sample(gate, 2, frame_id=1, stamp_s=.1)
    assert not gate.current(.21, .003)
    assert not sample(gate, 10)
    assert sample(gate, 11)
    assert not gate.current(1.26, .003)
    assert not sample(gate, 13)
    assert not sample(gate, 14, stamp_s=.1)


def test_handoff_needs_qualified_near_stream_and_holds_on_loss():
    router = RangeChannelHandoff()
    assert router.update(far_ready=True, near_ready=False)['active'] == 'far'
    assert router.update(far_ready=False, near_ready=False)['hold_required']
    assert router.update(far_ready=True, near_ready=True)['active'] == 'near'
    result = router.update(far_ready=True, near_ready=False)
    assert result['active'] == 'near' and result['hold_required']


def test_withdrawn_camera_candidate_rejects_without_mutating_original_scene():
    near = dict(calibration=dict(image_size=[1920, 1920]), hardware={},
                camera_ids=dict(left='l', right='r'), marker_rig={'markers': [1, 2]},
                cameras=[dict(camera_id=c, position_body_m=[0, 0, 0],
                              field_of_view_rad=np.deg2rad(100)) for c in ('l', 'r')])
    scene = dict(cameras=copy.deepcopy(near['cameras']), settings={'sunlight': True},
                 celestial_bodies=['earth', 'sun'], objects=[{'material': 'gold'}])
    old_near, old_scene = copy.deepcopy(near), copy.deepcopy(scene)
    with pytest.raises(ValueError, match='withdrawn'):
        long_focus_candidate(near, scene, fov_deg=6)
    assert near == old_near and scene == old_scene
