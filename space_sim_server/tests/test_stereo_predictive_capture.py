"""The merged controller must use exposure-time FK and both real image eyes."""
import copy
import json
from pathlib import Path

import numpy as np
import pytest

from simulation.stereo_predictive_capture import StereoPredictiveCapture
from space_arm_platform.stereo_capture_packet import capture_packet, publish_packet

ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT / 'config/vision/capture26_near23_front_mount_20261029.json'
NOW = 10_000_000_000


def measurement(point=(0., 0., -.1)):
    config = json.loads(CONFIG.read_text('utf-8'))
    rgc = np.array(config['parent_from_optical']['left']['R_parent_from_optical'])
    offset = np.array(config['parent_from_optical']['left']['position_parent_m'])
    rti = np.array([[1., 0, 0], [0, 0, 1], [0, -1, 0]])
    rct = rgc.T @ rti.T
    t = rgc.T @ (np.array(point)-offset)-rct @ config['marker_rig']['grasp_center_target_m']
    return dict(status='ok', route='joint_multicamera', used_ids={'left': [10], 'right': [21]},
                reprojection_error_px={'left': .2, 'right': .3},
                maximum_corner_error_px={'left': .4, 'right': .5},
                minimum_marker_side_px={'left': 18, 'right': 20}, positive_depth=True,
                pose_target_to_left={'R': rct.tolist(), 't_m': t.tolist()})


def packet(t, **kwargs):
    config = json.loads(CONFIG.read_text('utf-8'))
    return capture_packet(measurement(**kwargs), config, session_id='s',
                          source_frame_id=str(t), sim_time_ns=round(t*1e9),
                          published_wall_time_ns=NOW)


def controller(tmp_path):
    return StereoPredictiveCapture(tmp_path/'p.json', 's', config_file=CONFIG,
                                   wall_now_ns=lambda: NOW)


def test_hand_eye_converts_model_origin_to_interface_not_tvec(tmp_path):
    p = packet(1)
    assert np.allclose(p['result']['alignment']['interface_position_grip_m'], [0, 0, -.1])
    assert np.allclose(p['result']['alignment']['R_grip_from_interface'], np.eye(3))
    c = controller(tmp_path)
    publish_packet(c.path, p)
    assert c._observe(1, np.zeros(3), np.eye(3))
    assert c._complete_observation()  # Different IDs; one small marker in each eye.


def test_delayed_pose_uses_exposure_encoder_not_reception_encoder(tmp_path):
    c = controller(tmp_path)
    c._observe(1, np.array([.01, 0, 0]), np.eye(3))
    publish_packet(c.path, packet(1))
    assert c._observe(1.1, np.array([.02, 0, 0]), np.eye(3))
    assert np.allclose(c.p_base, [.01, 0, -.1])


@pytest.mark.parametrize('mutation,reason', [
    (lambda p: p.update(configuration_id='old_mount'), 'configuration'),
    (lambda p: p.update(session_id='old_scene'), 'session'),
    (lambda p: p.update(sim_time_ns='0'), 'stale'),
    (lambda p: p.update(sim_time_ns='2000000000'), 'future'),
])
def test_rejects_wrong_mount_session_or_exposure(tmp_path, mutation, reason):
    c = controller(tmp_path); p = packet(1); mutation(p); publish_packet(c.path, p)
    with pytest.raises(ValueError, match=reason):
        c._pose(1)


def test_single_eye_never_becomes_valid_packet():
    config = json.loads(CONFIG.read_text('utf-8')); m = measurement(); m['used_ids']['right'] = []
    with pytest.raises(ValueError, match='each eye'):
        capture_packet(m, config, session_id='s', source_frame_id='1', sim_time_ns=1)


def test_loss_prediction_recovery_and_jump_rejection(tmp_path):
    c = controller(tmp_path); c.sequence.phase = 'approach'
    q = np.zeros(12); q[6] = -.006
    own = (np.zeros(3), np.eye(3))
    for t in np.linspace(1, 2, 11):
        publish_packet(c.path, packet(t))
        c.command(t, q, own, {}, reference_from_base=own)
    assert len(c.observations) == 11
    lost = copy.deepcopy(packet(2)); lost.update(valid=False, status='lost')
    publish_packet(c.path, lost)
    _, status = c.command(2.1, q, (np.array([0, 0, -.001]), np.eye(3)), {}, reference_from_base=own)
    assert status == 'predictive_approach_encoder'
    publish_packet(c.path, packet(2.2, point=(0, 0, -.099)))
    _, status = c.command(2.2, q, (np.array([0, 0, -.001]), np.eye(3)), {}, reference_from_base=own)
    assert status == 'predictive_approach_visual'
    publish_packet(c.path, packet(2.3, point=(.05, 0, -.099)))
    c.command(2.3, q, own, {}, reference_from_base=own)
    assert c.input_status == 'visual_jump_rejected'
    assert c.source_t == 2.2


def test_service_publishes_rejection_instead_of_reusing_last_pose(tmp_path):
    from space_arm_platform.vision_service import StereoVisionService
    config = json.loads(CONFIG.read_text('utf-8'))
    responses = iter([measurement(), {'status': 'rejected', 'reason': 'one eye occluded'}])
    service = StereoVisionService(tmp_path, config=config, estimator=lambda l, r: next(responses))
    service.start()
    try:
        for frame in (1, 2):
            for eye in ('left', 'right'):
                service.on_capture(dict(camera_id=config['camera_ids'][eye], session_id='s',
                    source_frame_id=str(frame), sim_time_ns=frame*100000000,
                    extrinsics={'unused': 'simulator truth'}), {'rgb': b'image'})
            assert service.wait_idle()
            exported = json.loads((tmp_path/config['capture_export']).read_text('utf-8'))
            assert exported['source_frame_id'] == str(frame)
            assert exported['valid'] is (frame == 1)
            assert 'extrinsics' not in exported
        assert exported['result']['alignment'] == {'valid': False}
    finally:
        service.close()
