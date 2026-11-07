"""Keep the capture collision gate aligned with the native contact-row ABI."""
from types import SimpleNamespace

import numpy as np
import pytest


def test_initial_arm_bus_penetration_is_rejected_before_camera_or_ik(tmp_path, monkeypatch):
    from simulation.scripted_tasks import capture_preflight

    model = tmp_path / 'capture.xml'
    model.write_text('''<mujoco><worldbody><body name="cubesat_bus">
      <freejoint name="cubesat_free"/><geom name="bus_geom"/>
      <body name="bottom_link1"><body name="bottom_link2">
        <body name="bottom_link3"><geom name="arm_geom"/></body>
      </body></body></body></worldbody></mujoco>''')

    class Probe:
        def __init__(self, _model):
            self.q = np.array([0., 0., 0., 1., 0., 0., 0.])
            self.v = np.zeros(6)
            self.handle = object()
            self.lib = SimpleNamespace(lms_probe_set_state=lambda *_: 0,
                                       lms_contacts=self.contacts)

        @staticmethod
        def contacts(_handle, output, capacity):
            if capacity:
                # geom1, geom2, body1, body2, signed separation in metres.
                output[:] = [[0., 1., 1., 4., -.01]]
            return 1

        @staticmethod
        def ptr(array):
            return array

        @staticmethod
        def check(status):
            assert status >= 0

        @staticmethod
        def address(name):
            assert name == 'cubesat_free'
            return 0

        @staticmethod
        def geom_name(index):
            return ['bus_geom', 'arm_geom'][int(index)]

        def close(self):
            pass

    monkeypatch.setattr(capture_preflight, 'NativePoseProbe', Probe)
    state = SimpleNamespace(qpos=np.array([0., 0., 0., 1., 0., 0., 0.]))
    result = capture_preflight.validate_initial_and_path(model, state, None, None, {})
    assert not result['accepted']
    assert result['reason'] == 'initial_collision'
    assert result['contacts'] == [dict(geoms=['bus_geom', 'arm_geom'], distance_m=-.01)]


@pytest.mark.parametrize('blocked,frame_hits,accepted', [(False, 5, True),
                                                       (True, 5, False),
                                                       (False, 2, False)])
def test_hollow_capture_interface_keeps_visibility_and_occlusion_checks(
        tmp_path, monkeypatch, blocked, frame_hits, accepted):
    from simulation.scripted_tasks import capture_preflight
    from simulation.scripted_tasks.capture_grasp import INTERFACE_CENTER, TARGET_FROM_INTERFACE

    model = tmp_path/'capture.xml'
    model.write_text('''<mujoco><worldbody><body name="cubesat_bus">
      <freejoint name="cubesat_free"/></body><body name="capture_target"/>
      </worldbody></mujoco>''')
    frame_indices = [0, 2, 6, 7, 8][:frame_hits]

    class Probe:
        def __init__(self, _):
            self.q, self.v = np.zeros(13), np.zeros(12)
            self.handle, self.ray = object(), 0
            self.lib = SimpleNamespace(lms_probe_set_state=lambda *_: 0,
                                       lms_contacts=lambda *_: 0,
                                       lms_visual_ray=self.visual_ray)

        def visual_ray(self, _, origin, direction, result):
            # At the hole, the first surface is behind the reference plane.
            result[:] = [np.linalg.norm(direction) + .02, 1]
            if self.ray in frame_indices:
                result[:] = [np.linalg.norm(direction), 0]
            if blocked and self.ray == 4:
                result[:] = [.5 * np.linalg.norm(direction), 1]
            self.ray += 1
            return 0

        ptr = staticmethod(lambda value: value)
        check = staticmethod(lambda status: None)
        close = lambda self: None
        address = staticmethod(lambda name: 0 if name == 'cubesat_free' else 7 + int(name[-1]))
        geom_name = staticmethod(lambda index: ('target_instance_0030_part_015_color_00',
                                                'backplate_or_occluder')[int(index)])

    monkeypatch.setattr(capture_preflight, 'NativePoseProbe', Probe)
    monkeypatch.setattr(capture_preflight, 'double_pointer', lambda value: value)
    rotation = TARGET_FROM_INTERFACE.T
    initial = SimpleNamespace(geometry=lambda *_: (-rotation @ INTERFACE_CENTER, rotation))
    chain = SimpleNamespace(joint_names=[f'j{i}' for i in range(6)],
                            forward=lambda q: (np.asarray(q[:3]), np.eye(3)),
                            jacobian=lambda q: np.eye(6))
    task = SimpleNamespace(chain=chain, reference=np.zeros(6), lower=np.full(6, -2.),
                           upper=np.full(6, 2.),
                           config=SimpleNamespace(clearance_m=.06, seated_gap_m=.006))
    projection = dict(camera_position_service_m=[0., 0., 1.],
                       region_points_service_m=[[x, y, 0.] for x in (-.04, 0., .04)
                                               for y in (-.04, 0., .04)])
    result = capture_preflight.validate_initial_and_path(
        model, SimpleNamespace(qpos=np.zeros(13)), task, initial, projection)
    assert result['accepted'] is accepted
    if not accepted:
        assert result['reason'] == 'capture_camera_occluded'
    else:
        assert result['rays'][4]['interface_hit'] is False
        assert not any(r['occluded'] for r in result['rays'])
        assert result['path_samples'] > 0
