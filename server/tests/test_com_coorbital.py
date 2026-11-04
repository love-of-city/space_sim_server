"""Co-orbital initialization applies to each complete spacecraft COM."""
import numpy as np
import pytest

from simulation.formation_hold import reference


def test_nonzero_com_offsets_preserve_one_circle_and_orbital_plane():
    radius = 6878136.6
    speed = 7612.60833919537
    r, v = np.array([radius, 0., 0.]), np.array([0., speed, 0.])
    cs = np.array([.2744, .0948, .0746])
    ct = np.array([.1615, -.0107, .0094])
    result = reference(r, v, 1.05, service_com_body_m=cs, target_com_body_m=ct,
                       service_body_rotation_wxyz=[.573576436351046, .8191520442889918, 0, 0])
    states = []
    for prefix, offset in [('service', cs), ('target', ct)]:
        delta = result[prefix + '_rotation'] @ offset
        position = result[prefix + '_position'] + delta
        velocity = result[prefix + '_velocity'] + np.cross(result['omega'], delta)
        states.append((position, velocity))
        assert np.linalg.norm(position) == pytest.approx(radius, abs=2e-9)
        assert np.linalg.norm(velocity) == pytest.approx(speed, abs=2e-12)
        assert abs(position[2]) < 1e-12
        assert abs(velocity[2]) < 1e-12
        assert abs(position @ velocity) < 1e-5
    np.testing.assert_allclose(states[0][0], r, atol=1e-9)
    np.testing.assert_allclose(states[0][1], v, atol=1e-12)
    rho = states[1][0] - states[0][0]
    np.testing.assert_allclose(states[1][1] - states[0][1],
                               np.cross(result['omega'], rho), atol=1e-12)


@pytest.mark.parametrize('offset', [[1, 2], [0, float('nan'), 0]])
def test_invalid_com_offsets_are_rejected(offset):
    with pytest.raises(ValueError, match='COM'):
        reference([6878136.6, 0, 0], [0, 7612, 0], 1., service_com_body_m=offset)


def test_authoritative_compiled_mass_centres_include_arm_and_inferred_mass(model_path):
    from test_local_mujoco_stepper import build
    sim, scene, stepper, keep = build(model_path, 'local')
    try:
        scene.getBody('link').getScalarJoint('arm').setPosition(np.pi / 2)
        before_q = np.asarray(scene.assembleFullQpos()).copy()
        before_v = np.asarray(scene.assembleFullQvel()).copy()
        props = stepper.initial_assembly_properties(model_path, ['bus', 'ball'])
        assert props['bus']['mass_kg'] == pytest.approx(12.)
        np.testing.assert_allclose(props['bus']['com_body_m'], [.3/12, .2/12, .3/12], atol=1e-12)
        assert props['ball']['mass_kg'] == pytest.approx(.2)
        np.testing.assert_allclose(props['ball']['com_body_m'], [0, 0, 0], atol=1e-12)
        np.testing.assert_array_equal(scene.assembleFullQpos(), before_q)
        np.testing.assert_array_equal(scene.assembleFullQvel(), before_v)
        assert stepper.step_count == 0
    finally:
        stepper.close()


@pytest.fixture
def model_path(tmp_path):
    from test_local_mujoco_stepper import MJCF
    path = tmp_path / 'assembly_com.xml'
    path.write_text(MJCF, encoding='utf-8')
    return path


def test_orbital_setters_use_whole_assembly_com_after_initial_arm_pose(tmp_path):
    from types import SimpleNamespace
    from Basilisk.utilities import RigidBodyKinematics as rbk, simIncludeGravBody
    from test_orbital_initial_state import StateSink, expected_circular_state
    from space_arm_platform.scene_runtime import _DEFAULT_ORBIT
    from simulation.capture_arm_target import INITIAL_JOINTS
    from simulation.teleop_grasp_unreal import _apply_orbital_initial_state

    path = tmp_path / 'sarm_dual_capture26.xml'
    path.write_text('<mujoco><worldbody/></mujoco>')
    names = {'cubesat_bus', 'capture_target', *(b for b, _, _ in INITIAL_JOINTS)}
    bodies = {name: StateSink() for name in names}
    scene = SimpleNamespace(getBody=bodies.__getitem__, getBodyNames=lambda: list(names))
    native = SimpleNamespace(MODEL_PATH=path, COMMON_VELOCITY=[0, 0, 0], JOINTS=[], _quaternion_to_mrp=rbk.EP2MRP)
    props = {'cubesat_bus': {'mass_kg': 39., 'com_body_m': [.2744, .0948, .0746]},
             'capture_target': {'mass_kg': 10., 'com_body_m': [.1615, -.0107, .0094]}}
    q = [.1, -.2, .3, -.4, .5, -.6]
    def measure(model_path, roots):
        assert model_path == path
        assert roots == ['cubesat_bus', 'capture_target']
        for i, value in enumerate(q, 1):
            assert bodies[f'bottom_link{i}'].joints[f'bottom_joint{i}'].position == value
        return props
    stepper = SimpleNamespace(initial_assembly_properties=measure)
    randomized = dict(target_position_m=[1, 0, 0], target_orientation_wxyz=[1, 0, 0, 0],
                      target_linear_velocity_m_s=[0, 0, 0], target_angular_velocity_rad_s=[0, 0, 0],
                      capture_arm_joint_position_rad=q,
                      coorbital_initialization={'enabled': True, 'separation_m': 1.05,
                                                'reference_point': 'assembly_com'})
    earth = simIncludeGravBody.gravBodyFactory().createEarth()
    result = _apply_orbital_initial_state(scene, native, earth, _DEFAULT_ORBIT, randomized,
                                        np.array([]), local_stepper=stepper)
    r, v = expected_circular_state(earth, _DEFAULT_ORBIT['true_anomaly_deg'])
    expected = reference(r, v, 1.05, service_com_body_m=props['cubesat_bus']['com_body_m'],
                         target_com_body_m=props['capture_target']['com_body_m'])
    for name, prefix in [('cubesat_bus', 'service'), ('capture_target', 'target')]:
        np.testing.assert_allclose(bodies[name].position, expected[prefix+'_position'], atol=1e-8)
        np.testing.assert_allclose(bodies[name].velocity, expected[prefix+'_velocity'], atol=1e-10)
    assert result['assembly_com_properties'] == props
