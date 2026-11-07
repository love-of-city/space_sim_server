"""Moving-frame state and closed-loop capture alignment regression."""
from pathlib import Path
from types import SimpleNamespace

import numpy as np

from simulation.serial_chain_kinematics import axis_angle_to_matrix, matrix_to_quaternion_wxyz
from simulation.scripted_tasks.capture_grasp import CaptureGraspTask, INTERFACE_CENTER
from simulation.scripted_tasks.native_state import CaptureState


def reader(base_p, base_r, target_p, target_r, base_v, target_v, base_w, target_w):
    poses = {'cubesat_bus': np.r_[base_p, matrix_to_quaternion_wxyz(base_r)],
             'capture_target': np.r_[target_p, matrix_to_quaternion_wxyz(target_r)]}
    rates = np.r_[base_v, base_r.T @ base_w, target_v, target_r.T @ target_w]
    scene = SimpleNamespace(getBody=lambda name: SimpleNamespace(getFreeJoint=lambda:
        SimpleNamespace(getQvelAdr=lambda: 0 if name == 'cubesat_bus' else 6)))
    stepper = SimpleNamespace(qvel=rates, body_detail=poses.__getitem__)
    return CaptureState(scene, stepper)


def test_interface_twist_matches_numerical_world_pose_derivative():
    rb = axis_angle_to_matrix([0., 0., 1.], .6)
    rt = axis_angle_to_matrix([0., 1., 0.], -.4)
    bp, tp = np.array([.1, -.2, .3]), np.array([1., .6, -.4])
    bv, tv = np.array([7500., -2., .5]), np.array([7500.003, -1.999, .502])
    bw, tw = np.array([.01, -.02, .03]), np.array([-.02, .04, .01])
    p, r, v, w = reader(bp, rb, tp, rt, bv, tv, bw, tw).interface()
    h = 1e-4
    def pose(dt):
        b = axis_angle_to_matrix(bw / np.linalg.norm(bw), np.linalg.norm(bw)*dt) @ rb
        t = axis_angle_to_matrix(tw / np.linalg.norm(tw), np.linalg.norm(tw)*dt) @ rt
        return b.T @ (tp + tv*dt + t @ INTERFACE_CENTER - bp - bv*dt)
    np.testing.assert_allclose(v, (pose(h)-pose(-h))/(2*h), atol=1e-8)
    np.testing.assert_allclose(w, rb.T @ (tw-bw), atol=1e-12)
    np.testing.assert_allclose(p, pose(0.), atol=1e-12)


def test_rigid_common_translation_and_rotation_have_zero_relative_twist():
    rb = axis_angle_to_matrix([0., 0., 1.], .3)
    rt = rb @ axis_angle_to_matrix([1., 0., 0.], -.2)
    bp, tp = np.array([.1, -.2, .3]), np.array([.7, -.1, .8])
    bw = np.array([.1, -.2, .3]); bv = np.array([7500., -300., .2])
    _, _, v, w = reader(bp, rb, tp, rt, bv, bv+np.cross(bw, tp-bp), bw, bw).interface()
    np.testing.assert_allclose(v, 0., atol=1e-11)
    np.testing.assert_allclose(w, 0., atol=1e-12)


def test_moving_target_alignment_converges_under_original_error_gate():
    from simulation.scripted_tasks.capture_grasp import CaptureGraspConfig
    model = Path(__file__).resolve().parents[1]/'model/SARM/platform/sarm_dual_capture26.xml'
    task = CaptureGraspTask(model,CaptureGraspConfig(position_tolerance_m=.0004,angle_tolerance_rad=.0087))
    task.stage = 'align'
    q = task.reference.copy(); p, r = task.chain.forward(q[:6])
    v = np.array([.0011, 0., 0.])
    initial = p-r @ [0., 0., task.config.clearance_m]+[.0014, 0., 0.]
    for index in range(601):
        t = index*.005
        q, _ = task.step(dict(t=t, q=q, interface_position=initial+t*v,
            interface_rotation=r, interface_linear_velocity=v,
            interface_angular_velocity=np.zeros(3), feedback={}, unplanned_contacts=[]))
        if task.stage == 'approach':
            break
    assert task.stage == 'approach'
    assert t < 3.
    assert task.tracking['position_error_m'] < .0004
    assert task.config.position_tolerance_m == .0004
