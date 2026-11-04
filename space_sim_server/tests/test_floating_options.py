"""Public collection options reach sampled releases and the tracking controller."""
import argparse
from copy import deepcopy
import json
from types import SimpleNamespace

import numpy as np
import pytest
from test_floating_capture import MODEL, source_scene
from simulation.scripted_tasks.drawer_initialization import randomized_scene
from simulation.scripted_tasks.floating_capture import create_floating_task
from simulation.scripted_tasks.floating_options import add_floating_options, apply_floating_options
from simulation.scripted_tasks.floating_profiles import apply_floating_profile
from simulation.scripted_tasks.randomization import write_model_variant


def parse(*flags):
    parser = argparse.ArgumentParser()
    add_floating_options(parser)
    args = parser.parse_args(flags)
    args.task = 'floating_plug_capture'
    return args


def test_fixed_release_and_controller_replay(tmp_path):
    source = apply_floating_profile(source_scene(), 'right-incoming-blue')
    before = deepcopy(source)
    args = parse('--object-speed', '.03', '--object-spin', '.3', '--spin-axis', 'grasp_z', '--spin-sign', '-1',
                 '--tracking-linear-speed', '.08', '--tracking-angular-speed', '.6', '--tracking-joint-speed', '.7')
    configured = apply_floating_options(source, args)
    scene = randomized_scene(configured, 75, 0, args.task)
    model = write_model_variant(MODEL, tmp_path/'model.xml', scene)
    state = scene['collection_config']['floating_initial_state']
    assert np.linalg.norm(state['velocity_body_m_s']) == pytest.approx(.03)
    assert np.linalg.norm(state['angular_velocity_body_rad_s']) == pytest.approx(.3)
    task = create_floating_task(args.task, model, scene)
    assert task.config.angular_speed_rad_s == .6
    assert task.config.arm_speed_rad_s == .7
    assert task.config.linear_speed_m_s == .08
    assert task.config.hold_s == 2 and task.config.force_limit_n == 80
    serialized = json.loads(json.dumps(scene))
    assert write_model_variant(MODEL, tmp_path/'replay.xml', serialized).read_bytes() == model.read_bytes()
    assert source == before
    serialized['collection_config']['floating_initial_state']['angular_velocity_body_rad_s'] = (-np.asarray(state['angular_velocity_body_rad_s'])).tolist()
    with pytest.raises(ValueError, match='spin axis'):
        write_model_variant(MODEL, tmp_path/'wrong.xml', serialized)
    scene['floating_controller']['angular_speed_rad_s'] = .9
    with pytest.raises(ValueError, match='controller'):
        write_model_variant(MODEL, tmp_path/'wrong-controller.xml', scene)


def test_ranges_vary_with_seed(tmp_path):
    args = parse('--object-speed-range', '.01', '.03', '--object-spin-range', '.1', '.3')
    source = apply_floating_options(source_scene(), args)
    observed = []
    for seed in (75, 76, 77):
        scene = randomized_scene(source, seed, 0, args.task)
        write_model_variant(MODEL, tmp_path/f'{seed}.xml', scene)
        state = scene['collection_config']['floating_initial_state']
        speed, spin = (np.linalg.norm(state[k]) for k in ('velocity_body_m_s', 'angular_velocity_body_rad_s'))
        assert .01 <= speed <= .03 and .1 <= spin <= .3
        observed.append(speed)
    assert len(set(observed)) == 3


@pytest.mark.parametrize('flags', [('--object-speed', '-.1'), ('--object-spin', 'inf'),
    ('--object-speed-range', '.03', '.01'), ('--tracking-angular-speed', 'nan'),
    ('--tracking-joint-speed', '0'), ('--object-speed', '.03', '--object-direction', 'stationary')])
def test_invalid_parameters(flags):
    with pytest.raises(ValueError):
        apply_floating_options(source_scene(), parse(*flags))


def test_override_scope_and_mutual_exclusion():
    args = parse('--object-speed', '.03')
    with pytest.raises(ValueError, match='prepared'):
        apply_floating_options(source_scene(), args, prepared=True)
    args.task = 'drawer_cycle'
    with pytest.raises(ValueError, match='floating capture'):
        apply_floating_options(source_scene(), args)
    with pytest.raises(SystemExit):
        parse('--object-spin', '.3', '--object-spin-range', '0', '.3')


def test_tracking_uses_angular_limit(tmp_path):
    args = parse('--tracking-angular-speed', '.6')
    scene = randomized_scene(apply_floating_options(source_scene(), args), 75, 0, args.task)
    model = write_model_variant(MODEL, tmp_path/'model.xml', scene)
    task = create_floating_task(args.task, model, scene)
    twist = []
    task.chain = SimpleNamespace(forward=lambda q: (np.zeros(3), np.eye(3)),
        inverse_velocity_ik_pose=lambda q, v, **kw: (twist.append(v) or SimpleNamespace(joint_velocity_rad_s=np.zeros(6))))
    task.grasp_pose = lambda sample: (np.zeros(3), np.eye(3))
    sample = SimpleNamespace(joints=np.zeros(8), plug_velocity_body_m_s=np.zeros(3),
        plug_rotation_body=np.eye(3), plug_angular_velocity_body_rad_s=np.array([0., 0., .8]))
    task._track(sample, .005)
    np.testing.assert_allclose(twist[0][3:], [0., 0., .6])


@pytest.mark.parametrize('field', ['hold_s', 'force_limit_n', 'torque_limit_nm'])
def test_controller_rejects_success_gate_overrides(field):
    source = source_scene()
    source['floating_controller'] = {field: 1.}
    with pytest.raises(ValueError, match='only tracking'):
        apply_floating_options(source, parse())
