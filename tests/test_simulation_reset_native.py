"""Native reset regression: exercise the real run loop, not a second physics engine.

Set SPACE_SIM_RESET_ADAPTER to the matching adapter worktree. Run with the
Basilisk environment; native MuJoCo and Python MuJoCo must not share a process.
"""
import copy
import gc
import weakref
import os
from pathlib import Path
import sys
from types import SimpleNamespace

import numpy as np
import pytest

pytest.importorskip('Basilisk')
from simulation import teleop_grasp_unreal as teleop
from space_arm_platform.models import SceneInstanceCreate
from space_arm_platform.scene_runtime import SceneRuntimeManager

ROOT = Path(__file__).resolve().parents[1]
ADAPTER = Path(os.environ.get('SPACE_SIM_RESET_ADAPTER', str(ROOT.parent / 'space_sim_UE_adapter_lerobot_v3')))


def control_action(generation=''):
    return dict(protocol=teleop.CONTROL_PROTOCOL, type='action', reset_generation=generation,
                server_sequence='9', deadman=True, end_effector_linear_velocity_body_m_s=[.03, 0, 0],
                end_effector_angular_velocity_body_rad_s=[0, .02, 0], gripper_velocity_m_s=-.002,
                gripper_velocity_rad_s=0)


def test_client_reset_barrier_and_duplicate_delivery():
    client = teleop.SimulationControlClient('127.0.0.1', 0, 'test')
    client._accept_message(control_action())
    assert client.latest_action()[0]['deadman']
    reset = dict(protocol=teleop.CONTROL_PROTOCOL, type='reset', request_id='reset-1')
    client._accept_message(reset)
    assert not client.latest_action()[0]['deadman']
    assert client.reset_generation == ''  # old observations must remain old until main-thread consumption
    assert client.take_reset() == 'reset-1'
    assert client.reset_generation == 'reset-1'
    client._accept_message(control_action('reset-1'))
    assert not client.latest_action()[0]['deadman']  # even new commands blocked during rebuild
    client.complete_reset()
    client._accept_message(control_action())
    assert not client.latest_action()[0]['deadman']  # buffered previous generation rejected
    client._accept_message(control_action('reset-1'))
    assert client.latest_action()[0]['deadman']
    client._accept_message(reset)
    assert client.take_reset() is None  # reconnect resends are idempotent


@pytest.mark.skipif(not (ADAPTER / 'Adapters').is_dir(), reason='matching adapter worktree unavailable')
@pytest.mark.parametrize("initial_angles", [None, [-45., -20., 25., -90., -50., 275.], [0.] * 6])
def test_real_run_rebuilds_identical_initial_state_twice(tmp_path, monkeypatch, initial_angles):
    monkeypatch.syspath_prepend(str(ADAPTER / 'Adapters'))
    monkeypatch.syspath_prepend(str(ADAPTER / 'Unreal/BskUnrealRenderer/examples'))
    # Establish the production DLL import order before loading any native scene.
    from scenario_spacecraft_arm_grasp_unreal import load_native_grasp_module
    native = load_native_grasp_module(ROOT / 'model/SARM/platform')
    import bsk_render_adapter
    from bsk_render_adapter.protocol import RecordingOnlyPublisher

    bridges, observations, events, scene_refs = [], [], [], []
    authoritative_joint_samples = {}
    class ProbeBridge(bsk_render_adapter.BasiliskRenderBridge):
        def __init__(self, **kwargs):
            super().__init__(**kwargs, publisher=RecordingOnlyPublisher())
            self.initial = None
            self.last = None
            self.physics_stamps = []
            bridges.append(self)
        def add_mj_scene(self, scene, **kwargs):
            self.scene = scene
            self.state_reader = scene.stateOutMsg.addSubscriber()
            scene_refs.append(weakref.ref(scene))
            kwargs['mesh_asset_catalog'] = None  # no UE asset import needed for state transport
            return super().add_mj_scene(scene, **kwargs)
        def UpdateState(self, nanos):
            self.physics_stamps.append(int(nanos))
            assert int(self.state_reader.timeWritten()) == int(nanos)
            super().UpdateState(nanos)
            if self.last_published_sim_time_ns == int(nanos):
                authoritative_joint_samples[(self.session_id, str(self.last_published_frame_id))] = (
                    str(nanos), [float(self.scene.getBody(body).getScalarJoint(joint).stateOutMsg.read().state)
                                 for body, joint in native.JOINTS])
            data = self.scene.stateOutMsg.read()
            snapshot = (np.array(data.qpos), np.array(data.qvel))
            if self.initial is None:
                self.initial = (int(nanos), snapshot)
                if initial_angles is not None:
                    actual = [float(self.scene.getBody(body).getScalarJoint(joint).stateOutMsg.read().state)
                              for body, joint in native.JOINTS[:6]]
                    np.testing.assert_allclose(actual, np.deg2rad(initial_angles), rtol=0, atol=1.e-12)
            self.last = snapshot
        def close(self):
            super().close()
            # Keep only snapshots, not the native graph, so the real run-loop
            # garbage collection/destructor path is exercised between resets.
            bridges[bridges.index(self)] = SimpleNamespace(
                session_id=self.session_id, initial=self.initial, last=self.last,
                physics_stamps=self.physics_stamps)
        def publish_event(self, kind, payload=None):
            events.append((self.session_id, kind, payload))
            return super().publish_event(kind, payload)

    class OfflineClient(teleop.SimulationControlClient):
        def start(self):
            pass
        def close(self):
            pass
        def send_observation(self, message):
            observations.append(copy.deepcopy(message))
            step = int(message['step_id'])
            if step == 1:
                self._accept_message(control_action(self.reset_generation))
            if step == 2:
                # Deliberately disturb independent state, including passive and
                # wheel DOFs, so merely resetting IK references cannot pass.
                scene = bridges[-1].scene
                bus = scene.getBody('cubesat_bus')
                bus.setAttitude([.01, .02, 0])
                bus.setAttitudeRate([.01, 0, 0])
                target = scene.getBody('capture_target')
                target.setVelocity([1., 2., 3.])
                target.setAttitude([0, .02, 0])
                hinge = scene.getBody('satellite_outer_panel').getScalarJoint('outer_panel_hinge')
                hinge.setPosition(.1)
                hinge.setVelocity(.02)
                scene.getBody('rw_x').getScalarJoint('rw_x_spin').setVelocity(12.)
            if step == 4 and len(bridges) < 3:
                self._accept_message(dict(protocol=teleop.CONTROL_PROTOCOL, type='reset',
                                          request_id=f'reset-{len(bridges)}'))
            return True

    monkeypatch.setattr(bsk_render_adapter, 'BasiliskRenderBridge', ProbeBridge)
    monkeypatch.setattr(teleop, 'SimulationControlClient', OfflineClient)
    manager = SceneRuntimeManager(None, project_root=tmp_path)
    instance = manager.create_instance(SceneInstanceCreate(seed=123, randomize_orbit_phase=True, randomization_profile="teleop-balanced-v1" if initial_angles is not None else "teleop-zero-prepare-v1", initial_arm_joint_position_deg=initial_angles))
    config_path = Path(instance['config_path'])
    before = config_path.read_bytes()
    args = SimpleNamespace(adapter_root=ADAPTER, model_root=ROOT / 'model/SARM/platform',
                           scene_instance=config_path, catalog=tmp_path / 'unused.json',
                           control_host='127.0.0.1', control_port=0, render_host='127.0.0.1', render_port=0,
                           duration=.2, simulation_rate=1., capture_rate=30., ik_rate=120.,
                           ik_mode=teleop.IK_MODE_IK_POSE,
                           disable_attitude_control=False)
    teleop.run(args)
    gc.collect()
    assert all(ref() is None for ref in scene_refs)
    assert config_path.read_bytes() == before
    assert len(bridges) == 3
    assert len({b.session_id for b in bridges}) == 3
    baseline = bridges[0].initial
    assert baseline[0] == 0
    assert np.linalg.norm(baseline[1][1]) > 1000  # orbital speed restored, NOT zeroed
    for bridge in bridges:
        assert bridge.initial[0] == 0
        assert np.isfinite(bridge.last[0]).all() and np.isfinite(bridge.last[1]).all()
        np.testing.assert_allclose(bridge.initial[1][0], baseline[1][0], rtol=0, atol=1.e-12)
        np.testing.assert_allclose(bridge.initial[1][1], baseline[1][1], rtol=0, atol=1.e-12)
        assert not np.array_equal(bridge.initial[1][0], bridge.last[0])
    # Recorded joints must be the render-time snapshot, not a later state read
    # after ExecuteSimulation reaches the outer 30 Hz loop's stop time.
    for observation in observations:
        stamp, joints = authoritative_joint_samples[(observation['render_session_id'], observation['render_frame_id'])]
        assert observation['observation_source'] == 'authoritative_render_snapshot'
        assert observation['sim_time_ns'] == stamp
        np.testing.assert_array_equal(observation['joint_position_rad'], joints)
    from space_arm_platform.sampling import tick_time_ns, sample_tick
    for bridge in bridges:
        assert bridge.physics_stamps == [tick_time_ns(n, 240) for n in range(len(bridge.physics_stamps))]
        rows = [o for o in observations if o['render_session_id'] == bridge.session_id]
        assert [int(o['sim_time_ns']) for o in rows] == [tick_time_ns(n, 30) for n in range(len(rows))]
        for index, obs in enumerate(rows):
            assert sample_tick(int(obs['sim_time_ns']), 30) == index
            assert obs['dynamics_rate_hz'] == 240
            assert obs['ik_control_rate_hz'] == 120
            assert int(obs['ik_update_count']) == index * 4
    first = [o for o in observations if o['step_id'] == '1']
    assert [o['reset_generation'] for o in first] == ['', 'reset-1', 'reset-2']
    assert len({o['sim_time_ns'] for o in first}) == 1
    for obs in first:
        assert obs['applied_action_sequence'] == '0'
        assert obs['scene_seed'] == 123
        assert obs['scene_instance_id'] == instance['instance_id']
        assert obs['command_stale']
        np.testing.assert_allclose(obs['reaction_wheels']['speed_rad_s'], first[0]['reaction_wheels']['speed_rad_s'], atol=1.e-12)
        np.testing.assert_allclose(obs['target_joint_position_rad'], first[0]['target_joint_position_rad'], atol=1.e-12)
    assert len([event for event in events if event[1] == 'scene_reset']) == 2
