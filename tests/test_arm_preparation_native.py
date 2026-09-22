"""Opt-in end-to-end native preparation test (isolated Basilisk process)."""
import copy
import json
import os
from pathlib import Path
from types import SimpleNamespace
import numpy as np
import pytest

pytest.importorskip('Basilisk')
from simulation import teleop_grasp_unreal as teleop
from space_arm_platform.models import SceneInstanceCreate
from space_arm_platform.scene_runtime import SceneRuntimeManager

ROOT = Path(__file__).resolve().parents[1]
ADAPTER = Path(os.environ.get('SPACE_SIM_RESET_ADAPTER', str(ROOT.parents[1] / 'space_sim_UE_adapter/space_sim_UE_Adapter')))
pytestmark = pytest.mark.skipif(os.environ.get('SPACE_SIM_RUN_PREPARATION_NATIVE') != '1' or not (ADAPTER/'Adapters').is_dir(), reason='opt-in native preparation integration')


def test_zero_start_fixed_sequence_real_joint_controllers(tmp_path, monkeypatch):
    monkeypatch.syspath_prepend(str(ADAPTER/'Adapters'))
    monkeypatch.syspath_prepend(str(ADAPTER/'Unreal/BskUnrealRenderer/examples'))
    from scenario_spacecraft_arm_grasp_unreal import load_native_grasp_module
    load_native_grasp_module(ROOT/'model/SARM/platform')
    import bsk_render_adapter
    from bsk_render_adapter.protocol import RecordingOnlyPublisher
    class ProbeBridge(bsk_render_adapter.BasiliskRenderBridge):
        def __init__(self, **kwargs):
            super().__init__(**kwargs,publisher=RecordingOnlyPublisher())
        def add_mj_scene(self,scene,**kwargs):
            kwargs['mesh_asset_catalog']=None
            return super().add_mj_scene(scene,**kwargs)
    observations=[]
    class Finished(Exception):pass
    class Client(teleop.SimulationControlClient):
        def start(self):pass
        def close(self):pass
        def latest_action(self):
            action=dict(deadman=True,server_sequence='1',end_effector_linear_velocity_body_m_s=[0.]*3,end_effector_angular_velocity_body_rad_s=[0.]*3,gripper_velocity_m_s=0.)
            if len(observations)>6 and not any(o['arm_preparation']['ready'] for o in observations):
                action['arm_preparation']=dict(request_id='native-preparation',joint_position_deg=list(teleop.DEFAULT_OPERATING_JOINT_DEG))
            return action,False
        def send_observation(self,message):
            observations.append(copy.deepcopy(message))
            status=message['arm_preparation']['status']
            if len(observations)==1:
                np.testing.assert_allclose(message['arm_joint_position_rad'],0.,atol=1e-12)
            if len(observations)<7:
                assert status=='waiting'
                np.testing.assert_allclose(message['target_arm_joint_position_rad'],0.,atol=1e-12)
            if len(observations)%30==0:
                print(json.dumps({'sim_time':int(message['sim_time_ns'])/1e9,'preparation':message['arm_preparation']},ensure_ascii=True),flush=True)
            if status in {'failed','cancelled'}:
                raise AssertionError(message['arm_preparation'])
            if status=='moving':args.simulation_rate=20.
            ready=[o for o in observations if o['arm_preparation']['ready']]
            if len(ready)>=15:raise Finished()
            return True
    monkeypatch.setattr(bsk_render_adapter,'BasiliskRenderBridge',ProbeBridge)
    monkeypatch.setattr(teleop,'SimulationControlClient',Client)
    manager=SceneRuntimeManager(None,project_root=tmp_path)
    instance=manager.create_instance(SceneInstanceCreate(seed=123))
    args=SimpleNamespace(adapter_root=ADAPTER,model_root=ROOT/'model/SARM/platform',scene_instance=Path(instance['config_path']),
        catalog=tmp_path/'unused.json',control_host='127.0.0.1',control_port=0,render_host='127.0.0.1',render_port=0,
        duration=180.,simulation_rate=1.,capture_rate=30.,ik_rate=120.,ik_mode=teleop.IK_MODE_IK_POSE,disable_attitude_control=False)
    try:teleop.run(args)
    except Finished:pass
    report={'final':observations[-1],'count':len(observations),'statuses':sorted({o['arm_preparation']['status'] for o in observations})}
    (ROOT/'run/arm-preparation-native-result.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    ready=[o for o in observations if o['arm_preparation']['ready']]
    assert len(ready)>=15
    assert {'waiting','arming','moving','settling','ready'} <= set(report['statuses'])
    np.testing.assert_allclose(ready[0]['arm_joint_position_rad'],np.deg2rad(teleop.DEFAULT_OPERATING_JOINT_DEG),atol=np.deg2rad(.8))
    assert np.max(np.abs(ready[0]['arm_joint_velocity_rad_s'])) <= .02
    assert ready[0]['arm_preparation']['phase_count'] == 2
    assert len(ready[0]['arm_preparation']['phase_times']) == 2
    active=[o for o in observations if o['arm_preparation']['status'] in {'arming','moving','settling'}]
    np.testing.assert_allclose(np.asarray([o['target_arm_joint_position_rad'] for o in active])[:,[0,5]],0.,atol=1e-12)
    assert all(np.all(np.isfinite(o['arm_joint_position_rad'])) for o in observations)

    assert np.max(np.abs([o["arm_joint_velocity_rad_s"] for o in observations])) < 1.2
