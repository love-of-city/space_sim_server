"""Capture component compatibility; imports the real native module graph."""
import io
import json
from types import SimpleNamespace

import numpy as np
import pytest

from simulation.components import capture26


class FakeTarget:
    def __init__(self,model):
        self.position=np.zeros(12)
        self.kinematics=SimpleNamespace(forward=lambda q:(np.zeros(3),np.eye(3)))
        self.command=None

    def step(self,dt,command,stale,measured):
        self.command=command
        return self.position.copy(),np.zeros(12)


@pytest.mark.parametrize('mode',['servo',None,'other'])
def test_held_target_pose_rejects_nonpredictive_modes_before_startup(tmp_path,monkeypatch,mode):
    model=tmp_path/'model.xml';model.write_text('<mujoco/>')
    monkeypatch.setattr(capture26,'CaptureArmTarget',FakeTarget)
    config=dict(enabled=True,mode=mode,held_target_pose=True,output_directory=str(tmp_path/'out'))
    with pytest.raises(ValueError,match='held_target_pose requires predictive_grasp'):
        capture26.Capture26Component(model,SimpleNamespace(),visual_test=config)


def test_legacy_servo_preserves_existing_contact_overload_abort(tmp_path,monkeypatch):
    model=tmp_path/'model.xml';model.write_text('<mujoco/>')
    monkeypatch.setattr(capture26,'CaptureArmTarget',FakeTarget)
    client=SimpleNamespace(latest_action=lambda:({'deadman':False},True))
    config=dict(enabled=True,mode='servo',output_directory=str(tmp_path/'out'))
    c=capture26.Capture26Component(model,client,experiment=True,visual_test=config,render_session_id='s')
    c.sequence=c.visual.sequence;c.sequence.phase='approach'
    c.visual.wall_now_ns=lambda:10_000_000_000
    c.visual.path=tmp_path/'vision.json'
    c.visual.path.write_text(json.dumps(dict(valid=True,status='tracking',session_id='s',
        source_frame_id='1',sim_time_ns='3000000000',age_s=.1,published_wall_time_ns=10_000_000_000,
        result={'alignment':dict(valid=True,interface_position_grip_m=[0,0,-.6],R_grip_from_interface=np.eye(3).tolist())})))
    c.positions=[lambda:SimpleNamespace(state=0.) for _ in range(12)]
    bus=SimpleNamespace(r_BN_N=np.array([7e6,0,0]),sigma_BN=[0,0,0],v_BN_N=[0,0,0],omega_BN_B=[0,0,0])
    target=SimpleNamespace(r_BN_N=np.array([7e6+1,0,0]),sigma_BN=[0,0,0],v_BN_N=[0,0,0],omega_BN_B=[0,0,0])
    c.body_readers=[lambda:bus,lambda:target]
    c.audit=SimpleNamespace(record=lambda *args:[],feedback={'peak_target_contact_force_n':11.})
    c.own_motion=None;c.log_time=3.;c.control_file=io.StringIO()
    c.reference(3.)
    assert c.sequence.phase=='abort'
    assert c.sequence.failure_reason=='target_contact_force_over_10n'
    assert not c.target.command['deadman']
    assert c.target.command['end_effector_linear_velocity_body_m_s']==[0.,0.,0.]
