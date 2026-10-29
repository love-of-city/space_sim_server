import json
import numpy as np
import importlib.util
mod=__import__('simulation.visual_predictive_capture',fromlist=['VisualPredictiveCapture']) if importlib.util.find_spec('simulation.visual_predictive_capture') else None


def setup(tmp_path):
    assert mod is not None, 'visual prediction and contact capture controller is not implemented'
    path=tmp_path/'vision.json'
    c=mod.VisualPredictiveCapture(path,'s',wall_now_ns=lambda:10_000_000_000)
    q=np.zeros(12);q[6]=-.006
    return c,path,q


def image(path,t,p=(0,0,-.1),session='s'):
    path.write_text(json.dumps(dict(valid=True,status='tracking',session_id=session,
        source_frame_id=str(round(t*1000)),sim_time_ns=str(round(t*1e9)),age_s=.1,
        published_wall_time_ns=10_000_000_000,result={'alignment':dict(valid=True,
        interface_position_grip_m=list(p),R_grip_from_interface=np.eye(3).tolist())})))


def test_lost_image_uses_encoder_motion_not_frozen_old_relative_pose(tmp_path):
    c,p,q=setup(tmp_path);c.sequence.phase='approach'
    for t in np.linspace(.5,1.,6):
        image(p,t);c.command(t,q,(np.zeros(3),np.eye(3)),{})
    p.write_text(json.dumps(dict(valid=False,status='lost',session_id='s')))
    a,status=c.command(2,q,(np.array([0,0,-.02]),np.eye(3)),{})
    assert a['deadman'] and status.startswith('predictive_')
    assert np.allclose(c.predicted_position_grip,[0,0,-.08])


def test_delayed_image_is_registered_with_source_time_encoder_pose(tmp_path):
    c,p,q=setup(tmp_path);c.sequence.phase='approach'
    p.write_text(json.dumps(dict(valid=False,status='waiting',session_id='s')))
    c.command(1,q,(np.zeros(3),np.eye(3)),{})
    image(p,1);c.command(1.1,q,(np.array([0,0,-.01]),np.eye(3)),{})
    assert np.allclose(c.predicted_position_grip,[0,0,-.09])


def test_scene_change_and_prediction_budget_stop(tmp_path):
    c,p,q=setup(tmp_path);c.sequence.phase='approach'
    image(p,1);c.command(1,q,(np.zeros(3),np.eye(3)),{})
    image(p,2,session='other')
    assert not c.command(2,q,(np.zeros(3),np.eye(3)),{})[0]['deadman']
    assert c.sequence.phase=='abort'
    c,p,q=setup(tmp_path);c.sequence.phase='approach'
    image(p,1);c.command(1,q,(np.zeros(3),np.eye(3)),{})
    p.write_text(json.dumps(dict(valid=False,status='lost',session_id='other')))
    assert not c.command(2,q,(np.zeros(3),np.eye(3)),{})[0]['deadman']
    assert c.sequence.phase=='abort'
    assert c.sequence.failure_reason=='scene_changed'
    c,p,q=setup(tmp_path);c.sequence.phase='approach'
    image(p,1);c.command(1,q,(np.zeros(3),np.eye(3)),{})
    p.write_text(json.dumps(dict(valid=False,status='lost',session_id='s')))
    assert not c.command(100,q,(np.zeros(3),np.eye(3)),{})[0]['deadman']


def test_contact_near_seat_enters_force_closure_and_regulates_tension(tmp_path):
    c,p,q=setup(tmp_path);c.sequence.phase='approach'
    image(p,1,p=(0,0,-.006))
    f=dict(pad_force_n=.4,pad_compression_m=.0001,jaw_force_n=0,jaw_support_count=0,
        peak_target_contact_force_n=.4)
    a,status=c.command(1,q,(np.zeros(3),np.eye(3)),f)
    assert c.sequence.phase=='close' and np.allclose(a['end_effector_linear_velocity_body_m_s'],0)
    c.sequence.phase='tighten';c.sequence.start=1
    f.update(pad_force_n=5,jaw_force_n=5,jaw_support_count=4)
    c.command(2,q,(np.zeros(3),np.eye(3)),f)
    a,status=c.command(3.1,q,(np.zeros(3),np.eye(3)),f)
    assert c.sequence.phase=='hold_force'
    assert np.allclose(a['end_effector_linear_velocity_body_m_s'],0)


def test_no_contact_does_not_claim_capture_and_early_contact_aborts(tmp_path):
    c,p,q=setup(tmp_path);c.sequence.phase='approach'
    image(p,1,p=(0,0,-.006))
    c.command(1,q,(np.zeros(3),np.eye(3)),{})
    assert c.sequence.phase not in ('close','tighten','hold_force')
    c,p,q=setup(tmp_path);c.sequence.phase='approach'
    image(p,1,p=(0,0,-.1))
    a,_=c.command(1,q,(np.zeros(3),np.eye(3)),dict(peak_target_contact_force_n=1.))
    assert not a['deadman'] and c.sequence.phase=='abort'


def test_startup_waits_for_own_session_instead_of_aborting_on_previous_run_file(tmp_path):
    c,p,q=setup(tmp_path);image(p,1,session='previous')
    a,status=c.command(1,q,(np.zeros(3),np.eye(3)),{})
    assert not a['deadman'] and c.sequence.phase=='wait'


def test_visual_velocity_prediction_and_wrist_rotation_are_compensated(tmp_path):
    c,p,q=setup(tmp_path);c.sequence.phase='approach'
    for t in np.linspace(1.,2.,11):
        image(p,t,p=(.0002*t,0,-.1))
        c.command(t,q,(np.zeros(3),np.eye(3)),{})
    p.write_text(json.dumps(dict(valid=False,status='lost',session_id='s')))
    r=np.array([[0.,-1,0],[1,0,0],[0,0,1]])
    c.command(3,q,(np.zeros(3),r),{})
    assert np.allclose(c.predicted_position_grip,r.T@np.array([.0006,0,-.1]),atol=1e-8)
