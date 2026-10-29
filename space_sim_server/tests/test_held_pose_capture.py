import json
import numpy as np
from simulation.visual_predictive_capture import VisualPredictiveCapture
from simulation import serial_chain_kinematics as kin
import importlib.util
own=__import__('simulation.own_motion',fromlist=['OwnMotionFeedback']) if importlib.util.find_spec('simulation.own_motion') else None

def measured_gyro(ns,rate):
    return dict(sensor_name='service_gyro',body_name='cubesat_bus',frame='spacecraft_body',
        reference_frame='J2000',measurement='simulated_body_angular_rate',sample_rate_hz=50,
        sample_time_ns=str(ns),angular_velocity_body_rad_s=list(rate),error_variance_rad2_s2=[0.]*3,
        saturated_axes=[False]*3,valid=True)


def test_encoder_com_shift_compensates_free_base_without_body_position(tmp_path):
    assert own is not None,'encoder/gyro own-motion observer is not implemented'
    p=tmp_path/'m.xml'
    p.write_text('<mujoco><worldbody><body name="cubesat_bus"><body name="moving"><joint name="j" type="slide" axis="1 0 0"/></body></body></worldbody></mujoco>')
    masses={'cubesat_bus':np.r_[9.,[0,0,0],np.zeros(7)],'moving':np.r_[1.,[0,0,0],np.zeros(7)]}
    c=own.EncoderMassModel(p,lambda name:masses[name])
    assert np.allclose(c.com({'j':1.}),[.1,0,0])
    o=own.OwnMotionFeedback()
    for k in range(101):
        pos,r=o.update(k*20_000_000,measured_gyro(k*20_000_000,[0,0,0]),
                       c.com({'j':float(k==100)}),epoch=('s','0'))
    assert np.allclose(pos,[-.1,0,0]) and np.allclose(r,np.eye(3))


def test_gyro_subtracts_initial_common_orbit_spin_and_tracks_recoil():
    assert own is not None,'encoder/gyro own-motion observer is not implemented'
    o=own.OwnMotionFeedback();com=np.array([.2,0,0])
    for k in range(101):
        pos,r=o.update(k*20_000_000,measured_gyro(k*20_000_000,[0,0,.001 if k<100 else .101]),com,epoch=('s','0'))
    assert np.allclose(r,np.eye(3),atol=1e-9)
    for k in range(101,151):
        pos,r=o.update(k*20_000_000,measured_gyro(k*20_000_000,[0,0,.101]),com,epoch=('s','0'))
    assert np.allclose(r,kin.axis_angle_to_matrix(np.array([0,0,1]),.1),atol=1e-8)
    assert np.allclose(pos+r@com,com)


def test_held_target_pose_does_not_extrapolate_visual_velocity(tmp_path):
    path=tmp_path/'v.json'
    c=VisualPredictiveCapture(path,'s',held_pose=True,wall_now_ns=lambda:10_000_000_000)
    c.sequence.phase='approach';q=np.zeros(12);q[6]=-.006
    for t in np.linspace(1,2,11):
        path.write_text(json.dumps(dict(valid=True,status='tracking',session_id='s',source_frame_id=str(t),
            sim_time_ns=str(round(t*1e9)),age_s=.1,published_wall_time_ns=10_000_000_000,
            result={'alignment':dict(valid=True,interface_position_grip_m=[.0002*t,0,-.1],R_grip_from_interface=np.eye(3).tolist())})))
        c.command(t,q,(np.zeros(3),np.eye(3)),{},reference_from_base=(np.zeros(3),np.eye(3)))
    last=c.p_base.copy();path.write_text(json.dumps(dict(valid=False,status='lost',session_id='s')))
    c.command(3,q,(np.array([0,0,-.01]),np.eye(3)),{},reference_from_base=(np.array([-.002,0,0]),np.eye(3)))
    assert np.allclose(c.p_base,last) and np.allclose(c.v_base,0) and np.allclose(c.w_base,0)
    assert np.allclose(c.predicted_position_grip,last-np.array([-.002,0,-.01]))


def test_held_pose_refuses_to_move_without_own_motion_feedback(tmp_path):
    c=VisualPredictiveCapture(tmp_path/'no.json','s',held_pose=True)
    a,status=c.command(1,np.zeros(12),(np.zeros(3),np.eye(3)),{})
    assert not a['deadman'] and 'own_motion' in status


def test_unexpected_jaw_contact_stops_before_pushing_with_stale_vision(tmp_path):
    path=tmp_path/'v.json';c=VisualPredictiveCapture(path,'s',held_pose=True,wall_now_ns=lambda:10_000_000_000)
    c.sequence.phase='approach'
    path.write_text(json.dumps(dict(valid=True,status='tracking',session_id='s',source_frame_id='1',
        sim_time_ns='1000000000',age_s=.1,published_wall_time_ns=10_000_000_000,
        result={'alignment':dict(valid=True,interface_position_grip_m=[0,0,-.013],R_grip_from_interface=np.eye(3).tolist())})))
    a,status=c.command(1,np.zeros(12),(np.zeros(3),np.eye(3)),{'peak_target_contact_force_n':.4,'pad_force_n':0},reference_from_base=(np.zeros(3),np.eye(3)))
    assert not a['deadman'] and 'jaw_contact_before_seated' in status
