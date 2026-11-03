import json
from pathlib import Path
import numpy as np
from simulation import capture_sequence


def packet(p=(0.,0.,-.5),session='s',age=.1):
    return dict(valid=True,status='tracking',session_id=session,source_frame_id='10',
        sim_time_ns='1000000000',age_s=age,published_wall_time_ns=10_000_000_000,
        result={'alignment':{'valid':True,'interface_position_grip_m':list(p),
            'R_grip_from_interface':np.eye(3).tolist()}})


def probe(tmp_path):
    cls=getattr(capture_sequence,'VisualCaptureProbe',None)
    assert cls is not None, 'image-driven probe must not use truth-driven capture commands'
    p=tmp_path/'input.json';p.write_text(json.dumps(packet()))
    v=cls(p,'s',goal_gap_m=.48,wall_now_ns=lambda:10_000_000_000)
    v.sequence.phase='approach'
    return v,p


def test_visual_position_generates_bounded_motion_without_target_truth(tmp_path):
    v,p=probe(tmp_path)
    q=np.zeros(12)
    a,_=v.command(1.,q,np.eye(3),{})
    assert a['deadman'] and a['end_effector_linear_velocity_body_m_s'][2]<0
    assert np.linalg.norm(a['end_effector_linear_velocity_body_m_s'])<=.0020001
    x=packet(p=(.001,0.,-.48));x['truth_position']=[-100,0,0]
    p.write_text(json.dumps(x))
    a,_=v.command(1.,q,np.eye(3),{})
    assert a['end_effector_linear_velocity_body_m_s'][0]>0


def test_failed_stale_or_previous_session_stops_instead_of_using_truth(tmp_path):
    v,p=probe(tmp_path)
    for x in ({'valid':False,'status':'lost'},packet(age=2.),packet(session='old')):
        p.write_text(json.dumps(x))
        a,status=v.command(1.,np.zeros(12),np.eye(3),{})
        assert not a['deadman'] and np.allclose(a['end_effector_linear_velocity_body_m_s'],0)
        assert status.startswith('visual_waiting')


def test_publication_delay_is_included_and_visual_goal_can_stop(tmp_path):
    v,p=probe(tmp_path)
    x=packet();x['published_wall_time_ns']=8_000_000_000;p.write_text(json.dumps(x))
    assert not v.command(1.,np.zeros(12),np.eye(3),{})[0]['deadman']
    p.write_text(json.dumps(packet(p=(0,0,-.48))))
    a,status=v.command(1.,np.zeros(12),np.eye(3),{})
    assert status=='visual_probe_complete' and not a['deadman']


def test_invalid_rotation_cannot_drive_arm(tmp_path):
    v,p=probe(tmp_path);x=packet();x['result']['alignment']['R_grip_from_interface']=[[0]*3]*3
    p.write_text(json.dumps(x))
    assert not v.command(1.,np.zeros(12),np.eye(3),{})[0]['deadman']
