"""Formal route selection, continuous sensor history and control ownership."""
from copy import deepcopy
import json
from pathlib import Path
import sys
from types import SimpleNamespace

import numpy as np
import pytest

from simulation.capture26_mission import load_mission, load_vision, vision_path
from simulation.capture26_near23 import Near23Route
from simulation.gyroscope import Gyroscope, GyroConfig
from simulation.near_field.orbital_guidance import vbar_frame
from simulation.orbital_rendezvous.reference import FlightProfile
from simulation.own_motion import OwnMotionFeedback
from space_arm_platform.rendezvous_config import FlightOptions

ROOT=Path(__file__).resolve().parents[1]


def test_formal_cli_freezes_selected_profile_at_far_start(tmp_path,monkeypatch):
    from simulation import capture26_run
    received=[]
    monkeypatch.setitem(sys.modules,'simulation.capture26_runtime',SimpleNamespace(run=received.append))
    options=tmp_path/'options.json'
    options.write_text(json.dumps(FlightOptions(model_id='sarm-dual-capture26',capture26_profile='near23_vbar_v1').to_dict()))
    monkeypatch.setattr(sys,'argv',['capture26_run','--scenario','in-orbit','--config',str(options),'--output-dir',str(tmp_path/'out')])
    capture26_run.main()
    args,=received
    assert args.mission_stage=='far' and args.handoff is None
    assert args.mission_config.is_near23 and not args.mission_config.is_demo
    assert args.mission_config.feedback_source=='navigation_estimate'
    assert vision_path(args.mission_config).name=='capture26_near23_front_mount_20261029.json'
    assert load_mission(ROOT).is_demo  # An explicit new route does not relabel old runs.
    with pytest.raises(ValueError):FlightOptions(capture26_profile='near23_vbar_v1')


def test_gyro_odometry_restores_history_without_recalibration_or_time_reset():
    sensor=Gyroscope(GyroConfig(),seed=7)
    first=OwnMotionFeedback()
    for k in range(101):
        sensor.sample(k*20_000_000,[.001,0.,0.])
        first.update(k*20_000_000,sensor.latest,[.1,.2,.3],epoch=('mission','continuous'))
    saved=json.loads(json.dumps(first.snapshot()))
    second=OwnMotionFeedback();second.restore(saved)
    assert second.snapshot()==saved
    for k in range(101,111):
        sensor.sample(k*20_000_000,[.001,.002,0.])
        a=first.update(k*20_000_000,sensor.latest,[.2,.2,.3],epoch=('mission','continuous'))
        b=second.update(k*20_000_000,sensor.latest,[.2,.2,.3],epoch=('mission','continuous'))
        for x,y in zip(a,b):np.testing.assert_array_equal(x,y)
    damaged=deepcopy(saved);damaged['unique_samples']-=1
    with pytest.raises(ValueError):OwnMotionFeedback().restore(damaged)
    with pytest.raises(ValueError,match='gyro_epoch_changed'):
        second.update(2_200_000_000,sensor.latest,[.2,.2,.3],epoch=('other','continuous'))


def test_near_receives_late_absolute_epoch_and_vbar_remains_the_reference_owner():
    cfg=load_mission(ROOT,'near23_vbar_v1')
    route=Near23Route(ROOT,load_vision(ROOT,cfg),epoch_ns=5_005_000_000_000)
    p,v,_=FlightProfile().target(5005.)
    frame,omega=vbar_frame(np.r_[p,v])
    relative=frame@np.array([110.,0.,0.])
    rs,rt=frame@route.service_mount,frame@route.target_mount
    state=dict(p=np.array([p-relative,p]),v=np.array([v-np.cross(omega,relative),v]),
        r=np.array([rs,rt]),rates=np.array([rs.T@omega,rt.T@omega]),b=np.zeros((2,3)))
    inputs=dict(service_com=np.zeros(3),target_com=np.zeros(3),q=route.q_prepared,qdot=np.zeros(6),
        covariance=np.eye(6)*1e-6,sigma=.001,healthy=True,visual_ready=False,visual_stamp_ns=None,mass=300.)
    for k in range(4):
        now=route.epoch_ns+k*50_000_000
        g=route.step(now,state,last_gnss_ns=now,last_cooperative_ns=now,**inputs)
        assert g['reference_owner']=='vbar' and not g['released']
    np.testing.assert_allclose(g['position_goal'],state['p'][0],atol=1e-8)
    assert route.sequence.epoch==5005.
    assert route.sequence.vbar.original_camera_policy.gnss_bridge_enabled
    assert not route.sequence.vbar.original_camera_policy.gnss_bridge_diagnostic
    assert g['vbar']['mission_profile']=='near23_vbar_v1'
    assert route.sequence.image_release_s is None


def test_completed_preparation_does_not_gain_credit_from_elapsed_mission_time():
    from simulation.capture26_flight import Capture26Flight
    cfg=load_mission(ROOT,'near23_vbar_v1');flight=Capture26Flight('in-orbit',config=cfg)
    p0,v0,_=flight.profile.chaser(5000.);p1,v1,_=flight.profile.target(5000.)
    frame,w=vbar_frame(np.r_[p1,v1]);r=np.array([frame,frame])
    s=dict(p=np.array([p0,p1]),v=np.array([v0,v1]),r=r,rates=np.array([frame.T@w]*2),b=np.zeros((2,3)))
    for k in range(102):
        g=flight.step(5_000_000_000_000+k*50_000_000,s,healthy=True,preparation_ready=False)
        assert not g['released']
    assert not flight.safe_hold


@pytest.mark.parametrize('damage',[None,'physical','gyro_history'])
def test_receiver_audit_blocks_changed_native_state_and_sensor_history(tmp_path,damage):
    from simulation.capture26_near23 import verify_handoff
    navigation={key:[] for key in ('base','events','snapshots','accepted','base_committed','last_visual_epoch')}
    navigation['state']={'time_ns':100_000_000}
    source=dict(phase='FarReady',physical=dict(time_ns=100_000_000,qpos_inertial=[1.,2.],
        qvel_inertial=[0.,0.],origin_position_m=[0.,0.,0.],origin_velocity_m_s=[0.,0.,0.]),
        navigation=navigation,gyros=[{'sample':5}],gnss_receiver={},gnss_pending=[],retention=None,
        control=dict(operating_reference=[0.]*8,capture_reference=[0.]*12,
            own_motion=dict(last_sample_ns=100_000_000,unique_samples=6)),environment=[])
    received=deepcopy(source)
    if damage=='physical':received['physical']['qpos_inertial'][0]+=.001
    if damage=='gyro_history':received['control']['own_motion']['unique_samples']=1
    path=tmp_path/'handoff.json';path.write_text(json.dumps(source))
    (tmp_path/'restored-state.json').write_text(json.dumps(received))
    reasons=[];route=SimpleNamespace(_hold=reasons.append);capture=SimpleNamespace(failure=None)
    audit=verify_handoff(path,tmp_path,route,capture)
    assert audit['continuity_passed']==(damage is None)
    assert bool(capture.failure)==bool(reasons)==(damage is not None)
    if damage is not None:
        assert capture.failure=='far_handoff_continuity_failed'
        assert not audit['mission_accepted']


@pytest.mark.parametrize('damage',[None,'transverse_error','force_gap'])
def test_independent_score_uses_frozen_nodes_and_continuous_physical_force(tmp_path,damage):
    from simulation.capture26_near23_score import score_near
    names=('Vbar50m','Optical17p4m','GNSS5p5m','PlatformStandoff')
    targets=[np.array([d,0.,0.]) for d in (50.,17.4,5.5,.6)]
    gates={name:SimpleNamespace(completed_at_s=5.+10*i) for i,name in enumerate(names)}
    seq=SimpleNamespace(epoch=100.,vbar=SimpleNamespace(gates=gates,targets=targets,node_ids=names))
    center=np.array([.1,.05,.04])
    route=SimpleNamespace(sequence=seq,geometry=SimpleNamespace(center=center))
    p1,v1=np.array([7.e6,0.,0.]),np.array([0.,7500.,0.])
    frame,omega=vbar_frame(np.r_[p1,v1])
    with (tmp_path/'score.jsonl').open('w') as stream:
        for i,(name,target) in enumerate(zip(names,targets)):
            relative=frame@(target+([0.,.06,0.] if damage=='transverse_error' and i==0 else np.zeros(3)))
            for k in range(101):
                row=dict(time_s=100.+10*i+k*.05,
                    truth_com_position_n_m=[(p1-relative).tolist(),p1.tolist()],
                    truth_com_velocity_n_m_s=[(v1-np.cross(omega,relative)).tolist(),v1.tolist()],
                    truth_rotation_n_from_body=[np.eye(3).tolist()]*2,truth_com_body_m=[[0.,0.,0.]]*2,
                    encoder_geometry=dict(cubesat_bus=dict(sites=dict(grip_frame=dict(
                        position_body_m=(relative+center+[0.,0.,.5]).tolist(),rotation_body_from_site=np.eye(3).tolist())))),
                    # A corrupt control-side reference cannot hide the 6 cm error.
                    guidance=dict(vbar=dict(reference_com_m=(frame.T@relative).tolist())))
                stream.write(json.dumps(row)+'\n')
    with (tmp_path/'contacts_400hz_force.jsonl').open('w') as stream:
        for k in range(8001):
            if damage=='force_gap' and k==4000:continue
            stream.write(json.dumps(dict(time_s=200.+k*.0025,phase='hold_force',self_contact_count=0,
                feedback=dict(jaw_support_count=4,jaw_force_n=5.,peak_target_contact_force_n=2.)))+'\n')
    audit=score_near(tmp_path,route)
    assert audit['passed']==(damage!='transverse_error')
    assert audit['force_hold']['passed']==(damage!='force_gap')
    if damage=='transverse_error':
        assert audit['nodes']['Vbar50m']['max_position_m']==pytest.approx(.06,abs=1e-8)
