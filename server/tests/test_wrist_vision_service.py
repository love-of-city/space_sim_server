import threading
import numpy as np
from space_arm_platform.wrist_vision_service import WristVisionService


def packet(frame=1,session='s',**extra):
    return dict(camera_id='teleop/camera/capture_wrist_cam_alignment',session_id=session,
                source_frame_id=str(frame),sim_time_ns=str(frame*100000000),**extra)


def valid():
    return {'status':'tracking','alignment':{'valid':True,'control_enabled':False,
        'position_error_display_mm':[1,2,3],'rotation_error_display_deg':[0,0,0]},'detections':[]}


def service(tmp_path,solver,now=lambda:1.):
    return WristVisionService(tmp_path,estimator=solver,config={'image_size':[1024,1024]},now=now)


def test_native_pixels_only_reach_solver_and_loss_clears_numbers(tmp_path):
    calls=[]
    s=service(tmp_path,lambda p:calls.append(p) or (valid() if p==b'good' else {'status':'lost','alignment':{'valid':False}}))
    s.start()
    try:
        s.on_capture(packet(extrinsics={'secret':'truth'}),{'rgb':b'good'})
        assert s.wait_idle() and s.status()['valid']
        assert calls==[b'good']
        s.on_capture(packet(2),{'rgb':b'bad'})
        assert s.wait_idle()
        state=s.status()
        assert not state['valid'] and state['status']=='lost' and state['result'] is None
    finally:s.close()


def test_stale_success_and_exception_never_retain_errors(tmp_path):
    clock=[1.]
    def solve(p):
        if p==b'bad':raise ValueError('decode failed')
        return valid()
    s=service(tmp_path,solve,lambda:clock[0]);s.start()
    try:
        s.on_capture(packet(),{'rgb':b'good'});assert s.wait_idle()
        clock[0]=4
        assert s.status()['status']=='stale' and s.status()['result'] is None
        s.on_capture(packet(2),{'rgb':b'bad'});assert s.wait_idle()
        assert s.status()['status']=='error' and s.status()['result'] is None
    finally:s.close()


def test_capture_error_cannot_be_overwritten_by_inflight_old_success(tmp_path):
    entered,release=threading.Event(),threading.Event()
    def solve(p):entered.set();release.wait(2);return valid()
    s=service(tmp_path,solve);s.start()
    try:
        s.on_capture(packet(),{'rgb':b'good'});assert entered.wait(1)
        s.on_capture(packet(2,type='capture_error',error='readback failed'),{})
        release.set();assert s.wait_idle()
        assert s.status()['status']=='capture_error' and s.status()['result'] is None
    finally:release.set();s.close()


def test_new_session_and_out_of_order_frames_do_not_reuse_old_pose(tmp_path):
    s=service(tmp_path,lambda p:valid());s.start()
    try:
        s.on_capture(packet(8),{'rgb':b'good'});assert s.wait_idle()
        s.on_capture(packet(7),{'rgb':b'old'});assert s.wait_idle()
        assert s.status()['source_frame_id']=='8'
        s.invalidate('resetting')
        assert s.status()['result'] is None
        s.on_capture(packet(1,session='new'),{'rgb':b'new'});assert s.wait_idle()
        assert s.status()['session_id']=='new' and s.status()['source_frame_id']=='1'
    finally:s.close()


def test_old_source_frame_is_invalid_even_if_just_received(tmp_path):
    calls=[]
    s=WristVisionService(tmp_path,config={'image_size':[1024,1024]},
        estimator=lambda p:calls.append(p) or valid(),now=lambda:10.,wall_now_ns=lambda:3000000000)
    s.on_capture(packet(source_wall_time_ns='1000000000'),{'rgb':b'old'})
    assert not calls and s.status()['status']=='stale' and s.status()['result'] is None


def test_opt_in_export_keeps_images_separate_from_truth_and_invalidates(tmp_path):
    import json
    (tmp_path/'run').mkdir()
    dest=tmp_path/'evaluation'
    (tmp_path/'run/vision_runtime.json').write_text(json.dumps({'wrist_evaluation_directory':str(dest)}))
    s=service(tmp_path,lambda p:valid());s.start()
    try:
        s.on_capture(packet(extrinsics={'forbidden':'truth'}),{'rgb':b'good'})
        assert s.wait_idle()
        s.publish_evaluation()
        exported=json.loads((tmp_path/'run/wrist_alignment_observation.json').read_text())
        assert exported['valid'] and 'published_wall_time_ns' in exported
        assert 'forbidden' not in json.dumps(exported)
        s.invalidate('lost');s.publish_evaluation()
        assert json.loads((tmp_path/'run/wrist_alignment_observation.json').read_text())['result'] is None
    finally:s.close()
