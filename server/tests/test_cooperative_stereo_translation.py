import numpy as np
import pytest

cv2 = pytest.importorskip('cv2', reason='optional vision environment required')

from space_arm_platform.cooperative_stereo_translation import CooperativeStereoTranslation


def fixture():
    k=np.array([[1800.,0,959.5],[0,1800.,959.5],[0,0,1.]])
    cal=dict(K_left=k.tolist(),K_right=k.tolist(),D_left=[0.]*5,D_right=[0.]*5,
             image_size=[1920,1920],R_right_from_left=np.eye(3).tolist(),t_right_from_left_m=[-.12,0,0])
    layout=dict(dictionary='DICT_4X4_50',markers={str(i):
        [[x-.025,y-.025,0],[x+.025,y-.025,0],[x+.025,y+.025,0],[x-.025,y+.025,0]]
        for i,(x,y) in enumerate([(-.12,-.1),(.12,-.1),(-.12,.1),(.12,.1)])})
    r=cv2.Rodrigues(np.array([.2,-.1,.1]))[0]; t=np.array([.02,-.03,2.2])
    obs=[]
    for eye in (0,1):
        obs.append({i:cv2.projectPoints(np.array(layout['markers'][str(i)],float),cv2.Rodrigues(r)[0],
            t+[-.12*eye,0,0],k,np.zeros(5))[0].reshape(4,2) for i in (range(2) if eye==0 else range(2,4))})
    estimator=CooperativeStereoTranslation(cal,layout)
    return estimator,obs,r,t


def test_different_eye_markers_recover_position_without_translation_prior(monkeypatch):
    estimator,obs,r,t=fixture()
    values=iter(obs);monkeypatch.setattr(estimator,'detect',lambda blob:next(values))
    out=estimator.estimate_aided(b'left',b'right',rotation_target_to_left=r,attitude_stamp_ns=100,image_stamp_ns=100)
    m=out['measurement']
    assert m['status']=='ok' and m['independent_image_attitude'] is False
    np.testing.assert_allclose(m['pose_target_to_left']['t_m'],t,atol=1e-8)
    assert set(m['used_ids']['left']).isdisjoint(m['used_ids']['right'])


def test_wrong_attitude_and_wrong_epoch_do_not_produce_false_ok(monkeypatch):
    estimator,obs,r,t=fixture()
    with pytest.raises(ValueError,match='image epoch'):
        estimator.estimate_aided(b'',b'',rotation_target_to_left=r,attitude_stamp_ns=99,image_stamp_ns=100)
    values=iter(obs);monkeypatch.setattr(estimator,'detect',lambda blob:next(values))
    wrong=cv2.Rodrigues(np.array([1.,.4,-.2]))[0]
    result=estimator.estimate_aided(b'',b'',rotation_target_to_left=wrong,attitude_stamp_ns=100,image_stamp_ns=100)
    assert result['measurement']['status']=='rejected'


def test_missing_eye_rejects_even_with_cooperative_attitude(monkeypatch):
    estimator,obs,r,t=fixture()
    values=iter([obs[0],{}]);monkeypatch.setattr(estimator,'detect',lambda blob:next(values))
    result=estimator.estimate_aided(b'',b'',rotation_target_to_left=r,attitude_stamp_ns=100,image_stamp_ns=100)
    assert result['measurement']['status']=='rejected'
