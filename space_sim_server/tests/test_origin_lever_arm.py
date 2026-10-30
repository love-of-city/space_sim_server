import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from simulation.origin_lever_arm import OriginLeverArm


def sample(model,t,*,service=False,gradient=None):
    rotation=Rotation.from_rotvec([0,0,t]).as_matrix()
    args=dict(target_rotation=np.eye(3),service_rotation=np.eye(3),
              target_com=np.zeros(3),service_com=np.zeros(3),
              target_com_rate=np.zeros(3),service_com_rate=np.zeros(3),
              target_omega=np.zeros(3),service_omega=np.zeros(3))
    role='service' if service else 'target'
    args[role+'_rotation']=rotation
    args[role+'_com']=np.array([1.,0,0])
    args[role+'_omega']=np.array([0.,0,1.])
    return model.update(t,np.zeros((3,3)) if gradient is None else gradient,**args)


@pytest.mark.parametrize('service',[False,True])
def test_rotating_origin_acceleration_has_the_correct_sign(service):
    model=OriginLeverArm()
    sample(model,0.,service=service)
    t=.05
    result=sample(model,t,service=service)
    # Body origin = COM - R*c, so origin acceleration opposes lever acceleration.
    mean_lever_acc=np.array([-np.sin(t),np.cos(t)-1,0])/t
    np.testing.assert_allclose(result,mean_lever_acc*(1 if service else -1),atol=1e-12)


def test_gravity_is_evaluated_at_assembly_centers():
    gradient=np.diag([2e-6,-1e-6,-1e-6])
    result=sample(OriginLeverArm(),0.,gradient=gradient)
    np.testing.assert_array_equal(result,gradient@np.array([1.,0,0]))


def test_joint_induced_center_acceleration_rotates_into_navigation_axes():
    model=OriginLeverArm()
    rotation=Rotation.from_rotvec([0,0,np.pi/2]).as_matrix()
    args=dict(target_rotation=rotation,service_rotation=np.eye(3),
              target_com=np.zeros(3),service_com=np.zeros(3),
              target_com_rate=np.zeros(3),service_com_rate=np.zeros(3),
              target_omega=np.zeros(3),service_omega=np.zeros(3))
    model.update(0.,np.zeros((3,3)),**args)
    # Joint motion accelerates the target COM lever by +2 m/s² along body X.
    args.update(target_com=np.array([.0025,0,0]),target_com_rate=np.array([.1,0,0]))
    result=model.update(.05,np.zeros((3,3)),**args)
    np.testing.assert_allclose(result,[0,-2,0],atol=1e-12)


def test_rejected_epoch_does_not_corrupt_last_telemetry():
    model=OriginLeverArm()
    sample(model,0.)
    for t in (0.,-.1,1.,float('nan')):
        with pytest.raises(ValueError):sample(model,t)
    expected=sample(OriginLeverArm(),0.)
    assert np.isfinite(expected).all()
    result=sample(model,.05)
    np.testing.assert_allclose(result,[np.sin(.05)/.05,(1-np.cos(.05))/.05,0],atol=1e-12)
