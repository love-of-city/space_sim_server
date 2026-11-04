import numpy as np
import pytest
from simulation.formation_hold import reference

Q=np.array([np.cos(np.deg2rad(55)),np.sin(np.deg2rad(55)),0,0])

def test_coorbital_service_rotation_keeps_same_circle_and_velocity():
 r=np.array([6878136.6,0.,0.]);v=np.array([0.,7612.,0.]);a=reference(r,v,1.05)
 b=reference(r,v,1.05,service_body_rotation_wxyz=Q)
 R=np.array([[1,0,0],[0,np.cos(np.deg2rad(110)),-np.sin(np.deg2rad(110))],[0,np.sin(np.deg2rad(110)),np.cos(np.deg2rad(110))]])
 np.testing.assert_allclose(b['service_rotation'],a['service_rotation']@R,atol=1e-12)
 for key in ['target_position','target_velocity','target_rotation','omega']:np.testing.assert_allclose(b[key],a[key],atol=1e-12)
 assert abs(np.linalg.norm(b['target_position'])-np.linalg.norm(r))<1e-8

@pytest.mark.parametrize('q',[[0,0,0,0],[1,0,0],[float('nan'),0,0,1]])
def test_invalid_service_rotation_rejected(q):
 with pytest.raises(ValueError):reference([6878136.6,0,0],[0,7612,0],1.05,service_body_rotation_wxyz=q)

def test_attitude_guidance_keeps_configured_body_rotation():
 from types import SimpleNamespace as NS
 from Basilisk.architecture import messaging
 from Basilisk.utilities import RigidBodyKinematics as rbk
 from simulation.components.orbit_attitude_reference import OrbitAttitudeReference
 r=np.array([6878136.6,0.,0.]);v=np.array([0.,7612.,0.])
 state=messaging.SCStatesMsg().write(messaging.SCStatesMsgPayload(r_BN_N=r.tolist(),v_BN_N=v.tolist()))
 control=NS(tracking_error=NS(attRefInMsg=NS(subscribeTo=lambda m:None)),initial_inertia=np.eye(3))
 ctx=NS(extra={'attitude_control':control},ports=NS(body_state=lambda n:state))
 model=OrbitAttitudeReference(ctx,service_body_rotation_wxyz=Q);reader=model.output.addSubscriber();model.UpdateState(0)
 expected=reference(r,v,1.05,service_body_rotation_wxyz=Q)['service_rotation']
 np.testing.assert_allclose(rbk.MRP2C(reader().sigma_RN).T,expected,atol=1e-12)
