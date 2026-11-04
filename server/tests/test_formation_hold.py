import numpy as np
from simulation.formation_hold import reference, bounded_force

def test_same_circle_and_rotating_relative_velocity():
    r=np.array([6878136.6,0.,0.]); v=np.array([0.,7612.,0.])
    s=reference(r,v,1.5)
    assert abs(np.linalg.norm(s['target_position'])-np.linalg.norm(r))<1e-8
    assert abs(np.linalg.norm(s['target_velocity'])-np.linalg.norm(v))<1e-8
    assert np.allclose(s['target_velocity']-v,np.cross(s['omega'],s['target_position']-r),atol=1e-10)
    assert np.dot(s['service_rotation']@np.array([0,0,-1]),v/np.linalg.norm(v))>0.999999
    assert np.dot(s['target_rotation']@np.array([0,1,0]),-v/np.linalg.norm(v))>0.999999

def test_restoring_force_is_bounded_and_mass_aware():
    f=bounded_force(np.array([.01,0,0]),np.zeros(3),10.,.2)
    assert f[0]>0 and np.allclose(f[1:],0)
    assert np.allclose(bounded_force(np.array([.01,0,0]),np.zeros(3),20.,.2),2*f)
    assert np.isclose(np.linalg.norm(bounded_force(np.ones(3)*100,np.ones(3),10.,.2)),.2)

def test_component_uses_distinct_wrench_sources_and_restoring_torque():
    from pathlib import Path
    from types import SimpleNamespace as NS
    from Basilisk.architecture import messaging
    from Basilisk.utilities import RigidBodyKinematics as rbk
    from simulation.components.formation_hold import FormationHoldComponent
    r=np.array([6878136.6,0.,0.]);v=np.array([0.,7612.,0.]);ref=reference(r,v,1.5)
    origin=messaging.SCStatesMsg().write(messaging.SCStatesMsgPayload(r_BN_N=r.tolist(),v_BN_N=v.tolist()))
    states={}
    for name,rotation,p,vel in [('cubesat_bus',ref['service_rotation'],r,v),('capture_target',ref['target_rotation']@rbk.PRV2C([.01,0,0]).T,ref['target_position'],ref['target_velocity'])]:
        states[name]=messaging.SCStatesMsg().write(messaging.SCStatesMsgPayload(r_BN_N=p.tolist(),v_BN_N=vel.tolist(),sigma_BN=rbk.C2MRP(rotation.T).tolist(),omega_BN_B=(rotation.T@ref['omega']).tolist()))
    sources=set()
    def add_wrench(body,source,**kwargs):
        assert source not in sources
        sources.add(source)
    control=NS(tracking_error=NS(attRefInMsg=NS(subscribeTo=lambda m:None)),initial_inertia=np.eye(3))
    ctx=NS(extra={'attitude_control':control},ports=NS(orbital_providers=lambda:(NS(scStateOutMsg=origin),None),body_state=states.__getitem__,add_body_wrench=add_wrench),config=NS(publish_stride=8),add=lambda *a,**k:None,keep_alive=lambda *a:None)
    component=FormationHoldComponent(dict(separation_m=1.5,force_limit_n=.2,torque_limit_nm=.02),Path(__file__).resolve().parents[1]/'model/SARM/platform/sarm_dual_capture26.xml')
    component.install(ctx);component.guidance.UpdateState(0)
    assert component.guidance.state['target_torque_nm'][0]<0
    assert len(sources)==2
    assert np.linalg.norm(component.guidance.state['relative_velocity_rotating_m_s'])<1e-9
    # The orbital carrier publishes one dynamics step later than body messages.
    # Without timestamp alignment this looks like a false 19-metre error.
    dt=.0025;n=np.linalg.norm(v)/np.linalg.norm(r);angle=n*dt
    r2=np.linalg.norm(r)*np.array([np.cos(angle),np.sin(angle),0.])
    v2=np.linalg.norm(v)*np.array([-np.sin(angle),np.cos(angle),0.])
    origin.write(messaging.SCStatesMsgPayload(r_BN_N=r2.tolist(),v_BN_N=v2.tolist()),2_500_000)
    component.guidance.UpdateState(2_500_000)
    assert np.linalg.norm(component.guidance.state['position_error_m'][0])<1e-7
    assert np.linalg.norm(component.guidance.state['force_n'][0])<1e-5
    rotation=ref['target_rotation']
    states['capture_target'].write(messaging.SCStatesMsgPayload(r_BN_N=ref['target_position'].tolist(),v_BN_N=ref['target_velocity'].tolist(),sigma_BN=rbk.C2MRP(rotation.T).tolist(),omega_BN_B=(rotation.T@ref['omega']).tolist()))
    component.guidance.UpdateState(2_500_000)
    assert np.all(np.isfinite(component.guidance.state['target_torque_nm']))
    assert np.linalg.norm(component.guidance.state['target_torque_nm'])<1e-8
