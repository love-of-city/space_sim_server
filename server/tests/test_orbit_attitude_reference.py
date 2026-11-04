from types import SimpleNamespace as NS
import numpy as np
from Basilisk.architecture import messaging
from simulation.components.orbit_attitude_reference import OrbitAttitudeReferenceComponent

def test_attitude_only_component_needs_no_target_or_wrench_ports():
    state=messaging.SCStatesMsg().write(messaging.SCStatesMsgPayload(r_BN_N=[6878136.6,0,0],v_BN_N=[0,7612,0]))
    kept=[];refs=[]
    control=NS(initial_inertia=np.eye(3),tracking_error=NS(attRefInMsg=NS(subscribeTo=refs.append)))
    def read(name):
        assert name=='cubesat_bus'
        return state
    ctx=NS(extra={'attitude_control':control},ports=NS(body_state=read),config=NS(publish_stride=8),add=lambda *a,**k:None,keep_alive=kept.append)
    OrbitAttitudeReferenceComponent().install(ctx);kept[0].UpdateState(0)
    assert len(refs)==1
    assert np.all(np.isfinite(refs[0].read().sigma_RN))
    assert np.isclose(refs[0].read().omega_RN_N[2],7612/6878136.6)
