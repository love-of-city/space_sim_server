"""Service attitude guidance only: no position target, force or target torque."""
import numpy as np
from Basilisk.architecture import sysModel, messaging
from Basilisk.utilities import RigidBodyKinematics as rbk
from simulation.assembly import Slot
from simulation.formation_hold import service_body_rotation

class OrbitAttitudeReference(sysModel.SysModel):
    def __init__(self, ctx, service_body_rotation_wxyz=None):
        super().__init__(); self.ModelTag='serviceOrbitAttitudeReference'
        self.control=ctx.extra['attitude_control']
        self.body_rotation=service_body_rotation(service_body_rotation_wxyz)
        self.reader=ctx.ports.body_state('cubesat_bus').addSubscriber()
        self.output=messaging.AttRefMsg()
        self.control.tracking_error.attRefInMsg.subscribeTo(self.output)
    def UpdateState(self,ns):
        state=self.reader(); r=np.asarray(state.r_BN_N);v=np.asarray(state.v_BN_N)
        radius=np.linalg.norm(r);h=np.cross(r,v);hn=np.linalg.norm(h)
        if radius<1 or hn<1: return
        er=r/radius;en=h/hn;et=np.cross(en,er)
        rotation=np.column_stack((er,en,-et))@self.body_rotation;sigma=rbk.C2MRP(rotation.T)
        self.output.write(messaging.AttRefMsgPayload(sigma_RN=sigma.tolist(),omega_RN_N=(h/radius**2).tolist(),domega_RN_N=[0.,0.,0.]),ns,self.moduleID)
        if self.control.initial_inertia is not None:self.control.reference_mrp=sigma

class OrbitAttitudeReferenceComponent:
    name='service_orbit_attitude_reference'
    def __init__(self, service_body_rotation_wxyz=None):
        self.body_rotation_wxyz=service_body_rotation_wxyz
    def install(self,ctx):
        model=OrbitAttitudeReference(ctx, self.body_rotation_wxyz)
        ctx.add(model,Slot.ENVIRONMENT,every=ctx.config.publish_stride)
        ctx.keep_alive(model)
