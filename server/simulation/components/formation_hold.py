"""Opt-in orbit-frame station keeping through additive force/torque ports."""
import json
import numpy as np
import xml.etree.ElementTree as ET
from Basilisk.architecture import sysModel, messaging
from Basilisk.utilities import RigidBodyKinematics as rbk
from simulation.assembly import Slot
from simulation.formation_hold import reference, bounded_force, limit_norm

class FormationGuidance(sysModel.SysModel):
    def __init__(self, ctx, config, model):
        super().__init__(); self.ModelTag='dualFormationGuidance'
        self.config=config; self.attitude=ctx.extra['attitude_control']
        origin,_=ctx.ports.orbital_providers()
        self.origin=origin.scStateOutMsg.addSubscriber()
        self.readers=[ctx.ports.body_state(n).addSubscriber() for n in ('cubesat_bus','capture_target')]
        self.forces=[messaging.CmdForceInertialMsg() for _ in self.readers]
        self.targetTorque=messaging.CmdTorqueBodyMsg(); self.attRef=messaging.AttRefMsg()
        self.attitude.tracking_error.attRefInMsg.subscribeTo(self.attRef)
        root=ET.parse(model).getroot()
        self.masses=[sum(float(b.find('inertial').get('mass')) for b in root.find(f'./worldbody/body[@name="{name}"]').iter('body') if b.find('inertial') is not None) for name in ('cubesat_bus','capture_target')]
        self.last_log=-1; self.state={}

    def UpdateState(self, ns):
        o=self.origin(); r=np.array(o.r_BN_N); v=np.array(o.v_BN_N)
        if np.linalg.norm(r)<1: return
        # Orbit updates before this module; body messages describe the previous
        # physics publication. Compare at that SAME timestamp (7.6 km/s orbit).
        age=(self.origin.timeWritten()-self.readers[0].timeWritten())*1e-9
        if abs(age)>.1: raise RuntimeError('formation state timestamps differ by more than 100 ms')
        n2=np.linalg.norm(np.cross(r,v))**2/np.linalg.norm(r)**4
        acceleration=-n2*r
        r=r-v*age+.5*acceleration*age**2
        v=v-acceleration*age
        ref=reference(r,v,self.config['separation_m']); omega=ref['omega']
        states=[reader() for reader in self.readers]
        errors=[]; forces=[]
        for i,(state,pd,vd) in enumerate(zip(states,[r,ref['target_position']],[v,ref['target_velocity']])):
            e=pd-np.array(state.r_BN_N); ev=vd-np.array(state.v_BN_N)
            offset=pd-r; er=r/np.linalg.norm(r); n2=np.dot(omega,omega)
            # Desired circular-offset acceleration minus linearized Earth gradient.
            ff=np.cross(omega,np.cross(omega,offset))-n2*(3*er*np.dot(er,offset)-offset)
            f=bounded_force(e,ev,self.masses[i],self.config['force_limit_n'],ff)
            self.forces[i].write(messaging.CmdForceInertialMsgPayload(forceRequestInertial=f.tolist()),ns,self.moduleID)
            errors.append(e.tolist()); forces.append(f.tolist())
        rs=ref['service_rotation']; rt=ref['target_rotation']
        self.attRef.write(messaging.AttRefMsgPayload(sigma_RN=rbk.C2MRP(rs.T).tolist(),omega_RN_N=omega.tolist(),domega_RN_N=[0.,0.,0.]),ns,self.moduleID)
        # InitialReference still owns startup inertia setup; update only after it ran.
        if self.attitude.initial_inertia is not None: self.attitude.reference_mrp=np.asarray(rbk.C2MRP(rs.T))
        s=states[1]; actual=np.asarray(rbk.MRP2C(s.sigma_BN)).T
        mrp=np.asarray(rbk.C2MRP((actual.T@rt).T))
        norm=np.linalg.norm(mrp)
        error=4*np.arctan(norm)*mrp/norm if norm>1e-12 else np.zeros(3)
        torque=limit_norm(.05*np.asarray(error)+.3*(actual.T@omega-np.array(s.omega_BN_B)),self.config['torque_limit_nm'])
        self.targetTorque.write(messaging.CmdTorqueBodyMsgPayload(torqueRequestBody=torque.tolist()),ns,self.moduleID)
        relative=np.array(states[1].r_BN_N)-np.array(states[0].r_BN_N)
        relvel=np.array(states[1].v_BN_N)-np.array(states[0].v_BN_N)-np.cross(omega,relative)
        self.state=dict(enabled=True,actuation='bounded_ideal_force_target_torque_existing_service_wheels',
            position_error_m=errors,force_n=forces,target_torque_nm=torque.tolist(),mass_kg=self.masses,
            relative_position_lvlh_m=rs.T@relative,relative_velocity_rotating_m_s=relvel.tolist(),
            target_attitude_error_deg=float(np.linalg.norm(error)*180/np.pi))
        self.state['relative_position_lvlh_m']=self.state['relative_position_lvlh_m'].tolist()
        if int(ns//10_000_000_000)>self.last_log:
            self.last_log=int(ns//10_000_000_000)
            print(json.dumps(dict(type='formation_hold',sim_time_s=ns*1e-9,**self.state)),flush=True)

class FormationHoldComponent:
    name='formation_hold'
    def __init__(self, config, model): self.config=config; self.model=model; self.guidance=None
    def install(self,ctx):
        self.guidance=g=FormationGuidance(ctx,self.config,self.model)
        for i,name in enumerate(('cubesat_bus','capture_target')):
            ctx.ports.add_body_wrench(name,source=f'{self.name}_{name}',force_inertial=g.forces[i],torque_body=g.targetTorque if i==1 else None)
        ctx.add(g,Slot.ENVIRONMENT,every=ctx.config.publish_stride)
        ctx.keep_alive(g)
    def telemetry(self): return self.guidance.state
