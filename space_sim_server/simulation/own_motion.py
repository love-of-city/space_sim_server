"""Encoder mass-model translation and incremental gyro attitude feedback.

Only known, unchanged service-assembly mass properties and own joint/gyro
measurements enter this observer. No body position/orientation or target state
is an input. Before contact the service COM is assumed fixed in the common
co-orbit frame. This is an experimental odometry model, not a full navigator.
"""
import xml.etree.ElementTree as ET
from copy import deepcopy
import numpy as np
from simulation.serial_chain_kinematics import quaternion_wxyz_to_matrix,axis_angle_to_matrix


class EncoderMassModel:
    def __init__(self,path,mass_reader):
        root=ET.parse(path).find("./worldbody/body[@name='cubesat_bus']")
        if root is None:raise ValueError('missing service assembly')
        self.nodes=[];self.joints=[]
        def vector(value,default):return np.array([float(v) for v in value.split()]) if value else np.array(default,float)
        def visit(body,parent):
            name=body.get('name');properties=np.asarray(mass_reader(name),float)
            joints=[]
            for j in body.findall('joint'):
                kind=j.get('type','hinge')
                if kind=='free':continue
                if kind not in ('slide','hinge'):raise ValueError('unsupported own encoder joint')
                joint=(j.get('name'),kind,vector(j.get('axis'),[0,0,1]),vector(j.get('pos'),[0,0,0]),float(j.get('ref','0')))
                joints.append(joint);self.joints.append((name,j.get('name')))
            index=len(self.nodes)
            self.nodes.append((parent,vector(body.get('pos') if parent is not None else None,[0,0,0]),
                quaternion_wxyz_to_matrix(vector(body.get('quat') if parent is not None else None,[1,0,0,0])),
                properties[0],properties[1:4].copy(),joints))
            for child in body.findall('body'):
                if child.find('freejoint') is None and child.find('joint[@type="free"]') is None:visit(child,index)
        visit(root,None)
        self.mass=sum(n[3] for n in self.nodes)
        if self.mass<=0:raise ValueError('missing service mass calibration')

    def com(self,encoder_positions):
        poses=[];weighted=np.zeros(3)
        for parent,offset,fixed_r,mass,local_com,joints in self.nodes:
            if parent is None:p=offset.copy();r=fixed_r.copy()
            else:
                pp,pr=poses[parent];p=pp+pr@offset;r=pr@fixed_r
            for name,kind,axis,pivot,reference in joints:
                value=float(encoder_positions[name])-reference
                if kind=='slide':p=p+r@(axis*value)
                else:
                    turn=axis_angle_to_matrix(axis,value)
                    p=p+r@(pivot-turn@pivot);r=r@turn
            poses.append((p,r));weighted+=mass*(p+r@local_com)
        return weighted/self.mass


class OwnMotionFeedback:
    """Causal 50 Hz measured-rate integration, queried on the 400 Hz control grid.

    The first 1.5 seconds estimate common orbit spin plus constant sensor bias;
    this is not an independently observable absolute bias calibration. Repeated
    reads neither add samples nor change the anchored rotation. Only prediction
    to the requested control time uses a held measurement between sensor ticks.
    """
    PERIOD_NS=20_000_000
    CALIBRATION_NS=1_500_000_000

    def __init__(self, *, max_age_ns=40_000_000):
        self.max_age_ns=int(max_age_ns)
        self.reset()

    def reset(self):
        self.epoch=None;self.last_control_ns=None;self.last_sample_ns=None
        self.initial_com=None;self.rotation=np.eye(3);self.reference_spin=None
        self.spin_samples=[];self.unique_samples=0;self.last_reading=None;self.last_rate=None

    @staticmethod
    def _advance(rotation, rate_body, reference_spin, seconds):
        relative=rate_body-rotation.T@reference_spin
        speed=float(np.linalg.norm(relative))
        return rotation@axis_angle_to_matrix(relative,speed*seconds) if speed>1e-12 else rotation.copy()

    def update(self, control_time_ns, reading, com_body, *, epoch):
        from space_arm_platform.models import GyroObservation
        if isinstance(control_time_ns,bool) or not isinstance(control_time_ns,int) or control_time_ns<0:
            raise ValueError('invalid_control_time')
        now=control_time_ns
        identity=tuple(epoch)
        if len(identity)!=2 or not identity[0]:raise ValueError('invalid_gyro_epoch')
        if self.epoch is not None and identity!=self.epoch:raise ValueError('gyro_epoch_changed')
        if self.last_control_ns is not None and now<self.last_control_ns:raise ValueError('control_time_reversed')
        com=np.asarray(com_body,float)
        if com.shape!=(3,) or not np.isfinite(com).all():raise ValueError('invalid_encoder_com')
        if reading is None:raise ValueError('missing_service_gyro')
        if not isinstance(reading,dict):raise ValueError('invalid_service_gyro_packet')
        if reading.get('frame')!='spacecraft_body' or reading.get('reference_frame')!='J2000' or reading.get('body_name')!='cubesat_bus':
            raise ValueError('invalid_gyro_frame')
        try:
            checked=GyroObservation.model_validate(reading)
        except ValueError as error:
            raise ValueError('invalid_service_gyro_packet') from error
        if not checked.valid or any(checked.saturated_axes):raise ValueError('invalid_service_gyro_measurement')
        stamp=int(checked.sample_time_ns)
        if stamp>now:raise ValueError('future_service_gyro')
        if now-stamp>self.max_age_ns:raise ValueError('stale_service_gyro')
        if self.last_sample_ns is not None and stamp<self.last_sample_ns:raise ValueError('gyro_time_reversed')
        fresh=self.last_sample_ns is None or stamp!=self.last_sample_ns
        if fresh:
            if self.last_sample_ns is None:
                if stamp!=0:raise ValueError('gyro_stream_gap_at_start')
            elif stamp-self.last_sample_ns!=self.PERIOD_NS:
                raise ValueError('gyro_stream_gap')
        elif reading!=self.last_reading:
            raise ValueError('duplicate_gyro_sample_changed')
        # Commit only after validation; a malformed packet never advances state.
        self.epoch=identity
        if self.initial_com is None:self.initial_com=com.copy()
        if fresh:
            if self.last_sample_ns is not None and stamp>self.CALIBRATION_NS:
                duration=(stamp-max(self.last_sample_ns,self.CALIBRATION_NS))*1e-9
                self.rotation=self._advance(self.rotation,self.last_rate,self.reference_spin,duration)
            rate=np.asarray(checked.angular_velocity_body_rad_s,float)
            if stamp<=self.CALIBRATION_NS:
                self.spin_samples.append(rate.copy())
                self.reference_spin=np.mean(self.spin_samples,axis=0)
            self.last_sample_ns=stamp;self.last_rate=rate;self.last_reading=deepcopy(reading)
            self.unique_samples+=1
        rotation=self.rotation.copy()
        if now>self.CALIBRATION_NS:
            duration=(now-max(stamp,self.CALIBRATION_NS))*1e-9
            rotation=self._advance(rotation,self.last_rate,self.reference_spin,duration)
        self.last_control_ns=now
        return self.initial_com-rotation@com,rotation

    def diagnostics(self):
        return dict(source='service_gyro',sample_time_ns=str(self.last_sample_ns) if self.last_sample_ns is not None else None,
                    sample_age_ns=(self.last_control_ns-self.last_sample_ns) if self.last_control_ns is not None else None,
                    unique_samples=self.unique_samples,calibration_samples=len(self.spin_samples),
                    reference_spin_estimate_rad_s=self.reference_spin.tolist() if self.reference_spin is not None else None,
                    epoch=list(self.epoch) if self.epoch is not None else None,
                    reading=deepcopy(self.last_reading))

    def snapshot(self):
        return dict(schema='space-sim-own-motion/1', max_age_ns=self.max_age_ns,
            epoch=list(self.epoch) if self.epoch else None, last_control_ns=self.last_control_ns,
            last_sample_ns=self.last_sample_ns, initial_com=self.initial_com.tolist() if self.initial_com is not None else None,
            rotation=self.rotation.tolist(), reference_spin=self.reference_spin.tolist() if self.reference_spin is not None else None,
            spin_samples=[s.tolist() for s in self.spin_samples], unique_samples=self.unique_samples,
            last_reading=deepcopy(self.last_reading), last_rate=self.last_rate.tolist() if self.last_rate is not None else None)

    def restore(self, document):
        from space_arm_platform.stereo_capture_packet import rotation
        d=deepcopy(document)
        if d.get('schema')!='space-sim-own-motion/1' or d.get('max_age_ns')!=self.max_age_ns:
            raise ValueError('own-motion configuration mismatch')
        r=rotation(d['rotation'])
        samples=np.asarray(d['spin_samples'],float)
        stamp=d['last_sample_ns']; now=d['last_control_ns']; count=d['unique_samples']
        vectors=[np.asarray(d[k],float) for k in ('initial_com','reference_spin','last_rate')]
        if (type(stamp) is not int or stamp<self.CALIBRATION_NS or stamp%self.PERIOD_NS
                or type(now) is not int or not stamp<=now<=stamp+self.max_age_ns
                or type(count) is not int or count!=stamp//self.PERIOD_NS+1
                or samples.shape!=(self.CALIBRATION_NS//self.PERIOD_NS+1,3)
                or any(v.shape!=(3,) for v in vectors) or not np.isfinite(np.r_[samples.ravel(),*vectors]).all()
                or not np.allclose(vectors[1],samples.mean(axis=0),atol=1e-14,rtol=0)
                or not isinstance(d['epoch'],list) or len(d['epoch'])!=2 or not d['epoch'][0]
                or int(d['last_reading']['sample_time_ns'])!=stamp
                or not np.array_equal(d['last_reading']['angular_velocity_body_rad_s'],vectors[2])):
            raise ValueError('invalid own-motion history')
        self.epoch=tuple(d['epoch']); self.last_control_ns=now; self.last_sample_ns=stamp
        self.initial_com,self.reference_spin,self.last_rate=vectors
        self.rotation=r.copy(); self.spin_samples=list(samples); self.unique_samples=count
        self.last_reading=d['last_reading']
