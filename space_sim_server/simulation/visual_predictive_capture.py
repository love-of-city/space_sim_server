"""Opt-in image/encoder capture trial. No target-state input.

The observer fits relative motion in the service-base frame. It does not claim
to observe base inertial motion: unmodelled recoil is an independently scored
prediction error. Frame-time encoder registration avoids freezing eye-in-hand
coordinates when an image is delayed or markers leave the field of view.
"""
from collections import deque
import json
import time
import numpy as np
from simulation.capture_sequence import VisualCaptureProbe
from simulation.serial_chain_kinematics import rotation_matrix_to_vector, axis_angle_to_matrix


class VisualPredictiveCapture(VisualCaptureProbe):
    def __init__(self,path,session_id,*,wall_now_ns=time.time_ns,held_pose=False):
        super().__init__(path,session_id,wall_now_ns=wall_now_ns)
        self.held_pose=bool(held_pose)
        self.encoders=deque(maxlen=4000);self.observations=deque(maxlen=80)
        self.last_frame=None;self.source_t=None;self.fit_t=None
        self.p_base=None;self.r_base=None;self.v_base=np.zeros(3);self.w_base=np.zeros(3)
        self.predicted_position_grip=None;self.predicted_rotation_grip=None
        self.blind_since=None;self.blind_path_m=0.;self.prediction_mode='waiting'
        self.last_contact=None;self.hold_since=None

    @staticmethod
    def neutral():
        return dict(deadman=False,active_arm='capture',
            end_effector_linear_velocity_body_m_s=[0.,0.,0.],
            end_effector_angular_velocity_body_rad_s=[0.,0.,0.],
            capture_radial_velocity_m_s=0.,capture_axial_velocity_m_s=0.)

    def abort(self,reason):
        self.sequence.phase='abort';self.sequence.failure_reason=reason
        return self.neutral(),'predictive_abort_'+reason

    @staticmethod
    def _force_failure(feedback):
        if any(not np.isfinite(float(v)) for k,v in feedback.items() if k.endswith(('_n','_m'))):
            return 'invalid_force_feedback'
        if feedback.get('peak_target_contact_force_n',0)>10 or feedback.get('jaw_force_n',0)>20 or feedback.get('pad_compression_m',0)>.002:
            return 'contact_overload'
        return None

    def gyro_unavailable(self,reason,t,q,feedback):
        """No approach without rate feedback; never release a held target."""
        failure=self._force_failure(feedback or {})
        if failure:return self.abort(failure)
        if self.sequence.phase in ('tighten','hold_force'):
            command,phase=self.sequence.command(t,q,np.zeros(3),np.eye(3),feedback)
            return command,'predictive_'+phase+'_gyro_unavailable'
        if self.sequence.phase=='wait' and reason=='missing_service_gyro':
            return self.neutral(),'predictive_waiting_service_gyro'
        return self.abort('service_gyro_'+reason)

    def _observe(self,t,position,rotation):
        self.encoders.append((float(t),position.copy(),rotation.copy()))
        try:
            p,r=self._pose(t)
        except (OSError,ValueError,TypeError,KeyError) as error:
            self.input_status=str(error)
            if self.input_status=='session_mismatch' or (self.last_packet or {}).get('session_id') not in (None,'',self.session_id):
                if self.source_t is not None:raise ValueError('scene_changed')
                return False  # A new scene waits for its first own-session image.
            return False
        frame=str(self.last_packet['source_frame_id'])
        if frame==self.last_frame:return t-self.source_t<.4
        if not self._complete_observation():
            self.input_status='incomplete_marker_set';return False
        source_t=int(self.last_packet['sim_time_ns'])/1e9
        if self.source_t is not None and source_t<=self.source_t:return False
        source=min(self.encoders,key=lambda h:abs(h[0]-source_t))
        if abs(source[0]-source_t)>.012:
            self.input_status='no_source_encoder_sample';return False
        p_base=source[1]+source[2]@p;r_base=source[2]@r
        if self.p_base is not None:
            predicted=self.p_base+self.v_base*(source_t-self.fit_t)
            if np.linalg.norm(p_base-predicted)>.008:
                self.input_status='visual_jump_rejected';return False
        self.last_frame=frame;self.source_t=source_t
        self.observations.append((source_t,p_base.copy(),r_base.copy()))
        samples=[x for x in self.observations if source_t-x[0]<=2.]
        self.fit_t=source_t;self.p_base=p_base;self.r_base=r_base
        if self.held_pose:
            stable=samples[-5:]
            self.p_base=np.mean([x[1] for x in stable],axis=0)
            u,_,vh=np.linalg.svd(np.mean([x[2] for x in stable],axis=0))
            self.r_base=u@np.diag([1.,1.,np.linalg.det(u@vh)])@vh
            self.v_base.fill(0);self.w_base.fill(0)
        elif len(samples)>=6 and source_t-samples[0][0]>=.5:
            times=np.array([x[0]-source_t for x in samples]);design=np.c_[times,np.ones(len(times))]
            fit=np.linalg.lstsq(design,np.array([x[1] for x in samples]),rcond=None)[0]
            self.v_base=fit[0];self.p_base=fit[1]
            angles=np.array([rotation_matrix_to_vector(x[2]@r_base.T) for x in samples])
            self.w_base=np.linalg.lstsq(design,angles,rcond=None)[0][0]
            for vector,limit in ((self.v_base,.003),(self.w_base,.01)):
                vector*=min(1.,limit/max(np.linalg.norm(vector),1e-12))
        return True

    def _complete_observation(self):
        """Legacy wrist frontend requires its original four-marker layout."""
        return not self.held_pose or self.last_packet.get('marker_count',4)>=4

    def command(self,t,q,base_from_grip,feedback,*,reference_from_base=None):
        if not self.held_pose:return self._command_in_frame(t,q,base_from_grip,feedback)
        if reference_from_base is None:return self.abort('missing_own_motion')
        pb,rb=(np.asarray(x,float) for x in reference_from_base)
        pg,rg=(np.asarray(x,float) for x in base_from_grip)
        if pb.shape!=(3,) or rb.shape!=(3,3) or not np.all(np.isfinite(np.r_[pb,rb.ravel()])):
            return self.abort('invalid_own_motion')
        a,status=self._command_in_frame(t,q,(pb+rb@pg,rb@rg),feedback)
        for key in ('end_effector_linear_velocity_body_m_s','end_effector_angular_velocity_body_rad_s'):
            a[key]=(rb.T@np.asarray(a[key])).tolist()
        return a,status

    def _command_in_frame(self,t,q,base_from_grip,feedback):
        dt=0. if self.last_time is None else float(np.clip(t-self.last_time,0.,.02));self.last_time=t
        position,rotation=(np.asarray(x,float) for x in base_from_grip)
        if position.shape!=(3,) or rotation.shape!=(3,3) or not np.all(np.isfinite(position)) or not np.all(np.isfinite(rotation)):
            return self.abort('invalid_encoder_fk')
        if self.sequence.phase=='abort':return self.neutral(),'predictive_abort'
        try:visible=self._observe(t,position,rotation)
        except ValueError:return self.abort('scene_changed')
        f=feedback or {}
        failure=self._force_failure(f)
        if failure:return self.abort(failure)
        phase=self.sequence.phase
        if phase in ('tighten','hold_force'):
            a,phase=self.sequence.command(t,q,np.zeros(3),np.eye(3),f)
            return a,'predictive_'+phase
        if phase in ('close','contact_seek'):
            a=self.neutral();a['deadman']=True
            if f.get('peak_target_contact_force_n',0)>.05:self.last_contact=t;self.sequence.phase='close'
            if np.max(np.abs(q[8:]+.012))<.0002:
                self.sequence.phase,self.sequence.start='tighten',t
            elif t-self.sequence.start>35:return self.abort('closure_timeout')
            elif f.get('jaw_force_n',0)>8 and f.get('jaw_support_count',0)>=2:
                self.sequence.phase,self.sequence.start='tighten',t
            else:
                a['capture_radial_velocity_m_s']=-.0005 if f.get('peak_target_contact_force_n',0)>.1 else -.001
            return a,'predictive_'+self.sequence.phase
        if phase in ('open','extend'):
            a,phase=self.sequence.command(t,q,np.zeros(3),np.eye(3),f)
            return a,'predictive_'+phase
        if self.p_base is None:return self.neutral(),'predictive_waiting_vision'
        if phase=='wait':
            if t<2 or len(self.observations)<6:return self.neutral(),'predictive_waiting_stable_vision'
            self.sequence.phase,self.sequence.start='open',t
            return self.neutral(),'predictive_open'
        if visible:
            self.blind_since=None;self.blind_path_m=0.;self.prediction_mode='visual'
        else:
            if self.blind_since is None:self.blind_since=t
            self.prediction_mode='encoder_prediction'
            if t-self.source_t>90 or self.blind_path_m>.18:return self.abort('prediction_budget')
            if len(self.observations)<6:return self.abort('insufficient_visual_history')
        age=t-self.fit_t
        p_base=self.p_base+self.v_base*age
        speed=np.linalg.norm(self.w_base)
        r_base=axis_angle_to_matrix(self.w_base,speed*age)@self.r_base if speed>1e-12 else self.r_base
        p=rotation.T@(p_base-position);r=rotation.T@r_base
        self.predicted_position_grip=p;self.predicted_rotation_grip=r
        gap=-float(p[2]);angular=rotation_matrix_to_vector(r)
        error=p-np.array([0.,0.,-.006])
        if self.held_pose and f.get('peak_target_contact_force_n',0)>.3 and gap>.008 and f.get('pad_force_n',0)<.1:
            return self.abort('jaw_contact_before_seated')
        if f.get('peak_target_contact_force_n',0)>.5 and gap>.02:
            return self.abort('contact_before_insertion_zone')
        if f.get('pad_force_n',0)>.2 and gap<.015:
            self.sequence.phase,self.sequence.start='close',t;self.last_contact=t
            return dict(self.neutral(),deadman=True),'predictive_close'
        if np.linalg.norm(error)<.0005 and np.linalg.norm(angular)<.0087:
            self.sequence.phase,self.sequence.start='contact_seek',t
            return dict(self.neutral(),deadman=True),'predictive_contact_seek'
        if t-self.sequence.start>240:return self.abort('approach_timeout')
        # Base-frame target velocity is estimated from images, never body truth.
        linear=rotation@(.8*error)+self.v_base
        limit=.001 if gap<.04 else .004
        linear*=min(1.,limit/max(np.linalg.norm(linear),1e-12))
        omega=rotation@(.4*angular)+self.w_base
        omega*=min(1.,.01/max(np.linalg.norm(omega),1e-12))
        self.commanded_path_m+=float(np.linalg.norm(linear))*dt
        if not visible:self.blind_path_m+=float(np.linalg.norm(linear))*dt
        if self.commanded_path_m>.75:return self.abort('total_motion_budget')
        self.motion_samples+=1
        return dict(self.neutral(),deadman=True,end_effector_linear_velocity_body_m_s=linear.tolist(),
            end_effector_angular_velocity_body_rad_s=omega.tolist()),'predictive_'+('approach_visual' if visible else 'approach_encoder')

    def diagnostics(self):
        return dict(mode=self.prediction_mode,held_target_pose=self.held_pose,
            reference_frame='initial_coorbit_own_odometry' if self.held_pose else 'service_base',source_sim_time_s=self.source_t,
            blind_path_mm=self.blind_path_m*1000,
            predicted_position_grip_m=self.predicted_position_grip.tolist() if self.predicted_position_grip is not None else None,
            predicted_rotation_grip=self.predicted_rotation_grip.tolist() if self.predicted_rotation_grip is not None else None,
            image_velocity_base_m_s=self.v_base.tolist(),image_angular_velocity_base_rad_s=self.w_base.tolist())
