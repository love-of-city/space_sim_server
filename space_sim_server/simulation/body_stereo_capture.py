"""Encoder/contact/image-only capture servo. No target dynamics state input."""
from pathlib import Path
import json
import numpy as np
from simulation.capture_sequence import CaptureSequence
from simulation.serial_chain_kinematics import rotation_matrix_to_vector


class BodyStereoCaptureServo:
    def __init__(self, path, session_id, setup, *, maximum_image_age_s=.25):
        self.path=Path(path);self.session_id=session_id
        self.mount_parent=setup.get('mount_parent_id','teleop/cubesat_bus')
        if self.mount_parent=='teleop/gripper':
            mount=setup['parent_from_optical']['left']
            self.body_from_left=np.asarray(mount['R_parent_from_optical'],float)
            self.left_position_body=np.asarray(mount['position_parent_m'],float)
        elif self.mount_parent=='teleop/cubesat_bus':
            mount=setup['body_from_optical']['left']
            self.body_from_left=np.asarray(mount['R_body_from_optical'],float)
            self.left_position_body=np.asarray(mount['position_body_m'],float)
        else:raise ValueError('unsupported stereo mount parent')
        self.center=np.array([.1755,.05475,.0420787001299893])
        self.target_from_interface=np.array([[1.,0,0],[0,0,1],[0,-1,0]])
        self.maximum_age=float(maximum_image_age_s)
        self.sequence=CaptureSequence(force_control=True)
        self.last_packet=None;self.input_status='waiting';self.commanded_path_m=0.;self.motion_samples=0
        self.last_time=None;self.last_source=-1;self.estimated_gap_m=None
        self.force_hold_established=False

    def _pose(self,t,grip_position_body,body_from_grip):
        packet=json.loads(self.path.read_text(encoding='utf-8'))
        self.last_packet=packet
        if packet.get('session_id')!=self.session_id:raise ValueError('session_mismatch')
        if packet.get('camera_ids')!=['teleop/camera/capture_stereo_left','teleop/camera/capture_stereo_right']:
            raise ValueError('wrong_camera_pair')
        source=int(packet['source_frame_id']);source_t=int(packet['sim_time_ns'])/1e9
        if source<self.last_source:raise ValueError('out_of_order_frame')
        if not np.isfinite(source_t) or source_t>t+.025 or t-source_t>self.maximum_age:
            raise ValueError('stale_or_future_frame')
        result=packet['measurement']
        if result.get('status')!='ok' or result.get('route')!='joint_multicamera':raise ValueError('stereo_unavailable')
        pose=result['pose_target_to_left'];r=np.asarray(pose['R'],float);v=np.asarray(pose['t_m'],float)
        if r.shape!=(3,3) or v.shape!=(3,) or not np.isfinite(r).all() or not np.isfinite(v).all():raise ValueError('invalid_pose')
        if not np.allclose(r.T@r,np.eye(3),atol=1e-5) or abs(np.linalg.det(r)-1)>1e-5:raise ValueError('invalid_rotation')
        center_body=self.body_from_left@(r@self.center+v)+self.left_position_body
        interface_body=self.body_from_left@r@self.target_from_interface
        if self.mount_parent=='teleop/gripper':
            # Fixed hand-eye extrinsics already express the image measurement
            # in the gripper frame. Do not apply the arm's FK a second time.
            p=center_body
            orientation=interface_body
        else:
            p=body_from_grip.T@(center_body-grip_position_body)
            orientation=body_from_grip.T@interface_body
        self.last_source=source;self.input_status='tracking';self.estimated_gap_m=float(-p[2])
        return p,orientation

    def command(self,t,q,body_from_grip,feedback,*,grip_position_body_m):
        dt=0. if self.last_time is None else float(np.clip(t-self.last_time,0,.02));self.last_time=t
        neutral=dict(deadman=False,active_arm='capture',end_effector_linear_velocity_body_m_s=[0.,0.,0.],
            end_effector_angular_velocity_body_rad_s=[0.,0.,0.],capture_radial_velocity_m_s=0.,capture_axial_velocity_m_s=0.)
        if self.sequence.phase=='abort':return neutral,'abort'
        force_only=self.force_hold_established and self.sequence.phase in ('tighten','hold_force')
        try:p,r=self._pose(t,np.asarray(grip_position_body_m),np.asarray(body_from_grip))
        except (OSError,ValueError,KeyError,TypeError) as error:
            self.input_status=str(error)
            if not force_only:return neutral,'stereo_waiting_'+self.input_status
        if force_only:
            # Once image-guided capture has established force hold, a rejected
            # photograph must not reset the loaded joint references. Keep the
            # arm stationary and regulate the existing grip with CURRENT force
            # feedback. No cached target pose or scene truth drives this branch.
            try:
                values=[float(feedback[k]) for k in ('pad_force_n','pad_compression_m')]
                values.append(float(feedback.get('jaw_force_n',0.)))
                if not np.isfinite(values).all():raise ValueError('nonfinite force')
            except (KeyError,TypeError,ValueError):
                self.sequence.phase='abort';self.sequence.failure_reason='invalid_force_feedback'
                return neutral,'abort'
            command,phase=self.sequence.command(t,q,np.zeros(3),np.eye(3),feedback)
            command['end_effector_linear_velocity_body_m_s']=[0.,0.,0.]
            command['end_effector_angular_velocity_body_rad_s']=[0.,0.,0.]
            self.motion_samples+=int(command['deadman'])
            return (neutral if phase=='abort' else command),phase
        if self.sequence.phase=='wait':
            if t<2:return neutral,'stereo_waiting_start'
            self.sequence.phase,self.sequence.start='open',t
        # The only target-relative geometry here comes from image pose and FK.
        offset=r.T@(-p)
        command,phase=self.sequence.command(t,q,offset,r,feedback)
        if phase=='hold_force':self.force_hold_established=True
        command['end_effector_linear_velocity_body_m_s']=(body_from_grip@r@command['end_effector_linear_velocity_body_m_s']).tolist()
        if phase in ('approach','close'):
            command['end_effector_angular_velocity_body_rad_s']=(body_from_grip@np.clip(.8*rotation_matrix_to_vector(r),-.04,.04)).tolist()
        else:command['end_effector_angular_velocity_body_rad_s']=[0.,0.,0.]
        self.commanded_path_m+=float(np.linalg.norm(command['end_effector_linear_velocity_body_m_s']))*dt
        self.motion_samples+=int(command['deadman'])
        if phase=='abort':command=neutral
        return command,phase
