"""Formal near23 route: estimated COM guidance and the existing capture servos.

No physical coordinates or target truth are inputs to the guidance/predictor.
The contact audit reads dynamics only for force feedback and safety.
"""
import json
from pathlib import Path

import numpy as np

from simulation.capture26_mission import MODEL_PATH
from simulation.capture26_preparation import JointPreparation
from simulation.end_stereo_pointing import EndStereoGeometry, PrealignmentController
from simulation.near_field.orbital_guidance import vbar_frame, rotating_relative_state
from simulation.pose_navigation import skew
from simulation.vbar_stereo_sequence import VbarStereoSequence


def preparation_plan(root):
    return JointPreparation(json.loads((Path(root)/'config/missions/capture26-near23-preparation-v1.json').read_text('utf-8')),
                            Path(root)/MODEL_PATH)


def verify_handoff(source, output, route, capture):
    """Block motion when the native receiver differs from its accepted sender."""
    from tools.check_capture26_continuity import check
    audit = check(source, output)
    if not audit['continuity_passed']:
        route._hold('far_handoff_continuity_failed')
        capture.failure = 'far_handoff_continuity_failed'
    return audit


class Near23Route:
    def __init__(self, root, vision, *, epoch_ns):
        self.geometry = EndStereoGeometry(Path(root)/MODEL_PATH, vision)
        self.preparation = preparation_plan(root)
        self.q_prepared = self.preparation.points[-1, 6:]
        _, rc = self.geometry.cameras_service(self.q_prepared)['left']
        self.service_mount = np.array([[0.,0.,1.],[-1.,0.,0.],[0.,-1.,0.]])@rc.T
        self.target_mount = np.array([[0.,-1.,0.],[1.,0.,0.],[0.,0.,1.]])
        self.alignment = PrealignmentController(self.geometry)
        self.sequence = None
        self.epoch_ns = int(epoch_ns)
        self.visual_mount = None
        self.safe_hold = False
        self.depth = 110.
        self.last_state = None
        self.target_torque = np.zeros(3)
        self.last_reference = None
        self.audit = []
        self.capture_feedback = None

    def reference_vision(self):
        """Nominal final FK for reference planning; images use exposure FK."""
        return dict(body_from_optical={eye:dict(position_body_m=p.tolist(),R_body_from_optical=r.tolist())
                    for eye,(p,r) in self.geometry.cameras_service(self.q_prepared).items()})

    def attitude_reference(self, state):
        frame, omega = vbar_frame(np.r_[state['p'][1],state['v'][1]])
        return frame@self.service_mount, omega, np.zeros(3)

    def _hold(self, reason):
        self.safe_hold = True
        if self.sequence is not None and self.last_state is not None:
            self.sequence.vbar.fail(reason,self.last_state)

    def step(self, now_ns, state, *, service_com, target_com, q, qdot, covariance,
             sigma, last_gnss_ns, last_cooperative_ns, healthy, visual_ready,
             visual_stamp_ns, mass, grasp_complete=False, grasp_failure=None):
        from simulation.cooperative_pointing import bounded_pointing_torque
        now = now_ns*1e-9
        target = np.r_[state['p'][1],state['v'][1]]
        frame, omega = vbar_frame(target)
        relative = np.r_[state['p'][1]-state['p'][0],state['v'][1]-state['v'][0]]
        x = rotating_relative_state(target,relative)
        self.last_state = x
        transform = np.zeros((6,6)); transform[:3,:3]=transform[3:,3:]=frame.T
        transform[3:,:3]=-frame.T@skew(omega)
        cov = transform@covariance@transform.T
        rs,rt = state['r']
        origins = relative[:3]-rt@target_com+rs@service_com
        cam_p,cam_r = self.geometry.cameras_service(q)['left']
        mechanism = origins+rt@self.geometry.center
        self.depth = float((cam_r.T@rs.T@(mechanism-rs@cam_p))[2])
        if self.sequence is None:
            offset = float((self.geometry.cameras_service(self.q_prepared)['left'][1].T@
                self.service_mount.T@(-self.target_mount@target_com+self.service_mount@service_com+
                self.target_mount@self.geometry.center-self.service_mount@
                self.geometry.cameras_service(self.q_prepared)['left'][0]))[2])
            gp,gr = self.geometry.chain.forward(self.q_prepared)
            platform = (self.service_mount@(gp+gr@np.array([0.,0.,-.5]))
                        -self.target_mount@self.geometry.center+self.target_mount@target_com-self.service_mount@service_com)
            points = [np.array([depth-offset,0.,0.]) for depth in (17.4,5.5,1.5)]
            self.sequence = VbarStereoSequence(x[:3],*points,platform,
                epoch_s=now,mission_profile='near23_vbar_v1',with_grasp=True)
        seq = self.sequence
        rates = state['rates']-state['b']
        if self.visual_mount is None:
            reference,reference_rate = self.geometry.reference_motion(relative_position_n_m=origins,
                relative_velocity_n_m_s=relative[3:]-rt@np.cross(rates[1],target_com)+rs@np.cross(rates[0],service_com),
                service_rotation_n_from_body=rs,service_angular_rate_body_rad_s=rates[0],joint_position_rad=q,joint_velocity_rad_s=qdot)
        else:
            reference,reference_rate = frame@self.visual_mount,omega
        aligned = self.alignment.update(now_s=now,input_stamps_s=dict(navigation=last_gnss_ns*1e-9 if last_gnss_ns is not None else -1.,
            service_attitude=now,target_telemetry=last_cooperative_ns*1e-9 if last_cooperative_ns is not None else -1.,camera_kinematics=now),
            relative_position_n_m=origins,navigation_position_sigma_m=sigma,service_rotation_n_from_body=rs,
            target_rotation_n_from_body=rt,target_angular_rate_body_rad_s=rates[1],reference_angular_rate_n_rad_s=reference_rate,
            joint_position_rad=q,enabled=seq.target_pointing_enabled,inputs_valid=healthy,
            reference_rotation_n_from_body=reference if self.visual_mount is not None else None)
        held = frame@self.target_mount
        if seq.target_pointing_enabled or seq.phase in ('Grasp','Complete'):
            held = reference
        self.target_torque = bounded_pointing_torque(target_rotation_n_from_body=rt,reference_rotation_n_from_body=held,
            target_angular_rate_body_rad_s=rates[1],reference_angular_rate_n_rad_s=reference_rate if held is reference else omega)
        service_error = np.linalg.norm(rs.T@(frame@self.service_mount)-np.eye(3))
        gp,gr = self.geometry.chain.forward(q)
        ref,vel,acc = seq.update(now,x,covariance=cov,optical_depth_m=self.depth,
            available_acceleration_m_s2=min(.005,25./mass),observation_age_s=(now_ns-last_gnss_ns)*1e-9 if last_gnss_ns is not None else 1.,
            position_sigma_m=sigma,inputs_valid=healthy,service_ready=service_error<.0247,corridor_ready=True,
            prealigned=aligned.aligned,visual_ready=visual_ready,
            visual_stamp_s=visual_stamp_ns*1e-9 if visual_stamp_ns is not None else None,
            estimated_tool_center_gripper_m=gr.T@(rs.T@mechanism-gp),grasp_complete=grasp_complete,grasp_failure=grasp_failure)
        if seq.image_release_s is not None and self.visual_mount is None:
            self.visual_mount = frame.T@reference
        self.safe_hold = bool(seq.failure)
        self.last_reference = ref
        ref_n,vel_n = frame@ref,frame@vel
        phase = 'GraspHold' if seq.phase=='Complete' else seq.phase
        return dict(phase=phase,reason=seq.failure or aligned.reason,node=seq.telemetry['node_id'],
            released=seq.phase=='Complete',position_goal=state['p'][1]-ref_n,
            velocity_goal=state['v'][1]-vel_n-np.cross(omega,ref_n),
            differential_acceleration=-(frame@acc+2*np.cross(omega,vel_n)+np.cross(omega,np.cross(omega,ref_n))),
            reference_owner='vbar',optical_depth_m=self.depth,vbar=seq.telemetry,
            at_node=seq.telemetry['subsegment'].endswith('Hold'),
            position_error_m=float(np.linalg.norm(x[:3]-ref)),velocity_error_m_s=float(np.linalg.norm(x[3:]-vel)),
            platform_stable=seq.platform_completed_s is not None,platform_stable_duration_s=5. if seq.platform_completed_s else 0.)

    def snapshot(self):
        return dict(schema='space-sim-near23-route/1',epoch_ns=self.epoch_ns,
            events=self.sequence.events if self.sequence else [],
            telemetry=self.sequence.telemetry if self.sequence else {},
            visual_mount=self.visual_mount.tolist() if self.visual_mount is not None else None)


class Near23CaptureClient:
    """A single action producer for Capture26Component's existing arm drives."""
    def __init__(self, output, config_file, session_id, *, epoch_ns, own_motion, gyro_provider, com_provider):
        from simulation.stereo_predictive_capture import StereoPredictiveCapture
        self.servo = StereoPredictiveCapture(Path(output)/'capture_measurement.json',session_id,config_file=config_file)
        self.epoch_ns = int(epoch_ns)
        self.motion,self.gyro_provider,self.com_provider = own_motion,gyro_provider,com_provider
        self.active = False
        self.failure = None
        self.complete = False
        self.force_since = None
        self.hold_s = 0.
        self.component = None
        self.audit = None

    def latest_action(self):
        if not self.active:
            return {},True
        if self.failure:
            return self.servo.neutral(),False
        c = self.component
        t = self.epoch_ns*1e-9+c.last_time
        q = np.array([float(reader().state) for reader in c.positions])
        hits = self.audit.record(t,self.servo.sequence.phase,q)
        feedback = self.audit.feedback
        if hits:
            self.failure = 'service_self_contact'
        p,r = c.target.kinematics.forward(q[:6])
        try:
            own = self.motion.update(round(t*1e9),self.gyro_provider(),self.com_provider(),epoch=self.motion.epoch)
            action,_ = self.servo.command(t,q,(p,r),feedback,own_feedback=own)
        except ValueError as error:
            action,_ = self.servo.gyro_unavailable(str(error),t,q,feedback)
        if self.servo.sequence.phase=='abort':
            self.failure = self.servo.sequence.failure_reason or 'capture_controller_aborted'
        supported = (not self.failure and self.servo.sequence.phase=='hold_force'
                     and feedback.get('jaw_support_count',0)>=4 and 4.5<=feedback.get('jaw_force_n',0)<=5.5)
        self.force_since = (t if self.force_since is None else self.force_since) if supported else None
        self.hold_s = 0. if self.force_since is None else t-self.force_since
        self.complete = self.hold_s>=20.
        if self.failure:
            action['deadman']=False
        return action,False
