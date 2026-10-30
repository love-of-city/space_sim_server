"""Independent capture arm servos driven only by selected-arm commands."""
import math
import json
import xml.etree.ElementTree as ET
import numpy as np
from pathlib import Path
from simulation.assembly import Slot
from simulation.joint_reference_publisher import HeldJointReferencePublisher
from simulation.capture_arm_target import CaptureArmTarget, INITIAL_JOINTS
from simulation.capture_sequence import CaptureContactGuard

class Capture26Component:
    name = 'capture26_hold'

    def __init__(self, model, client, initial=None, experiment=False, visual_test=None, render_session_id='',
                 passive_hold=False):
        if (visual_test and visual_test.get('enabled') and visual_test.get('held_target_pose')
                and visual_test.get('mode')!='predictive_grasp'):
            raise ValueError('held_target_pose requires predictive_grasp')
        self.target = CaptureArmTarget(model)
        if initial is not None:
            initial = np.asarray(initial, dtype=float)
            if initial.shape != (6,) or not np.all(np.isfinite(initial)) or np.any(initial < self.target.lower[:6]) or np.any(initial > self.target.upper[:6]):
                raise ValueError('invalid capture arm initial joints')
            self.target.position[:6] = initial
        self.initial = self.target.position.copy()
        self.client = client
        self.passive_hold = passive_hold
        if passive_hold and (experiment or visual_test):
            raise ValueError('passive flight arm hold cannot run capture experiments')
        self.last_time = None
        self.experiment = experiment
        self.sequence = None
        self.log_time = -10.
        self.baseline = None
        self.relative_baseline = None
        self.contact_guard = CaptureContactGuard()
        self.model_path = model
        self.force_control = ET.parse(model).find("./custom/numeric[@name='capture_pad_force_control']") is not None
        self.visual_config=visual_test if visual_test and visual_test.get('enabled') else None
        self.visual=None;self.visual_status='';self.visual_log_time=-10.;self.visual_log_key=None
        self.render_session_id=render_session_id
        self.truth_file=None;self.control_file=None
        self.gyro_file=None;self.last_logged_gyro=None
        if self.visual_config:
            folder=Path(self.visual_config['output_directory']);folder.mkdir(parents=True,exist_ok=True)
            self.truth_file=(folder/'truth.jsonl').open('a',encoding='utf-8',buffering=1)
            if self.visual_config.get('mode') in ('servo','predictive_grasp'):
                from simulation.capture_sequence import VisualCaptureProbe
                source=Path(__file__).resolve().parents[2]/'run/wrist_alignment_observation.json'
                if self.visual_config.get('mode')=='predictive_grasp':
                    from simulation.visual_predictive_capture import VisualPredictiveCapture
                    self.predictor_type=VisualPredictiveCapture
                    self.predictor_kwargs=dict(held_pose=self.visual_config.get('held_target_pose',False))
                    if self.visual_config.get('input_source')=='near23_stereo':
                        from simulation.stereo_predictive_capture import StereoPredictiveCapture
                        root=Path(__file__).resolve().parents[2]
                        config=(root/self.visual_config['config_file']).resolve()
                        source=(root/self.visual_config.get('observation_file','run/near23_stereo_observation.json')).resolve()
                        if not config.is_relative_to(root) or not source.is_relative_to(root/'run'):
                            raise ValueError('stereo configuration/mailbox must remain inside server config/run')
                        self.predictor_type=StereoPredictiveCapture
                        self.predictor_kwargs['config_file']=config
                    elif self.visual_config.get('input_source','wrist')!='wrist':
                        raise ValueError('unknown capture input_source')
                    self.visual=self.predictor_type(source,render_session_id,**self.predictor_kwargs)
                else:
                    self.visual=VisualCaptureProbe(source,render_session_id,goal_gap_m=self.visual_config.get('goal_gap_m',.48))
                self.control_file=(folder/'visual_control.jsonl').open('a',encoding='utf-8',buffering=1)
                if self.visual_config.get('held_target_pose'):
                    self.gyro_file=(folder/'consumed_service_gyro.jsonl').open('a',encoding='utf-8',buffering=1)

    def install(self, ctx):
        self.stepper = getattr(ctx.simulation, 'local_stepper', None)
        self.own_motion=None
        if self.visual_config and self.visual_config.get('held_target_pose'):
            from simulation.own_motion import EncoderMassModel,OwnMotionFeedback
            self.mass_model=EncoderMassModel(self.model_path,self.stepper.body_inertial)
            self.own_encoder_readers={joint:ctx.scene.getBody(body).getScalarJoint(joint).stateOutMsg.addSubscriber()
                for body,joint in self.mass_model.joints}
            self.own_motion=OwnMotionFeedback()
            # Resolve lazily: installation order must not create a private sensor.
            self.gyro_provider=lambda:ctx.extra.get('service_gyro')
            self.own_com=None;self.own_com_time=-1.
        if self.experiment:
            from simulation.capture_sequence import CaptureSequence
            self.sequence = self.visual.sequence if self.visual else CaptureSequence(force_control=self.force_control)
            from simulation.capture_audit import CaptureAudit
            audit_options = ({'output_path': Path(self.visual_config['output_directory'])/'contacts_400hz.jsonl'}
                             if self.visual_config else {})
            self.audit = CaptureAudit(self.model_path, self.stepper, **audit_options)
            self.body_readers = [ctx.ports.body_state(n).addSubscriber()
                                 for n in ('cubesat_bus', 'capture_target')]
        elif self.visual_config:
            self.body_readers = [ctx.ports.body_state(n).addSubscriber()
                                 for n in ('cubesat_bus', 'capture_target')]
        joints = [ctx.scene.getBody(b).getScalarJoint(j) for b,j,_ in INITIAL_JOINTS]
        self.positions = [j.stateOutMsg.addSubscriber() for j in joints]
        self.rates = [j.stateDotOutMsg.addSubscriber() for j in joints]
        publisher = HeldJointReferencePublisher(self.reference, 12, publish_on_change=self.passive_hold)
        specs = [(f'bottom_joint{i}_act',1500.,80.,80.) for i in range(1,7)] + [
            ('axial_act',10000.,80.,15.), ('cam_act',10.,1.,1.),
            *[(f'jaw_act_{i}',8000.,40.,5.) for i in range(4)]]
        for i, ((_, joint, _), (act, kp, kd, limit)) in enumerate(zip(INITIAL_JOINTS, specs, strict=True)):
            ctx.ports.drive_servo(act, publisher.positionOutMsgs[i], publisher.velocityOutMsgs[i],
                                  owner=self.name, joint=joint, kp=kp, kd=kd, limit=limit)
        ctx.add(publisher, Slot.COMMAND)
        ctx.keep_alive(publisher)

    def reference(self, t):
        if not getattr(self.client, 'enabled', True):
            self.last_time = t
            return self.target.position.copy(), np.zeros(12)
        if self.passive_hold:
            # Flight owns a fixed authored reference. No operator/IK path runs,
            # and joint encoder reads remain available to independent telemetry.
            return self.target.position.copy(), np.zeros(12)
        if self.last_time is not None and t < self.last_time:
            self.target.position = self.initial.copy()
            self.target.was_active = False
            if self.sequence is not None:
                from simulation.capture_sequence import CaptureSequence
                if self.visual:
                    if self.visual_config.get('mode')=='predictive_grasp':
                        self.visual=self.predictor_type(self.visual.path,self.render_session_id,**self.predictor_kwargs)
                        if self.own_motion is not None:
                            from simulation.own_motion import OwnMotionFeedback
                            self.own_motion=OwnMotionFeedback();self.own_com_time=-1.;self.last_logged_gyro=None
                    else:
                        self.visual=type(self.visual)(self.visual.path,self.render_session_id,goal_gap_m=self.visual.goal_gap_m)
                self.sequence = self.visual.sequence if self.visual else CaptureSequence(force_control=self.force_control)
                self.baseline = None
                self.relative_baseline = None
                self.contact_guard = CaptureContactGuard()
                self.log_time = -10.
        dt = 0 if self.last_time is None else max(0, t-self.last_time)
        self.last_time = t
        action, stale = self.client.latest_action()
        measured = np.array([float(r().state) for r in self.positions])
        if self.sequence is not None:
            # Any fresh manual deadman command takes over and cancels the experiment.
            if not stale and action.get('deadman'):
                self.sequence.phase = 'abort'
            from Basilisk.utilities import RigidBodyKinematics as rbk
            from simulation.serial_chain_kinematics import axis_angle_to_matrix
            bus, target = [r() for r in self.body_readers]
            if np.linalg.norm(bus.r_BN_N) < 1e6 or np.linalg.norm(target.r_BN_N) < 1e6:
                # Initialization publishes once before orbital states are installed.
                return self.initial.copy(), np.zeros(12)
            Rs = np.asarray(rbk.MRP2C(bus.sigma_BN)).T
            Rt = np.asarray(rbk.MRP2C(target.sigma_BN)).T
            Ri = Rs.T @ Rt @ np.array([[1.,0,0],[0,0,1],[0,-1,0]])
            pif = Rs.T @ (np.asarray(target.r_BN_N)-bus.r_BN_N + Rt@np.array([.1755,.05475,.0420787001299893]))
            p, R = self.target.kinematics.forward(measured[:6])
            offset = Ri.T @ (p-pif)
            # Experiment guard: inspect real contacts, never teleport or constrain bodies.
            self_contacts = self.audit.record(t, self.sequence.phase, measured)
            if self_contacts and self.sequence.phase != 'abort':
                self.sequence.phase = 'abort'
                print(json.dumps(dict(type='capture_safety_abort', t=t, reason='service_self_contact', contacts=self_contacts)), flush=True)
            relative_position = Rs.T @ (np.asarray(target.r_BN_N)-bus.r_BN_N)
            if t >= 2.0 and self.relative_baseline is None:
                self.relative_baseline = relative_position.copy()
            if self.sequence.phase != 'abort' and not (self.visual and self.visual_config.get('mode')=='predictive_grasp'):
                reason = self.contact_guard.observe(t, relative_position,
                    self.audit.feedback['peak_target_contact_force_n'])
                if reason is not None:
                    self.sequence.phase = 'abort'
                    self.sequence.failure_reason = reason['reason']
                    print(json.dumps(dict(type='capture_safety_abort', t=t, **reason)), flush=True)
            from simulation.serial_chain_kinematics import rotation_matrix_to_vector
            if self.visual:
                # R comes from joint encoders/FK. Ri, pif and offset above are
                # independent truth diagnostics and MUST NOT enter this call.
                predictive=self.visual_config.get('mode')=='predictive_grasp'
                own_feedback=None
                gyro_error=None
                if self.own_motion is not None:
                    if self.own_com is None or t-self.own_com_time>=.019999:
                        self.own_com=self.mass_model.com({name:float(reader().state) for name,reader in self.own_encoder_readers.items()})
                        if self.own_com_time<0:
                            print(json.dumps(dict(type='capture_own_motion_configuration',service_mass_kg=self.mass_model.mass,
                                initial_encoder_com_body_m=self.own_com.tolist(),gyro_source='service_gyro',
                                gyro_sample_rate_hz=50,target_motion_extrapolation=False,
                                feedback='own encoders + shared timestamped gyro; COM model assumes free flight before contact')),flush=True)
                        self.own_com_time=t
                    # Only the shared sensor may synthesize a measurement from
                    # physical rates. Capture consumes its immutable cached packet.
                    sensor=self.gyro_provider()
                    reading=sensor.latest if sensor is not None else None
                    epoch=(self.render_session_id,str(self.client.reset_generation))
                    try:
                        own_feedback=self.own_motion.update(int(round(t*1e9)),reading,self.own_com,epoch=epoch)
                        stamp=(epoch,reading['sample_time_ns'])
                        if self.gyro_file is not None and stamp!=self.last_logged_gyro:
                            self.gyro_file.write(json.dumps(dict(render_session_id=epoch[0],reset_generation=epoch[1],
                                control_time_ns=str(round(t*1e9)),measurement=reading))+'\n')
                            self.last_logged_gyro=stamp
                    except ValueError as error:
                        gyro_error=str(error)
                if gyro_error is not None:
                    command,phase=self.visual.gyro_unavailable(gyro_error,t,measured,self.audit.feedback)
                elif own_feedback is not None:
                    command,phase=self.visual.command(t,measured,(p,R),self.audit.feedback,reference_from_base=own_feedback)
                else:
                    command, phase=self.visual.command(t,measured,(p,R) if predictive else R,self.audit.feedback)
                self.visual_status=phase
                packet=self.visual.last_packet or {}
                key=(packet.get('source_frame_id'),phase)
                if key!=self.visual_log_key or t-self.visual_log_time>=.2:
                    self.visual_log_key=key;self.visual_log_time=t
                    self.control_file.write(json.dumps(dict(t=t,status=phase,session_id=self.render_session_id,
                        source_frame_id=packet.get('source_frame_id'),input_status=self.visual.input_status,
                        commanded_path_m=self.visual.commanded_path_m,motion_samples=self.visual.motion_samples,
                        prediction=dict(self.visual.diagnostics(),own_reference_from_base=[x.tolist() for x in own_feedback] if own_feedback is not None else None,
                            gyro_feedback=dict(valid=own_feedback is not None,error=gyro_error,
                                **self.own_motion.diagnostics()) if self.own_motion is not None else None) if predictive else None,
                        command=command))+'\n')
            else:
                command, phase = self.sequence.command(t, measured, offset, R.T @ Ri, feedback=self.audit.feedback)
                command['end_effector_angular_velocity_body_rad_s'] = np.clip(.8*rotation_matrix_to_vector(Ri@R.T),-.04,.04).tolist() if phase in ('approach','close') or (phase=='tighten' and not self.force_control) else [0.,0.,0.]
                command['end_effector_linear_velocity_body_m_s'] = (Ri@np.array(command['end_effector_linear_velocity_body_m_s'])).tolist()
            if phase == 'abort':
                command['deadman'] = False
            result = self.target.step(dt, command, False, measured)
            self.target.status = phase if self.visual else 'experiment_' + phase
            tr = np.asarray(target.r_BN_N);tv = np.asarray(target.v_BN_N)
            if self.baseline is None or t <= 1.0:
                omega = np.cross(tr,tv)/np.dot(tr,tr)
                self.baseline = (t,tr.copy(),Rt.copy(),omega)
            if t-self.log_time >= 2.:
                self.log_time=t
                t0,r0,R0,w=self.baseline
                orbit_rotation=axis_angle_to_matrix(w,float(np.linalg.norm(w)*(t-t0)))
                record=dict(type='capture_window_trial',t=t,phase=phase,clamp=self.audit.feedback,force_control=self.force_control,failure_reason=self.sequence.failure_reason,interface_offset_m=offset.tolist(),
                    joints_deg=np.rad2deg(measured[:6]).tolist(),axial_m=float(measured[6]),radial_m=measured[8:].tolist(),
                    relative_position_change_body_m=(relative_position-self.relative_baseline).tolist() if self.relative_baseline is not None else [0.,0.,0.],
                    first_target_contact_t=self.contact_guard.first_contact_time,
                    post_contact_relative_change_m=self.contact_guard.last_displacement_m,
                    target_rotation_residual_deg=np.rad2deg(rotation_matrix_to_vector(Rt@(orbit_rotation@R0).T)).tolist(),
                    target_omega_rad_s=list(target.omega_BN_B),relative_origin_body_m=(Rs.T@(tr-bus.r_BN_N)).tolist())
                print(json.dumps(record),flush=True)
            return result
        return self.target.step(dt, action, stale, measured)

    def telemetry(self):
        return dict(self.target.telemetry([float(r().state) for r in self.positions],
                                     [float(r().state) for r in self.rates]),
                    visual_probe_enabled=self.visual is not None,
                    visual_predictive_enabled=bool(self.visual and self.visual_config.get('mode')=='predictive_grasp'),
                    visual_prediction=self.visual.diagnostics() if self.visual and self.visual_config.get('mode')=='predictive_grasp' else None)

    def scoring_snapshot(self,frame,time_ns):
        """Independent same-frame truth for scoring; never read by visual control."""
        if self.truth_file is None:return
        from Basilisk.utilities import RigidBodyKinematics as rbk
        bus,target=[reader() for reader in self.body_readers]
        rs=np.asarray(rbk.MRP2C(bus.sigma_BN)).T;rt=np.asarray(rbk.MRP2C(target.sigma_BN)).T
        q=np.array([float(r().state) for r in self.positions])
        p,rg=self.target.kinematics.forward(q[:6])
        interface=rs.T@(np.asarray(target.r_BN_N)-bus.r_BN_N+rt@np.array([.1755,.05475,.0420787001299893]))
        self.truth_file.write(json.dumps(dict(session_id=self.render_session_id,source_frame_id=str(frame),
            sim_time_ns=str(time_ns),interface_position_grip_m=(rg.T@(interface-p)).tolist(),
            R_grip_from_interface=(rg.T@rs.T@rt@np.array([[1.,0,0],[0,0,1],[0,-1,0]])).tolist(),
            phase=self.visual_status,feedback=self.audit.feedback if self.experiment else None,
            prediction_control_time_s=self.visual.last_time if self.visual else None,
            prediction=self.visual.diagnostics() if self.visual and self.visual_config.get('mode')=='predictive_grasp' else None))+'\n')
