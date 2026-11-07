"""Physical camera-relative approach, image fidelity and native force receipts."""
import numpy as np

from .benchmark_contract import Contract
from .benchmark_vision import VisionPoseEstimateEvidence,VisionPoseTruthSample
from .benchmark_vision_approach import VisionApproachEstimateEvidence
from .benchmark_vision_score import VisionPoseScorer,physical_camera,rotation


def optical_relative_motion(sample,camera):
    origin,orientation=physical_camera(sample,camera)
    target_com=np.asarray(sample.target_com_j2000)
    chaser_com=np.asarray(sample.chaser_com_j2000)
    target_rate=rotation(sample.target_orientation_body_to_j2000_wxyz)@sample.target_angular_velocity_body_rad_s
    chaser_rate=rotation(sample.chaser_orientation_body_to_j2000_wxyz)@sample.chaser_angular_velocity_body_rad_s
    target_velocity=target_com[3:]+np.cross(target_rate,np.asarray(sample.target_origin_j2000_m)-target_com[:3])
    camera_velocity=chaser_com[3:]+np.cross(chaser_rate,origin-chaser_com[:3])
    position=orientation.T@(np.asarray(sample.target_origin_j2000_m)-origin)
    velocity=orientation.T@(target_velocity-camera_velocity)-np.cross(orientation.T@chaser_rate,position)
    return position,velocity


class VisionApproachScorer(VisionPoseScorer):
    def __init__(self,task,chaser_mass_kg):
        super().__init__(task,chaser_mass_kg)
        self.mass=chaser_mass_kg
        self.command=np.zeros(3)
        self.inside_since_ns=None
        self.dwell_s=0.
        self.impulse_n_s=0.
        self.position_error=self.optical_speed=self.initial_position_error=None

    def add(self,raw):
        document=raw.model_dump() if isinstance(raw,Contract) else raw
        try:
            kind=document.get('record_type') if isinstance(document,dict) else None
            if kind=='vision_approach_action':
                self._command(VisionApproachEstimateEvidence.model_validate(document))
            elif kind is None:
                self._physical(VisionPoseTruthSample.model_validate(document))
            elif kind=='vision_capture':
                super().add(document)
            else:
                raise ValueError('unexpected visual approach evidence record')
        except (ValueError,TypeError) as error:
            self.evidence_errors.add('malformed_visual_approach_evidence')
            raise ValueError('invalid visual approach evidence') from error

    def _command(self,record):
        before=self.estimate_count
        self._estimate(VisionPoseEstimateEvidence(sequence=record.sequence,estimate_time_ns=record.estimate_time_ns,
            reference_frame=record.reference_frame,image_sha256=record.image_sha256,pose=record.pose))
        if self.estimate_count!=before+1:
            return
        if record.force_frame!=self.task.force_frame:
            self.evidence_errors.add('wrong_body_force_reference_frame')
        if np.linalg.norm(record.force_body_n)>self.task.safety.maximum_force_n+1e-12:
            self.safety_violations.add('requested_force_limit')
        self.command=np.asarray(record.force_body_n)

    def _check_actuation(self,sample):
        limits=self.task.safety
        actual=np.asarray(sample.applied_motor_force_body_n)
        if np.linalg.norm(actual-self.command)>max(limits.maximum_uncommanded_force_n,1e-10):
            self.evidence_errors.add('body_force_receipt_differs_from_command')
        if np.linalg.norm(actual)>limits.maximum_force_n+1e-10:
            self.safety_violations.add('physical_force_limit')
        if np.linalg.norm(sample.applied_wheel_torque_xyz_nm)>limits.maximum_uncommanded_wheel_torque_nm:
            self.safety_violations.add('undeclared_attitude_actuation')

    def _physical(self,sample):
        previous=self.latest
        super()._physical(sample)
        position,velocity=optical_relative_motion(sample,self.task.cameras[0])
        goal=self.task.objective
        self.position_error=float(np.linalg.norm(position-goal.target_origin_optical_goal_m))
        self.optical_speed=float(np.linalg.norm(velocity))
        if previous is None:
            self.initial_position_error=self.position_error
            if self.position_error<goal.minimum_initial_position_error_m:
                self.initialization_errors.add('visual_approach_started_too_close_to_goal')
        elif sample.time_ns==previous.time_ns+self.task.physics_period_ns:
            # The force is held over the just-completed physical interval.
            self.impulse_n_s+=float(np.linalg.norm(sample.applied_motor_force_body_n))*self.task.physics_period_ns*1e-9
        if self.position_error<=goal.position_tolerance_m and self.optical_speed<=goal.optical_relative_speed_tolerance_m_s:
            if self.inside_since_ns is None:
                self.inside_since_ns=sample.time_ns
            self.dwell_s=(sample.time_ns-self.inside_since_ns)*1e-9
        else:
            self.inside_since_ns=None
            self.dwell_s=0.

    def result(self):
        score=super().result()
        time_ns=0 if self.latest is None else self.latest.time_ns
        period=self.task.decision_period_ns
        expected=(time_ns+period-1)//period
        at_boundary=time_ns%period==0
        pending_command=(at_boundary and self.estimate_count==expected+1
                         and self.capture_count==expected+1 and time_ns<round(self.task.duration_s*1e9))
        fraction=self.available_count/self.estimate_count if self.estimate_count else 0.
        quality=[name for name in score['quality_violations'] if name!='pose_availability']
        if fraction<self.task.objective.minimum_available_fraction:
            quality.append('pose_availability')
        reached=self.dwell_s>=self.task.objective.dwell_s
        end=self.latest is not None and self.latest.time_ns>=round(self.task.duration_s*1e9)
        if score['outcome'] in {'invalid_initialization','invalid_evidence','safety_violation'}:
            outcome=score['outcome']
        elif pending_command:
            # An accepted request may be cancelled before its first native
            # interval. It has no physical completion credit.
            outcome='incomplete'
        elif self.estimate_count!=expected or self.capture_count not in {expected,expected+1}:
            outcome='invalid_evidence'
            score['evidence_errors'].append('incomplete_visual_control_coverage')
        elif not at_boundary or self.capture_count==expected:
            # Native motion precedes the next image. Cancellation can interrupt
            # that image or the motion interval. All applied controls still need
            # their preceding captures and receipts; a missing terminal image
            # can never certify success, even when physical dwell was reached.
            outcome='incomplete'
        elif reached:
            outcome='task_failure' if quality else 'success'
        else:
            outcome='time_limit' if end else 'incomplete'
        score.update(schema_version='spacesim-autonomy-visual-approach-score/1',outcome=outcome,success=outcome=='success',
            completion_time_s=score['final_time_s'] if outcome=='success' else None,
            expected_estimates=expected,maximum_planned_estimates=self.task.expected_estimates,
            available_pose_fraction=fraction,quality_violations=quality,
            initial_position_error_m=self.initial_position_error,position_error_m=self.position_error,
            optical_relative_speed_m_s=self.optical_speed,dwell_s=self.dwell_s,
            thrust_impulse_n_s=self.impulse_n_s,thrust_delta_v_m_s=self.impulse_n_s/self.mass,
            force_frame=self.task.force_frame,
            evaluation_scope='real stereo image feedback to body-axis COM motors; passive marked target; physical continuous dwell')
        return score
