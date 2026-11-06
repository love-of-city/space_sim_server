"""Pose accuracy from native body motion, independent of image-only methods."""
from __future__ import annotations

import math
import json
import numpy as np
from scipy.spatial.transform import Rotation

from .benchmark_contract import Contract
from .benchmark_navigation_score import initial_orbit_errors
from .benchmark_vision import VisionCaptureEvidence, VisionPoseEstimateEvidence, VisionPoseTruthSample, VisionPoseObservation


def verify_public_image_evidence(task, observations_path, physics_path):
    """Bind exported image bytes to the private captures used by the grader."""
    captures={}
    with physics_path.open(encoding='utf-8') as stream:
        for line in stream:
            value=json.loads(line)
            if value.get('record_type')=='vision_capture':
                capture=VisionCaptureEvidence.model_validate(value)
                if capture.sequence in captures:
                    raise ValueError('duplicate capture evidence')
                captures[capture.sequence]=capture
    observation_count=0
    with observations_path.open(encoding='utf-8') as stream:
        for sequence,line in enumerate(stream):
            observation_count+=1
            observation=VisionPoseObservation.model_validate_json(line)
            capture=captures.get(sequence)
            if (capture is None or observation.sequence!=sequence
                    or observation.sample_time_ns!=sequence*task.decision_period_ns
                    or tuple(image.camera_id for image in observation.images)!=capture.camera_ids
                    or tuple(image.png_sha256 for image in observation.images)!=capture.image_sha256
                    or any(image.renderer_session_id!=capture.renderer_session_id
                           or image.source_frame_id!=capture.source_frame_id for image in observation.images)
                    or any((image.width,image.height)!=(camera.width,camera.height)
                           for image,camera in zip(observation.images,task.cameras,strict=True))):
                raise ValueError('public images differ from authoritative physical capture evidence')
    if observation_count!=len(captures):
        raise ValueError('public image log is incomplete relative to authoritative captures')


def rotation(q):
    return Rotation.from_quat(np.asarray(q)[[1,2,3,0]]).as_matrix()


def rotation_error(a,b):
    # atan2 remains accurate near zero and pi without clipping away small errors.
    relative = a@b.T
    sine = np.linalg.norm([relative[2,1]-relative[1,2],relative[0,2]-relative[2,0],relative[1,0]-relative[0,1]])/2
    cosine = (np.trace(relative)-1)/2
    return float(math.atan2(sine,cosine))


def physical_camera(sample,camera):
    body = rotation(sample.chaser_orientation_body_to_j2000_wxyz)
    origin = np.asarray(sample.chaser_origin_j2000_m)+body@camera.position_body_m
    orientation = body@rotation(camera.orientation_body_from_optical_wxyz)
    return origin,orientation


class VisionPoseScorer:
    def __init__(self,task,chaser_mass_kg):
        if not math.isfinite(chaser_mass_kg) or chaser_mass_kg <= 0:
            raise ValueError('physical spacecraft mass must be finite and positive')
        self.task = task
        self.latest = self.capture = None
        self.count = self.capture_count = self.estimate_count = self.available_count = 0
        self.renderer_session_id = self.last_source_frame = None
        self.sum_position_squared = self.sum_orientation_squared = 0.
        self.peak_position = self.peak_orientation = 0.
        self.minimum_range = None
        self.initialization_errors,self.evidence_errors,self.safety_violations = set(),set(),set()

    def add(self,raw):
        document = raw.model_dump() if isinstance(raw,Contract) else raw
        try:
            kind = document.get('record_type') if isinstance(document,dict) else None
            if kind == 'vision_pose_estimate':
                self._estimate(VisionPoseEstimateEvidence.model_validate(document))
            elif kind == 'vision_capture':
                self._capture(VisionCaptureEvidence.model_validate(document))
            else:
                self._physical(VisionPoseTruthSample.model_validate(document))
        except (ValueError,TypeError) as error:
            self.evidence_errors.add('malformed_vision_evidence')
            raise ValueError('invalid rendered-pose evidence') from error

    def _physical(self,sample):
        task,limits = self.task,self.task.safety
        expected = 0 if self.latest is None else self.latest.time_ns+task.physics_period_ns
        if sample.time_ns != expected or sample.time_ns > round(task.duration_s*1e9):
            self.evidence_errors.add('missing_duplicate_reordered_or_out_of_range_physics')
        if sample.external_state_writes:
            self.evidence_errors.add('physical_state_overwritten_after_initialization')
        relative = np.asarray(sample.chaser_com_j2000)-sample.target_com_j2000
        radius = float(np.linalg.norm(relative[:3]))
        self.minimum_range = radius if self.minimum_range is None else min(self.minimum_range,radius)
        if not limits.minimum_range_m <= radius <= limits.maximum_range_m:
            self.safety_violations.add('relative_range')
        if np.linalg.norm(relative[3:]) > limits.maximum_relative_speed_m_s:
            self.safety_violations.add('relative_speed')
        if sample.contacts:
            self.safety_violations.add('unplanned_physical_contact')
        if any(np.linalg.norm(rate) > limits.maximum_body_rate_rad_s for rate in
               (sample.target_angular_velocity_body_rad_s,sample.chaser_angular_velocity_body_rad_s)):
            self.safety_violations.add('body_angular_rate')
        if (np.linalg.norm(sample.applied_motor_force_body_n) > limits.maximum_uncommanded_force_n
                or np.linalg.norm(sample.applied_wheel_torque_xyz_nm) > limits.maximum_uncommanded_wheel_torque_nm):
            self.safety_violations.add('pose_estimator_has_no_actuation_authority')
        if self.latest is None:
            initial = task.initial
            if initial_orbit_errors(sample.target_com_j2000,initial.target_altitude_m,0.,initial.inclination_deg,initial.true_anomaly_deg,0.):
                self.initialization_errors.add('initial_target_orbit_outside_distribution')
            if (np.any(abs(relative[:3]-initial.chaser_minus_target_com_j2000_m) > initial.position_half_width_m+1e-5)
                    or np.any(abs(relative[3:]-initial.chaser_minus_target_velocity_j2000_m_s) > initial.velocity_half_width_m_s+1e-8)):
                self.initialization_errors.add('initial_relative_state_outside_distribution')
            r = rotation(sample.target_orientation_body_to_j2000_wxyz)
            normal = r[:,1]
            tilt = math.atan2(float(np.linalg.norm(normal[[0,2]])),float(normal[1]))
            if not initial.target_tilt_min_rad-1e-8 <= tilt <= initial.target_tilt_max_rad+1e-8:
                self.initialization_errors.add('initial_target_tilt_outside_distribution')
            axis = np.cross([0.,1.,0.],normal)
            if np.linalg.norm(axis) > 1e-12:
                tilt_matrix = Rotation.from_rotvec(axis/np.linalg.norm(axis)*tilt).as_matrix()
                roll_matrix = tilt_matrix.T@r
                roll = abs(math.atan2(float(roll_matrix[0,2]),float(roll_matrix[0,0])))
                if roll > initial.target_roll_half_width_rad+1e-8:
                    self.initialization_errors.add('initial_target_roll_outside_distribution')
            if (rotation_error(rotation(sample.chaser_orientation_body_to_j2000_wxyz),np.eye(3)) > 1e-8
                    or np.linalg.norm(sample.chaser_angular_velocity_body_rad_s) > 1e-8
                    or np.linalg.norm(sample.target_angular_velocity_body_rad_s) > 1e-8):
                self.initialization_errors.add('initial_spacecraft_attitude_or_rate_outside_distribution')
        self.latest = sample
        self.count += 1
        # Captures/estimates at every decision instant before terminal t=duration.
        # The terminal observation may contain the last image, but earns no
        # estimate credit and cannot compensate a missing earlier sample.
        if sample.time_ns == round(task.duration_s*1e9) and self.estimate_count != task.expected_estimates:
            self.evidence_errors.add('incomplete_pose_estimate_coverage')

    def _capture(self,capture):
        expected_time = self.capture_count*self.task.decision_period_ns
        if (self.latest is None or capture.sequence != self.capture_count or capture.sample_time_ns != expected_time
                or self.latest.time_ns != expected_time or self.capture_count > self.task.expected_estimates
                or self.capture_count != self.estimate_count):
            self.evidence_errors.add('missing_duplicate_reordered_or_unconsumed_image')
            return
        if (capture.camera_ids != tuple(camera.camera_id for camera in self.task.cameras)
                or capture.calibration_sha256 != tuple(camera.fingerprint() for camera in self.task.cameras)):
            self.evidence_errors.add('image_camera_or_calibration_differs')
        if self.renderer_session_id is None:
            self.renderer_session_id = capture.renderer_session_id
        if (capture.renderer_session_id != self.renderer_session_id
                or self.last_source_frame is not None and capture.source_frame_id != self.last_source_frame+1):
            self.evidence_errors.add('renderer_session_changed_or_frame_gap')
        for camera,actual_origin,actual_q in zip(self.task.cameras,capture.camera_origins_j2000_m,
                                                capture.camera_orientations_j2000_from_optical_wxyz,strict=True):
            origin,r = physical_camera(self.latest,camera)
            if np.linalg.norm(origin-actual_origin) > 1e-5 or rotation_error(r,rotation(actual_q)) > 1e-7:
                self.evidence_errors.add('rendered_camera_pose_differs_from_physics_and_mount')
        self.capture,self.last_source_frame = capture,capture.source_frame_id
        self.capture_count += 1

    def _estimate(self,estimate):
        expected_time = self.estimate_count*self.task.decision_period_ns
        if (self.latest is None or self.capture is None or estimate.sequence != self.estimate_count
                or estimate.estimate_time_ns != expected_time or self.latest.time_ns != expected_time
                or self.capture.sequence != self.estimate_count or self.estimate_count >= self.task.expected_estimates
                or estimate.image_sha256 != self.capture.image_sha256 or estimate.reference_frame != self.task.reference_frame):
            self.evidence_errors.add('missing_duplicate_stale_or_wrong_image_pose_estimate')
            return
        self.estimate_count += 1
        if estimate.pose is None:
            return
        camera_origin,camera_rotation = physical_camera(self.latest,self.task.cameras[0])
        position = camera_rotation.T@(np.asarray(self.latest.target_origin_j2000_m)-camera_origin)
        orientation = camera_rotation.T@rotation(self.latest.target_orientation_body_to_j2000_wxyz)
        error_p = float(np.linalg.norm(np.asarray(estimate.pose.position_target_origin_optical_m)-position))
        error_r = rotation_error(rotation(estimate.pose.orientation_optical_from_target_wxyz),orientation)
        self.available_count += 1
        self.sum_position_squared += error_p**2
        self.sum_orientation_squared += error_r**2
        self.peak_position,self.peak_orientation = max(self.peak_position,error_p),max(self.peak_orientation,error_r)

    def result(self):
        task = self.task
        complete = self.latest is not None and self.latest.time_ns == round(task.duration_s*1e9)
        fraction = self.available_count/task.expected_estimates
        p = math.sqrt(self.sum_position_squared/self.available_count) if self.available_count else None
        r = math.sqrt(self.sum_orientation_squared/self.available_count) if self.available_count else None
        goal = task.objective
        quality = [name for name,value,limit in (
            ('position_rmse',p,goal.position_rmse_limit_m),('orientation_rmse',r,goal.orientation_rmse_limit_rad),
            ('maximum_position_error',self.peak_position,goal.maximum_position_error_m),
            ('maximum_orientation_error',self.peak_orientation,goal.maximum_orientation_error_rad))
            if value is not None and value > limit]
        if fraction < goal.minimum_available_fraction:
            quality.append('pose_availability')
        if self.initialization_errors:
            outcome = 'invalid_initialization'
        elif not self.count or self.evidence_errors:
            outcome = 'invalid_evidence'
        elif self.safety_violations:
            outcome = 'safety_violation'
        elif complete:
            outcome = 'task_failure' if quality else 'success'
        else:
            outcome = 'incomplete'
        return dict(schema_version='spacesim-autonomy-rendered-pose-score/1',task_fingerprint=task.fingerprint(),
            outcome=outcome,success=outcome=='success',sample_count=self.count,capture_count=self.capture_count,
            estimate_count=self.estimate_count,expected_estimates=task.expected_estimates,
            available_pose_count=self.available_count,missing_pose_count=self.estimate_count-self.available_count,
            available_pose_fraction=fraction,position_rmse_m=p,orientation_rmse_rad=r,
            maximum_position_error_m=self.peak_position if self.available_count else None,
            maximum_orientation_error_rad=self.peak_orientation if self.available_count else None,
            final_time_s=self.latest.time_ns*1e-9 if self.latest else None,
            completion_time_s=task.duration_s if outcome=='success' else None,minimum_range_m=self.minimum_range,
            initialization_errors=sorted(self.initialization_errors),evidence_errors=sorted(self.evidence_errors),
            safety_violations=sorted(self.safety_violations),quality_violations=quality,
            fuel_kg=None,energy_j=None,estimate_frame=task.reference_frame,
            evaluation_scope='real rendered image pose estimation; passive marked target; no active target link or actuation')
