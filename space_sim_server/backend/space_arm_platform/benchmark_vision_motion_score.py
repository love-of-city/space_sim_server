"""Score target spin against native motion, never the method's own verdict."""
import json
import math

import numpy as np

from .benchmark_contract import Contract
from .benchmark_vision import VisionPoseEstimateEvidence
from .benchmark_vision_motion import VisionMotionEstimateEvidence, VisionMotionObservation, VisionMotionTruthSample
from .benchmark_vision_score import VisionPoseScorer, verify_public_image_evidence


def verify_motion_observations(task,observations_path,physics_path):
    verify_public_image_evidence(task,observations_path,physics_path,observation_type=VisionMotionObservation)
    gyros={}
    with physics_path.open(encoding='utf-8') as stream:
        for line in stream:
            raw=json.loads(line)
            if raw.get('record_type') is None:
                sample=VisionMotionTruthSample.model_validate(raw)
                if sample.time_ns in gyros:
                    raise ValueError('duplicate private gyroscope evidence')
                gyros[sample.time_ns]=sample.chaser_gyro
    seen=set()
    with observations_path.open(encoding='utf-8') as stream:
        for line in stream:
            for packet in VisionMotionObservation.model_validate_json(line).gyro_samples:
                if packet.sample_time_ns in seen or gyros.get(packet.sample_time_ns)!=packet:
                    raise ValueError('public own-gyro differs from the sampled physical sensor evidence')
                seen.add(packet.sample_time_ns)
    # A failed acquisition may have physical samples after the last delivered
    # image. Every delivered interval must match; unobserved tail samples do
    # not provide estimator credit or make a partial episode successful.
    if seen and seen!=set(range(0,max(seen)+1,task.own_gyro.sample_period_ns)):
        raise ValueError('public gyro history contains a sampling gap')


class VisionMotionScorer(VisionPoseScorer):
    def __init__(self,task,chaser_mass_kg):
        super().__init__(task,chaser_mass_kg)
        self.spin_count=self.available_spin_count=self.invalid_gyro_count=0
        self.sum_spin_error_squared=self.peak_spin_error=0.

    def initial_rates_valid(self,sample):
        initial=self.task.initial
        def valid(name):
            actual=np.asarray(getattr(sample,name+'_angular_velocity_body_rad_s'))
            mean=np.asarray(getattr(initial,name+'_rate_body_rad_s'))
            return any(np.all(np.abs(actual-sign*mean)<=getattr(initial,name+'_rate_half_width_rad_s')+1e-8)
                       for sign in ((-1.,1.) if name=='target' else (1.,)))
        return all(valid(name) for name in ('target','chaser'))

    def add(self,raw):
        document=raw.model_dump() if isinstance(raw,Contract) else raw
        try:
            kind=document.get('record_type') if isinstance(document,dict) else None
            if kind=='vision_motion_estimate':
                self._spin_estimate(VisionMotionEstimateEvidence.model_validate(document))
            elif kind is None:
                sample=VisionMotionTruthSample.model_validate(document)
                self._physical(sample)
                model=self.task.own_gyro
                expected=(model.white_noise_std_rad_s**2+model.initial_bias_std_rad_s**2
                          +model.bias_random_walk_rad_s_sqrt_s**2*sample.time_ns*1e-9)
                if (sample.chaser_gyro.sample_time_ns!=sample.time_ns
                        or any(not math.isclose(value,expected,rel_tol=1e-12,abs_tol=1e-20)
                               for value in sample.chaser_gyro.error_variance_rad2_s2)):
                    self.evidence_errors.add('stale_or_misconfigured_own_gyro_evidence')
                self.invalid_gyro_count+=not sample.chaser_gyro.valid
            elif kind=='vision_capture':
                super().add(document)
            else:
                raise ValueError('unexpected spin evidence record type')
        except (ValueError,TypeError) as error:
            self.evidence_errors.add('malformed_motion_evidence')
            raise ValueError('invalid image/gyro spin evidence') from error

    def _spin_estimate(self,estimate):
        before=self.estimate_count
        self._estimate(VisionPoseEstimateEvidence(sequence=estimate.sequence,estimate_time_ns=estimate.estimate_time_ns,
            reference_frame=estimate.reference_frame,image_sha256=estimate.image_sha256,pose=estimate.pose))
        if self.estimate_count!=before+1:
            return
        if estimate.angular_velocity_frame!=self.task.angular_velocity_frame:
            self.evidence_errors.add('wrong_angular_velocity_reference_frame')
        rate=estimate.target_angular_velocity_body_rad_s
        if rate is not None and estimate.pose is None:
            self.evidence_errors.add('target_body_rate_without_target_pose')
        if estimate.estimate_time_ns<self.task.objective.spin_warmup_ns:
            return
        self.spin_count+=1
        if rate is None:
            return
        error=float(np.linalg.norm(np.asarray(rate)-self.latest.target_angular_velocity_body_rad_s))
        self.available_spin_count+=1
        self.sum_spin_error_squared+=error**2
        self.peak_spin_error=max(self.peak_spin_error,error)

    def result(self):
        result=super().result()
        goal=self.task.objective
        rmse=math.sqrt(self.sum_spin_error_squared/self.available_spin_count) if self.available_spin_count else None
        quality=[]
        if self.available_spin_count<self.task.expected_spin_estimates:
            quality.append('spin_availability')
        if rmse is not None and rmse>goal.angular_velocity_rmse_limit_rad_s:
            quality.append('angular_velocity_rmse')
        if self.available_spin_count and self.peak_spin_error>goal.maximum_angular_velocity_error_rad_s:
            quality.append('maximum_angular_velocity_error')
        if result['outcome']=='success' and quality:
            result.update(outcome='task_failure',success=False,completion_time_s=None)
        result['quality_violations']+=quality
        result.update(schema_version='spacesim-autonomy-rendered-spin-score/1',spin_estimate_count=self.spin_count,
            expected_spin_estimates=self.task.expected_spin_estimates,available_spin_count=self.available_spin_count,
            missing_spin_count=self.spin_count-self.available_spin_count,
            angular_velocity_rmse_rad_s=rmse,
            maximum_angular_velocity_error_rad_s=self.peak_spin_error if self.available_spin_count else None,
            invalid_own_gyro_count=self.invalid_gyro_count,angular_velocity_frame=self.task.angular_velocity_frame,
            evaluation_scope='real stereo images plus own 50 Hz gyro; passive spinning marked target; no target link')
        return result
