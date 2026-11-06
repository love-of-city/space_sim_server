"""Passive original-team spacecraft, real UE images, independent pose grading."""
from __future__ import annotations

import json
from pathlib import Path

import numpy as np
from Basilisk.utilities import RigidBodyKinematics as rbk
from scipy.spatial.transform import Rotation

from simulation.benchmark_native_flight import NativeWaypointEnvironment
from simulation.benchmark_rendered_camera import FLU_TO_OPTICAL, RenderedCameraSensor
from simulation.orbital_rendezvous.reference import EARTH_RADIUS, MU
from space_arm_platform.benchmark_vision import (
    FiducialMarker, VisionCaptureEvidence, VisionPoseContext, VisionPoseEstimateEvidence,
    VisionPoseObservation, VisionPoseTruthSample,
)
from space_arm_platform.benchmark_vision_score import VisionPoseScorer


def public_context(task,chaser_mass_kg):
    root=Path(__file__).resolve().parents[1]
    rig=json.loads((root/'config/visual_attachments/target_aruco_flush_5cm_v2/aruco_rig.json').read_text('utf-8'))
    markers=[]
    for marker in rig['markers']:
        c,u,v=np.asarray(marker['center_m']),np.asarray(marker['right']),np.asarray(marker['down'])
        h=marker['size_m']/2
        corners=[tuple(c+h*(x*u+y*v)) for x,y in ((-1,-1),(1,-1),(1,1),(-1,1))]
        markers.append(FiducialMarker(marker_id=marker['id'],corners_target_m=tuple(corners)))
    return VisionPoseContext(task=task,chaser_mass_kg=chaser_mass_kg,marker_dictionary=rig['dictionary'],markers=tuple(markers))


class NativeVisionPoseEnvironment(NativeWaypointEnvironment):
    context_type=staticmethod(public_context)
    scorer_type=VisionPoseScorer

    def __init__(self,task,output_directory):
        self._camera_sensor=None
        super().__init__(task,output_directory)

    def _setup_observation(self,ctx):
        # No target telemetry, orbit filter, gyro or truth-initialized estimator.
        self._body_readers=[ctx.ports.body_state(name).addSubscriber() for name in ('capture_target','cubesat_bus')]
        self._wheel_indices=[self._stepper.actuator_names.index(f'rw_{axis}_motor') for axis in 'xyz']

    def _initial_states(self,seed):
        initial=self.task.initial
        theta,inc=np.deg2rad([initial.true_anomaly_deg,initial.inclination_deg])
        radius=EARTH_RADIUS+initial.target_altitude_m
        radial=np.array([np.cos(theta),np.sin(theta)*np.cos(inc),np.sin(theta)*np.sin(inc)])
        tangent=np.array([-np.sin(theta),np.cos(theta)*np.cos(inc),np.cos(theta)*np.sin(inc)])
        target=np.r_[radius*radial,np.sqrt(MU/radius)*tangent]
        rng=np.random.default_rng(np.random.SeedSequence([seed,1]))
        position=np.asarray(initial.chaser_minus_target_com_j2000_m)+rng.uniform(-initial.position_half_width_m,initial.position_half_width_m,3)
        velocity=np.asarray(initial.chaser_minus_target_velocity_j2000_m_s)+rng.uniform(-initial.velocity_half_width_m_s,initial.velocity_half_width_m_s,3)
        return np.array([target,target+np.r_[position,velocity]])

    def _initialize_bodies(self,states,seed):
        initial=self.task.initial
        rng=np.random.default_rng(np.random.SeedSequence([seed,2]))
        azimuth=rng.uniform(-np.pi,np.pi)
        axis=np.array([np.cos(azimuth),0.,np.sin(azimuth)])
        tilt=Rotation.from_rotvec(axis*rng.uniform(initial.target_tilt_min_rad,initial.target_tilt_max_rad))
        roll=Rotation.from_euler('y',rng.uniform(-initial.target_roll_half_width_rad,initial.target_roll_half_width_rad))
        orientations=[tilt*roll,Rotation.identity()]
        self.initial_attitudes=[]
        for index,name in enumerate(('capture_target','cubesat_bus')):
            rotation=orientations[index].as_matrix()
            q=orientations[index].as_quat()[[3,0,1,2]]
            center=np.asarray(self.model_manifest['bodies'][name]['center_body_m'])
            body=self._scene.getBody(name)
            body.setPosition((states[index,:3]-rotation@center).tolist())
            body.setVelocity(states[index,3:].tolist())
            body.setAttitude(rbk.EP2MRP(q).tolist())
            body.setAttitudeRate([0.,0.,0.])
            self.initial_attitudes.append(dict(body=name,orientation_body_to_j2000_wxyz=q.tolist(),angular_velocity_body_rad_s=[0.,0.,0.]))

    def _truth_sample(self,nanos):
        states=self._read_physical_states(nanos)
        if any(int(reader.timeWritten())!=nanos for reader in self._body_readers):
            raise RuntimeError('stale native body pose cannot be grading evidence')
        target,chaser=[reader() for reader in self._body_readers]
        return VisionPoseTruthSample(time_ns=int(nanos),target_com_j2000=tuple(states[0]),chaser_com_j2000=tuple(states[1]),
            target_origin_j2000_m=tuple(target.r_BN_N),chaser_origin_j2000_m=tuple(chaser.r_BN_N),
            target_orientation_body_to_j2000_wxyz=tuple(rbk.MRP2EP(target.sigma_BN)),
            chaser_orientation_body_to_j2000_wxyz=tuple(rbk.MRP2EP(chaser.sigma_BN)),
            target_angular_velocity_body_rad_s=tuple(target.omega_BN_B),chaser_angular_velocity_body_rad_s=tuple(chaser.omega_BN_B),
            applied_motor_force_body_n=tuple(float(self._stepper.actuator_force[i]) for i in self._force_indices),
            applied_wheel_torque_xyz_nm=tuple(float(self._stepper.actuator_force[i]) for i in self._wheel_indices),
            contacts=len(self._stepper.contact_snapshot()),external_state_writes=self._stepper.external_writes)

    def _measure(self,sequence):
        if self._camera_sensor is None:
            self._camera_sensor=RenderedCameraSensor(self,self.task.cameras)
        images,private=self._camera_sensor.capture(self._time_ns)
        optical_q=[]
        for metadata in private:
            flu=np.asarray(metadata['extrinsics']['c_NC']).reshape(3,3)
            optical_q.append(tuple(Rotation.from_matrix(flu@FLU_TO_OPTICAL.T).as_quat()[[3,0,1,2]]))
        capture=VisionCaptureEvidence(sequence=sequence,sample_time_ns=self._time_ns,
            renderer_session_id=images[0].renderer_session_id,source_frame_id=images[0].source_frame_id,
            camera_ids=tuple(image.camera_id for image in images),
            calibration_sha256=tuple(camera.fingerprint() for camera in self.task.cameras),
            image_sha256=tuple(image.png_sha256 for image in images),
            camera_origins_j2000_m=tuple(tuple(metadata['extrinsics']['position_N_m']) for metadata in private),
            camera_orientations_j2000_from_optical_wxyz=tuple(optical_q))
        self._scorer.add(capture)
        self._truth_stream.write(capture.model_dump_json()+'\n')
        return VisionPoseObservation(episode_id=self.episode_id,sequence=sequence,sample_time_ns=self._time_ns,
            delivery_time_ns=self._time_ns,images=images)

    def step(self,action):
        if self._simulation is None or self._observation is None:
            raise RuntimeError('reset must precede a pose estimate')
        if self.evaluation()['outcome']!='incomplete':
            raise RuntimeError('episode has terminated; reset before another estimate')
        action.validate_for(self._observation,self.task)
        estimate=VisionPoseEstimateEvidence(sequence=action.observation_sequence,estimate_time_ns=action.estimate_time_ns,
            reference_frame=action.reference_frame,image_sha256=tuple(image.png_sha256 for image in self._observation.images),pose=action.pose)
        self._scorer.add(estimate)
        self._truth_stream.write(estimate.model_dump_json()+'\n')
        # All original physical motor commands remain at zero. Estimates do
        # not control bodies, write state or stabilize a hidden target pose.
        self._time_ns+=self.task.decision_period_ns
        self._simulation.ConfigureStopTime(self._time_ns)
        self._simulation.ExecuteSimulation()
        self._observation=self._measure(self._observation.sequence+1)
        return self._observation

    def provenance(self):
        value=super().provenance()
        value.update(observation_model='actual calibrated UE RGB PNG; no target telemetry or runtime poses in method inputs',
            actuation_model='passive free-flight pose evaluation; translation and wheel motor commands remain zero',
            initial_attitudes=self.initial_attitudes,
            target_cooperation='P: known passive markings only; no active alignment, telemetry or communication',
            estimate_clock='image exposure and estimate at t=0,...,duration-dt; instantaneous pinhole, zero simulated delivery delay',
            renderer_sensor=json.loads((self._camera_sensor.output/'sensor.json').read_text('utf-8')))
        return value

    def close(self):
        try:
            if self._camera_sensor is not None:
                self._camera_sensor.close()
                self._camera_sensor=None
        finally:
            super().close()
