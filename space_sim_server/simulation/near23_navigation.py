"""Exposure-bound eye-in-hand image poses for the COM navigation filter.

The bridge uses image measurements, static calibration and joint encoders.
It never reads spacecraft truth or rotates a measurement using its own EKF
posterior. Assembly-COM lever arms belong to PoseNavigation's delayed replay.
"""
from __future__ import annotations

from copy import deepcopy
from collections import OrderedDict
import hashlib
from pathlib import Path

import numpy as np

from simulation.end_stereo_pointing import EndStereoGeometry
from simulation.pose_navigation import log, skew
from space_arm_platform.stereo_capture_packet import capture_packet
from space_arm_platform.visual_navigation import (
    NEAR23_BODY_PROFILE, POSE_ERROR_ORDER, calibration_id, covariance,
)


def _prior(value):
    a = np.asarray(value, float)
    if (a.shape != (6, 6) or not np.isfinite(a).all()
            or not np.allclose(a, a.T, atol=1e-14, rtol=0)
            or np.linalg.eigvalsh(a)[0] < -1e-14):
        raise ValueError('extrinsic/encoder covariance must be positive semidefinite 6x6')
    return a.copy()


class Near23NavigationBridge:
    def __init__(self, model_path, config, *, joint_covariance_rad2,
                 mount_covariance, uncertainty_source):
        """Require explicit uncertainty assumptions; zeros mean ideal simulation.

        Joint covariance: bottom_joint1..6, rad². Mount covariance order:
        [position_m, left_multiplicative_rotation_rad], in the gripper frame.
        The pixel prior is supplied by the accepted joint image solver.
        """
        if not isinstance(uncertainty_source, str) or not uncertainty_source.strip():
            raise ValueError('name the encoder/mount uncertainty source')
        self.config = deepcopy(config)
        self.calibration_id = calibration_id(config)
        self.model_sha256 = hashlib.sha256(Path(model_path).read_bytes()).hexdigest()
        self.geometry = EndStereoGeometry(model_path, self.config)
        self.joint_covariance = _prior(joint_covariance_rad2)
        self.mount_covariance = _prior(mount_covariance)
        self.uncertainty_source = uncertainty_source
        self.encoders = OrderedDict()

    def remember_encoders(self, sample_time_ns, joint_position_rad):
        """Retain exact physical exposure epochs; never substitute latest joints."""
        epoch = int(sample_time_ns)
        q = np.asarray(joint_position_rad, float)
        if epoch < 0 or q.shape != (6,) or not np.isfinite(q).all():
            raise ValueError('invalid timestamped capture encoders')
        if self.encoders and epoch < next(reversed(self.encoders)):
            raise ValueError('encoder clock moved backwards')
        if epoch in self.encoders and not np.array_equal(q, self.encoders[epoch]):
            raise ValueError('encoder sample changed at the same epoch')
        self.encoders[epoch] = q.copy()
        while self.encoders and next(iter(self.encoders)) < epoch-1_000_000_000:
            self.encoders.popitem(last=False)

    def image_result(self, measurement):
        """Consume the existing vision-service result with its original identity."""
        epoch = int(measurement['sim_time_ns'])
        result = self.packet(measurement, mission_id=measurement['mission_id'],
            session_id=measurement['session_id'], source_frame_id=measurement['source_frame_id'],
            sim_time_ns=epoch, encoder_time_ns=epoch,
            joint_position_rad=self.encoders.get(epoch, []))
        # Do not relabel an image produced under another calibration.
        if measurement.get('calibration_id') != self.calibration_id:
            for key in ('pose_target_to_service', 'pose_covariance'):
                result.pop(key, None)
            result.update(status='rejected', ambiguous=True, reason='image calibration mismatch')
        if epoch not in self.encoders:
            result.update(status='rejected', ambiguous=True, reason='no encoders at image exposure')
        if 'capture_timing' in measurement:
            result['capture_timing'] = deepcopy(measurement['capture_timing'])
        return result

    def _extrinsic_covariance(self, q):
        jacobian = np.empty((6, 6))
        for axis in range(6):
            delta = np.zeros(6)
            delta[axis] = 1e-6
            plus_p, plus_r = self.geometry.cameras_service(q+delta)['left']
            minus_p, minus_r = self.geometry.cameras_service(q-delta)['left']
            jacobian[:, axis] = np.r_[(plus_p-minus_p)/2e-6, log(plus_r@minus_r.T)/2e-6]
        _, grip_rotation = self.geometry.chain.forward(q)
        transform = np.zeros((6, 6))
        transform[:3, :3] = transform[3:, 3:] = grip_rotation
        return (jacobian@self.joint_covariance@jacobian.T
                + transform@self.mount_covariance@transform.T)

    def packet(self, measurement, *, mission_id, session_id, source_frame_id,
               sim_time_ns, encoder_time_ns, joint_position_rad):
        """Export a new invalid packet on loss; never carry forward an old pose."""
        packet = dict(schema='near23-body-navigation/1', quality_profile=NEAR23_BODY_PROFILE,
            mission_id=mission_id, session_id=session_id, calibration_id=self.calibration_id,
            source_frame_id=str(source_frame_id), sim_time_ns=str(sim_time_ns),
            status='rejected', ambiguous=True)
        try:
            if (any(not isinstance(x, str) or not x for x in (mission_id, session_id))
                    or not str(source_frame_id)):
                raise ValueError('nonempty mission/session/frame required')
            exposure, encoder_epoch = int(sim_time_ns), int(encoder_time_ns)
            if exposure < 0 or encoder_epoch != exposure:
                raise ValueError('image requires joint encoders at its exposure epoch')
            packet['sim_time_ns'] = str(exposure)
            accepted = capture_packet(measurement, self.config, session_id=session_id,
                source_frame_id=source_frame_id, sim_time_ns=exposure)
            if not accepted['valid']:
                raise ValueError(accepted['reason'])
            q = np.asarray(joint_position_rad, float)
            if q.shape != (6,) or not np.isfinite(q).all():
                raise ValueError('six finite capture arm encoder positions required')
            pc, rc = self.geometry.cameras_service(q)['left']
            pose = measurement['pose_target_to_left']
            t, r = np.asarray(pose['t_m'], float), np.asarray(pose['R'], float)
            c = measurement['pose_covariance']
            if (c['frame'] != 'left_optical' or c['order'] != POSE_ERROR_ORDER
                    or c['rotation_error'] != 'left_multiplicative'):
                raise ValueError('unsupported optical pose covariance convention')
            pixel_covariance = covariance(c['matrix'], 6)
            image_transform = np.zeros((6, 6))
            image_transform[:3, :3] = image_transform[3:, 3:] = rc
            extrinsic_transform = np.eye(6)
            extrinsic_transform[:3, 3:] = -skew(rc@t)
            extrinsic = self._extrinsic_covariance(q)
            noise = (image_transform@pixel_covariance@image_transform.T
                     + extrinsic_transform@extrinsic@extrinsic_transform.T)
            noise = covariance(.5*(noise+noise.T), 6)
            used = measurement['used_ids']
            packet.update(status='ok', ambiguous=False, reason='exposure-time image + encoder FK',
                route='joint_multicamera',
                pose_target_to_service=dict(t_m=(pc+rc@t).tolist(), R=(rc@r).tolist()),
                pose_covariance=dict(matrix=noise.tolist(), order=POSE_ERROR_ORDER.copy(),
                    frame='service_body', rotation_error='left_multiplicative',
                    pixel_prior=deepcopy(c), encoder_mount_prior=self.uncertainty_source),
                optical_depth_sigma_m=float(np.sqrt(pixel_covariance[2, 2])),
                exposure_geometry=dict(sample_time_ns=str(encoder_epoch),
                    calibration_id=self.calibration_id, model_sha256=self.model_sha256,
                    joint_position_rad=q.tolist(), camera_position_service_body_m=pc.tolist(),
                    camera_rotation_service_from_optical=rc.tolist(),
                    camera_extrinsic_covariance_service_body=extrinsic.tolist(),
                    joint_covariance_rad2=self.joint_covariance.tolist(),
                    mount_covariance=self.mount_covariance.tolist()),
                counts=dict(used_markers_left=len(used['left']), used_markers_right=len(used['right']),
                    common_markers=len(set(used['left']) & set(used['right']))),
                used_ids=deepcopy(used), positive_depth=True,
                minimum_marker_side_px=deepcopy(measurement['minimum_marker_side_px']),
                reprojection_error_px=deepcopy(measurement['reprojection_error_px']),
                max_corner_error_px=deepcopy(measurement['maximum_corner_error_px']))
        except (KeyError, TypeError, ValueError, OverflowError, np.linalg.LinAlgError) as exc:
            packet['reason'] = str(exc)
        return packet


def relative_navigation_output(navigation):
    """Consumer output for guidance, NOT another independent EKF observation."""
    s = navigation.state
    h = np.zeros((6, 24))
    h[:, :6], h[:, 12:18] = -np.eye(6), np.eye(6)
    return dict(schema='two-spacecraft-relative-navigation/1', sample_time_ns=str(s['time_ns']),
        frame='J2000', reference='target_assembly_com_minus_service_assembly_com',
        source='navigation_posterior',
        position_m=(s['p'][1]-s['p'][0]).tolist(),
        velocity_m_s=(s['v'][1]-s['v'][0]).tolist(),
        position_velocity_covariance=(h@s['covariance']@h.T).tolist())
