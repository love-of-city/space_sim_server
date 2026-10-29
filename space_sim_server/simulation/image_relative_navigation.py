"""Convert an accepted image pose into a near_field.RelativeEKF position packet.

No diagnostic capture extrinsics, target truth, lighting or prior target pose
are read. Uncertainty must be supplied explicitly by the integration layer;
reprojection pixels are not a position covariance in square metres.
"""
import numpy as np

from simulation.cooperative_pointing import _rotation, _vector


def _unsigned_integer(value, name):
    if isinstance(value, str) and value.isascii() and value.isdecimal():
        return int(value)
    if isinstance(value, (int, np.integer)) and not isinstance(value, bool) and value >= 0:
        return int(value)
    raise ValueError(f'{name} must be a nonnegative integer or decimal string')


class ImageRelativeNavigation:
    def __init__(self, geometry, session_id, *, max_age_s=.35,
                 allow_cooperative_attitude=False):
        if not isinstance(session_id, str) or not session_id:
            raise ValueError('an explicit session ID is required')
        if not np.isfinite(max_age_s) or max_age_s <= 0:
            raise ValueError('maximum image age must be positive')
        self.geometry = geometry
        self.session = session_id
        self.max_age_ns = round(max_age_s*1e9)
        self.last_frame = None
        self.last_stamp = None
        self.allow_cooperative_attitude = bool(allow_cooperative_attitude)

    def convert(self, packet, *, now_ns, joint_position_rad, kinematics_stamp_ns,
                service_rotation_n_from_body, service_attitude_stamp_ns,
                position_covariance_n_m2):
        """Require arm and service transforms AT THE IMAGE EPOCH.

        ``position_covariance_n_m2`` is the total covariance of the converted
        body-origin relative position, including installation, attitude and
        encoder uncertainty as applicable. No unvalidated default is invented.
        A caller with asynchronous transforms must interpolate/propagate them
        before calling, preserving their provenance outside this adapter.
        """
        if packet.get('session_id') != self.session:
            raise ValueError('session_mismatch')
        expected = [self.geometry.config['camera_ids'][eye] for eye in ('left','right')]
        if packet.get('camera_ids') != expected:
            raise ValueError('wrong_camera_pair')
        frame = _unsigned_integer(packet['source_frame_id'], 'frame ID')
        stamp = _unsigned_integer(packet['sim_time_ns'], 'image timestamp')
        now = _unsigned_integer(now_ns, 'control timestamp')
        if stamp > now or now-stamp > self.max_age_ns:
            raise ValueError('stale_or_future_image')
        if self.last_frame is not None and (frame <= self.last_frame or stamp <= self.last_stamp):
            raise ValueError('duplicate_or_out_of_order_image')
        if (_unsigned_integer(kinematics_stamp_ns, 'kinematics timestamp') != stamp
                or _unsigned_integer(service_attitude_stamp_ns, 'attitude timestamp') != stamp):
            raise ValueError('image_epoch_transforms_required')
        measurement = packet['measurement']
        aided = (self.allow_cooperative_attitude
                 and measurement.get('route') == 'cooperative_attitude_stereo_translation'
                 and measurement.get('attitude_source') == 'same_epoch_cooperative_telemetry'
                 and _unsigned_integer(measurement.get('attitude_stamp_ns'), 'cooperative attitude timestamp') == stamp
                 and measurement.get('independent_image_attitude') is False)
        if measurement.get('status') != 'ok' or (measurement.get('route') != 'joint_multicamera' and not aided):
            raise ValueError('image_pose_not_accepted')
        used = measurement['used_ids']
        for eye in ('left','right'):
            ids = used.get(eye, [])
            if (not ids or len(set(ids)) != len(ids)
                    or any(str(i) not in self.geometry.config['layout']['markers'] for i in ids)):
                raise ValueError('invalid_image_marker_correspondences')
        pose = measurement['pose_target_to_left']
        image_rotation = _rotation(pose['R'])
        image_translation = _vector(pose['t_m'], 'image translation')
        service = _rotation(service_rotation_n_from_body)
        covariance = np.asarray(position_covariance_n_m2, float)
        if (covariance.shape != (3,3) or not np.isfinite(covariance).all()
                or not np.allclose(covariance,covariance.T,atol=1e-14,rtol=0)
                or np.linalg.eigvalsh(covariance).min() <= 0):
            raise ValueError('positive_definite_position_covariance_required')
        camera_position, camera_rotation = self.geometry.cameras_service(joint_position_rad)['left']
        relative = service@(camera_position+camera_rotation@image_translation)
        target_rotation = service@camera_rotation@image_rotation
        self.last_frame, self.last_stamp = frame, stamp
        return dict(source='stereo', stamp=stamp*1e-9, z=relative, R=covariance.copy(),
            rotation_n_from_target=target_rotation, frame='J2000',
            reference='target_body_origin_minus_service_body_origin',
            session_id=self.session, source_frame_id=frame,
            quality=dict(route=measurement['route'], used_ids={e:list(used[e]) for e in ('left','right')},
                         reprojection_error_px=measurement.get('reprojection_error_px')),
            scope='navigation measurement only; not a visual-readiness or mission-release permit')
