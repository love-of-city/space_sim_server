"""Image translation with explicitly supplied cooperative attitude telemetry.

This is NOT independent six-DOF image PnP. Rotation comes from same-epoch
cooperative attitude and the measured camera installation/encoders. Translation
is solved from the two real images and the target model, without GNSS position,
previous target translation or renderer extrinsics. Native pixel/depth/complete
marker checks remain. Loss of telemetry must reject this route at integration.
"""
import cv2
import numpy as np
from scipy.optimize import least_squares

from .multicamera_pose import MultiCameraPoseEstimator


class CooperativeStereoTranslation(MultiCameraPoseEstimator):
    route = 'cooperative_attitude_stereo_translation'

    def estimate_aided(self, left_blob, right_blob, *, rotation_target_to_left,
                       attitude_stamp_ns, image_stamp_ns):
        if attitude_stamp_ns != image_stamp_ns:
            raise ValueError('cooperative attitude must be transported to the image epoch')
        rotation = np.asarray(rotation_target_to_left, float)
        if (rotation.shape != (3, 3) or not np.isfinite(rotation).all()
                or not np.allclose(rotation.T@rotation, np.eye(3), atol=1e-8)
                or not np.isclose(np.linalg.det(rotation), 1., atol=1e-8)):
            raise ValueError('proper cooperative relative attitude required')
        raw = [self.detect(left_blob), self.detect(right_blob)]
        output = dict(detected_corners_px={e: {str(k): c.tolist() for k, c in obs.items()}
                      for e, obs in zip(('left', 'right'), raw)})
        observations = [self._filter(obs) for obs in raw]
        try:
            if any(len(obs) < self.minimum for obs in observations):
                raise ValueError('both eyes need complete resolved markers')
            used = {e: sorted(obs) for e, obs in zip(('left', 'right'), observations)}
            pairs = {eye: self.geometry._points(observations[eye], used[e])
                     for eye, e in enumerate(('left', 'right'))}
            # Bearing constraints P*(R*X+t)=0 provide a position-only seed.
            rows, values = [], []
            for eye, (xyz, uv) in pairs.items():
                normalized = cv2.undistortPoints(uv.reshape(-1, 1, 2),
                    self.geometry.k[eye], self.geometry.d[eye]).reshape(-1, 2)
                rig = np.eye(3) if eye == 0 else self.geometry.rig_r
                offset = np.zeros(3) if eye == 0 else self.geometry.rig_t
                points = (xyz@rotation.T)@rig.T+offset
                for (u, v), p in zip(normalized, points):
                    bearing = np.array([[1., 0, -u], [0, 1., -v]])
                    rows.extend(bearing@rig); values.extend(-bearing@p)
            translation, _, rank, _ = np.linalg.lstsq(rows, values, rcond=None)
            if rank != 3: raise ValueError('translation is unobservable')

            def residual(t):
                return np.concatenate([(self.geometry._project(xyz,
                    rotation if eye == 0 else self.geometry.rig_r@rotation,
                    t if eye == 0 else self.geometry.rig_r@t+self.geometry.rig_t, eye)-uv).ravel()
                    for eye, (xyz, uv) in pairs.items()])

            fit = least_squares(residual, translation, loss='soft_l1', f_scale=1., max_nfev=100)
            # Remove whole markers using the unchanged per-corner limit.
            for eye, e in enumerate(('left', 'right')):
                xyz, uv = pairs[eye]
                rr = rotation if eye == 0 else self.geometry.rig_r@rotation
                tt = fit.x if eye == 0 else self.geometry.rig_r@fit.x+self.geometry.rig_t
                pixel = np.linalg.norm(self.geometry._project(xyz, rr, tt, eye)-uv, axis=1)
                used[e] = [i for n, i in enumerate(used[e])
                           if pixel[4*n:4*n+4].max() <= self.geometry.max_corner]
                if len(used[e]) < self.minimum: raise ValueError('too few complete aided marker inliers')
            pairs = {eye: self.geometry._points(observations[eye], used[e])
                     for eye, e in enumerate(('left', 'right'))}
            fit = least_squares(residual, fit.x, method='lm', max_nfev=100)
            if not fit.success or not np.isfinite(fit.x).all():
                raise ValueError('translation refinement failed')
            chosen = self._accept(self.route, rotation, fit.x, pairs, used)
            chosen.update(attitude_source='same_epoch_cooperative_telemetry',
                          attitude_stamp_ns=image_stamp_ns,
                          translation_source='two_image_reprojection',
                          independent_image_attitude=False)
        except (ValueError, cv2.error, np.linalg.LinAlgError) as error:
            chosen = self._rejected(self.route, error)
        output['measurement'] = chosen
        return output
