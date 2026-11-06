"""Causal marked-target stereo tracking for the slow approach baseline.

Current image corners are mandatory. A short-lived previous image pose is a
regularization prior, never a replacement measurement or a simulator input.
The general per-frame stereo estimator and its published baselines are unchanged.
"""
import cv2
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation


class ApproachStereoTracker:
    def __init__(self, geometry):
        self.geometry = geometry
        self.previous = None
        self.missed = 0
        params = cv2.aruco.DetectorParameters()
        params.adaptiveThreshWinSizeMin = 3
        params.adaptiveThreshWinSizeMax = 31
        params.adaptiveThreshWinSizeStep = 2
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_CONTOUR
        self.detector = cv2.aruco.ArucoDetector(
            cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50), params)

    def estimate(self, left_png, right_png):
        try:
            observations = []
            for encoded in (left_png, right_png):
                image = cv2.imdecode(np.frombuffer(encoded, np.uint8), cv2.IMREAD_GRAYSCALE)
                if image is None or image.shape[::-1] != self.geometry.image_size:
                    raise ValueError('tracking image dimensions differ from calibration')
                corners, ids, _ = self.detector.detectMarkers(image)
                ids = [] if ids is None else ids.reshape(-1).tolist()
                if len(ids) != len(set(ids)):
                    raise ValueError('duplicate image marker identities')
                observations.append({int(i): np.asarray(c).reshape(4, 2)
                                     for i, c in zip(ids, corners)})
            return self.estimate_from_detections(*observations)
        except (ValueError, cv2.error) as error:
            return self._lost(str(error))

    def _lost(self, reason):
        self.missed += 1
        if self.missed >= 2:
            self.previous = None
        result = self.geometry._result()
        result['reason'] = reason
        return result

    def estimate_from_detections(self, left, right):
        """Fit only this stereo pair; a lost pair never emits a predicted pose."""
        geometry = self.geometry
        try:
            observations = [{int(i): geometry._array(corners, (4, 2), 'image corners')
                             for i, corners in eye.items() if i in geometry.markers}
                            for eye in (left, right)]
            if self.previous is None:
                result = geometry.estimate_from_detections(*observations)
                if result['status'] != 'ok':
                    return self._lost(result['reason'])
                pose = result['pose_target_to_left']
                self.previous = (np.asarray(pose['R']), np.asarray(pose['t_m']))
                self.missed = 0
                return result
            used = [sorted(eye) for eye in observations]
            common = sorted(set(used[0]) & set(used[1]))
            if any(len(ids) < geometry.minimum for ids in (*used, common)):
                raise ValueError('current stereo pair lacks complete common markers')
            # Keep the original positive-depth/parallax/epipolar gates.
            geometry._triangulate(*observations, common)
            pairs = [geometry._points(eye, ids) for eye, ids in zip(observations, used)]
            previous_r, previous_t = self.previous
            # Slow-approach engineering prior, in metres and radians. It is
            # deliberately weaker after one missing pair and discarded after
            # two. No task truth or scorer tolerances set these weights.
            inflation = np.sqrt(1. + self.missed)
            position_sigma, rotation_sigma = .08 * inflation, .03 * inflation

            def pixels(parameters):
                r = Rotation.from_rotvec(parameters[3:]).as_matrix() @ previous_r
                t = parameters[:3]
                errors = []
                for eye, (xyz, uv) in enumerate(pairs):
                    rr, tt = (r, t) if eye == 0 else (geometry.rig_r @ r, geometry.rig_r @ t + geometry.rig_t)
                    errors.append((geometry._project(xyz, rr, tt, eye) - uv).reshape(-1))
                return np.concatenate(errors)

            def residual(parameters):
                return np.r_[pixels(parameters) / geometry.corner_sigma,
                             (parameters[:3] - previous_t) / position_sigma,
                             parameters[3:] / rotation_sigma]

            fit = least_squares(residual, np.r_[previous_t, np.zeros(3)],
                                loss='soft_l1', f_scale=1., max_nfev=100,
                                ftol=1e-9, xtol=1e-9, gtol=1e-9)
            if not fit.success or not np.isfinite(fit.x).all():
                raise ValueError('no finite causal stereo solution')
            r = Rotation.from_rotvec(fit.x[3:]).as_matrix() @ previous_r
            t = fit.x[:3]
            flat = pixels(fit.x)
            normalized = float(flat @ flat) / (geometry.corner_sigma**2 * max(1, len(flat)-6))
            if normalized > 25:
                raise ValueError('current rigid layout inconsistent with pixel uncertainty')
            result = geometry._result()
            offset = 0
            for index, (eye, (xyz, _)) in enumerate(zip(('left', 'right'), pairs)):
                rr, tt = (r, t) if index == 0 else (geometry.rig_r @ r, geometry.rig_r @ t + geometry.rig_t)
                if np.any((xyz @ rr.T + tt)[:, 2] <= 0):
                    raise ValueError('current tracked points have nonpositive depth')
                errors = np.linalg.norm(flat[offset:offset+2*len(xyz)].reshape(-1, 2), axis=1)
                offset += 2*len(xyz)
                rms, peak = float(np.sqrt(np.mean(errors**2))), float(errors.max())
                if rms > geometry.max_rms or peak > geometry.max_corner:
                    raise ValueError('current tracked pixel residual exceeds geometric gates')
                result['reprojection_error_px'][eye] = rms
                result['max_corner_error_px'][eye] = peak
                result['detected_ids'][eye] = result['used_ids'][eye] = used[index]
                result['detected_corners_px'][eye] = {str(i): c.tolist() for i, c in observations[index].items()}
                result['counts']['detected_markers_' + eye] = len(used[index])
                result['counts']['used_markers_' + eye] = len(used[index])
                result['counts']['used_corners_' + eye] = len(xyz)
            result['counts']['common_markers'] = len(common)
            q = Rotation.from_matrix(r).as_quat()[[3, 0, 1, 2]]
            if q[0] < 0:
                q = -q
            result.update(status='ok', reason='current stereo pixels with causal slow-motion prior',
                          normalized_rigid_residual=normalized,
                          pose_target_to_left=dict(R=r.tolist(), t_m=t.tolist(), quaternion_wxyz=q.tolist()))
            self.previous = (r, t)
            self.missed = 0
            return result
        except (ValueError, cv2.error, np.linalg.LinAlgError) as error:
            return self._lost(str(error))
