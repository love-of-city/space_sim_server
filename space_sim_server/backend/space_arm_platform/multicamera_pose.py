"""Image-only comparison of calibrated mono and three two-camera pose routes.

All routes share detected full marker corners and the same calibration/layout.
Joint reprojection uses each eye's own model correspondences, so common IDs are
not required. Triangulation explicitly requires common IDs. No scene state,
renderer extrinsics or preceding target pose is accepted by this interface.
"""
from __future__ import annotations

import cv2
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

from .stereo_pose import StereoPoseEstimator
from .visual_navigation import POSE_ERROR_ORDER


class MultiCameraPoseEstimator:
    def __init__(self, calibration, layout, *, minimum_marker_side_px=12., min_markers_per_eye=1):
        self.geometry = StereoPoseEstimator(calibration, layout)
        # Keep the smaller adaptive windows and add larger ones. Close markers
        # under uneven light can have low local contrast across a wide border;
        # the OpenCV default maximum of 23 px misses otherwise decodable IDs.
        # This changes detection only, not the raw image or pose quality gates.
        parameters = self.geometry.detector.getDetectorParameters()
        parameters.adaptiveThreshWinSizeMax = 103
        self.geometry.detector.setDetectorParameters(parameters)
        if min_markers_per_eye not in (1, 2):
            raise ValueError('min_markers_per_eye must be 1 or 2')
        self.minimum = min_markers_per_eye
        self.minimum_side = float(minimum_marker_side_px)
        if not np.isfinite(self.minimum_side) or self.minimum_side < 0:
            raise ValueError('invalid marker side threshold')

    def detect(self, blob):
        im = cv2.imdecode(np.frombuffer(blob, np.uint8), cv2.IMREAD_GRAYSCALE)
        if im is None or tuple(im.shape[1::-1]) != self.geometry.image_size:
            raise ValueError('image dimensions do not match calibration')
        corners, ids, _ = self.geometry.detector.detectMarkers(im)
        return {} if ids is None else {int(i): np.asarray(c).reshape(4, 2)
                                      for i, c in zip(ids.ravel(), corners)}

    def _filter(self, observations):
        out = {}
        for key, values in observations.items():
            i = int(key)
            c = np.asarray(values, dtype=float)
            if i not in self.geometry.markers:
                continue
            if c.shape != (4, 2) or not np.isfinite(c).all():
                raise ValueError('invalid measured marker corners')
            if np.min(np.linalg.norm(c - np.roll(c, 1, axis=0), axis=1)) >= self.minimum_side:
                out[i] = c
        return out

    def _eye_candidates(self, obs, eye):
        if len(obs) < self.minimum:
            raise ValueError(f'fewer than {self.minimum} complete resolved markers')
        try:
            used, r, t = self.geometry._pnp(obs, eye)
        except (ValueError, cv2.error):
            used = []
        if len(used) < self.minimum:
            # EPNP minimal samples can fail for near-coplanar multi-scale
            # layouts. Try deterministic nonminimal SQPnP seeds, using the
            # same complete-marker pixel/depth gates on ALL observations.
            ids = sorted(obs)
            xyz_all, uv_all = self.geometry._points(obs, ids)
            subsets = [ids]
            subsets.extend([i, j] for n, i in enumerate(ids) for j in ids[n+1:])
            fallback = []
            for subset in subsets:
                xyz_seed, uv_seed = self.geometry._points(obs, subset)
                origin_seed = xyz_seed.mean(axis=0)
                try:
                    solutions = cv2.solvePnPGeneric(xyz_seed-origin_seed, uv_seed,
                        self.geometry.k[eye], self.geometry.d[eye], flags=cv2.SOLVEPNP_SQPNP)
                except cv2.error:
                    continue
                for rv, tv in zip(solutions[1], solutions[2]):
                    rr = cv2.Rodrigues(rv)[0]; tt = tv.reshape(3)-rr@origin_seed
                    errors = np.linalg.norm(self.geometry._project(xyz_all, rr, tt, eye)-uv_all, axis=1)
                    depth = (xyz_all@rr.T+tt)[:, 2]
                    kept = [i for n, i in enumerate(ids) if np.all(depth[4*n:4*n+4] > 0)
                            and errors[4*n:4*n+4].max() <= self.geometry.max_corner]
                    if len(kept) >= self.minimum:
                        residual = np.concatenate([errors[4*n:4*n+4] for n, i in enumerate(ids) if i in kept])
                        fallback.append((-len(kept), float(np.mean(residual**2)), kept, rr, tt))
            if not fallback:
                raise ValueError(f'fewer than {self.minimum} complete marker inliers')
            _, _, used, r, t = min(fallback, key=lambda f: f[:2])
        xyz, uv = self.geometry._points(obs, used)
        origin = xyz.mean(axis=0)
        seeds = [(r, t)]
        if np.linalg.matrix_rank(xyz - origin, tol=1e-8) == 2:
            x = xyz[1] - xyz[0]; x /= np.linalg.norm(x)
            y = xyz[3] - xyz[0]; y -= x * np.dot(x, y); y /= np.linalg.norm(y)
            basis = np.column_stack((x, y, np.cross(x, y)))
            local = (xyz - origin) @ basis
            local[:, 2] = 0.
            solutions = cv2.solvePnPGeneric(local, uv, self.geometry.k[eye], self.geometry.d[eye], flags=cv2.SOLVEPNP_IPPE)
            for rv, tv in zip(solutions[1], solutions[2]):
                rr = cv2.Rodrigues(rv)[0] @ basis.T
                seeds.append((rr, tv.reshape(3) - rr @ origin))
        else:
            solutions = cv2.solvePnPGeneric(xyz - origin, uv, self.geometry.k[eye], self.geometry.d[eye], flags=cv2.SOLVEPNP_SQPNP)
            for rv, tv in zip(solutions[1], solutions[2]):
                rr = cv2.Rodrigues(rv)[0]
                seeds.append((rr, tv.reshape(3) - rr @ origin))
        fits = []
        for rr, tt in seeds:
            rv, tv = cv2.solvePnPRefineLM(xyz, uv, self.geometry.k[eye], self.geometry.d[eye],
                cv2.Rodrigues(rr)[0], tt.reshape(3, 1).copy(),
                criteria=(cv2.TERM_CRITERIA_COUNT + cv2.TERM_CRITERIA_EPS, 100, 1e-11))
            rr, tt = cv2.Rodrigues(rv)[0], tv.reshape(3)
            if np.any((xyz @ rr.T + tt)[:, 2] <= 0):
                continue
            errors = np.linalg.norm(self.geometry._project(xyz, rr, tt, eye) - uv, axis=1)
            if np.sqrt(np.mean(errors**2)) <= self.geometry.max_rms and errors.max() <= self.geometry.max_corner:
                fits.append((float(np.sqrt(np.mean(errors**2))), rr, tt))
        if not fits:
            raise ValueError('no positive-depth mono solution within reprojection limits')
        fits.sort(key=lambda f: f[0])
        return used, fits

    @staticmethod
    def _ambiguous(fits):
        best = fits[0]
        return any(f[0] - best[0] < .05 and
                   (np.linalg.norm(Rotation.from_matrix(f[1] @ best[1].T).as_rotvec()) > np.deg2rad(5)
                    or np.linalg.norm(f[2] - best[2]) > .005) for f in fits[1:])

    @staticmethod
    def _rejected(route, reason):
        return dict(status='rejected', route=route, reason=str(reason), pose_target_to_left=None)

    def _left_pose(self, fit, eye):
        _, r, t = fit
        if eye:
            r, t = self.geometry.rig_r.T @ r, self.geometry.rig_r.T @ (t - self.geometry.rig_t)
        return r, t

    def _accept(self, route, r, t, point_pairs, used):
        errors = {}
        maximum = {}
        sides = {}
        for eye, (xyz, uv) in point_pairs.items():
            rr, tt = (r, t) if eye == 0 else (self.geometry.rig_r @ r, self.geometry.rig_r @ t + self.geometry.rig_t)
            if np.any((xyz @ rr.T + tt)[:, 2] <= 0):
                raise ValueError('negative camera depth')
            e = np.linalg.norm(self.geometry._project(xyz, rr, tt, eye) - uv, axis=1)
            rms = float(np.sqrt(np.mean(e**2)))
            if rms > self.geometry.max_rms or e.max() > self.geometry.max_corner:
                raise ValueError(f'eye {eye}: reprojection threshold exceeded')
            errors['left' if eye == 0 else 'right'] = rms
            maximum['left' if eye == 0 else 'right'] = float(e.max())
            corners = uv.reshape(-1, 4, 2)
            sides['left' if eye == 0 else 'right'] = float(np.min(
                np.linalg.norm(corners-np.roll(corners, 1, axis=1), axis=2)))
        result = dict(status='ok', route=route, reason='image quality and geometry accepted',
            pose_target_to_left=dict(R=r.tolist(), t_m=t.tolist()), used_ids=used,
            reprojection_error_px=errors,
            maximum_corner_error_px=maximum, minimum_marker_side_px=sides,
            positive_depth=True,
            reduced_redundancy=any(len(ids) < 2 for ids in used.values()))
        if route == 'joint_multicamera':
            # Use only the final retained corners, including different IDs in
            # the two eyes. This is a local pixel-noise prior, not a measured
            # accuracy certificate or a replacement for mount/encoder errors.
            pairs = [point_pairs[eye] for eye in (0, 1)]
            residual = np.concatenate([(self.geometry._project(xyz,
                r if eye == 0 else self.geometry.rig_r @ r,
                t if eye == 0 else self.geometry.rig_r @ t + self.geometry.rig_t, eye)-uv).ravel()
                for eye, (xyz, uv) in enumerate(pairs)])
            matrix, variance = self.geometry._pose_covariance(r, t, pairs, residual)
            result['pose_covariance'] = dict(matrix=matrix.tolist(), order=POSE_ERROR_ORDER.copy(),
                frame='left_optical', rotation_error='left_multiplicative',
                corner_variance_px2=variance, variance_inflation=self.geometry.covariance_inflation,
                calibration_status='engineering_pixel_prior_requires_sequence_validation')
        return result

    def estimate(self, left_bytes, right_bytes):
        return self.estimate_from_detections(self.detect(left_bytes), self.detect(right_bytes))

    def estimate_from_detections(self, left, right):
        observations = [self._filter(left), self._filter(right)]
        output = dict(detected_corners_px={eye: {str(k): np.asarray(v).tolist() for k, v in obs.items()}
                      for eye, obs in zip(('left', 'right'), (left, right))},
                      resolved_ids={eye: sorted(obs) for eye, obs in zip(('left', 'right'), observations)},
                      methods={})
        candidates, pairs, used = {}, {}, {}
        for eye, name in enumerate(('mono_left', 'mono_right')):
            try:
                cv2.setRNGSeed(26)
                ids, fits = self._eye_candidates(observations[eye], eye)
                candidates[eye] = fits
                pairs[eye] = self.geometry._points(observations[eye], ids)
                used['left' if eye == 0 else 'right'] = ids
                if self._ambiguous(fits):
                    raise ValueError('multiple refined mono branches are ambiguous')
                r, t = self._left_pose(fits[0], eye)
                output['methods'][name] = self._accept(name, r, t, {eye: pairs[eye]}, {str(eye): ids})
            except (ValueError, cv2.error, np.linalg.LinAlgError) as error:
                output['methods'][name] = self._rejected(name, error)
        for route in ('pnp_fusion', 'triangulation_registration', 'joint_multicamera'):
            try:
                if len(candidates) != 2:
                    raise ValueError(f'both eyes need at least {self.minimum} complete resolved inlier markers')
                if route == 'pnp_fusion':
                    if any(output['methods'][m]['status'] != 'ok' for m in ('mono_left', 'mono_right')):
                        raise ValueError('independent PnP not accepted in both eyes')
                    a, b = [self._left_pose(candidates[i][0], i) for i in (0, 1)]
                    disagreement = np.linalg.norm(Rotation.from_matrix(a[0] @ b[0].T).as_rotvec())
                    if disagreement > np.deg2rad(10) or np.linalg.norm(a[1] - b[1]) > .03:
                        raise ValueError('independent PnP poses disagree')
                    r = Rotation.from_matrix(np.stack((a[0], b[0]))).mean().as_matrix()
                    t = (a[1] + b[1]) / 2
                elif route == 'triangulation_registration':
                    common = sorted(set(used['left']) & set(used['right']))
                    if len(common) < self.minimum:
                        raise ValueError(f'triangulation needs at least {self.minimum} common complete markers')
                    r, t = self.geometry._triangulate(*observations, common)
                else:
                    # A corner displaced by partial occlusion can pass a mono
                    # fit by biasing that fit. Use a robust joint seed, reject
                    # the entire offending marker with the SAME pixel gate,
                    # then solve ordinary reprojection on the retained data.
                    robust_pairs=dict(pairs)
                    seed_r,seed_t=self._left_pose(candidates[0][0],0)
                    def robust_residual(x):
                        rr=cv2.Rodrigues(x[:3])[0];tt=x[3:]
                        return np.concatenate([(self.geometry._project(xyz,
                            rr if eye==0 else self.geometry.rig_r@rr,
                            tt if eye==0 else self.geometry.rig_r@tt+self.geometry.rig_t,eye)-uv).ravel()
                            for eye,(xyz,uv) in robust_pairs.items()])
                    robust=least_squares(robust_residual,np.r_[cv2.Rodrigues(seed_r)[0].ravel(),seed_t],
                        loss='soft_l1',f_scale=1.,max_nfev=150)
                    rr=cv2.Rodrigues(robust.x[:3])[0];tt=robust.x[3:]
                    joint_used={}
                    for eye,ids in enumerate((used['left'],used['right'])):
                        xyz,uv=pairs[eye]
                        errors=np.linalg.norm(self.geometry._project(xyz,rr if eye==0 else self.geometry.rig_r@rr,
                            tt if eye==0 else self.geometry.rig_r@tt+self.geometry.rig_t,eye)-uv,axis=1)
                        kept=[i for j,i in enumerate(ids) if errors[4*j:4*j+4].max()<=self.geometry.max_corner]
                        if len(kept)<self.minimum:raise ValueError('too few complete joint marker inliers')
                        joint_used['left' if eye==0 else 'right']=kept
                    pairs={eye:self.geometry._points(observations[eye],joint_used['left' if eye==0 else 'right']) for eye in (0,1)}
                    def residual(x):
                        rr = cv2.Rodrigues(x[:3])[0]; tt = x[3:]
                        return np.concatenate([(self.geometry._project(xyz,
                            rr if eye == 0 else self.geometry.rig_r @ rr,
                            tt if eye == 0 else self.geometry.rig_r @ tt + self.geometry.rig_t, eye) - uv).ravel()
                            for eye, (xyz, uv) in pairs.items()])
                    fits = []
                    for eye, eye_fits in candidates.items():
                        for seed in eye_fits:
                            rr, tt = self._left_pose(seed, eye)
                            fit = least_squares(residual, np.r_[cv2.Rodrigues(rr)[0].ravel(), tt], method='lm',
                                max_nfev=150, ftol=1e-10, xtol=1e-10, gtol=1e-10)
                            if fit.success and np.isfinite(fit.x).all():
                                fits.append((float(np.sqrt(np.mean(residual(fit.x)**2))),
                                             cv2.Rodrigues(fit.x[:3])[0], fit.x[3:]))
                    if not fits:
                        raise ValueError('joint refinement did not converge')
                    fits.sort(key=lambda f: f[0])
                    if self._ambiguous(fits):
                        raise ValueError('joint camera branches are ambiguous')
                    _, r, t = fits[0]
                output['methods'][route] = self._accept(route, r, t, pairs, joint_used if route=='joint_multicamera' else used)
            except (ValueError, cv2.error, np.linalg.LinAlgError) as error:
                output['methods'][route] = self._rejected(route, error)
        return output
