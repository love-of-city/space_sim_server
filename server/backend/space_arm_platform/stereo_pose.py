"""Image-only calibrated stereo ArUco baseline (optional vision dependency).

Requires opencv-contrib-python-headless==4.13.0.92, NumPy and SciPy. No
simulation state, renderer metadata or previous pose enters this estimator.
Frames follow OpenCV: +X right, +Y down, +Z forward. Extrinsics obey
X_right = R_right_from_left @ X_left + t_right_from_left_m; a parallel
right camera 0.12 m to the right therefore has t=[-0.12, 0, 0].

Calibration keys: K_left/K_right (3x3), D_left/D_right (OpenCV distortion),
R_right_from_left (3x3), t_right_from_left_m (3), image_size=[width,height].
Layout: {"dictionary":"DICT_4X4_50", "markers":{"0":[[x,y,z], ...]}}.
Each marker has four metre-valued corners in decoded marker order TL, TR,
BR, BL (canonical marker orientation, not sorted by image coordinates).
Different markers may occupy different planes. Rejected results never
carry a pose. One-marker stereo is available only by explicit opt-in and
is labelled degraded. This is an unlearned geometric baseline, not a
validated measurement or capture/control system.
"""
from __future__ import annotations

import cv2
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation


class _Rejected(ValueError):
    pass


class StereoPoseEstimator:
    def __init__(self, calibration: dict, layout: dict, *, min_markers_per_eye=2,
                 allow_single_marker=False, max_reprojection_error_px=2.5,
                 max_marker_reprojection_error_px=5.0,
                 min_triangulation_angle_deg=.1):
        self.k = [self._array(calibration.get("K_" + eye), (3, 3), "K_" + eye)
                  for eye in ("left", "right")]
        for k in self.k:
            if k[0, 0] <= 0 or k[1, 1] <= 0 or not np.allclose(k[2], [0, 0, 1]):
                raise ValueError("camera intrinsic matrix must have positive focal lengths and [0,0,1] bottom row")
        self.d = []
        for eye in ("left", "right"):
            d = np.asarray(calibration.get("D_" + eye), dtype=float).reshape(-1)
            if d.size not in (0, 4, 5, 8, 12, 14) or not np.isfinite(d).all():
                raise ValueError("invalid OpenCV distortion coefficients")
            self.d.append(d.copy())
        self.rig_r = self._array(calibration.get("R_right_from_left"), (3, 3), "stereo rotation")
        if not np.allclose(self.rig_r.T @ self.rig_r, np.eye(3), atol=1e-7) or not np.isclose(np.linalg.det(self.rig_r), 1):
            raise ValueError("stereo rotation must be a proper orthonormal rotation")
        self.rig_t = self._array(calibration.get("t_right_from_left_m"), (3,), "stereo translation")
        if np.linalg.norm(self.rig_t) < 1e-6:
            raise ValueError("stereo baseline must be nonzero")
        size = np.asarray(calibration.get("image_size"), dtype=float)
        if size.shape != (2,) or not np.isfinite(size).all() or np.any(size < 1) or np.any(size != np.round(size)):
            raise ValueError("image_size must contain positive integer width and height")
        self.image_size = tuple(size.astype(int))
        self.minimum = int(min_markers_per_eye)
        self.allow_single = bool(allow_single_marker)
        self.max_rms = float(max_reprojection_error_px)
        self.max_corner = float(max_marker_reprojection_error_px)
        self.min_angle = float(min_triangulation_angle_deg)
        if self.minimum < 2 or not all(np.isfinite(x) and x > 0 for x in (self.max_rms, self.max_corner, self.min_angle)):
            raise ValueError("invalid pose acceptance thresholds")
        if layout.get("dictionary", "DICT_4X4_50") != "DICT_4X4_50":
            raise ValueError("this baseline supports DICT_4X4_50 only")
        self.markers = {}
        for marker_id, corners in layout.get("markers", {}).items():
            marker_id = int(marker_id)
            if marker_id in self.markers or not 0 <= marker_id < 50:
                raise ValueError("marker ids must be unique and inside DICT_4X4_50")
            xyz = self._array(corners, (4, 3), "marker corners")
            singular = np.linalg.svd(xyz - xyz.mean(axis=0), compute_uv=False)
            if singular[1] < 1e-8 or singular[2] > max(singular[0], 1) * 1e-7:
                raise ValueError("each marker must have noncollinear coplanar corners")
            self.markers[marker_id] = xyz
        if not self.markers:
            raise ValueError("layout must contain known marker corners in metres")
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        params = cv2.aruco.DetectorParameters()
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        params.cornerRefinementWinSize = 3
        params.cornerRefinementMaxIterations = 50
        params.cornerRefinementMinAccuracy = .01
        self.detector = cv2.aruco.ArucoDetector(dictionary, params)

    @staticmethod
    def _array(value, shape, name):
        arr = np.asarray(value, dtype=float)
        if arr.shape != shape or not np.isfinite(arr).all():
            raise ValueError(f"{name} must be finite with shape {shape}")
        return arr.copy()

    @staticmethod
    def _result():
        return dict(status="rejected", detected_ids={"left": [], "right": []},
                    detected_corners_px={"left": {}, "right": {}},
                    used_ids={"left": [], "right": []}, pose_target_to_left=None,
                    reprojection_error_px={"left": None, "right": None},
                    counts={"detected_markers_left": 0, "detected_markers_right": 0,
                            "used_markers_left": 0, "used_markers_right": 0,
                            "used_corners_left": 0, "used_corners_right": 0,
                            "common_markers": 0}, reason="")

    def estimate(self, left_png: bytes, right_png: bytes) -> dict:
        """Decode actual images, detect canonical corners and estimate pose."""
        result = self._result()
        observations = []
        try:
            for eye, encoded in zip(("left", "right"), (left_png, right_png)):
                if not isinstance(encoded, (bytes, bytearray, memoryview)) or not encoded:
                    raise _Rejected(f"{eye}: invalid encoded image")
                image = cv2.imdecode(np.frombuffer(encoded, np.uint8), cv2.IMREAD_GRAYSCALE)
                if image is None:
                    raise _Rejected(f"{eye}: image decode failed")
                if image.shape[::-1] != self.image_size:
                    raise _Rejected(f"{eye}: image size disagrees with calibration")
                corners, ids, _ = self.detector.detectMarkers(image)
                ids = [] if ids is None else [int(i) for i in ids.reshape(-1)]
                result["detected_ids"][eye] = sorted(ids)
                result["detected_corners_px"][eye] = {str(i): np.asarray(c).reshape(4, 2).tolist() for i, c in zip(ids, corners)}
                result["counts"]["detected_markers_" + eye] = len(ids)
                if len(set(ids)) != len(ids):
                    raise _Rejected(f"{eye}: duplicate marker ids are ambiguous")
                observations.append({i: np.asarray(c).reshape(4, 2) for i, c in zip(ids, corners)})
            return self._estimate_from_detections(*observations)
        except (_Rejected, cv2.error, ValueError) as exc:
            result["reason"] = str(exc)
            return result

    def _points(self, detections, ids):
        return (np.concatenate([self.markers[i] for i in ids]),
                np.concatenate([detections[i] for i in ids]))

    def _project(self, xyz, rotation, translation, eye):
        return cv2.projectPoints(xyz, cv2.Rodrigues(rotation)[0], translation,
                                 self.k[eye], self.d[eye])[0].reshape(-1, 2)

    def _pnp(self, detections, eye):
        ids = sorted(detections.keys() & self.markers.keys())
        xyz, uv = self._points(detections, ids)
        origin, basis = np.zeros(3), np.eye(3)
        if np.linalg.matrix_rank(xyz - xyz.mean(axis=0), tol=1e-8) == 2:
            # OpenCV 4.13 solvePnPRansac uses EPNP minimal samples even
            # when flags=SQPNP. Its planar kernel is numerically sensitive
            # to the object plane's coordinate orientation (e.g. Y=const).
            # Express the SAME physical points in a proper local XY frame;
            # solve there, then convert the pose back to the target frame.
            # The first decoded marker defines stable right/down directions.
            origin = xyz.mean(axis=0)
            right = xyz[1] - xyz[0]
            right /= np.linalg.norm(right)
            down = xyz[3] - xyz[0]
            down -= right * np.dot(right, down)
            down /= np.linalg.norm(down)
            basis = np.column_stack((right, down, np.cross(right, down)))
            xyz = (xyz - origin) @ basis
            xyz[:, 2] = 0.0  # Remove only roundoff normal to an exact plane.
        candidates = []
        if len(ids) == 1:
            solved = cv2.solvePnPGeneric(xyz, uv, self.k[eye], self.d[eye], flags=cv2.SOLVEPNP_IPPE)
            if solved[0]:
                candidates = [(r.reshape(3), t.reshape(3)) for r, t in zip(solved[1], solved[2])]
            inliers = set(range(len(xyz)))
        else:
            ok, rvec, tvec, indices = cv2.solvePnPRansac(
                xyz, uv, self.k[eye], self.d[eye], iterationsCount=300,
                reprojectionError=self.max_corner, confidence=.999, flags=cv2.SOLVEPNP_SQPNP)
            if not ok or indices is None:
                raise _Rejected("PnP RANSAC did not find a pose")
            candidates = [(rvec.reshape(3), tvec.reshape(3))]
            inliers = set(indices.reshape(-1).tolist())
        if not candidates:
            raise _Rejected("PnP did not find a pose")
        rvec, tvec = min(candidates, key=lambda p: np.sum((self._project(xyz, cv2.Rodrigues(p[0])[0], p[1], eye) - uv)**2))
        errors = np.linalg.norm(self._project(xyz, cv2.Rodrigues(rvec)[0], tvec, eye) - uv, axis=1)
        # A marker is a unit: never retain three good corners from a bad marker.
        kept = [marker_id for j, marker_id in enumerate(ids)
                if all(4*j+k in inliers for k in range(4)) and np.max(errors[4*j:4*j+4]) <= self.max_corner]
        if not kept:
            raise _Rejected("PnP has no complete inlier markers")
        xyz, uv = self._points(detections, kept)
        xyz = (xyz - origin) @ basis
        rvec, tvec = cv2.solvePnPRefineLM(xyz, uv, self.k[eye], self.d[eye], rvec.copy(), tvec.copy())
        rotation = cv2.Rodrigues(rvec)[0] @ basis.T
        translation = tvec.reshape(3) - rotation @ origin
        return kept, rotation, translation

    def _triangulate(self, left, right, ids):
        xyz, uv_l = self._points(left, ids)
        _, uv_r = self._points(right, ids)
        nl = cv2.undistortPoints(uv_l.reshape(-1, 1, 2), self.k[0], self.d[0]).reshape(-1, 2)
        nr = cv2.undistortPoints(uv_r.reshape(-1, 1, 2), self.k[1], self.d[1]).reshape(-1, 2)
        ray_l = np.column_stack((nl, np.ones(len(nl))))
        ray_r = np.column_stack((nr, np.ones(len(nr)))) @ self.rig_r
        ray_l /= np.linalg.norm(ray_l, axis=1)[:, None]
        ray_r /= np.linalg.norm(ray_r, axis=1)[:, None]
        angles = np.rad2deg(np.arccos(np.clip(np.sum(ray_l*ray_r, axis=1), -1, 1)))
        if np.min(angles) < self.min_angle:
            raise _Rejected("stereo parallax is too small for an observable pose")
        homogeneous = cv2.triangulatePoints(np.column_stack((np.eye(3), np.zeros(3))),
                                           np.column_stack((self.rig_r, self.rig_t)), nl.T, nr.T)
        if np.any(np.abs(homogeneous[3]) < 1e-12):
            raise _Rejected("stereo triangulation is at infinity")
        points = (homogeneous[:3] / homogeneous[3]).T
        right_points = points @ self.rig_r.T + self.rig_t
        if not np.isfinite(points).all() or np.any(points[:, 2] <= 0) or np.any(right_points[:, 2] <= 0):
            raise _Rejected("stereo triangulation has negative camera depth")
        for eye, uv in [(0, uv_l), (1, uv_r)]:
            r, t = (np.eye(3), np.zeros(3)) if eye == 0 else (self.rig_r, self.rig_t)
            error = np.linalg.norm(self._project(points, r, t, eye) - uv, axis=1)
            if np.max(error) > self.max_corner:
                raise _Rejected("stereo epipolar/reprojection mismatch")
        x = xyz - xyz.mean(axis=0)
        y = points - points.mean(axis=0)
        u, _, vt = np.linalg.svd(x.T @ y)
        r = vt.T @ np.diag([1, 1, np.linalg.det(vt.T @ u.T)]) @ u.T
        t = points.mean(axis=0) - r @ xyz.mean(axis=0)
        extent = np.max(np.linalg.norm(x, axis=1))
        if np.max(np.linalg.norm(xyz @ r.T + t - points, axis=1)) > max(.01, .05*extent):
            raise _Rejected("triangulated corners disagree with known rigid layout")
        return r, t

    def estimate_from_detections(self, left: dict, right: dict) -> dict:
        """Fuse image-measured corners from a shared frontend (no state/truth).

        Inputs map decoded marker ID to four full-resolution optical pixels in
        TL/TR/BR/BL order. The same geometric acceptance gates as estimate() apply.
        """
        return self._estimate_from_detections(left, right)

    def _estimate_from_detections(self, left: dict, right: dict) -> dict:
        """Numerical solver seam; production observations come from estimate()."""
        result = self._result()
        eyes = ("left", "right")
        result["detected_ids"] = {eye: sorted(int(i) for i in obs) for eye, obs in zip(eyes, (left, right))}
        result["detected_corners_px"] = {eye: {str(i): np.asarray(c).tolist() for i, c in obs.items()}
                                         for eye, obs in zip(eyes, (left, right))}
        result["counts"].update(detected_markers_left=len(left), detected_markers_right=len(right))
        required = 1 if self.allow_single else self.minimum
        try:
            observations = []
            for eye, obs in zip(eyes, (left, right)):
                known = {int(i): self._array(p, (4, 2), "image corners") for i, p in obs.items() if i in self.markers}
                if len(known) < required:
                    raise _Rejected(f"{eye}: insufficient known complete markers")
                observations.append(known)
            pnp = [self._pnp(obs, i) for i, obs in enumerate(observations)]
            used = [p[0] for p in pnp]
            if any(len(ids) < required for ids in used):
                raise _Rejected("insufficient complete inlier markers after RANSAC")
            common = sorted(set(used[0]) & set(used[1]))
            if len(common) < required:
                raise _Rejected("insufficient common stereo inlier markers")
            result["used_ids"] = {eye: ids for eye, ids in zip(eyes, used)}
            result["counts"] = {"detected_markers_left": len(left), "detected_markers_right": len(right),
                                "used_markers_left": len(used[0]), "used_markers_right": len(used[1]),
                                "used_corners_left": 4*len(used[0]), "used_corners_right": 4*len(used[1]),
                                "common_markers": len(common)}
            tri_r, tri_t = self._triangulate(*observations, common)
            point_pairs = [self._points(obs, ids) for obs, ids in zip(observations, used)]

            def residual(parameters):
                rotation = cv2.Rodrigues(parameters[:3])[0]
                translation = parameters[3:]
                errors = []
                for eye, (xyz, uv) in enumerate(point_pairs):
                    r, t = (rotation, translation) if eye == 0 else (self.rig_r @ rotation, self.rig_r @ translation + self.rig_t)
                    errors.append((self._project(xyz, r, t, eye) - uv).reshape(-1))
                return np.concatenate(errors)

            seeds = [(tri_r, tri_t), (pnp[0][1], pnp[0][2]),
                     (self.rig_r.T @ pnp[1][1], self.rig_r.T @ (pnp[1][2] - self.rig_t))]
            # IPPE alternatives explicitly expose planar pose ambiguity.
            xyz_l, uv_l = point_pairs[0]
            if np.linalg.matrix_rank(xyz_l - xyz_l.mean(axis=0), tol=1e-8) == 2:
                alternatives = cv2.solvePnPGeneric(xyz_l, uv_l, self.k[0], self.d[0], flags=cv2.SOLVEPNP_IPPE)
                if alternatives[0]:
                    seeds.extend((cv2.Rodrigues(rv)[0], tv.reshape(3)) for rv, tv in zip(alternatives[1], alternatives[2]))
            solutions = []
            for rotation, translation in seeds:
                initial = np.r_[cv2.Rodrigues(rotation)[0].reshape(3), translation]
                fit = least_squares(residual, initial, method="lm", max_nfev=200, ftol=1e-10, xtol=1e-10, gtol=1e-10)
                if not fit.success or not np.isfinite(fit.x).all():
                    continue
                rotation, translation = cv2.Rodrigues(fit.x[:3])[0], fit.x[3:]
                if any(np.any((xyz @ (rotation if i == 0 else self.rig_r @ rotation).T
                                  + (translation if i == 0 else self.rig_r @ translation + self.rig_t))[:, 2] <= 0)
                       for i, (xyz, _) in enumerate(point_pairs)):
                    continue
                solutions.append((float(np.sqrt(np.mean(residual(fit.x)**2))), rotation, translation, fit.x))
            if not solutions:
                raise _Rejected("no finite positive-depth stereo pose")
            solutions.sort(key=lambda solution: solution[0])
            best = solutions[0]
            for other in solutions[1:]:
                angle = np.linalg.norm(cv2.Rodrigues(other[1] @ best[1].T)[0])
                if other[0] <= best[0] + .05 and (angle > np.deg2rad(5) or np.linalg.norm(other[2]-best[2]) > .005):
                    raise _Rejected("multiple stereo pose solutions are ambiguous")
            flat_errors = residual(best[3])
            offset = 0
            for eye, (xyz, _) in zip(eyes, point_pairs):
                error = np.linalg.norm(flat_errors[offset:offset+2*len(xyz)].reshape(-1, 2), axis=1)
                offset += 2*len(xyz)
                rms = float(np.sqrt(np.mean(error**2)))
                result["reprojection_error_px"][eye] = rms
                if rms > self.max_rms or np.max(error) > self.max_corner:
                    raise _Rejected(f"{eye}: stereo refined reprojection error exceeds threshold")
            qxyzw = Rotation.from_matrix(best[1]).as_quat()
            quaternion = np.r_[qxyzw[3], qxyzw[:3]]
            if quaternion[0] < 0:
                quaternion = -quaternion
            result["pose_target_to_left"] = dict(R=best[1].tolist(), t_m=best[2].tolist(), quaternion_wxyz=quaternion.tolist())
            degraded = any(len(ids) < self.minimum for ids in used)
            result["status"] = "degraded" if degraded else "ok"
            result["reason"] = "single-marker stereo explicitly allowed; reduced redundancy" if degraded else "accepted complete-marker stereo consensus"
        except (_Rejected, cv2.error, ValueError, np.linalg.LinAlgError) as exc:
            result["reason"] = str(exc)
        return result
