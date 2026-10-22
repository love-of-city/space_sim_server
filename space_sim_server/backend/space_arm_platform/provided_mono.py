"""Adapter for the byte-original user-supplied monocular ArUco/IPPE package.

Only decoded pixels, each eye's static K, and the current marker layout are
used. Stereo extrinsics, legacy tool offsets and the package's old wrist
intrinsics helper are never consulted. The original planar solver assumes
D=0; other distortion and noncoplanar boards are explicitly unsupported.
The original result's camera_id is a legacy source label; the wrapper's
left/right keys identify the eye. Neither output is navigation for control.
"""
from __future__ import annotations

import hashlib
import importlib.util
from pathlib import Path
import sys

import cv2
import numpy as np

SOURCE_SHA256 = "80a5bf3b97bf3b360307593894166124d87667e70bc944e70807262568478dac"
MANIFEST_SHA256 = "4728dc22d4b2f8e0dfa9f3c07cf8dafca49129aa409f6ee6f22a42c75e9c5b24"


def _load_original(root: Path):
    """Verify all packaged source bytes, then import under a private package."""
    root = Path(root).resolve()
    manifest = (root / "FILES.sha256").read_bytes()
    if hashlib.sha256(manifest).hexdigest() != MANIFEST_SHA256:
        raise ValueError("provided package FILES.sha256 SHA256 mismatch")
    for line in manifest.decode("utf-8").splitlines():
        digest, relative = line.split(None, 1)
        path = (root / relative.strip()).resolve()
        if not path.is_relative_to(root) or not path.is_file() or hashlib.sha256(path.read_bytes()).hexdigest() != digest:
            raise ValueError(f"provided package SHA256 mismatch: {relative.strip()}")
    source = root / "source/space_arm_platform/fiducials.py"
    if hashlib.sha256(source.read_bytes()).hexdigest() != SOURCE_SHA256:
        raise ValueError("provided FiducialEstimator source SHA256 mismatch")
    namespace = "_provided_aruco_" + SOURCE_SHA256[:12] + "_" + hashlib.sha256(str(root).encode()).hexdigest()[:8]
    if namespace + ".fiducials" in sys.modules:
        return sys.modules[namespace + ".fiducials"]

    def load(name, path, package=False):
        spec = importlib.util.spec_from_file_location(name, path,
                    submodule_search_locations=[str(path.parent)] if package else None)
        module = importlib.util.module_from_spec(spec)
        sys.modules[name] = module
        # Execute original bytes without writing __pycache__ into the vendor
        # snapshot. Preload the one relative import the original module uses.
        exec(compile(path.read_bytes(), str(path), "exec"), module.__dict__)
        return module

    directory = source.parent
    load(namespace, directory / "__init__.py", package=True)
    load(namespace + ".camera_defaults", directory / "camera_defaults.py")
    return load(namespace + ".fiducials", source)


def _current_rig(layout):
    if layout.get("dictionary", "DICT_4X4_50") != "DICT_4X4_50":
        raise ValueError("provided source requires DICT_4X4_50")
    markers = []
    for identifier, corners in layout.get("markers", {}).items():
        points = np.asarray(corners, dtype=float)
        if points.shape != (4, 3) or not np.isfinite(points).all():
            raise ValueError("current marker corners must be a finite 4x3 array")
        right, down = points[1]-points[0], points[3]-points[0]
        width, height = np.linalg.norm(right), np.linalg.norm(down)
        if min(width, height) <= 1e-8 or not np.isclose(width, height, rtol=1e-6, atol=1e-9):
            raise ValueError("provided source requires square physical markers")
        if abs(np.dot(right, down)) > width*height*1e-6:
            raise ValueError("current marker right/down axes must be orthogonal")
        right, down = right/width, down/height
        marker = dict(id=int(identifier), center_m=points.mean(axis=0).tolist(),
                      right=right.tolist(), down=down.tolist(), size_m=float((width+height)/2))
        center, half = np.array(marker["center_m"]), marker["size_m"]/2
        rebuilt = np.array([center+half*(u*right+v*down) for u, v in ((-1, -1), (1, -1), (1, 1), (-1, 1))])
        if not np.allclose(rebuilt, points, rtol=1e-7, atol=1e-9):
            raise ValueError("current marker corners do not describe a square in decoded order")
        if not 0 <= marker["id"] < 50 or any(m["id"] == marker["id"] for m in markers):
            raise ValueError("marker ids must be unique in DICT_4X4_50")
        markers.append(marker)
    if not markers:
        raise ValueError("current marker layout is empty")
    return dict(dictionary="DICT_4X4_50", markers=markers, body="capture_target", units="metres")


class ProvidedMonoEstimator:
    def __init__(self, calibration: dict, layout: dict, root: Path):
        if tuple(calibration.get("image_size", ())) != (1920, 1200):
            raise ValueError("provided camera requires exact 1920x1200 image_size")
        self.k = []
        for eye in ("left", "right"):
            k = np.asarray(calibration.get("K_"+eye), float)
            if k.shape != (3, 3) or not np.isfinite(k).all() or min(k[0, 0], k[1, 1]) <= 0 or not np.allclose(k[2], [0, 0, 1]):
                raise ValueError("invalid static camera intrinsic matrix")
            distortion = np.asarray(calibration.get("D_"+eye, []), float)
            if not np.isfinite(distortion).all() or np.any(distortion != 0):
                raise ValueError("provided algorithm supports zero distortion only")
            self.k.append(k.copy())
        rig = _current_rig(layout)
        points = np.concatenate([np.asarray(c, float) for c in layout["markers"].values()])
        self.planar = np.linalg.matrix_rank(points-points.mean(axis=0), tol=1e-8) <= 2
        original = _load_original(root)
        self.estimators = [original.FiducialEstimator(), original.FiducialEstimator()]
        for estimator in self.estimators:
            # Replace geometry configuration only. Detector, IPPE candidate
            # selection, ambiguity thresholds and LM code remain original.
            estimator.rig = rig
            estimator.markers = {m["id"]: m for m in rig["markers"]}

    def _eye(self, payload, index):
        try:
            if not isinstance(payload, (bytes, bytearray, memoryview)) or not payload:
                raise ValueError("invalid encoded image")
            image = cv2.imdecode(np.frombuffer(payload, np.uint8), cv2.IMREAD_COLOR)
            if image is None:
                raise ValueError("image decode failed")
            if image.shape[:2] != (1200, 1920):
                raise ValueError("image must be exactly 1920x1200 without resizing")
            if not self.planar:
                return dict(status="unsupported_layout", pose=None,
                            reason="provided IPPE algorithm requires all observed marker corners coplanar")
            return self.estimators[index].estimate(image, self.k[index])
        except (ValueError, cv2.error, np.linalg.LinAlgError) as exc:
            return dict(status="adapter_rejected", pose=None, reason=str(exc))

    def estimate(self, left_png: bytes, right_png: bytes) -> dict:
        return dict(left=self._eye(left_png, 0), right=self._eye(right_png, 1),
                    source="aruco_pnp_basler_mono_v1/fiducials.py:sha256="+SOURCE_SHA256,
                    scope="independent_monocular_no_stereo_fusion",
                    navigation_for_control={"valid": False, "reason": "single-frame camera poses only; no navigation transform, velocity or time filter"})
